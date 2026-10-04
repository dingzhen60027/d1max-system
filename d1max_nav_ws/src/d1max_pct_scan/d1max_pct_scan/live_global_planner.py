"""Fresh-goal PCT global planning for RViz/SCAN *visualization only*.

The source localization map and the conditioned PCT map are NOT related by a
rigid TF.  Only proven ground-path samples pass the explicit non-rigid bridge;
live clouds/IMU/body odometry remain in the original localization frame.
There is deliberately no command publisher and no restored route on startup.
"""
from __future__ import annotations

import hashlib
from functools import partial
import json
import math
import multiprocessing
import os
from pathlib import Path
import signal
import time

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.node import Node
from rclpy._rclpy_pybind11 import RCLError
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                      qos_profile_sensor_data)
from std_msgs.msg import Empty, String

from .live_global_contract import (
    FLOOR_NAMES, GlobalPlanError, choose_floor_surface, finite_xyz,
    floor_from_original_ground_z, floor_from_planning_z, fresh_stamp,
    new_goal_stamp,
    RequestEvidence, supported_start_floor,
)
from .live_global_worker_cleanup import RetiredChildren
from .live_goal_pause import (RetainedGlobalComputation, GoalIntent,
                              confirmed_goal_identity, hard_identity_issue)
from .live_map_geometry import validate_map_binding
from .live_scan_contract import validate_continuous_pose, checked_pose_status_envelope
from .native_global_worker import native_worker
from .navigation_task_policy import POLICY_ID, RouteAction, TaskEvent, task_transition
from .static_route_validation import StaticRouteValidator, validate_static_route
from .atomic_navigation_inbox import uses_atomic_navigation
from .source_route import SourceRouteBuilder
from .source_route_ros import to_message as route_message
from d1max_navigation_bt_interfaces.msg import RouteSnapshot as RouteSnapshotMessage

PREFIX = '/d1max/live_planning/'
ORIGINAL_FRAME = 'd1max_loc_map'
PLANNING_FRAME = 'd1max_multifloor_planning'
MAX_SNAPSHOT_REPLANS = 3


def _stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def prepare_validated_path(result,tomogram,bridge, *, builder=None, map_version_id=''):
    """Worker-thread static proof and message allocation; no Node/publication."""
    checked = validate_static_route(result,tomogram,bridge, builder=builder,
                                   map_version_id=map_version_id)
    began = time.monotonic()
    path = NavPath()
    path.header.frame_id = bridge.source_frame
    for xyz in checked['xyz']:
        pose = PoseStamped()
        # One shared header is stamped by the owner immediately before commit.
        pose.header = path.header
        pose.pose.position.x,pose.pose.position.y,pose.pose.position.z = map(float,xyz)
        pose.pose.orientation.w = 1.
        path.poses.append(pose)
    return {**checked,'path_message':path,'message_build_sec':time.monotonic()-began}


class LiveGlobalPlanner(Node):
    def __init__(self):
        super().__init__('d1max_live_global_planner')
        if os.environ.get('RMW_IMPLEMENTATION') != 'rmw_zenoh_cpp':
            raise ValueError('live_global_planner_requires_unchanged_rmw_zenoh_cpp')
        defaults = dict(session_id='', planning_manifest='', tomogram_npz='',
                        pipeline_contract='',map_version_id='',transport_mode='live',
                        crossfloor_route_config='', output_directory='',
                        current_floor='', goal_floor='',
                        body_height_min_m=.25, body_height_max_m=.85,
                        max_start_move_m=.3, result_timeout_s=10., warmup_timeout_s=30.,
                        commit_wait_timeout_s=10., freshness_s=.5,
                        retain_preview_task_on_soft_loss=False)
        self.p = {name: self.declare_parameter(name, value).value
                  for name, value in defaults.items()}
        if (type(self.p['retain_preview_task_on_soft_loss']) is not bool
                or not self.p['session_id'] or len(self.p['session_id']) > 128
                or self.p['current_floor'] not in FLOOR_NAMES
                or self.p['goal_floor'] not in FLOOR_NAMES
                or not .1 <= self.p['freshness_s'] <= .75
                or not .05 <= self.p['max_start_move_m'] <= .3
                or not 5 <= self.p['result_timeout_s'] <= 60
                or not 1 <= self.p['warmup_timeout_s'] <= 60
                or not 1 <= self.p['commit_wait_timeout_s'] <= 30
                or not 0 < self.p['body_height_min_m'] < self.p['body_height_max_m'] <= 2):
            raise ValueError('invalid_bounded_live_global_parameters')
        if not all(Path(self.p[key]).is_file() for key in
                   ('planning_manifest', 'tomogram_npz', 'crossfloor_route_config')):
            raise ValueError('missing_planning_map_or_route_configuration')
        manifest_path = Path(self.p['planning_manifest']).resolve(strict=True)
        # Same audited source implementation as the offline tools, packaged
        # inside this release instead of resolved from the map directory/CWD.
        from .pointcloud_helpers.ground_path_bridge import GroundPathBridge
        identity = json.loads(manifest_path.read_text()).get('geometry_operation') == 'source_identity'
        if identity:
            from .source_identity import SourceIdentityBridge
            self.bridge = SourceIdentityBridge.from_artifacts(manifest_path)
        else:
            self.bridge = GroundPathBridge.from_artifacts(manifest_path)
        from d1max_pct_planner.tomogram_map import TomogramMap
        from d1max_pct_planner.crossfloor_preview import load_config
        if identity:
            from d1max_pct_planner.singlefloor_route import load_config
        from d1max_pct_planner.paths import expand_tree
        route_raw = expand_tree(yaml.safe_load(Path(self.p['crossfloor_route_config']).read_text()))
        self.map_options = dict(
            minimum_headroom_m=route_raw.get('minimum_headroom_m'),
            unknown_ceiling_policy=route_raw.get('unknown_ceiling_policy', 'allow_unobserved'),
            max_ground_step_m=route_raw['limits']['max_ground_step_m'])
        self.tomogram = TomogramMap(self.p['tomogram_npz'], **self.map_options)
        for value in vars(self.tomogram).values():
            if isinstance(value,np.ndarray):
                value.flags.writeable = False
        _, self.route_settings = load_config(self.p['crossfloor_route_config'], self.tomogram)
        self.route_config_sha256 = hashlib.sha256(
            Path(self.p['crossfloor_route_config']).read_bytes()).hexdigest()
        validate_map_binding(manifest_path, self.bridge, self.route_settings,
                             source_frame=ORIGINAL_FRAME)
        self.source_route_builder = SourceRouteBuilder.from_artifacts(
            manifest_path, self.tomogram, self.bridge)
        from d1max_pct_planner.native_runtime import prepare_native_environment
        self.native_environment = prepare_native_environment(
            self.route_settings['vendor_root'], os.environ)
        self.startup_stamp = self.now_s()
        self.last_goal_stamp = self.startup_stamp
        self.generation = 0
        self.navigation = {}
        self.pose_status = {}
        self.scan = {}
        self.localizer = {}
        self.atomic_state=None
        from .atomic_navigation_inbox import AtomicNavigationInbox
        self.atomic_inbox=AtomicNavigationInbox()
        self.nav_received = self.scan_received = self.body_received = self.localizer_received = -math.inf
        self.pose_received = -math.inf
        self.pose_watermark = -math.inf
        self.nav_watermark = self.localizer_watermark = -math.inf
        self.pause = RetainedGlobalComputation(good_samples=3, good_span_s=.2)
        self.body = None
        self.body_context = None
        self.sensor_identity = None
        self.body_barrier = self.startup_stamp
        self.last_body_stamp = 0.
        self.body_stamp = 0.
        self.pause_wall = None
        self.current = None
        self.pending_goal = None
        self.cached_result = None
        self.cached_evidence = None
        self.commit_wait_started = None
        self.replan_count = 0
        self.goal_started_monotonic = None
        self.goal_deadline_monotonic = None
        self.computation_cycle = 0
        self.route_committed = False
        self.recovery_hold = None
        self.refresh_requested = False
        self.last_refresh_id = 0
        self.last_reference_stamp = 0.
        self.active_context = None
        self.child = None
        self.pipe = None
        self.worker_generation = None
        self.worker_metrics = {}
        self.warmup = dict(phase='not_started',warmup_id=0,deadline=None)
        self.static_validator = StaticRouteValidator()
        self.active_reference = False
        self.visual_path_available = False
        self.last_status_at = -math.inf
        self.state = 'waiting_for_fresh_localization'
        self.reason = 'startup_requires_new_goal'
        self.goal_kind = None
        self.goal_stamp = None
        self.last_route = {}
        self.mp = multiprocessing.get_context('spawn')
        self.retired_children = RetiredChildren()
        display_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(NavPath, PREFIX + 'reference_path', display_qos)
        self.route_pub = self.create_publisher(RouteSnapshotMessage,
                                              PREFIX+'bt/worker_route', display_qos)
        # A retained route is useful context, not permission to run the local
        # planner. Only reference_path participates in execution admission.
        self.visual_path_pub = self.create_publisher(
            NavPath, PREFIX + 'global_path_visual', display_qos)
        self.status_pub = self.create_publisher(String, PREFIX + 'global_status', 5)
        if uses_atomic_navigation(self.p):
            from d1max_planning_interfaces.msg import NavigationState
            self.create_subscription(NavigationState,'/d1max/localization/navigation/state',self.on_atomic_state,qos_profile_sensor_data)
        else:
            self.create_subscription(String, '/d1max/localization/navigation/status', self.on_navigation, 5)
            self.create_subscription(String, '/d1max/localization/navigation/pose_status', self.on_pose_status, qos_profile_sensor_data)
            self.create_subscription(String, '/d1max/localization/status', self.on_localizer, 5)
        self.create_subscription(String, PREFIX + 'scan_bridge_status', self.on_scan, 5)
        self.create_subscription(String, PREFIX + 'reference_refresh_request',
                                 self.on_reference_refresh_request, 5)
        if not uses_atomic_navigation(self.p):
            self.create_subscription(Odometry, '/d1max/localization/odometry/global', self.on_body, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, PREFIX + 'goal', self.on_goal2d, 5)
        self.create_subscription(PointStamped, PREFIX + 'goal3d', self.on_goal3d, 5)
        self.create_subscription(Empty, PREFIX + 'cancel', self.on_cancel, 5)
        self.timer = self.create_timer(.05, self.tick)
        self.publish_empty('startup_requires_new_goal')
        self.clear_visual_path()
        LiveGlobalPlanner._start_warmup(self)

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def publish_empty(self, reason):
        empty = NavPath()
        empty.header.frame_id = self.bridge.source_frame
        empty.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(empty)
        self.active_reference = False
        self.reason = reason

    def clear_visual_path(self):
        empty = NavPath()
        empty.header.frame_id = self.bridge.source_frame
        empty.header.stamp = self.get_clock().now().to_msg()
        self.visual_path_pub.publish(empty)
        self.visual_path_available = False

    def stop_child(self):
        child, pipe = self.child, self.pipe
        self.child = self.pipe = None
        self.worker_generation = None
        if getattr(self,'warmup',{}).get('phase') == 'running':
            self.warmup.update(phase='stopped',deadline=None)
        self.retired_children.retire(child, pipe)

    def idle_worker_available(self):
        return (self.child is not None and self.pipe is not None
                and self.worker_generation is None and self.child.is_alive()
                and getattr(self,'warmup',{}).get('phase') != 'running')

    def _spawn_native_worker(self):
        parent, child = self.mp.Pipe(duplex=True)
        self.pipe = parent
        from d1max_pct_planner.compute_budget import budgeted_worker, spawn_with_environment
        self.child = self.mp.Process(target=budgeted_worker,
            args=(native_worker, child, self.p['tomogram_npz'], self.p['crossfloor_route_config'],
                  self.map_options, self.tomogram.sha256, self.route_config_sha256),daemon=True)
        try:
            spawn_with_environment(self.child,self.native_environment)
        finally:
            child.close()

    def _start_warmup(self):
        """One attempt per explicit startup/replacement, never a tick retry."""
        if (self.child is not None or self.retired_children.pending
                or self.warmup.get('phase') in ('running','failed')):
            return False
        token=self.warmup.get('warmup_id',0)+1
        self.warmup=dict(phase='running',warmup_id=token,
            deadline=time.monotonic()+self.p.get('warmup_timeout_s',30.))
        try:
            LiveGlobalPlanner._spawn_native_worker(self)
            self.pipe.send(dict(kind='warmup',warmup_id=token))
            self.state='warming_native_maps'
            self.reason='native_resources_initializing_without_goal'
            return True
        except Exception as exc:
            self.stop_child()
            self.warmup.update(phase='failed',deadline=None,error=str(exc)[:500])
            self.state='native_warmup_failed';self.reason='startup_warmup_failed'
            return False

    def _poll_warmup(self):
        warmup=getattr(self,'warmup',{})
        if warmup.get('phase')!='running':
            return
        try:
            if time.monotonic()>=warmup['deadline']:
                raise GlobalPlanError('native_warmup_deadline_expired')
            for _ in range(2):
                if self.pipe is None or not self.pipe.poll(0):
                    break
                packet=self.pipe.recv()
                if not isinstance(packet,dict) or packet.get('warmup_id')!=warmup['warmup_id']:
                    raise GlobalPlanError('native_warmup_identity_mismatch')
                if packet.get('kind')=='warmup_progress':
                    warmup['worker_phase']=packet.get('phase');continue
                if packet.get('kind')!='warmed':
                    raise GlobalPlanError('native_warmup_failed:'+str(packet.get('error','invalid_packet')))
                warmup.update(phase='ready',deadline=None,elapsed_sec=packet.get('elapsed_sec'),
                              metrics=packet.get('metrics',{}))
                if self.current is None:
                    self.state='waiting_for_fresh_localization';self.reason='native_maps_ready_no_goal'
                return
            if self.child is None or not self.child.is_alive():
                raise GlobalPlanError('native_warmup_process_exited')
        except (EOFError,OSError,GlobalPlanError) as exc:
            self.stop_child()
            warmup.update(phase='failed',deadline=None,error=str(exc)[:500])
            self.state='native_warmup_failed';self.reason=str(exc)[:500]

    def revoke(self, reason, *, publish=True, preserve_idle_worker=False):
        keep_worker = preserve_idle_worker and (self.idle_worker_available() or
            (getattr(self,'warmup',{}).get('phase')=='running' and self.child is not None))
        self.static_validator.invalidate()
        self.generation += 1
        self.pause.clear()
        self.pause_wall = None
        self.current = None
        self.pending_goal = None
        self.cached_result = None
        self.cached_evidence = None
        self.commit_wait_started = None
        self.refresh_requested = False
        self.worker_metrics = {}
        self.last_reference_stamp = 0.
        self.goal_started_monotonic = self.goal_deadline_monotonic = None
        self.route_committed = False
        self.recovery_hold = None
        self.active_context = None
        if not keep_worker:
            self.stop_child()
        self.state = 'waiting_for_new_goal'
        self.last_route = {}
        self.goal_kind = self.goal_stamp = None
        if publish:
            self.publish_empty(reason)
            self.clear_visual_path()
        else:
            self.active_reference = False
            self.visual_path_available = False
            self.reason = reason

    def on_atomic_state(self,message):
        event=self.atomic_inbox.accept(message,session_id=self.p['session_id'],
            map_version_id=self.p['map_version_id'],now_ns=self.get_clock().now().nanoseconds,
            monotonic=time.monotonic())
        if event is None:
            return
        self.atomic_state=state=event.state
        stamp=event.source_ns*1e-9
        self.localizer=dict(session_id=event.identity[0],local_epoch=event.identity[1],
            active_seed_ns=event.identity[2],confirmed_seed_ns=event.identity[2],
            map_version_id=message.map_version_id,wall_time=stamp)
        self.navigation=dict(epoch=event.identity[1],seed_id=event.identity[2],received_at_unix=stamp,
            valid=state is not None,fault='atomic_navigation_hard_loss' if event.hard_failure else '')
        self.pose_status=dict(self.navigation,output_stamp_sec=stamp)
        self.nav_received=self.pose_received=self.localizer_received=self.body_received=event.received
        self.body=np.asarray(state.global_body.position) if state is not None else None
        self.body_stamp=self.last_body_stamp=stamp
        self.body_context=self.confirmed_identity() if state is not None else None
        self._revoke_if_context_lost()

    def on_navigation(self, message):
        try:
            if len(message.data) > 65536:
                raise ValueError('oversize_navigation_status')
            value = json.loads(message.data)
            if not isinstance(value, dict):
                raise ValueError('navigation_status_not_object')
            stamp = value.get('received_at_unix')
            if not fresh_stamp(stamp,self.now_s(),self.p['freshness_s'],future_tolerance_s=.01):
                raise ValueError('navigation_status_source_time_invalid')
            if stamp <= self.nav_watermark:
                return
            self.navigation = value
            self.nav_watermark = stamp
            self.nav_received = time.monotonic()
        except (ValueError, TypeError):
            self.navigation = {}
        self._revoke_if_context_lost()

    def on_pose_status(self, message):
        try:
            if len(message.data) > 8192:
                raise ValueError('oversize_continuous_pose_status')
            value = json.loads(message.data)
            sample = checked_pose_status_envelope(value,now=self.now_s(),timeout=self.p['freshness_s'])
            if (value['frame_id'] != self.bridge.source_frame
                    or value['body_frame'] != 'd1max_loc_base_link'):
                raise ValueError('continuous_pose_frame_mismatch')
            if sample <= self.pose_watermark:
                return
            self.pose_status, self.pose_received = value, time.monotonic()
            self.pose_watermark = sample
        except (ValueError, TypeError):
            self.pose_status = {}
            self.pose_received = -math.inf
        self._revoke_if_context_lost()

    def on_localizer(self, message):
        try:
            if len(message.data) > 262144:
                raise ValueError('oversize_localizer_status')
            value = json.loads(message.data)
            if not isinstance(value, dict):
                raise ValueError('localizer_status_not_object')
            stamp = value.get('wall_time')
            if not fresh_stamp(stamp,self.now_s(),self.p['freshness_s'],future_tolerance_s=.01):
                raise ValueError('localizer_status_source_time_invalid')
            if stamp <= self.localizer_watermark:
                return
            self.localizer = value
            self.localizer_watermark = stamp
            self.localizer_received = time.monotonic()
        except (ValueError, TypeError):
            self.localizer = {}
        self._revoke_if_context_lost()

    def on_scan(self, message):
        try:
            if len(message.data) > 65536:
                raise ValueError('oversize_scan_status')
            value = json.loads(message.data)
            if not isinstance(value, dict):
                raise ValueError('scan_status_not_object')
            if (self.scan and type(value.get('received_at_unix')) in (int, float)
                    and value['received_at_unix'] <= self.scan.get('received_at_unix', 0.)):
                return
            self.scan = value
            self.scan_received = time.monotonic()
        except (ValueError, TypeError):
            self.scan = {}

    def on_reference_refresh_request(self, message):
        """Bridge asks for a newly stamped global commit, never an old replay."""
        try:
            if len(message.data) > 4096:
                return
            value = json.loads(message.data)
            request_id = value.get('request_id')
            if (not isinstance(value, dict) or type(request_id) is not int
                    or request_id <= self.last_refresh_id or request_id < 1
                    or value.get('schema') != 1 or value.get('motion_enabled') is not False
                    or self.pause.intent is None or self.cached_result is None
                    or value.get('session_id') != self.p['session_id']
                    or value.get('localization_epoch') != self.pause.intent.identity.epoch
                    or value.get('localization_seed_id') != self.pause.intent.identity.confirmed_seed
                    or value.get('rejected_reference_stamp') != self.last_reference_stamp
                    or not fresh_stamp(value.get('received_at_unix'), self.now_s(), .5)):
                return
            self.last_refresh_id = request_id
            self.refresh_requested = True
        except (ValueError, TypeError, AttributeError):
            return

    def on_body(self, message):
        if (self.pause.intent is not None and
                (message.header.frame_id != self.bridge.source_frame
                 or message.child_frame_id != 'd1max_loc_base_link')):
            self.revoke('body_frame_changed')
            return
        try:
            stamp = _stamp_seconds(message.header.stamp)
            xyz = finite_xyz([message.pose.pose.position.x, message.pose.pose.position.y,
                              message.pose.pose.position.z], 'body')
            if (message.header.frame_id != self.bridge.source_frame
                    or message.child_frame_id != 'd1max_loc_base_link'
                    or not fresh_stamp(stamp,
                                       self.now_s(), self.p['freshness_s'])
                    or not np.isfinite([message.pose.pose.orientation.x,
                                        message.pose.pose.orientation.y,
                                        message.pose.pose.orientation.z,
                                        message.pose.pose.orientation.w]).all()):
                raise ValueError('invalid_or_stale_original_frame_body_odometry')
            identity = self.confirmed_identity()
            if self.sensor_identity != identity:
                self.sensor_identity = identity
                self.body_barrier = max(self.body_barrier,self.now_s())
                self.body,self.body_context = None,None
            # Odometry carries no epoch. A post-reset arrival cannot relabel an
            # old stamped sample as the new confirmed seed/map context.
            if stamp <= self.body_barrier or stamp <= self.last_body_stamp:
                self._revoke_if_context_lost()
                return
            validate_continuous_pose(self.pose_status,epoch=identity.epoch,
                seed=identity.confirmed_seed,now=self.now_s(),map_frame=self.bridge.source_frame,
                body_frame='d1max_loc_base_link',timeout=self.p['freshness_s'],
                receipt_age=time.monotonic()-self.pose_received)
            self.body, self.body_received = xyz, time.monotonic()
            self.body_stamp = self.last_body_stamp = stamp
            self.body_context = identity
        except (ValueError, TypeError):
            self.body = None
        self._revoke_if_context_lost()

    def planning_context(self):
        now, mono = self.now_s(), time.monotonic()
        if self.navigation.get('fault') or self.navigation.get('reset_pending') is True:
            raise GlobalPlanError('navigation_explicit_fault_or_reset')
        if (not 0 <= mono-self.pose_received <= self.p['freshness_s']
                or not 0 <= mono-self.body_received <= self.p['freshness_s']
                or self.body is None
                or not fresh_stamp(self.body_stamp, now, self.p['freshness_s'])):
            raise GlobalPlanError('continuous_pose_or_body_subscription_stale')
        identity = self.confirmed_identity()
        if self.body_context != identity:
            raise GlobalPlanError('body_not_bound_to_confirmed_epoch_seed_and_map')
        if uses_atomic_navigation(self.p):
            return identity.epoch,identity.confirmed_seed
        try:
            return validate_continuous_pose(self.pose_status, epoch=identity.epoch,
                seed=identity.confirmed_seed, now=now, map_frame=self.bridge.source_frame,
                body_frame='d1max_loc_base_link', timeout=self.p['freshness_s'],
                receipt_age=mono-self.pose_received)
        except ValueError as exc:
            raise GlobalPlanError(str(exc)) from exc

    def confirmed_identity(self):
        if uses_atomic_navigation(self.p):
            from .live_goal_pause import GoalIdentity
            if not self.atomic_inbox.usable(now_ns=self.get_clock().now().nanoseconds,
                    monotonic=time.monotonic(),receipt_timeout_s=self.p['freshness_s']):
                raise ValueError('fresh_atomic_navigation_state_required')
            identity=self.atomic_state.identity
            return GoalIdentity(identity[0],self.p['map_version_id'],identity[1],identity[2],
                self.bridge.source_frame,self.tomogram.sha256)
        if time.monotonic()-self.localizer_received > self.p['freshness_s']:
            raise ValueError('localizer_subscription_stale')
        return confirmed_goal_identity(
            self.localizer, session_id=self.p['session_id'],
            frame_id=self.bridge.source_frame, tomogram_sha256=self.tomogram.sha256,
            now=self.now_s(), freshness_s=self.p['freshness_s'])

    def pause_reference(self, reason):
        if getattr(self, 'recovery_hold', None) is not None:
            return  # A retained blocked task is not a new computation attempt.
        if not self.pause.pause(time.monotonic()):
            return
        self.pause_wall = self.now_s()
        # A retained task is NOT a live pose/map lease. The local bridge owns
        # those leases and immediately suspends admission on missing evidence.
        # Empty Path is reserved for cancellation/hard invalidation: sending it
        # for a 20 ms pose gap turned one user task into repeated cancel/new
        # generations, racing the bridge's recovery barrier indefinitely.
        self.active_context = None
        if not getattr(self, 'route_committed', False):
            self.last_route = {}
        if getattr(self, 'p', {}).get('retain_preview_task_on_soft_loss') is True:
            self.active_reference = False
            self.reason = 'paused:' + reason
        else:
            # Execution/strict adapters retain the previous immediate withdraw
            # contract. Only the preview owner may opt into task retention.
            self.publish_empty('paused:' + reason)
        if (self.cached_result is not None and self.commit_wait_started is None
                and not getattr(self, 'route_committed', False)):
            self.commit_wait_started = time.monotonic()
        self.state = ('route_retained_following_paused' if getattr(self, 'route_committed', False)
                      else 'awaiting_navigation_to_commit' if self.cached_result is not None
                      else 'computing_while_navigation_unavailable')

    def resume_intent(self):
        intent = self.pause.intent
        if intent is None or getattr(self, 'recovery_hold', None) is not None:
            return
        try:
            if self.confirmed_identity() != intent.identity:
                raise GlobalPlanError('resume_identity_changed')
            context = self.planning_context()
            if context != (intent.identity.epoch, intent.identity.confirmed_seed):
                raise GlobalPlanError('resume_navigation_identity_changed')
            self.pause.resumed()
            self.pause_wall = None
            evidence = self.current or self.cached_evidence
            decision = task_transition(TaskEvent.INPUT_RECOVERED,
                route_committed=bool(getattr(self, 'route_committed', False)))
            if decision.route is RouteAction.RETAIN:
                # Robot progress does not invalidate a route whose initial
                # start was already admitted. Renew the local reference
                # handshake, not the global planning request.
                self._commit_cached_result()
            elif evidence is not None and self._start_moved(evidence):
                self._replan_from_fresh_body(context, 'body_moved_after_soft_outage')
            elif self.cached_result is not None:
                self._commit_cached_result()
            elif self.current is not None:
                self.state = 'computing_native_pct'
                self.reason = 'native_computation_continued_same_snapshot'
        except Exception as exc:
            self.revoke('resume_revalidation_failed:' + str(exc)[:500])
            self.state = 'goal_rejected'

    def _begin_computation_cycle(self):
        """One finite calculation budget; the immutable user goal lives longer.

        A successful commit ends a cycle. Only a new explicit user planning
        request starts another one. Moving-start retries BEFORE first commit
        stay within the original deadline; recovery never reopens a cycle.
        """
        if self.pause.intent is None:
            raise GlobalPlanError('computation_requires_user_goal')
        self.computation_cycle = getattr(self, 'computation_cycle', 0) + 1
        self.goal_started_monotonic = time.monotonic()
        self.goal_deadline_monotonic = self.goal_started_monotonic + self.p['result_timeout_s']
        self.replan_count = 0
        self.route_committed = False
        self.recovery_hold = None

    def _hold_goal(self, reason):
        """Withdraw route authority without pretending the user's goal vanished.

        A failed/expired calculation is latched until a new user request.
        Body movement and recovered sensor inputs do not allocate another
        computation cycle or override the owner's explicit planning policy.
        """
        if self.pause.intent is None:
            return
        self.static_validator.invalidate()
        self.generation += 1
        self.current = self.pending_goal = None
        self.commit_wait_started = None
        self.refresh_requested = False
        self.goal_started_monotonic = self.goal_deadline_monotonic = None
        self.pause.resumed()
        self.pause_wall = None
        self.active_context = None
        self.last_route = {}
        self.last_reference_stamp = 0.
        if not self.idle_worker_available():
            self.stop_child()
        self.publish_empty('goal_retained:' + reason)
        self.state = 'goal_retained_waiting_recovery'
        self.recovery_hold = dict(reason=reason,
            user_stamp=self.pause.intent.user_stamp,
            retry_on_supported_body_change=False,
            last_support_check_monotonic=time.monotonic(),
            body_xyz=tuple(float(v) for v in self.body) if self.body is not None else None)

    def _revoke_if_context_lost(self):
        intent = self.pause.intent
        if intent is None:
            return
        mono = time.monotonic()
        issue = hard_identity_issue(intent.identity, localizer=self.localizer,
                                    navigation=self.navigation, scan={})
        issue = issue or hard_identity_issue(intent.identity, localizer={},
                                             navigation=self.pose_status, scan={})
        if issue or self.tomogram.sha256 != intent.identity.tomogram_sha256:
            self.revoke('goal_identity_or_fault_changed:' + (issue or 'map_hash_changed'))
            return
        if (self.commit_wait_started is not None
                and mono-self.commit_wait_started >= self.p['commit_wait_timeout_s']):
            LiveGlobalPlanner._hold_goal(self, 'global_result_commit_wait_expired')
            return
        try:
            context = self.planning_context()
            identity = self.confirmed_identity()
        except (GlobalPlanError, ValueError) as exc:
            self.pause_reference(str(exc))
            return
        if (identity != intent.identity
                or context != (intent.identity.epoch, intent.identity.confirmed_seed)
                or self.active_context is not None and context != self.active_context):
            self.revoke('confirmed_goal_identity_or_context_changed')
            return
        if getattr(self, 'recovery_hold', None) is not None:
            return
        if self.pause.paused_at is not None and not self._pause_barrier_passed():
            self.pause.reset_good()
            self.reason = 'paused:waiting_fresh_body_and_localizer_after_pause'
            return
        if (self.pause.paused_at is not None and
                self.pause.observe_good(mono, self.pose_status.get('received_at_unix'))):
            self.resume_intent()
        elif self.pause.paused_at is None and self.refresh_requested and self.cached_result is not None:
            self.refresh_requested = False
            self._commit_cached_result()

    def _pause_barrier_passed(self):
        if self.pause_wall is None or self.pause.paused_at is None:
            return False
        return (self.body_received > self.pause.paused_at
                and self.localizer_received > self.pause.paused_at
                and self.body_stamp > self.pause_wall
                and self.localizer.get('wall_time', 0.) > self.pause_wall)

    def on_cancel(self, _message):
        self.revoke('explicit_user_cancel')

    def on_goal2d(self, message):
        self._new_goal(message, kind='2d')

    def on_goal3d(self, message):
        self._new_goal(message, kind='3d')

    def _new_goal(self, message, *, kind):
        # A delayed/duplicated old publication cannot cancel a newer route.
        # A fresh NEW goal cancels first, even if its geometry later fails.
        stamp = _stamp_seconds(message.header.stamp)
        try:
            new_goal_stamp(stamp, now=self.now_s(), startup_stamp=self.startup_stamp,
                           previous_stamp=self.last_goal_stamp)
        except GlobalPlanError:
            self.reason = 'obsolete_or_stale_goal_ignored'
            return
        self.last_goal_stamp = stamp
        self.revoke('new_goal_cancels_previous_reference', preserve_idle_worker=True)
        try:
            self.goal_stamp, self.goal_kind = stamp, kind
            context = self.planning_context()
            identity = self.confirmed_identity()
            if context != (identity.epoch, identity.confirmed_seed):
                raise GlobalPlanError('goal_navigation_localizer_identity_mismatch')
            if kind == '2d':
                if message.header.frame_id != self.bridge.source_frame:
                    raise GlobalPlanError('2d_goal_must_use_original_localization_frame')
                goal_xy = np.asarray([message.pose.position.x, message.pose.position.y], dtype=float)
                if not np.isfinite(goal_xy).all():
                    raise GlobalPlanError('goal_xy_invalid')
                goal_floor = self.get_parameter('goal_floor').value
                xyz = (float(goal_xy[0]), float(goal_xy[1]), 0.)
            else:
                goal_xyz = finite_xyz([message.point.x, message.point.y, message.point.z], 'goal3d')
                if getattr(self.bridge,'projection_kind',None)=='source_identity' and message.header.frame_id==self.bridge.source_frame:
                    goal_floor=floor_from_original_ground_z(self.bridge,goal_xyz)
                elif message.header.frame_id == self.bridge.planning_frame:
                    goal_floor = floor_from_planning_z(
                        goal_xyz[2], self.route_settings['floor_z_ranges'])
                elif message.header.frame_id == self.bridge.source_frame:
                    goal_floor = floor_from_original_ground_z(self.bridge, goal_xyz)
                else:
                    raise GlobalPlanError('3d_goal_frame_must_be_original_or_planning')
                xyz = tuple(float(v) for v in goal_xyz)
            intent = GoalIntent(stamp, kind, message.header.frame_id, xyz,
                                goal_floor, identity)
            self.pause.install(intent)
            LiveGlobalPlanner._begin_computation_cycle(self)
            self.plan_intent(intent, context=context, attempt_stamp=stamp)
        except Exception as exc:
            self.pause.clear()
            self.pending_goal = None
            self.current = None
            self.goal_started_monotonic = self.goal_deadline_monotonic = None
            self.stop_child()
            self.goal_kind = self.goal_stamp = None
            self.state = 'goal_rejected'
            self.reason = str(exc)[:1000]

    def plan_intent(self, intent, *, context, attempt_stamp):
        """Reproject both endpoints from current body and revalidate each attempt."""
        if (self.pause.intent != intent or context != (intent.identity.epoch,
                                                       intent.identity.confirmed_seed)):
            raise GlobalPlanError('goal_intent_or_context_not_current')
        if not fresh_stamp(attempt_stamp, self.now_s(), 2.):
            raise GlobalPlanError('goal_attempt_stamp_not_fresh')
        if (self.goal_deadline_monotonic is None
                or time.monotonic() >= self.goal_deadline_monotonic):
            raise GlobalPlanError('total_goal_computation_deadline_expired')
        body = self.body.copy()
        current_floor, projected = supported_start_floor(
            self.bridge, body, height_interval_m=(self.p['body_height_min_m'],
                                                 self.p['body_height_max_m']))
        start_info = choose_floor_surface(
            self.tomogram, projected.xyz[:2], current_floor,
            self.route_settings['floor_z_ranges'],
            hint_z=(float(projected.xyz[2]) if getattr(self.bridge,'projection_kind',None)=='source_identity'
                    else self.bridge.floors[current_floor].reference_z_m))
        if intent.kind == '2d':
            if intent.frame_id != self.bridge.source_frame:
                raise GlobalPlanError('2d_goal_must_use_original_localization_frame')
            goal_hint = (float(self.bridge.query(np.asarray(intent.xyz[:2]).reshape(1,2))[0][0])
                if getattr(self.bridge,'projection_kind',None)=='source_identity'
                else self.bridge.floors[intent.floor_id].reference_z_m)
        elif intent.kind == '3d':
            if (getattr(self.bridge, 'projection_kind', None) == 'source_identity'
                    and intent.frame_id == self.bridge.source_frame):
                # An identity map keeps the measured local height. Its whole-
                # floor median is a UI hint, not a replacement for a 3D goal.
                # Check identity first: source/planning frame aliases must not
                # accidentally decide the geometry contract.
                if floor_from_original_ground_z(self.bridge, intent.xyz) != intent.floor_id:
                    raise GlobalPlanError('original_goal_floor_changed')
                goal_hint = intent.xyz[2]
            elif intent.frame_id == self.bridge.planning_frame:
                if floor_from_planning_z(intent.xyz[2], self.route_settings['floor_z_ranges']) != intent.floor_id:
                    raise GlobalPlanError('planning_goal_floor_changed')
                goal_hint = intent.xyz[2]
            elif intent.frame_id == self.bridge.source_frame:
                if floor_from_original_ground_z(self.bridge, intent.xyz) != intent.floor_id:
                    raise GlobalPlanError('original_goal_floor_changed')
                goal_hint = self.bridge.floors[intent.floor_id].reference_z_m
            else:
                raise GlobalPlanError('3d_goal_frame_must_be_original_or_planning')
        else:
            raise GlobalPlanError('unknown_goal_kind')
        goal_info = choose_floor_surface(
            self.tomogram, intent.xyz[:2], intent.floor_id,
            self.route_settings['floor_z_ranges'], hint_z=goal_hint)
        if (not np.array_equal(start_info['xyz'][:2], projected.xyz[:2])
                or not np.array_equal(goal_info['xyz'][:2], intent.xyz[:2])):
            raise GlobalPlanError('endpoint_xy_was_modified')
        self.generation += 1
        generation = self.generation
        self.current = RequestEvidence(generation, context[0], str(context[1]),
                                       attempt_stamp, tuple(float(v) for v in body),
                                       time.monotonic())
        # A retired native worker may still hold >2 GiB. Keep only the
        # latest verified request until it is fully reaped; no overlap.
        self.pending_goal = (start_info, goal_info, generation)
        self._maybe_launch_pending()
        if self.pause.intent != intent or self.current is None:
            raise GlobalPlanError('native_attempt_could_not_be_started_or_queued')

    def _maybe_launch_pending(self):
        if self.pending_goal is None:
            return
        self.retired_children.reap()
        if self.retired_children.pending:
            self.state = 'waiting_for_native_worker_cleanup'
            self.reason = 'latest_fresh_goal_queued_until_old_worker_exits'
            return
        if getattr(self,'warmup',{}).get('phase')=='running':
            self.state='warming_native_maps';self.reason='goal_waits_for_same_startup_warmup'
            return
        try:
            if getattr(self,'warmup',{}).get('phase')=='failed':
                raise GlobalPlanError('native_warmup_failed_restart_session_required')
            start_info, goal_info, generation = self.pending_goal
            evidence = self.current
            if evidence is None or evidence.generation != generation:
                raise GlobalPlanError('pending_goal_generation_obsolete')
            if (self.goal_deadline_monotonic is None
                    or time.monotonic() >= self.goal_deadline_monotonic):
                raise GlobalPlanError('total_goal_computation_deadline_expired')
            intent = self.pause.intent
            if (intent is None or evidence.epoch != intent.identity.epoch
                    or evidence.seed_id != intent.identity.confirmed_seed
                    or self.tomogram.sha256 != intent.identity.tomogram_sha256
                    or hard_identity_issue(intent.identity, localizer=self.localizer,
                                           navigation=self.navigation, scan={})):
                raise GlobalPlanError('pending_goal_hard_identity_changed')
            if self.worker_generation is not None:
                raise GlobalPlanError('native_worker_already_busy')
            if not self.idle_worker_available():
                if self.child is not None:
                    self.stop_child()
                    if self.retired_children.pending:
                        return
                LiveGlobalPlanner._start_warmup(self)
                return
            self.pipe.send(dict(kind='plan', generation=generation,
                start_xyz=start_info['xyz'], goal_xyz=goal_info['xyz'],
                start_layer=int(start_info['layer_id']), goal_layer=int(goal_info['layer_id'])))
            self.worker_generation = generation
            self.worker_metrics = dict(generation=generation, phase='worker_request_queued')
            self.pending_goal = None
            self.state = ('computing_while_navigation_unavailable'
                          if self.pause.paused_at is not None else 'computing_native_pct')
            self.reason = 'immutable_goal_start_snapshot_locked'
        except Exception as exc:
            if str(exc) == 'total_goal_computation_deadline_expired':
                LiveGlobalPlanner._hold_goal(self, str(exc))
                return
            self.revoke('native_attempt_start_failed:' + str(exc)[:500])
            self.state = 'goal_rejected'

    def _start_moved(self, evidence):
        return (float(np.linalg.norm(
            finite_xyz(self.body, 'current_body') -
            finite_xyz(evidence.start_body_xyz, 'request_start_body')))
            > self.p['max_start_move_m'])

    def _replan_from_fresh_body(self, context, reason):
        intent = self.pause.intent
        if intent is None:
            return
        committed = getattr(self, 'route_committed', False)
        if task_transition(TaskEvent.BODY_PROGRESS,
                           route_committed=bool(committed)).route is RouteAction.RETAIN:
            # Defense in depth for every future caller. There is no implicit
            # global replan policy: a new operator request uses _new_goal.
            self.reason = 'committed_route_retained_during_body_progress'
            return
        # Every replacement start needs real support, including movement while
        # the FIRST route is still computing. Never drop the mission merely
        # because its current pose lies inside the protected stair volume.
        try:
            supported_start_floor(self.bridge, self.body,
                height_interval_m=(self.p['body_height_min_m'], self.p['body_height_max_m']))
        except GlobalPlanError as exc:
            LiveGlobalPlanner._hold_goal(self, str(exc))
            return
        if self.replan_count >= MAX_SNAPSHOT_REPLANS:
            LiveGlobalPlanner._hold_goal(self, 'snapshot_replan_limit_exceeded')
            return
        if (self.goal_deadline_monotonic is None
                or time.monotonic() >= self.goal_deadline_monotonic):
            LiveGlobalPlanner._hold_goal(self, 'total_goal_computation_deadline_expired')
            return
        self.replan_count += 1
        self.static_validator.invalidate()
        self.current = self.pending_goal = None
        self.cached_result = self.cached_evidence = None
        self.commit_wait_started = None
        self.refresh_requested = False
        self.last_reference_stamp = 0.
        if not self.idle_worker_available():
            self.stop_child()
        self.publish_empty('replanning_from_current_body:' + reason)
        self.last_route = {}
        self.plan_intent(intent, context=context, attempt_stamp=self.now_s())

    def _validate_static_result(self, result):
        """Validate immutable native map geometry once, before any commit."""
        return validate_static_route(result,self.tomogram,self.bridge,
            builder=self.source_route_builder, map_version_id=self.pause.intent.identity.map_version_id)

    def _commit_cached_result(self):
        """Lightweight dynamic admission and fresh-stamped publication only."""
        intent, evidence = self.pause.intent, self.cached_evidence
        if intent is None or evidence is None or self.cached_result is None:
            return
        try:
            context = self.planning_context()
            identity = self.confirmed_identity()
        except (GlobalPlanError, ValueError) as exc:
            self.pause_reference(str(exc))
            return
        if (identity != intent.identity or context != (evidence.epoch, evidence.seed_id)
                or evidence.generation != self.generation
                or self.tomogram.sha256 != self.cached_result['source_tomogram_sha256']):
            self.revoke('cached_route_hard_identity_or_map_changed')
            return
        if self.pause.paused_at is not None:
            return  # Requires fresh body/localizer and three new nav samples.
        retain_committed = task_transition(TaskEvent.REFERENCE_REFRESH,
            route_committed=bool(getattr(self, 'route_committed', False))).route is RouteAction.RETAIN
        if not retain_committed and self._start_moved(evidence):
            try:
                self._replan_from_fresh_body(context, 'snapshot_moved_before_commit')
            except Exception as exc:
                self.revoke('snapshot_replan_failed:' + str(exc)[:500])
                self.state = 'planning_failed'
            return
        if (not retain_committed and self.commit_wait_started is not None and
                time.monotonic()-self.commit_wait_started >= self.p['commit_wait_timeout_s']):
            LiveGlobalPlanner._hold_goal(self, 'global_result_commit_wait_expired')
            return
        path = self.cached_result['path_message']
        path.header.stamp = self.get_clock().now().to_msg()
        try:
            if (not fresh_stamp(_stamp_seconds(path.header.stamp), self.now_s(), .2)
                    or self.confirmed_identity() != intent.identity
                    or self.planning_context() != context
                    or not retain_committed and self._start_moved(evidence)):
                raise GlobalPlanError('commit_context_changed_before_publish')
        except (GlobalPlanError, ValueError):
            self.pause_reference('commit_context_changed_before_publish')
            return
        self.path_pub.publish(path)
        # Legacy Path remains a display/diagnostic compatibility output. The
        # BT consumes only the typed, provenance-bound candidate below.
        snapshot = self.cached_result.get('route_snapshot')
        if snapshot is not None:
            self.route_pub.publish(route_message(snapshot, session_id=self.p['session_id'],
                task_id='', route_id='', epoch=evidence.epoch, seed_id=evidence.seed_id,
                stamp=path.header.stamp))
        # A reference ACK refresh is not a new route/display commit. The
        # transient-local visual snapshot stays unchanged while the bridge
        # independently validates current pose, map and collision admission.
        if not retain_committed:
            self.visual_path_pub.publish(path)
            self.visual_path_available = True
        self.active_reference = True
        self.active_context = context
        self.route_committed = True
        self.recovery_hold = None
        self.goal_started_monotonic = self.goal_deadline_monotonic = None
        self.last_reference_stamp = _stamp_seconds(path.header.stamp)
        self.commit_wait_started = None
        self.last_route = dict(goal_stamp=intent.user_stamp,
                               attempt_stamp=evidence.goal_stamp,
                               path_stamp=self.last_reference_stamp,
                               points=len(path.poses), start_moved_m=float(np.linalg.norm(
                                   self.body-np.asarray(evidence.start_body_xyz))),
                               source_tomogram_sha256=self.tomogram.sha256,
                               route_hash=snapshot.route_hash if snapshot is not None else None,
                               projection=self.cached_result['diagnostics'])
        self.state = 'active'
        self.reason = 'estimated_ground_not_collision_validated'
        # A completed route is an event, not a 500 ms diagnostic heartbeat.
        # Publish only after every identity/hash/stamp field is committed so
        # the action adapter can pair the already-published typed candidate.
        # Periodic status remains unchanged for liveness and cancellation.
        self.last_status_at = time.monotonic()
        self.publish_status()

    def _finish(self, packet):
        if not isinstance(packet, dict):
            self.revoke('native_result_packet_invalid')
            self.state = 'planning_failed'
            return
        evidence = self.current
        if evidence is None or packet.get('generation') != evidence.generation:
            return
        if self.worker_generation != evidence.generation:
            return
        if packet.get('kind') == 'progress':
            self.worker_metrics.update({key:packet.get(key) for key in (
                'phase', 'worker_reused', 'initialization_sec') if key in packet})
            return
        if packet.get('kind') != 'planned':
            self.revoke('native_planning_failed:' + str(packet.get('error', 'unknown'))[:500])
            self.state = 'planning_failed'
            return
        try:
            intent = self.pause.intent
            if (intent is None or evidence.epoch != intent.identity.epoch
                    or evidence.seed_id != intent.identity.confirmed_seed
                    or hard_identity_issue(intent.identity, localizer=self.localizer,
                                           navigation=self.navigation, scan={})):
                raise GlobalPlanError('native_result_hard_identity_changed')
            if (self.goal_deadline_monotonic is None
                    or time.monotonic() >= self.goal_deadline_monotonic):
                raise GlobalPlanError('total_goal_computation_deadline_expired')
            self.worker_generation = None
            self.worker_metrics = {key:packet.get(key) for key in (
                'worker_reused', 'initialization_sec', 'worker_elapsed_sec', 'native_plan_elapsed_sec')}
            self.worker_metrics.update(generation=evidence.generation, phase='validating_source_frame')
            self.worker_metrics['native_map_cache'] = packet['result'].get('native_map_cache', {})
            self.worker_metrics['fixed_stair_cache'] = packet['result'].get('fixed_stair_cache', {})
            self.static_validator.submit(evidence.generation,
                partial(prepare_validated_path,packet['result'],self.tomogram,self.bridge,
                    builder=self.source_route_builder,
                    map_version_id=intent.identity.map_version_id))
        except Exception as exc:
            if str(exc) == 'total_goal_computation_deadline_expired':
                LiveGlobalPlanner._hold_goal(self, str(exc))
                return
            self.revoke('result_rejected:' + str(exc)[:500])
            self.state = 'result_rejected'

    def _finish_validation(self, packet):
        generation,validated,error,elapsed = packet
        evidence,intent = self.current,self.pause.intent
        if evidence is None or generation != evidence.generation:
            return
        try:
            if error is not None:
                raise error
            if (intent is None or self.generation != generation
                    or evidence.epoch != intent.identity.epoch
                    or evidence.seed_id != intent.identity.confirmed_seed
                    or hard_identity_issue(intent.identity,localizer=self.localizer,
                                           navigation=self.navigation,scan={})
                    or hard_identity_issue(intent.identity,localizer={},
                                           navigation=self.pose_status,scan={})
                    or validated['source_tomogram_sha256'] != self.tomogram.sha256):
                raise GlobalPlanError('static_result_hard_identity_changed')
            if self.goal_deadline_monotonic is None or time.monotonic() >= self.goal_deadline_monotonic:
                raise GlobalPlanError('total_goal_computation_deadline_expired_after_validation')
            self.cached_result,self.cached_evidence = validated,evidence
            self.current = None
            self.worker_metrics.update(phase='validated',parent_validation_elapsed_sec=elapsed,
                message_build_sec=validated.get('message_build_sec'),
                request_attempt_elapsed_sec=time.monotonic()-evidence.issued_monotonic,
                total_goal_elapsed_sec=time.monotonic()-self.goal_started_monotonic)
            self.state = 'awaiting_navigation_to_commit'
            self.reason = 'native_map_route_validated_once'
            if self.pause.paused_at is not None:
                self.commit_wait_started = time.monotonic()
            else:
                self._commit_cached_result()
        except Exception as exc:
            if str(exc) == 'total_goal_computation_deadline_expired_after_validation':
                LiveGlobalPlanner._hold_goal(self, str(exc))
                return
            self.revoke('result_rejected:' + str(exc)[:500])
            self.state = 'result_rejected'

    def tick(self):
        self.retired_children.reap()
        LiveGlobalPlanner._poll_warmup(self)
        self._revoke_if_context_lost()
        if (self.current is not None and self.goal_deadline_monotonic is not None
                and time.monotonic() >= self.goal_deadline_monotonic):
            LiveGlobalPlanner._hold_goal(self, 'total_goal_computation_deadline_expired')
        if self.pending_goal is not None:
            self._maybe_launch_pending()
        if self.current is not None:
            if self.pending_goal is not None:
                pass  # Never poll a child while its launch is deferred.
            elif self.pipe is not None and self.pipe.poll(0):
                # At most two progress packets plus one result per request.
                # Drain this bounded batch without adding one timer period
                # of artificial latency to every worker phase.
                for _ in range(4):
                    if self.current is None or self.pipe is None or not self.pipe.poll(0):
                        break
                    try:
                        packet = self.pipe.recv()
                    except (EOFError, OSError) as exc:
                        packet = {'kind': 'failed', 'generation': self.current.generation,
                                  'error': str(exc)}
                    self._finish(packet)
            elif self.child is not None and not self.child.is_alive():
                self.revoke('native_worker_exited_without_result')
                self.state = 'planning_failed'
        elif self.child is not None and not self.child.is_alive():
            # Idle exit does not erase a separately validated visible route.
            self.stop_child()
        validation = self.static_validator.poll()
        if validation is not None:
            self._finish_validation(validation)
        mono = time.monotonic()
        if mono-self.last_status_at >= .5:
            self.last_status_at = mono
            self.publish_status()

    def publish_status(self):
        mono = time.monotonic()
        intent = self.pause.intent
        active_goal = (dict(user_stamp=intent.user_stamp, kind=intent.kind,
            frame_id=intent.frame_id, xyz=list(intent.xyz), floor_id=intent.floor_id,
            session_id=intent.identity.session_id, epoch=intent.identity.epoch,
            seed_id=intent.identity.confirmed_seed,
            map_version_id=intent.identity.map_version_id,
            tomogram_sha256=intent.identity.tomogram_sha256)
            if intent is not None else None)
        scan_fresh = mono-self.scan_received <= self.p['freshness_s']
        if getattr(self, 'route_committed', False):
            # Planning is complete even when following inputs are unavailable.
            # paused_for_recovery/active_reference describe that separate lease.
            phase = 'active'
        elif getattr(self, 'recovery_hold', None) is not None:
            phase = 'goal_retained_waiting_recovery'
        elif self.state == 'expired':
            phase = 'expired'
        elif self.current is not None:
            phase = 'computing'
        elif self.pause.paused_at is not None:
            phase = 'awaiting_navigation'
        elif self.cached_result is not None and self.active_reference:
            phase = ('active' if scan_fresh and self.scan.get('active_reference') is True
                     else 'awaiting_local_sensor')
        elif self.cached_result is not None:
            phase = 'awaiting_navigation'
        else:
            phase = 'idle'
        native_elapsed = (mono-self.goal_started_monotonic
                          if self.current is not None and self.goal_started_monotonic is not None
                          else self.worker_metrics.get('total_goal_elapsed_sec'))
        native_remaining = (max(0., self.goal_deadline_monotonic-mono)
                            if self.current is not None and self.goal_deadline_monotonic is not None
                            else None)
        commit_elapsed = (mono-self.commit_wait_started
                          if self.commit_wait_started is not None else None)
        commit_remaining = (max(0., self.p['commit_wait_timeout_s']-commit_elapsed)
                            if commit_elapsed is not None else None)
        value = dict(schema=1, mode='LIVE_SHADOW_NO_MOTION', session_id=self.p['session_id'],
                     received_at_unix=self.now_s(),callback_wall_time=time.time(),state=self.state,reason=self.reason,
                     planning_phase=phase, replan_count=self.replan_count,
                     task_policy=POLICY_ID,
                     native_elapsed_sec=native_elapsed,
                     native_deadline_remaining_sec=native_remaining,
                     commit_wait_elapsed_sec=commit_elapsed,
                     commit_wait_remaining_sec=commit_remaining,
                     generation=self.generation, goal_kind=self.goal_kind,
                     last_goal_stamp=self.last_goal_stamp,
                     computation_cycle=getattr(self, 'computation_cycle', 0),
                     route_committed=getattr(self, 'route_committed', False),
                     recovery_hold=getattr(self, 'recovery_hold', None),
                     active_goal=active_goal,
                     retain_preview_task_on_soft_loss=self.p['retain_preview_task_on_soft_loss'],
                     goal_stamp=self.goal_stamp, active_reference=self.active_reference,
                     visual_path_available=self.visual_path_available,
                     visual_path_execution_authorized=False,
                     goal_retained=self.pause.intent is not None,
                     paused_for_recovery=self.pause.paused_at is not None,
                     pause_remaining_sec=None,
                     pause_good_samples=self.pause.good_count,
                     planning=self.current is not None,
                     validated_route_cached=self.cached_result is not None,
                     pending_worker_start=self.pending_goal is not None,
                     retired_native_workers=len(self.retired_children.pending),
                     resident_native_worker=self.child is not None,
                     worker_generation=self.worker_generation,
                     worker_metrics=self.worker_metrics,
                     native_warmup=getattr(self,'warmup',{}),
                     localization_valid=self.pose_status.get('valid') is True,
                     navigation_ready=self.navigation.get('navigation_ready') is True,
                     scan_sensor_ready=self.scan.get('sensor_ready') is True,
                     localization_epoch=self.navigation.get('epoch'),
                     localization_seed_id=self.navigation.get('seed_id'),
                     original_frame=self.bridge.source_frame,
                     planning_frame=self.bridge.planning_frame,
                     no_tf_bridge=True, estimated_ground_shadow_only=True,
                     collision_validated=False, hardware_validated=False,
                     body_height_gate_debug_not_calibration=True,
                     motion_enabled=False, sdk_command_published=False,
                     last_route=self.last_route)
        encoded = json.dumps(value, ensure_ascii=False, allow_nan=False)
        self.status_pub.publish(String(data=encoded))
        if self.p['output_directory']:
            directory = Path(self.p['output_directory'])
            if not directory.is_dir():
                self.get_logger().error('global_status_output_directory_missing')
                return
            target = directory / 'global_status.json'
            temporary = directory / 'global_status.json.tmp'
            try:
                temporary.write_text(encoded + '\n')
                os.replace(temporary, target)
            except OSError as exc:
                self.get_logger().error('global_status_write_failed: ' + str(exc))

    def close(self):
        try:
            self.revoke('global_planner_shutdown', publish=rclpy.ok())
        finally:
            self.retired_children.drain_on_shutdown()
            self.static_validator.close()
            self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LiveGlobalPlanner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            node.close()
        except RCLError:
            # Shutdown can close the context while the final empty Path is
            # being published. Retired native children were still drained.
            if rclpy.ok():
                raise
        try:
            rclpy.try_shutdown()
        except RCLError:
            if rclpy.ok():
                raise


if __name__ == '__main__':
    main()
