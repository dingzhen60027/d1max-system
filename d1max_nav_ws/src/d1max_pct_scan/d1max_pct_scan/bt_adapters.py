"""Asynchronous ROS actions around the existing PCT and SCAN implementations.

The BT owns task sequencing. This adapter is the sole public reference owner;
PCT outputs are private candidates, and SCAN keeps its native collision checks.
No SDK client, velocity publisher, controller, or automatic localization input.
"""
from copy import deepcopy
import json
import math
import os
import time

import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.task import Future
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Empty, String
from d1max_navigation_bt_interfaces.action import ComputeRoute, FollowRoute
from d1max_navigation_bt_interfaces.msg import NavigationHealth, RouteSnapshot as RouteSnapshotMessage
from d1max_navigation_bt_interfaces.msg import RouteReference
from d1max_planning_interfaces.msg import LocalPlanDebug

from .bt_adapter_contract import (ArrivalEvidence, route_fingerprint,
                                  stamp_seconds, valid_compute_request, valid_task)
from .bt_follow_policy import (FOLLOW_DEFAULTS, FollowRecovery, classify_follow,
                               validate_follow_policy)
from .live_scan_contract import (fresh, localization_identity_context,
                                 validate_continuous_pose)
from .source_route_ros import from_message as decode_route, to_message as encode_route
from .atomic_navigation_inbox import uses_atomic_navigation

PREFIX = '/d1max/live_planning/'
COMPUTE_FAILURE_STATES = frozenset(('goal_rejected', 'planning_failed', 'expired', 'result_rejected'))


def same_execution_task(a,b):
    """A measured SDK stop covers the execution, not one reference window.

    SDK ownership binds the initial task once. Its stop report may retain the
    initial anchor while the BT has committed later validated windows. Those
    geometry versions must not erase a true stop, nor may another task's stop
    satisfy this execution. SDK/execution/control/arm IDs are checked separately.
    """
    return (a.schema_version==b.schema_version==3 and all(getattr(a,key)==getattr(b,key)
        for key in ('session_id','task_id','route_id','route_hash','map_version_id',
                    'localization_epoch','localization_seed_id')))


class NavigationBTAdapters(Node):
    def __init__(self):
        super().__init__('d1max_navigation_bt_adapters')
        defaults = dict(session_id='', localization_session_id='', execution_mode='preview',
            pipeline_contract='',map_version_id='',transport_mode='live',
            expected_source_map_sha256='', expected_tomogram_sha256='', expected_conditioning_sha256='',
            map_frame='d1max_loc_map', body_frame='d1max_loc_base_link',
            tracking_frame='d1max_loc_tracking', freshness_s=.5, result_timeout_s=60.,
            worker_status_timeout_s=1.,worker_startup_timeout_s=60.,local_state_enabled=False,
            cancel_timeout_s=2.5, body_height=.55, goal_xy_tolerance_m=.20,
            goal_z_tolerance_m=.15)
        defaults.update(FOLLOW_DEFAULTS)
        self.p = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        validate_follow_policy(self.p)
        if (os.environ.get('RMW_IMPLEMENTATION') != 'rmw_zenoh_cpp'
                or not self.p['session_id'] or len(self.p['session_id']) > 128
                or not self.p['localization_session_id'] or self.p['execution_mode'] not in ('preview','single_floor')
                or self.p['execution_mode']=='single_floor' and (self.p['pipeline_contract']!='single_floor_v3'
                    or not self.p['map_version_id'] or self.p['transport_mode'] not in ('live','isolated_mock'))
                or not .1 <= self.p['freshness_s'] <= .5
                or not .5 <= self.p['worker_status_timeout_s'] <= 1.5
                or not 10 <= self.p['worker_startup_timeout_s'] <= 60
                or not 5 <= self.p['result_timeout_s'] <= 60
                or not .5 <= self.p['cancel_timeout_s'] <= 3
                or not .1 <= self.p['body_height'] <= 1.
                or not .05 <= self.p['goal_xy_tolerance_m'] <= .3
                or not .05 <= self.p['goal_z_tolerance_m'] <= .2):
            raise ValueError('bt_adapters_require_bounded_preview_zenoh_configuration')
        if any(len(self.p[k]) != 64 or any(c not in '0123456789abcdef' for c in self.p[k])
               for k in ('expected_source_map_sha256', 'expected_tomogram_sha256',
                         'expected_conditioning_sha256')):
            raise ValueError('bt_adapters_require_exact_route_artifact_hashes')
        self.initialize_state()
        durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.goal_pub = self.create_publisher(PoseStamped, PREFIX+'bt/worker_goal', 5)
        self.goal3d_pub = self.create_publisher(PointStamped, PREFIX+'bt/worker_goal3d', 5)
        self.worker_cancel_pub = self.create_publisher(Empty, PREFIX+'bt/worker_cancel', 5)
        self.reference_pub = self.create_publisher(Path, PREFIX+'reference_path', durable)
        self.route_reference_pub = self.create_publisher(RouteReference, PREFIX+'committed_route', durable)
        self.health_pub = self.create_publisher(NavigationHealth, PREFIX+'bt/health', 5)
        self.create_subscription(RouteSnapshotMessage, PREFIX+'bt/worker_route', self.on_candidate, durable)
        self.create_subscription(String, PREFIX+'global_status', self.on_worker_status, 5)
        self.create_subscription(String, PREFIX+'scan_bridge_status', self.on_scan_status, 5)
        self.create_subscription(LocalPlanDebug, PREFIX+'native_local_debug', self.on_native_debug, 5)
        self.create_subscription(String, PREFIX+'reference_refresh_request', self.on_refresh, 5)
        if uses_atomic_navigation(self.p):
            from d1max_planning_interfaces.msg import NavigationState,ExecutionPermit,StopReport
            self.create_subscription(NavigationState,'/d1max/localization/navigation/state',self.on_atomic_state,qos_profile_sensor_data)
            self.create_subscription(ExecutionPermit,PREFIX+'execution/permit',self.on_execution_permit,5)
            self.create_subscription(StopReport,PREFIX+'execution/stop_report',self.on_stop_report,5)
            if self.p['local_state_enabled']:
                from d1max_planning_interfaces.msg import LocalNavigationState
                self.create_subscription(LocalNavigationState,'/d1max/localization/navigation/local_state',
                    self.on_local_state,qos_profile_sensor_data)
        else:
            self.create_subscription(String, '/d1max/localization/status', self.on_localizer, 5)
            self.create_subscription(String, '/d1max/localization/navigation/status', self.on_navigation, 5)
            self.create_subscription(String, '/d1max/localization/navigation/pose_status', self.on_pose, qos_profile_sensor_data)
            self.create_subscription(Odometry, '/d1max/localization/odometry/global', self.on_body, qos_profile_sensor_data)
        self.compute_server = ActionServer(self, ComputeRoute, PREFIX+'bt/compute_route',
            self.execute_compute, goal_callback=self.compute_goal,
            handle_accepted_callback=self.accept_compute, cancel_callback=self.cancel_compute)
        self.follow_server = ActionServer(self, FollowRoute, PREFIX+'bt/follow_route',
            self.execute_follow, goal_callback=self.follow_goal,
            handle_accepted_callback=self.accept_follow, cancel_callback=self.cancel_follow)
        self.timer = self.create_timer(.05, self.tick)

    def initialize_state(self):
        self.compute = self.follow = self.snapshot = None
        self.reserved_compute = self.reserved_follow = False
        self.quarantine = ''
        self.localizer = self.navigation = self.pose = self.scan = self.worker = {}
        self.received = {k: -math.inf for k in ('localizer', 'navigation', 'pose', 'scan', 'worker')}
        self.body = None
        self.body_yaw = None
        self.body_identity = None
        self.body_received = -math.inf
        self.body_source = self.last_goal_stamp = self.last_reference_stamp = 0.
        self.body_barrier = self.now_s()
        self.observed_identity = None
        self.candidates = {}
        self.executions = {}
        self.native_cancel_ack = None
        self.arrival = ArrivalEvidence()
        self.used_tasks = set()
        self.last_feedback = -math.inf
        self.reference_delivery_sequence = 0
        self.atomic_state=None
        from .atomic_navigation_inbox import AtomicNavigationInbox
        self.atomic_inbox=AtomicNavigationInbox()
        from .local_navigation_state import LocalNavigationInbox
        self.local_inbox=LocalNavigationInbox()
        self.execution_permit=None
        self.stop_report=None
        self.permit_received=self.stop_received=-math.inf

    def on_atomic_state(self,message):
        event=self.atomic_inbox.accept(message,session_id=self.p['session_id'],
            map_version_id=self.p['map_version_id'],now_ns=self.get_clock().now().nanoseconds,
            monotonic=time.monotonic())
        if event is None:
            return  # A rejected packet does not erase, or renew, prior evidence.
        self.atomic_state=state=event.state
        self.body_identity=(event.identity[1],event.identity[2],message.map_version_id)
        self.body_source=event.source_ns*1e-9
        self.body_received=event.received
        if state is None:
            self.body=self.body_yaw=None
            # A soft sensor gap revokes current pose admission, not the fixed
            # user route. The execution owner enforces HOLD/recovery. Explicit
            # hard loss cannot be hidden by recovery before the next timer.
            if event.hard_failure:
                self.execution_permit=self.stop_report=None
                if self.compute is not None:
                    self.begin_cancel(self.compute,'atomic_navigation_hard_loss',compute=True)
                if self.follow is not None:
                    self.begin_cancel(self.follow,'atomic_navigation_hard_loss',compute=False)
            return
        self.body=tuple(state.global_body.position)
        q=message.global_odometry.pose.pose.orientation
        self.body_yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

    def on_local_state(self,message):
        event=self.local_inbox.accept(message,session_id=self.p['session_id'],
            map_version_id=self.p['map_version_id'],now_ns=self.get_clock().now().nanoseconds,
            monotonic=time.monotonic())
        if event is not None and event.hard_failure:
            self.execution_permit=self.stop_report=None
            if self.compute is not None: self.begin_cancel(self.compute,'local_navigation_hard_loss',compute=True)
            if self.follow is not None: self.begin_cancel(self.follow,'local_navigation_hard_loss',compute=False)

    def ready_local_identity(self):
        if not self.p.get('local_state_enabled'):
            return self.ready_identity()
        if not self.local_inbox.usable(now_ns=self.get_clock().now().nanoseconds,
                monotonic=time.monotonic(),receipt_timeout_s=self.p['freshness_s']):
            raise ValueError('waiting_fresh_local_navigation_state')
        event=self.local_inbox.latest
        context=(*event.identity[1:],self.p['map_version_id'])
        problem=self.hard_reason(context)
        if problem: raise ValueError(problem)
        return context

    def ready_local_task_identity(self):
        """Reference workflow capability, separate from instantaneous control."""
        if not self.p.get('local_state_enabled'):
            return self.ready_identity()
        if not self.local_inbox.task_usable(now_ns=self.get_clock().now().nanoseconds,
                monotonic=time.monotonic(),receipt_timeout_s=self.p['freshness_s']):
            raise ValueError('waiting_fresh_local_navigation_state')
        event=self.local_inbox.latest
        context=(*event.identity[1:],self.p['map_version_id'])
        problem=self.hard_reason(context)
        if problem: raise ValueError(problem)
        return context

    def on_execution_permit(self,message):
        snapshot=self.snapshot
        v=message.version
        if (snapshot is None or v.schema_version!=3 or v.session_id!=self.p['session_id']
                or v.task_id!=snapshot['task_id'] or v.route_id!=snapshot['route_id']
                or v.route_hash!=snapshot['source_snapshot'].route_hash
                or v.map_version_id!=snapshot['context'][2]
                or (v.localization_epoch,v.localization_seed_id)!=snapshot['context'][:2]
                or message.transport_mode!=self.p.get('transport_mode','live')
                or not fresh(stamp_seconds(message.source_stamp),self.now_s(),.4)
                or self.now_s()>stamp_seconds(message.valid_until)
                or self.execution_permit is not None and message.sequence<=self.execution_permit.sequence):
            return
        self.execution_permit=deepcopy(message); self.permit_received=time.monotonic()

    def on_stop_report(self,message):
        permit=self.execution_permit
        if (permit is None or not same_execution_task(message.version,permit.version) or message.execution_id!=permit.execution_id
                or message.control_epoch!=permit.control_epoch or message.sdk_session!=permit.sdk_session
                or message.sdk_arm_generation!=permit.sdk_arm_generation
                or message.transport_mode!=self.p.get('transport_mode','live')
                or not fresh(stamp_seconds(message.source_stamp),self.now_s(),.4)
                or self.stop_report is not None and message.sequence<=self.stop_report.sequence): return
        self.stop_report=deepcopy(message); self.stop_received=time.monotonic()

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def fresh_status(self, key):
        timeout = self.p.get('worker_status_timeout_s', 1.) if key == 'worker' else self.p['freshness_s']
        stamp_key = 'wall_time' if key == 'localizer' else 'received_at_unix'
        return (0 <= time.monotonic()-self.received[key] <= timeout
                and fresh(getattr(self, key).get(stamp_key), self.now_s(), timeout))

    def receive(self, message, key, stamp_key='received_at_unix', session=None):
        try:
            if len(message.data) > 2_000_000:
                return
            value = json.loads(message.data)
            previous = getattr(self, key)
            timeout = self.p.get('worker_status_timeout_s', 1.) if key == 'worker' else self.p['freshness_s']
            if (not isinstance(value, dict) or not fresh(value.get(stamp_key), self.now_s(),
                    timeout) or value.get(stamp_key, 0) <= previous.get(stamp_key, 0)
                    or session is not None and value.get('session_id') != session):
                return
            setattr(self, key, value)
            self.received[key] = time.monotonic()
        except (ValueError, TypeError, AttributeError):
            return

    def on_localizer(self, message):
        self.receive(message, 'localizer', 'wall_time', self.p['localization_session_id'])

    def on_navigation(self, message):
        self.receive(message, 'navigation')

    def on_pose(self, message):
        self.receive(message, 'pose')

    def on_scan_status(self, message):
        self.receive(message, 'scan', session=self.p['session_id'])

    def on_worker_status(self, message):
        self.receive(message, 'worker', session=self.p['session_id'])
        # React to completed work; the 50 ms timer remains a watchdog and
        # startup fallback, not an extra scheduling delay on every Action.
        if self.compute is not None:
            self.compute_tick()

    def on_native_debug(self, message):
        stamp = stamp_seconds(message.header.stamp)
        if (message.session_id != self.p['session_id'] or message.phase != 'cancelled'
                or message.valid or message.header.frame_id != ('d1max_loc_odom' if uses_atomic_navigation(self.p) else self.p['map_frame'])
                or not fresh(stamp, self.now_s(), self.p['freshness_s'])):
            return
        if self.native_cancel_ack is not None and stamp <= self.native_cancel_ack['stamp']:
            return
        self.native_cancel_ack = dict(stamp=stamp, received=time.monotonic(), generation=message.generation)

    def identity(self):
        if uses_atomic_navigation(self.p):
            if not self.atomic_inbox.usable(now_ns=self.get_clock().now().nanoseconds,
                    monotonic=time.monotonic(),receipt_timeout_s=self.p['freshness_s']):
                raise ValueError('waiting_fresh_atomic_navigation_state')
            return self.body_identity
        if not self.fresh_status('localizer') or not self.fresh_status('navigation'):
            raise ValueError('waiting_localization_identity')
        _, epoch, seed = localization_identity_context(self.localizer, self.navigation,
            session_id=self.p['localization_session_id'], now=self.now_s(),
            timeout=self.p['freshness_s'], map_frame=self.p['map_frame'],
            tracking_frame=self.p['tracking_frame'])
        return epoch, seed, self.localizer.get('map_version_id', '')

    def hard_reason(self, expected=None):
        if self.quarantine:
            return self.quarantine
        if uses_atomic_navigation(self.p):
            event=self.atomic_inbox.latest
            local=self.local_inbox.latest if self.p.get('local_state_enabled') else None
            if local is not None and (local.hard_failure or expected is not None
                    and (*local.identity[1:],self.p['map_version_id'])!=expected):
                return 'local_navigation_hard_loss_or_identity_changed'
            if event is not None and event.hard_failure:
                return 'atomic_navigation_hard_loss'
            if expected is not None and event is not None and self.body_identity!=expected:
                return 'atomic_navigation_identity_changed'
            return ''
        for key in ('localizer', 'navigation', 'pose', 'scan'):
            if not self.fresh_status(key):
                continue
            value = getattr(self, key)
            if (value.get('fault') or value.get('local_fault') or value.get('reset_pending')
                    or value.get('native_map_context_fault') or value.get('motion_enabled') is True
                    or value.get('motion_control_enabled') is True
                    or isinstance(value.get('frontend'), dict) and value['frontend'].get('fault')):
                return key+'_explicit_fault_or_reset'
            if expected is not None:
                epoch_key = 'local_epoch' if key == 'localizer' else (
                    'localization_epoch' if key == 'scan' else 'epoch')
                seed_key = 'confirmed_seed_ns' if key == 'localizer' else (
                    'localization_seed_id' if key == 'scan' else 'seed_id')
                if (value.get(epoch_key) not in (None, 0, expected[0])
                        or value.get(seed_key) not in (None, '', expected[1])):
                    return key+'_identity_changed'
                if key == 'localizer' and value.get('map_version_id') != expected[2]:
                    return 'localizer_map_version_changed'
        return ''

    def ready_identity(self):
        context = self.identity()
        problem = self.hard_reason(context)
        if problem:
            raise ValueError(problem)
        if uses_atomic_navigation(self.p):
            return context
        validate_continuous_pose(self.pose, epoch=context[0], seed=context[1],
            now=self.now_s(), map_frame=self.p['map_frame'], body_frame=self.p['body_frame'],
            timeout=self.p['freshness_s'], receipt_age=time.monotonic()-self.received['pose'])
        if (self.body_identity != context or self.body is None
                or not fresh(self.body_source, self.now_s(), self.p['freshness_s'])
                or not 0 <= time.monotonic()-self.body_received <= self.p['freshness_s']):
            raise ValueError('waiting_current_body_odometry')
        return context

    def on_body(self, message):
        try:
            context = self.identity()
            if context != self.observed_identity:
                self.observed_identity = context
                self.body_barrier = self.now_s()
                self.body = self.body_identity = None
            stamp = stamp_seconds(message.header.stamp)
            if (message.header.frame_id != self.p['map_frame']
                    or message.child_frame_id != self.p['body_frame']
                    or not fresh(stamp, self.now_s(), self.p['freshness_s'])
                    or stamp <= max(self.body_barrier, self.body_source)):
                return
            validate_continuous_pose(self.pose, epoch=context[0], seed=context[1],
                now=self.now_s(), map_frame=self.p['map_frame'], body_frame=self.p['body_frame'],
                timeout=self.p['freshness_s'], receipt_age=time.monotonic()-self.received['pose'])
            position = tuple(getattr(message.pose.pose.position, a) for a in 'xyz')
            q = message.pose.pose.orientation
            if (not all(math.isfinite(v) for v in (*position,q.x,q.y,q.z,q.w))
                    or abs(math.hypot(q.x,q.y,q.z,q.w)-1.) > .01):
                return
            self.body, self.body_identity, self.body_source = position, context, stamp
            self.body_yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            self.body_received = time.monotonic()
        except ValueError:
            return

    def compute_goal(self, request):
        if (self.compute or self.follow or self.reserved_compute or self.reserved_follow
                or self.quarantine or request.task_id in self.used_tasks
                or not valid_compute_request(request, self.p['session_id'], self.p['map_frame'])):
            return GoalResponse.REJECT
        self.reserved_compute = True
        return GoalResponse.ACCEPT

    def accept_compute(self, handle):
        self.reserved_compute = False
        self.snapshot = None
        self.execution_permit=self.stop_report=None
        self.candidates.clear()
        self.used_tasks.add(handle.request.task_id)
        if len(self.used_tasks) > 4096:
            # Session-level budget, not silent task-ID reuse.
            self.quarantine = 'task_budget_requires_new_session'
        self.compute = dict(handle=handle, future=Future(), began=time.monotonic(),
                            dispatched=False, cancel=None, context=None, goal_stamp=0.,
                            wait_phase='waiting_worker', wait_reason='waiting_private_worker_ready')
        self.executions[id(handle)] = self.compute
        handle.execute()
        self.compute_tick()

    async def execute_compute(self, handle):
        slot = self.executions.get(id(handle))
        if slot is None or slot['handle'] is not handle:
            handle.abort()
            return ComputeRoute.Result(schema_version=2, success=False, reason='obsolete_compute_handle')
        try:
            return await slot['future']
        finally:
            self.executions.pop(id(handle), None)

    def cancel_compute(self, handle):
        if self.compute is None or self.compute['handle'] is not handle:
            return CancelResponse.REJECT
        if self.compute['cancel'] and self.compute['cancel']['reason'] != 'user_cancel':
            return CancelResponse.REJECT
        self.begin_cancel(self.compute, 'user_cancel', compute=True)
        return CancelResponse.ACCEPT

    def follow_goal(self, request):
        snapshot = self.snapshot
        try:
            candidate = decode_route(request.snapshot)
            wire = request.snapshot
            valid = (request.schema_version == 2 and request.execution_confirmed is False
                and request.confirmation_id == '' and valid_task(request, self.p['session_id']) and snapshot is not None
                and request.task_id == snapshot['task_id'] and request.route_id == snapshot['route_id']
                and (request.localization_epoch, request.localization_seed_id) == snapshot['context'][:2]
                and wire.session_id == request.session_id and wire.task_id == request.task_id
                and wire.route_id == request.route_id
                and (wire.localization_epoch, wire.localization_seed_id) == snapshot['context'][:2]
                and candidate.route_hash == snapshot['digest']
                and candidate.encoded == snapshot['source_snapshot'].encoded
                and route_fingerprint(request.route, self.p['map_frame'])
                    == route_fingerprint(wire.path, self.p['map_frame']))
        except (ValueError, TypeError, AttributeError):
            valid = False
        if (not valid or self.compute or self.follow or self.reserved_compute
                or self.reserved_follow or self.quarantine):
            return GoalResponse.REJECT
        self.reserved_follow = True
        return GoalResponse.ACCEPT

    def accept_follow(self, handle):
        self.reserved_follow = False
        self.follow = dict(handle=handle, future=Future(), context=self.snapshot['context'],
            committed=False, cancel=None, reference_stamp=0., refresh_count=0,
            refresh_deadline=None, last_refresh_id=0, started=time.monotonic(),
            recovery=FollowRecovery(self.p), measured_arrival_stamp=0.,
            wait_phase='waiting_reference', wait_reason='waiting_reference_delivery')
        self.executions[id(handle)] = self.follow
        self.arrival.reset()
        handle.execute()

    async def execute_follow(self, handle):
        slot = self.executions.get(id(handle))
        if slot is None or slot['handle'] is not handle:
            handle.abort()
            return FollowRoute.Result(schema_version=2, success=False, reason='obsolete_follow_handle')
        try:
            return await slot['future']
        finally:
            self.executions.pop(id(handle), None)

    def cancel_follow(self, handle):
        if self.follow is None or self.follow['handle'] is not handle:
            return CancelResponse.REJECT
        if self.follow['cancel'] and self.follow['cancel']['reason'] != 'user_cancel':
            return CancelResponse.REJECT
        self.begin_cancel(self.follow, 'user_cancel', compute=False)
        return CancelResponse.ACCEPT

    def stamped_path(self, path):
        result = deepcopy(path)
        # Existing bridge compares seconds as doubles; 1 ns is not distinct at
        # Unix-epoch magnitudes. A microsecond is still within its future budget.
        stamp_ns = max(self.get_clock().now().nanoseconds, int(self.last_reference_stamp*1e9)+1000)
        result.header.stamp.sec, result.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
        self.last_reference_stamp = stamp_ns*1e-9
        for pose in result.poses:
            pose.header = result.header
        return result

    def publish_reference(self, slot):
        message = self.stamped_path(self.snapshot['route'])
        self.publish_route_delivery(message.header.stamp, active=True)
        # Display compatibility only. The production bridge trusts the typed
        # RouteReference envelope and never a Path-only task.
        self.reference_pub.publish(message)
        slot['committed'], slot['reference_stamp'] = True, stamp_seconds(message.header.stamp)
        slot['reference_stamp_ns'] = message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
        slot['reference_delivery_sequence'] = self.reference_delivery_sequence

    def publish_route_delivery(self, stamp, *, active):
        snapshot = self.snapshot
        self.reference_delivery_sequence += 1
        wire = encode_route(snapshot['source_snapshot'], session_id=self.p['session_id'],
            task_id=snapshot['task_id'], route_id=snapshot['route_id'],
            epoch=snapshot['context'][0], seed_id=snapshot['context'][1], stamp=stamp)
        self.route_reference_pub.publish(RouteReference(schema_version=2,
            source_stamp=stamp, session_id=wire.session_id, task_id=wire.task_id,
            route_id=wire.route_id, route_hash=wire.route_hash,
            delivery_sequence=self.reference_delivery_sequence, active=active, snapshot=wire))

    def begin_cancel(self, slot, reason, *, compute):
        if slot['cancel'] is not None:
            return
        slot['cancel'] = dict(reason=reason, wall=self.now_s(), mono=time.monotonic(),
                              generation=self.scan.get('generation', 0))
        if compute:
            # Nothing was sent while waiting for heavy worker startup. The
            # active slot itself proves there can be no old worker request to
            # retire, so a missing subscriber must not quarantine this cancel.
            if slot['dispatched']:
                self.worker_cancel_pub.publish(Empty())
        else:
            if slot['committed']:
                empty = Path()
                empty.header.frame_id = self.p['map_frame']
                message = self.stamped_path(empty)
                self.publish_route_delivery(message.header.stamp, active=False)
                self.reference_pub.publish(message)
            # Compute succeeded earlier, but the legacy worker still owns its
            # completed intent/display. End that private owner as part of the
            # same terminal fence; soft pauses never reach this method.
            self.worker_cancel_pub.publish(Empty())

    def worker_retired(self, cancel):
        status = self.worker
        return (self.fresh_status('worker') and self.received['worker'] > cancel['mono']
            and status.get('received_at_unix', 0.) > cancel['wall']
            and status.get('planning') is False
            and status.get('pending_worker_start') is False
            and status.get('retired_native_workers') == 0
            and status.get('active_goal') is None and status.get('active_reference') is False)

    def finish(self, slot, result, *, compute, canceled=False):
        if slot['future'].done():
            return
        handle = slot['handle']
        if canceled:
            handle.canceled()
        elif result.success:
            handle.succeed()
        else:
            handle.abort()
        slot['future'].set_result(result)
        if compute and self.compute is slot:
            self.compute = None
        elif not compute and self.follow is slot:
            self.follow = None
            # A completed/failed/cancelled Follow consumes its computed route.
            # A duplicate old Action request cannot resurrect that task.
            self.snapshot = None

    def cancellation_tick(self, slot, *, compute):
        cancel = slot['cancel']
        if cancel is None:
            return False
        if compute and not slot['dispatched']:
            self.finish(slot, ComputeRoute.Result(schema_version=2, success=False, reason=cancel['reason'],
                        retirement_confirmed=True),
                compute=True, canceled=cancel['reason'] == 'user_cancel')
            return True
        key = 'worker' if compute else 'scan'
        status = getattr(self, key)
        fresh_ack = (self.fresh_status(key) and self.received[key] > cancel['mono']
                     and status.get('received_at_unix', 0.) > cancel['wall'])
        if compute or not slot['committed']:
            # Follow accepted but no public reference ever sent: only the
            # completed private global intent exists. Do not invent a native
            # operation/ACK by publishing an empty reference to a missing peer.
            acknowledged = self.worker_retired(cancel)
        else:
            native = self.native_cancel_ack
            acknowledged = (fresh_ack and status.get('active_reference') is False
                and type(status.get('generation')) is int
                and status['generation'] >= cancel['generation']
                and native is not None and native['generation'] == status['generation']
                and native['received'] > cancel['mono'] and native['stamp'] > cancel['wall']
                # This one-shot ACK was source-age checked by its callback.
                # Keep the proven cancellation event while waiting for the
                # slower worker retirement ACK within the fixed fence budget.
                and self.worker_retired(cancel))
        expired = time.monotonic()-cancel['mono'] > self.p['cancel_timeout_s']
        if acknowledged or expired:
            cls = ComputeRoute if compute else FollowRoute
            success = acknowledged and cancel['reason'] == 'goal_reached'
            if expired and not acknowledged:
                self.quarantine = 'cancel_ack_timeout_requires_session_restart'
            reason = cancel['reason'] if acknowledged else self.quarantine
            result = cls.Result(schema_version=2, success=success, reason=reason, retirement_confirmed=acknowledged)
            if not compute and success:
                ns = round(slot['measured_arrival_stamp']*1e9)
                result.measured_arrival_stamp.sec, result.measured_arrival_stamp.nanosec = divmod(ns, 1_000_000_000)
                if uses_atomic_navigation(self.p):
                    result.physical_stop_confirmed=bool(self.stop_report and self.stop_report.measured_stop_confirmed
                        and self.stop_report.transport_mode=='live')
            self.finish(slot, result, compute=compute,
                        canceled=acknowledged and cancel['reason'] == 'user_cancel')
        return True

    def on_candidate(self, message):
        slot = self.compute
        if slot is None or not slot['dispatched'] or slot['cancel']:
            return
        try:
            snapshot = decode_route(message)
            value = snapshot.payload()
            if (message.session_id != self.p['session_id']
                    or message.task_id != '' or message.route_id != ''
                    or (message.localization_epoch, message.localization_seed_id) != slot['context'][:2]
                    or value['map_version_id'] != slot['context'][2]
                    or any(value[k] != self.p['expected_'+k] for k in
                           ('source_map_sha256', 'tomogram_sha256', 'conditioning_sha256'))):
                return
            stamp = stamp_seconds(message.path.header.stamp)
            if not fresh(stamp, self.now_s(), 2.) or stamp < slot['goal_stamp']:
                return
            # Keep the already fully validated immutable value. Re-decoding
            # thousands of Path poses on the next tick adds no trust boundary;
            # retaining a mutable wire message would require another copy.
            self.candidates[stamp] = dict(source=snapshot,stamp=deepcopy(message.path.header.stamp))
            while len(self.candidates) > 4:
                self.candidates.pop(next(iter(self.candidates)))
        except (ValueError, TypeError, AttributeError):
            return
        self.compute_tick()

    def compute_tick(self):
        slot = self.compute
        if slot is None or self.cancellation_tick(slot, compute=True):
            return
        hard = self.hard_reason(slot['context'])
        if hard:
            self.begin_cancel(slot, hard, compute=True)
            return
        # Heavy map loading has its own startup allowance. Once the worker is
        # ready, localization wait and computation share one fixed deadline;
        # transient loss of worker status or eventual dispatch cannot renew it.
        work_started = slot.get('prework_at')
        budget = (self.p['result_timeout_s'] if work_started is not None
                  else self.p.get('worker_startup_timeout_s', 60.))
        start = work_started if work_started is not None else slot['began']
        if time.monotonic()-start > budget:
            self.begin_cancel(slot, 'compute_route_timeout' if work_started is not None
                              else 'compute_worker_startup_timeout', compute=True)
            return
        if not slot['dispatched']:
            # Action-server discovery is not proof that the heavy private PCT
            # worker has loaded its map and created the goal subscriber. A
            # volatile one-shot goal must not be lost during that startup gap.
            request = slot['handle'].request
            publisher = self.goal_pub if request.goal_kind == '2d' else self.goal3d_pub
            if (not self.fresh_status('worker')
                    or self.worker.get('session_id') != self.p['session_id']
                    or (self.worker.get('active_goal') is not None
                        and self.worker.get('route_committed') is not True)
                    or self.worker.get('planning') is not False
                    or self.worker.get('pending_worker_start') is not False
                    or self.worker.get('retired_native_workers') != 0
                    or publisher.get_subscription_count() < 1):
                slot['wait_phase'], slot['wait_reason'] = 'waiting_worker', 'waiting_private_worker_ready'
                return
            if slot.get('prework_at') is None:
                slot['prework_at'] = time.monotonic()
            try:
                context = self.ready_identity()
            except ValueError as exc:
                slot['wait_phase'], slot['wait_reason'] = 'waiting_localization', str(exc)
                return
            message = deepcopy(request.goal)
            message.header.stamp = self.get_clock().now().to_msg()
            stamp = stamp_seconds(message.header.stamp)
            if stamp <= self.last_goal_stamp:
                return
            self.last_goal_stamp = slot['goal_stamp'] = stamp
            slot['context'], slot['dispatched'] = context, True
            slot['dispatched_at']=time.monotonic()
            if request.goal_kind == '2d':
                self.goal_pub.publish(message)
            else:
                point = PointStamped(header=message.header, point=message.pose.position)
                self.goal3d_pub.publish(point)
            return
        if not self.fresh_status('worker'):
            return
        status = self.worker
        if status.get('last_goal_stamp') != slot['goal_stamp']:
            return
        if status.get('state') in COMPUTE_FAILURE_STATES or status.get('recovery_hold'):
            self.begin_cancel(slot, 'compute_route_failed:'+str(status.get('reason', '')), compute=True)
            return
        goal = status.get('active_goal') or {}
        last = status.get('last_route') or {}
        context = slot['context']
        if (not status.get('route_committed') or goal.get('user_stamp') != slot['goal_stamp']
                or (goal.get('epoch'), goal.get('seed_id')) != context[:2]
                or goal.get('map_version_id') != context[2]):
            return
        candidate = self.candidates.get(last.get('path_stamp'))
        if candidate is None or candidate['source'].route_hash != last.get('route_hash'):
            return
        request = slot['handle'].request
        q = request.goal.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        source = candidate['source'].with_goal(has_goal_yaw=request.has_goal_yaw,
            goal_yaw=yaw, goal_yaw_tolerance_rad=request.goal_yaw_tolerance_rad)
        digest = source.route_hash
        task = slot['handle'].request.task_id
        route_id = task+':'+digest
        wire = encode_route(source, session_id=self.p['session_id'], task_id=task,
            route_id=route_id, epoch=context[0], seed_id=context[1], stamp=candidate['stamp'])
        path = wire.path
        self.snapshot = dict(task_id=task, route_id=route_id, context=context,
                             route=deepcopy(path), digest=digest, source_snapshot=source, wire=wire)
        self.finish(slot, ComputeRoute.Result(schema_version=2, success=True, reason='route_computed',
            route_id=route_id, localization_epoch=context[0], localization_seed_id=context[1],
            route=path, snapshot=wire), compute=True)

    def local_ready(self, context):
        return (self.fresh_status('scan') and self.scan.get('sensor_ready') is True
            and self.scan.get('native_map_context_ready') is True
            and not self.scan.get('native_map_context_fault')
            and (self.scan.get('localization_epoch'), self.scan.get('localization_seed_id')) == context[:2])

    def on_refresh(self, message):
        slot = self.follow
        if slot is None or slot['cancel'] or not slot['committed']:
            return
        try:
            value = json.loads(message.data)
            if (value.get('session_id') != self.p['session_id'] or value.get('motion_enabled') is not False
                    or not fresh(value.get('received_at_unix'), self.now_s(), .5)
                    or (value.get('localization_epoch'), value.get('localization_seed_id')) != slot['context'][:2]
                    or value.get('rejected_reference_stamp') != slot['reference_stamp']
                    or type(value.get('request_id')) is not int
                    or value['request_id'] <= slot['last_refresh_id']
                    or self.ready_identity() != slot['context']
                    or not self.local_ready(slot['context'])):
                return
            if slot['refresh_deadline'] is None:
                slot['refresh_deadline'] = time.monotonic()+12.
            if slot['refresh_count'] >= 20 or time.monotonic() > slot['refresh_deadline']:
                self.begin_cancel(slot, 'reference_handshake_episode_exhausted', compute=False)
                return
            slot['last_refresh_id'] = value['request_id']
            slot['refresh_count'] += 1
            self.publish_reference(slot)
        except (ValueError, TypeError, AttributeError):
            return

    def follow_tick(self):
        slot = self.follow
        if slot is None or self.cancellation_tick(slot, compute=False):
            return
        hard = self.hard_reason(slot['context'])
        if hard:
            self.begin_cancel(slot, hard, compute=False)
            return
        try:
            # This keeps the fixed-route workflow alive, not a motion lease.
            # Owner, tracker, safety and SDK retain instantaneous IMU checks.
            ready = self.ready_local_task_identity() == slot['context']
        except ValueError:
            ready = False
        local_ready = self.local_ready(slot['context'])
        if not slot['committed']:
            if ready and local_ready and self.route_reference_pub.get_subscription_count() > 0:
                self.publish_reference(slot)
        scan = self.scan
        same_delivery = False
        same_reference = (self.fresh_status('scan') and scan.get('active_reference') is True
            and scan.get('owner_reference_stamp') == slot['reference_stamp'])
        if uses_atomic_navigation(self.p):
            # Delivery is an exact identity acknowledgement, not a float-clock
            # comparison. Unix-epoch ns * 1e-9 and sec + ns * 1e-9 can differ
            # by one ULP even when they describe the very same ROS Time.
            same_delivery = (self.fresh_status('scan')
                and scan.get('owner_task_id') == slot['handle'].request.task_id
                and scan.get('owner_route_id') == self.snapshot['route_id']
                and scan.get('owner_route_hash') == self.snapshot['source_snapshot'].route_hash
                and type(scan.get('owner_delivery_sequence')) is int
                and scan['owner_delivery_sequence'] == slot.get('reference_delivery_sequence')
                and type(scan.get('owner_reference_stamp_ns')) is int
                and scan['owner_reference_stamp_ns'] == slot.get('reference_stamp_ns'))
            same_reference = same_delivery and scan.get('active_reference') is True
        if same_reference:
            # Bounded handshake attempts belong to an unaccepted delivery
            # episode, not the lifetime of a long navigation task.
            slot['refresh_count'], slot['refresh_deadline'] = 0, None
        endpoint = self.snapshot['route'].poses[-1].pose.position
        goal = self.snapshot['source_snapshot'].payload()
        yaw = self.body_yaw
        heading_ready = (not goal['has_goal_yaw'] or yaw is not None
            and abs(math.atan2(math.sin(yaw-goal['goal_yaw']), math.cos(yaw-goal['goal_yaw'])))
                <= goal['goal_yaw_tolerance_rad'])
        try: global_ready=self.ready_identity()==slot['context']
        except ValueError: global_ready=False
        reached = self.arrival.observe(body=self.body or (), endpoint=(endpoint.x, endpoint.y, endpoint.z),
            stamp=self.body_source, valid=global_ready and local_ready and same_reference and heading_ready,
            body_height=self.p['body_height'], xy_tolerance=self.p['goal_xy_tolerance_m'],
            z_tolerance=self.p['goal_z_tolerance_m'])
        if uses_atomic_navigation(self.p):
            permit=self.execution_permit; stop=self.stop_report
            # Terminal execution evidence is necessary, not a replacement for
            # current measured arrival. SCAN may already be retired here, so
            # do not depend on local_ready/reference/trajectory availability.
            position_ready=bool(global_ready and self.body and heading_ready
                and math.hypot(self.body[0]-endpoint.x,self.body[1]-endpoint.y)<=self.p['goal_xy_tolerance_m']
                and abs(self.body[2]-self.p['body_height']-endpoint.z)<=self.p['goal_z_tolerance_m'])
            stopped_goal_event=bool(permit and permit.phase=='terminal' and permit.reason=='goal_reached'
                and stop and stop.measured_stop_confirmed and stop.nonzero_blocked
                and same_execution_task(stop.version,permit.version) and stop.execution_id==permit.execution_id
                and stop.control_epoch==permit.control_epoch and stop.sdk_session==permit.sdk_session
                and stop.sdk_arm_generation==permit.sdk_arm_generation
                and self.body_source>=stamp_seconds(stop.source_stamp)
                and fresh(stamp_seconds(permit.source_stamp),self.now_s(),.4)
                and self.now_s()<=stamp_seconds(permit.valid_until)
                and fresh(stamp_seconds(stop.source_stamp),self.now_s(),.4)
                and 0<=time.monotonic()-self.permit_received<=.4 and 0<=time.monotonic()-self.stop_received<=.4)
            # Control completion is expressed in the committed odom anchor.
            # A subsequent map correction can disagree with the source-map
            # goal. Retire explicitly for review instead of waiting forever
            # or implicitly starting a new motion toward the corrected goal.
            if stopped_goal_event and global_ready and not position_ready:
                self.begin_cancel(slot,'post_stop_goal_mismatch',compute=False)
                return
            reached=bool(position_ready and stopped_goal_event)
        if reached:
            slot['measured_arrival_stamp'] = self.body_source
            self.begin_cancel(slot, 'goal_reached', compute=False)
            return
        pending_phase = ''
        if (same_delivery and scan.get('native_reference_received') is True
                and type(scan.get('pending_reference_generation')) is int
                and scan['pending_reference_generation'] > 0
                and scan.get('local_debug_generation') == scan['pending_reference_generation']):
            # Source node binds this durable diagnostic outcome to the exact
            # pending reference. It is not a renewable sensor/curve lease.
            pending_phase = str(scan.get('native_pending_phase') or '')[:120]
        stage = classify_follow(pose_ready=ready, map_ready=local_ready,
            committed=slot['committed'], subscriber_ready=self.route_reference_pub.get_subscription_count() > 0,
            same_reference=same_reference, scan=scan, pending_native_phase=pending_phase)
        slot['wait_phase'], slot['wait_reason'] = stage.phase, stage.reason
        problem = slot['recovery'].observe(stage, time.monotonic(),
            evidence_stamp=scan.get('received_at_unix', 0.) if self.fresh_status('scan') else 0.)
        if problem and (stage.terminal or not uses_atomic_navigation(self.p)):
            self.begin_cancel(slot, problem, compute=False)

    def tick(self):
        self.compute_tick()
        self.follow_tick()
        health = NavigationHealth(schema_version=2, stamp=self.get_clock().now().to_msg(), session_id=self.p['session_id'])
        if uses_atomic_navigation(self.p):
            # Identity is independent of the pose lease. Keep a soft-unusable
            # or hard-reset event identifiable so the owner actually receives
            # ready=False instead of dropping an empty-map health message.
            health.map_version_id=self.p['map_version_id']
            if self.atomic_inbox.latest is not None:
                health.localization_epoch,health.localization_seed_id=self.atomic_inbox.latest.identity[1:]
            if self.p.get('local_state_enabled') and self.local_inbox.latest is not None:
                health.localization_epoch,health.localization_seed_id=self.local_inbox.latest.identity[1:]
        health.hard_fault = bool(self.hard_reason())
        try:
            identity = self.identity()
            health.localization_epoch, health.localization_seed_id = identity[:2]
            health.map_version_id = identity[2]
            self.ready_identity()
            health.global_planning_ready = not health.hard_fault
            health.ready = health.global_planning_ready
            if self.atomic_inbox.latest is not None:
                health.evidence_source_stamp.sec,health.evidence_source_stamp.nanosec=divmod(self.atomic_inbox.latest.source_ns,10**9)
            health.reason = 'ready' if health.ready else self.hard_reason()
        except ValueError as exc:
            health.reason = self.hard_reason() or str(exc)
        try:
            local=self.ready_local_identity()
            health.local_control_ready=not health.hard_fault
            if self.p.get('local_state_enabled'):
                health.localization_epoch,health.localization_seed_id=local[:2]
                event=self.local_inbox.latest
                health.evidence_source_stamp.sec,health.evidence_source_stamp.nanosec=divmod(event.source_ns,10**9)
            elif self.atomic_inbox.latest is not None:
                event=self.atomic_inbox.latest
                health.evidence_source_stamp.sec,health.evidence_source_stamp.nanosec=divmod(event.source_ns,10**9)
        except ValueError:
            health.local_control_ready=False
        # ready retains legacy startup/global meaning. BT reads the capability
        # it needs for the current stage, never a renamed global status.
        self.health_pub.publish(health)
        if time.monotonic()-self.last_feedback < .2:
            return
        self.last_feedback = time.monotonic()
        if self.compute is not None:
            slot = self.compute
            slot['handle'].publish_feedback(ComputeRoute.Feedback(
                phase='canceling' if slot['cancel'] else (
                    'computing' if slot['dispatched'] else slot['wait_phase']),
                reason=slot['cancel']['reason'] if slot['cancel'] else (
                    str(self.worker.get('reason', '')) if slot['dispatched'] else slot['wait_reason'])))
        if self.follow is not None:
            slot = self.follow
            self.follow['handle'].publish_feedback(FollowRoute.Feedback(
                phase='canceling' if slot['cancel'] else slot['wait_phase'],
                reason=slot['cancel']['reason'] if slot['cancel'] else slot['wait_reason'],
                local_plan_id=int(self.scan.get('last_spline_id') or -1),
                progress_m=float(self.scan.get('local_debug_progress_arc_m') or 0.)))


def main(args=None):
    rclpy.init(args=args)
    node = NavigationBTAdapters()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.try_shutdown()
        except RCLError:
            if rclpy.ok():
                raise


if __name__ == '__main__':
    main()
