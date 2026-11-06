"""Live-only RViz initial-pose mailbox and state. Never sends robot commands."""
import argparse
import json
import math
import os
from pathlib import Path
import time
import uuid
from copy import deepcopy
import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy._rclpy_pybind11 import RCLError
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool, Empty, ColorRGBA
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray, InteractiveMarkerFeedback
from d1max_navigation_bt_interfaces.action import Navigate
from d1max_navigation_bt_interfaces.srv import ConfirmExecution, PrepareInitialPose
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from d1max_pct_planner.preview_markers import pose_marker
from d1max_pct_planner.selection_markers import selection_evidence_markers
from .live_ui_contract import (initial_pose_command, initial_pose_feedback,
                               checked_session_status, source_status_fresh, localization_preview_state,
                               floor_initial_body_z)
from .live_diagnostics import diagnostics, body_identity, body_visual_state
from .live_goal_editor import goal_position
from .live_goal_selection import LiveGoalSelection
from .live_view_reload import preview_reload_session
from .bt_presentation import awaiting_goal_owner, tree_presentation


BODY_AXIS_LENGTH_M = .9
BODY_AXIS_WIDTH_M = .02


def execution_menu_capable(session):
    """UI intent only; the BT and SDK still own all execution authorization."""
    if session.get('pipeline_contract') == 'single_floor_v3':
        if session.get('purpose') != 'execution':
            return False
        if session.get('transport_mode') == 'isolated_mock':
            return session.get('motion_control_enabled') is False
        return (session.get('transport_mode') == 'live'
                and session.get('motion_control_enabled') is True)
    return (session.get('mode') == 'LIVE_NAVIGATION'
            and session.get('motion_control_enabled') is True)


def body_outline_bounds(session):
    """Read the uninflated robot proxy once for display in the body frame."""
    try:
        profile = session.get('robot_profile_snapshot')
        if profile is None:
            profile = yaml.safe_load(Path(session['robot_profile']).read_text())
        geometry = profile['engineering']
        length, width, below, above = (geometry[key] for key in
            ('box_length_m', 'box_width_m', 'box_body_below_m', 'box_body_above_m'))
        if any(type(v) not in (int, float) or not math.isfinite(v) or v <= 0.
               for v in (length, width, below, above)):
            return None
        return ((-length/2., -width/2., -below), (length/2., width/2., above))
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError):
        return None


def body_axes_marker(body, lifetime_sec, outline=None):
    """One measured-pose marker holds RGB axes and an optional wireframe body."""
    marker = Marker()
    marker.header = body.header
    # Keep the old marker identity so RViz replaces the former arrow in-place.
    marker.ns, marker.id = 'measured_body', 0
    marker.type, marker.action = Marker.LINE_LIST, Marker.ADD
    marker.pose = body.pose.pose
    marker.scale.x = BODY_AXIS_WIDTH_M
    marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1.
    for endpoint, rgb in (((BODY_AXIS_LENGTH_M, 0., 0.), (1., 0., 0.)),
                          ((0., BODY_AXIS_LENGTH_M, 0.), (0., 1., 0.)),
                          ((0., 0., BODY_AXIS_LENGTH_M), (0., 0., 1.))):
        marker.points.extend((Point(), Point(x=endpoint[0], y=endpoint[1], z=endpoint[2])))
        marker.colors.extend(ColorRGBA(r=rgb[0], g=rgb[1], b=rgb[2], a=1.) for _ in range(2))
    if outline is not None:
        low, high = outline
        corners = tuple((x, y, z) for x in (low[0], high[0])
                        for y in (low[1], high[1]) for z in (low[2], high[2]))
        # Flip exactly one of the three corner bits: twelve unique edges,
        # no filled faces. Use the same pose, lifetime and ID as the axes.
        for index, start in enumerate(corners):
            for bit in (1, 2, 4):
                end = index ^ bit
                if index < end:
                    marker.points.extend(Point(x=p[0], y=p[1], z=p[2])
                                         for p in (start, corners[end]))
                    marker.colors.extend(ColorRGBA(r=1., g=1., b=1., a=1.) for _ in range(2))
    # The caller supplies only the unexpired part of the original source lease.
    marker.lifetime = Duration(seconds=max(1e-9, lifetime_sec)).to_msg()
    return marker


class LiveView(Node):
    def __init__(self, directory):
        super().__init__('d1max_live_view')
        self.directory = directory
        self.session = json.loads((directory / 'session.json').read_text())
        self.body_outline = body_outline_bounds(self.session)
        self.single_floor = self.session.get('pipeline_contract') == 'single_floor_v3'
        # Keep the existing display field for execution-menu compatibility.
        # A simulated execution intent is not a physical motion capability.
        self.motion_capable = execution_menu_capable(self.session)
        self.physical_motion_capable = (self.session.get('motion_control_enabled') is True
            and (not self.single_floor or self.session.get('transport_mode') == 'live'))
        self.typed_view = None
        self.committed_route_view = None
        if self.single_floor:
            from .pose_continuity_view import PoseContinuityView
            self.pose_continuity_view=PoseContinuityView(self.session['id'],self.session['version_id'])
            from .typed_view_state import TypedViewState
            from .committed_route_view import CommittedRouteView, session_route_provenance
            self.typed_view = TypedViewState(self.session['id'], self.session['version_id'])
            self.committed_route_view = CommittedRouteView(self.session['id'], self.session['version_id'],
                expected_provenance=session_route_provenance(self.session))
        self.frame = self.session['frame_id']
        self.last_state = {}
        self.state_at = 0.
        self.last_seed_id = None
        self.last_snapshot_at = 0.
        self.reason = '等待真实传感器' if self.motion_capable else '等待真实传感器；只显示，不发送速度'
        self.reason_scope = 'input'
        self.goal_submitted_at = 0.
        self.global_state, self.scan_state = {}, {}
        self.global_at = self.scan_at = 0.
        self.bt_state, self.bt_at = {}, 0.
        self.goal_pose = None
        self.goal_selection = None
        self.goal_selection_error = ''
        if self.single_floor:
            try:
                self.goal_selection = LiveGoalSelection.from_session(self.session)
            except (ValueError, OSError, KeyError) as error:
                # A map error is an editable UI diagnostic, not a reason to
                # kill localization or create an unvalidated navigation goal.
                self.goal_selection_error = str(error)
        self.goal_marker_signature = None
        self.goal_visual_dirty = False
        self.goal_has_yaw = False
        self.goal_request_sequence = 0
        self.goal_request_pending = False
        self.goal_request_active = False
        self.goal_submission_signature = None
        self.goal_request_task_id = None
        self.goal_request_deadline = 0.
        self.execute_binding = None
        self.execute_request_id = None
        self.execute_pending = False
        self.execute_sequence = 0
        self.execute_deadline = 0.
        self.initial_floor = self.session.get('current_floor', 'floor1')
        self.initial_ground_bridge = (self.goal_selection.bridge if self.goal_selection is not None else None)
        choices = getattr(self.goal_selection, 'floor_choices', [])
        self.initial_floor_choices = (tuple(choice['floor_id'] for choice in choices)
            if isinstance(choices, list) else ()) if self.single_floor else ('floor1', 'floor2')
        if self.initial_floor_choices and self.initial_floor not in self.initial_floor_choices:
            self.initial_floor = self.initial_floor_choices[0]
        self.goal_surfaces = None
        self.map_layer_metadata = {}
        self.pending_goal_surface = None
        self.body = None
        self.body_received = 0.
        self.body_context = None
        self.body_sample_context = None
        self.body_barrier = 0.
        self.retired_body_caption = False
        self.marker = self.create_publisher(Marker, '/d1max/live_planning/view_status', 1)
        self.body_marker = self.create_publisher(Marker, '/d1max/live_planning/body_marker', 10)
        self.diagnostics_pub = self.create_publisher(String, '/d1max/live_planning/diagnostics', 1)
        # The formal BT runtime owns both preview and execution. Its typed
        # contract is independent of the menu's physical motion capability.
        # Never revive the legacy always-true freeze publisher for a v3 preview.
        runtime_freeze = self.session.get('preview_freeze_owner') == 'supervisor_child'
        if runtime_freeze:
            preview_reload_session(self.session)
        self.freeze = (None if self.single_floor or self.motion_capable or runtime_freeze else
                       self.create_publisher(Bool, '/d1max/live_planning/execution_frozen', 10))
        self.cancel_pub = self.create_publisher(Empty, '/d1max/live_planning/cancel', 10)
        self.navigate_client = ActionClient(self, Navigate, '/d1max/live_planning/bt/navigate')
        self.execute_client = self.create_client(ConfirmExecution,
            '/d1max/live_planning/bt/confirm_execution')
        from .initial_pose_client import InitialPoseClient
        self.initial_pose_transaction = InitialPoseClient()
        self.initial_pose_future = None
        self.initial_pose_request = None
        self.initial_pose_rpc_at = 0.
        self.initial_pose_client = self.create_client(PrepareInitialPose,
            '/d1max/live_planning/bt/prepare_initial_pose')
        self.create_timer(.1, self.poll_initial_pose)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.goal_editor_pub = self.create_publisher(String, '/d1max/live_planning/goal_editor_status', latched)
        self.goal_support_pub = self.create_publisher(MarkerArray,
            '/d1max/live_planning/goal_support', latched)
        self.create_subscription(String, '/d1max/live_planning/edit_goal3d', self.edit_goal, 10)
        self.create_subscription(Empty, '/d1max/live_planning/activate_goal3d', self.activate_goal, 10)
        self.create_subscription(String, '/d1max/live_planning/map_layer_status', self.on_map_metadata, latched)
        self.create_subscription(PointCloud2, '/d1max/live_planning/traversable_surface', self.on_surfaces, latched)
        if self.single_floor:
            from d1max_planning_interfaces.msg import NavigationState
            from d1max_navigation_bt_interfaces.msg import InitialPoseOutcome, RouteReference, RouteSnapshot
            self.create_subscription(NavigationState, '/d1max/localization/navigation/state',
                                     self.on_atomic_state, qos_profile_sensor_data)
            self.create_subscription(String, '/d1max/localization/navigation/pose_continuity',
                                     self.on_pose_continuity, qos_profile_sensor_data)
            self.create_subscription(InitialPoseOutcome, '/d1max/localization/initial_pose/outcome',
                                     self.on_initial_pose_outcome, latched)
            if self.session.get('planning_scope') == 'global_route':
                self.create_subscription(RouteSnapshot, '/d1max/live_planning/bt/route_snapshot',
                                         self.on_route_snapshot, latched)
            else:
                self.create_subscription(RouteReference, '/d1max/live_planning/committed_route',
                                         self.on_committed_route, latched)
        else:
            self.create_subscription(String, '/d1max/localization/status', self.status, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/d1max/live_planning/initialpose', self.initial, 10)
        self.create_subscription(String, '/d1max/live_planning/global_status', self.on_global, 10)
        self.create_subscription(String, '/d1max/live_planning/scan_bridge_status', self.on_scan, 10)
        self.create_subscription(String, '/d1max/live_planning/bt/status', self.on_bt, 10)
        self.create_subscription(PointStamped, '/d1max/live_planning/place_goal3d', self.place_goal, 10)
        if not self.single_floor:
            self.create_subscription(Odometry, '/d1max/localization/odometry/global', self.on_body,
                                     qos_profile_sensor_data)
        self.markers = InteractiveMarkerServer(self, '/d1max_live_goal')
        self.menu = MenuHandler()
        if self.motion_capable:
            self.menu.insert('导航到此处', callback=lambda feedback:self.commit_goal(feedback,request_execution=True))
            self.menu.insert('仅规划预览', callback=self.commit_goal)
        else:
            self.menu.insert('规划到此处', callback=self.commit_goal)
        self.yaw_menu = self.menu.insert('指定终点朝向', callback=self.toggle_goal_yaw)
        self.menu.setCheckState(self.yaw_menu, MenuHandler.UNCHECKED)
        self.ground_menu = self.menu.insert('贴地拖动', callback=lambda _: self.set_goal_mode('ground'))
        self.free_menu = self.menu.insert('自由 XYZ', callback=lambda _: self.set_goal_mode('free'))
        self.menu.insert('贴到当前地面', callback=self.snap_goal_ground)
        self.menu.setCheckState(self.ground_menu, MenuHandler.CHECKED)
        self.menu.setCheckState(self.free_menu, MenuHandler.UNCHECKED)
        choices = getattr(self.goal_selection, 'region_choices', [])
        if isinstance(choices, list) and len(choices) > 1:
            floor_menu = self.menu.insert('目标区域')
            self.menu.insert('按高度选择', parent=floor_menu,
                             callback=lambda _: self.select_goal_floor(None))
            for choice in choices:
                self.menu.insert(choice['label'], parent=floor_menu,
                    callback=lambda _, floor=choice['floor_id']: self.select_goal_floor(floor))
        if self.motion_capable:
            self.menu.insert('执行已预览路线', callback=self.confirm_execution)
        self.menu.insert('取消任务（不是机器人急停）', callback=self.cancel)
        initial_menu = self.menu.insert('初始楼层')
        for floor in self.initial_floor_choices:
            self.menu.insert({'floor1': '一楼', 'floor2': '二楼'}.get(floor, floor), parent=initial_menu,
                             callback=lambda _, selected=floor: self.select_initial_floor(selected))
        self.create_timer(.2, self.tick)
        self.create_timer(.05, self.publish_body)
        self.create_timer(.05, self.refresh_goal_visual)

    def source_now_s(self):
        # Sensor/task headers use the ROS clock, including isolated bag time.
        # Receipt watchdogs remain monotonic; do not re-stamp observed data.
        clock = getattr(self, 'get_clock', None)
        stamp = getattr(clock().now(), 'nanoseconds', None) if clock else None
        return stamp * 1e-9 if isinstance(stamp, int) else time.time()

    def localization_ready(self):
        if getattr(self, 'single_floor', False):
            return self.typed_view.lease(now_ns=self.get_clock().now().nanoseconds,
                                         monotonic=time.monotonic()) > 0.
        return localization_preview_state(self.last_state,
            session_id=self.session['id'], now=LiveView.source_now_s(self))[0]

    def on_atomic_state(self, message):
        try:
            if not self.typed_view.observe(message, now_ns=self.get_clock().now().nanoseconds,
                                            monotonic=time.monotonic()):
                return
        except ValueError:
            return
        self.body = deepcopy(message.global_odometry)
        self.body_received = self.typed_view.received
        self.body_sample_context = self.typed_view.state.identity
        self.pose_continuity_view.bind_real_pose()

    def on_pose_continuity(self, message):
        try:
            state=self.typed_view.state
            self.pose_continuity_view.observe(json.loads(message.data),
                identity=state.identity if state else None,
                now_ns=self.get_clock().now().nanoseconds,monotonic=time.monotonic())
        except (ValueError,TypeError,KeyError):return

    def on_committed_route(self, message):
        # Read-only copy of the same audited envelope displayed in RViz. This
        # is a result/history cache, never another task or reference publisher.
        self.committed_route_view.receive(message)

    def on_route_snapshot(self, message):
        # Compute-only results are never promoted into execution references.
        if self.session.get('planning_scope') == 'global_route':
            self.committed_route_view.receive_snapshot(message)

    def on_map_metadata(self, message):
        try:
            value = json.loads(message.data)
            if (value.get('session_id') == self.session['id'] and value.get('frame_id') == self.frame
                    and value.get('ground_display_only') is True and value.get('motion_enabled') is False):
                self.map_layer_metadata = value
                if self.pending_goal_surface is not None:
                    pending, self.pending_goal_surface = self.pending_goal_surface, None
                    self.on_surfaces(pending)
        except (ValueError, AttributeError):
            pass

    def on_surfaces(self, message):
        if (message.header.frame_id != self.frame
                or message.is_bigendian or message.height != 1 or message.point_step != 16
                or not 0 < message.width <= 1000000 or len(message.data) != message.width*16):
            return
        fields = {f.name: (f.offset, f.datatype, f.count) for f in message.fields}
        if any(fields.get(name) != (i*4, 7, 1) for i, name in enumerate(('x', 'y', 'z'))):
            return
        if not self.map_layer_metadata:
            self.pending_goal_surface = message  # One bounded latched packet, not a growing queue.
            return
        xyz = np.ndarray((message.width, 3), dtype='<f4', buffer=message.data, strides=(16, 4))
        if np.isfinite(xyz).all():
            self.goal_surfaces = xyz.copy()

    def activate_goal(self, _):
        existing = None if self.goal_pose is None else (
            self.goal_pose.position.x, self.goal_pose.position.y, self.goal_pose.position.z)
        body = None
        if (self.body is not None and self.body.header.frame_id == self.frame
                and 0 <= time.monotonic()-self.body_received < .5
                and LiveView.localization_ready(self)):
            p, q = self.body.pose.pose.position, self.body.pose.pose.orientation
            body = (p.x, p.y, p.z, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
        # v3 sessions have no legacy initial_body_z. A dict.get() default is
        # evaluated eagerly, even when a current target or map seed exists.
        # Drafts use an existing map-ground target, a verified map-body sample
        # (converted to ground once), or this same map's explicit ground seed.
        metadata = self.map_layer_metadata
        fallback = (metadata.get('initial_goal_xyz') if (
            metadata.get('session_id') == self.session['id']
            and metadata.get('frame_id') == self.frame
            and metadata.get('ground_display_only') is True
            and metadata.get('motion_enabled') is False) else None)
        if existing is None and body is None and fallback is None:
            self.reason = '等待定位或当前地图的可通行地面目标'
            self.reason_scope = 'input'
            return
        try:
            point = goal_position(existing=existing, body=body, surfaces=self.goal_surfaces,
                                  fallback=fallback, body_height=self.session.get('body_height', .55))
        except (ValueError, TypeError, IndexError):
            self.reason = '3D 目标草稿坐标无效'
            self.reason_scope = 'input'
            return
        message = PointStamped()
        message.header.frame_id = self.frame
        message.header.stamp = self.get_clock().now().to_msg()
        message.point.x, message.point.y, message.point.z = point
        self.place_goal(message)

    def publish_goal_editor(self):
        if self.goal_pose is None:
            return
        p = self.goal_pose.position
        evidence = LiveView.goal_evidence(self)
        selection = getattr(self, 'goal_selection', None)
        self.goal_editor_pub.publish(String(data=json.dumps({
            'schema': 1, 'mode': 'LIVE_GOAL_EDITOR_NO_MOTION', 'session_id': self.session['id'],
            'received_at_unix': time.time(), 'frame_id': self.frame, 'motion_enabled': False,
            'selection_mode': selection.mode if selection else 'free',
            'goal_xyz': [p.x, p.y, p.z], 'goal_validation': evidence,
            'floor_choices': selection.floor_choices if selection else [],
            'region_choices': selection.region_choices if selection else [],
            'has_goal_yaw': self.goal_has_yaw,
            # The BT can wait for localization. Geometry validity belongs to
            # this draft and must not flicker with a localization heartbeat.
            'can_submit': evidence['valid']}, ensure_ascii=False, allow_nan=False)))

    def goal_evidence(self):
        selection = getattr(self, 'goal_selection', None)
        if selection is not None:
            return selection.feedback()
        if getattr(self, 'single_floor', False):
            return dict(valid=False, reason='目标地图未就绪', reason_code='selection_map_unavailable',
                        detail=getattr(self, 'goal_selection_error', ''))
        # Preserve the legacy editor only in its historical entry; the formal
        # runtime never uses this compatibility branch to bypass map evidence.
        return dict(valid=True, reason='legacy_free_xyz')

    def on_body(self, message):
        if (message.header.frame_id != self.frame or message.child_frame_id != 'd1max_loc_base_link'):
            return
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        p, q = message.pose.pose.position, message.pose.pose.orientation
        if (not -.1 <= LiveView.source_now_s(self)-stamp <= .5 or stamp < self.body_barrier or
                not all(math.isfinite(x) for x in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)) or
                max(abs(p.x), abs(p.y), abs(p.z)) > 10000 or
                abs(math.hypot(q.x, q.y, q.z, q.w)-1.) > .01):
            return
        previous = self.body.header.stamp if self.body else None
        if previous and stamp <= previous.sec + previous.nanosec * 1e-9:
            return
        identity = body_identity(self.last_state, session_id=self.session['id'], now=LiveView.source_now_s(self))
        if identity is None or not 0 <= time.monotonic()-self.state_at < .6:
            return
        self.body, self.body_received = message, time.monotonic()
        self.body_sample_context = identity

    def publish_body(self):
        now, wall = time.monotonic(), LiveView.source_now_s(self)
        marker = Marker()
        marker.header.frame_id = self.frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns, marker.id = 'measured_body', 0
        stamp = (self.body.header.stamp.sec + self.body.header.stamp.nanosec * 1e-9) if self.body else 0.
        display_body=self.body
        if getattr(self, 'single_floor', False):
            ttl = self.typed_view.lease(now_ns=self.get_clock().now().nanoseconds, monotonic=now)
            visual = dict(visible=ttl > 0., lifetime_sec=ttl)
            continuity=getattr(self,'pose_continuity_view',None)
            if ttl<=0 and continuity is not None:
                state=self.typed_view.state
                lease=continuity.lease(identity=state.identity if state else None,
                    now_ns=self.get_clock().now().nanoseconds,monotonic=now)
                if lease>0:
                    # This body is local to rendering. Do not replace self.body,
                    # typed state, localization_ready, goals or any permission.
                    value=continuity.sample;display_body=Odometry()
                    display_body.header.frame_id=value['frame_id']
                    display_body.header.stamp.sec,display_body.header.stamp.nanosec=divmod(value['estimate_stamp_ns'],10**9)
                    display_body.child_frame_id=value['body_frame']
                    p,q=display_body.pose.pose.position,display_body.pose.pose.orientation
                    p.x,p.y,p.z=value['position'];q.x,q.y,q.z,q.w=value['orientation']
                    visual=dict(visible=True,lifetime_sec=lease)
        else:
            visual = body_visual_state(self.last_state, session_id=self.session['id'], now=wall,
                body_stamp=stamp, body_receipt_age=now-self.body_received,
                state_receipt_age=now-self.state_at, body_context=self.body_sample_context,
                body_barrier=self.body_barrier)
        marker.action = Marker.ADD if display_body is not None and visual['visible'] else Marker.DELETE
        if marker.action == Marker.ADD:
            marker = body_axes_marker(display_body, visual['lifetime_sec'],
                                      getattr(self, 'body_outline', None))
        if not self.retired_body_caption:
            # Retire the old caption before the first axes update, never after
            # every ADD: a depth-one RViz subscriber can drop the useful marker
            # when a second ID immediately replaces it in its receive queue.
            label = Marker()
            label.header = marker.header
            label.ns, label.id = 'last_known_body', 1
            label.action = Marker.DELETE
            self.body_marker.publish(label)
            self.retired_body_caption = True
        self.body_marker.publish(marker)

    def on_global(self, message):
        value = checked_session_status(message.data, session_id=self.session['id'],
            stamp_field='received_at_unix', now=LiveView.source_now_s(self), timeout=1.,
            previous_stamp=self.global_state.get('received_at_unix', 0.), maximum_bytes=65536)
        if value is not None:
            self.global_state, self.global_at = value, time.monotonic()

    def on_scan(self, message):
        value = checked_session_status(message.data, session_id=self.session['id'],
            stamp_field='received_at_unix', now=LiveView.source_now_s(self), timeout=1.,
            previous_stamp=self.scan_state.get('received_at_unix', 0.), maximum_bytes=65536)
        if value is not None:
            self.scan_state, self.scan_at = value, time.monotonic()

    def on_bt(self, message):
        value = checked_session_status(message.data, session_id=self.session['id'],
            stamp_field='received_at_unix', now=LiveView.source_now_s(self), timeout=1.,
            previous_stamp=self.bt_state.get('received_at_unix', 0.), maximum_bytes=65536)
        if value is not None:
            self.bt_state, self.bt_at = value, time.monotonic()

    def status(self, message):
        state = checked_session_status(message.data, session_id=self.session['id'],
            stamp_field='wall_time', now=LiveView.source_now_s(self), timeout=.6,
            previous_stamp=self.last_state.get('wall_time', 0.))
        if state is not None:
            context = (self.session['id'], state.get('local_epoch'), state.get('confirmed_seed_ns'),
                       state.get('active_seed_ns'))
            navigation = state.get('navigation')
            navigation = navigation if isinstance(navigation, dict) else {}
            hard_invalid = bool(state.get('local_fault') or navigation.get('fault'))
            if context != self.body_context or hard_invalid:
                continuity=getattr(self,'pose_continuity_view',None)
                if continuity is not None:
                    continuity.withdraw()
                self.body = None
                self.body_received = 0.
                self.body_sample_context = None
                self.body_barrier = state['wall_time']
                self.body_context = context
            self.last_state, self.state_at = state, time.monotonic()

    def initial(self, message):
        if not getattr(self, 'single_floor', False):
            self.reason_scope = 'input'
        try:
            stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            p, q = message.pose.pose.position, message.pose.pose.orientation
            if self.initial_ground_bridge is None:
                if getattr(self, 'single_floor', False):
                    # Reuse the same session-bound ground evidence as the goal
                    # editor. A failed map binding cannot fall back to an
                    # unpinned bridge or silently guess the first floor.
                    raise ValueError('初值地图未就绪：'+self.goal_selection_error)
                else:
                    from .pointcloud_helpers.ground_path_bridge import GroundPathBridge
                    self.initial_ground_bridge = GroundPathBridge.from_artifacts(self.session['planning_manifest'])
            body_z = floor_initial_body_z(self.initial_ground_bridge, self.initial_floor,
                                         (p.x, p.y), self.session['body_height'])
            # A seed only, not a terrain constraint. PCD registration estimates full SE(3).
            if getattr(self, 'single_floor', False):
                from .initial_pose_client import TRANSACTION
                if self.session.get('initial_pose_transaction') != TRANSACTION:
                    raise ValueError('会话初值接口版本不匹配，请使用一致的新运行包')
                from .typed_view_state import seed_command
                command = seed_command(message, now=LiveView.source_now_s(self), session_id=self.session['id'],
                                       body_z=body_z, command_id=uuid.uuid4().hex)
                request = PrepareInitialPose.Request()
                request.schema_version = 1
                request.operation = request.PREPARE
                request.session_id, request.request_id = self.session['id'], command['id']
                request.source_map_sha256 = self.session['input_hashes'][str(Path(self.session['map_pcd']).resolve())]
                request.body_pose = deepcopy(message)
                request.body_pose.pose.pose.position.z = body_z  # Ground -> body exactly once.
                self.initial_pose_transaction.begin(request, command, time.monotonic())
                # Latest valid input replaces one pending UI intent. Removing
                # the RPC registration is not a BT/SDK cancellation; the new
                # PREPARE performs the unchanged owner admission/retirement.
                LiveView._forget_initial_pose_rpc(self, self.initial_pose_future)
                self.get_logger().info(self.initial_pose_transaction.reason)
                return
            else:
                command = initial_pose_command(frame=message.header.frame_id, stamp=stamp,
                    now=LiveView.source_now_s(self), xy=(p.x, p.y), quaternion=(q.x, q.y, q.z, q.w),
                    body_z=body_z, state=self.last_state,
                    session_id=self.session['id'], state_age=time.monotonic()-self.state_at,
                    command_id=uuid.uuid4().hex)
            self.cancel_pub.publish(Empty())
            path = self.directory / 'initial_pose.tmp'
            path.write_text(json.dumps(command))
            os.replace(path, self.directory / 'initial_pose.json')
            self.last_seed_id = command['id']
            self.reason = '已提交机身初值；等待连续匹配确认'
        except (ValueError, OSError, KeyError) as error:
            if getattr(self, 'single_floor', False):
                self.initial_pose_transaction.reject_input(str(error), self.session['id'])
                self.get_logger().info(str(error))
                return
            self.reason = str(error)
        self.get_logger().info(self.reason)

    def _forget_initial_pose_rpc(self, future):
        if future is None or self.initial_pose_future is not future:
            return
        self.initial_pose_future = None
        self.initial_pose_request = None
        try:
            self.initial_pose_client.remove_pending_request(future)
        except (RuntimeError, RCLError, KeyError) as error:
            # The current binding is already detached. A late transport error
            # must not clear or fail the replacement's state.
            self.get_logger().warning('旧初值回执登记已弃用：' + str(error))

    def on_initial_pose_outcome(self, message):
        self.initial_pose_transaction.observe_outcome(message,
            map_version_id=self.session['version_id'], now_ns=self.get_clock().now().nanoseconds)

    def poll_initial_pose(self):
        transaction = self.initial_pose_transaction
        future, sent = self.initial_pose_future, self.initial_pose_request
        if future is not None:
            if not transaction.matches(sent):
                LiveView._forget_initial_pose_rpc(self, future)
            elif not future.done():
                if time.monotonic()-self.initial_pose_rpc_at <= 2.:
                    return
                transaction.fail('初值准入回执超时，未写入新初值；可重新给初值',
                                 request=sent, phase='timeout')
                LiveView._forget_initial_pose_rpc(self, future)
            else:
                try:
                    command = transaction.receive(sent, future.result(), time.monotonic())
                    if command is not None and transaction.owns(sent):
                        # New command issue time, not the original pose/sensor
                        # time. The immutable intent stamp travels separately.
                        command['created_at'] = time.time()
                        path = self.directory / 'initial_pose.tmp'
                        path.write_text(json.dumps(command))
                        os.replace(path, self.directory / 'initial_pose.json')
                        self.last_seed_id = command['id']
                        transaction.mark_submitted(sent)
                except (ValueError, OSError, RuntimeError, RCLError) as error:
                    transaction.fail('初值提交结果不确定，需核对：'+str(error),
                                     request=sent, phase='uncertain')
                LiveView._forget_initial_pose_rpc(self, future)
        request = transaction.request(time.monotonic())
        if request is None:
            # request() also expires the transaction. Its independent feedback
            # is rendered below, not hidden behind a stale generic goal notice.
            return
        if not self.initial_pose_client.service_is_ready():
            transaction.waiting_service()
            return
        try:
            self.initial_pose_request = request
            self.initial_pose_future = self.initial_pose_client.call_async(request)
            self.initial_pose_rpc_at = time.monotonic()
        except (RuntimeError, RCLError) as error:
            transaction.fail('初值准入发送失败：'+str(error), request=request)

    def select_initial_floor(self, floor_id):
        if floor_id not in getattr(self, 'initial_floor_choices', ()):
            return
        self.initial_floor = floor_id
        self.reason_scope = 'input'
        self.reason = ('初始楼层：一楼' if floor_id == 'floor1' else '初始楼层：二楼') + ' · 请用 2D Pose Estimate 给初值'

    def place_goal(self, message):
        self.reason_scope = 'input'
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        position = (message.point.x, message.point.y, message.point.z)
        if (message.header.frame_id != self.frame or not -.1 <= LiveView.source_now_s(self)-stamp <= 2
                or not all(math.isfinite(x) and abs(x) < 10000 for x in position)):
            self.reason = '3D 目标的坐标或时间无效'
            return
        selection = getattr(self, 'goal_selection', None)
        if selection is not None:
            position = selection.edit(position)['xyz']
        from geometry_msgs.msg import Pose
        previous = self.goal_pose
        self.goal_pose = Pose()
        self.goal_pose.position.x, self.goal_pose.position.y, self.goal_pose.position.z = map(float, position)
        # Reactivation focuses an existing draft, including its chosen heading.
        if previous is not None:
            self.goal_pose.orientation = deepcopy(previous.orientation)
        else:
            self.goal_pose.orientation.w = 1.
        self.goal_visual_dirty = True
        LiveView.refresh_goal_visual(self)
        self.reason = '拖动目标；右键规划'
        self.publish_goal_editor()

    def on_feedback(self, feedback):
        if feedback.header.frame_id != self.frame:
            return
        p = feedback.pose.position
        if not all(math.isfinite(x) and abs(x) < 10000 for x in (p.x, p.y, p.z)):
            return
        if feedback.event_type in (InteractiveMarkerFeedback.POSE_UPDATE, InteractiveMarkerFeedback.MOUSE_UP):
            q = feedback.pose.orientation
            if not all(math.isfinite(v) for v in (q.x, q.y, q.z, q.w)) or math.hypot(q.x,q.y,q.z,q.w) < 1e-9:
                return
            self.goal_pose = deepcopy(feedback.pose)
            selection = getattr(self, 'goal_selection', None)
            if selection is not None:
                point = selection.edit((p.x, p.y, p.z))['xyz']
                self.goal_pose.position.x, self.goal_pose.position.y, self.goal_pose.position.z = map(float, point)
            self.goal_visual_dirty = True
            # Coalesce rendering at 20 Hz; queries are local immutable-grid
            # reads. No planner calls or marker reinsertion on every mouse event.
            if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
                LiveView.refresh_goal_visual(self)
                self.publish_goal_editor()

    def refresh_goal_visual(self):
        if not getattr(self, 'goal_visual_dirty', False) or self.goal_pose is None:
            return
        self.goal_visual_dirty = False
        evidence = LiveView.goal_evidence(self)
        selection = getattr(self, 'goal_selection', None)
        ground = selection is not None and selection.mode == 'ground'
        p, q = self.goal_pose.position, self.goal_pose.orientation
        signature = (ground, evidence['valid'])
        if signature != self.goal_marker_signature:
            marker = pose_marker('goal', (p.x,p.y,p.z), (q.x,q.y,q.z,q.w), self.frame, 1.5,
                                 valid=evidence['valid'], ground_follow=ground)
            marker.description = '目标 · 右键规划' if evidence['valid'] else '目标 · 调整到地面'
            self.markers.insert(marker, feedback_callback=self.on_feedback)
            self.menu.apply(self.markers, 'goal')
            self.goal_marker_signature = signature
        else:
            self.markers.setPose('goal', self.goal_pose)
        self.markers.applyChanges()
        from std_msgs.msg import Header
        header = Header(frame_id=self.frame, stamp=self.get_clock().now().to_msg())
        self.goal_support_pub.publish(MarkerArray(markers=selection_evidence_markers(
            'goal', (p.x,p.y,p.z), evidence, header, 0)))

    def set_goal_mode(self, mode):
        selection = getattr(self, 'goal_selection', None)
        if selection is None:
            return
        selection.set_mode(mode)
        if self.goal_pose is not None:
            p = self.goal_pose.position
            point = selection.edit((p.x,p.y,p.z))['xyz']
            self.goal_pose.position.x, self.goal_pose.position.y, self.goal_pose.position.z = map(float, point)
        self.menu.setCheckState(self.ground_menu, MenuHandler.CHECKED if mode == 'ground' else MenuHandler.UNCHECKED)
        self.menu.setCheckState(self.free_menu, MenuHandler.CHECKED if mode == 'free' else MenuHandler.UNCHECKED)
        self.menu.reApply(self.markers)
        self.goal_visual_dirty = True
        LiveView.refresh_goal_visual(self)
        self.publish_goal_editor()

    def snap_goal_ground(self, _=None):
        selection = getattr(self, 'goal_selection', None)
        if selection is None or self.goal_pose is None:
            return
        try:
            point = selection.snap_ground()['xyz']
            self.goal_pose.position.x, self.goal_pose.position.y, self.goal_pose.position.z = map(float, point)
        except ValueError:
            self.reason = '这里没有可用地面；请调整 XY'
        self.goal_visual_dirty = True
        LiveView.refresh_goal_visual(self)
        self.publish_goal_editor()

    def select_goal_floor(self, floor_id):
        selection = getattr(self, 'goal_selection', None)
        if selection is None or self.goal_pose is None:
            return
        try:
            point = selection.set_floor(floor_id)['xyz']
            self.goal_pose.position.x, self.goal_pose.position.y, self.goal_pose.position.z = map(float,point)
        except ValueError:
            return
        self.goal_visual_dirty = True
        LiveView.refresh_goal_visual(self)
        self.publish_goal_editor()

    def edit_goal(self, message):
        """Session-bound draft edits only. Never publish a navigation intent."""
        if len(message.data) > 4096 or self.goal_pose is None:
            return
        try:
            edit = json.loads(message.data)
            if (edit.get('schema') != 1 or edit.get('session_id') != self.session['id']
                    or edit.get('frame_id') != self.frame):
                return
            selection = getattr(self, 'goal_selection', None)
            if selection is None:
                return
            mode = edit.get('selection_mode', selection.mode)
            point = edit.get('goal_xyz')
            # Validate the whole command before mutating either mode or pose.
            if mode not in ('ground', 'free') or (point is not None and (
                    not isinstance(point, list) or len(point) != 3
                    or any(isinstance(v, bool) or not isinstance(v, (int,float))
                           or not math.isfinite(v) or abs(v) >= 10000 for v in point))):
                return
            selection.set_mode(mode)
            if point is not None:
                xyz = selection.edit(point)['xyz']
                self.goal_pose.position.x, self.goal_pose.position.y, self.goal_pose.position.z = map(float,xyz)
            if edit.get('operation') == 'snap_ground':
                self.snap_goal_ground()
            self.set_goal_mode(mode)
        except (ValueError, TypeError, AttributeError):
            return

    def commit_goal(self, feedback, *, request_execution=False):
        if self.goal_pose is None:
            return
        evidence = LiveView.goal_evidence(self)
        if not evidence['valid']:
            self.reason = '目标无效：'+evidence.get('reason', '请调整到可通行地面')
            self.reason_scope = 'input'
            self.publish_goal_editor()
            return
        if request_execution and not self.motion_capable:
            self.reason='当前入口仅提供规划预览'
            return
        # The task owner waits for localization. A UI callback between valid
        # pose updates must not drop an otherwise explicit navigation goal.
        if not self.navigate_client.server_is_ready():
            self.reason = '导航任务服务未就绪'
            return
        goal = Navigate.Goal()
        goal.schema_version, goal.goal_kind = 2, '3d'
        goal.goal.header.frame_id = self.frame
        goal.goal.header.stamp = self.get_clock().now().to_msg()
        goal.goal.pose = deepcopy(self.goal_pose)
        goal.has_goal_yaw, goal.goal_yaw_tolerance_rad = self.goal_has_yaw, math.radians(10.)
        goal.request_execution=bool(request_execution)
        p, q = goal.goal.pose.position, goal.goal.pose.orientation
        signature = (self.frame, p.x, p.y, p.z, q.x, q.y, q.z, q.w,
                     goal.has_goal_yaw, goal.goal_yaw_tolerance_rad, goal.request_execution)
        # Every explicit user submission is a new intent, including the same
        # XYZ after a failure or while an old result waits for retirement.
        # Action UUID/source admission (not equal geometry) handles replay.
        # A new intent reaches the single BT owner immediately even while the
        # previous goal-acceptance RPC is pending; marker dragging alone does
        # not call this method or cancel the current task.
        # Do not send an unscoped Empty cancel here: it could arrive after the
        # replacement and cancel the new task instead of the old one.
        self.goal_request_sequence += 1
        sequence = self.goal_request_sequence
        self.goal_submission_signature = signature
        self.goal_request_task_id = None
        self.goal_request_pending = True
        self.goal_request_active = True
        self.goal_request_deadline = time.monotonic()+5.
        # A route confirmation belongs to the old intent, not the editable
        # marker. Retire its UI wait now; a late ACK cannot rename this target.
        self.execute_sequence = getattr(self, 'execute_sequence', 0)+1
        self.execute_pending = False
        self.execute_binding = self.execute_request_id = None
        self.reason = '导航目标已提交' if request_execution else '目标已提交，等待规划'
        self.reason_scope = 'planning'
        self.goal_submitted_at = LiveView.source_now_s(self)
        def feedback(message):
            if sequence != self.goal_request_sequence:
                return
            task_id = getattr(getattr(message, 'feedback', None), 'task_id', None)
            if isinstance(task_id, str) and 0 < len(task_id) <= 128:
                # UUID-scoped Action feedback names this exact BT task. Wall
                # time alone cannot distinguish a late A route from new C.
                self.goal_request_task_id = task_id
        try:
            future = self.navigate_client.send_goal_async(goal, feedback_callback=feedback)
        except Exception as error:
            self.goal_request_pending = False
            self.goal_request_active = False
            self.reason = '目标提交失败：'+type(error).__name__
            return
        def accepted(future):
            try:
                handle = future.result()
                if sequence != self.goal_request_sequence:
                    if handle.accepted:
                        handle.cancel_goal_async()
                    return
                self.goal_request_pending = False
                self.goal_request_active = bool(handle.accepted)
                self.reason = (('导航目标已接收' if request_execution else '目标已接收，等待路线预览')
                               if handle.accepted else '目标被任务管理器拒绝')
                if handle.accepted:
                    def completed(_):
                        if sequence == self.goal_request_sequence:
                            self.goal_request_active = False
                    handle.get_result_async().add_done_callback(completed)
            except Exception as error:
                if sequence == self.goal_request_sequence:
                    self.goal_request_pending = False
                    self.goal_request_active = False
                    self.reason = '目标提交失败：'+type(error).__name__
        future.add_done_callback(accepted)

    def toggle_goal_yaw(self, _):
        self.goal_has_yaw = not self.goal_has_yaw
        self.menu.setCheckState(self.yaw_menu,
            MenuHandler.CHECKED if self.goal_has_yaw else MenuHandler.UNCHECKED)
        self.menu.reApply(self.markers)
        self.markers.applyChanges()
        self.publish_goal_editor()

    def confirm_execution(self, _):
        if not self.motion_capable:
            self.reason = '当前入口仅提供规划预览'
            return
        state = self.bt_state
        binding = tuple(state.get(key, '') for key in ('task_id', 'route_id', 'route_hash'))
        if (self.goal_request_pending or (self.goal_request_sequence > 0
                and binding[0] != getattr(self, 'goal_request_task_id', None))):
            self.reason = '等待当前目标的路线预览'
            return
        if (state.get('schema') != 2 or state.get('session_id') != self.session['id']
                or not 0 <= time.monotonic()-self.bt_at <= .6 or not all(binding)):
            self.reason = '没有可确认的当前路线预览'
            return
        if self.execute_pending or not self.execute_client.service_is_ready():
            self.reason = '等待执行确认服务'
            return
        if binding != self.execute_binding:
            self.execute_binding, self.execute_request_id = binding, uuid.uuid4().hex
        request = ConfirmExecution.Request()
        request.schema_version, request.session_id = 2, self.session['id']
        request.task_id, request.route_id, request.route_hash = binding
        request.request_id = self.execute_request_id
        self.execute_pending = True
        self.execute_sequence = getattr(self, 'execute_sequence', 0)+1
        operation = self.execute_sequence
        self.execute_deadline = time.monotonic()+5.
        sequence = self.goal_request_sequence
        def complete(future):
            if operation != self.execute_sequence:
                return
            self.execute_pending = False
            current = tuple(self.bt_state.get(key, '') for key in ('task_id', 'route_id', 'route_hash'))
            # A response only belongs to the route the user actually confirmed.
            # A later cancel/new goal may already have revoked that task.
            if sequence != self.goal_request_sequence or current != binding:
                return
            try:
                result = future.result()
                # Even a transport-successful call is not an execution permit.
                self.reason = ('已授权当前路线' if result.schema_version == 2
                    and result.accepted and result.execution_authorized
                    else '未授权：'+result.reason)
            except Exception as error:
                self.reason = '执行确认结果未知：'+type(error).__name__
        try:
            self.execute_client.call_async(request).add_done_callback(complete)
        except Exception as error:
            self.execute_pending = False
            self.reason = '执行确认结果未知：'+type(error).__name__

    def cancel(self, _):
        self.goal_request_sequence += 1
        self.goal_request_pending = False
        self.goal_request_active = False
        self.goal_submission_signature = self.goal_request_task_id = None
        self.execute_sequence = getattr(self, 'execute_sequence', 0)+1
        self.execute_pending = False
        self.cancel_pub.publish(Empty())
        self.reason = '已取消任务并撤销导航运动许可' if self.motion_capable else '已取消规划预览；本入口从不发送运动'
        self.reason_scope = 'input'

    def expire_requests(self, now):
        """Bound only UI waits. Timeout is unknown, never an implicit permit."""
        if self.goal_request_pending and now >= self.goal_request_deadline:
            self.goal_request_sequence += 1
            self.goal_request_pending = False
            self.goal_request_active = False
            self.goal_submission_signature = self.goal_request_task_id = None
            self.reason = '目标接收超时；迟到的接收结果将取消'
        if self.execute_pending and now >= self.execute_deadline:
            self.execute_sequence += 1
            self.execute_pending = False
            # Keep route binding/request_id for a user's explicit idempotent
            # retry. Do not invent a second authorization after an uncertain ACK.
            self.reason = '执行确认超时，授权状态未知；请查看当前任务状态'

    def tick(self):
        # Freeze the native spline execution clock, NOT the robot. SCAN can
        # still replan from current measured odometry; there is no tracker.
        if self.freeze is not None:
            self.freeze.publish(Bool(data=True))
        now, wall = time.monotonic(), LiveView.source_now_s(self)
        self.expire_requests(now)
        fresh = (0 <= now-self.state_at < .6 and source_status_fresh(
            self.last_state, stamp_field='wall_time', now=wall, timeout=.6))
        s = self.last_state if fresh else {}
        m = Marker()
        m.header.frame_id = self.frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'live_state'
        m.id = 0
        # World-space captions obscure geometry and duplicate the diagnostics
        # panel. Explicitly clear old RViz state while retaining status outputs.
        m.action = Marker.DELETE
        global_state = self.global_state if (0 <= now-self.global_at < 1 and source_status_fresh(
            self.global_state, stamp_field='received_at_unix', now=wall, timeout=1.)) else {}
        scan_state = self.scan_state if (0 <= now-self.scan_at < 1 and source_status_fresh(
            self.scan_state, stamp_field='received_at_unix', now=wall, timeout=1.)) else {}
        feedback = initial_pose_feedback(s, self.last_seed_id) if fresh else None
        bt_state = getattr(self, 'bt_state', {})
        bt_state = bt_state if (0 <= now-getattr(self, 'bt_at', 0.) < 1. and source_status_fresh(
            bt_state, stamp_field='received_at_unix', now=wall, timeout=1.)) else {}
        waiting_goal_owner = awaiting_goal_owner(bt_state,
            pending=getattr(self, 'goal_request_pending', False),
            active=getattr(self, 'goal_request_active', False),
            task_id=getattr(self, 'goal_request_task_id', None))
        typed = None
        initial_feedback = None
        if getattr(self, 'single_floor', False):
            typed = self.typed_view.display_context(now_ns=self.get_clock().now().nanoseconds,
                                                   monotonic=now)
            transaction = getattr(self, 'initial_pose_transaction', None)
            if transaction is not None:
                sample = self.typed_view.state
                transaction.observe_ready(typed.identity, ready=typed.ready,
                    source_ns=sample.source_ns if sample is not None else 0)
                initial_feedback = transaction.feedback()
        report = diagnostics(s, global_state, scan_state,
            session_id=self.session['id'], now=wall, feedback=feedback,
            navigation_context=typed, committed_route_view=getattr(self, 'committed_route_view', None),
            owner_state={} if waiting_goal_owner else bt_state,
            planning_scope=self.session.get('planning_scope', 'navigation'),
            initial_pose_status=initial_feedback)
        if typed is not None:
            feedback = None
        report['behavior_tree'] = tree_presentation(bt_state)
        if waiting_goal_owner:
            # Old task geometry may remain visible while retirement finishes,
            # but an old failure/trajectory cannot describe this new request.
            # Keep self.bt_state intact: this is not a second task authority.
            report['behavior_tree'] = dict(
                label='切换目标' if bt_state else '等待当前任务状态',
                detail='等待当前目标的任务状态',
                task_id=getattr(self, 'goal_request_task_id', None) or '',
                root_status='', active_node='', nodes=[], transitions=[])
            report['stages']['global'] = dict(label='等待当前目标路线', tone='muted', detail='')
            report['stages']['local'] = dict(label='等待新路线', tone='muted', detail='')
            report['local_target'] = report['reference_horizon_m'] = report['plan_id'] = None
        if (self.session.get('planning_scope') == 'global_route'
                and report['behavior_tree']['label'] == '任务完成'):
            report['behavior_tree']['label'] = '全局规划完成'
            report['behavior_tree']['detail'] = '仅生成路线，不表示机器人已到达'
        reason = self.reason
        if waiting_goal_owner:
            reason = report['behavior_tree']['label']
        elif self.reason_scope == 'planning':
            # A submission notice must not outlive the actual request. Display
            # the same authoritative stage as the panel, including expiry and
            # failure; never leave "waiting for planning" after goal revocation.
            if bt_state.get('received_at_unix', 0.) >= self.goal_submitted_at:
                reason = report['behavior_tree']['label']
            elif global_state.get('received_at_unix', 0.) >= self.goal_submitted_at:
                reason = report['stages']['global']['label']
            elif wall-self.goal_submitted_at > 2.:
                reason = '未收到规划状态'
        self.marker.publish(m)
        self.publish_goal_editor()
        self.diagnostics_pub.publish(String(data=json.dumps(
            report, ensure_ascii=False, allow_nan=False)))
        # Small read-only Web status; all pose/goal input remains exclusively RViz.
        if now - self.last_snapshot_at >= .5:
            self.last_snapshot_at = now
            snapshot = {'session_id': self.session['id'], 'received_at_unix': time.time(),
                'mode': self.session['mode'], 'motion_enabled': False, 'motion_capable': self.motion_capable,
                'physical_motion_capable': self.physical_motion_capable,
                'reason': reason, 'seed_feedback': feedback,
                'initial_pose_feedback': initial_feedback,
                'scan_status': scan_state, 'global_status': global_state,
                'behavior_tree': report['behavior_tree']}
            try:
                target = self.directory / 'view_status.tmp'
                target.write_text(json.dumps(snapshot, ensure_ascii=False))
                os.replace(target, self.directory / 'view_status.json')
            except OSError as error:
                self.get_logger().warning('View status snapshot unavailable: ' + str(error))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--session', type=Path, required=True)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = LiveView(args.session)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.try_shutdown()
        except RCLError:
            # SIGINT can finish shutdown between the context check and call.
            # Suppress only an already-closed context, not other ROS errors.
            if rclpy.ok():
                raise


if __name__ == '__main__':
    main()
