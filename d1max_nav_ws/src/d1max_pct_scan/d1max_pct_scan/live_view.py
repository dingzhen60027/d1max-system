"""Live-only RViz initial-pose mailbox and state. Never sends robot commands."""
import argparse
import json
import math
import os
from pathlib import Path
import time
import uuid
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy._rclpy_pybind11 import RCLError
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool, Empty, ColorRGBA
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, InteractiveMarkerFeedback
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from d1max_pct_planner.preview_markers import pose_marker
from .live_ui_contract import (initial_pose_command, initial_pose_feedback,
                               checked_session_status, source_status_fresh, localization_preview_state)
from .live_diagnostics import diagnostics, body_identity, body_visual_state
from .live_goal_editor import goal_position
from .live_view_reload import preview_reload_session


BODY_AXIS_LENGTH_M = .9
BODY_AXIS_WIDTH_M = .05


def body_axes_marker(body, lifetime_sec):
    """Local RGB axes at the measured pose, not an offset or a new pose sample."""
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
    # The caller supplies only the unexpired part of the original source lease.
    marker.lifetime = Duration(seconds=max(1e-9, lifetime_sec)).to_msg()
    return marker


class LiveView(Node):
    def __init__(self, directory):
        super().__init__('d1max_live_view')
        self.directory = directory
        self.session = json.loads((directory / 'session.json').read_text())
        self.motion_capable = (self.session.get('mode') == 'LIVE_NAVIGATION'
                               and self.session.get('motion_control_enabled') is True)
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
        self.goal_pose = None
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
        # In execution sessions the coordinator owns freeze feedback, including
        # the disarmed state. A second always-true publisher would fight it.
        runtime_freeze = self.session.get('preview_freeze_owner') == 'supervisor_child'
        if runtime_freeze:
            preview_reload_session(self.session)
        self.freeze = (None if self.motion_capable or runtime_freeze else
                       self.create_publisher(Bool, '/d1max/live_planning/execution_frozen', 10))
        self.cancel_pub = self.create_publisher(Empty, '/d1max/live_planning/cancel', 10)
        self.goal_pub = self.create_publisher(PointStamped, '/d1max/live_planning/goal3d', 10)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.goal_editor_pub = self.create_publisher(String, '/d1max/live_planning/goal_editor_status', latched)
        self.create_subscription(Empty, '/d1max/live_planning/activate_goal3d', self.activate_goal, 10)
        self.create_subscription(String, '/d1max/live_planning/map_layer_status', self.on_map_metadata, latched)
        self.create_subscription(PointCloud2, '/d1max/live_planning/traversable_surface', self.on_surfaces, latched)
        self.create_subscription(String, '/d1max/localization/status', self.status, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/d1max/live_planning/initialpose', self.initial, 10)
        self.create_subscription(String, '/d1max/live_planning/global_status', self.on_global, 10)
        self.create_subscription(String, '/d1max/live_planning/scan_bridge_status', self.on_scan, 10)
        self.create_subscription(PointStamped, '/d1max/live_planning/place_goal3d', self.place_goal, 10)
        self.create_subscription(Odometry, '/d1max/localization/odometry/global', self.on_body,
                                 qos_profile_sensor_data)
        self.markers = InteractiveMarkerServer(self, '/d1max_live_goal')
        self.menu = MenuHandler()
        self.menu.insert('规划到此处' if self.motion_capable else '规划到此处（只显示）', callback=self.commit_goal)
        self.menu.insert('取消任务（不是机器人急停）', callback=self.cancel)
        self.create_timer(.2, self.tick)
        self.create_timer(.05, self.publish_body)

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
        if (self.body is not None and time.monotonic()-self.body_received < .5
                and localization_preview_state(self.last_state, session_id=self.session['id'], now=time.time())[0]):
            p, q = self.body.pose.pose.position, self.body.pose.pose.orientation
            body = (p.x, p.y, p.z, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
        fallback = self.map_layer_metadata.get('initial_goal_xyz',
            (0., 0., self.session['initial_body_z']-self.session.get('body_height', .55)))
        point = goal_position(existing=existing, body=body, surfaces=self.goal_surfaces,
                              fallback=fallback, body_height=self.session.get('body_height', .55))
        message = PointStamped()
        message.header.frame_id = self.frame
        message.header.stamp = self.get_clock().now().to_msg()
        message.point.x, message.point.y, message.point.z = point
        self.place_goal(message)

    def publish_goal_editor(self):
        if self.goal_pose is None:
            return
        p = self.goal_pose.position
        self.goal_editor_pub.publish(String(data=json.dumps({
            'mode': 'LIVE_GOAL_EDITOR_NO_MOTION', 'session_id': self.session['id'],
            'received_at_unix': time.time(), 'frame_id': self.frame, 'motion_enabled': False,
            'selection_mode': 'free_xyz', 'goal_xyz': [p.x, p.y, p.z],
            'can_submit': localization_preview_state(self.last_state,
                session_id=self.session['id'], now=time.time())[0]})))

    def on_body(self, message):
        if (message.header.frame_id != self.frame or message.child_frame_id != 'd1max_loc_base_link'):
            return
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        p, q = message.pose.pose.position, message.pose.pose.orientation
        if (not -.1 <= time.time()-stamp <= .5 or stamp < self.body_barrier or
                not all(math.isfinite(x) for x in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)) or
                max(abs(p.x), abs(p.y), abs(p.z)) > 10000 or
                abs(math.hypot(q.x, q.y, q.z, q.w)-1.) > .01):
            return
        previous = self.body.header.stamp if self.body else None
        if previous and stamp <= previous.sec + previous.nanosec * 1e-9:
            return
        identity = body_identity(self.last_state, session_id=self.session['id'], now=time.time())
        if identity is None or not 0 <= time.monotonic()-self.state_at < .6:
            return
        self.body, self.body_received = message, time.monotonic()
        self.body_sample_context = identity

    def publish_body(self):
        now, wall = time.monotonic(), time.time()
        marker = Marker()
        marker.header.frame_id = self.frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns, marker.id = 'measured_body', 0
        stamp = (self.body.header.stamp.sec + self.body.header.stamp.nanosec * 1e-9) if self.body else 0.
        visual = body_visual_state(self.last_state, session_id=self.session['id'], now=wall,
            body_stamp=stamp, body_receipt_age=now-self.body_received,
            state_receipt_age=now-self.state_at, body_context=self.body_sample_context,
            body_barrier=self.body_barrier)
        marker.action = Marker.ADD if self.body is not None and visual['visible'] else Marker.DELETE
        if marker.action == Marker.ADD:
            marker = body_axes_marker(self.body, visual['lifetime_sec'])
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
            stamp_field='received_at_unix', now=time.time(), timeout=1.,
            previous_stamp=self.global_state.get('received_at_unix', 0.), maximum_bytes=65536)
        if value is not None:
            self.global_state, self.global_at = value, time.monotonic()

    def on_scan(self, message):
        value = checked_session_status(message.data, session_id=self.session['id'],
            stamp_field='received_at_unix', now=time.time(), timeout=1.,
            previous_stamp=self.scan_state.get('received_at_unix', 0.), maximum_bytes=65536)
        if value is not None:
            self.scan_state, self.scan_at = value, time.monotonic()

    def status(self, message):
        state = checked_session_status(message.data, session_id=self.session['id'],
            stamp_field='wall_time', now=time.time(), timeout=.6,
            previous_stamp=self.last_state.get('wall_time', 0.))
        if state is not None:
            context = (self.session['id'], state.get('local_epoch'), state.get('confirmed_seed_ns'),
                       state.get('active_seed_ns'))
            navigation = state.get('navigation')
            navigation = navigation if isinstance(navigation, dict) else {}
            hard_invalid = bool(state.get('local_fault') or navigation.get('fault'))
            if context != self.body_context or hard_invalid:
                self.body = None
                self.body_received = 0.
                self.body_sample_context = None
                self.body_barrier = state['wall_time']
                self.body_context = context
            self.last_state, self.state_at = state, time.monotonic()

    def initial(self, message):
        self.reason_scope = 'input'
        try:
            stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            p, q = message.pose.pose.position, message.pose.pose.orientation
            # A seed only, not a terrain constraint. PCD registration estimates full SE(3).
            command = initial_pose_command(frame=message.header.frame_id, stamp=stamp,
                now=time.time(), xy=(p.x, p.y), quaternion=(q.x, q.y, q.z, q.w),
                body_z=self.session['initial_body_z'], state=self.last_state,
                session_id=self.session['id'], state_age=time.monotonic()-self.state_at,
                command_id=uuid.uuid4().hex)
            self.cancel_pub.publish(Empty())
            path = self.directory / 'initial_pose.tmp'
            path.write_text(json.dumps(command))
            os.replace(path, self.directory / 'initial_pose.json')
            self.last_seed_id = command['id']
            self.reason = '已提交机身初值；等待连续匹配确认'
        except (ValueError, OSError) as error:
            self.reason = str(error)
        self.get_logger().info(self.reason)

    def place_goal(self, message):
        self.reason_scope = 'input'
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        position = (message.point.x, message.point.y, message.point.z)
        if (message.header.frame_id != self.frame or not -.1 <= time.time()-stamp <= 2
                or not all(math.isfinite(x) and abs(x) < 10000 for x in position)):
            self.reason = '3D 目标的坐标或时间无效'
            return
        marker = pose_marker('goal', position, (0, 0, 0, 1), self.frame, 1.5)
        marker.description = '3D GOAL · 右键规划'
        self.goal_pose = marker.pose
        self.markers.insert(marker, feedback_callback=self.on_feedback)
        self.menu.apply(self.markers, 'goal')
        self.markers.applyChanges()
        self.reason = '拖 XYZ 调整目标；右键“规划到此处”提交，旋转不约束终点朝向'
        self.publish_goal_editor()

    def on_feedback(self, feedback):
        if feedback.header.frame_id != self.frame:
            return
        p = feedback.pose.position
        if not all(math.isfinite(x) and abs(x) < 10000 for x in (p.x, p.y, p.z)):
            return
        if feedback.event_type in (InteractiveMarkerFeedback.POSE_UPDATE, InteractiveMarkerFeedback.MOUSE_UP):
            self.goal_pose = feedback.pose

    def commit_goal(self, feedback):
        if self.goal_pose is None:
            return
        if not localization_preview_state(self.last_state, session_id=self.session['id'], now=time.time())[0]:
            self.reason = '目标仅供编辑：请先完成定位，再右键明确提交'
            self.publish_goal_editor()
            return
        m = PointStamped()
        m.header.frame_id = self.frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.point = self.goal_pose.position
        self.goal_pub.publish(m)
        self.reason = '目标已提交，等待规划'
        self.reason_scope = 'planning'
        self.goal_submitted_at = time.time()

    def cancel(self, _):
        self.cancel_pub.publish(Empty())
        self.reason = '已取消任务并撤销导航运动许可' if self.motion_capable else '已取消规划预览；本入口从不发送运动'
        self.reason_scope = 'input'

    def tick(self):
        # Freeze the native spline execution clock, NOT the robot. SCAN can
        # still replan from current measured odometry; there is no tracker.
        if self.freeze is not None:
            self.freeze.publish(Bool(data=True))
        now, wall = time.monotonic(), time.time()
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
        report = diagnostics(s, global_state, scan_state,
            session_id=self.session['id'], now=wall, feedback=feedback)
        reason = self.reason
        if self.reason_scope == 'planning':
            # A submission notice must not outlive the actual request. Display
            # the same authoritative stage as the panel, including expiry and
            # failure; never leave "waiting for planning" after goal revocation.
            if global_state.get('received_at_unix', 0.) >= self.goal_submitted_at:
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
                'reason': reason, 'seed_feedback': feedback,
                'scan_status': scan_state, 'global_status': global_state}
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
