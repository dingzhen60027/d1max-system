"""Read-only localization health text for the RViz map test.

The classifier intentionally has no ROS imports so its stale-data and context
guards can be tested without a robot or ROS graph. No HTTP, TF or commands.
"""
from __future__ import annotations

from dataclasses import dataclass
import json
import math
import signal
import threading
import time


@dataclass(frozen=True)
class DisplayStatus:
    state: str
    text: str
    rgb: tuple[float, float, float]


AMBER = (1.0, 0.65, 0.15)
RED = (1.0, 0.22, 0.18)
CYAN = (0.20, 0.82, 1.0)
GREEN = (0.28, 0.93, 0.55)


def _display(state, title, color, detail=''):
    text = title + '\nMOTION DISABLED'
    if detail:
        text += '\n' + detail
    return DisplayStatus(state, text, color)


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def motion_label(gate, received, now, wall):
    """Never infer motor state from a stale command-boundary status."""
    if (not isinstance(gate, dict) or not _finite(received) or not _finite(now)
            or not 0 <= now-received <= .8 or not _finite(gate.get('wall_time'))
            or not _finite(wall) or not 0 <= wall-gate['wall_time'] <= .8):
        return 'DRIVE STATE UNKNOWN - CHECK GATE'
    if gate.get('mode') != 'live':
        return 'DRIVE STATE UNKNOWN - WRONG MODE'
    if gate.get('motion_enabled') is not True:
        return 'MOTION DISABLED'
    if gate.get('armed') is not True:
        return 'DRIVE LOCKED: ' + str(gate.get('reason', 'not_armed'))[:80]
    return 'DRIVE ARMED' + (' - COMMAND ACTIVE' if gate.get('allowed') is True else ' - IDLE')


def classify_status(status, *, offline, expected_version_id,
                    expected_session_id='', map_frame='d1max_loc_map',
                    now_wall, now_monotonic, received_monotonic=None,
                    max_age=1.5):
    if offline:
        return _display('offline', 'OFFLINE - MAP ONLY', AMBER)
    if not expected_version_id:
        return _display('fault', 'FAULT', RED, 'MAP VERSION NOT PINNED')
    if received_monotonic is None:
        return _display('waiting_sensors', 'WAITING SENSORS', AMBER)
    if (not _finite(now_monotonic) or not _finite(received_monotonic)
            or not _finite(max_age) or not 0 < max_age <= 1.5
            or not 0 <= now_monotonic-received_monotonic <= max_age):
        return _display('stale', 'STALE', RED, 'LOCALIZATION STATUS NOT FRESH')
    if not isinstance(status, dict) or not status:
        return _display('fault', 'FAULT', RED, 'INVALID LOCALIZATION STATUS')
    wall = status.get('wall_time')
    if not _finite(wall) or not _finite(now_wall) or not 0 <= now_wall-wall <= max_age:
        return _display('stale', 'STALE', RED, 'STATUS TIMESTAMP NOT FRESH')
    session = status.get('session_id')
    if (not isinstance(session, str) or not session
            or (expected_session_id and session != expected_session_id)):
        return _display('fault', 'FAULT', RED, 'LOCALIZATION SESSION MISMATCH')
    if status.get('map_version_id') != expected_version_id:
        return _display('fault', 'FAULT', RED, '2D MAP / PCD VERSION MISMATCH')
    frames = status.get('frames')
    if not isinstance(frames, dict) or frames.get('map') != map_frame:
        return _display('fault', 'FAULT', RED, 'MAP FRAME MISMATCH')
    state = status.get('state')
    navigation = status.get('navigation') or {}
    if not isinstance(navigation, dict):
        return _display('fault', 'FAULT', RED, 'INVALID NAVIGATION STATUS')
    if status.get('local_fault') or navigation.get('fault') or state in {
            'fault', 'navigation_fault', 'degraded', 'lost'}:
        return _display('fault', 'FAULT', RED, 'LOCALIZATION OUTPUT NOT TRUSTED')
    sensors = status.get('sensors') or {}
    if (not isinstance(sensors, dict) or sensors.get('imu') is not True
            or sensors.get('cloud') is not True or state in {'waiting_sensors', 'calibrating', 'recovering_local'}):
        return _display('waiting_sensors', 'WAITING SENSORS', AMBER, 'WAIT FOR FRESH LIO DATA')
    if status.get('localized') is True and state == 'tracking':
        if (status.get('frontend_ready') is not True or status.get('global_ekf_fresh') is not True
                or status.get('local_ekf_fresh') is not True
                or (navigation and navigation.get('valid') is not True)):
            return _display('fault', 'FAULT', RED, 'LOCALIZATION OUTPUT NOT READY')
        # Navigation readiness and calibration acceptance are separate from a
        # successful map match. This test never enables movement.
        calibration = status.get('calibration') or {}
        checked = (isinstance(calibration, dict)
                   and calibration.get('extrinsics_verified') is True
                   and calibration.get('time_alignment_verified') is True)
        detail = '' if checked else 'CALIBRATION NOT VERIFIED'
        return _display('localized', 'LOCALIZED', GREEN, detail)
    if state == 'waiting_initial_pose':
        if status.get('initial_pose_ready') is True:
            return _display('set_initial_pose', 'SET INITIAL POSE', CYAN, 'RVIZ: 2D POSE ESTIMATE')
        return _display('waiting_sensors', 'WAITING SENSORS', AMBER, 'LIO / HEAD STATE NOT READY')
    if state in {'acquiring', 'relocalizing', 'filter_initializing'}:
        return _display('localizing', 'LOCALIZING', AMBER, 'WAIT FOR VERIFIED MATCH')
    return _display('fault', 'FAULT', RED, 'UNRECOGNIZED LOCALIZATION STATE')


def main(args=None):
    import rclpy
    from builtin_interfaces.msg import Duration
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import String
    from visualization_msgs.msg import Marker

    class StatusDisplay(Node):
        def __init__(self):
            super().__init__('navigation_test_status')
            defaults = {
                'offline': True, 'expected_version_id': '', 'expected_session_id': '',
                'map_frame': 'd1max_loc_map', 'anchor_x': 0.0, 'anchor_y': 0.0,
                'anchor_z': 1.0, 'text_height': 0.8, 'max_status_age_sec': 1.5,
                'status_topic': '/d1max/localization/status',
                'marker_topic': '/d1max/navigation/test_status',
                'show_navigation_gate': False,
            }
            self.p = {key: self.declare_parameter(key, value).value for key, value in defaults.items()}
            for name in ('anchor_x', 'anchor_y', 'anchor_z', 'text_height', 'max_status_age_sec'):
                if not _finite(self.p[name]):
                    raise ValueError(f'{name} must be finite')
            if not 0.1 <= self.p['text_height'] <= 10:
                raise ValueError('text_height must be within 0.1–10 metres')
            if not 0 < self.p['max_status_age_sec'] <= 1.5:
                raise ValueError('max_status_age_sec must be within 0–1.5 seconds')
            if not self.p['map_frame']:
                raise ValueError('map_frame must not be empty')
            self.status = None
            self.received = None
            self.gate = None
            self.gate_received = None
            marker_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                    durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.publisher = self.create_publisher(Marker, self.p['marker_topic'], marker_qos)
            self.subscription = self.create_subscription(String, self.p['status_topic'], self.receive, 1)
            if self.p['show_navigation_gate']:
                self.gate_subscription = self.create_subscription(String,
                    '/d1max/navigation/command_gate/status', self.receive_gate, 1)
            self.timer = self.create_timer(0.5, self.publish_status)
            self.publish_status()

        def receive(self, msg):
            if self.p['offline']:
                return  # Offline data can never turn the marker green.
            self.received = time.monotonic()
            try:
                self.status = json.loads(msg.data) if len(msg.data) <= 262144 else None
            except (ValueError, TypeError):
                self.status = None

        def receive_gate(self, msg):
            self.gate_received = time.monotonic()
            try:
                self.gate = json.loads(msg.data) if len(msg.data) <= 16384 else None
            except (ValueError, TypeError):
                self.gate = None

        def publish_status(self):
            state = classify_status(
                self.status, offline=self.p['offline'],
                expected_version_id=self.p['expected_version_id'],
                expected_session_id=self.p['expected_session_id'], map_frame=self.p['map_frame'],
                now_wall=time.time(), now_monotonic=time.monotonic(),
                received_monotonic=self.received, max_age=self.p['max_status_age_sec'])
            marker = Marker()
            marker.header.frame_id = self.p['map_frame']
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'd1max_navigation_test_status'
            marker.id = 0
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.position.x = float(self.p['anchor_x'])
            marker.pose.position.y = float(self.p['anchor_y'])
            marker.pose.position.z = float(self.p['anchor_z'])
            marker.pose.orientation.w = 1.0
            marker.scale.z = float(self.p['text_height'])
            marker.color.r, marker.color.g, marker.color.b = state.rgb
            marker.color.a = 1.0
            marker.text = state.text
            if self.p['show_navigation_gate'] and not self.p['offline']:
                marker.text = marker.text.replace('MOTION DISABLED', motion_label(
                    self.gate, self.gate_received, time.monotonic(), time.time()))
            marker.lifetime = Duration(sec=1)
            self.publisher.publish(marker)

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = StatusDisplay()
        while not stop.is_set() and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
