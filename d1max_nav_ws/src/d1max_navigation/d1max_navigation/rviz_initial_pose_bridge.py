"""Bounded RViz initial-pose adapter. Does not start services or move the robot."""
from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
import json
from queue import Empty, Full, Queue
import signal
import threading
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from .initial_pose import (
    LocalWebClient, PoseRejected, SubmissionUnknown, WebUnavailable,
    finite_number, pose_payload, submit_checked_pose, validate_context,
    validate_pose_age,
)


class RvizInitialPoseBridge(Node):
    def __init__(self):
        super().__init__('rviz_initial_pose_bridge')
        defaults = {
            'web_url': 'http://127.0.0.1:8766',
            'initial_pose_topic': '/d1max/navigation/initialpose',
            'status_topic': '/d1max/navigation/initial_pose_status',
            'map_frame': 'd1max_loc_map',
            'expected_version_id': '',
            'expected_session_id': '',
            'initial_pose_z': 0.0,
            'max_pose_age_sec': 3.0,
            'max_health_age_sec': 2.0,
            'request_timeout_sec': 1.0,
            'ack_timeout_sec': 3.0,
            'offline': False,
        }
        self.p = {key: self.declare_parameter(key, value).value for key, value in defaults.items()}
        finite_number(self.p['initial_pose_z'], '机身初值高度')
        for name in ('max_pose_age_sec', 'max_health_age_sec', 'ack_timeout_sec'):
            value = finite_number(self.p[name], name)
            if not 0 < value <= 5:
                raise ValueError(f'{name} 必须位于 0–5 秒')
        if abs(self.p['initial_pose_z']) > 100:
            raise ValueError('initial_pose_z 超出范围')
        self.client = LocalWebClient(self.p['web_url'], self.p['request_timeout_sec'])
        self.events = Queue(maxsize=32)
        self.event_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix='rviz-pose-http')
        self.future = None
        self.busy_lock = threading.Lock()
        status_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, self.p['status_topic'], status_qos)
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped, self.p['initial_pose_topic'], self.on_pose, 1)
        self.event_timer = self.create_timer(0.1, self.flush_events)
        if self.p['offline']:
            self.emit('offline', '离线地图检查；未连接机器人，初值提交已禁用，未启动定位或运动')
        else:
            self.emit('waiting_for_pose', '等待 RViz 2D Pose Estimate；仅设置匹配初值，不移动机器人')

    def emit(self, state, message, **extra):
        event = {'state': state, 'message': message, 'wall_time': time.time(),
                 'map_version_id': self.p['expected_version_id'],
                 'map_frame': self.p['map_frame'], 'body_z': self.p['initial_pose_z'],
                 'motion_enabled': False, **extra}
        with self.event_lock:
            try:
                self.events.put_nowait(event)
            except Full:
                try:
                    self.events.get_nowait()
                except Empty:
                    pass
                self.events.put_nowait(event)

    def flush_events(self):
        while True:
            try:
                event = self.events.get_nowait()
            except Empty:
                break
            msg = String()
            msg.data = json.dumps(event, ensure_ascii=False, allow_nan=False)
            self.status_pub.publish(msg)
            line = f"[{event['state']}] {event['message']}"
            if event['state'] in {'rejected', 'submission_unknown', 'ack_timeout', 'unavailable'}:
                self.get_logger().warning(line)
            else:
                self.get_logger().info(line)

    def on_pose(self, msg):
        if self.p['offline']:
            self.emit('rejected', '当前为离线模式：未向 Web 提交初值，也不会启动机器人连接')
            return
        with self.busy_lock:
            if self.future is not None and not self.future.done():
                self.emit('rejected', '上一条初值正在核对；本条已丢弃，不排队、不重试')
                return
            pose = msg.pose.pose
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
            now = self.get_clock().now().nanoseconds*1e-9
            try:
                payload = pose_payload(
                    frame=msg.header.frame_id, expected_frame=self.p['map_frame'],
                    stamp=stamp, now=now,
                    position=(pose.position.x, pose.position.y, pose.position.z),
                    quaternion=(pose.orientation.x, pose.orientation.y,
                                pose.orientation.z, pose.orientation.w),
                    body_z=self.p['initial_pose_z'], max_age=self.p['max_pose_age_sec'])
                # Covariance is not consumed by the existing bounded seed search.
                for value in msg.pose.covariance:
                    finite_number(value, 'RViz 初值协方差')
            except (PoseRejected, TypeError, ValueError) as error:
                self.emit('rejected', str(error))
                return
            self.emit('checking', '核对地图版本、定位会话和传感器就绪状态')
            self.future = self.worker.submit(self.submit, payload, stamp, now, time.monotonic())

    def submit(self, payload, stamp, received_ros_time, received_monotonic):
        def check_fresh():
            if self.stop_event.is_set():
                raise PoseRejected('RViz 初值桥正在退出，取消尚未提交的请求')
            validate_pose_age(stamp, received_ros_time + time.monotonic() - received_monotonic,
                              self.p['max_pose_age_sec'])

        receipt = None
        try:
            receipt = submit_checked_pose(
                self.client, payload, expected_version_id=self.p['expected_version_id'],
                expected_session_id=self.p['expected_session_id'], map_frame=self.p['map_frame'],
                max_health_age=self.p['max_health_age_sec'], check_pose_fresh=check_fresh)
            self.emit('queued', 'Web 已接收，尚未获定位节点确认；不代表匹配成功',
                      request_id=receipt.request_id, session_id=receipt.session_id)
            deadline = time.monotonic() + self.p['ack_timeout_sec']
            while not self.stop_event.wait(0.2) and time.monotonic() < deadline:
                overview = self.client.request('/api/localization/overview')
                _, health = validate_context(
                    overview, expected_version_id=receipt.version_id,
                    expected_session_id=receipt.session_id, map_frame=self.p['map_frame'],
                    max_health_age=self.p['max_health_age_sec'], require_ready=False)
                result = health.get('command_result') or {}
                if result.get('id') != receipt.request_id:
                    continue
                if result.get('accepted') is True:
                    self.emit('seed_accepted', '定位节点已接收初值，等待点云匹配确认；不代表定位成功',
                              request_id=receipt.request_id, session_id=receipt.session_id)
                else:
                    self.emit('rejected', str(result.get('message', '定位节点拒绝初值')),
                              request_id=receipt.request_id, session_id=receipt.session_id)
                return
            if not self.stop_event.is_set():
                self.emit('ack_timeout', '初值确认超时，结果待核对；不会自动重发',
                          request_id=receipt.request_id, session_id=receipt.session_id)
        except SubmissionUnknown as error:
            self.emit('submission_unknown', str(error))
        except (PoseRejected, WebUnavailable) as error:
            if receipt is None:
                self.emit('rejected' if isinstance(error, PoseRejected) else 'unavailable', str(error))
            else:
                self.emit('ack_timeout', f'初值已提交，但确认不可用：{error}；不会自动重发',
                          request_id=receipt.request_id, session_id=receipt.session_id)
        except Exception as error:
            self.emit('submission_unknown' if receipt else 'unavailable',
                      f'初值桥异常：{type(error).__name__}；停止本次请求，不自动重试')

    def close(self):
        self.stop_event.set()
        self.worker.shutdown(wait=True, cancel_futures=True)
        self.destroy_node()


def main(args=None):
    # The systemd cgroup and ros2 launch can both signal this child. Do not let
    # rclpy's asynchronous handler race a second context shutdown in finally.
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = RvizInitialPoseBridge()
        while not stop.is_set() and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if node is not None:
            node.close()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
