"""RViz goal + map-frame odometry -> validated native PCT reference route.

No velocity publisher, SDK import, take-over or motion authorization exists here.
"""
import json
import math
import multiprocessing
import queue
import time
import uuid

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Empty, String
from d1max_planning_interfaces.msg import ReferencePath

from .route_engine import worker_main
from .progress_watchdog import TranslationProgressWatchdog


def pose_xyz(pose):
    point = pose.position
    values = np.array([point.x, point.y, point.z], dtype=float)
    if not np.isfinite(values).all():
        raise ValueError('Pose contains non-finite coordinates')
    return values


class PctRouteServer(Node):
    def __init__(self):
        super().__init__('pct_route_server')
        defaults = {
            'planning_grid': '', 'vendor_root': '/home/dndx/d1max_nav_ws/src/pct_planner_vendor',
            'planning_frame': 'd1max_loc_map', 'body_frame': 'd1max_loc_base_link',
            'odometry_topic': '/d1max/localization/odometry/global',
            'goal_topic': '/d1max/pct_scan/goal', 'cancel_topic': '/d1max/pct_scan/cancel',
            'path_topic': '/d1max/pct_scan/global_path',
            'reference_topic': '/d1max/pct_scan/reference_path',
            'status_topic': '/d1max/pct_scan/global_status', 'task_topic': '/d1max/pct_scan/task',
            'session_id': '', 'odometry_timeout_s': 0.5, 'goal_max_age_s': 5.0,
            'planning_timeout_s': 20.0, 'max_start_motion_m': 0.4,
            'progress_timeout_s': 20.0, 'progress_distance_m': 0.10,
            'goal_tolerance_m': 0.25, 'body_height_m': 0.55,
            'cost_margin_m': 0.6, 'minimum_clearance_m': 0.2,
            'optimization_guard_cells': 1,
            'max_heading_rate': 10.0, 'max_ground_step_m': 0.15,
        }
        self.settings = {name: self.declare_parameter(name, default).value
                         for name, default in defaults.items()}
        self.frame = self.settings['planning_frame']
        self.session = self.settings['session_id'] or str(uuid.uuid4())
        for name in ('odometry_timeout_s', 'goal_max_age_s', 'planning_timeout_s',
                     'max_start_motion_m', 'goal_tolerance_m', 'body_height_m',
                     'cost_margin_m', 'max_heading_rate', 'max_ground_step_m'):
            value = float(self.settings[name])
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if not self.frame or not self.settings['body_frame']:
            raise ValueError('Explicit planning and body frames are required')
        self.generation = 0
        self.state = 'initializing'
        self.reason = 'Loading native PCT and measured map'
        self.active = False
        self.issued_at = 0.0
        self.target = None
        self.odom = None
        self.odom_receipt = 0.0
        self.worker_ready = False
        self.last_clock = None
        self.pending = None
        self.map_info = {}
        self.last_result = {}
        self.progress_watchdog = TranslationProgressWatchdog(
            self.settings['progress_timeout_s'], self.settings['progress_distance_m'])
        self.worker_context = multiprocessing.get_context('spawn')
        self.requests = self.worker_context.Queue(maxsize=1)
        self.results = self.worker_context.Queue(maxsize=4)
        self.worker = self.worker_context.Process(target=worker_main,
            args=(self.settings, self.requests, self.results), daemon=True)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, self.settings['path_topic'], latched)
        self.reference_pub = self.create_publisher(ReferencePath, self.settings['reference_topic'], latched)
        self.task_pub = self.create_publisher(String, self.settings['task_topic'], latched)
        self.status_pub = self.create_publisher(String, self.settings['status_topic'], latched)
        self.create_subscription(Odometry, self.settings['odometry_topic'], self.on_odom,
                                 qos_profile_sensor_data)
        self.create_subscription(PoseStamped, self.settings['goal_topic'], self.on_goal, 1)
        self.create_subscription(Empty, self.settings['cancel_topic'], self.on_cancel, 1)
        self.timer = self.create_timer(0.2, self.tick)
        self.created_monotonic = time.monotonic()
        self.publish_task()
        self.clear_route()
        self.worker.start()

    def ros_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def task(self):
        return {
            'session_id': self.session, 'generation': self.generation,
            'active': self.active, 'issued_at': self.issued_at,
            'heartbeat_at': self.ros_seconds(), 'frame_id': self.frame,
            'target_xyz': self.target, 'state': self.state, 'reason': self.reason,
        }

    def publish_task(self):
        self.task_pub.publish(String(data=json.dumps(self.task(), allow_nan=False)))

    def clear_route(self):
        path = Path()
        path.header.frame_id = self.frame
        path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(path)
        self.reference_pub.publish(ReferencePath(session_id=self.session,
                                                 generation=self.generation, path=path))

    def invalidate(self, state, reason, new_generation=False):
        # SCAN deliberately rejects repeated typed-reference generations. Any
        # active/pending task termination therefore needs a newer generation
        # for its empty reference, not just an inactive tracker heartbeat.
        if new_generation or self.active or self.pending is not None:
            self.generation += 1
            self.issued_at = self.ros_seconds()
        self.active = False
        self.progress_watchdog.deactivate()
        self.target = None
        self.pending = None
        self.state, self.reason = state, reason
        self.publish_task()
        self.clear_route()

    def on_cancel(self, _message):
        self.invalidate('canceled', 'Canceled by operator', new_generation=True)

    def on_odom(self, msg):
        try:
            pose_xyz(msg.pose.pose)
            quaternion = msg.pose.pose.orientation
            quat = np.array([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
            if not np.isfinite(quat).all() or abs(np.linalg.norm(quat) - 1.0) > 0.05:
                raise ValueError('Odometry quaternion is invalid')
            if msg.header.frame_id != self.frame or msg.child_frame_id != self.settings['body_frame']:
                raise ValueError('Odometry frame contract mismatch; do not relabel coordinates')
        except ValueError as exc:
            self.odom = None
            self.invalidate('waiting_odometry', str(exc))
            return
        self.odom, self.odom_receipt = msg, time.monotonic()

    def fresh_odom(self):
        if self.odom is None or time.monotonic() - self.odom_receipt > self.settings['odometry_timeout_s']:
            raise ValueError('Waiting for fresh map-frame body odometry')
        stamp = self.odom.header.stamp.sec + self.odom.header.stamp.nanosec * 1e-9
        age = self.ros_seconds() - stamp
        if age > self.settings['odometry_timeout_s'] or age < -0.1:
            raise ValueError('Odometry timestamp is stale or ahead of the ROS clock')
        return pose_xyz(self.odom.pose.pose)

    def on_goal(self, msg):
        # Withdraw old task permission before checking/replanning a replacement.
        self.invalidate('planning', 'Checking new RViz goal', new_generation=True)
        try:
            if not self.worker_ready or not self.worker.is_alive():
                raise ValueError('Native PCT worker is not ready')
            if msg.header.frame_id != self.frame:
                raise ValueError(f'Goal must be expressed in {self.frame}; no implicit TF/relabeling')
            target = pose_xyz(msg.pose)
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            age = self.ros_seconds() - stamp
            if stamp and (age > self.settings['goal_max_age_s'] or age < -0.1):
                raise ValueError('Goal timestamp is stale or in the future')
            start = self.fresh_odom()
            request = {'generation': self.generation, 'start_xy': start[:2].tolist(),
                       'goal_xy': target[:2].tolist()}
            self.requests.put_nowait(request)
            self.pending = {**request, 'requested_at': time.monotonic()}
            self.reason = 'Native PCT search and GPMP optimization running'
        except (ValueError, queue.Full) as exc:
            self.invalidate('failed', str(exc) or 'Native PCT request queue is busy')

    def accept_result(self, result):
        if not self.pending or result['generation'] != self.generation:
            return
        if result['kind'] == 'failed':
            self.invalidate('failed', result['error'])
            return
        try:
            current = self.fresh_odom()
            if np.linalg.norm(current[:2] - self.pending['start_xy']) > self.settings['max_start_motion_m']:
                raise ValueError('Robot moved too far during global planning; request a new goal')
            points = np.asarray(result['path'], dtype=float)
            self.target = points[-1].tolist()
            self.target[2] += self.settings['body_height_m']
            path = Path()
            path.header.frame_id = self.frame
            path.header.stamp = self.get_clock().now().to_msg()
            for i, point in enumerate(points):
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, point)
                direction = points[min(i + 1, len(points) - 1), :2] - points[max(i - 1, 0), :2]
                yaw = math.atan2(direction[1], direction[0])
                pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
                path.poses.append(pose)
            # Planning first advertised an inactive generation to revoke old
            # trajectories. Activation must use a *new* generation: trackers
            # deliberately never resurrect a canceled/terminal generation.
            self.generation += 1
            self.issued_at = self.ros_seconds()
            self.active, self.state, self.reason = True, 'active', 'Validated native PCT reference published'
            self.progress_watchdog.activate(current[:2], time.monotonic())
            self.last_result = {k: v for k, v in result.items() if k != 'path'}
            self.pending = None
            self.publish_task()
            self.path_pub.publish(path)
            self.reference_pub.publish(ReferencePath(session_id=self.session,
                                                     generation=self.generation, path=path))
        except ValueError as exc:
            self.invalidate('failed', str(exc))

    def tick(self):
        now = self.ros_seconds()
        if self.last_clock is not None and now < self.last_clock - 0.1:
            self.odom = None
            self.invalidate('waiting_odometry', 'ROS clock moved backwards', new_generation=True)
        self.last_clock = now
        try:
            while True:
                result = self.results.get_nowait()
                if result['kind'] == 'ready':
                    self.worker_ready = True
                    self.map_info = result
                    if self.state == 'initializing':
                        self.state, self.reason = 'waiting_odometry', 'Map ready; waiting for odometry'
                elif result['kind'] == 'initialization_failed':
                    self.invalidate('failed', result['error'])
                else:
                    self.accept_result(result)
        except queue.Empty:
            pass
        if not self.worker.is_alive() and self.state != 'failed':
            self.worker_ready = False
            self.invalidate('failed', 'Native PCT worker exited; restart the planner')
        try:
            current = self.fresh_odom()
            if self.state == 'waiting_odometry' and self.worker_ready:
                self.state, self.reason = 'idle', 'Ready for an RViz goal'
            if self.active and self.target is not None and np.linalg.norm(current[:2] - self.target[:2]) <= self.settings['goal_tolerance_m']:
                self.invalidate('arrived', 'Goal position reached')
            elif self.active and self.progress_watchdog.observe(current[:2], time.monotonic()):
                self.invalidate('blocked',
                    'No translation progress; inspect planner/obstacle safety stop',
                    new_generation=True)
        except ValueError as exc:
            if self.active or self.pending:
                self.invalidate('waiting_odometry', str(exc), new_generation=True)
        timeout = float(self.settings['planning_timeout_s'])
        if self.pending and time.monotonic() - self.pending['requested_at'] > timeout:
            self.worker.terminate()
            self.worker_ready = False
            self.invalidate('failed', 'Native PCT planning timed out; restart required', new_generation=True)
        if self.state == 'initializing' and time.monotonic() - self.created_monotonic > timeout:
            self.worker.terminate()
            self.invalidate('failed', 'Native PCT initialization timed out')
        self.publish_task()
        status = {**self.task(), 'ready': self.worker_ready and self.state in ('idle', 'active', 'arrived', 'canceled'),
                  'map': self.map_info, 'last_result': self.last_result,
                  'translation_progress': self.progress_watchdog.snapshot(),
                  'algorithm': 'native_pct_gpmp', 'robot_commands_sent': False}
        self.status_pub.publish(String(data=json.dumps(status, allow_nan=False)))

    def destroy_node(self):
        if rclpy.ok():
            self.invalidate('stopped', 'PCT route server stopped', new_generation=True)
        if self.worker.is_alive():
            self.worker.terminate()
        self.worker.join(timeout=2.0)
        for channel in (self.requests, self.results):
            channel.cancel_join_thread()
            channel.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PctRouteServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
