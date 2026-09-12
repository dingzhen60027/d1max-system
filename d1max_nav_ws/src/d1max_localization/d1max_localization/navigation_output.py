"""ROS boundary for robot_localization and the only public navigation TF writer.

PCD coordinator and IMU predictor communicate through versioned, epoch-tagged
contracts. robot_localization's unguarded output is PRIVATE. Its SetPose service
is serialized and never retried on an ambiguous timeout. All public outputs are
admitted against source ages, seed/epoch, common-time history and jump limits.
This node never connects to the robot or publishes a motion command.
"""

from collections import deque
from dataclasses import fields
import json
import math
import signal
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistWithCovarianceStamped
from robot_localization.srv import SetPose
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from .math_utils import compose, inverse, rotate_vector
from .initial_pose import body_to_tracking_transform
from .estimation.contracts import MotionState, body_state, fresh
from .estimation.navigation import NavigationState, NavigationLimits
from .estimation_ros import (
    fill_pose,
    odometry,
    pose_message,
    read_pose,
    seconds,
    stamp_time,
    transform,
)

PREFIX = "/d1max/localization/"


class NavigationOutput(Node):
    def __init__(self):
        super().__init__("navigation_output")
        self.rate = self.declare_parameter("output_rate_hz", 50.0).value
        if not 20.0 <= self.rate <= 100.0:
            raise ValueError("navigation output rate must be 20..100 Hz")
        defaults = {
            "map_frame": "d1max_loc_map",
            "odom_frame": "d1max_loc_odom",
            "tracking_frame": "d1max_loc_tracking",
            "body_frame": "d1max_loc_base_link",
            "tracking_offset_body": [0.4043, 0.0, -0.0377],
            "sdk_to_tracking_yaw": 1.56646,
            "extrinsics_verified": False,
            "time_alignment_verified": False,
            "trajectory_rate_hz": 5.0,
            "trajectory_max_points": 1800,
        }
        self.p = {key: self.declare_parameter(key, value).value for key, value in defaults.items()}
        if (
            not math.isfinite(self.p["trajectory_rate_hz"])
            or not 0.1 <= self.p["trajectory_rate_hz"] <= self.rate
            or type(self.p["trajectory_max_points"]) is not int
            or not 1 <= self.p["trajectory_max_points"] <= 10000
        ):
            raise ValueError("invalid bounded trajectory settings")
        if (
            len(set(self.p[k] for k in ("map_frame", "odom_frame", "tracking_frame", "body_frame")))
            != 4
        ):
            raise ValueError("navigation frames must be distinct")
        limits = {
            f.name: self.declare_parameter(
                "limits." + f.name, getattr(NavigationLimits(), f.name)
            ).value
            for f in fields(NavigationLimits)
        }
        self.core = NavigationState(
            NavigationLimits(**limits),
            self.p["odom_frame"],
            self.p["tracking_frame"],
            self.p["map_frame"],
        )
        self.extrinsic = body_to_tracking_transform(
            self.p["tracking_offset_body"], self.p["sdk_to_tracking_yaw"]
        )
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.static_tf.sendTransform(
            transform(self.p["body_frame"], self.p["tracking_frame"], self.extrinsic, self.now_s())
        )
        self.local_pub = self.create_publisher(Odometry, PREFIX + "odometry/local", 20)
        self.global_pub = self.create_publisher(Odometry, PREFIX + "odometry/global", 20)
        self.pose_pub = self.create_publisher(PoseStamped, PREFIX + "pose", 20)
        self.path_pub = self.create_publisher(Path, PREFIX + "trajectory", 2)
        self.status_pub = self.create_publisher(String, PREFIX + "navigation/status", 10)
        self.motion_pub = self.create_publisher(
            TwistWithCovarianceStamped, PREFIX + "estimator/motion", 20
        )
        self.map_pub = self.create_publisher(
            PoseWithCovarianceStamped, PREFIX + "estimator/map_pose", 10
        )
        self.reset_client = self.create_client(SetPose, PREFIX + "estimator/set_pose")
        self.local_sub = self.create_subscription(
            String, PREFIX + "prediction/local_sample", self.on_local, 20
        )
        self.map_sub = self.create_subscription(String, PREFIX + "map_alignment", self.on_map, 10)
        self.filter_sub = self.create_subscription(
            Odometry, PREFIX + "estimator/odometry_raw", self.on_filtered, 20
        )
        self.future = None
        self.request_key = None
        self.request_stamp = 0.0
        self.request_at = 0.0
        self.last_map_sent = 0.0
        self.last_local_sent = 0.0
        self.last_path = 0.0
        self.last_status = 0.0
        self.last_error = ""
        self.local_times = deque(maxlen=256)
        self.global_times = deque(maxlen=256)
        self.path = deque(maxlen=int(self.p["trajectory_max_points"]))
        self.predictor_status = {}
        self.timer = self.create_timer(1.0 / self.rate, self.tick)

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def clear_path(self):
        self.path.clear()
        self.last_path = 0.0
        m = Path()
        m.header.frame_id = self.p["map_frame"]
        m.header.stamp = stamp_time(self.now_s())
        self.path_pub.publish(m)

    def on_local(self, message):
        try:
            value = json.loads(message.data)
            old_epoch = self.core.epoch
            if not self.core.push_local(value, self.now_s()):
                return
            self.predictor_status = {
                k: value.get(k)
                for k in (
                    "reason",
                    "propagation_age_sec",
                    "extrapolation_sec",
                    "imu_received",
                    "imu_rejected",
                )
            }
            if old_epoch != self.core.epoch:
                self.clear_path()
                self.local_times.clear()
                self.global_times.clear()
                self.last_local_sent = 0.0
                self.last_map_sent = 0.0
            if not self.core.local_ready(self.now_s()):
                return
            state = self.core.local[-1]
            # Only twist from the LIO/IMU predictor enters the global EKF. Do not
            # also fuse LIO pose, raw IMU or MC as independent copies of motion.
            m = TwistWithCovarianceStamped()
            m.header.frame_id = self.p["tracking_frame"]
            m.header.stamp = stamp_time(state.stamp)
            m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z = state.linear
            m.twist.twist.angular.x, m.twist.twist.angular.y, m.twist.twist.angular.z = (
                state.angular
            )
            m.twist.covariance = state.twist_covariance
            self.motion_pub.publish(m)
        except (ValueError, KeyError, TypeError, OverflowError) as error:
            self.last_error = str(error)

    def on_map(self, message):
        try:
            old_key = self.core.key()
            if (
                self.core.accept_map(json.loads(message.data), self.now_s())
                and old_key != self.core.key()
            ):
                self.clear_path()
        except (ValueError, KeyError, TypeError, OverflowError) as error:
            self.last_error = str(error)

    def on_filtered(self, message):
        if (
            message.header.frame_id != self.p["map_frame"]
            or message.child_frame_id != self.p["tracking_frame"]
        ):
            return
        try:
            v = message.twist.twist.linear
            w = message.twist.twist.angular
            self.core.push_filtered(
                seconds(message),
                read_pose(message.pose.pose),
                [float(x) for x in message.pose.covariance],
                (v.x, v.y, v.z),
                (w.x, w.y, w.z),
                self.now_s(),
            )
        except (ValueError, TypeError, OverflowError) as error:
            self.last_error = str(error)

    def filter_lifecycle(self, now):
        if self.future is not None:
            if self.future.done():
                try:
                    if self.future.result() is None:
                        raise RuntimeError("empty filter reset result")
                    self.core.reset_ack(self.request_key, self.request_stamp, now)
                except Exception as error:
                    self.core.filter_fault = "filter_reset_unknown"
                    self.last_error = str(error)
                self.future = None
            elif time.monotonic() - self.request_at > self.core.limits.reset_timeout:
                self.core.filter_fault = "filter_reset_unknown"
                self.last_error = "滤波器重置回执超时，禁止自动重试；停止并重启定位"
            return
        if not self.reset_client.service_is_ready():
            return
        reset = self.core.begin_reset(now)
        if reset:
            key, stamp, pose = reset
            request = SetPose.Request()
            request.pose = pose_message(
                self.p["map_frame"], pose, stamp, self.core.map_contract["covariance"]
            )
            self.request_key = key
            self.request_stamp = stamp
            self.request_at = time.monotonic()
            self.last_map_sent = stamp
            try:
                self.future = self.reset_client.call_async(request)
            except Exception as error:
                self.core.filter_fault = "filter_reset_unknown"
                self.last_error = str(error)
        if (
            self.core.reset_ack_at
            and self.core.filter_key == self.core.key()
            and self.core.map_ready(now)
        ):
            c = self.core.map_contract
            if c["stamp"] > self.last_map_sent:
                self.map_pub.publish(
                    pose_message(self.p["map_frame"], c["pose"], c["stamp"], c["covariance"])
                )
                self.last_map_sent = c["stamp"]

    def publish_local(self, state):
        if state.stamp <= self.last_local_sent:
            return False
        pose, linear, angular, pc, tc = body_state(state, self.extrinsic)
        self.local_pub.publish(
            odometry(
                self.p["odom_frame"],
                self.p["body_frame"],
                pose,
                state.stamp,
                linear,
                angular,
                pc,
                tc,
            )
        )
        self.tf.sendTransform(
            transform(self.p["odom_frame"], self.p["body_frame"], pose, state.stamp)
        )
        self.last_local_sent = state.stamp
        self.local_times.append(state.stamp)
        return True

    def publish_global(self, local, pose, pc):
        state = MotionState(
            local.stamp,
            pose,
            rotate_vector(pose.orientation, local.linear),
            local.angular,
            pc,
            local.twist_covariance,
            local.source_stamp,
            local.imu_stamp,
            local.extrapolation,
        )
        body, linear, angular, body_cov, tc = body_state(state, self.extrinsic)
        self.global_pub.publish(
            odometry(
                self.p["map_frame"],
                self.p["body_frame"],
                body,
                local.stamp,
                linear,
                angular,
                body_cov,
                tc,
            )
        )
        self.tf.sendTransform(
            transform(
                self.p["map_frame"],
                self.p["odom_frame"],
                compose(pose, inverse(local.pose)),
                local.stamp,
            )
        )
        m = pose_message(self.p["map_frame"], pose, local.stamp)
        self.pose_pub.publish(m)
        self.global_times.append(local.stamp)
        if local.stamp - self.last_path >= 1.0 / self.p["trajectory_rate_hz"]:
            self.path.append(m)
            path = Path()
            path.header = m.header
            path.poses = list(self.path)
            self.path_pub.publish(path)
            self.last_path = local.stamp

    @staticmethod
    def rate_of(values, now):
        while values and now - values[0] > 2.0:
            values.popleft()
        if len(values) < 2 or not fresh(values[-1], now, 0.12):
            return 0.0
        span = values[-1] - values[0]
        return (len(values) - 1) / span if span >= 0.5 else 0.0

    def tick(self):
        now = self.now_s()
        self.filter_lifecycle(now)
        output = self.core.output(now)
        if output:
            local, pose, pc = output
            if self.publish_local(local):
                self.publish_global(local, pose, pc)
        elif self.core.local_ready(now) and self.core.reason != "tracking":
            self.publish_local(self.core.local[-1])
        if now - self.last_status < 0.2:
            return
        self.last_status = now
        local_hz = self.rate_of(self.local_times, now)
        global_hz = self.rate_of(self.global_times, now)
        output_fresh = bool(
            self.global_times and fresh(self.global_times[-1], now, self.core.limits.local_timeout)
        )
        valid = (
            output_fresh
            and self.core.reason == "tracking"
            and self.core.local_ready(now)
            and self.core.map_ready(now)
            and not self.core.fault
            and not self.core.filter_fault
            and self.core.filter_key == self.core.key()
        )
        ready = (
            valid
            and self.p["extrinsics_verified"]
            and self.p["time_alignment_verified"]
            and 0.8 * self.rate <= global_hz <= 1.2 * self.rate
        )
        value = {
            "schema": 1,
            "epoch": self.core.epoch,
            "seed_id": self.core.key()[1] if self.core.key() else None,
            "received_at_unix": now,
            "valid": valid,
            "navigation_ready": bool(ready),
            "motion_control_enabled": False,
            "state": self.core.reason,
            "fault": self.core.fault or self.core.filter_fault,
            "last_error": self.last_error,
            "backend": "faster_lio_imu_prediction_robot_localization",
            "target_hz": self.rate,
            "local_observed_hz": local_hz,
            "global_observed_hz": global_hz,
            "local_fresh": self.core.local_ready(now),
            "output_age_sec": now - self.global_times[-1] if self.global_times else None,
            "prediction": self.predictor_status,
            "reset_pending": self.future is not None,
            "body_frame": self.p["body_frame"],
            "display_pose_reference": self.p["tracking_frame"],
            "calibration": {
                "extrinsics_verified": self.p["extrinsics_verified"],
                "time_alignment_verified": self.p["time_alignment_verified"],
            },
        }
        message = String()
        message.data = json.dumps(value, ensure_ascii=False, allow_nan=False)
        self.status_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationOutput()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        rclpy.try_shutdown()
