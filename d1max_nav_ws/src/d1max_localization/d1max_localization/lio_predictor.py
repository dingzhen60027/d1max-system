"""ROS adapter for bounded LIO/IMU prediction; no map, SDK or filter service."""

from dataclasses import fields
import json
import signal
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from .estimation.prediction import InertialPredictor, PredictionLimits

PREFIX = "/d1max/localization/"


class LioPredictor(Node):
    def __init__(self):
        super().__init__("lio_predictor")
        rate = self.declare_parameter("output_rate_hz", 50.0).value
        if not 20.0 <= rate <= 100.0:
            raise ValueError("prediction rate must be 20..100 Hz")
        limits = {
            f.name: self.declare_parameter(f.name, getattr(PredictionLimits(), f.name)).value
            for f in fields(PredictionLimits)
        }
        odom = self.declare_parameter("odom_frame", "d1max_loc_odom").value
        tracking = self.declare_parameter("tracking_frame", "d1max_loc_tracking").value
        self.imu_frame = self.declare_parameter("imu_frame", "d1max_loc_lidar").value
        self.core = InertialPredictor(PredictionLimits(**limits), odom, tracking)
        self.pub = self.create_publisher(String, PREFIX + "prediction/local_sample", 20)
        self.sample_sub = self.create_subscription(
            String, PREFIX + "lio/local_sample", self.on_snapshot, 20
        )
        self.imu_sub = self.create_subscription(
            Imu,
            PREFIX + "imu",
            self.on_imu,
            QoSProfile(depth=128, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.timer = self.create_timer(1.0 / rate, self.tick)
        self.last_error = ""
        self.sequence = 0

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_snapshot(self, message):
        try:
            self.core.accept(json.loads(message.data), self.now_s())
        except (ValueError, KeyError, TypeError, OverflowError) as error:
            self.last_error = str(error)

    def on_imu(self, message):
        if message.header.frame_id != self.imu_frame:
            return
        a = message.linear_acceleration
        w = message.angular_velocity
        try:
            self.core.push_imu(
                message.header.stamp.sec + message.header.stamp.nanosec * 1e-9,
                (a.x, a.y, a.z),
                (w.x, w.y, w.z),
                self.now_s(),
            )
        except (ValueError, TypeError) as error:
            self.last_error = str(error)

    def tick(self):
        now = self.now_s()
        state = self.core.evaluate(now)
        self.sequence += 1
        if self.core.epoch == 0:
            return  # no invented pre-LIO epoch
        value = {
            "schema": 1,
            "epoch": self.core.epoch,
            "valid": state is not None,
            "fault": bool(self.core.fault),
            "reason": self.core.reason,
            "received_at_unix": now,
            "sequence": self.sequence,
            "imu_received": self.core.imu_received,
            "imu_rejected": self.core.rejected,
            "last_error": self.last_error,
        }
        if state:
            value.update(
                stamp_ns=str(round(state.stamp * 1e9)),
                source_stamp_ns=str(round(state.source_stamp * 1e9)),
                imu_stamp_ns=str(round(state.imu_stamp * 1e9)),
                frame=self.core.odom,
                child_frame=self.core.tracking,
                position=state.pose.position,
                orientation=state.pose.orientation,
                world_velocity=state.world_velocity,
                linear=state.linear,
                angular=state.angular,
                pose_covariance=state.pose_covariance,
                twist_covariance=state.twist_covariance,
                propagation_age_sec=state.stamp - state.source_stamp,
                extrapolation_sec=state.extrapolation,
                covariance_model="conservative_posterior_propagation_not_independent_sensor",
            )
        message = String()
        message.data = json.dumps(value, ensure_ascii=False, allow_nan=False)
        self.pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = LioPredictor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        rclpy.try_shutdown()
