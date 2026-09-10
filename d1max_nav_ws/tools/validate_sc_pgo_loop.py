#!/usr/bin/env python3
import json
import math
import random
import struct
import sys
import time

import rclpy
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, UInt32


def quaternion_from_yaw(yaw):
    return (0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw))


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def make_environment():
    rng = random.Random(7)
    points = []
    for _ in range(9000):
        wall = rng.randrange(4)
        u = rng.uniform(-4.0, 18.0)
        z = rng.uniform(-0.8, 3.0)
        if wall == 0:
            x, y = -4.0, u
        elif wall == 1:
            x, y = 18.0, u
        elif wall == 2:
            x, y = u, -4.0
        else:
            x, y = u, 18.0
        points.append((x, y, z, 15.0 + wall * 20.0 + z))

    for pillar_index, (cx, cy, radius) in enumerate(
        [(3.0, 5.0, 0.45), (13.0, 3.0, 0.70), (7.0, 14.0, 0.55)]
    ):
        for _ in range(1400):
            angle = rng.uniform(-math.pi, math.pi)
            z = rng.uniform(-0.8, 3.2)
            points.append(
                (
                    cx + radius * math.cos(angle),
                    cy + radius * math.sin(angle),
                    z,
                    100.0 + pillar_index * 25.0 + z,
                )
            )
    return points


def make_square_poses(side=12.0, step=0.5):
    poses = []
    segments = [
        ((0.0, 0.0), (side, 0.0), 0.0),
        ((side, 0.0), (side, side), math.pi / 2.0),
        ((side, side), (0.0, side), math.pi),
        ((0.0, side), (0.0, 0.0), -math.pi / 2.0),
    ]
    count = int(round(side / step))
    for (x0, y0), (x1, y1), yaw in segments:
        for index in range(count):
            alpha = index / float(count)
            poses.append((x0 + alpha * (x1 - x0), y0 + alpha * (y1 - y0), yaw))
    poses.append((0.0, 0.0, 0.0))
    return poses


def cloud_message(points, true_pose, stamp):
    tx, ty, yaw = true_pose
    c = math.cos(yaw)
    s = math.sin(yaw)
    visible = []
    for wx, wy, wz, intensity in points:
        dx = wx - tx
        dy = wy - ty
        bx = c * dx + s * dy
        by = -s * dx + c * dy
        if 0.4 < math.hypot(bx, by) < 28.0:
            visible.append((bx, by, wz, intensity))

    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    data = bytearray(16 * len(visible))
    for index, point in enumerate(visible):
        struct.pack_into("<ffff", data, 16 * index, *point)

    msg = PointCloud2()
    msg.header = Header(stamp=stamp, frame_id="body")
    msg.height = 1
    msg.width = len(visible)
    msg.fields = fields
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = 16 * len(visible)
    msg.data = bytes(data)
    msg.is_dense = True
    return msg


class ClosedLoopValidator(Node):
    def __init__(self):
        super().__init__("sc_pgo_closed_loop_validator")
        self.odom_pub = self.create_publisher(Odometry, "/validation/odom", 100)
        self.cloud_pub = self.create_publisher(PointCloud2, "/validation/cloud", 100)
        self.create_subscription(Path, "/validation/pgo/path", self.path_callback, 10)
        self.create_subscription(UInt32, "/validation/pgo/loop_count", self.loop_callback, 10)
        self.latest_path = None
        self.loop_count = 0

    def path_callback(self, msg):
        self.latest_path = msg

    def loop_callback(self, msg):
        self.loop_count = msg.data

    def wait_for_backend(self, timeout=15.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.odom_pub.get_subscription_count() and self.cloud_pub.get_subscription_count():
                return True
        return False

    def publish_loop(self):
        environment = make_environment()
        poses = make_square_poses()
        for index, true_pose in enumerate(poses):
            alpha = index / float(len(poses) - 1)
            raw_x = true_pose[0] + 2.0 * alpha
            raw_y = true_pose[1] - 1.2 * alpha
            raw_yaw = true_pose[2] + math.radians(8.0) * alpha
            stamp = self.get_clock().now().to_msg()

            odom = Odometry()
            odom.header = Header(stamp=stamp, frame_id="camera_init")
            odom.child_frame_id = "body"
            odom.pose.pose.position.x = raw_x
            odom.pose.pose.position.y = raw_y
            odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, \
                odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = \
                quaternion_from_yaw(raw_yaw)

            self.odom_pub.publish(odom)
            self.cloud_pub.publish(cloud_message(environment, true_pose, stamp))
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.09)
        return math.hypot(2.0, -1.2), math.radians(8.0), len(poses)

    def wait_for_result(self, timeout=25.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.loop_count > 0 and self.latest_path and len(self.latest_path.poses) > 65:
                time.sleep(2.0)
                rclpy.spin_once(self, timeout_sec=0.2)
                return True
        return False


def main():
    rclpy.init()
    node = ClosedLoopValidator()
    try:
        if not node.wait_for_backend():
            print(json.dumps({"passed": False, "reason": "backend subscribers unavailable"}))
            return 2
        raw_error, raw_yaw_error, frames = node.publish_loop()
        node.wait_for_result()

        if not node.latest_path or not node.latest_path.poses:
            print(json.dumps({"passed": False, "reason": "no optimized path"}))
            return 3

        final_pose = node.latest_path.poses[-1].pose
        optimized_error = math.hypot(final_pose.position.x, final_pose.position.y)
        optimized_yaw_error = abs(yaw_from_quaternion(final_pose.orientation))
        passed = (
            node.loop_count > 0
            and optimized_error < raw_error * 0.65
            and optimized_yaw_error < raw_yaw_error * 0.75
        )
        report = {
            "passed": passed,
            "published_frames": frames,
            "optimized_keyframes": len(node.latest_path.poses),
            "accepted_loops": node.loop_count,
            "raw_closure_error_m": round(raw_error, 4),
            "optimized_closure_error_m": round(optimized_error, 4),
            "raw_yaw_error_deg": round(math.degrees(raw_yaw_error), 3),
            "optimized_yaw_error_deg": round(math.degrees(optimized_yaw_error), 3),
        }
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0 if passed else 4
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
