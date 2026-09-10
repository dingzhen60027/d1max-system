#!/usr/bin/env python3
import argparse
import csv
from pathlib import Path

import numpy as np
import open3d as o3d
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMsg
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


class PctVisualizer(Node):
    def __init__(self, pcd_path, path_csv, frame):
        super().__init__('pct_visualizer')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.cloud_pub = self.create_publisher(type(point_cloud2.create_cloud_xyz32(
            Header(), [])), '/pct/global_map', qos)
        self.path_pub = self.create_publisher(PathMsg, '/pct/global_path', qos)
        cloud = o3d.io.read_point_cloud(str(pcd_path))
        xyz = np.asarray(cloud.points, dtype=np.float32)
        header = Header(frame_id=frame)
        header.stamp = self.get_clock().now().to_msg()
        self.cloud_msg = point_cloud2.create_cloud_xyz32(header, xyz)
        self.path_msg = PathMsg()
        self.path_msg.header.frame_id = frame
        with open(path_csv, newline='', encoding='utf-8') as stream:
            for row in csv.DictReader(stream):
                pose = PoseStamped()
                pose.header.frame_id = frame
                pose.pose.position.x = float(row['x'])
                pose.pose.position.y = float(row['y'])
                pose.pose.position.z = float(row['z'])
                pose.pose.orientation.w = 1.0
                self.path_msg.poses.append(pose)
        self.timer = self.create_timer(1.0, self.publish_all)
        self.publish_all()

    def publish_all(self):
        now = self.get_clock().now().to_msg()
        self.cloud_msg.header.stamp = now
        self.path_msg.header.stamp = now
        for pose in self.path_msg.poses:
            pose.header.stamp = now
        self.cloud_pub.publish(self.cloud_msg)
        self.path_pub.publish(self.path_msg)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--pcd', required=True, type=Path)
    parser.add_argument('--path', required=True, type=Path)
    parser.add_argument('--frame', default='map')
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = PctVisualizer(args.pcd, args.path, args.frame)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
