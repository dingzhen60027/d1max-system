#!/usr/bin/env python3
"""Select a paired LiDAR subset without changing point timestamps or geometry.

The dual adapter still decides scan pairing in BOTH arms of an experiment.
This node never synthesizes a scan, point, IMU sample or motion compensation.
"""
import argparse
from array import array
from collections import Counter
import csv
import json
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


def select_cloud(source, selection):
    if selection not in ('dual', 'paired_front', 'paired_rear'):
        raise ValueError('Unknown paired-cloud selection')
    if source.height != 1 or source.point_step <= 0 or source.row_step != source.width * source.point_step:
        raise ValueError('Expected packed unorganized adapter cloud')
    if len(source.data) != source.row_step:
        raise ValueError('Cloud byte count does not match dimensions')
    ring = next((f for f in source.fields if f.name == 'ring'), None)
    if ring is None or ring.datatype != PointField.UINT16 or ring.count != 1 or ring.offset + 2 > source.point_step:
        raise ValueError('Expected valid UINT16 ring field')
    rings = np.ndarray((source.width,), dtype='>u2' if source.is_bigendian else '<u2',
                       buffer=source.data, offset=ring.offset, strides=(source.point_step,))
    if np.any(rings >= 192):
        raise ValueError('Expected front rings [0,96) and rear rings [96,192)')
    mask = np.ones(source.width, dtype=bool) if selection == 'dual' else (
        rings < 96 if selection == 'paired_front' else rings >= 96)
    rows = np.frombuffer(source.data, dtype=np.uint8).reshape(source.width, source.point_step)
    output = PointCloud2()
    output.header = source.header
    output.height, output.width = 1, int(np.count_nonzero(mask))
    output.fields = source.fields
    output.is_bigendian, output.is_dense = source.is_bigendian, source.is_dense
    output.point_step = source.point_step
    output.row_step = output.point_step * output.width
    # The generated ROS setter validates bytes element-by-element in Python.
    # A typed buffer avoids millions of Python iterations per scan.
    output.data = array('B', rows[mask].tobytes())
    return output


class Selector(Node):
    def __init__(self, args):
        super().__init__('paired_cloud_subset')
        self.args, self.counts, self.first_ns, self.last_ns = args, Counter(), None, None
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(PointCloud2, '/d1max/slam/selected_points', qos)
        self.subscription = self.create_subscription(PointCloud2, '/d1max/slam/points', self.callback, qos)
        self.audit_stream = args.audit_file.open('x', buffering=1)
        self.writer = csv.writer(self.audit_stream)
        self.writer.writerow(['stamp_ns', 'input_points', 'output_points', 'admitted'])

    def callback(self, source):
        stamp = source.header.stamp.sec * 1_000_000_000 + source.header.stamp.nanosec
        if self.last_ns is not None and stamp <= self.last_ns:
            raise ValueError('Paired scan timestamps must be strictly increasing')
        self.last_ns = stamp
        if self.first_ns is None:
            self.first_ns = stamp
        self.counts['received'] += 1
        if self.args.window > 0 and stamp - self.first_ns >= round(self.args.window * 1e9):
            self.counts['outside_window'] += 1
            self.writer.writerow([stamp, source.width, 0, False])
            return
        output = select_cloud(source, self.args.selection)
        if output.width == 0:
            raise ValueError('Empty paired subset; refusing an invalid comparison')
        self.publisher.publish(output)
        self.counts['published'] += 1
        self.counts['published_points'] += output.width
        self.writer.writerow([stamp, source.width, output.width, True])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--selection', choices=['dual', 'paired_front', 'paired_rear'], required=True)
    parser.add_argument('--window', type=float, default=0.)
    parser.add_argument('--audit-file', type=Path, required=True)
    args = parser.parse_args()
    if not np.isfinite(args.window) or args.window < 0:
        raise ValueError('Window must be finite and nonnegative')
    rclpy.init(args=[])
    node = Selector(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.audit_stream.close()
        print(json.dumps(dict(selection=args.selection, counts=dict(node.counts),
                              first_stamp_ns=node.first_ns, last_stamp_ns=node.last_ns)), flush=True)
        context = node.context
        node.destroy_node()
        try:
            rclpy.try_shutdown(context=context)
        except _rclpy.RCLError as error:
            if 'rcl_shutdown already called on the given context' not in str(error) or context.ok():
                raise


if __name__ == '__main__':
    main()
