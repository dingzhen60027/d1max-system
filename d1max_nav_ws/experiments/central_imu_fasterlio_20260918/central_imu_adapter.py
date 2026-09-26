#!/usr/bin/env python3
"""Experiment-only selected raw IMU normalization; no robot SDK or control output.

Vectors stay in the recorded source IMU axes. LiDAR->IMU R/t belongs to
Faster-LIO's extrinsic parameters, not a second vector rotation here.
"""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import time

import yaml
import rclpy
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu


def load_config(path):
    config = yaml.safe_load(Path(path).read_text())
    for key in ('input_topic', 'output_topic', 'frame_id'):
        if not isinstance(config.get(key), str) or not config[key]:
            raise ValueError(f'Missing {key}')
    if config['input_topic'] == config['output_topic']:
        raise ValueError('IMU input/output must be distinct')
    for key in ('acceleration_scale', 'gyro_scale'):
        value = float(config[key])
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f'Invalid {key}')
    offset = float(config['timestamp_offset_sec'])
    if not math.isfinite(offset) or abs(offset) > .1:
        raise ValueError('Experiment timestamp shift must be within 100 ms')
    if config.get('duplicate_policy', 'preserve_and_count') != 'preserve_and_count':
        raise ValueError('This adapter does not silently delete or fabricate samples')
    return config


def normalize_message(source, config):
    a, g = source.linear_acceleration, source.angular_velocity
    if not all(math.isfinite(x) for x in (a.x, a.y, a.z, g.x, g.y, g.z)):
        raise ValueError('Nonfinite six-axis sample')
    offset_ns = round(float(config['timestamp_offset_sec']) * 1e9)
    stamp_ns = source.header.stamp.sec * 1_000_000_000 + source.header.stamp.nanosec + offset_ns
    if stamp_ns <= 0 or source.header.stamp.nanosec >= 1_000_000_000:
        raise ValueError('Invalid input timestamp')
    result = Imu()
    result.header.stamp.sec, result.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
    result.header.frame_id = config['frame_id']
    for axis in ('x', 'y', 'z'):
        setattr(result.linear_acceleration, axis, getattr(a, axis) * config['acceleration_scale'])
        setattr(result.angular_velocity, axis, getattr(g, axis) * config['gyro_scale'])
    for name, scale in (('linear_acceleration_covariance', config['acceleration_scale']),
                        ('angular_velocity_covariance', config['gyro_scale'])):
        cov = list(getattr(source, name))
        if cov[0] >= 0:
            cov = [x * scale * scale for x in cov]
        setattr(result, name, cov)
    # Bag has a constant identity quaternion, NOT a measured body attitude.
    result.orientation.w = 1.0
    result.orientation_covariance[0] = -1.0
    return result, stamp_ns


class CentralImuAdapter(Node):
    def __init__(self, config):
        super().__init__('central_imu_experiment_adapter')
        self.config = config
        self.stats = Counter()
        self.frames = Counter()
        self.first_ns = None
        self.last_ns = None
        self.last_values = None
        self.max_gap_ns = 0
        self.started = time.monotonic()
        qos = QoSProfile(depth=2000, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.pub = self.create_publisher(Imu, config['output_topic'], qos)
        self.sub = self.create_subscription(Imu, config['input_topic'], self.callback, qos)
        self.get_logger().info('Raw IMU axes retained from %s as %s; acceleration scale=%g, gyro scale=%g, header shift=%g s; duplicates preserved, orientation disabled' %
                               (config['input_topic'], config['frame_id'], config['acceleration_scale'], config['gyro_scale'], config['timestamp_offset_sec']))

    def callback(self, msg):
        self.stats['received'] += 1
        self.frames[msg.header.frame_id] += 1
        try:
            output, stamp = normalize_message(msg, self.config)
        except ValueError:
            self.stats['invalid_rejected'] += 1
            return
        if self.last_ns is not None and stamp <= self.last_ns:
            self.stats['nonmonotonic_rejected'] += 1
            return
        values = tuple(getattr(msg.linear_acceleration, axis) for axis in ('x','y','z')) + tuple(getattr(msg.angular_velocity, axis) for axis in ('x','y','z'))
        if self.last_values == values:
            self.stats['six_axis_repeated_pairs'] += 1
        if self.last_ns is not None:
            gap = stamp - self.last_ns
            self.max_gap_ns = max(gap, self.max_gap_ns)
            if gap > 30_000_000:
                self.stats['gaps_over_30ms'] += 1
            if gap > 50_000_000:
                self.stats['gaps_over_50ms'] += 1
        if self.first_ns is None:
            self.first_ns = stamp
        self.last_ns = stamp
        self.last_values = values
        self.pub.publish(output)
        self.stats['published'] += 1

    def audit(self):
        return {'config': self.config, 'counts': dict(self.stats), 'input_frames': dict(self.frames),
                'first_output_stamp_ns': self.first_ns, 'last_output_stamp_ns': self.last_ns,
                'maximum_header_gap_ms': self.max_gap_ns * 1e-6,
                'elapsed_wall_sec': time.monotonic() - self.started,
                'notes': ['No frame rotation applied to IMU vectors; Faster-LIO owns LiDAR-to-selected-IMU extrinsics.',
                          'Repeated values preserved, not classified as hardware duplicates or synthesized.',
                          'A time shift inferred from relative signal phase is an experimental assumption.']}


def cleanup_ros_node(node):
    """Tolerate only Humble's confirmed concurrent already-shutdown race."""
    context = node.context
    try:
        node.destroy_node()
    finally:
        try:
            rclpy.try_shutdown(context=context)
        except _rclpy.RCLError as error:
            # Humble checks ok() before calling the C shutdown function. A
            # signal can close this same context between those two operations.
            # Keep unexpected shutdown failures visible, including a context
            # that is still active despite an "already called" error message.
            if ('rcl_shutdown already called on the given context' not in str(error)
                    or context.ok()):
                raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--audit-file', type=Path)
    args = parser.parse_args()
    config = load_config(args.config)
    rclpy.init(args=[])
    node = CentralImuAdapter(config)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        audit = node.audit()
        if args.audit_file:
            # Fresh, run-owned artifact, written once on controlled shutdown.
            with args.audit_file.open('x') as f:
                json.dump(audit, f, ensure_ascii=False, indent=2)
        print(json.dumps(audit, ensure_ascii=False), flush=True)
        cleanup_ros_node(node)


if __name__ == '__main__':
    main()
