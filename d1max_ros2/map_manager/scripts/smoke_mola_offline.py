#!/usr/bin/env python3
"""Generate a tiny synthetic rosbag and run native MOLA. No hardware/live ROS graph."""
import json
from pathlib import Path
import struct
import tempfile
import yaml
import rosbag2_py
from rclpy.serialization import serialize_message
from sensor_msgs.msg import PointCloud2, PointField, Imu
from backend.mapping.configuration import MolaConfig, config_yaml
from backend.mapping.mola_runtime import atomic_json, now
from backend.mapping.worker import Worker


def main():
    root = Path(tempfile.mkdtemp(prefix='d1max-mola-smoke-'))
    bag = root / 'bag'
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    for topic, kind in (('/points', 'sensor_msgs/msg/PointCloud2'), ('/imu', 'sensor_msgs/msg/Imu')):
        writer.create_topic(rosbag2_py.TopicMetadata(name=topic, type=kind, serialization_format='cdr'))
    points = []
    for a in range(-25, 26):
        for b in range(-8, 17):
            points.extend([(6., a * .2, b * .2), (a * .2, 6., b * .2), (-6., a * .2, b * .2)])
    for a in range(-25, 26):
        for b in range(-25, 26):
            points.append((a * .2, b * .2, -1.6))
    epoch = 1700000000000000000
    for tick in range(1400):
        stamp = epoch + tick * 5000000
        imu = Imu(); imu.header.frame_id = 'sensor'
        imu.header.stamp.sec, imu.header.stamp.nanosec = divmod(stamp, 1000000000)
        imu.linear_acceleration.z = 9.80665
        imu.orientation_covariance[0] = -1.
        writer.write('/imu', serialize_message(imu), stamp)
        if tick % 20 == 19:
            cloud = PointCloud2(); cloud.header = imu.header
            cloud.height = 1; cloud.width = len(points); cloud.is_dense = True
            cloud.fields = [PointField(name=n, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                            for i, n in enumerate(('x', 'y', 'z', 'intensity', 'time'))]
            cloud.point_step = 20; cloud.row_step = cloud.width * 20
            cloud.data = b''.join(struct.pack('<fffff', x - tick * .002, y, z, 30., -.095 + i / len(points) * .095)
                                  for i, (x, y, z) in enumerate(points))
            writer.write('/points', serialize_message(cloud), stamp)
    del writer
    session = root / 'session'; session.mkdir()
    config = MolaConfig(input={'bag_path': str(bag), 'lidar_topic': '/points', 'imu_topic': '/imu', 'base_frame': 'sensor'},
                        sensors={'source': 'fixed'}, execution={'stage_timeout_sec': 90, 'threads': 2})
    (session / 'task.yaml').write_text(config_yaml(config))
    atomic_json(session / 'manifest.json', {'id': 'synthetic-smoke', 'started_at': now(), 'artifacts': []})
    result = Worker(session, Path(__file__).resolve().parents[1]).run()
    print('SMOKE_SESSION=' + str(session))
    print((session / 'manifest.json').read_text())
    raise SystemExit(result)


if __name__ == '__main__':
    main()
