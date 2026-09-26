#!/usr/bin/env python3
"""Read-only ROS bag diagnostics. No ROS initialization, publication or replay."""
import argparse
import json
import sqlite3
from pathlib import Path

import numpy as np
import yaml
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu, PointCloud2

IMU_TOPICS = ('/front_lidar/imu', '/rear_lidar/imu', '/imu_driver/imu_central')


def quantiles(values):
    values = np.asarray(values)
    return dict(zip(('min', 'p50', 'p95', 'p99', 'max'),
                    np.quantile(values, [0, .5, .95, .99, 1]).tolist()))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    bag = args.bag.resolve(strict=True)
    meta = yaml.safe_load((bag / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    streams = {topic: [] for topic in IMU_TOPICS}
    cloud_samples = []
    start_ns = meta['starting_time']['nanoseconds_since_epoch']
    span_ns = meta['duration']['nanoseconds']
    requested_records = [start_ns + int(f * span_ns) for f in np.linspace(.005, .995, 12)]
    for relative in meta['relative_file_paths']:
        path = (bag / relative).resolve(strict=True)
        if path.parent != bag:
            raise ValueError('Unexpected database outside bag')
        db = sqlite3.connect(path.as_uri() + '?mode=ro', uri=True)
        topics = {name: (idx, typ) for idx, name, typ in db.execute('SELECT id,name,type FROM topics')}
        for topic in IMU_TOPICS:
            idx, typ = topics[topic]
            assert typ == 'sensor_msgs/msg/Imu'
            for received, data in db.execute('SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp', (idx,)):
                msg = deserialize_message(data, Imu)
                stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
                acc = msg.linear_acceleration
                gyro = msg.angular_velocity
                streams[topic].append((stamp, received, acc.x, acc.y, acc.z, gyro.x, gyro.y, gyro.z))
        bounds = db.execute('SELECT min(timestamp),max(timestamp) FROM messages').fetchone()
        for target in requested_records:
            if not bounds[0] <= target <= bounds[1]:
                continue
            for topic in ('/front_lidar', '/rear_lidar'):
                idx, typ = topics[topic]
                assert typ == 'sensor_msgs/msg/PointCloud2'
                row = db.execute('SELECT timestamp,data FROM messages WHERE topic_id=? AND timestamp>=? ORDER BY timestamp LIMIT 1', (idx, target)).fetchone()
                if row is None:
                    continue
                msg = deserialize_message(row[1], PointCloud2)
                stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                fields = {f.name: f for f in msg.fields}
                sample = {'topic': topic, 'record_elapsed_s': (row[0] - start_ns) * 1e-9,
                          'header_s': stamp, 'frame': msg.header.frame_id,
                          'fields': {f.name: {'offset': f.offset, 'type': f.datatype} for f in msg.fields},
                          'point_count': msg.height * msg.width}
                if 'timestamp' in fields and fields['timestamp'].datatype == 8:
                    dtype = np.dtype({'names': ['t'], 'formats': [('>' if msg.is_bigendian else '<') + 'f8'],
                                      'offsets': [fields['timestamp'].offset], 'itemsize': msg.point_step})
                    times = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                                       strides=(msg.row_step, msg.point_step))['t'].reshape(-1)
                    times = times[np.isfinite(times)]
                    sample.update(point_timestamp_range_s=[float(times.min()), float(times.max())],
                                  scan_span_ms=float((times.max() - times.min()) * 1000),
                                  first_point_minus_header_ms=float((times.min() - stamp) * 1000),
                                  last_point_minus_header_ms=float((times.max() - stamp) * 1000))
                cloud_samples.append(sample)
        db.close()

    report = {'bag': str(bag), 'read_only': True,
              'interpretation': 'Source gaps and units only; neither ground truth nor a complete cause attribution.',
              'imus': {}, 'sampled_clouds': cloud_samples}
    gap_record_times = {}
    for topic, rows in streams.items():
        stamps = np.array([r[0] for r in rows], dtype=np.int64)
        received = np.array([r[1] for r in rows], dtype=np.int64)
        values = np.array([r[2:] for r in rows], dtype=float)
        delta = np.diff(stamps)
        elapsed = (stamps - stamps[0]) * 1e-9
        large = np.flatnonzero(delta > 50_000_000) + 1
        gap_record_times[topic] = (received[large] - start_ns) * 1e-9
        buckets = []
        for begin in range(0, int(elapsed[-1]) + 1, 60):
            mask = (elapsed >= begin) & (elapsed < begin + 60)
            buckets.append({'begin_s': begin, 'count': int(mask.sum()),
                            'gaps_over_50ms': int(((elapsed[large] >= begin) & (elapsed[large] < begin + 60)).sum()),
                            'raw_acc_norm_median': float(np.median(np.linalg.norm(values[mask, :3], axis=1))),
                            'raw_gyro_norm_p95': float(np.quantile(np.linalg.norm(values[mask, 3:], axis=1), .95))})
        report['imus'][topic] = {
            'count': len(rows), 'span_s': float(elapsed[-1]),
            'nonpositive_intervals': int((delta <= 0).sum()),
            'header_interval_ms': quantiles(delta * 1e-6),
            'gaps_over_50ms': len(large),
            'raw_acc_norm': quantiles(np.linalg.norm(values[:, :3], axis=1)),
            'raw_gyro_norm': quantiles(np.linalg.norm(values[:, 3:], axis=1)),
            'first_2s_acc_mean': values[elapsed <= 2, :3].mean(axis=0).tolist(),
            'first_2s_gyro_mean': values[elapsed <= 2, 3:].mean(axis=0).tolist(),
            'per_minute': buckets,
            'gap_events': [{'elapsed_s': float(elapsed[i]), 'record_elapsed_s': float((received[i] - start_ns) * 1e-9),
                            'header_gap_ms': float(delta[i - 1] * 1e-6),
                            'record_gap_ms': float((received[i] - received[i - 1]) * 1e-6)} for i in large]}
    front_gaps = gap_record_times[IMU_TOPICS[0]]
    report['coincident_gap_counts_within_100ms_record_time'] = {
        topic: sum(bool(len(times) and np.min(np.abs(times - t)) <= .1) for t in front_gaps)
        for topic, times in gap_record_times.items() if topic != IMU_TOPICS[0]}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2)
    print(json.dumps({topic: {k: v for k, v in info.items() if k not in ('gap_events', 'per_minute')}
                      for topic, info in report['imus'].items()}, indent=2))
    print('Coincident gaps:', report['coincident_gap_counts_within_100ms_record_time'])
    print('Sampled clouds:', len(cloud_samples), 'Report:', args.output)


if __name__ == '__main__':
    main()
