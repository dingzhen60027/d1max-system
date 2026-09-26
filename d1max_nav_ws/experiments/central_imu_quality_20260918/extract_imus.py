#!/usr/bin/env python3
"""Extract only three recorded IMU streams; never initialize or publish ROS."""
import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sqlite3

import numpy as np
import yaml
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu


TOPICS = {
    '/imu_driver/imu_central': 'central',
    '/front_lidar/imu': 'front',
    '/rear_lidar/imu': 'rear',
}
COV_FIELDS = ('linear_acceleration_covariance', 'angular_velocity_covariance',
              'orientation_covariance')


def json_float(value):
    value = float(value)
    if np.isnan(value):
        return 'NaN'
    if np.isposinf(value):
        return '+Infinity'
    if np.isneginf(value):
        return '-Infinity'
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    bag = args.bag.resolve(strict=True)
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    for name in ('central.npz', 'front.npz', 'rear.npz', 'fields.json'):
        if (output / name).exists():
            raise FileExistsError(output / name)
    meta_raw = (bag / 'metadata.yaml').read_bytes()
    meta = yaml.safe_load(meta_raw)['rosbag2_bagfile_information']
    streams = {topic: {'stamp_ns': [], 'record_ns': [], 'accel': [], 'gyro': [],
                       'orientation': [], 'frame_id': [],
                       'covariance_counts': {field: Counter() for field in COV_FIELDS},
                       'invalid_stamp_fields': 0, 'nonfinite_covariance_messages': 0}
               for topic in TOPICS}
    report = {
        'bag': str(bag), 'created_utc': datetime.now(timezone.utc).isoformat(),
        'read_only': True,
        'method': 'SQLite mode=ro and PRAGMA query_only; deserialize_message only; no ROS initialization, replay, or publishing.',
        'metadata_sha256': hashlib.sha256(meta_raw).hexdigest(),
        'extractor_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'metadata_start_ns': int(meta['starting_time']['nanoseconds_since_epoch']),
        'metadata_duration_ns': int(meta['duration']['nanoseconds']),
        'timestamp_precision': 'stamp_ns and record_ns constructed separately as integer nanoseconds, stored as np.int64; never converted through floating point.',
        'array_fields': {'stamp_ns': 'int64 ROS header timestamp in ns',
                         'record_ns': 'int64 rosbag record timestamp in ns',
                         'accel': 'float64 N x 3 [x,y,z], original message values',
                         'gyro': 'float64 N x 3 [x,y,z], original message values',
                         'orientation': 'float64 N x 4 [x,y,z,w], original message values',
                         'frame_id': 'Unicode N, original message header frame_id'},
        'covariance_patterns': 'Exact little-endian float64 bytes distinguish patterns; values are original message arrays in row-major order. Nonfinite values represented as strings in JSON.',
        'invalid_definition': 'Invalid numeric sample means at least one nonfinite accel, gyro, orientation or covariance component, or invalid ROS stamp nanosecond field. Zero quaternion separately counted and is not automatically interpreted as sensor failure.',
        'source_files': [], 'topics': {},
    }
    for relative in meta['relative_file_paths']:
        path = (bag / relative).resolve(strict=True)
        if path.parent != bag:
            raise ValueError('Unexpected database outside source bag')
        before = path.stat()
        source = {'path': str(path), 'size_bytes': before.st_size,
                  'mtime_ns': before.st_mtime_ns, 'topic_counts': {}}
        with sqlite3.connect(path.as_uri() + '?mode=ro', uri=True) as db:
            db.execute('PRAGMA query_only=ON')
            topics = {name: (idx, typ) for idx, name, typ in db.execute('SELECT id,name,type FROM topics')}
            for topic in TOPICS:
                idx, typ = topics[topic]
                if typ != 'sensor_msgs/msg/Imu':
                    raise ValueError(f'Unexpected type {topic}: {typ}')
                state = streams[topic]
                count = 0
                for record_ns, data in db.execute('SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp,id', (idx,)):
                    msg = deserialize_message(data, Imu)
                    stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
                    state['stamp_ns'].append(stamp_ns)
                    state['record_ns'].append(int(record_ns))
                    a, g, q = msg.linear_acceleration, msg.angular_velocity, msg.orientation
                    state['accel'].append((a.x, a.y, a.z))
                    state['gyro'].append((g.x, g.y, g.z))
                    state['orientation'].append((q.x, q.y, q.z, q.w))
                    state['frame_id'].append(msg.header.frame_id)
                    state['invalid_stamp_fields'] += not (0 <= msg.header.stamp.nanosec < 1_000_000_000)
                    nonfinite_cov = False
                    for field in COV_FIELDS:
                        covariance = np.asarray(getattr(msg, field), dtype='<f8')
                        state['covariance_counts'][field][covariance.tobytes()] += 1
                        nonfinite_cov |= not bool(np.isfinite(covariance).all())
                    state['nonfinite_covariance_messages'] += nonfinite_cov
                    count += 1
                source['topic_counts'][topic] = count
                print(f'{path.name} {topic}: {count}', flush=True)
        after = path.stat()
        if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
            raise RuntimeError(f'Source changed during read: {path}')
        report['source_files'].append(source)
    for topic, stem in TOPICS.items():
        state = streams[topic]
        stamps = np.asarray(state['stamp_ns'], dtype=np.int64)
        records = np.asarray(state['record_ns'], dtype=np.int64)
        order = np.argsort(records, kind='stable')
        arrays = {'stamp_ns': stamps[order], 'record_ns': records[order],
                  'accel': np.asarray(state['accel'], dtype=np.float64)[order],
                  'gyro': np.asarray(state['gyro'], dtype=np.float64)[order],
                  'orientation': np.asarray(state['orientation'], dtype=np.float64)[order],
                  'frame_id': np.asarray(state['frame_id'], dtype=np.str_)[order]}
        numeric_invalid = np.zeros(len(stamps), dtype=bool)
        value_counts = {}
        for field in ('accel', 'gyro', 'orientation'):
            values = arrays[field]
            nonfinite = ~np.isfinite(values)
            numeric_invalid |= nonfinite.any(axis=1)
            value_counts[field] = {'nan_elements': int(np.isnan(values).sum()),
                                   'inf_elements': int(np.isinf(values).sum()),
                                   'nonfinite_elements': int(nonfinite.sum()),
                                   'nonfinite_messages': int(nonfinite.any(axis=1).sum())}
        details = {
            'file': stem + '.npz', 'count': len(stamps),
            'ordering': 'Stable ascending rosbag record_ns, retaining source-file and SQLite message-id order for ties.',
            'frame_id_counts': dict(Counter(state['frame_id'])),
            'stamp_first_ns': int(arrays['stamp_ns'][0]),
            'stamp_last_ns': int(arrays['stamp_ns'][-1]),
            'stamp_min_ns': int(stamps.min()), 'stamp_max_ns': int(stamps.max()),
            'stamp_span_ns': int(stamps.max()) - int(stamps.min()),
            'record_first_ns': int(arrays['record_ns'][0]),
            'record_last_ns': int(arrays['record_ns'][-1]),
            'record_span_ns': int(records.max()) - int(records.min()),
            'invalid_stamp_fields': int(state['invalid_stamp_fields']),
            'nonfinite_value_messages': int(numeric_invalid.sum()),
            'nonfinite_covariance_messages': int(state['nonfinite_covariance_messages']),
            'numeric_counts': value_counts,
            'zero_quaternion_messages': int((arrays['orientation'] == 0).all(axis=1).sum()),
            'covariances': {},
        }
        for field, counts in state['covariance_counts'].items():
            details['covariances'][field] = [
                {'count': count, 'values': [json_float(v) for v in np.frombuffer(raw, dtype='<f8')],
                 'float64_little_endian_hex': raw.hex()}
                for raw, count in counts.items()]
        with (output / (stem + '.npz')).open('xb') as handle:
            np.savez_compressed(handle, **arrays)
        with np.load(output / (stem + '.npz'), allow_pickle=False) as saved:
            for field, values in arrays.items():
                if saved[field].dtype != values.dtype or not np.array_equal(saved[field], values, equal_nan=values.dtype.kind == 'f'):
                    raise RuntimeError(f'Output verification failed: {topic} {field}')
        report['topics'][topic] = details
        print(f'{stem}.npz verified: {len(stamps)} samples, header duration {details["stamp_span_ns"] * 1e-9:.9f} s, record duration {details["record_span_ns"] * 1e-9:.9f} s', flush=True)
    with (output / 'fields.json').open('x') as handle:
        json.dump(report, handle, indent=2, allow_nan=False)
        handle.write('\n')
    print(f'Ready: {output}', flush=True)


if __name__ == '__main__':
    main()
