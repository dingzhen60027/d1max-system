#!/usr/bin/env python3
"""Read-only source IMU timestamp audit; never starts ROS or publishes data.

Source the ROS environment, then run:
  python3 audit_bag_imu.py /path/to/finalized_bag
Results are written only to stdout. SQLite connections use URI mode=ro.
"""
import argparse
import json
from pathlib import Path
import sqlite3

import yaml
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu


THRESHOLDS_MS = (15, 30, 50, 100)


def percentile(values, fraction):
    if not values:
        return None
    index = (len(values) - 1) * fraction
    lo = int(index)
    hi = min(lo + 1, len(values) - 1)
    return values[lo] + (values[hi] - values[lo]) * (index - lo)


def audit(bag, topic):
    metadata = yaml.safe_load((bag / 'metadata.yaml').read_text())
    info = metadata['rosbag2_bagfile_information']
    if info['storage_identifier'] != 'sqlite3':
        raise ValueError('This audit reads finalized sqlite3 rosbag storage only')
    source_files = []
    first = previous = None
    count = 0
    deltas_ns = []
    nonpositive = []
    gaps = []
    counts = {str(value): 0 for value in THRESHOLDS_MS}
    first30 = dict(counts)
    for relative in info['relative_file_paths']:
        path = (bag / relative).resolve(strict=True)
        if path.parent != bag:
            raise ValueError('Metadata database path must stay inside this bag')
        source_files.append(str(path))
        connection = sqlite3.connect(path.as_uri() + '?mode=ro', uri=True)
        try:
            row = connection.execute(
                'SELECT id, type FROM topics WHERE name=?', (topic,)).fetchone()
            if row is None:
                continue
            if row[1] != 'sensor_msgs/msg/Imu':
                raise ValueError(f'{topic} is not sensor_msgs/msg/Imu')
            rows = connection.execute(
                'SELECT timestamp, data FROM messages WHERE topic_id=? '
                'ORDER BY timestamp', (row[0],))
            for received_ns, data in rows:
                message = deserialize_message(data, Imu)
                stamp_ns = (message.header.stamp.sec * 1_000_000_000
                            + message.header.stamp.nanosec)
                count += 1
                if first is None:
                    first = stamp_ns
                if previous is not None:
                    delta_ns = stamp_ns - previous[0]
                    elapsed = (stamp_ns - first) * 1e-9
                    deltas_ns.append(delta_ns)
                    for threshold in THRESHOLDS_MS:
                        if delta_ns > threshold * 1_000_000:
                            counts[str(threshold)] += 1
                            if elapsed <= 30.0:
                                first30[str(threshold)] += 1
                    if delta_ns > 50_000_000 or delta_ns <= 0:
                        result = {
                            'previous_header_ns': previous[0],
                            'header_ns': stamp_ns,
                            'gap_ns': delta_ns,
                            'gap_ms': delta_ns * 1e-6,
                            'elapsed_from_first_imu_s': elapsed,
                            'record_timestamp_ns': received_ns,
                            'record_interval_ms': (received_ns - previous[1]) * 1e-6,
                        }
                        (gaps if delta_ns > 0 else nonpositive).append(result)
                previous = stamp_ns, received_ns
        finally:
            connection.close()
    if first is None or not deltas_ns:
        raise ValueError('Need at least two recorded IMU samples')
    ordered = sorted(deltas_ns)
    return {
        'schema': 1,
        'read_only': True,
        'bag': str(bag),
        'topic': topic,
        'source_files': source_files,
        'message_count': count,
        'first_header_ns': first,
        'last_header_ns': previous[0],
        'source_span_s': (previous[0] - first) * 1e-9,
        'threshold_rule': 'strictly greater than threshold; integer header nanoseconds',
        'gap_count_over_ms': counts,
        'first_30s_gap_count_over_ms': first30,
        'interval_ms': {
            name: percentile(ordered, fraction) * 1e-6
            for name, fraction in [('min', 0), ('median', .5), ('p95', .95),
                                   ('p99', .99), ('p999', .999), ('max', 1)]
        },
        'nonpositive_interval_count': len(nonpositive),
        'nonpositive_examples': nonpositive[:10],
        'first_30s_gaps_over_50ms': [
            gap for gap in gaps if gap['elapsed_from_first_imu_s'] <= 30.0],
        'largest_gaps': sorted(gaps, key=lambda item: item['gap_ns'], reverse=True)[:10],
        'interpretation': (
            'These gaps already exist in the recorded IMU headers. This alone '
            'cannot distinguish device-side loss from transport/recorder loss. '
            'It is not an IMU intrinsic noise or localization accuracy test.'),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--topic', default='/front_lidar/imu')
    args = parser.parse_args()
    print(json.dumps(audit(args.bag.resolve(strict=True), args.topic), indent=2))


if __name__ == '__main__':
    main()
