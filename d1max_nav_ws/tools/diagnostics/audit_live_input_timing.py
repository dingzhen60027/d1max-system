#!/usr/bin/env python3
"""Bounded read-only timing probe. No publishers, parameter writes or robot calls."""
import argparse
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--seconds', type=float, default=15.)
    p.add_argument('--session', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--include-samples', action='store_true')
    args = p.parse_args()
    if not 1 <= args.seconds <= 30:
        raise ValueError('bounded probe: 1..30 seconds')
    initial = json.loads((args.session/'status.json').read_text())
    initial_status_fresh = 0 <= time.time() - float(initial.get('wall_time', 0)) < 2.
    offset = float(initial['input_clock']['offset_seconds'])
    rclpy.init(args=[])
    node = Node('d1max_read_only_timing_probe', enable_rosout=False)
    records, endpoints = {}, {}
    streams = [
        ('raw_imu', '/front_lidar/imu', Imu),
        ('raw_imu_reliable', '/front_lidar/imu', Imu),
        ('normalized_imu', '/d1max/localization/imu', Imu),
        ('lio_odom', '/d1max/localization/lio/odometry', Odometry),
        ('global_odom', '/d1max/localization/odometry/global', Odometry),
        ('local_odom', '/d1max/localization/odometry/local', Odometry),
    ]
    for name, topic, cls in streams:
        records[name] = []
        def receive(msg, key=name):
            if len(records[key]) < 25000:
                stamp = msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
                records[key].append((stamp, time.monotonic_ns()))
        node.create_subscription(cls, topic, receive,
            QoSProfile(depth=512, reliability=(ReliabilityPolicy.RELIABLE
                if name == 'raw_imu_reliable' else ReliabilityPolicy.BEST_EFFORT)))
    started = time.monotonic()
    try:
        while time.monotonic()-started < args.seconds:
            rclpy.spin_once(node, timeout_sec=.02)
        for name, topic, _ in streams:
            endpoints[name] = [{'node': e.node_namespace+'/'+e.node_name,
                'reliability': str(e.qos_profile.reliability), 'depth': e.qos_profile.depth}
                for e in node.get_publishers_info_by_topic(topic)]
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    result = {'read_only': True, 'duration_sec': time.monotonic()-started,
              'host_time': time.time(), 'source_offset_sec': offset, 'endpoints': endpoints,
              'streams': {}}
    final = json.loads((args.session/'status.json').read_text())
    metadata_valid = (initial_status_fresh
        and 0 <= time.time() - float(final.get('wall_time', 0)) < 2.
        and final.get('session_id') == initial.get('session_id')
        and abs(float(final['input_clock']['offset_seconds']) - offset) < 1e-6)
    result['normalization_metadata_valid'] = metadata_valid
    for name, samples in records.items():
        x = np.asarray(samples, dtype=np.int64).reshape(-1, 2)
        report = {'count': len(x)}
        if len(x) > 1:
            ds, dr = np.diff(x[:, 0])*1e-6, np.diff(x[:, 1])*1e-6
            report.update(source_hz=(len(x)-1)*1e9/(x[-1, 0]-x[0, 0]),
                receipt_hz=(len(x)-1)*1e9/(x[-1, 1]-x[0, 1]),
                source_gap_ms_p50_p95_max=np.percentile(ds, [50, 95, 100]).tolist(),
                receipt_gap_ms_p50_p95_max=np.percentile(dr, [50, 95, 100]).tolist(),
                source_nonmonotonic=int(np.sum(ds <= 0)),
                source_gaps_over_50ms=int(np.sum(ds > 50.001)),
                source_gap_intervals=[[int(x[i,0]), int(x[i+1,0]), float(ds[i])]
                                      for i in np.flatnonzero(ds > 50.001)[:40]])
        result['streams'][name] = report
    raw = np.sort(np.asarray([s[0] for s in records['raw_imu']], dtype=np.int64))
    normalized = np.sort(np.asarray([s[0] for s in records['normalized_imu']], dtype=np.int64))
    if len(raw) and len(normalized) and metadata_valid:
        shifted = normalized - round(offset*1e9)
        inside = raw[(raw > shifted[0]+1000000) & (raw < shifted[-1]-1000000)]
        indexes = np.searchsorted(shifted, inside)
        distances = np.minimum(abs(inside-shifted[np.clip(indexes,0,len(shifted)-1)]),
                               abs(inside-shifted[np.clip(indexes-1,0,len(shifted)-1)]))
        result['raw_to_normalized'] = {'interior_raw_samples': len(inside),
            'matched_within_2us': int(np.sum(distances <= 2000)),
            'unmatched_source_stamps': inside[distances > 2000][:100].tolist()}
    elif len(raw) and len(normalized):
        result['raw_to_normalized'] = {
            'inconclusive': 'session metadata stale or changed; do not infer adapter loss from old offset'}
    reliable = {s[0] for s in records['raw_imu_reliable']}
    best_effort = {s[0] for s in records['raw_imu']}
    if reliable and best_effort:
        low, high = max(min(reliable), min(best_effort)), min(max(reliable), max(best_effort))
        reliable = {s for s in reliable if low < s < high}
        best_effort = {s for s in best_effort if low < s < high}
        result['raw_qos_comparison'] = {'reliable_count': len(reliable),
            'best_effort_count': len(best_effort),
            'only_reliable': sorted(reliable-best_effort),
            'only_best_effort': sorted(best_effort-reliable)}
    result['frontend_before'] = initial.get('frontend')
    result['frontend_after'] = final.get('frontend')
    if args.include_samples:
        result['samples'] = records
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2))
    brief = {**{k:v for k,v in result.items() if k != 'samples'}, 'streams': {k:{a:b for a,b in v.items() if a != 'source_gap_intervals'}
                                  for k,v in result['streams'].items()}}
    print(json.dumps(brief, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
