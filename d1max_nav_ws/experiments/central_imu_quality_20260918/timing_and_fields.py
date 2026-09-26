#!/usr/bin/env python3
"""Offline quality checks on lossless IMU extracts; never starts a ROS node."""
import json
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parent


def stats(x):
    return dict(zip(('min', 'p01', 'p50', 'p95', 'p99', 'max'),
                    np.quantile(x, [0, .01, .5, .95, .99, 1]).tolist()))


def max_equal_run(x):
    same = np.all(x[1:] == x[:-1], axis=1)
    edges = np.flatnonzero(np.r_[True, ~same, True])
    return {'consecutive_equal_pairs': int(same.sum()),
            'longest_constant_samples': int(np.diff(edges).max())}


def analyze(name):
    d = np.load(ROOT / f'{name}.npz')
    stamp, record = d['stamp_ns'], d['record_ns']
    acc, gyro, quat = d['accel'], d['gyro'], d['orientation']
    dt = np.diff(stamp) / 1e6
    dr = np.diff(record) / 1e6
    elapsed = (stamp - stamp[0]) * 1e-9
    # Clock epochs differ. Relative offset variation is NOT one-way latency.
    offset = (record - stamp) * 1e-9
    relative_offset = (offset - np.median(offset)) * 1000
    qnorm = np.linalg.norm(quat, axis=1)
    large_idx = np.flatnonzero(dt > 50) + 1
    small_idx = np.flatnonzero(dt < 1) + 1
    per_minute = []
    for begin in range(0, int(elapsed[-1]), 60):
        end = min(begin + 60, float(elapsed[-1]))
        mask = (elapsed >= begin) & (elapsed < end)
        per_minute.append({'begin_s': begin, 'duration_s': end-begin,
                           'count': int(mask.sum()), 'count_per_window_hz': float(mask.sum()/(end-begin)),
                           'gaps_over_50ms': int(((elapsed[large_idx] >= begin) & (elapsed[large_idx] < end)).sum())})
    return {
        'count': len(stamp), 'span_s': float(elapsed[-1]),
        'effective_rate_hz': float((len(stamp)-1)/elapsed[-1]),
        'median_cadence_hz': float(1000/np.median(dt)),
        'header_interval_ms': stats(dt), 'record_interval_ms': stats(dr),
        'interval_counts': {label: int(mask.sum()) for label, mask in {
            'nonpositive': dt <= 0, 'under_1ms': dt < 1,
            'within_4_5_to_5_5ms': (dt >= 4.5) & (dt <= 5.5),
            'over_7_5ms': dt > 7.5, 'over_15ms': dt > 15,
            'over_30ms': dt > 30, 'over_50ms': dt > 50, 'over_100ms': dt > 100}.items()},
        'finite_rows': {key: int(np.isfinite(d[key]).all(axis=1).sum()) for key in ('accel','gyro','orientation')},
        'zero_vector_rows': {'accel': int(np.all(acc == 0, axis=1).sum()), 'gyro': int(np.all(gyro == 0, axis=1).sum())},
        'accel_same_value': max_equal_run(acc), 'gyro_same_value': max_equal_run(gyro),
        'accel_norm_raw': stats(np.linalg.norm(acc, axis=1)),
        'gyro_norm_raw': stats(np.linalg.norm(gyro, axis=1)),
        'orientation_norm': stats(qnorm),
        'orientation_zero_rows': int((qnorm == 0).sum()),
        'orientation_unit_rows_1e_3': int((np.abs(qnorm - 1) < .001).sum()),
        'record_minus_header_median_s_not_latency': float(np.median(offset)),
        'relative_record_minus_header_ms_not_latency': stats(relative_offset),
        'offset_last10s_minus_first10s_ms': float((np.median(offset[elapsed > elapsed[-1]-10]) - np.median(offset[elapsed < 10]))*1000),
        'per_minute': per_minute,
        'gaps_over_50ms': [{'elapsed_s': float(elapsed[i]), 'header_gap_ms': float(dt[i-1]), 'record_gap_ms': float(dr[i-1])} for i in large_idx],
        'under_1ms_examples': [{'elapsed_s': float(elapsed[i]), 'header_gap_ms': float(dt[i-1]), 'record_gap_ms': float(dr[i-1])} for i in small_idx[:10]],
    }


def main():
    report = {'input': 'lossless decoded extracts of slam_raw_20260917_171716_fe8f38',
              'notes': ['Rates use sensor header timestamps; recorded intervals checked independently.',
                        'No sample sequence number: gaps cannot be assigned to sensor, transport, or recorder.',
                        'Same values can reflect quantization and do not alone prove stale samples.',
                        'Unknown clock offset prohibits interpreting recorder-header as physical latency.'],
              'streams': {name: analyze(name) for name in ('central', 'front', 'rear')}}
    with (ROOT / 'timing_and_fields.json').open('x') as f:
        json.dump(report, f, ensure_ascii=False, indent=2)
    print(json.dumps({name: {k: v for k, v in info.items() if k not in ('per_minute','gaps_over_50ms','under_1ms_examples')}
                      for name, info in report['streams'].items()}, indent=2))


if __name__ == '__main__':
    main()
