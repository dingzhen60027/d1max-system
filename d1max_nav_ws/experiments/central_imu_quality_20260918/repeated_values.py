#!/usr/bin/env python3
"""Measure unchanged six-axis runs, without assuming why they occurred."""
import json
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parent


def analyze(name):
    d = np.load(ROOT / f'{name}.npz')
    stamp = d['stamp_ns']
    values = np.c_[d['accel'], d['gyro']]
    equal = np.all(values[1:] == values[:-1], axis=1)
    starts = np.r_[0, np.flatnonzero(~equal)+1]
    ends = np.r_[starts[1:] - 1, len(stamp)-1]
    lengths = ends-starts+1
    holds = (stamp[ends]-stamp[starts])*1e-6
    next_change = np.minimum(ends+1, len(stamp)-1)
    update_intervals = np.diff(stamp[starts])*1e-6
    elapsed = (stamp-stamp[0])*1e-9
    top = np.argsort(holds)[-10:][::-1]
    return {
        'count': len(stamp), 'unchanged_six_axis_pairs': int(equal.sum()),
        'unchanged_pair_percent': float(equal.mean()*100),
        'distinct_run_count': len(starts),
        'value_change_rate_hz_not_hardware_rate': float((len(starts)-1)/elapsed[-1]),
        'longest_run_samples': int(lengths.max()),
        'longest_same_value_stamp_span_ms': float(holds.max()),
        'value_change_interval_ms': dict(zip(('min','p50','p95','p99','max'), np.quantile(update_intervals,[0,.5,.95,.99,1]).tolist())),
        'constant_orientation': bool(np.all(d['orientation'] == d['orientation'][0])),
        'first_orientation_xyzw': d['orientation'][0].tolist(),
        'per_minute': [{'begin_s': begin, 'unchanged_pairs': int(equal[(elapsed[1:] >= begin) & (elapsed[1:] < begin+60)].sum()),
                        'pairs': int(((elapsed[1:] >= begin) & (elapsed[1:] < begin+60)).sum())}
                       for begin in range(0,int(elapsed[-1]),60)],
        'longest_runs': [{'begin_s': float(elapsed[starts[i]]), 'end_s': float(elapsed[ends[i]]),
                          'samples': int(lengths[i]), 'same_value_span_ms': float(holds[i]),
                          'until_next_different_value_ms': float((stamp[next_change[i]]-stamp[starts[i]])*1e-6),
                          'accel': values[starts[i],:3].tolist(), 'gyro': values[starts[i],3:].tolist()}
                         for i in top],
    }


def main():
    report = {'interpretation': 'Value change rate is NOT hardware sample rate. Repeats may be stale publication, upstream replay, quantization or other driver behavior; the bag alone cannot identify the cause.',
              'streams': {n: analyze(n) for n in ('central','front','rear')}}
    with (ROOT / 'repeated_values.json').open('x') as f:
        json.dump(report, f, indent=2)
    print(json.dumps({n: {k:v for k,v in s.items() if k not in ('per_minute','longest_runs')} for n,s in report['streams'].items()},indent=2))


if __name__ == '__main__':
    main()
