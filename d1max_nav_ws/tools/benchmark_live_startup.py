#!/usr/bin/env python3
"""Read-only-map benchmark of equivalent old/new startup hashing (no ROS)."""
import argparse
import hashlib
import json
from pathlib import Path
import statistics
import sys
import time
import tracemalloc

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/d1max_pct_scan'))
from d1max_pct_scan.live_runtime import atomic_json, file_sha256


def measure(operation, repeat):
    times, peaks, digests = [], [], []
    for _ in range(repeat):
        tracemalloc.start()
        start = time.perf_counter()
        digests.append(operation())
        times.append(1000*(time.perf_counter()-start))
        peaks.append(tracemalloc.get_traced_memory()[1])
        tracemalloc.stop()
    return {'median_ms': statistics.median(times), 'max_ms': max(times),
            'peak_python_bytes': max(peaks), 'sha256': digests[0],
            'stable_digest': len(set(digests)) == 1}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--map', type=Path, required=True)
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args()
    args.map = args.map.resolve(strict=True)
    if args.report.exists():
        raise ValueError('Choose a new report path; existing evidence is preserved')
    old = measure(lambda: hashlib.sha256(args.map.read_bytes()).hexdigest(), 5)
    new = measure(lambda: file_sha256(args.map), 5)
    report = {'kind': 'OFFLINE_NO_ROS_READ_ONLY_MAP', 'map': str(args.map),
              'map_bytes': args.map.stat().st_size, 'repeat': 5,
              'before_whole_file_read': old, 'after_streaming_read': new,
              'equivalent_digest': old['sha256'] == new['sha256'],
              'limitations': 'Warm filesystem cache; tracemalloc measures Python allocations, not process RSS.'}
    args.report.parent.mkdir(parents=True, exist_ok=True)
    atomic_json(args.report, report)
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
