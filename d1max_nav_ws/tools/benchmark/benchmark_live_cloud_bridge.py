#!/usr/bin/env python3
"""Offline before/after microbenchmark; no ROS nodes, network or robot access.

Baseline is the exact pre-optimization decode/transform/pack implementation.
--ros-serialization additionally measures native PointCloud2 serialization,
without rclpy.init or any publisher. Peak is Python/NumPy tracemalloc high-water,
NOT total ROS/RMW process RSS. No throughput or hardware-validation claim.
"""
from array import array
import argparse
from collections import deque
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import sys
import time
import tracemalloc

import numpy as np

WS = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WS/'src/d1max_pct_scan'))
from d1max_pct_scan.live_scan_contract import (decode_xyz, pack_xyz, quaternion_matrix,
                                             take_latest_exact, transform_xyz)


def baseline_decode(*, data, fields, point_step, row_step, width, height, bigendian,
                    max_input_points=250000, max_output_points=100000):
    if (type(width) is not int or type(height) is not int or width < 1 or height < 1
            or width * height > max_input_points or not 12 <= point_step <= 256
            or row_step < width * point_step or len(data) != row_step * height
            or len(data) > 32 * 1024 * 1024):
        raise ValueError('invalid_or_oversize_cloud')
    names = {f[0]: f for f in fields}
    columns = []
    for name in ('x', 'y', 'z'):
        if name not in names:
            raise ValueError('cloud_missing_xyz')
        _, offset, datatype, count = names[name]
        if datatype not in (7, 8) or count != 1 or not 0 <= offset <= point_step-(4 if datatype == 7 else 8):
            raise ValueError('unsupported_xyz_field')
        dtype = ('>' if bigendian else '<') + ('f4' if datatype == 7 else 'f8')
        column = np.ndarray((height, width), dtype=dtype, buffer=data,
                            offset=offset, strides=(row_step, point_step)).reshape(-1)
        columns.append(column)
    xyz = np.column_stack(columns)
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    if not len(xyz):
        raise ValueError('cloud_has_no_finite_points')
    stride = max(1, math.ceil(len(xyz) / max_output_points))
    return np.asarray(xyz[::stride], dtype='<f4')


def baseline_transform(points, translation, quaternion):
    xyz, translation = np.asarray(points), np.asarray(translation, dtype=float)
    if (xyz.ndim != 2 or xyz.shape[1] != 3 or not np.isfinite(xyz).all()
            or translation.shape != (3,) or not np.isfinite(translation).all()):
        raise ValueError('invalid_transform_geometry')
    return np.asarray(xyz @ quaternion_matrix(quaternion).T + translation, dtype='<f4')


def baseline_pack(points):
    return array('B', np.asarray(points, dtype='<f4').tobytes())


def native_serializer():
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import PointCloud2, PointField
    def serialize(payload):
        cloud = PointCloud2()
        cloud.header.frame_id = 'd1max_loc_map'
        cloud.header.stamp.sec, cloud.header.stamp.nanosec = 100, 123456789
        cloud.height, cloud.width, cloud.point_step = 1, len(payload)//12, 12
        cloud.row_step, cloud.is_dense, cloud.data = len(payload), True, payload
        cloud.fields = [PointField(name=name, offset=i*4, datatype=7, count=1)
                        for i, name in enumerate('xyz')]
        return serialize_message(cloud)
    return serialize


def fixture(points):
    raw = np.zeros((len(points), 8), dtype='<f4')
    raw[:, :3] = points
    return dict(data=array('B', raw.tobytes()), fields=[(n, i*4, 7, 1) for i, n in enumerate('xyz')],
                point_step=32, row_step=len(raw)*32, width=len(raw), height=1, bigendian=False)


def summarize(values):
    return {'p50': float(np.percentile(values, 50)), 'p95': float(np.percentile(values, 95))}


def compare(arguments, iterations, serializer):
    q = np.asarray([.03, -.04, .12, .991513]); q /= np.linalg.norm(q)
    variants = {'before': (baseline_decode, baseline_transform, baseline_pack),
                'after': (decode_xyz, transform_xyz, pack_xyz)}
    measurements = {name: [] for name in variants}
    def run(name):
        decode, transform, pack = variants[name]
        t0 = time.perf_counter_ns()
        points = decode(**arguments)
        t1 = time.perf_counter_ns()
        transformed = transform(points, [11., 21., .6], q)
        t2 = time.perf_counter_ns()
        payload = pack(transformed)
        t3 = time.perf_counter_ns()
        wire = serializer(payload) if serializer else payload
        t4 = time.perf_counter_ns()
        return wire, payload, [(t1-t0)/1e6, (t2-t1)/1e6, (t3-t2)/1e6, (t4-t3)/1e6, (t4-t0)/1e6]
    before, before_payload, _ = run('before'); after, after_payload, _ = run('after')
    if before_payload != after_payload:
        raise AssertionError('XYZ payload bytes differ: optimization is not equivalent')
    if serializer:
        # CDR alignment padding is unspecified and differs between allocations.
        # Compare every deserialized ROS field, not those non-semantic bytes.
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import PointCloud2
        if deserialize_message(before, PointCloud2) != deserialize_message(after, PointCloud2):
            raise AssertionError('Serialized PointCloud2 fields differ')
    output_digest = hashlib.sha256(before_payload).hexdigest()
    for iteration in range(iterations+8):
        # Alternate order to reduce cache/thermal bias; inputs exactly shared.
        for name in (('before', 'after') if iteration % 2 else ('after', 'before')):
            _, _, phases = run(name)
            if iteration >= 8:
                measurements[name].append(phases)
    report = dict(input_points=arguments['width']*arguments['height'],
                  output_points=len(decode_xyz(**arguments)), payload_sha256=output_digest,
                  identical_payload_bytes=True, ros_message_fields_equal=True if serializer else None)
    names = ['decode', 'transform', 'pack', 'ros_serialize', 'total']
    for name, samples in measurements.items():
        tracemalloc.start()
        result, payload, _ = run(name)
        peak = tracemalloc.get_traced_memory()[1]
        tracemalloc.stop()
        del result, payload
        report[name] = dict(ms={stage: summarize(np.asarray(samples)[:, i])
                               for i, stage in enumerate(names)}, traced_peak_mib=peak/2**20)
    report['total_p50_speedup'] = report['before']['ms']['total']['p50']/report['after']['ms']['total']['p50']
    return report


def queue_profile():
    """Deterministic overload model: source 20Hz, consume 10Hz, queue capacity3.

    Exact TF for the first scan never arrives; every later scan has its own TF.
    This tests scheduling only, not a synthetic TF as a substitute for real TF.
    """
    report = {}
    for mode in ('before_fifo', 'after_latest_exact'):
        pending, published, age_ms, dropped = deque(maxlen=3), [], [], 0
        for tick in range(401):
            now = 100.+tick*.01
            if tick % 5 == 0:
                dropped += int(len(pending) == 3)
                pending.append(now)
            if tick % 10:
                continue
            if mode == 'before_fifo':
                while pending and now-pending[0] > .25:
                    pending.popleft(); dropped += 1
                selected = pending.popleft() if pending and pending[0] != 100. else None
            else:
                selected, _, rejected, _ = take_latest_exact(pending,
                    rejection=lambda stamp: 'deadline_or_age' if now-stamp > .25 else None,
                    resolve=lambda stamp: None if stamp == 100. else ('exact', stamp))
                dropped += sum(rejected.values())
            if selected is not None:
                published.append(selected); age_ms.append((now-selected)*1000.)
            assert len(pending) <= 3
        assert all(b > a for a, b in zip(published, published[1:]))
        report[mode] = dict(published=len(published), dropped=dropped,
                            source_age_ms=summarize(age_ms), strictly_increasing_stamps=True)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--iterations', type=int, default=100)
    parser.add_argument('--input-npy', type=Path, help='Optional real saved XYZ crop; never read live data')
    parser.add_argument('--ros-serialization', action='store_true')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if not 10 <= args.iterations <= 1000:
        parser.error('--iterations must be 10..1000')
    serializer = native_serializer() if args.ros_serialization else None
    rng = np.random.default_rng(14)
    cases = {f'synthetic_{n}': fixture(rng.normal(size=(n, 3))*10.) for n in (33444, 100000, 250000)}
    invalid = rng.normal(size=(100000, 3))*10.; invalid[::100, 1] = np.nan
    cases['synthetic_100000_one_percent_nonfinite'] = fixture(invalid)
    if args.input_npy:
        points = np.load(args.input_npy, allow_pickle=False)
        if points.ndim != 2 or points.shape[1] != 3 or not 1 <= len(points) <= 250000:
            raise ValueError('Expected bounded Nx3 saved crop')
        cases['saved_pcd_crop'] = fixture(points)
    result = dict(schema=1, kind='OFFLINE_MICROBENCHMARK_NO_ROBOT', motion_enabled=False,
        python=platform.python_version(), numpy=np.__version__, iterations=args.iterations,
        ros_serialization=args.ros_serialization,
        threads={name: os.environ.get(name) for name in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS')},
        limitation='Allocation peak excludes source fixture, middleware/network buffers and full process RSS.',
        cases={name: compare(data, args.iterations, serializer) for name, data in cases.items()},
        queue_model=queue_profile())
    if args.input_npy:
        result['saved_input'] = str(args.input_npy.resolve())
    encoded = json.dumps(result, indent=2, allow_nan=False)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded+'\n')
    print(encoded)


if __name__ == '__main__':
    main()
