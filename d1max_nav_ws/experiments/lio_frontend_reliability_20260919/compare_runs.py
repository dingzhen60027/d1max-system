#!/usr/bin/env python3
"""Audit a matched-input frontend comparison without modifying either run."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
import yaml


def load(run):
    manifest = json.loads((run / 'manifest.json').read_text())
    result = json.loads((run / 'result.json').read_text())
    if not result['complete'] or manifest['loop_closure_enabled'] or manifest['height_lock_enabled']:
        raise ValueError('Need completed, unconstrained frontend-only runs')
    data = np.genfromtxt(run / 'frontend_state.csv', delimiter=',', names=True)
    if len(data) < 2 or not all(np.isfinite(data[k]).all() for k in data.dtype.names):
        raise ValueError('Invalid frontend state log')
    if np.any(np.diff(data['t']) <= 0):
        raise ValueError('Nonmonotonic state log')
    with (run / 'paired_scans.csv').open() as stream:
        scans = [r for r in csv.DictReader(stream) if r['admitted'] == 'True']
    config = yaml.safe_load((run / 'config/frontend.yaml').read_text())
    config['laserMapping']['ros__parameters'].pop('diagnostics', None)
    return dict(manifest=manifest, result=result, data=data, scans=scans, config=config,
                calibration=yaml.safe_load((run / 'config/calibration.yaml').read_text()))


def metrics(data, lo, hi):
    data = data[(data['t'] >= lo) & (data['t'] <= hi)]
    xyz = np.column_stack([data[k] for k in ('x', 'y', 'z')])
    gravity = np.column_stack([data[k] for k in ('gx', 'gy', 'gz')])
    ba = np.column_stack([data[k] for k in ('bax', 'bay', 'baz')])
    tilt = np.degrees(np.arctan2(np.linalg.norm(gravity[:, :2], axis=1), -gravity[:, 2]))
    distance = np.r_[0., np.cumsum(np.linalg.norm(np.diff(xyz[:, :2], axis=0), axis=1))]
    dz = xyz[:, 2] - xyz[0, 2]
    return dict(rows=len(data), first_last_stamp_sec=data['t'][[0, -1]].tolist(),
                estimated_xy_path_m=float(distance[-1]), estimated_endpoint_delta_z_m=float(dz[-1]),
                estimated_z_span_m=float(np.ptp(xyz[:, 2])),
                estimated_gravity_tilt_initial_final_max_deg=tilt[[0, -1]].tolist()+[float(tilt.max())],
                estimated_accel_bias_final_norm=float(np.linalg.norm(ba[-1])),
                residual_rmse_median_p95_m=np.percentile(data['residual_rmse'], [50, 95]).tolist(),
                effective_features_min_median=np.percentile(data['features'], [0, 50]).tolist(),
                samples=[dict(elapsed_sec=float(t), estimated_dz_m=float(np.interp(data['t'][0]+t, data['t'], dz)),
                              estimated_xy_path_m=float(np.interp(data['t'][0]+t, data['t'], distance)))
                         for t in (30, 60, 120, 170) if data['t'][0]+t <= data['t'][-1]])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('reference', type=Path)
    parser.add_argument('candidate', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    a, b = load(args.reference), load(args.candidate)
    stamps_a = np.array([int(s['stamp_ns']) for s in a['scans']], dtype=np.int64)
    stamps_b = np.array([int(s['stamp_ns']) for s in b['scans']], dtype=np.int64)
    checks = dict(paired_scan_stamps_identical=np.array_equal(stamps_a, stamps_b),
                  input_point_counts_identical=[s['input_points'] for s in a['scans']] == [s['input_points'] for s in b['scans']],
                  binary_hashes_identical=a['manifest']['binary_sha256'] == b['manifest']['binary_sha256'],
                  effective_frontend_config_identical=a['config'] == b['config'],
                  calibration_identical=a['calibration'] == b['calibration'],
                  same_bag=a['manifest']['bag'] == b['manifest']['bag'],
                  same_replay_rate=a['manifest']['rate'] == b['manifest']['rate'],
                  same_window=a['manifest']['paired_cloud_window_sec'] == b['manifest']['paired_cloud_window_sec'])
    # Point filtering can change the last point by tens of microseconds. Use
    # shared time coverage, and disclose rather than hide that scan-end effect.
    lo = max(a['data']['t'][0], b['data']['t'][0])
    hi = min(a['data']['t'][-1], b['data']['t'][-1])
    report = dict(reference=str(args.reference.resolve()), candidate=str(args.candidate.resolve()),
                  controlled_comparison_checks=checks, checks_all_passed=all(checks.values()),
                  admitted_input_scans=[len(stamps_a), len(stamps_b)],
                  common_header_interval=[float(lo), float(hi)],
                  reference_metrics=metrics(a['data'], lo, hi), candidate_metrics=metrics(b['data'], lo, hi),
                  limits=['Estimated sensor-origin height is not surveyed trajectory error; physical body motion remains.',
                          'No floor, gravity-direction prior, loop closure or height clamp is added by this experiment.',
                          'Dropping a sensor changes field of view, point density and geometry together; this does not estimate an extrinsic correction.',
                          'Passing this short test is not certification for long routes, high speed or outdoor feature-poor environments.'])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write('\n')
    print(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
