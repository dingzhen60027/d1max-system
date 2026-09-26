"""Offline evidence for upper-floor sampling holes; no official files changed.

Restore only measured keyframe returns to final-map cells which have lost their
upper-floor surface. The original W cost equations and curve checks are retained.
"""
import copy
import hashlib
import json
import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import open3d as o3d
import yaml
from scipy.ndimage import convolve, generic_filter

sys.path.insert(0, '/home/dndx/d1max_nav_ws/src/d1max_pct_planner')
VENDOR = '/home/dndx/d1max_nav_ws/src/pct_planner_vendor'
if os.environ.get('SWEEP_CHILD') != '1':
    from d1max_pct_planner.native_runtime import prepare_native_environment
    env = prepare_native_environment(VENDOR)
    env['SWEEP_CHILD'] = '1'
    raise SystemExit(subprocess.run([sys.executable, __file__], env=env).returncode)

import native_sweep as ns
from curve_diag import cached_tomogram
from d1max_pct_planner.crossfloor_route import plan_crossfloor, _native_route

ROOT = Path(__file__).resolve().parent
RUN = Path('/home/dndx/d1max_nav_ws/maps/runs/20260923_132601_150357_central_imu_faster_lio_sc_pgo_zenoh/sc_pgo')


def main():
    _, tomo = cached_tomogram('W')
    original = np.asarray(o3d.io.read_point_cloud(ns.PCD).points, dtype=np.float32)
    kept = original[ns.visibility_keep(original, 10, 20, 0)]
    ground = tomo.ground[12]
    support = np.isfinite(ground) & (ground > 3.0) & (ground < 3.23)
    neighbor_count = convolve(support.astype(int), np.ones((5, 5), int), mode='constant')
    # The local nominal height is an inference for selecting raw evidence only;
    # the returned PCD point is one actual transformed scan return.
    total = convolve(np.where(support, ground, 0), np.ones((5, 5)), mode='constant')
    estimate = total / np.maximum(neighbor_count, 1)
    xx, yy = np.meshgrid(np.arange(ground.shape[0]), np.arange(ground.shape[1]), indexing='ij')
    wx = tomo.center[0] + (xx - tomo.offset[0]) * tomo.resolution
    wy = tomo.center[1] + (yy - tomo.offset[1]) * tomo.resolution
    holes = ((~np.isfinite(ground) | (ground < 2.9)) & (neighbor_count >= 8)
             & (wx >= -33) & (wx <= -19) & (wy >= 43.5) & (wy <= 51.2))
    hole_ids = set(np.flatnonzero(holes))
    records = {k: [] for k in hole_ids}
    poses = np.loadtxt(RUN / 'optimized_poses.txt').reshape(-1, 3, 4)
    for frame in range(940, 986):
        scan = np.asarray(o3d.io.read_point_cloud(str(RUN / 'Scans' / f'{frame:06d}.pcd')).points)
        pose = poses[frame]
        world = scan @ pose[:, :3].T + pose[:, 3]
        region = ((world[:, 0] >= -33.1) & (world[:, 0] <= -18.9)
                  & (world[:, 1] >= 43.4) & (world[:, 1] <= 51.3)
                  & (world[:, 2] >= 2.95) & (world[:, 2] <= 3.3))
        original_scan_indices = np.flatnonzero(region)
        world = world[region]
        idx = np.rint((world[:, :2] - tomo.center) / tomo.resolution).astype(int) + tomo.offset
        ids = np.ravel_multi_index(idx.T, ground.shape)
        for k in np.intersect1d(ids, np.asarray(list(hole_ids))):
            cell = np.unravel_index(k, ground.shape)
            local = np.flatnonzero(ids == k)
            local = local[np.abs(world[local, 2] - estimate[cell]) <= .06]
            points = world[local]
            if not len(points):
                continue
            # One vote per frame; actual median-height sample prevents dense
            # near-range scans from outweighing independent observations.
            selected = int(np.argsort(points[:, 2])[len(points) // 2])
            sample = points[selected]
            records[k].append((frame, sample, int(original_scan_indices[local[selected]])))
    points, evidence = [], []
    for k, rows in sorted(records.items()):
        if len(rows) < 5:
            continue
        samples = np.asarray([r[1] for r in rows])
        if np.ptp(samples[:, 2]) > .08:
            continue
        selected = int(np.argsort(samples[:, 2])[len(samples) // 2])
        sample = samples[selected]
        points.append(sample)
        cell = np.unravel_index(k, ground.shape)
        evidence.append({'cell': list(map(int, cell)), 'xy': tomo.world(cell).tolist(),
                         'old_ground': float(ground[cell]), 'sample_xyz': sample.tolist(),
                         'source_frame_id': int(rows[selected][0]),
                         'source_point_index': int(rows[selected][2]),
                         'frames': [r[0] for r in rows], 'height_range': [float(samples[:, 2].min()), float(samples[:, 2].max())]})
    restored = np.asarray(points, dtype=np.float32)
    np.savez_compressed(ROOT / 'upper_floor_measured_restore.npz', points=restored)
    (ROOT / 'upper_floor_measured_restore.json').write_text(json.dumps(evidence, indent=2))
    def sha256(path):
        digest = hashlib.sha256()
        with open(path, 'rb') as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b''):
                digest.update(block)
        return digest.hexdigest()
    sources = [Path(ns.PCD), RUN / 'optimized_map.pcd', RUN / 'optimized_poses.txt']
    sources += [RUN / 'Scans' / f'{frame:06d}.pcd' for frame in range(940, 986)]
    provenance = {str(path): sha256(path) for path in sources}
    (ROOT / 'upper_floor_measured_restore_sources.json').write_text(json.dumps(provenance, indent=2))
    print('candidate missing cells', len(hole_ids), 'restored measured cells', len(restored), flush=True)
    combined = np.concatenate([kept, restored])
    test = ns.build(combined, .10, .50, ns.CROP, {'slope_max_rad': .60, 'safe_margin': .10, 'inflation': .10})
    np.savez_compressed(ROOT / 'cache_W_measured_restore.npz', data=test.data, resolution=test.resolution,
                        center=test.center, slice_h0=test.slice_h0, slice_dh=test.slice_dh,
                        selected_source_layers=test.source_layers)
    config = yaml.safe_load(open(ns.ROUTE_CFG))
    for key, anchor in config['anchors'].items():
        a = ns.snap_anchor(test, anchor['xyz'][:2], ns.EXPECTED[key])
        config['anchors'][key] = {'xyz': a['xyz'], 'layer_id': a['layer_id']}
    try:
        native_results = {}
        def factory(tomogram, name, settings):
            route = _native_route(tomogram, name, settings)
            class Capture:
                def plan(self, *args):
                    value = route.plan(*args)
                    native_results[name] = {k: value[k] for k in (
                        'curve_validation', 'quintic_segments', 'curve_partition_points',
                        'path_refinement', 'algorithm')}
                    return value
            return Capture()
        result = plan_crossfloor(config, test, route_factory=factory)
        path = np.asarray(result['path_xyz'])
        diagnostics = []
        for segment in result['segments']:
            pts = path[segment['first_index']:segment['last_index'] + 1]
            diff = np.diff(pts, axis=0)
            xy = np.linalg.norm(diff[:, :2], axis=1)
            ratio = np.abs(diff[:, 2]) / np.maximum(xy, 1e-6)
            worst = int(np.argmax(ratio))
            diagnostics.append({
                'name': segment['name'], 'length_m': segment['length_m'],
                'layers': segment['layer_ids'], 'max_step_m': float(np.abs(diff[:, 2]).max()),
                'maximum_grade_edge': {'start': pts[worst].tolist(), 'end': pts[worst + 1].tolist(),
                                      'xy_m': float(xy[worst]), 'dz_m': float(diff[worst, 2]),
                                      'grade': float(ratio[worst])},
                'native_validation': native_results[segment['name']], 'checks': segment['checks']})
        result['segment_diagnostics'] = diagnostics
        (ROOT / 'upper_floor_measured_restore_route.json').write_text(json.dumps(result, indent=2))
        print('FULL ROUTE OK', result['length_m'], result['stair_profile'], flush=True)
        print('SEGMENT DIAGNOSTICS', json.dumps(diagnostics), flush=True)
    except Exception as exc:
        print('ROUTE FAIL', getattr(exc, 'code', type(exc).__name__), str(exc), getattr(exc, 'details', None), flush=True)
        print('CAUSE', getattr(exc.__cause__, 'details', None), flush=True)


if __name__ == '__main__':
    main()
