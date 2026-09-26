"""Offline per-map structure protection before strict native PCT validation.

Protect the reviewed western walls and upper slab edge from visibility deletion.
All removal candidates still require the original independent free/hit rule.
"""
import hashlib
import json
import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import open3d as o3d
import yaml

sys.path.insert(0, '/home/dndx/d1max_nav_ws/src/d1max_pct_planner')
VENDOR = '/home/dndx/d1max_nav_ws/src/pct_planner_vendor'
if os.environ.get('SWEEP_CHILD') != '1':
    from d1max_pct_planner.native_runtime import prepare_native_environment
    env = prepare_native_environment(VENDOR)
    env['SWEEP_CHILD'] = '1'
    raise SystemExit(subprocess.run([sys.executable, __file__, *sys.argv[1:]], env=env).returncode)

import native_sweep as ns
from d1max_pct_planner.crossfloor_route import plan_crossfloor

ROOT = Path(__file__).resolve().parent
PROTECT = [
    {'id': 'reviewed_west_wall', 'min': [-34.5, 47.5, -1.4], 'max': [-32.5, 50.0, 4.9]},
    {'id': 'reviewed_upper_slab_edge', 'min': [-34.5, 44.0, 3.02], 'max': [-25.5, 45.5, 3.3]},
]


def main():
    original = np.asarray(o3d.io.read_point_cloud(ns.PCD).points, dtype=np.float32)
    with np.load(ROOT / 'visibility_votes.npz') as z:
        free, hit, roi = z['free'], z['hit'], z['in_roi']
    remove = roi & (free >= 10 * np.maximum(hit, 1)) & (free >= 20)
    original_removed = int(remove.sum())
    areas = []
    for box in PROTECT:
        protected = np.all((original >= box['min']) & (original <= box['max']), axis=1)
        areas.append({**box, 'source_points': int(protected.sum()), 'restored_points': int(np.sum(remove & protected))})
        remove[protected] = False
    with np.load(ROOT / 'upper_floor_measured_restore.npz') as z:
        restored = z['points']
    source_hash = hashlib.sha256(Path(ns.PCD).read_bytes()).hexdigest()
    np.savez_compressed(ROOT / 'structure_protected_visibility_mask.npz', keep=~remove,
                        source_sha256=source_hash, source_point_count=len(original))
    test = ns.build(np.concatenate([original[~remove], restored]), .10, .50, ns.CROP,
                    {'slope_max_rad': .60, 'safe_margin': .10, 'inflation': .10})
    np.savez_compressed(ROOT / 'cache_W_structures_measured_restore.npz', data=test.data,
                        resolution=test.resolution, center=test.center, slice_h0=test.slice_h0,
                        slice_dh=test.slice_dh, selected_source_layers=test.source_layers)
    config = yaml.safe_load(open(ns.ROUTE_CFG))
    for key, anchor in config['anchors'].items():
        a = ns.snap_anchor(test, anchor['xyz'][:2], ns.EXPECTED[key])
        config['anchors'][key] = {'xyz': a['xyz'], 'layer_id': a['layer_id']}
    report = {'source_pcd': ns.PCD, 'source_sha256': source_hash,
              'rule': {'free_ratio': 10, 'minimum_free_frames': 20, 'sphere_or_ellipsoid': None,
                       'visibility_roi_min': [-34.5, 44., -1.4], 'visibility_roi_max': [-25.5, 57.5, 4.9]},
              'protection_regions': areas, 'unprotected_removed': original_removed,
              'removed': int(remove.sum()), 'measured_returns_added': len(restored),
              'pct': {'resolution': .1, 'slice_dh': .5, 'slope_max_rad': .60,
                      'safe_margin': .1, 'inflation': .1, 'crop': ns.CROP}, 'anchors': config['anchors']}
    try:
        result = plan_crossfloor(config, test)
        report.update(ok=True, length_m=result['length_m'], stair_profile=result['stair_profile'])
        # Persist the full JSON-compatible result for independent route inspection.
        def plain(obj):
            if isinstance(obj, np.ndarray): return obj.tolist()
            if isinstance(obj, np.generic): return obj.item()
            raise TypeError(type(obj).__name__)
        (ROOT / 'structure_protected_route.json').write_text(json.dumps(result, default=plain, indent=2)+'\n')
    except Exception as exc:
        report.update(ok=False, code=getattr(exc, 'code', type(exc).__name__), message=str(exc),
                      details=getattr(exc, 'details', None), cause=getattr(exc.__cause__, 'details', None))
    print(json.dumps(report, indent=2), flush=True)
    (ROOT / 'structure_protection_trial.json').write_text(json.dumps(report, indent=2)+'\n')


if __name__ == '__main__':
    main()
