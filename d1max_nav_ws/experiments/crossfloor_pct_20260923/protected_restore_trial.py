"""Keep elliptically protected surfaces, restore measured upper-floor returns, test strict PCT."""
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
from d1max_pct_planner.tomogram_map import TomogramMap

ROOT = Path(__file__).resolve().parent


def main():
    cache = ROOT / 'cache_Y_measured_restore.npz'
    if cache.exists():
        with np.load(cache) as z:
            payload = {k: z[k] for k in z.files}
        payload.update(frame_id='d1max_loc_map')
        test = TomogramMap(payload, unknown_ceiling_policy='allow_unobserved', max_ground_step_m=.17)
    else:
        original = np.asarray(o3d.io.read_point_cloud(ns.PCD).points, dtype=np.float32)
        kept = original[ns.visibility_keep(original, 10, 20, .12, .05)]
        with np.load(ROOT / 'upper_floor_measured_restore.npz') as z:
            restored = z['points']
        test = ns.build(np.concatenate([kept, restored]), .10, .50, ns.CROP,
                        {'slope_max_rad': .60, 'safe_margin': .10, 'inflation': .10})
        np.savez_compressed(cache, data=test.data, resolution=test.resolution, center=test.center,
                            slice_h0=test.slice_h0, slice_dh=test.slice_dh,
                            selected_source_layers=test.source_layers)
    config = yaml.safe_load(open(ns.ROUTE_CFG))
    for key, anchor in config['anchors'].items():
        a = ns.snap_anchor(test, anchor['xyz'][:2], ns.EXPECTED[key])
        config['anchors'][key] = {'xyz': a['xyz'], 'layer_id': a['layer_id']}
    try:
        result = plan_crossfloor(config, test)
        report = {'ok': True, 'length_m': result['length_m'], 'stair_profile': result['stair_profile']}
    except Exception as exc:
        report = {'ok': False, 'code': getattr(exc, 'code', type(exc).__name__), 'message': str(exc),
                  'details': getattr(exc, 'details', None), 'cause': getattr(exc.__cause__, 'details', None)}
    print(json.dumps(report, indent=2), flush=True)
    (ROOT / 'protected_restore_trial.json').write_text(json.dumps(report, indent=2)+'\n')


if __name__ == '__main__':
    main()
