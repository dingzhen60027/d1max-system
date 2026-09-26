"""Bounded native GPMP support-spacing trial; no cost/check relaxation."""
import copy
import json
import os
from pathlib import Path
import subprocess
import sys

import numpy as np
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
from d1max_pct_planner.planner_core import TomogramPlanner
from d1max_pct_planner.tomogram_map import TomogramMap

ROOT = Path(__file__).resolve().parent


def main():
    data = np.zeros((5, 1, 61, 61), np.float32)
    data[4] = 2
    fixture = {'data': data, 'resolution': .1, 'center': np.zeros(2), 'slice_h0': .5, 'slice_dh': .5}
    counts = {}
    for interval in [10, 2]:
        planner = TomogramPlanner(VENDOR, optimizer_sample_interval=interval, ground_z=True)
        planner.load_payload(fixture)
        result = planner.plan([-1.5, 0], [1.5, 0], return_details=True)
        counts[interval] = {'native_initial_samples': planner.planner.get_trajectory_optimizer_wnoj().get_opt_init_value().shape[1],
                            'output_points': len(result['path'])}
    assert counts[2]['native_initial_samples'] > counts[10]['native_initial_samples'], counts
    print('NATIVE PARAMETER PROBE', counts, flush=True)
    with np.load(ROOT / 'cache_Y_measured_restore.npz') as z:
        payload = {k: z[k] for k in z.files}
    payload['frame_id'] = 'd1max_loc_map'
    tomo = TomogramMap(payload, unknown_ceiling_policy='allow_unobserved', max_ground_step_m=.17)
    base = yaml.safe_load(open(ns.ROUTE_CFG))
    for key, anchor in base['anchors'].items():
        selected = ns.snap_anchor(tomo, anchor['xyz'][:2], ns.EXPECTED[key])
        base['anchors'][key] = {'xyz': selected['xyz'], 'layer_id': selected['layer_id']}
    reports = []
    for interval in ([int(value) for value in sys.argv[1:]] if len(sys.argv) > 1 else [5, 3, 2]):
        config = copy.deepcopy(base)
        config.setdefault('planning', {})['optimizer_sample_interval'] = interval
        try:
            result = plan_crossfloor(config, tomo)
            (ROOT / f'sample_interval_Y_{interval}_route.json').write_text(json.dumps(result, indent=2))
            report = {'interval': interval, 'ok': True, 'length_m': result['length_m'],
                      'stair_profile': result['stair_profile'],
                      'segments': result['segments']}
        except Exception as exc:
            report = {'interval': interval, 'ok': False, 'code': getattr(exc, 'code', type(exc).__name__),
                      'message': str(exc), 'details': getattr(exc, 'details', None),
                      'cause': getattr(exc.__cause__, 'details', None)}
        reports.append(report)
        print('TRIAL', json.dumps(report), flush=True)
    (ROOT / 'sample_interval_trial.json').write_text(json.dumps({'probe': counts, 'trials': reports}, indent=2))


if __name__ == '__main__':
    main()
