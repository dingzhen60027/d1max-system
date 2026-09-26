"""Offline strict native PCT regression beyond the local stair portals."""
import copy
import gc
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
from d1max_pct_planner.crossfloor_route import plan_crossfloor, _masked_tomogram, validate_config
from d1max_pct_planner.tomogram_map import TomogramMap
from d1max_pct_planner.tomogram_route import TomogramRoute

ROOT = Path(__file__).resolve().parent


def main():
    with np.load(ROOT / 'cache_composed.npz') as z:
        payload = {k: z[k] for k in z.files}
    payload.update(frame_id='d1max_loc_map')
    tomo = TomogramMap(payload, unknown_ceiling_policy='allow_unobserved', max_ground_step_m=.17)
    poses = np.loadtxt('/home/dndx/d1max_nav_ws/maps/runs/20260923_132601_150357_central_imu_faster_lio_sc_pgo_zenoh/sc_pgo/optimized_poses.txt')[:, [3, 7, 11]]
    base = yaml.safe_load(open(ns.ROUTE_CFG))
    for key, anchor in base['anchors'].items():
        a = ns.snap_anchor(tomo, anchor['xyz'][:2], ns.EXPECTED[key])
        base['anchors'][key] = {k: a[k] for k in ('xyz', 'layer_id')}

    def select(kf):
        # The layer's measured ground, not the robot IMU/body position Z.
        return ns.snap_anchor(tomo, poses[kf, :2], -.64, search_m=.4)

    trials = []
    for kf in ([] if '--corridors-only' in sys.argv else [50, 200, 400, 600]):
        chosen = select(kf)
        report = {'trial': f'crossfloor_kf{kf}', 'keyframe': kf, 'anchor': chosen}
        if chosen is None:
            report.update(ok=False, code='no_start_surface_within_0.4m')
        else:
            config = copy.deepcopy(base)
            config['anchors']['start'] = {k: chosen[k] for k in ('xyz', 'layer_id')}
            try:
                result = plan_crossfloor(config, tomo)
                report.update(ok=True, length_m=result['length_m'], segments=result['segments'])
                (ROOT / f'long_route_kf{kf}.json').write_text(json.dumps(result, indent=2)+'\n')
            except Exception as exc:
                report.update(ok=False, code=getattr(exc, 'code', type(exc).__name__), message=str(exc),
                              details=getattr(exc, 'details', None), cause=getattr(exc.__cause__, 'details', None))
        trials.append(report)
        print('TRIAL', json.dumps(report), flush=True)
        gc.collect()
    masked = _masked_tomogram(tomo, 'lower_floor', validate_config(base))
    for a, b in [(25, 75), (350, 400)]:
        first, last = select(a), select(b)
        report = {'trial': f'corridor_{a}_{b}', 'start': first, 'goal': last}
        if first is None or last is None:
            report.update(ok=False, code='no_endpoint_surface_within_0.4m')
        else:
            try:
                planner = TomogramRoute(masked, VENDOR, max_heading_rate=1., astar_cost_weight=1.,
                                        optimizer_cost_margin=8., path_refinement='visibility_c2')
                result = planner.plan(first['xyz'], last['xyz'], first['layer_id'], last['layer_id'])
                path = np.array(result['path'])
                report.update(ok=True, length_m=float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum()),
                              direct_xy_m=float(np.linalg.norm(np.array(first['xyz'])[:2]-np.array(last['xyz'])[:2])),
                              quality=result.get('path_quality'))
                (ROOT / f'corridor_route_{a}_{b}.json').write_text(json.dumps(result, indent=2,
                    default=lambda value: value.tolist() if isinstance(value, np.ndarray) else value.item())+'\n')
            except Exception as exc:
                report.update(ok=False, code=getattr(exc, 'code', type(exc).__name__), message=str(exc),
                              details=getattr(exc, 'details', None), cause=getattr(exc.__cause__, 'details', None))
        trials.append(report)
        print('TRIAL', json.dumps(report), flush=True)
        gc.collect()
    output = 'corridor_route_verification.json' if '--corridors-only' in sys.argv else 'long_route_verification.json'
    (ROOT / output).write_text(json.dumps(trials, indent=2)+'\n')


if __name__ == '__main__':
    main()
