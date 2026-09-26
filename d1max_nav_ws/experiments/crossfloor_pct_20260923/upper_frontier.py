"""Where does native A* stop on a leg of a cached variant (cache_<V>.npz)?
Prints the visited-set frontier and, along the recorded track, per-layer
ground/cost/valid with R=reached. Offline only.

usage: upper_frontier.py VARIANT LEG KF0 KF1 [STEP]
"""
import copy
import os
import subprocess
import sys

import numpy as np

sys.path.insert(0, '/home/dndx/d1max_nav_ws/src/d1max_pct_planner')
VENDOR = '/home/dndx/d1max_nav_ws/src/pct_planner_vendor'
if os.environ.get('SWEEP_CHILD') != '1':
    from d1max_pct_planner.native_runtime import prepare_native_environment
    env = prepare_native_environment(VENDOR)
    env['SWEEP_CHILD'] = '1'
    raise SystemExit(subprocess.run([sys.executable, __file__, *sys.argv[1:]], env=env).returncode)

import yaml  # noqa: E402

import native_sweep as ns  # noqa: E402
from curve_diag import cached_tomogram  # noqa: E402
from d1max_pct_planner.crossfloor_route import LEGS, _masked_tomogram, validate_config  # noqa: E402
from d1max_pct_planner.planner_core import TomogramPlanner  # noqa: E402

POSES = ('/home/dndx/d1max_nav_ws/maps/runs/20260923_132601_150357_central_imu_faster_lio_sc_pgo_zenoh/'
         'sc_pgo/optimized_poses.txt')


def main():
    variant, leg, k0, k1 = sys.argv[1], sys.argv[2], int(sys.argv[3]), int(sys.argv[4])
    step = int(sys.argv[5]) if len(sys.argv) > 5 else 3
    _, tomo = cached_tomogram(variant)
    base = yaml.safe_load(open(ns.ROUTE_CFG))
    config = copy.deepcopy(base)
    for key, item in base['anchors'].items():
        a = ns.snap_anchor(tomo, item['xyz'][:2], ns.EXPECTED[key])
        config['anchors'][key] = {'xyz': a['xyz'], 'layer_id': a['layer_id']}
    settings = validate_config(config)
    _, begin, end = next(item for item in LEGS if item[0] == leg)
    masked = _masked_tomogram(tomo, leg, settings)
    planner = TomogramPlanner(VENDOR, use_quintic=True, max_heading_rate=1.0, ground_z=True,
                              astar_cost_weight=1.0, optimizer_cost_margin=8.0)
    planner.load_payload(masked.native_payload())
    planner.planner.debug()
    a, b = settings['anchors'][begin], settings['anchors'][end]
    start = np.array([a['layer_id'], *planner._position_index(a['xyz'][:2])], dtype=np.int32)
    goal = np.array([b['layer_id'], *planner._position_index(b['xyz'][:2])], dtype=np.int32)
    ok = planner.planner.plan(start, goal, False)
    visited = np.asarray(planner.planner.get_path_finder().get_visited_set())
    lay, ix, iy = visited[:, 0], visited[:, 1], visited[:, 2]
    ground = masked.ground[lay, ix, iy]
    print(f'[{leg}] {begin}{a["xyz"]} L{a["layer_id"]} -> {end}{b["xyz"]} L{b["layer_id"]} found={ok} '
          f'visited={len(visited)} layers={sorted(set(lay.tolist()))}')
    for lo, hi in [(1.0, 1.6), (1.6, 2.2), (2.2, 2.8), (2.8, 3.4)]:
        sel = (ground >= lo) & (ground < hi)
        if sel.any():
            w = masked.center + (np.stack([ix[sel], iy[sel]], 1) - masked.offset) * masked.resolution
            print(f'   visited ground {lo:.1f}..{hi:.1f}: n={sel.sum()} x[{w[:, 0].min():.2f},{w[:, 0].max():.2f}]'
                  f' y[{w[:, 1].min():.2f},{w[:, 1].max():.2f}]')
    reached = {(int(l), int(i), int(j)) for l, i, j in visited}
    poses = np.loadtxt(POSES)[:, [3, 7, 11]]
    print(' kf   xy               body_z | layer: ground cost valid ceil-ground (R=visited)')
    for k in range(k0, k1 + 1, step):
        cx, cy = masked.index(poses[k, :2])
        parts = []
        for l in range(masked.layers):
            gr = masked.ground[l, cx, cy]
            if not np.isfinite(gr) or abs(gr - (poses[k, 2] - 0.42)) > 0.4:
                continue
            ce = masked.ceiling[l, cx, cy]
            parts.append(f'L{l}:{gr:+.2f} {masked.cost[l, cx, cy]:4.1f} {"v" if masked.valid[l, cx, cy] else "-"} '
                         f'{(ce - gr) if np.isfinite(ce) else np.inf:4.2f}{"R" if (l, cx, cy) in reached else "."}')
        print(f'{k:4d} {poses[k, :2].round(2)} {poses[k, 2]:+.2f} | ' + ' | '.join(parts))


if __name__ == '__main__':
    main()
