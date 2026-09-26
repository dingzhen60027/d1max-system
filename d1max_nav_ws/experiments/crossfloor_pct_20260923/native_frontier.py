"""Where does native PCT A* stop on the stair? Uses the real native planner.

Runs plan_crossfloor's own leg masking, then native A* in debug mode and
reports the visited set's height/position frontier. Offline analysis only.
"""
import os
import subprocess
import sys

import numpy as np
import yaml

sys.path.insert(0, '/home/dndx/d1max_nav_ws/src/d1max_pct_planner')
VENDOR = '/home/dndx/d1max_nav_ws/src/pct_planner_vendor'
ROUTE_CFG = '/home/dndx/d1max_nav_ws/src/d1max_pct_planner/config/crossfloor_route_20260923.yaml'

if os.environ.get('FRONTIER_CHILD') != '1':
    from d1max_pct_planner.native_runtime import prepare_native_environment
    env = prepare_native_environment(VENDOR)
    env['FRONTIER_CHILD'] = '1'
    raise SystemExit(subprocess.run([sys.executable, __file__, *sys.argv[1:]], env=env).returncode)

from d1max_pct_planner.crossfloor_route import _masked_tomogram, validate_config  # noqa: E402
from d1max_pct_planner.planner_core import TomogramPlanner  # noqa: E402
from d1max_pct_planner.tomogram_map import TomogramMap  # noqa: E402


def frontier(tomogram, settings, leg, begin, end):
    masked = _masked_tomogram(tomogram, leg, settings)
    planner = TomogramPlanner(VENDOR, use_quintic=True, max_heading_rate=1.0, ground_z=True,
                              astar_cost_weight=1.0, optimizer_cost_margin=8.0)
    planner.load_payload(masked.native_payload())
    planner.planner.debug()
    a, b = settings['anchors'][begin], settings['anchors'][end]
    start = np.array([a['layer_id'], *planner._position_index(a['xyz'][:2])], dtype=np.int32)
    goal = np.array([b['layer_id'], *planner._position_index(b['xyz'][:2])], dtype=np.int32)
    ok = planner.planner.plan(start, goal, False)
    finder = planner.planner.get_path_finder()
    visited = np.asarray(finder.get_visited_set())
    layer, ix, iy = visited[:, 0], visited[:, 1], visited[:, 2]
    ground = masked.ground[layer, ix, iy]
    xy = masked.center + (np.stack([ix, iy], 1) - masked.offset) * masked.resolution
    top = int(np.nanargmax(np.where(np.isfinite(ground), ground, -1e9)))
    print(f'[{leg}] found={ok} visited={len(visited)} layers={sorted(set(layer.tolist()))}')
    print(f'   highest visited ground z={ground[top]:.3f} at xy={xy[top].round(2)} layer={layer[top]}'
          f'  (target {end} z={b["xyz"][2]:.3f})')
    return masked, visited, ok


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else None
    raw = yaml.safe_load(open(ROUTE_CFG))
    settings = validate_config(raw)
    tomogram = TomogramMap(path or settings['tomogram_path'],
                           unknown_ceiling_policy=settings['unknown_ceiling_policy'],
                           max_ground_step_m=settings['limits']['max_ground_step_m'])
    masked, visited, ok = frontier(tomogram, settings, 'stair_lower', 'entry', 'landing')
    # Layer-by-layer stair profile along the recorded lower flight centreline.
    poses = np.loadtxt('/home/dndx/d1max_nav_ws/maps/runs/20260923_132601_150357_central_imu_faster_lio_'
                       'sc_pgo_zenoh/sc_pgo/optimized_poses.txt')[:, [3, 7, 11]]
    seen = {(int(l), int(x), int(y)) for l, x, y in visited}
    print('   centreline (kf, xy, per-layer ground/cost/gateway, * = visited):')
    gw = np.zeros_like(masked.cost, dtype=int)
    g = np.nan_to_num(masked.ground, nan=-100.)
    c = masked.native_payload()['data'][0]
    dt, dg = c[1:] - c[:-1], np.abs(g[1:] - g[:-1])
    gw[:-1][(dt < -8) & (dg < .1) & (g[1:] > -99)] = 2
    gw[1:][(dt > 8) & (dg < .1) & (g[:-1] > -99)] = -2
    for k in range(729, 830, 6):
        ix, iy = masked.index(poses[k, :2])
        cells = []
        for l in range(3, 10):
            if np.isfinite(masked.ground[l, ix, iy]) and c[l, ix, iy] < 49:
                mark = '*' if (l, ix, iy) in seen else ' '
                cells.append(f'L{l}:{masked.ground[l, ix, iy]:+.2f}/{c[l, ix, iy]:4.1f}/{gw[l, ix, iy]:+d}{mark}')
        print(f'   kf{k} {poses[k, :2].round(2)} ' + ' '.join(cells))


if __name__ == '__main__':
    main()
