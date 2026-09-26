"""Exact Python replica of native PCT A* reachability (a_star_search.cc rules),
used to locate where the stair climb stops and which rule stops it. Offline only.
"""
import sys
from collections import deque

import numpy as np
import open3d as o3d
import yaml

sys.path.insert(0, '/home/dndx/d1max_nav_ws/src/d1max_pct_planner')
from d1max_pct_planner.cpu_tomography import (_inflate, height_layers, mapping_geometry,  # noqa: E402
                                              tomogram_from_points, traversal_cost)
from d1max_pct_planner.crossfloor_route import _masked_tomogram, validate_config  # noqa: E402
from d1max_pct_planner.tomogram_map import TomogramMap  # noqa: E402

PCD = ('/home/dndx/智元四足机器人D1 Max二次开发文档资料包v0.1.0/d1max_ros2/map_manager/data/processed/'
       '20260923_141005_sc-pgo-0923-crossfloor-structure-v1_2c7218/processed_map.pcd')
POSES = ('/home/dndx/d1max_nav_ws/maps/runs/20260923_132601_150357_central_imu_faster_lio_sc_pgo_zenoh/'
         'sc_pgo/optimized_poses.txt')
ROUTE_CFG = '/home/dndx/d1max_nav_ws/src/d1max_pct_planner/config/crossfloor_route_20260923.yaml'
TRAV = {'kernel_size': 7, 'interval_min': 0.55, 'interval_free': 0.70, 'slope_max_rad': 0.40,
        'step_max': 0.17, 'standable_ratio': 0.20, 'cost_barrier': 50.0,
        'safe_margin': 0.20, 'inflation': 0.20}
CROP = ([-40.0, -4.0, -3.0], [26.0, 60.0, 9.0])
NEIGHBORS = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


def native_arrays(masked):
    payload = masked.native_payload()
    trav = payload['data'][0]
    g = np.nan_to_num(payload['data'][3], nan=-100.0)
    dt, dg = trav[1:] - trav[:-1], np.abs(g[1:] - g[:-1])
    ele = np.zeros_like(trav, dtype=np.int32)
    ele[:-1][(dt < -8.0) & (dg < 0.1) & (g[1:] > -99.0)] = 2
    ele[1:][(dt > 8.0) & (dg < 0.1) & (g[:-1] > -99.0)] = -2
    return trav, g, ele


def reachable(trav, g, ele, start):
    L, X, Y = trav.shape

    def decide(layer, i, j):
        for off in (0, -1, 1):
            k = layer + off
            if not 0 <= k < L or abs(g[k, i, j] - g[layer, i, j]) > 0.2:
                continue
            if ele[k, i, j] > 0.5:
                return min(k + 1, L - 1)
            if ele[k, i, j] < -0.5:
                return max(k - 1, 0)
        return layer

    seen = {start}
    queue = deque([start])
    while queue:
        layer, i, j = queue.popleft()
        nl = decide(layer, i, j)
        for di, dj in NEIGHBORS:
            a, b = i + di, j + dj
            if not (0 <= a < X and 0 <= b < Y):
                continue
            if trav[nl, a, b] > 20.0:
                if abs(ele[nl, a, b]) < 0.5 or abs(g[nl, a, b] - g[layer, i, j]) > 0.3:
                    continue
            node = (nl, a, b)
            if node not in seen:
                seen.add(node)
                queue.append(node)
    return seen


def main():
    res = float(sys.argv[1]) if len(sys.argv) > 1 else 0.10
    margin = float(sys.argv[2]) if len(sys.argv) > 2 else 0.10
    infl = float(sys.argv[3]) if len(sys.argv) > 3 else 0.10
    slope = float(sys.argv[4]) if len(sys.argv) > 4 else 0.40
    points = np.asarray(o3d.io.read_point_cloud(PCD).points, dtype=np.float32)
    points = points[np.all((points >= CROP[0]) & (points <= CROP[1]), axis=1)]
    cfg = {'resolution': res, 'slice_dh': 0.5, 'ground_height': -3.0, 'simplify_layers': False,
           'traversability': {**TRAV, 'safe_margin': margin, 'inflation': infl, 'slope_max_rad': slope}}
    payload, _ = tomogram_from_points(points, cfg)
    tomo = TomogramMap(payload, max_ground_step_m=0.17)
    geo = mapping_geometry(points, cfg)
    ground_raw, ceiling_raw = height_layers(points, geo, res, 0.5)
    raw = traversal_cost(ground_raw, ceiling_raw, res, cfg['traversability'])
    settings = validate_config(yaml.safe_load(open(ROUTE_CFG)))
    masked = _masked_tomogram(tomo, 'stair_lower', settings)
    trav, g, ele = native_arrays(masked)
    entry = settings['anchors']['entry']['xyz']
    ix, iy = tomo.index(entry[:2])
    layer = int(np.argmin(np.where(tomo.valid[:, ix, iy], np.abs(tomo.ground[:, ix, iy] - entry[2]), 9)))
    seen = reachable(trav, g, ele, (layer, ix, iy))
    zs = [g[n] for n in seen]
    print(f'res={res} margin={margin} infl={infl} slope={slope} start layer={layer} '
          f'reachable={len(seen)} max_ground={max(zs):.2f} gateway_cells_in_roi={(ele != 0).sum()}')
    poses = np.loadtxt(POSES)[:, [3, 7, 11]]
    reach_xy = {(i, j) for _, i, j in seen}
    print(' kf    xy            | layer: ground raw infl ceil  (R=reached, G=gateway)')
    for k in range(729, 830, 4):
        cx, cy = tomo.index(poses[k, :2])
        parts = []
        for l in range(tomo.layers):
            gr = tomo.ground[l, cx, cy]
            if not np.isfinite(gr) or abs(gr - (poses[k, 2] - 0.42)) > 0.35:
                continue
            ce = tomo.ceiling[l, cx, cy]
            mark = ('R' if (l, cx, cy) in seen else '.') + ('G' if ele[l, cx, cy] else ' ')
            parts.append(f'L{l}:{gr:+.2f} {raw[l, cx, cy]:4.1f} {tomo.cost[l, cx, cy]:4.1f} '
                         f'{ce - gr if np.isfinite(ce) else np.inf:4.2f}{mark}')
        flag = '*' if (cx, cy) in reach_xy else ' '
        print(f'{flag}{k} {poses[k, :2].round(2)} | ' + ' | '.join(parts))


if __name__ == '__main__':
    main()
