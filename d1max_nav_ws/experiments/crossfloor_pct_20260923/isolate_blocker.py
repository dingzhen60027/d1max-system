"""Isolate the dominant blocker on the recorded stair: raw cost vs inflation."""
import sys
import numpy as np
import open3d as o3d

sys.path.insert(0, '/home/dndx/d1max_nav_ws/src/d1max_pct_planner')
from d1max_pct_planner.cpu_tomography import (SENTINEL, _inflate, height_layers,
                                              mapping_geometry, traversal_cost)

PCD = ('/home/dndx/智元四足机器人D1 Max二次开发文档资料包v0.1.0/d1max_ros2/map_manager/data/processed/'
       '20260923_141005_sc-pgo-0923-crossfloor-structure-v1_2c7218/processed_map.pcd')
POSES = ('/home/dndx/d1max_nav_ws/maps/runs/20260923_132601_150357_central_imu_faster_lio_sc_pgo_zenoh/'
         'sc_pgo/optimized_poses.txt')
BODY_TO_GROUND = 0.475


def run(points, res, dh, trav_cfg, margin, inflation, stair, label):
    cfg = {'resolution': res, 'slice_dh': dh, 'ground_height': -3.0, 'traversability': trav_cfg}
    geo = mapping_geometry(points, cfg)
    ground, ceiling = height_layers(points, geo, res, dh)
    trav = traversal_cost(ground, ceiling, res, trav_cfg)
    if margin is None:
        cost = trav
    else:
        cost = np.stack([_inflate(trav[l], res, margin, inflation) for l in range(len(ground))])
    center, dim = geo['center'], np.array(geo['shape'])
    hits = ok_near = ok_raw = 0
    for xyz in stair:
        ix, iy = (np.floor((xyz[:2] - center) / res + 0.5).astype(int) + dim // 2)
        col_g, col_c, col_t = ground[:, ix, iy], cost[:, ix, iy], trav[:, ix, iy]
        target = xyz[2] - BODY_TO_GROUND
        valid = col_g > -SENTINEL
        if not valid.any():
            continue
        near = valid & (np.abs(col_g - target) <= 0.30)
        if near.any():
            ok_near += 1
            if (col_t[near] <= 20.0).any():
                ok_raw += 1
            if (col_c[near] <= 20.0).any():
                hits += 1
    print(f'{label:46s} near={ok_near:3d}/{len(stair)}  raw_cost<=20={ok_raw:3d}  '
          f'after_inflation={hits:3d}', flush=True)


def main():
    points = np.asarray(o3d.io.read_point_cloud(PCD).points, dtype=np.float32)
    points = points[np.all((points >= [-48, -12, -3.0]) & (points <= [35, 62, 15.5]), axis=1)]
    poses = np.loadtxt(POSES)[:, [3, 7, 11]]
    stair = poses[729:955]
    trav = {'kernel_size': 7, 'interval_min': 0.10, 'interval_free': 0.70, 'slope_max_rad': 0.60,
            'step_max': 0.20, 'standable_ratio': 0.20, 'cost_barrier': 50.0,
            'safe_margin': 0.20, 'inflation': 0.20}
    for margin, infl, tag in [(None, 0.0, 'NO inflation'),
                              (0.05, 0.05, 'margin/inflation 0.05'),
                              (0.10, 0.10, 'margin/inflation 0.10'),
                              (0.20, 0.20, 'margin/inflation 0.20 (current)')]:
        run(points, 0.20, 0.50, trav, margin, infl, stair, f'res0.20 interval0.10 {tag}')


if __name__ == '__main__':
    main()
