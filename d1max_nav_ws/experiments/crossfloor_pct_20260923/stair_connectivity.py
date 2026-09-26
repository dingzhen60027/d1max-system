"""Decisive test: is the recorded stair corridor CONNECTED at cost<=20?

Proxy for the body's own path: at each XY the robot is at height h (recorded body
z minus the measured body-to-ground offset). A cell is passable if some retained
layer whose measured ground is near h has cost <= 20. Then 8-connected flood fill
from the stair foot must reach the stair head without leaving the corridor.
"""
import sys
import numpy as np
import open3d as o3d
from scipy import ndimage

sys.path.insert(0, '/home/dndx/d1max_nav_ws/src/d1max_pct_planner')
from d1max_pct_planner.cpu_tomography import (SENTINEL, height_layers, mapping_geometry,
                                              traversal_cost, _inflate)

PCD = ('/home/dndx/智元四足机器人D1 Max二次开发文档资料包v0.1.0/d1max_ros2/map_manager/data/processed/'
       '20260923_141005_sc-pgo-0923-crossfloor-structure-v1_2c7218/processed_map.pcd')
POSES = ('/home/dndx/d1max_nav_ws/maps/runs/20260923_132601_150357_central_imu_faster_lio_sc_pgo_zenoh/'
         'sc_pgo/optimized_poses.txt')
BODY_TO_GROUND = 0.475


def build(points, cfg):
    geo = mapping_geometry(points, cfg)
    res, dh = float(cfg['resolution']), float(cfg['slice_dh'])
    ground, ceiling = height_layers(points, geo, res, dh)
    trav = traversal_cost(ground, ceiling, res, cfg['traversability'])
    cost = np.stack([_inflate(trav[l], res, float(cfg['traversability']['safe_margin']),
                              float(cfg['traversability']['inflation']))
                     for l in range(len(ground))])
    return geo, ground, cost


def connectivity(geo, ground, cost, stair_xyz, res, corridor=1.5):
    """Return (blocked_samples, connected, reach_fraction)."""
    center = geo['center']
    dim = np.array(geo['shape'])
    lo = (stair_xyz[:, :2].min(0) - corridor)
    hi = (stair_xyz[:, :2].max(0) + corridor)
    i0 = np.floor((lo - center) / res + 0.5).astype(int) + dim // 2
    i1 = np.ceil((hi - center) / res + 0.5).astype(int) + dim // 2
    i0, i1 = np.maximum(i0, 0), np.minimum(i1, dim)
    sub_g = ground[:, i0[0]:i1[0], i0[1]:i1[1]]
    sub_c = cost[:, i0[0]:i1[0], i0[1]:i1[1]]
    xs = center[0] + (np.arange(i0[0], i1[0]) - dim[0] // 2) * res
    ys = center[1] + (np.arange(i0[1], i1[1]) - dim[1] // 2) * res
    # interpolated body height on the corridor grid
    order = np.argsort(stair_xyz[:, 1])
    h = np.interp(ys, stair_xyz[order, 1], (stair_xyz[order, 2] - BODY_TO_GROUND))
    H = np.repeat(h[None, :], len(xs), axis=0)
    near = np.abs(sub_g - H[None]) <= 0.30
    passable = ((sub_c <= 20.0) & near).any(axis=0)
    blocked = 0
    for i in range(len(stair_xyz)):
        ix, iy = (np.floor((stair_xyz[i, :2] - center) / res + 0.5).astype(int) + dim // 2)
        jx, jy = ix - i0[0], iy - i0[1]
        if 0 <= jx < passable.shape[0] and 0 <= jy < passable.shape[1]:
            if not passable[jx, jy]:
                blocked += 1
    labels, _ = ndimage.label(passable, structure=np.ones((3, 3), int))
    def lab_of(xyz):
        ix, iy = (np.floor((xyz[:2] - center) / res + 0.5).astype(int) + dim // 2)
        jx, jy = ix - i0[0], iy - i0[1]
        return labels[jx, jy] if (0 <= jx < passable.shape[0] and 0 <= jy < passable.shape[1]) else 0
    a, b = lab_of(stair_xyz[0]), lab_of(stair_xyz[-1])
    sizes = np.bincount(labels.ravel())
    return blocked, (a != 0 and a == b), float(sizes[a] / passable.sum()) if a else 0.0


def main():
    points = np.asarray(o3d.io.read_point_cloud(PCD).points, dtype=np.float32)
    points = points[np.all((points >= [-48, -12, -3.0]) & (points <= [35, 62, 15.5]), axis=1)]
    poses = np.loadtxt(POSES)[:, [3, 7, 11]]
    stair = poses[729:955]
    base = {'slice_dh': 0.50, 'ground_height': -3.0,
            'traversability': {'kernel_size': 7, 'interval_min': 0.55, 'interval_free': 0.70,
                               'slope_max_rad': 0.40, 'step_max': 0.17, 'standable_ratio': 0.20,
                               'cost_barrier': 50.0, 'safe_margin': 0.20, 'inflation': 0.20}}
    variants = [
        ('A current res.20 int.55 infl.20', {'resolution': 0.20}),
        ('B int.10 + infl.05', {'resolution': 0.20, 'interval_min': 0.10, 'inflation': 0.05}),
        ('C int.10 infl.05 slope.60 step.20', {'resolution': 0.20, 'interval_min': 0.10,
                                               'inflation': 0.05, 'slope_max_rad': 0.60,
                                               'step_max': 0.20}),
        ('D int.10 infl.10 slope.60 step.20', {'resolution': 0.20, 'interval_min': 0.10,
                                               'inflation': 0.10, 'slope_max_rad': 0.60,
                                               'step_max': 0.20}),
    ]
    for name, ov in variants:
        cfg = {'resolution': ov.get('resolution', 0.20), 'slice_dh': base['slice_dh'],
               'ground_height': base['ground_height'],
               'traversability': {**base['traversability'],
                                  **{k: v for k, v in ov.items()
                                     if k in base['traversability']}}}
        for extra in ('inflation', 'safe_margin'):
            if extra in ov:
                cfg['traversability'][extra] = ov[extra]
            elif extra not in base['traversability']:
                cfg['traversability'][extra] = 0.0 if extra == 'safe_margin' else 0.20
        geo, ground, cost = build(points, cfg)
        blocked, connected, frac = connectivity(geo, ground, cost, stair, cfg['resolution'])
        print(f'{name:42s} blocked={blocked:3d}/{len(stair)}  connected={str(connected):5s} '
              f'largest_component_share={frac:5.1%}', flush=True)


if __name__ == '__main__':
    main()
