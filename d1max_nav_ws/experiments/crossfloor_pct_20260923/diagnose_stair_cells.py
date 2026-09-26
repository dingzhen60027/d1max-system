"""Per-cell diagnosis of why PCT blocks the recorded stair (offline, read-only).

For each stair keyframe, pick the tomogram layer whose ground is nearest the
measured floor under the robot, then report which cost_barrier rule fired.
"""
import sys
from pathlib import Path

import numpy as np
import open3d as o3d
from scipy import ndimage

sys.path.insert(0, '/home/dndx/d1max_nav_ws/src/d1max_pct_planner')
from d1max_pct_planner.cpu_tomography import (height_gradients, height_layers,
                                              mapping_geometry, SENTINEL)

PCD = ('/home/dndx/智元四足机器人D1 Max二次开发文档资料包v0.1.0/d1max_ros2/map_manager/data/processed/'
       '20260923_141005_sc-pgo-0923-crossfloor-structure-v1_2c7218/processed_map.pcd')
POSES = ('/home/dndx/d1max_nav_ws/maps/runs/20260923_132601_150357_central_imu_faster_lio_sc_pgo_zenoh/'
         'sc_pgo/optimized_poses.txt')
BODY_TO_GROUND = 0.475  # entry sample: body -0.175, measured ground -0.650


def diagnose(points, cfg, stair_xyz):
    trav = cfg['traversability']
    geometry = mapping_geometry(points, cfg)
    res = float(cfg['resolution'])
    ground, ceiling = height_layers(points, geometry, res, float(cfg['slice_dh']))
    grad_sq, grad_max = height_gradients(ground)
    stand_sq = (1.2 * res * np.tan(float(trav['slope_max_rad']))) ** 2
    cross_sq = float(trav['step_max']) ** 2
    kernel = int(trav['kernel_size'])
    standable_th = int(float(trav['standable_ratio']) * kernel ** 2) - 1
    flat = grad_sq <= stand_sq
    neighbor_flat = np.stack([ndimage.convolve((grad_sq[l] < stand_sq).astype(np.int16),
                                               np.ones((kernel, kernel), np.int16), mode='constant')
                              for l in range(len(ground))])
    interval = ceiling - ground
    center = geometry['center']
    dim = np.array(geometry['shape'])
    rows = []
    for xyz in stair_xyz:
        ix, iy = (np.floor((xyz[:2] - center) / res + 0.5).astype(int) + dim // 2)
        target = xyz[2] - BODY_TO_GROUND
        column = ground[:, ix, iy]
        valid = column > -SENTINEL
        if not valid.any():
            rows.append(('no_ground', target, np.nan, np.nan, np.nan, 0)); continue
        layer = int(np.argmin(np.where(valid, np.abs(column - target), np.inf)))
        g, gs, gm, iv, nf = (column[layer], grad_sq[layer, ix, iy], grad_max[layer, ix, iy],
                             interval[layer, ix, iy], neighbor_flat[layer, ix, iy])
        if abs(g - target) > 0.25:
            reason = 'ground_mismatch'
        elif iv < float(trav['interval_min']):
            reason = 'interval_min'
        elif gs <= stand_sq:
            reason = 'flat_ok'
        elif gm > cross_sq:
            reason = 'step_max'
        elif nf < standable_th:
            reason = 'standable_ratio'
        else:
            reason = 'crossable_ok'
        rows.append((reason, target, g, np.sqrt(gs), iv, nf))
    return rows


def main():
    points = np.asarray(o3d.io.read_point_cloud(PCD).points, dtype=np.float32)
    roi_lo, roi_hi = np.array([-48, -12, -3.0]), np.array([35, 62, 15.5])
    points = points[np.all((points >= roi_lo) & (points <= roi_hi), axis=1)]
    poses = np.loadtxt(POSES)[:, [3, 7, 11]]
    stair = poses[729:955]
    base = {'resolution': 0.20, 'slice_dh': 0.50, 'ground_height': -3.0,
            'traversability': {'kernel_size': 7, 'interval_min': 0.55, 'interval_free': 0.70,
                               'slope_max_rad': 0.40, 'step_max': 0.17, 'standable_ratio': 0.20}}
    variants = [('current res0.20', {}),
                ('res0.10', {'resolution': 0.10}),
                ('res0.10 dh0.25', {'resolution': 0.10, 'slice_dh': 0.25}),
                ('res0.20 slope0.60', {'traversability': {'slope_max_rad': 0.60}})]
    for name, override in variants:
        cfg = {**base, **{k: v for k, v in override.items() if k != 'traversability'}}
        cfg['traversability'] = {**base['traversability'], **override.get('traversability', {})}
        rows = diagnose(points, cfg, stair)
        reasons = [r[0] for r in rows]
        counts = {k: reasons.count(k) for k in dict.fromkeys(reasons)}
        print(f'== {name}: {counts}')
        if name == 'current res0.20':
            for i in range(0, len(rows), 6):
                r = rows[i]
                print(f'   kf{729+i:4d} target={r[1]:6.2f} ground={r[2]:6.2f} grad={r[3]:5.3f} '
                      f'interval={r[4]:6.2f} nflat={r[5]:3d} -> {r[0]}')


if __name__ == '__main__':
    main()
