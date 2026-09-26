"""Point-cloud Z profile along the recorded stair flights + per-cell PCT rule flags.

Answers: are the stair treads clean steps in the fused SC-PGO PCD, and which
traversal rule (flat / crossable / interval) marks the flight cells as barrier.
"""
import sys

import numpy as np
import open3d as o3d

sys.path.insert(0, '/home/dndx/d1max_nav_ws/src/d1max_pct_planner')
from d1max_pct_planner.cpu_tomography import height_gradients, height_layers, mapping_geometry  # noqa: E402

PCD = ('/home/dndx/智元四足机器人D1 Max二次开发文档资料包v0.1.0/d1max_ros2/map_manager/data/processed/'
       '20260923_141005_sc-pgo-0923-crossfloor-structure-v1_2c7218/processed_map.pcd')
POSES = ('/home/dndx/d1max_nav_ws/maps/runs/20260923_132601_150357_central_imu_faster_lio_sc_pgo_zenoh/'
         'sc_pgo/optimized_poses.txt')
CROP = ([-40.0, -4.0, -3.0], [26.0, 60.0, 9.0])
BODY = 0.42


def main():
    res = float(sys.argv[1]) if len(sys.argv) > 1 else 0.10
    points = np.asarray(o3d.io.read_point_cloud(PCD).points, dtype=np.float32)
    points = points[np.all((points >= CROP[0]) & (points <= CROP[1]), axis=1)]
    poses = np.loadtxt(POSES)[:, [3, 7, 11]]
    track = poses[729:956]

    # 1) Raw point profile: for each pose, points within 0.15 m in XY, z within
    #    [ground-0.5, ground+0.3]; report count and z quantiles (step structure).
    print('kf   xy               est_ground | n   z05   z50   z95  spread')
    for k in range(729, 956, 3):
        p = poses[k]
        d = np.hypot(points[:, 0] - p[0], points[:, 1] - p[1])
        sel = points[(d < 0.15) & (points[:, 2] > p[2] - BODY - 0.5) & (points[:, 2] < p[2] - BODY + 0.3), 2]
        if len(sel) == 0:
            print(f'{k} {p[:2].round(2)} {p[2] - BODY:+.2f} | 0')
            continue
        q = np.quantile(sel, [0.05, 0.5, 0.95])
        print(f'{k} {p[:2].round(2)} {p[2] - BODY:+.2f} | {len(sel):4d} {q[0]:+.2f} {q[1]:+.2f} {q[2]:+.2f} {q[2] - q[0]:.2f}')

    # 2) PCT rule flags along the centreline at the layer whose ground matches.
    cfg = {'resolution': res, 'slice_dh': 0.5, 'ground_height': -3.0}
    geo = mapping_geometry(points, cfg)
    ground, ceiling = height_layers(points, geo, res, 0.5)
    grad_sq, grad_max = height_gradients(ground)
    stand_sq = (1.2 * res * np.tan(0.40)) ** 2
    center, dim = geo['center'], np.array(geo['shape'])
    print(f'\nres={res} stand_th={np.sqrt(stand_sq):.3f} m  cross_th=0.17 m')
    print('kf   layer ground  sqrt(grad_sq) sqrt(grad_max) flat  nflat7x7  interval  3x3 ground neighbourhood')
    from scipy import ndimage
    for k in range(729, 956, 6):
        p = poses[k]
        ix, iy = (np.floor((p[:2] - center) / res + .5).astype(int) + dim // 2)
        gz = ground[:, ix, iy]
        layer = int(np.argmin(np.abs(np.where(gz > -1e5, gz, 1e9) - (p[2] - BODY))))
        flat = grad_sq[layer] < stand_sq
        nflat = ndimage.convolve(flat[ix - 3:ix + 4, iy - 3:iy + 4].astype(int), np.ones((7, 7), int),
                                 mode='constant')[3, 3]
        nb = ground[layer, ix - 1:ix + 2, iy - 1:iy + 2]
        nbs = ' '.join(f'{v:+.2f}' if v > -1e5 else '  nan' for v in nb.ravel())
        print(f'{k} L{layer:2d} {gz[layer]:+.2f}  {np.sqrt(grad_sq[layer, ix, iy]):.3f}  '
              f'{np.sqrt(grad_max[layer, ix, iy]):.3f}  {int(grad_sq[layer, ix, iy] <= stand_sq)}  {nflat:3d}  '
              f'{ceiling[layer, ix, iy] - gz[layer]:5.2f}  [{nbs}]')


if __name__ == '__main__':
    main()
