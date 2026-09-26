"""Final: (1) smallest viable envelope, (2) gateway cells prove cross-floor mechanism."""
import sys
import numpy as np
import open3d as o3d
from scipy import ndimage

sys.path.insert(0, '/home/dndx/d1max_nav_ws/src/d1max_pct_planner')
from d1max_pct_planner.cpu_tomography import (SENTINEL, _inflate, height_layers,
                                              mapping_geometry, traversal_cost)

PCD = ('/home/dndx/智元四足机器人D1 Max二次开发文档资料包v0.1.0/d1max_ros2/map_manager/data/processed/'
       '20260923_141005_sc-pgo-0923-crossfloor-structure-v1_2c7218/processed_map.pcd')
POSES = ('/home/dndx/d1max_nav_ws/maps/runs/20260923_132601_150357_central_imu_faster_lio_sc_pgo_zenoh/'
         'sc_pgo/optimized_poses.txt')
TOMO = '/home/dndx/d1max_nav_ws/maps/processed/sc_pgo_20260923_pct_crossfloor/tomogram.npz'
BODY_TO_GROUND = 0.475


def main():
    points = np.asarray(o3d.io.read_point_cloud(PCD).points, dtype=np.float32)
    points = points[np.all((points >= [-48, -12, -3.0]) & (points <= [35, 62, 15.5]), axis=1)]
    poses = np.loadtxt(POSES)[:, [3, 7, 11]]
    stair = poses[729:955]

    print('--- (1) inflate radius = int((safe_margin+inflation)/resolution) ---')
    base = {'kernel_size': 7, 'interval_min': 0.10, 'interval_free': 0.70, 'slope_max_rad': 0.60,
            'step_max': 0.20, 'standable_ratio': 0.20, 'cost_barrier': 50.0}
    for margin, infl in [(0.03, 0.03), (0.06, 0.06), (0.10, 0.10), (0.15, 0.15), (0.20, 0.20)]:
        cfg = {'resolution': 0.20, 'slice_dh': 0.50, 'ground_height': -3.0,
               'traversability': {**base, 'safe_margin': margin, 'inflation': infl}}
        geo = mapping_geometry(points, cfg)
        ground, ceiling = height_layers(points, geo, 0.20, 0.50)
        trav = traversal_cost(ground, ceiling, 0.20, cfg['traversability'])
        cost = np.stack([_inflate(trav[l], 0.20, margin, infl) for l in range(len(ground))])
        center, dim = geo['center'], np.array(geo['shape'])
        lo, hi = stair[:, :2].min(0) - 1.5, stair[:, :2].max(0) + 1.5
        i0 = np.floor((lo - center) / 0.20 + 0.5).astype(int) + dim // 2
        i1 = np.ceil((hi - center) / 0.20 + 0.5).astype(int) + dim // 2
        i0, i1 = np.maximum(i0, 0), np.minimum(i1, dim)
        sg = ground[:, i0[0]:i1[0], i0[1]:i1[1]]
        sc = cost[:, i0[0]:i1[0], i0[1]:i1[1]]
        ys = center[1] + (np.arange(i0[1], i1[1]) - dim[1] // 2) * 0.20
        o = np.argsort(stair[:, 1])
        hh = np.interp(ys, stair[o, 1], stair[o, 2] - BODY_TO_GROUND)
        H = np.repeat(hh[None, :], sg.shape[1], axis=0)
        passable = ((sc <= 20.0) & (np.abs(sg - H[None]) <= 0.30)).any(axis=0)
        labels, _ = ndimage.label(passable, structure=np.ones((3, 3), int))
        def lab(p):
            ix, iy = np.floor((p[:2] - center) / 0.20 + 0.5).astype(int) + dim // 2
            jx, jy = ix - i0[0], iy - i0[1]
            return labels[jx, jy] if 0 <= jx < passable.shape[0] and 0 <= jy < passable.shape[1] else 0
        a, b = lab(stair[0]), lab(stair[-1])
        nb = 0
        for p in stair:
            jx = int(np.floor((p[0] - center[0]) / 0.20 + 0.5) + dim[0] // 2 - i0[0])
            jy = int(np.floor((p[1] - center[1]) / 0.20 + 0.5) + dim[1] // 2 - i0[1])
            if not (0 <= jx < passable.shape[0] and 0 <= jy < passable.shape[1]) or not passable[jx, jy]:
                nb += 1
        r = int((margin + infl) / 0.20)
        print(f'  margin/infl={margin:.2f}/{infl:.2f} radius={r} cells -> blocked={nb:3d}/226 '
              f'connected={mbool(a, b)}', flush=True)

    print()
    print('--- (2) gateway mechanism in the existing crossfloor tomogram ---')
    with np.load(TOMO) as a:
        data, res, sh0, sdh = a['data'], float(a['resolution']), float(a['slice_h0']), float(a['slice_dh'])
    trav, eg, ec = data[0], data[3], data[4]
    eg = np.nan_to_num(eg, nan=-100.0)
    diff_t = trav[1:] - trav[:-1]
    diff_g = np.abs(eg[1:] - eg[:-1])
    up = (diff_t < -8.0)
    dn = (diff_t > 8.0)
    both = up | dn
    ground_same = diff_g < 0.1
    print(f'  layers={trav.shape[0]} shape={trav.shape[1:]} res={res}')
    print(f'  adjacent-layer pairs with |cost drop|>8 : {int(both.sum())}')
    print(f'  ...of those with ground change <0.1 m   : {int((both & ground_same).sum())}')
    print(f'  => gateway cells actually usable        : {int((both & ground_same).sum())}')
    gw = np.zeros_like(trav, dtype=int)
    gwu = np.zeros_like(trav, dtype=bool); gwu[:-1] = (diff_t < -8.0) & ground_same & (eg[1:] > -99)
    gwd = np.zeros_like(trav, dtype=bool); gwd[1:] = (diff_t > 8.0) & ground_same & (eg[:-1] > -99)
    gw[gwu], gw[gwd] = 2, -2
    stair_lo, stair_hi = np.array([-34, 47]), np.array([-27, 56])
    ix0 = np.floor((stair_lo - a['center']) / res + 0.5).astype(int) + np.array(trav.shape[1:]) // 2
    ix1 = np.floor((stair_hi - a['center']) / res + 0.5).astype(int) + np.array(trav.shape[1:]) // 2
    sub = gw[:, ix0[0]:ix1[0], ix0[1]:ix1[1]]
    print(f'  gateway cells inside the stair ROI      : {int((sub != 0).sum())}')


def mbool(a, b):
    return a != 0 and a == b


if __name__ == '__main__':
    main()
