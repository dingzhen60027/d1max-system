"""Zoom on the stair-foot corridor (entry -> first step): raw/inflated cost per
layer and point height above floor, to find what blocks L4 at kf733-745."""
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import open3d as o3d  # noqa: E402

sys.path.insert(0, '/home/dndx/d1max_nav_ws/src/d1max_pct_planner')
from d1max_pct_planner.cpu_tomography import _inflate, height_layers, mapping_geometry, traversal_cost  # noqa: E402

PCD = ('/home/dndx/智元四足机器人D1 Max二次开发文档资料包v0.1.0/d1max_ros2/map_manager/data/processed/'
       '20260923_141005_sc-pgo-0923-crossfloor-structure-v1_2c7218/processed_map.pcd')
POSES = ('/home/dndx/d1max_nav_ws/maps/runs/20260923_132601_150357_central_imu_faster_lio_sc_pgo_zenoh/'
         'sc_pgo/optimized_poses.txt')
CROP = ([-40.0, -4.0, -3.0], [26.0, 60.0, 9.0])


def main():
    res, slope, margin, infl, out = 0.10, float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]), sys.argv[4]
    points = np.asarray(o3d.io.read_point_cloud(PCD).points, dtype=np.float32)
    points = points[np.all((points >= CROP[0]) & (points <= CROP[1]), axis=1)]
    trav_cfg = {'kernel_size': 7, 'interval_min': 0.55, 'interval_free': 0.70, 'slope_max_rad': slope,
                'step_max': 0.17, 'standable_ratio': 0.20, 'cost_barrier': 50.0}
    cfg = {'resolution': res, 'slice_dh': 0.5, 'ground_height': -3.0}
    geo = mapping_geometry(points, cfg)
    ground, ceiling = height_layers(points, geo, res, 0.5)
    raw = traversal_cost(ground, ceiling, res, trav_cfg)
    center, dim = geo['center'], np.array(geo['shape'])
    lo, hi = np.array([-32.3, 46.8]), np.array([-28.5, 51.8])
    i0 = np.floor((lo - center) / res + .5).astype(int) + dim // 2
    i1 = np.floor((hi - center) / res + .5).astype(int) + dim // 2
    poses = np.loadtxt(POSES)[725:775, [3, 7, 11]]
    extent = [lo[1], hi[1], hi[0], lo[0]]
    # Height of the tallest point in (-0.55, 1.2] above the lower floor, per cell.
    sel = points[(points[:, 2] > -0.55) & (points[:, 2] < 1.2)
                 & np.all((points[:, :2] >= lo) & (points[:, :2] <= hi), axis=1)]
    hmap = np.full(i1 - i0, np.nan)
    idx = np.floor((sel[:, :2] - center) / res + .5).astype(int) + dim // 2 - i0
    ok = np.all((idx >= 0) & (idx < hmap.shape), axis=1)
    for (a, b), z in zip(idx[ok], sel[ok, 2]):
        hmap[a, b] = z if np.isnan(hmap[a, b]) else max(hmap[a, b], z)
    fig, axes = plt.subplots(2, 4, figsize=(20, 10))
    panels = [(hmap + 0.63, 'max point z above floor (-0.55..1.2)', dict(cmap='magma', vmin=0, vmax=1.8))]
    for l in (4, 5):
        panels += [(raw[l, i0[0]:i1[0], i0[1]:i1[1]], f'L{l} raw', dict(cmap='RdYlGn_r', vmin=0, vmax=50)),
                   (_inflate(raw[l], res, margin, infl)[i0[0]:i1[0], i0[1]:i1[1]], f'L{l} inflated',
                    dict(cmap='RdYlGn_r', vmin=0, vmax=50))]
    panels.append((ceiling[4, i0[0]:i1[0], i0[1]:i1[1]] - ground[4, i0[0]:i1[0], i0[1]:i1[1]], 'L4 interval',
                   dict(cmap='viridis', vmin=0, vmax=1.0)))
    for l in (4, 5):
        g = ground[l, i0[0]:i1[0], i0[1]:i1[1]]
        panels.append((np.where(g > -1e5, g, np.nan), f'L{l} ground', dict(cmap='viridis', vmin=-0.7, vmax=0.1)))
    for ax, (img, title, kw) in zip(axes.ravel(), panels):
        im = ax.imshow(img, extent=extent, interpolation='nearest', **kw)
        ax.plot(poses[:, 1], poses[:, 0], 'c.-', lw=0.8, ms=3)
        for k in range(0, len(poses), 4):
            ax.annotate(str(725 + k), (poses[k, 1], poses[k, 0]), color='c', fontsize=6)
        ax.set_title(title, fontsize=9)
        fig.colorbar(im, ax=ax, fraction=0.04)
    fig.suptitle(f'stair foot corridor res={res} slope={slope} m{margin}/i{infl} (x down, y right)')
    fig.tight_layout()
    fig.savefig(out, dpi=90)


if __name__ == '__main__':
    main()
