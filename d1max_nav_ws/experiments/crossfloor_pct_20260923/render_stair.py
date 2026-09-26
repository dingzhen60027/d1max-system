"""Render raw vs inflated PCT cost per layer in the stair ROI (PNG)."""
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import open3d as o3d  # noqa: E402

sys.path.insert(0, '/home/dndx/d1max_nav_ws/src/d1max_pct_planner')
from d1max_pct_planner.cpu_tomography import (_inflate, height_gradients, height_layers,  # noqa: E402
                                              mapping_geometry, traversal_cost)

PCD = ('/home/dndx/智元四足机器人D1 Max二次开发文档资料包v0.1.0/d1max_ros2/map_manager/data/processed/'
       '20260923_141005_sc-pgo-0923-crossfloor-structure-v1_2c7218/processed_map.pcd')
POSES = ('/home/dndx/d1max_nav_ws/maps/runs/20260923_132601_150357_central_imu_faster_lio_sc_pgo_zenoh/'
         'sc_pgo/optimized_poses.txt')
CROP = ([-40.0, -4.0, -3.0], [26.0, 60.0, 9.0])
TRAV = {'kernel_size': 7, 'interval_min': 0.55, 'interval_free': 0.70, 'slope_max_rad': 0.40,
        'step_max': 0.17, 'standable_ratio': 0.20, 'cost_barrier': 50.0}


def main():
    res, margin, infl = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
    out = sys.argv[4]
    points = np.asarray(o3d.io.read_point_cloud(PCD).points, dtype=np.float32)
    points = points[np.all((points >= CROP[0]) & (points <= CROP[1]), axis=1)]
    cfg = {'resolution': res, 'slice_dh': 0.5, 'ground_height': -3.0,
           'traversability': {**TRAV, 'safe_margin': margin, 'inflation': infl}}
    geo = mapping_geometry(points, cfg)
    ground, ceiling = height_layers(points, geo, res, 0.5)
    raw = traversal_cost(ground, ceiling, res, cfg['traversability'])
    center, dim = geo['center'], np.array(geo['shape'])
    lo, hi = np.array([-33.5, 46.5]), np.array([-26.5, 57.0])
    i0 = np.floor((lo - center) / res + .5).astype(int) + dim // 2
    i1 = np.floor((hi - center) / res + .5).astype(int) + dim // 2
    poses = np.loadtxt(POSES)[729:955, [3, 7, 11]]
    layers = list(range(4, 13))
    fig, axes = plt.subplots(3, len(layers), figsize=(3.0 * len(layers), 13))
    extent = [lo[1], hi[1], hi[0], lo[0]]
    for c, l in enumerate(layers):
        g = ground[l, i0[0]:i1[0], i0[1]:i1[1]]
        gshow = np.where(g > -1e5, g, np.nan)
        r = raw[l, i0[0]:i1[0], i0[1]:i1[1]]
        f = _inflate(raw[l], res, margin, infl)[i0[0]:i1[0], i0[1]:i1[1]]
        slice_z = geo['slice_h0'] + l * 0.5
        near = np.abs(poses[:, 2] - 0.42 - (slice_z - 0.25)) < 0.5
        for row, (img, title, kw) in enumerate([
                (gshow, f'L{l} ground (slice {slice_z:+.1f})', dict(cmap='viridis', vmin=-0.7, vmax=3.3)),
                (np.where(g > -1e5, r, np.nan), 'raw cost', dict(cmap='RdYlGn_r', vmin=0, vmax=50)),
                (np.where(g > -1e5, f, np.nan), f'inflated m{margin}/i{infl}', dict(cmap='RdYlGn_r', vmin=0, vmax=50))]):
            ax = axes[row, c]
            ax.imshow(img, extent=extent, interpolation='nearest', **kw)
            if row:
                ax.contour(np.linspace(lo[1], hi[1], img.shape[1]), np.linspace(lo[0], hi[0], img.shape[0]),
                           np.nan_to_num(img, nan=50) <= 20, levels=[0.5], colors='k', linewidths=0.4)
            ax.plot(poses[:, 1], poses[:, 0], 'w-', lw=0.6)
            ax.plot(poses[near, 1], poses[near, 0], 'm.', ms=2)
            ax.set_title(title, fontsize=8)
            ax.tick_params(labelsize=6)
    fig.suptitle(f'stair ROI res={res} (x down, y right); white=trajectory, magenta=poses near slice')
    fig.tight_layout()
    fig.savefig(out, dpi=90)


if __name__ == '__main__':
    main()
