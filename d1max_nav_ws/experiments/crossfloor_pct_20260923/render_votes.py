"""Top/side views of points the free-space vote would remove (ratio rule)."""
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import open3d as o3d  # noqa: E402

PCD = ('/home/dndx/智元四足机器人D1 Max二次开发文档资料包v0.1.0/d1max_ros2/map_manager/data/processed/'
       '20260923_141005_sc-pgo-0923-crossfloor-structure-v1_2c7218/processed_map.pcd')
POSES = ('/home/dndx/d1max_nav_ws/maps/runs/20260923_132601_150357_central_imu_faster_lio_sc_pgo_zenoh/'
         'sc_pgo/optimized_poses.txt')
ratio, fmin, out = float(sys.argv[1]), int(sys.argv[2]), sys.argv[3]
p = np.asarray(o3d.io.read_point_cloud(PCD).points)
v = np.load('visibility_votes.npz')
F, H, roi = v['free'], v['hit'], v['in_roi']
rm = roi & (F >= ratio * np.maximum(H, 1)) & (F >= fmin)
protect = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
if protect > 0:
    # Keep candidates touching a persistent surface (misaligned wall/floor passes).
    from scipy.spatial import cKDTree
    persistent = roi & (H >= 10) & (F < 3 * H)
    near = cKDTree(p[persistent]).query_ball_point(p[rm], protect, return_length=True) > 0
    idx = np.flatnonzero(rm)
    print(f'protected {near.sum()} of {len(idx)} candidates within {protect} m of persistent points')
    rm[idx[near]] = False
print(f'remove {rm.sum()} of {roi.sum()} roi points ({rm.sum() / roi.sum():.2%})')
poses = np.loadtxt(POSES)[690:986, [3, 7, 11]]
keep = roi & ~rm
fig, ax = plt.subplots(2, 3, figsize=(24, 15))
bands = [(-1.4, 0.2, 'lower floor band z<0.2'), (0.2, 2.0, 'stair/landing band 0.2..2.0'), (2.0, 4.9, 'upper band z>2')]
for c, (z0, z1, title) in enumerate(bands):
    k = keep & (p[:, 2] >= z0) & (p[:, 2] < z1)
    r = rm & (p[:, 2] >= z0) & (p[:, 2] < z1)
    ax[0, c].scatter(p[k, 1], p[k, 0], s=0.2, c='0.6')
    sc = ax[0, c].scatter(p[r, 1], p[r, 0], s=1.5, c=p[r, 2], cmap='jet', vmin=z0, vmax=z1)
    ax[0, c].plot(poses[:, 1], poses[:, 0], 'k-', lw=0.6)
    ax[0, c].set_xlim(44, 57.5); ax[0, c].set_ylim(-25.5, -34.5)
    ax[0, c].set_title(f'{title}: removed={r.sum()} (colour=z)')
    fig.colorbar(sc, ax=ax[0, c], fraction=0.03)
# side views: y-z for the two flights (x bands)
for c, (x0, x1, title) in enumerate([(-30.0, -28.3, 'lower flight x -30..-28.3'), (-31.6, -30.0, 'upper flight x -31.6..-30')]):
    k = keep & (p[:, 0] >= x0) & (p[:, 0] < x1)
    r = rm & (p[:, 0] >= x0) & (p[:, 0] < x1)
    ax[1, c].scatter(p[k, 1], p[k, 2], s=0.2, c='0.6')
    ax[1, c].scatter(p[r, 1], p[r, 2], s=1.5, c='r')
    ax[1, c].plot(poses[:, 1], poses[:, 2] - 0.42, 'k--', lw=0.6)
    ax[1, c].set_title(f'side {title}: red removed'); ax[1, c].set_xlim(44, 57.5)
h = np.log10(np.maximum(F[roi], 1) / np.maximum(H[roi], 1))
ax[1, 2].hist(h, bins=100); ax[1, 2].set_yscale('log'); ax[1, 2].axvline(np.log10(ratio), color='r')
ax[1, 2].set_title('log10(F/H) over ROI points')
fig.tight_layout(); fig.savefig(out, dpi=80)
