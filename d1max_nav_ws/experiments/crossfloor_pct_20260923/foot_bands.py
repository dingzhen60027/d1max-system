"""Top-down scatter of stair-foot points in height bands above the lower floor."""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d

PCD = ('/home/dndx/智元四足机器人D1 Max二次开发文档资料包v0.1.0/d1max_ros2/map_manager/data/processed/'
       '20260923_141005_sc-pgo-0923-crossfloor-structure-v1_2c7218/processed_map.pcd')
POSES = ('/home/dndx/d1max_nav_ws/maps/runs/20260923_132601_150357_central_imu_faster_lio_sc_pgo_zenoh/'
         'sc_pgo/optimized_poses.txt')
lo, hi = np.array([-33.0, 46.5]), np.array([-28.0, 52.5])
p = np.asarray(o3d.io.read_point_cloud(PCD).points)
p = p[np.all((p[:, :2] >= lo) & (p[:, :2] <= hi), axis=1)]
poses = np.loadtxt(POSES)[:, [3, 7, 11]]
floor = -0.63
bands = [(-0.10, 0.10), (0.10, 0.30), (0.30, 0.60), (0.60, 1.20), (1.20, 2.2), (2.2, 4.0)]
fig, axes = plt.subplots(2, 3, figsize=(21, 13))
for ax, (a, b) in zip(axes.ravel(), bands):
    m = (p[:, 2] - floor > a) & (p[:, 2] - floor <= b)
    sc = ax.scatter(p[m, 1], p[m, 0], c=p[m, 2] - floor, s=1, cmap='jet', vmin=a, vmax=b)
    fig.colorbar(sc, ax=ax, fraction=0.04)
    ax.plot(poses[700:960, 1], poses[700:960, 0], 'k-', lw=0.8)
    for k in range(700, 960, 10):
        ax.annotate(str(k), (poses[k, 1], poses[k, 0]), fontsize=6)
    ax.set_xlim(lo[1], hi[1]); ax.set_ylim(hi[0], lo[0]); ax.set_aspect('equal')
    ax.set_title(f'{a:+.2f}..{b:+.2f} m above lower floor  n={m.sum()}  (x down, y right)')
fig.tight_layout()
fig.savefig('/home/dndx/d1max_nav_ws/experiments/crossfloor_pct_20260923/foot_bands.png', dpi=80)
