"""Side view of points inside the swept corridor of the recorded stair climb.

Any point in the robot body volume along a path the robot physically walked is
noise/ghost/self-return; this shows whether such points are what makes PCT
mark the stair foot and flight as barrier.
"""
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import open3d as o3d  # noqa: E402
from scipy.spatial import cKDTree  # noqa: E402

PCD = ('/home/dndx/智元四足机器人D1 Max二次开发文档资料包v0.1.0/d1max_ros2/map_manager/data/processed/'
       '20260923_141005_sc-pgo-0923-crossfloor-structure-v1_2c7218/processed_map.pcd')
POSES = ('/home/dndx/d1max_nav_ws/maps/runs/20260923_132601_150357_central_imu_faster_lio_sc_pgo_zenoh/'
         'sc_pgo/optimized_poses.txt')


def main():
    k0, k1, half = int(sys.argv[1]), int(sys.argv[2]), float(sys.argv[3])
    out = sys.argv[4]
    points = np.asarray(o3d.io.read_point_cloud(PCD).points, dtype=np.float32)
    poses = np.loadtxt(POSES)[:, [3, 7, 11]]
    track = poses[k0:k1 + 1]
    # Densify the polyline so nearest-sample distance approximates polyline distance.
    seg = np.linalg.norm(np.diff(track[:, :2], axis=0), axis=1)
    s_knots = np.concatenate([[0], np.cumsum(seg)])
    s = np.arange(0, s_knots[-1], 0.02)
    dense = np.stack([np.interp(s, s_knots, track[:, i]) for i in range(3)], 1)
    box = (np.all(points[:, :2] >= dense[:, :2].min(0) - 1, axis=1)
           & np.all(points[:, :2] <= dense[:, :2].max(0) + 1, axis=1))
    pts = points[box]
    dist, nn = cKDTree(dense[:, :2]).query(pts[:, :2])
    near = dist < half
    pts, nn = pts[near], nn[near]
    body = dense[nn, 2]
    rel = pts[:, 2] - (body - 0.42)  # height above the robot's own foot level
    fig, ax = plt.subplots(2, 1, figsize=(18, 10))
    ax[0].scatter(s[nn], pts[:, 2], c=rel, s=1, cmap='jet', vmin=-0.3, vmax=1.2)
    ax[0].plot(s, dense[:, 2], 'k-', lw=1, label='body z')
    ax[0].plot(s, dense[:, 2] - 0.42, 'k--', lw=1, label='foot z (body-0.42)')
    ax[0].set_ylim(dense[:, 2].min() - 1, dense[:, 2].max() + 1.5)
    for k in range(0, len(track), 6):
        ax[0].annotate(str(k0 + k), (s_knots[k], track[k, 2] + 0.2), fontsize=7)
    ax[0].legend()
    ax[0].set_title(f'points within {half} m of kf{k0}-{k1} track (x=arc length m)')
    ax[1].hist2d(s[nn], rel, bins=[int(s[-1] / 0.05), 60], range=[[0, s[-1]], [-0.3, 1.2]], cmin=1, cmap='viridis')
    ax[1].axhline(0.13, color='r', lw=0.6)
    ax[1].axhline(0.55, color='r', lw=0.6)
    ax[1].set_title('height above foot level vs arc length (red: PCT ceiling band that sets interval barrier)')
    fig.tight_layout()
    fig.savefig(out, dpi=90)
    inside = (rel > 0.10) & (rel < 0.55)
    print(f'points in corridor={len(pts)} in body band (0.10-0.55 m above foot)={inside.sum()}')
    for k in range(k0, k1 + 1, 3):
        m = inside & (np.abs(s[nn] - s_knots[k - k0]) < 0.15)
        print(f'kf{k} s={s_knots[k - k0]:5.2f} body-band points={m.sum():4d}')


if __name__ == '__main__':
    main()
