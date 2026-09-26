"""Removert/ERASOR-style free-space voting for the stair region.

Every SC-PGO keyframe scan (sc_pgo/Scans, IMU/body frame C, optimised pose in
optimized_poses.txt) is ray-cast from its real sensor origin: the D1 Max has a
front and a rear hemispherical RoboSense Airy; in C the front sensor is at
t_C_N and the rear one 0.7323 m behind it, and each point belongs to the
hemisphere it lies in. For every 0.1 m voxel we count, per keyframe, whether a
ray passed through it (free) or ended in it (hit). A map point whose voxel was
seen through by many keyframes and hit by few is a transient (the operator
following the robot, etc.). Static surfaces are hit whenever they are seen.

usage: visibility_clean.py [out.npz]   (offline; writes a keep-mask for the PCD)
"""
import sys
import time

import numpy as np
import open3d as o3d
import yaml

RUN = '/home/dndx/d1max_nav_ws/maps/runs/20260923_132601_150357_central_imu_faster_lio_sc_pgo_zenoh'
PCD = ('/home/dndx/智元四足机器人D1 Max二次开发文档资料包v0.1.0/d1max_ros2/map_manager/data/processed/'
       '20260923_141005_sc-pgo-0923-crossfloor-structure-v1_2c7218/processed_map.pcd')
ROI = (np.array([-34.5, 44.0, -1.4]), np.array([-25.5, 57.5, 4.9]))
VOXEL = 0.10
STEP = 0.04            # ray sample step (m)
NEAR = 0.30            # ignore the first 0.3 m of every ray (robot body / blind zone)
END_MARGIN = 0.15      # stop free carving this far before the measured return
KF_RADIUS = 15.0       # keyframes whose origin is within this distance of the ROI


def sensor_origins():
    cal = yaml.safe_load(open(f'{RUN}/config/calibration.yaml'))['lio_extrinsic']
    r_cn = np.array(cal['rotation']).reshape(3, 3)
    t_cn = np.array(cal['translation'])
    rear_n = np.array([-0.003, -0.7323, 0.003])     # rear_to_front [0,0,-0.7323] in normalized N axes
    return t_cn, r_cn @ rear_n + t_cn


def clip_to_box(origin, end, low, high):
    """Parametric [t_in, t_out] of segments origin->end inside the box."""
    d = end - origin
    with np.errstate(divide='ignore', invalid='ignore'):
        t0 = (low - origin) / d
        t1 = (high - origin) / d
    tmin = np.where(d != 0, np.minimum(t0, t1), np.where((origin >= low) & (origin <= high), -np.inf, np.inf))
    tmax = np.where(d != 0, np.maximum(t0, t1), np.where((origin >= low) & (origin <= high), np.inf, -np.inf))
    return np.maximum(tmin.max(axis=1), 0.0), np.minimum(tmax.min(axis=1), 1.0)


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else 'visibility_votes.npz'
    low, high = ROI
    dims = np.ceil((high - low) / VOXEL).astype(int)
    nvox = int(np.prod(dims))
    free = np.zeros(nvox, np.int32)
    hit = np.zeros(nvox, np.int32)
    front_c, rear_c = sensor_origins()
    poses = np.loadtxt(f'{RUN}/sc_pgo/optimized_poses.txt').reshape(-1, 3, 4)
    box_dist = np.linalg.norm(np.maximum(np.maximum(low - poses[:, :, 3], poses[:, :, 3] - high), 0), axis=1)
    frames = np.flatnonzero(box_dist < KF_RADIUS)
    print(f'grid {dims.tolist()} keyframes {len(frames)} ({frames.min()}..{frames.max()})', flush=True)
    started = time.time()

    def vid(p):
        idx = np.floor((p - low) / VOXEL).astype(np.int64)
        ok = np.all((idx >= 0) & (idx < dims), axis=1)
        return idx, ok

    for count, k in enumerate(frames):
        scan = np.asarray(o3d.io.read_point_cloud(f'{RUN}/sc_pgo/Scans/{k:06d}.pcd').points)
        rot, trans = poses[k, :, :3], poses[k, :, 3]
        y = scan[:, 1]
        front, rear = y < front_c[1], y > rear_c[1]
        use = front | rear
        scan, front = scan[use], front[use]
        origin_c = np.where(front[:, None], front_c, rear_c)
        world = scan @ rot.T + trans
        origin = origin_c @ rot.T + trans
        # hits
        idx, ok = vid(world)
        hid = np.ravel_multi_index(idx[ok].T, dims)
        hmask = np.zeros(nvox, bool)
        hmask[hid] = True
        # free carving along clipped rays
        t_in, t_out = clip_to_box(origin, world, low, high)
        length = np.linalg.norm(world - origin, axis=1)
        a = np.maximum(t_in * length, NEAR)
        b = np.minimum(t_out * length, length - END_MARGIN)
        keep = b > a
        a, b, origin, direction = a[keep], b[keep], origin[keep], ((world - origin) / length[:, None])[keep]
        n = np.ceil((b - a) / STEP).astype(np.int64)
        fmask = np.zeros(nvox, bool)
        for chunk in np.array_split(np.arange(len(n)), max(1, int(n.sum() // 4_000_000) + 1)):
            nn = n[chunk]
            rep = np.repeat(chunk, nn)
            offs = np.arange(nn.sum()) - np.repeat(np.cumsum(nn) - nn, nn)
            dist = a[rep] + (offs + 0.5) * STEP
            dist = np.minimum(dist, b[rep])
            p = origin[rep] + direction[rep] * dist[:, None]
            idx, ok = vid(p)
            fmask[np.ravel_multi_index(idx[ok].T, dims)] = True
        free += fmask
        hit += hmask
        if count % 25 == 0:
            print(f'  kf {k} ({count + 1}/{len(frames)}) {time.time() - started:.0f}s', flush=True)

    points = np.asarray(o3d.io.read_point_cloud(PCD).points)
    idx, ok = vid(points)
    pid = np.full(len(points), -1, np.int64)
    pid[ok] = np.ravel_multi_index(idx[ok].T, dims)
    pf = np.where(ok, free[np.maximum(pid, 0)], 0)
    ph = np.where(ok, hit[np.maximum(pid, 0)], 0)
    np.savez_compressed(out, free=pf, hit=ph, in_roi=ok, roi_low=low, roi_high=high, voxel=VOXEL,
                        frames=frames)
    print(f'saved {out}: roi points {ok.sum()} in {time.time() - started:.0f}s')


if __name__ == '__main__':
    main()
