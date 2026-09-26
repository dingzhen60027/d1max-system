"""Independent visibility audit: exact dual optics, exclusive hit/free, full upper corridor.

The saved SC-PGO scans no longer carry the original lidar ID. Membership is
therefore inferred conservatively from the two hemispherical optical axes;
points within 8 cm of their half-space boundary are ignored. This is not a
factory calibration or an exact reconstruction of per-beam deskewed origins.
"""
import hashlib
import json
import time
from pathlib import Path

import numpy as np
import open3d as o3d
import yaml
from scipy.spatial.transform import Rotation

from visibility_clean import RUN, PCD, clip_to_box

ROOT = Path(__file__).resolve().parent


def main():
    low, high = np.array([-35., 43., -1.4]), np.array([-19., 58., 4.9])
    voxel, step, near, end_margin = .10, .04, .30, .20
    dims = np.ceil((high - low) / voxel).astype(int)
    nvox = int(np.prod(dims))
    free, hit = np.zeros(nvox, np.int32), np.zeros(nvox, np.int32)
    cal = yaml.safe_load(Path(RUN, 'config/calibration.yaml').read_text())['lio_extrinsic']
    r_cn, front_c = np.array(cal['rotation']).reshape(3, 3), np.array(cal['translation'])
    # Same geometry used by the recorded mapping run, retained here verbatim.
    r_nf = Rotation.from_quat([-.499867275, .503186620, .497953310, .498977388]).as_matrix()
    rear_c = r_cn @ r_nf @ np.array([0., 0., -.7323]) + front_c
    optical = r_cn @ r_nf @ np.array([0., 0., 1.])
    poses = np.loadtxt(Path(RUN, 'sc_pgo/optimized_poses.txt')).reshape(-1, 3, 4)
    distance = np.linalg.norm(np.maximum(np.maximum(low - poses[:, :, 3], poses[:, :, 3] - high), 0), axis=1)
    frames = np.flatnonzero(distance < 15.)
    started = time.time()
    total, assigned = 0, 0

    def ids(points):
        idx = np.floor((points - low) / voxel).astype(np.int64)
        ok = np.all((idx >= 0) & (idx < dims), axis=1)
        return np.ravel_multi_index(idx[ok].T, dims), ok

    for count, k in enumerate(frames):
        scan = np.asarray(o3d.io.read_point_cloud(f'{RUN}/sc_pgo/Scans/{k:06d}.pcd').points)
        front = (scan - front_c) @ optical > .08
        rear = (scan - rear_c) @ optical < -.08
        use = front ^ rear
        total += len(scan)
        assigned += int(use.sum())
        scan, front = scan[use], front[use]
        rot, trans = poses[k, :, :3], poses[k, :, 3]
        origin_c = np.where(front[:, None], front_c, rear_c)
        world, origin = scan @ rot.T + trans, origin_c @ rot.T + trans
        hid, _ = ids(world)
        hmask = np.zeros(nvox, bool)
        hmask[hid] = True
        ti, to = clip_to_box(origin, world, low, high)
        length = np.linalg.norm(world - origin, axis=1)
        a, b = np.maximum(ti * length, near), np.minimum(to * length, length - end_margin)
        keep = b > a
        direction = ((world - origin) / length[:, None])[keep]
        a, b, origin = a[keep], b[keep], origin[keep]
        n = np.ceil((b - a) / step).astype(np.int64)
        fmask = np.zeros(nvox, bool)
        for chunk in np.array_split(np.arange(len(n)), max(1, int(n.sum() // 4_000_000) + 1)):
            nn = n[chunk]
            rep = np.repeat(chunk, nn)
            offset = np.arange(nn.sum()) - np.repeat(np.cumsum(nn) - nn, nn)
            dist = np.minimum(a[rep] + (offset + .5) * step, b[rep])
            fid, _ = ids(origin[rep] + direction[rep] * dist[:, None])
            fmask[fid] = True
        # A voxel with a return in this same frame is not free evidence.
        free += fmask & ~hmask
        hit += hmask
        if count % 25 == 0:
            print(f'kf {k}: {count+1}/{len(frames)}, elapsed {time.time()-started:.1f}s', flush=True)
    points = np.asarray(o3d.io.read_point_cloud(PCD).points)
    pid, ok = ids(points)
    point_free, point_hit = np.zeros(len(points), np.int32), np.zeros(len(points), np.int32)
    point_free[ok], point_hit[ok] = free[pid], hit[pid]
    digest = hashlib.sha256(Path(PCD).read_bytes()).hexdigest()
    np.savez_compressed(ROOT / 'visibility_votes_review.npz', free=point_free, hit=point_hit,
                        in_roi=ok, roi_low=low, roi_high=high, voxel=voxel, frames=frames,
                        source_sha256=digest, point_count=len(points))
    report = {'source_pcd': PCD, 'source_sha256': digest, 'roi': [low.tolist(), high.tolist()],
              'frames': len(frames), 'elapsed_s': time.time()-started,
              'front_origin_c': front_c.tolist(), 'rear_origin_c': rear_c.tolist(),
              'front_optical_axis_c': optical.tolist(), 'return_margin_m': end_margin,
              'ignored_ambiguous_point_fraction': 1-assigned/total,
              'candidate_count': int(np.sum(ok & (point_free >= 10*np.maximum(point_hit, 1)) & (point_free >= 20)))}
    (ROOT / 'visibility_votes_review.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2), flush=True)


if __name__ == '__main__':
    main()
