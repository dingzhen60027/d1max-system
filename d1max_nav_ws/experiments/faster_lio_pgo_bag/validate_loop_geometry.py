"""Read-only geometry checks; never treats a nonzero loop counter as accuracy.

Requires saved keyframe scans. Writes only a new report inside that run, with
an explicit name supplied by the caller; never overwrites maps or trajectories.
Run with OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 /usr/bin/python3 ...
"""
import argparse
import csv
import json
from collections import Counter
from pathlib import Path

import numpy as np
import open3d as o3d


def poses(path):
    rows = np.atleast_2d(np.loadtxt(path)).reshape(-1, 3, 4)
    matrices = np.repeat(np.eye(4)[None], len(rows), axis=0)
    matrices[:, :3, :] = rows
    return matrices


def cloud(path, voxel=0.15):
    value = o3d.io.read_point_cloud(str(path))
    xyz = np.asarray(value.points)
    mask = np.isfinite(xyz).all(axis=1) & (np.linalg.norm(xyz, axis=1) < 25)
    value = value.select_by_index(np.flatnonzero(mask))
    return value.voxel_down_sample(voxel)


def floor_normal(value):
    xyz = np.asarray(value.points)
    r = np.linalg.norm(xyz[:, :2], axis=1)
    mask = (r > 1) & (r < 8) & (xyz[:, 2] > -2.5) & (xyz[:, 2] < -0.15)
    part = value.select_by_index(np.flatnonzero(mask))
    for _ in range(4):
        if len(part.points) < 100:
            return None
        model, inliers = part.segment_plane(0.035, 3, 500)
        n = np.array(model[:3])
        if n[2] < 0:
            n = -n
        if n[2] > np.cos(np.radians(25)) and len(inliers) > 200:
            return n
        part = part.select_by_index(inliers, invert=True)
    return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('run', type=Path)
    parser.add_argument('--report', default='geometry_validation.json')
    parser.add_argument('--pgo-dir', default='sc_pgo')
    args = parser.parse_args()
    destination = args.run / args.report
    if destination.exists() or Path(args.report).name != args.report:
        raise RuntimeError('Use a new report basename; existing artifacts are never overwritten')
    raw = poses(args.run / 'sc_pgo/odom_poses.txt')
    optimized = poses(args.run / args.pgo_dir / 'optimized_poses.txt')
    times = np.loadtxt(args.run / 'sc_pgo/times.txt')
    if raw.shape != optimized.shape or len(raw) != len(times):
        raise RuntimeError('Pose/time counts differ')
    first = cloud(args.run / 'sc_pgo/Scans/000000.pcd')
    # Use a small initial submap to handle partial visibility. These are the
    # ORIGINAL early odometry transforms, not globally optimized poses, so the
    # independent target does not assume the proposed loop correction is true.
    for i in range(1,min(9,len(raw))):
        part=cloud(args.run / f'sc_pgo/Scans/{i:06d}.pcd')
        part.transform(np.linalg.inv(raw[0]) @ raw[i])
        first += part
    first=first.voxel_down_sample(.15)
    last = cloud(args.run / f'sc_pgo/Scans/{len(raw)-1:06d}.pcd')
    # Independent Open3D point-to-plane check, on raw body-frame scans, without
    # feeding the optimized vertical translation to the registration.
    for value in (first, last):
        value.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.6, max_nn=30))
    predicted_raw = np.linalg.inv(raw[0]) @ raw[-1]
    predicted_opt = np.linalg.inv(optimized[0]) @ optimized[-1]
    seeds = []
    for predicted in (predicted_raw, predicted_opt):
        seed = predicted.copy()
        seed[:3, 3] = 0
        seeds.append(seed)
    candidates = []
    for seed in seeds:
        coarse = o3d.pipelines.registration.registration_icp(
            last, first, 2.0, seed,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=80))
        fine = o3d.pipelines.registration.registration_icp(
            last, first, 0.35, coarse.transformation,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=80))
        candidates.append(fine)
    match = max(candidates, key=lambda result: result.fitness)
    verified = match.fitness >= 0.65 and match.inlier_rmse <= 0.15
    report = {'keyframes': len(raw), 'duration_sec': float(times[-1]-times[0]),
              'pgo_directory':args.pgo_dir,
              'first_last_submap_match': {'valid': verified, 'initial_submap_frames':min(9,len(raw)), 'overlap': match.fitness,
                  'rmse_m': match.inlier_rmse, 'relative_transform': match.transformation.tolist()},
              'ground_truth_accuracy_validated': False,
              'note': 'Internal loop consistency, not surveyed absolute accuracy. Physical start/end need not coincide.'}
    for name, values, pred in (('frontend', raw, predicted_raw), ('optimized', optimized, predicted_opt)):
        report[name] = {'endpoint_delta_xyz_m': (values[-1,:3,3]-values[0,:3,3]).tolist()}
        if verified:
            error = np.linalg.inv(match.transformation) @ pred
            report[name]['closure_translation_error_m'] = float(np.linalg.norm(error[:3,3]))
            report[name]['closure_rotation_error_deg'] = float(np.degrees(np.arccos(
                np.clip((np.trace(error[:3,:3])-1)/2, -1, 1))))
    floors = []
    for i in sorted(set(np.linspace(0, len(raw)-1, 10, dtype=int))):
        local = cloud(args.run / f'sc_pgo/Scans/{i:06d}.pcd')
        normal = floor_normal(local)
        if normal is None:
            continue
        row = {'keyframe': int(i), 'relative_sec': float(times[i]-times[0])}
        for name, matrices in [('frontend', raw), ('optimized', optimized)]:
            world_normal = matrices[i,:3,:3] @ normal
            row[name+'_floor_tilt_deg'] = float(np.degrees(np.arccos(np.clip(world_normal[2], -1, 1))))
        floors.append(row)
    report['sampled_floor_tilt'] = floors
    with (args.run / args.pgo_dir / 'loop_events.csv').open() as stream:
        report['loop_events'] = dict(Counter(row['event'] for row in csv.DictReader(stream)))
    destination.write_text(json.dumps(report, indent=2, ensure_ascii=False)+'\n')
    print(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
