#!/usr/bin/env python3
"""Offline saved-map floor-shape diagnosis. Never modifies maps or launches ROS.

Floor-like patches are observed geometry, not estimator constraints or surveyed GT.
Deterministic seeds and three ROI settings expose selection sensitivity.
"""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

ROOT = Path('/home/dndx/d1max_nav_ws')
BASE = ROOT / 'experiments/lio_frontend_reliability_20260919'
spec = importlib.util.spec_from_file_location('existing_result_reader', ROOT / 'experiments/central_imu_fasterlio_20260918/analyze_result.py')
reader = importlib.util.module_from_spec(spec)
spec.loader.exec_module(reader)
NAMES = ['central_device_paired_front_180s_20260919', 'internal_device_paired_front_180s_20260919', 'internal_historical_paired_front_180s_20260919']
PROFILES = [(0.75, 0.25, 0.85, 0.020), (1.0, 0.25, 0.85, 0.025), (1.25, 0.15, 1.1, 0.030)]


def angle(a, b):
    return float(np.degrees(np.arctan2(np.linalg.norm(np.cross(a, b)), np.dot(a, b))))


def fit_plane(points):
    center = points.mean(axis=0)
    _, _, vt = np.linalg.svd(points - center, full_matrices=False)
    normal = vt[-1]
    if normal[2] < 0:
        normal = -normal
    return normal, -normal @ center


def dominant_floor(points, threshold, seed):
    if len(points) < 60:
        return None
    rng = np.random.default_rng(seed)
    best = np.zeros(len(points), dtype=bool)
    for _ in range(160):
        q = points[rng.choice(len(points), 3, replace=False)]
        normal = np.cross(q[1] - q[0], q[2] - q[0])
        length = np.linalg.norm(normal)
        if length < 1e-9 or abs(normal[2]) / length < np.cos(np.radians(20)):
            continue
        mask = np.abs((points - q[0]) @ (normal / length)) < threshold
        if mask.sum() > best.sum():
            best = mask
    if best.sum() < 50 or best.mean() < .35:
        return None
    for _ in range(4):
        normal, d = fit_plane(points[best])
        best = np.abs(points @ normal + d) < threshold
    chosen = points[best]
    normal, d = fit_plane(chosen)
    eigen_xy = np.linalg.eigvalsh(np.cov(chosen[:, :2].T))
    if normal[2] < np.cos(np.radians(20)) or eigen_xy[0] < .012:
        return None
    return normal, float(d), best, eigen_xy


def profile_map(cloud, poses, s, radius, low, high, threshold):
    tree = cKDTree(cloud[:, :2])
    # Avoid extrapolating beyond the end; report the initial edge separately.
    ids = np.unique(np.searchsorted(s, np.arange(1., s[-1] - .5, 2.5)))
    patches, inlier_blocks = [], []
    for k, index in enumerate(ids):
        pose = poses[index]
        pts = cloud[tree.query_ball_point(pose[:2], radius)]
        pts = pts[(pts[:, 2] < pose[2] - low) & (pts[:, 2] > pose[2] - high)]
        fit = dominant_floor(pts, threshold, 260920 + k)
        item = {'state_index': int(index), 's_m': float(s[index]), 'lidar_xyz_m': pose.tolist(), 'candidate_count': len(pts)}
        if fit is None:
            item['status'] = 'insufficient_or_ambiguous_floor'
        else:
            normal, d, mask, eigen = fit
            chosen = pts[mask]
            z = -(normal[:2] @ pose[:2] + d) / normal[2]
            item.update(status='floor_like', normal=normal.tolist(), plane_d=d, floor_xyz_m=[float(pose[0]), float(pose[1]), float(z)], lidar_height_above_plane_m=float(pose[2]-z), inliers=len(chosen), inlier_fraction=float(mask.mean()), rmse_m=float(np.sqrt(np.mean((chosen@normal+d)**2))), xy_covariance_eigenvalues=eigen.tolist())
            # Equal number of deterministic points per patch avoids density dominance.
            inlier_blocks.append(chosen[np.linspace(0, len(chosen)-1, min(300, len(chosen)), dtype=int)])
        patches.append(item)
    good = [p for p in patches if p['status'] == 'floor_like']
    out = {'parameters': {'radius_m':radius,'below_lidar_band_m':[low,high],'ransac_threshold_m':threshold,'patch_spacing_m':2.5}, 'patches':patches,'accepted_patches':len(good),'attempted_patches':len(patches)}
    if len(good) < 8:
        out['valid_summary'] = False
        return out
    # One unconstrained global rigid plane: no flattening and no change to the map.
    all_inliers = np.concatenate(inlier_blocks)
    normal, d = fit_plane(all_inliers)
    centers = np.array([p['floor_xyz_m'] for p in good])
    distances = np.array([p['s_m'] for p in good])
    signed = centers @ normal + d
    centered_s = distances - distances.mean()
    linear = np.polyfit(centered_s, centers[:,2], 1)
    quadratic = np.polyfit(centered_s, centers[:,2], 2)
    linear_resid = centers[:,2] - np.polyval(linear, centered_s)
    quadratic_resid = centers[:,2] - np.polyval(quadratic, centered_s)
    patch_angles = [angle(np.array(p['normal']), normal) for p in good]
    point_residuals = all_inliers @ normal + d
    for patch, block in zip(good, inlier_blocks):
        residual = block @ normal + d
        patch['global_plane_signed_residual_m'] = {'median':float(np.median(residual)), 'p05':float(np.quantile(residual,.05)), 'p95':float(np.quantile(residual,.95)), 'rmse':float(np.sqrt(np.mean(residual**2)))}
    heights = [p['lidar_height_above_plane_m'] for p in good]
    out.update(valid_summary=True, global_plane_normal=normal.tolist(), global_plane_d=float(d), global_plane_tilt_deg=angle(normal,np.array([0.,0,1.])),
        covered_distance_m=[float(distances[0]),float(distances[-1])], floor_endpoint_delta_z_m=float(centers[-1,2]-centers[0,2]),
        floor_center_to_global_plane_rmse_m=float(np.sqrt(np.mean(signed**2))), floor_center_to_global_plane_span_m=float(np.ptp(signed)),
        floor_center_to_global_plane_maxabs_m=float(np.max(np.abs(signed))), local_normal_to_global_angle_deg={'median':float(np.median(patch_angles)),'max':float(np.max(patch_angles))},
        equal_patch_sample_to_global_plane_m={'points':len(all_inliers),'rmse':float(np.sqrt(np.mean(point_residuals**2))),'abs_p95':float(np.quantile(np.abs(point_residuals),.95)),'signed_p05_p95':[float(np.quantile(point_residuals,.05)),float(np.quantile(point_residuals,.95))]},
        along_path_linear_slope=float(linear[0]), along_path_linear_rmse_m=float(np.sqrt(np.mean(linear_resid**2))), along_path_linear_residual_span_m=float(np.ptp(linear_resid)),
        quadratic_coefficient_per_m=float(quadratic[0]), along_path_quadratic_rmse_m=float(np.sqrt(np.mean(quadratic_resid**2))),
        lidar_height_above_plane_m={'median':float(np.median(heights)),'range':[float(min(heights)),float(max(heights))]},
        plane_normal_first_last_angle_deg=angle(np.array(good[0]['normal']),np.array(good[-1]['normal'])))
    return out


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path)
    parser.add_argument('--runs',type=Path,nargs='+',help='Explicit completed run directories; defaults to the three controlled short trials.')
    args=parser.parse_args()
    report={'purpose':'Distinguish near-rigid floor tilt from along-path bending in saved maps; no map alteration.',
        'limits':['Floor-like surfaces selected relative to estimated LiDAR trajectory; not surveyed ground truth.', 'User reports a level corridor; this is only an offline diagnostic assumption, never a runtime constraint.', 'Local and global maps share estimator errors; a planar result cannot validate absolute pose or extrinsics.', '180-second initial straight section only, not full-loop or multi-floor validation.', 'Quadratic fit is descriptive, not independent model selection or a correction to apply.'], 'runs':[]}
    selected_runs=args.runs if args.runs else [BASE/'runs'/name for name in NAMES]
    report['selected_run_scope']='explicit directories; configurations may differ, not a controlled A/B' if args.runs else 'three controlled 180-second single-front trials'
    if args.runs:
        report['limits'][3]='Explicit run durations and configurations must be interpreted separately; no cross-run accuracy ranking.'
    for run in selected_runs:
        name=run.name
        result=json.loads((run/'result.json').read_text())
        if not result['complete']:
            raise ValueError('Only completed runs are allowed')
        pcd=Path(result['map_path'])
        meta,cloud=reader.read_pcd(pcd,max_plot_points=10_000_000)
        cloud=cloud.astype(float)
        f,validation=reader.read_state(run/'frontend_state.csv')
        if not validation['all_numeric_cells_finite']:
            raise ValueError('Nonfinite states')
        positions=np.column_stack([f[k] for k in ('x','y','z')])
        rotations=Rotation.from_quat(np.column_stack([f[k] for k in ('qx','qy','qz','qw')]))
        calibration=yaml.safe_load((run/'config/calibration.yaml').read_text())
        lever=np.asarray(calibration['lio_extrinsic']['translation'])
        poses=positions+rotations.apply(np.broadcast_to(lever,positions.shape))
        s=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(poses[:,:2],axis=0),axis=1))]
        item={'run':name,'pcd':str(pcd),'pcd_sha256':hashlib.sha256(pcd.read_bytes()).hexdigest(),'state_sha256':hashlib.sha256((run/'frontend_state.csv').read_bytes()).hexdigest(),'pcd_metadata':meta,'profiles':[]}
        for config in PROFILES:
            item['profiles'].append(profile_map(cloud,poses,s,*config))
        report['runs'].append(item)
    encoded=json.dumps(report,indent=2,allow_nan=False)
    if args.output:
        with args.output.open('x') as stream:
            stream.write(encoded+'\n')
    else:
        print(encoded)


if __name__=='__main__':
    main()
