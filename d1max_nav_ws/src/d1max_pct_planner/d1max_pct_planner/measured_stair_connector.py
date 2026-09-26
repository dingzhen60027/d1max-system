"""Conservative, offline audit of a measured staircase connector candidate.

The optimized keyframe trajectory supplies only a candidate centreline. Every
resampled station is checked against the original XYZ returns; no point, floor,
free-space ray, or robot motion is synthesized. Even a geometry-clean result is
``candidate_only`` because a fused PCD cannot prove unseen free space, friction,
footholds, or the gait/controller's ability to execute the staircase.
"""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import asdict, dataclass
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree


@dataclass(frozen=True)
class ConnectorLimits:
    # Standing size 0.930 x 0.480 x 0.585 m, plus 0.05 m horizontal and
    # 0.08 m vertical margins from the D1 Max traversability notes.
    sample_spacing_m: float = 0.10
    body_width_m: float = 0.48
    lateral_margin_m: float = 0.05
    body_height_m: float = 0.585
    vertical_margin_m: float = 0.08
    probe_radius_m: float = 0.12
    lateral_probe_count: int = 5
    min_ground_returns_per_probe: int = 3
    ground_bin_m: float = 0.05
    ground_mode_tolerance_m: float = 0.08
    body_to_ground_min_m: float = 0.20
    body_to_ground_max_m: float = 0.90
    max_lateral_height_spread_m: float = 0.25
    max_ground_step_m: float = 0.25
    max_grade: float = 1.0  # 45 degrees; candidate limit, not gait approval.
    grade_window_m: float = 0.50
    max_reverse_height_m: float = 0.25
    min_floor_rise_m: float = 2.0
    potential_obstacle_min_height_m: float = 0.25
    max_input_pose_gap_m: float = 0.75

    def __post_init__(self):
        for name, value in asdict(self).items():
            if name in ('lateral_probe_count', 'min_ground_returns_per_probe'):
                if isinstance(value, bool) or not isinstance(value, int) or value < 1:
                    raise ValueError(f'{name} must be a positive integer')
            elif isinstance(value, bool) or not np.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if self.lateral_probe_count < 3 or self.lateral_probe_count % 2 != 1:
            raise ValueError('lateral_probe_count must be odd and at least 3')
        if self.body_to_ground_min_m >= self.body_to_ground_max_m:
            raise ValueError('Body-to-ground search bounds must increase')
        if self.potential_obstacle_min_height_m >= self.body_height_m + self.vertical_margin_m:
            raise ValueError('Potential-obstacle band must reach the standing envelope')
        if self.sample_spacing_m > self.probe_radius_m:
            raise ValueError('Sampling must not skip beyond the lateral probe radius')


def _xyz_array(value, name):
    result = np.asarray(value, dtype=np.float64)
    if result.ndim != 2 or result.shape[1] != 3 or not len(result) or not np.isfinite(result).all():
        raise ValueError(f'{name} must be nonempty finite Nx3 XYZ')
    return result


def _resample_centerline(poses, spacing, max_input_gap):
    xy_steps = np.linalg.norm(np.diff(poses[:, :2], axis=0), axis=1)
    if len(xy_steps) and float(xy_steps.max()) > max_input_gap:
        return None, None, 'input_pose_gap'
    if np.any((xy_steps < 1e-5) & (np.abs(np.diff(poses[:, 2])) > 0.05)):
        return None, None, 'vertical_pose_jump'
    keep = np.r_[True, xy_steps >= 1e-5]
    vertices = poses[keep]
    if len(vertices) < 2:
        return None, None, 'zero_horizontal_length'
    distance = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(vertices[:, :2], axis=0), axis=1))]
    stations = np.r_[np.arange(0.0, distance[-1], spacing), distance[-1]]
    samples = np.column_stack([np.interp(stations, distance, vertices[:, axis]) for axis in range(3)])
    return stations, samples, None


def _measured_probe(xyz, xy, tree, cloud, limits):
    nearby = cloud[tree.query_ball_point(xy, limits.probe_radius_m), 2]
    low = xyz[2] - limits.body_to_ground_max_m
    high = xyz[2] - limits.body_to_ground_min_m
    floor = nearby[(nearby >= low) & (nearby <= high)]
    if not len(floor):
        return {'ground_z_m': None, 'ground_returns': 0,
                'potential_obstacle_returns': 0, 'nearest_upper_return_m': None}
    bins = np.arange(low, high + limits.ground_bin_m * 1.01, limits.ground_bin_m)
    if bins[-1] < high:
        bins = np.r_[bins, high]
    frequency, edges = np.histogram(floor, bins=bins)
    mode = float((edges[int(np.argmax(frequency))] + edges[int(np.argmax(frequency)) + 1]) / 2)
    support = int(np.count_nonzero(np.abs(floor - mode) <= limits.ground_mode_tolerance_m))
    top = mode + limits.body_height_m + limits.vertical_margin_m
    obstacles = int(np.count_nonzero((nearby > mode + limits.potential_obstacle_min_height_m)
                                     & (nearby < top)))
    upper = nearby[nearby > mode + limits.potential_obstacle_min_height_m]
    return {'ground_z_m': mode, 'ground_returns': support,
            'potential_obstacle_returns': obstacles,
            'nearest_upper_return_m': float(np.min(upper) - mode) if len(upper) else None}


def audit_measured_stair_connector(cloud_xyz, optimized_positions, *,
                                   start_index=729, end_index=954,
                                   limits: ConnectorLimits | None = None):
    """Return explicit evidence and reasons; never authorize navigation.

    ``optimized_positions`` is Nx3 in the same map frame as the PCD. Keyframe
    indices are inclusive. A history of robot motion is not treated as ground
    truth or as an independently safe route.
    """
    limits = limits or ConnectorLimits()
    cloud = _xyz_array(cloud_xyz, 'cloud_xyz')
    positions = _xyz_array(optimized_positions, 'optimized_positions')
    if (isinstance(start_index, bool) or isinstance(end_index, bool)
            or not isinstance(start_index, int) or not isinstance(end_index, int)
            or not 0 <= start_index < end_index < len(positions)):
        raise ValueError('Inclusive keyframe indices must select at least two poses')
    stations, candidate, centerline_error = _resample_centerline(
        positions[start_index:end_index + 1], limits.sample_spacing_m, limits.max_input_pose_gap_m)
    result = {'schema': 'd1max.measured_stair_connector/v1',
              'status': 'rejected', 'geometry_gates_passed': False,
              'execution_authorized': False, 'route_certified_safe': False,
              'frame_semantics': 'Caller must verify PCD and poses share the declared map frame; PCD has no frame ID.',
              'keyframe_range_inclusive': [start_index, end_index],
              'limits': asdict(limits), 'sample_count': 0, 'xy_length_m': None,
              'measured_floor_rise_m': None, 'samples': [], 'failure_counts': {},
              'limitations': [
                  'A fused PCD contains occupied returns, not sensor-ray free-space evidence.',
                  'Five sampled lateral ribs do not prove full rectangular body or foot-contact clearance.',
                  'Static geometry does not certify dynamic obstacles, stair gait, friction, or control.',
                  'The recorded trajectory is a candidate centreline, not a safe executable route.']}
    if centerline_error:
        result['failure_counts'] = {centerline_error: 1}
        return result
    result['xy_length_m'] = float(stations[-1])
    if len(candidate) < 3:
        result['failure_counts'] = {'insufficient_samples': 1}
        return result
    direction = np.gradient(candidate[:, :2], axis=0)
    direction /= np.linalg.norm(direction, axis=1, keepdims=True)
    if not np.isfinite(direction).all():
        result['failure_counts'] = {'undefined_heading': 1}
        return result
    normal = np.column_stack((-direction[:, 1], direction[:, 0]))
    half_width = limits.body_width_m / 2 + limits.lateral_margin_m
    offsets = np.linspace(-half_width, half_width, limits.lateral_probe_count)
    tree = cKDTree(cloud[:, :2])
    all_ground = []
    failures = Counter()
    for index, (station, body, lateral) in enumerate(zip(stations, candidate, normal)):
        probes = [_measured_probe(body, body[:2] + offset * lateral, tree, cloud, limits)
                  for offset in offsets]
        ground = [item['ground_z_m'] for item in probes]
        support = [item['ground_returns'] for item in probes]
        upper = [item['potential_obstacle_returns'] for item in probes]
        codes = []
        if support[len(support) // 2] < limits.min_ground_returns_per_probe:
            codes.append('center_ground_unobserved')
        if any(value < limits.min_ground_returns_per_probe for value in support):
            codes.append('lateral_ground_unobserved')
        spread = float(max(ground) - min(ground)) if all(z is not None for z in ground) else None
        if spread is not None and spread > limits.max_lateral_height_spread_m + 1e-9:
            codes.append('lateral_height_discontinuity')
        if any(value > 0 for value in upper):
            codes.append('potential_body_obstacle')
        for code in codes:
            failures[code] += 1
        center = ground[len(ground) // 2]
        all_ground.append(center)
        result['samples'].append({
            'sample_index': index, 'xy_distance_m': float(station),
            'candidate_body_xyz': body.tolist(),
            'measured_ground_xyz': [float(body[0]), float(body[1]), center] if center is not None else None,
            'lateral_offsets_m': offsets.tolist(), 'probes': probes,
            'lateral_height_spread_m': spread, 'failure_codes': codes})
    result['sample_count'] = len(result['samples'])
    if any(value is None for value in all_ground):
        failures['ground_profile_incomplete'] += 1
    else:
        ground = np.asarray(all_ground, dtype=float)
        result['measured_floor_rise_m'] = float(ground[-1] - ground[0])
        if abs(result['measured_floor_rise_m']) < limits.min_floor_rise_m:
            failures['floor_rise_unconfirmed'] += 1
        steps = np.abs(np.diff(ground))
        result['max_sampled_ground_step_m'] = float(np.max(steps))
        bad_steps = np.flatnonzero(steps > limits.max_ground_step_m + 1e-9)
        for index in bad_steps:
            failures['ground_step_exceeds_limit'] += 1
            result['samples'][int(index) + 1]['failure_codes'].append('ground_step_exceeds_limit')
        grade_span = max(1, int(np.ceil(limits.grade_window_m / limits.sample_spacing_m)))
        grades = np.abs(ground[grade_span:] - ground[:-grade_span]) / (
            stations[grade_span:] - stations[:-grade_span])
        result['maximum_window_grade'] = float(grades.max()) if len(grades) else None
        if np.any(grades > limits.max_grade + 1e-9):
            failures['grade_exceeds_limit'] += int(np.count_nonzero(grades > limits.max_grade + 1e-9))
        up = np.sign(ground[-1] - ground[0]) * ground
        reverse = np.maximum.accumulate(up) - up
        result['maximum_reverse_height_m'] = float(reverse.max())
        if reverse.max() > limits.max_reverse_height_m + 1e-9:
            failures['height_reversal_exceeds_limit'] += 1
    result['failure_counts'] = dict(failures)
    result['geometry_gates_passed'] = not failures
    result['status'] = 'candidate_only' if not failures else 'rejected'
    return result


def _sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def load_optimized_positions(path):
    matrix = np.loadtxt(path, dtype=np.float64)
    if matrix.ndim != 2 or matrix.shape[1] != 12 or not np.isfinite(matrix).all():
        raise ValueError('optimized_poses.txt must contain finite 3x4 matrices, one per line')
    return matrix[:, [3, 7, 11]]


def audit_from_files(pcd_path, optimized_poses_path, *, start_index=729, end_index=954,
                     limits: ConnectorLimits | None = None, frame_id='d1max_loc_map'):
    """Read an existing binary PCD/pose export and attach source provenance."""
    import open3d as o3d

    pcd = Path(pcd_path).resolve(strict=True)
    poses = Path(optimized_poses_path).resolve(strict=True)
    cloud = np.asarray(o3d.io.read_point_cloud(str(pcd)).points, dtype=float)
    result = audit_measured_stair_connector(
        cloud, load_optimized_positions(poses), start_index=start_index,
        end_index=end_index, limits=limits)
    result['source'] = {'pcd_path': str(pcd), 'pcd_sha256': _sha256(pcd),
                        'optimized_poses_path': str(poses),
                        'optimized_poses_sha256': _sha256(poses),
                        'declared_frame_id': frame_id, 'frame_independently_verified': False}
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pcd', type=Path, required=True)
    parser.add_argument('--poses', type=Path, required=True)
    parser.add_argument('--start-index', type=int, default=729)
    parser.add_argument('--end-index', type=int, default=954)
    parser.add_argument('--frame-id', default='d1max_loc_map')
    parser.add_argument('--output', type=Path, help='Explicit new JSON audit file; existing files are never replaced')
    args = parser.parse_args(argv)
    result = audit_from_files(args.pcd, args.poses, start_index=args.start_index,
                              end_index=args.end_index, frame_id=args.frame_id)
    serialized = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + '\n'
    if args.output:
        with args.output.open('x', encoding='utf-8') as stream:
            stream.write(serialized)
        print(json.dumps({key: result[key] for key in ('status', 'geometry_gates_passed',
                                                       'sample_count', 'failure_counts')}, ensure_ascii=False))
    else:
        print(serialized, end='')
    return 0 if result['geometry_gates_passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
