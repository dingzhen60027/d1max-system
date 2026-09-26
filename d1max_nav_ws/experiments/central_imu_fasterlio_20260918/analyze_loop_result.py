#!/usr/bin/env python3
"""Offline SC-PGO result inspection; never starts ROS or modifies its inputs.

Normal use: --run RUN (writes RUN/analysis/loop_result.json and .png).
Compatibility tests may select --pgo-dir sc_pgo_final and --output-dir DIR.
Missing frontend maps require explicit --allow-missing-frontend-map; no other
run's map or invented keyframe timestamps are substituted.
"""
import argparse
from collections import Counter
import csv
import hashlib
import io
import json
from pathlib import Path

import numpy as np

from analyze_result import read_pcd, read_state, summary, trajectory_metrics


SOURCE_ROOT = Path('/home/dndx/d1max_nav_ws')
EXPORTER = SOURCE_ROOT / 'src/sc_pgo/src/laserPosegraphOptimization.cpp'


def file_info(path):
    result = {'path': str(path), 'resolved_path': str(path.resolve()),
              'exists': path.is_file()}
    if path.is_file():
        result.update(bytes=path.stat().st_size, mtime_ns=path.stat().st_mtime_ns)
    return result


def load_numeric(path, columns):
    """Fail rather than silently drop, truncate or pad completed pose exports."""
    data = np.loadtxt(path, ndmin=2)
    if data.shape[1] != columns or not len(data) or not np.isfinite(data).all():
        raise ValueError(f'{path}: expected finite rows with {columns} columns')
    return data


def pose_metrics(xyz, times=None):
    if not len(xyz) or not np.isfinite(xyz).all():
        raise ValueError('Empty or nonfinite trajectory')
    delta = xyz[-1] - xyz[0]
    segments = np.diff(xyz, axis=0)
    result = {
        'pose_count': len(xyz), 'start_xyz_m': xyz[0].tolist(),
        'endpoint_xyz_m': xyz[-1].tolist(),
        'endpoint_minus_start_xyz_m': delta.tolist(),
        'endpoint_to_start_distance_xyz_m': float(np.linalg.norm(delta)),
        'endpoint_to_start_distance_xy_m': float(np.linalg.norm(delta[:2])),
        'height_span_m': float(np.ptp(xyz[:, 2])), 'height_z_m': summary(xyz[:, 2]),
        'path_length_xyz_m': float(np.linalg.norm(segments, axis=1).sum()),
        'path_length_xy_m': float(np.linalg.norm(segments[:, :2], axis=1).sum()),
    }
    if times is not None:
        result.update(first_last_header_sec=times[[0, -1]].tolist(),
                      duration_sec=float(times[-1] - times[0]))
    return result


def read_kitti(path):
    rows = load_numeric(path, 12).reshape(-1, 3, 4)
    rotation = rows[:, :, :3]
    metadata = {'source': file_info(path),
                'format': 'KITTI: r00 r01 r02 tx r10 r11 r12 ty r20 r21 r22 tz; no timestamps',
                'rotation_determinant': summary(np.linalg.det(rotation)),
                'rotation_orthonormality_frobenius_error': summary(np.linalg.norm(
                    np.swapaxes(rotation, 1, 2) @ rotation - np.eye(3), axis=(1, 2))),
                'precision_note': 'Exporter uses default stream precision for matrices; small rounding errors are expected.'}
    return rows[:, :, 3], metadata


def read_frontend(run):
    path = run / 'frontend_state.csv'
    if path.is_file():
        fields, validation = read_state(path)
        if validation['ignored_incomplete_final_line'] or validation['malformed_complete_rows']:
            raise ValueError('Frontend CSV has incomplete/malformed rows; wait for a clean completed run')
        metrics, xyz, valid, _ = trajectory_metrics(fields, validation)
        if not valid.all() or np.any(np.diff(fields['t']) <= 0):
            raise ValueError('Frontend state contains nonfinite poses/times or non-increasing time')
        return xyz, fields['t'], {'source': file_info(path),
                                 'source_kind': 'frontend_state.csv', 'metrics': metrics}
    path = run / 'frontend_odometry.tum'
    rows = load_numeric(path, 8)
    if np.any(np.diff(rows[:, 0]) <= 0):
        raise ValueError('Frontend TUM times are not strictly increasing')
    return rows[:, 1:4], rows[:, 0], {
        'source': file_info(path), 'source_kind': 'frontend_odometry.tum fallback',
        'metrics': pose_metrics(rows[:, 1:4], rows[:, 0]),
        'limitation': 'frontend_state.csv is absent; no gravity/bias/residual diagnostics are reconstructed.'}


def keyframe_times(pgo, raw_count, optimized_count, front_xyz, front_times, raw_xyz):
    path = pgo / 'times.txt'
    result = {'source': file_info(path), 'associated': False,
              'semantics': 'Cloud header time written once per selected keyframe by process_pg; not loop processing ros_time.',
              'row_order_assumption': 'Same-directory exports from contiguous integer Pose3 keyframe IDs in the inspected SC-PGO implementation.',
              'synthetic_timestamps_created': False}
    if not path.is_file():
        result['reason'] = 'times.txt unavailable; plots use keyframe index'
        return None, result
    times = load_numeric(path, 1)[:, 0]
    result['row_count'] = len(times)
    if not len(times) == raw_count == optimized_count:
        result['reason'] = 'Pose/time counts differ; no timestamp association or truncation performed'
        return None, result
    if np.any(np.diff(times) <= 0):
        result['reason'] = 'Keyframe times are not strictly increasing; use keyframe index'
        return None, result
    insertion = np.searchsorted(front_times, times)
    left, right = np.maximum(0, insertion - 1), np.minimum(len(front_times) - 1, insertion)
    nearest = np.where(abs(front_times[left] - times) <= abs(front_times[right] - times), left, right)
    dt = abs(front_times[nearest] - times)
    position_difference = np.linalg.norm(raw_xyz - front_xyz[nearest], axis=1)
    result.update(associated=True, first_last_header_sec=times[[0, -1]].tolist(),
                  duration_sec=float(times[-1] - times[0]),
                  nearest_frontend_time_abs_difference_sec=summary(dt),
                  nearest_frontend_position_difference_m=summary(position_difference),
                  exact_frontend_matches_within_1us=int(np.sum(dt <= 1e-6)),
                  frontend_matches_within_30ms=int(np.sum(dt <= .03)),
                  shared_frontend_time_axis_validated=bool(np.all(dt <= .03) and np.all(position_difference <= .05)),
                  alignment_note='Nearest-time and position checks are diagnostics only; no new times or poses are fitted.')
    return times, result


def read_events(path, keyframe_count):
    data = path.read_bytes()
    if data and not data.endswith(b'\n'):
        raise ValueError('loop_events.csv has an incomplete last line; wait for the completed run')
    reader = csv.DictReader(io.StringIO(data.decode('utf-8')))
    required = {'ros_time', 'event', 'history_keyframe', 'current_keyframe', 'value1', 'value2'}
    if not required.issubset(reader.fieldnames or []):
        raise ValueError('Unexpected loop_events.csv schema')
    counts, accepted = Counter(), []
    for line_number, row in enumerate(reader, 2):
        if None in row or any(value is None for value in row.values()):
            raise ValueError(f'Malformed event CSV line {line_number}')
        counts[row['event']] += 1
        if row['event'] != 'accepted':
            continue
        item = {'csv_line': line_number, 'ros_time': float(row['ros_time']),
                'history_keyframe': int(row['history_keyframe']),
                'current_keyframe': int(row['current_keyframe']),
                'cumulative_counter_value1': float(row['value1'])}
        item['indices_within_exported_optimized_poses'] = all(
            0 <= item[key] < keyframe_count for key in ('history_keyframe', 'current_keyframe'))
        accepted.append(item)
    pairs = {(a['history_keyframe'], a['current_keyframe']) for a in accepted}
    counters = [a['cumulative_counter_value1'] for a in accepted]
    return {'source': file_info(path), 'event_counts': dict(counts),
            'total_event_rows': sum(counts.values()), 'accepted_event_count': len(accepted),
            'accepted_unique_keyframe_pairs': len(pairs),
            'accepted_counter_is_contiguous_1_to_N': counters == list(range(1, len(accepted) + 1)),
            'all_accepted_indices_in_export': all(a['indices_within_exported_optimized_poses'] for a in accepted),
            'accepted_events': accepted,
            'count_definition': 'Only exact event=accepted, emitted after inserting a loop BetweenFactor; candidate/geometry events are not counted.',
            'time_warning': 'ros_time is backend processing clock; do not treat it as a keyframe sensor timestamp.',
            'accuracy_warning': 'Accepted means a factor was admitted, not independently verified physical correctness or surveyed map accuracy.'}


def make_plot(path, frontend, front_times, raw, optimized, times, timing, raw_map, opt_map, report):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import Normalize
    plt.rcParams.update({'font.size': 10, 'axes.spines.top': False, 'axes.spines.right': False})
    fig = plt.figure(figsize=(14, 12), layout='constrained')
    grid = fig.add_gridspec(3, 2, height_ratios=[1.2, 1., .8])
    top_axes = [fig.add_subplot(grid[0, i]) for i in range(2)]
    side_axes = [fig.add_subplot(grid[1, i]) for i in range(2)]
    trace = fig.add_subplot(grid[2, :])
    point_sets = [p for p in (raw_map, opt_map) if p is not None]
    color_z = np.concatenate([p[:, 2] for p in point_sets])
    vmin, vmax = np.percentile(color_z, [1, 99])
    norm = Normalize(vmin=vmin, vmax=max(vmax, vmin + .01), clip=True)
    all_positions = np.vstack([frontend, raw, optimized] + point_sets)
    lower, upper = all_positions.min(axis=0), all_positions.max(axis=0)
    # Include exact all-point bounds, not just plot samples.
    for key in ('frontend_map', 'optimized_map'):
        item = report[key]
        if item.get('available'):
            lower = np.minimum(lower, item['bounds_min_xyz_m'])
            upper = np.maximum(upper, item['bounds_max_xyz_m'])
    pad = np.maximum((upper - lower) * .03, .1)
    _, _, vt = np.linalg.svd(raw[:, :2] - raw[0, :2], full_matrices=False)
    principal = vt[0]
    if principal[np.argmax(abs(principal))] < 0:
        principal = -principal
    corners = np.array([[x, y] for x in (lower[0], upper[0]) for y in (lower[1], upper[1])])
    projected_bounds = (corners - raw[0, :2]) @ principal
    report['visualization'].update(side_projection_unit_xy=principal.tolist(),
        side_projection_origin_xy_m=raw[0, :2].tolist(),
        shared_z_color_limits_m=[float(vmin), float(max(vmax, vmin + .01))])
    colored = None
    for i, (cloud, poses, title, color) in enumerate((
            (raw_map, frontend, 'Raw frontend (all published poses)', '#b74136'),
            (opt_map, optimized, 'SC-PGO (optimized keyframes)', '#205e9b'))):
        top, side = top_axes[i], side_axes[i]
        if cloud is not None:
            colored = top.scatter(cloud[:, 0], cloud[:, 1], c=cloud[:, 2], norm=norm,
                                  cmap='viridis', s=.25, rasterized=True)
            side.scatter((cloud[:, :2] - raw[0, :2]) @ principal, cloud[:, 2],
                         c=cloud[:, 2], norm=norm, cmap='viridis', s=.25, rasterized=True)
        else:
            top.text(.5, .92, 'Raw map file unavailable; trajectory only',
                     transform=top.transAxes, ha='center', color='#555555')
        top.plot(poses[:, 0], poses[:, 1], color=color, lw=.9)
        top.scatter(*poses[0, :2], marker='s', s=35, facecolor='white', edgecolor=color, zorder=5, label='Start')
        top.scatter(*poses[-1, :2], marker='x', s=40, color=color, zorder=5, label='End')
        top.set(title=title, xlabel='World X [m]', ylabel='World Y [m]',
                xlim=(lower[0] - pad[0], upper[0] + pad[0]), ylim=(lower[1] - pad[1], upper[1] + pad[1]))
        top.set_aspect('equal', adjustable='box')
        top.legend(loc='upper right', fontsize=8)
        side.plot((poses[:, :2] - raw[0, :2]) @ principal, poses[:, 2], color=color, lw=1.)
        side.set(xlabel='Common horizontal projection [m]', ylabel='World Z [m]',
                 title='Side profile (no leveling / flattening)',
                 xlim=(projected_bounds.min() - .2, projected_bounds.max() + .2),
                 ylim=(lower[2] - pad[2], upper[2] + pad[2]))
    if colored is not None:
        fig.colorbar(colored, ax=top_axes, shrink=.7, label='Point Z [m]; shared color, clipped at 1/99 percentiles')
    use_time = times is not None and timing.get('shared_frontend_time_axis_validated', False)
    if use_time:
        horizontal = times - front_times[0]
        trace.plot(front_times - front_times[0], frontend[:, 2], color='#9a9a9a', lw=.8, label='Frontend: all poses')
        trace.set_xlabel('Sensor time from first frontend pose [s]')
    else:
        horizontal = np.arange(len(raw))
        trace.set_xlabel('Keyframe index (not synthetic sensor time)')
    trace.plot(horizontal, raw[:, 2], color='#b74136', lw=1.2, label='Frontend: selected keyframes')
    optimized_x = horizontal if len(raw) == len(optimized) else np.arange(len(optimized))
    trace.plot(optimized_x, optimized[:, 2], color='#205e9b', lw=1.2, linestyle='--', label='SC-PGO: optimized keyframes')
    trace.set(ylabel='Estimated world Z [m]', title='Trajectory height: endpoint consistency is not whole-site flatness')
    trace.legend(loc='best', fontsize=9)
    trace.grid(alpha=.18)
    accepted = report['loops']['accepted_event_count']
    raw_d = report['raw_keyframe_trajectory']['endpoint_minus_start_xyz_m']
    opt_d = report['optimized_keyframe_trajectory']['endpoint_minus_start_xyz_m']
    fig.suptitle(f'{report["run_name"]}\nAccepted loop events: {accepted} | Keyframes: {len(raw)} raw / {len(optimized)} optimized\n'
                 f'Keyframe end-start XYZ [m]: raw {np.array(raw_d).round(3)}; optimized {np.array(opt_d).round(3)}', fontsize=12)
    fig.supxlabel('Own estimator world frames, no fitted alignment. No ground truth; loop optimization does not repair the frontend.\n'
                  'Global map bounds include non-ground objects. Experimental central-IMU extrinsics remain unvalidated.', fontsize=9)
    fig.savefig(path, dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--pgo-dir', default='sc_pgo', help='Direct child of run; explicitly select old reprocessed results if needed')
    parser.add_argument('--output-dir', type=Path, help='Testing override; default RUN/analysis')
    parser.add_argument('--allow-missing-frontend-map', action='store_true')
    parser.add_argument('--max-plot-points', type=int, default=250000)
    parser.add_argument('--overwrite-analysis', action='store_true', help='Replace only generated loop_result.json/png')
    args = parser.parse_args()
    if Path(args.pgo_dir).name != args.pgo_dir or args.max_plot_points < 1:
        parser.error('pgo-dir must be a direct child name and max-plot-points must be positive')
    run = args.run.resolve()
    pgo = run / args.pgo_dir
    result_path = run / 'result.json'
    if not result_path.is_file() or json.loads(result_path.read_text()).get('complete') is not True:
        raise ValueError('This is a completed-run analyzer: result.json must exist with complete=true. '
                         'No live or failed run is silently labeled complete.')
    output = args.output_dir.resolve() if args.output_dir else run / 'analysis'
    targets = [output / f'loop_result.{suffix}' for suffix in ('json', 'png')]
    if not args.overwrite_analysis and any(path.exists() for path in targets):
        raise FileExistsError('Generated output exists; select a new --output-dir or --overwrite-analysis')
    frontend, front_times, front_report = read_frontend(run)
    raw, raw_source = read_kitti(pgo / 'odom_poses.txt')
    optimized, opt_source = read_kitti(pgo / 'optimized_poses.txt')
    if len(raw) != len(optimized):
        raise ValueError('Raw/optimized keyframe counts differ; cannot compare different terminal keyframes. '
                         'Wait for a completed export; no truncation is performed.')
    times, timing = keyframe_times(pgo, len(raw), len(optimized), frontend, front_times, raw)
    loops = read_events(pgo / 'loop_events.csv', len(optimized))
    if times is not None:
        for event in loops['accepted_events']:
            if event['indices_within_exported_optimized_poses']:
                event['history_cloud_header_sec'] = float(times[event['history_keyframe']])
                event['current_cloud_header_sec'] = float(times[event['current_keyframe']])
    opt_map_report, opt_map = read_pcd(pgo / 'optimized_map.pcd', args.max_plot_points)
    opt_map_report['available'] = True
    raw_path = run / 'scans.pcd'
    raw_map = None
    if raw_path.is_file():
        raw_map_report, raw_map = read_pcd(raw_path, args.max_plot_points)
        raw_map_report['available'] = True
    elif args.allow_missing_frontend_map:
        raw_map_report = {'available': False, 'source': file_info(raw_path),
                          'reason': 'Missing or broken-symlink raw map; no substitute map loaded'}
    else:
        raise FileNotFoundError(f'{raw_path}: missing raw map; wait for run completion')
    manifest = {}
    for name, source in [('run_manifest', run / 'manifest.json'), ('run_result', run / 'result.json'),
                         ('pgo_manifest', pgo / 'manifest.json'), ('pgo_status', pgo / 'status.json')]:
        if source.is_file():
            manifest[name] = {'source': file_info(source), 'contents': json.loads(source.read_text())}
    report = {
        'schema_version': 1, 'run': str(run), 'run_name': run.name,
        'pgo_directory': str(pgo), 'output_directory': str(output),
        'frontend': front_report, 'keyframe_time_association': timing,
        'raw_keyframe_trajectory': {**pose_metrics(raw, times), **raw_source},
        'optimized_keyframe_trajectory': {**pose_metrics(optimized, times), **opt_source},
        'loops': loops, 'frontend_map': raw_map_report, 'optimized_map': opt_map_report,
        'same_keyframe_count': len(raw) == len(optimized), 'run_provenance': manifest,
        'ground_truth_accuracy_validated': False, 'whole_site_floor_flatness_validated': False,
        'central_imu_extrinsics_status': 'Experimental rotation/time candidate and nominal lever arm; no surveyed factory calibration claim. Historical test runs may use front IMU instead; inspect manifest.',
        'interpretation': [
            'Endpoint delta is end minus start of the named trajectory, not an absolute error; physical start/end may differ.',
            'Compare raw and optimized selected-keyframe endpoints, not the final frontend frame against an earlier last keyframe.',
            'Smaller endpoint separation alone cannot establish whole-site floor flatness or correct local geometry.',
            'Global point-cloud height bounds include walls/objects/terrain and are not floor-flatness metrics.',
            'PGO changes keyframe poses and reconstructs a global map; it does not fix the frontend IMU estimate or rewrite its trajectory.',
            'Raw full map and optimized keyframe map use different selection/downsampling, so point counts are not a quality score.',
            'Historical Sept17 comparisons are not a controlled same-software/same-IMU experiment.',
        ],
        'visualization': {'coordinates_modified': False, 'world_frames_aligned': False,
                          'same_xy_z_axes_between_map_panels': True,
                          'point_sampling': 'Deterministic stride for display; reported map counts/bounds use all points'},
        'implementation_evidence': {
            'exporter': file_info(EXPORTER),
            'exporter_sha256_at_analysis': hashlib.sha256(EXPORTER.read_bytes()).hexdigest() if EXPORTER.is_file() else None,
            'source_note': 'Inspected source defines export semantics; current source hash alone does not prove historical binary identity.',
            'kitti_export_functions': 'saveOdometryVerticesKITTIformat; saveOptimizedVerticesKITTIformat',
            'times_writer': 'process_pg: pgTimeSaveStream << timeLaser',
            'accepted_writer': 'process_icp: add BetweenFactor, increment accepted_loop_count, logLoopEvent("accepted")'},
    }
    if len(raw) == len(optimized):
        correction = optimized - raw
        report['same_index_world_translation_difference'] = {
            'note': 'Coordinate difference in their existing world gauges, not a rigid-aligned error or ground truth.',
            'norm_m': summary(np.linalg.norm(correction, axis=1)),
            'z_m': summary(correction[:, 2]), 'first_last_xyz_m': correction[[0, -1]].tolist()}
    if times is not None:
        report['frontend_keyframe_time_coverage'] = {
            'first_keyframe_after_first_frontend_sec': float(times[0] - front_times[0]),
            'last_frontend_after_last_keyframe_sec': float(front_times[-1] - times[-1]),
            'note': 'Unselected frontend tail frames are excluded from the like-for-like keyframe endpoint comparison.'}
    output.mkdir(parents=True, exist_ok=True)
    make_plot(targets[1], frontend, front_times, raw, optimized, times, timing, raw_map, opt_map, report)
    targets[0].write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + '\n')
    print(json.dumps({'json': str(targets[0]), 'png': str(targets[1]),
                      'accepted_events': loops['accepted_event_count'],
                      'raw_keyframe_endpoint_delta_xyz_m': report['raw_keyframe_trajectory']['endpoint_minus_start_xyz_m'],
                      'optimized_keyframe_endpoint_delta_xyz_m': report['optimized_keyframe_trajectory']['endpoint_minus_start_xyz_m']}))


if __name__ == '__main__':
    main()
