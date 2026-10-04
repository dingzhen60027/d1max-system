"""Fail-closed, offline PCT route through one explicitly designated stair.

Run with ``python3 -m d1max_pct_planner.crossfloor_route --config FILE.yaml``.
This module does not launch ROS, SCAN, a controller, or the robot SDK. Native
PCT plans each leg; both the masked planning map and the original tomogram
must independently accept every returned curve. No trajectory fallback exists.
"""

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

import numpy as np
import yaml

from .tomogram_map import TomogramMap


ANCHORS = ('start', 'entry', 'landing', 'exit', 'goal')
LEGS = (
    ('lower_floor', 'start', 'entry'),
    ('stair_lower', 'entry', 'landing'),
    ('stair_upper', 'landing', 'exit'),
    ('upper_floor', 'exit', 'goal'),
)


class CrossfloorError(ValueError):
    def __init__(self, code, message, **details):
        super().__init__(message)
        self.code = code
        self.details = details

    def as_dict(self):
        return {'code': self.code, 'message': str(self), **self.details}


def _finite_vector(value, size, label):
    array = np.asarray(value, dtype=float)
    if array.shape != (size,) or not np.isfinite(array).all():
        raise CrossfloorError('invalid_config', f'{label} needs {size} finite numbers')
    return array


def _number(value, label, low, high):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise CrossfloorError('invalid_config', f'{label} must be numeric')
    value = float(value)
    if not np.isfinite(value) or not low <= value <= high:
        raise CrossfloorError('invalid_config', f'{label} must be in [{low}, {high}]')
    return value


def validate_config(config):
    """Return normalized settings without opening files or allocating a map."""
    if not isinstance(config, dict):
        raise CrossfloorError('invalid_config', 'Configuration must be a YAML mapping')
    required = {'schema_version', 'tomogram_path', 'source_pcd', 'vendor_root',
                'output_directory', 'frame_id', 'anchors', 'stair_roi', 'floor_z_ranges'}
    allowed = required | {'planning', 'limits', 'unknown_ceiling_policy'}
    if required - set(config) or set(config) - allowed:
        raise CrossfloorError('invalid_config', 'Missing or unknown configuration keys',
                              missing=sorted(required - set(config)), unknown=sorted(set(config) - allowed))
    if config['schema_version'] != 1:
        raise CrossfloorError('invalid_config', 'schema_version must be 1')
    for key in ('tomogram_path', 'source_pcd', 'vendor_root', 'output_directory'):
        if not isinstance(config[key], str) or not Path(config[key]).is_absolute():
            raise CrossfloorError('invalid_config', f'{key} must be an absolute path')
    frame = config['frame_id']
    if not isinstance(frame, str) or not frame or frame.startswith('/') or any(c.isspace() for c in frame):
        raise CrossfloorError('invalid_config', 'frame_id must be a ROS frame name')
    anchors = config['anchors']
    if not isinstance(anchors, dict) or set(anchors) != set(ANCHORS):
        raise CrossfloorError('invalid_config', 'anchors must contain start, entry, landing, exit, goal')
    normalized_anchors = {}
    for name in ANCHORS:
        anchor = anchors[name]
        if not isinstance(anchor, dict) or set(anchor) != {'xyz', 'layer_id'}:
            raise CrossfloorError('invalid_config', f'{name} needs xyz and layer_id')
        layer = anchor['layer_id']
        if isinstance(layer, bool) or not isinstance(layer, int) or layer < 0:
            raise CrossfloorError('invalid_config', f'{name}.layer_id must be a nonnegative integer')
        normalized_anchors[name] = {'xyz': _finite_vector(anchor['xyz'], 3, name + '.xyz'),
                                    'layer_id': layer}
    roi = config['stair_roi']
    if not isinstance(roi, dict) or set(roi) != {'min', 'max'}:
        raise CrossfloorError('invalid_config', 'stair_roi needs min/max XYZ')
    roi_min, roi_max = _finite_vector(roi['min'], 3, 'stair_roi.min'), _finite_vector(roi['max'], 3, 'stair_roi.max')
    if np.any(roi_min >= roi_max):
        raise CrossfloorError('invalid_config', 'stair_roi must have increasing bounds')
    ranges = config['floor_z_ranges']
    if not isinstance(ranges, dict) or set(ranges) != {'lower', 'upper'}:
        raise CrossfloorError('invalid_config', 'floor_z_ranges needs lower and upper')
    floor_ranges = {key: _finite_vector(ranges[key], 2, 'floor_z_ranges.' + key)
                    for key in ('lower', 'upper')}
    if any(bounds[0] >= bounds[1] for bounds in floor_ranges.values()):
        raise CrossfloorError('invalid_config', 'Floor Z ranges must increase')
    if floor_ranges['lower'][1] >= floor_ranges['upper'][0]:
        raise CrossfloorError('invalid_config', 'Lower and upper floor Z ranges must not overlap')
    limits = {'max_ground_step_m': 0.17, 'max_vertical_grade': 1.0,
              'vertical_step_allowance_m': 0.17, 'max_reverse_height_m': 0.35,
              'min_floor_separation_m': 2.0, 'max_selection_height_error_m': 0.08}
    supplied = config.get('limits', {})
    if not isinstance(supplied, dict) or set(supplied) - set(limits):
        raise CrossfloorError('invalid_config', 'Unknown route limit')
    ceilings = {'max_ground_step_m': 0.30, 'max_vertical_grade': 1.0,
                'vertical_step_allowance_m': 0.30, 'max_reverse_height_m': 1.0,
                'min_floor_separation_m': 20.0, 'max_selection_height_error_m': 0.15}
    for key, default in limits.items():
        limits[key] = _number(supplied.get(key, default), key, 0.001, ceilings[key])
    planning = {'max_heading_rate': 1.0, 'astar_cost_weight': 1.0,
                'optimizer_cost_margin': 8.0, 'path_refinement': 'visibility_c2',
                'refinement_corner_cut_m': 1.5, 'optimizer_sample_interval': 10}
    supplied = config.get('planning', {})
    if not isinstance(supplied, dict) or set(supplied) - set(planning):
        raise CrossfloorError('invalid_config', 'Unknown planning option')
    planning.update(supplied)
    from .planner_core import validate_sample_interval
    try:
        planning['optimizer_sample_interval'] = validate_sample_interval(planning['optimizer_sample_interval'])
    except ValueError as exc:
        raise CrossfloorError('invalid_config', str(exc)) from exc
    if planning['path_refinement'] not in ('none', 'visibility_c2'):
        raise CrossfloorError('invalid_config', 'Unsupported path_refinement')
    for key, ceiling in [('max_heading_rate', 100.0), ('astar_cost_weight', 10.0),
                         ('optimizer_cost_margin', 20.0), ('refinement_corner_cut_m', 5.0)]:
        planning[key] = _number(planning[key], key, 0.001, ceiling)
    policy = config.get('unknown_ceiling_policy', 'allow_unobserved')
    if policy not in ('allow_unobserved', 'reject'):
        raise CrossfloorError('invalid_config', 'unknown_ceiling_policy must be explicit')
    return {**config, 'anchors': normalized_anchors,
            'stair_roi': (roi_min, roi_max), 'floor_z_ranges': floor_ranges,
            'planning': planning, 'limits': limits, 'unknown_ceiling_policy': policy}


def _inside_roi(points, roi):
    values = np.asarray(points, dtype=float)
    return np.all((values >= roi[0] - 1e-6) & (values <= roi[1] + 1e-6), axis=-1)


def _masked_tomogram(original, leg, config):
    """Only remove admissible cells; never synthesize floor or lower a cost."""
    if leg not in {item[0] for item in LEGS}:
        raise ValueError('Unknown fixed map resource')
    # TomogramMap takes its own data copy. Mutate that owned buffer below,
    # instead of making a second, full-size temporary copy before construction.
    payload = {'data': original.data, 'resolution': original.resolution,
               'center': original.center.copy(), 'slice_h0': original.slice_h0,
               'slice_dh': original.slice_dh,
               'selected_source_layers': original.source_layers.copy(),
               'minimum_headroom_m': original.minimum_headroom_m,
               'open_sky_verified': original.open_sky_verified.copy()}
    masked = TomogramMap(payload, minimum_headroom_m=original.minimum_headroom_m,
                         unknown_ceiling_policy=original.unknown_ceiling_policy,
                         max_ground_step_m=original.max_ground_step_m)
    data = masked.data
    bounds = config['floor_z_ranges']
    if leg == 'lower_floor':
        low, high = bounds['lower']
        keep = original.valid & (original.ground >= low) & (original.ground <= high)
    elif leg == 'upper_floor':
        low, high = bounds['upper']
        keep = original.valid & (original.ground >= low) & (original.ground <= high)
    else:
        roi_min, roi_max = config['stair_roi']
        # Broadcast two one-dimensional axes; do not allocate four complete
        # XY index/world-coordinate images just to create a rectangular mask.
        x = original.center[0] + (np.arange(original.nx) - original.offset[0]) * original.resolution
        y = original.center[1] + (np.arange(original.ny) - original.offset[1]) * original.resolution
        xy_inside = (((x >= roi_min[0]) & (x <= roi_max[0]))[:, None]
                     & ((y >= roi_min[1]) & (y <= roi_max[1]))[None, :])
        keep = (original.valid & xy_inside[None, :, :]
                & (original.ground >= roi_min[2]) & (original.ground <= roi_max[2]))
    data[0, ~keep] = 50.0
    data[1:3] = 0
    data[1, :, 1:-1, :] = data[0, :, 2:, :] - data[0, :, :-2, :]
    data[2, :, :, 1:-1] = data[0, :, :, 2:] - data[0, :, :, :-2]
    # Geometry, ceiling policy and costs of retained cells are unchanged. The
    # only validity change is the removal of cells whose cost is now 50.
    masked.valid &= keep
    return masked


def _native_route(tomogram, leg, config, *, defer_map=False):
    from .tomogram_route import TomogramRoute
    planning = config['planning']
    return TomogramRoute(
        tomogram, config['vendor_root'],
        max_heading_rate=planning['max_heading_rate'],
        astar_cost_weight=planning['astar_cost_weight'],
        optimizer_cost_margin=planning['optimizer_cost_margin'],
        optimizer_sample_interval=planning['optimizer_sample_interval'],
        height_tolerance_m=config['limits']['max_selection_height_error_m'],
        path_refinement='none' if leg.startswith('stair_') else planning['path_refinement'],
        refinement_corner_cut_m=planning['refinement_corner_cut_m'], defer_map=defer_map)


def _check_leg(result, original, masked, leg, begin, end, config):
    points = np.asarray(result.get('path'), dtype=float)
    layers = np.asarray(result.get('layer_ids'))
    if (points.ndim != 2 or points.shape[1] != 3 or len(points) < 2
            or not np.isfinite(points).all() or layers.shape != (len(points),)):
        raise CrossfloorError('invalid_segment', f'{leg} returned malformed XYZ/layers')
    if (not np.array_equal(points[0], begin['xyz']) or not np.array_equal(points[-1], end['xyz'])
            or int(layers[0]) != begin['layer_id'] or int(layers[-1]) != end['layer_id']):
        raise CrossfloorError('segment_endpoint_mismatch', f'{leg} changed an exact portal endpoint')
    step = config['limits']['max_ground_step_m']
    try:
        masked_checks = masked.validate_path(points, layers, max_ground_step_m=step)
        original_checks = original.validate_path(points, layers, max_ground_step_m=step)
    except (ValueError, TypeError, IndexError) as exc:
        raise CrossfloorError('segment_geometry_rejected', f'{leg} failed full geometry validation',
                              cause_code=getattr(exc, 'code', type(exc).__name__), cause=str(exc)) from exc
    if leg.startswith('stair_'):
        if not np.all(_inside_roi(points, config['stair_roi'])):
            raise CrossfloorError('stair_outside_roi', f'{leg} leaves its configured 3D stair ROI')
    else:
        low, high = config['floor_z_ranges']['lower' if leg == 'lower_floor' else 'upper']
        if np.any(points[:, 2] < low - 1e-6) or np.any(points[:, 2] > high + 1e-6):
            raise CrossfloorError('floor_height_escape', f'{leg} leaves its declared floor height range')
        if np.any(np.diff(layers.astype(int)) != 0):
            # Multiple PCT slices can represent the same floor; permit only
            # switches at equal physical height, never an unnoticed climb.
            for i in np.flatnonzero(np.diff(layers.astype(int))):
                if abs(points[i + 1, 2] - points[i, 2]) > 0.05:
                    raise CrossfloorError('floor_layer_escape', f'{leg} changes height at a layer switch')
    return points, layers.astype(int), {'masked': masked_checks, 'original': original_checks}


def _check_stair_profile(points, edge_legs, config):
    limits = config['limits']
    entry, landing, exit_ = (config['anchors'][name]['xyz'] for name in ('entry', 'landing', 'exit'))
    direction = np.sign(exit_[2] - entry[2])
    if (direction == 0 or direction * (landing[2] - entry[2]) < -limits['max_reverse_height_m']
            or direction * (exit_[2] - landing[2]) < -limits['max_reverse_height_m']):
        raise CrossfloorError('invalid_portal_order', 'Stair landing is not between floor heights')
    stair_indices = [i for i, leg in enumerate(edge_legs) if leg.startswith('stair_')]
    if not stair_indices:
        raise CrossfloorError('missing_stair', 'No stair edges in the output route')
    a, b = stair_indices[0], stair_indices[-1] + 1
    stair = points[a:b + 1]
    if not np.all(_inside_roi(stair, config['stair_roi'])):
        raise CrossfloorError('stair_outside_roi', 'Stair path leaves its 3D ROI')
    diff = np.diff(stair, axis=0)
    horizontal = np.linalg.norm(diff[:, :2], axis=1)
    too_steep = np.abs(diff[:, 2]) > (
        limits['max_vertical_grade'] * horizontal
        + limits['vertical_step_allowance_m'] + 1e-6)
    if np.any(too_steep):
        raise CrossfloorError('stair_vertical_jump', 'Stair has a vertical jump beyond configured grade/step limits',
                              edge=int(np.flatnonzero(too_steep)[0]) + a)
    z = direction * stair[:, 2]
    reverse = np.maximum.accumulate(z) - z
    if np.max(reverse) > limits['max_reverse_height_m'] + 1e-6:
        raise CrossfloorError('stair_height_reversal', 'Stair route descends back toward the starting floor',
                              maximum_reverse_m=float(np.max(reverse)))
    return {'direction': 'up' if direction > 0 else 'down',
            'height_change_m': float(exit_[2] - entry[2]),
            'max_reverse_height_m': float(np.max(reverse)),
            'max_edge_grade': float(np.max(np.abs(diff[:, 2]) / np.maximum(horizontal, 1e-6)))}


def plan_crossfloor(config, tomogram, route_factory=None, *, resources=None,
                    fixed_leg_cache=None):
    """Plan and verify four ordered legs. route_factory is an injectable test seam."""
    settings = validate_config(config)
    anchors, limits = settings['anchors'], settings['limits']
    for name, anchor in anchors.items():
        try:
            tomogram.validate_endpoint(anchor['xyz'], anchor['layer_id'],
                                       limits['max_selection_height_error_m'])
        except ValueError as exc:
            raise CrossfloorError('invalid_anchor', f'{name} is not on an allowed measured surface',
                                  cause_code=getattr(exc, 'code', type(exc).__name__)) from exc
    lower, upper = settings['floor_z_ranges']['lower'], settings['floor_z_ranges']['upper']
    for name in ('start', 'entry'):
        if not lower[0] <= anchors[name]['xyz'][2] <= lower[1]:
            raise CrossfloorError('invalid_anchor_floor', f'{name} is not on the lower floor')
    for name in ('exit', 'goal'):
        if not upper[0] <= anchors[name]['xyz'][2] <= upper[1]:
            raise CrossfloorError('invalid_anchor_floor', f'{name} is not on the upper floor')
    for name in ('entry', 'landing', 'exit'):
        if not _inside_roi(anchors[name]['xyz'], settings['stair_roi']):
            raise CrossfloorError('invalid_anchor_roi', f'{name} is outside the stair ROI')
    if abs(anchors['exit']['xyz'][2] - anchors['entry']['xyz'][2]) < limits['min_floor_separation_m']:
        raise CrossfloorError('floor_separation_too_small', 'Configured portals do not span two floors')
    if resources is not None and route_factory is not None:
        raise ValueError('Use either owned native resources or an injected route factory')
    route_factory = route_factory or _native_route
    whole_points, whole_layers, edge_legs, records = [], [], [], []
    for name, start_name, end_name in LEGS:
        begin, end = anchors[start_name], anchors[end_name]
        try:
            if resources is None:
                masked = _masked_tomogram(tomogram, name, settings)
                route = route_factory(masked, name, settings)
            else:
                masked, route = resources(name, settings)
            raw = (fixed_leg_cache.get(tomogram, name, begin, end, settings)
                   if fixed_leg_cache is not None else None)
            cached = raw is not None
            if raw is None:
                raw = route.plan(begin['xyz'], end['xyz'], begin['layer_id'], end['layer_id'])
        except Exception as exc:
            raise CrossfloorError('segment_planning_failed', f'Native PCT failed on {name}',
                                  cause_code=getattr(exc, 'code', type(exc).__name__), cause=str(exc)) from exc
        if not isinstance(raw, dict):
            raise CrossfloorError('segment_planning_failed', f'Native PCT returned no route on {name}')
        points, layers, checks = _check_leg(raw, tomogram, masked, name, begin, end, settings)
        # A reused stair curve must still pass current exact endpoints,
        # original/masked geometry, ROI and all joined-route checks below.
        # Never cache an unvalidated native result or arbitrary floor route.
        if fixed_leg_cache is not None and not cached:
            fixed_leg_cache.put(tomogram, name, begin, end, settings, raw)
        if whole_points:
            if not np.array_equal(points[0], whole_points[-1]) or int(layers[0]) != whole_layers[-1]:
                raise CrossfloorError('portal_mismatch', f'{name} does not connect to previous leg')
            first_index = len(whole_points) - 1
            whole_points.extend(points[1:])
            whole_layers.extend(layers[1:].tolist())
        else:
            first_index = 0
            whole_points.extend(points)
            whole_layers.extend(layers.tolist())
        edge_legs.extend([name] * (len(points) - 1))
        records.append({'name': name, 'from': start_name, 'to': end_name,
                        'first_index': first_index, 'last_index': len(whole_points) - 1,
                        'length_m': float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum()),
                        'layer_ids': sorted(set(layers.tolist())), 'checks': checks,
                        'native_algorithm': raw.get('algorithm', 'PCT route (test seam)')})
        records[-1].update(curve_validation=raw.get('curve_validation'),
                           quintic_segments=raw.get('quintic_segments', 0),
                           curve_partition_points=raw.get('curve_partition_points', 0))
        records[-1]['fixed_leg_cache_hit'] = cached
    points = np.asarray(whole_points)
    layers = np.asarray(whole_layers, dtype=int)
    try:
        whole_check = tomogram.validate_path(points, layers,
                                             max_ground_step_m=limits['max_ground_step_m'])
    except ValueError as exc:
        raise CrossfloorError('joined_geometry_rejected', 'Joined PCT path failed original-map validation',
                              cause_code=getattr(exc, 'code', type(exc).__name__), cause=str(exc)) from exc
    transitions = []
    for index in np.flatnonzero(np.diff(layers)):
        leg = edge_legs[index]
        if not leg.startswith('stair_'):
            if abs(points[index + 1, 2] - points[index, 2]) > 0.05:
                raise CrossfloorError('transition_outside_stair', 'A height-changing layer transition is outside the stair',
                                      edge=int(index), leg=leg)
        elif not np.all(_inside_roi(points[index:index + 2], settings['stair_roi'])):
            raise CrossfloorError('transition_outside_stair', 'A stair layer transition is outside its ROI',
                                  edge=int(index))
        transitions.append({'edge': int(index), 'leg': leg,
                            'from_layer': int(layers[index]), 'to_layer': int(layers[index + 1]),
                            'xyz': points[index:index + 2].mean(axis=0).tolist()})
    if not any(event['leg'].startswith('stair_') for event in transitions):
        raise CrossfloorError('no_stair_gateway', 'Route contains no native PCT layer transition in the stair ROI')
    stair_profile = _check_stair_profile(points, edge_legs, settings)
    return {'status': 'validated_offline_route', 'frame_id': settings['frame_id'],
            'tomogram_path': str(tomogram.source), 'tomogram_sha256': tomogram.sha256,
            'source_pcd': settings['source_pcd'],
            'source_pcd_sha256': tomogram.provenance.get('source_sha256'),
            'source_layer_ids': tomogram.source_layers[layers].astype(int).tolist(),
            'path_xyz': points.tolist(), 'layer_ids': layers.tolist(), 'edge_legs': edge_legs,
            'anchors': {name: {'xyz': item['xyz'].tolist(), 'layer_id': item['layer_id']}
                        for name, item in anchors.items()},
            'segments': records, 'layer_transitions': transitions,
            'stair_profile': stair_profile, 'whole_route_check': whole_check,
            'length_m': float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum()),
            'execution_authorized': False}


def _sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def run_from_config(config_path, route_factory=None):
    config_path = Path(config_path).resolve(strict=True)
    from .paths import expand_tree
    raw = expand_tree(yaml.safe_load(config_path.read_text()))
    settings = validate_config(raw)
    for key in ('tomogram_path', 'source_pcd', 'vendor_root'):
        target = Path(settings[key])
        if not (target.is_dir() if key == 'vendor_root' else target.is_file()):
            raise CrossfloorError('missing_input', f'{key} does not exist', path=str(target))
    tomogram = TomogramMap(settings['tomogram_path'],
                           unknown_ceiling_policy=settings['unknown_ceiling_policy'],
                           max_ground_step_m=settings['limits']['max_ground_step_m'])
    tomogram.verify_source(settings['source_pcd'], settings['frame_id'])
    from .official_pipeline import unique_output
    output = unique_output(settings['output_directory'])
    base = {'schema_version': 1, 'created_utc': datetime.now(timezone.utc).isoformat(),
            'config_path': str(config_path), 'config_sha256': _sha256(config_path),
            'output_directory': str(output), 'tomogram_sha256': tomogram.sha256,
            'source_pcd_sha256': tomogram.provenance['source_sha256'],
            'execution_authorized': False}
    try:
        result = plan_crossfloor(raw, tomogram, route_factory=route_factory)
    except Exception as exc:
        failure = {**base, 'status': 'failed', 'error': {
            'code': getattr(exc, 'code', type(exc).__name__), 'message': str(exc),
            'details': getattr(exc, 'details', {})}}
        (output / 'audit.json').write_text(json.dumps(failure, indent=2, allow_nan=False) + '\n')
        raise
    audit = {**base, **result}
    with (output / 'path.csv').open('x', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['index', 'x_m', 'y_m', 'z_m', 'layer_id', 'source_layer_id', 'incoming_leg', 'outgoing_leg'])
        for index, (xyz, layer, source) in enumerate(zip(
                result['path_xyz'], result['layer_ids'], result['source_layer_ids'])):
            writer.writerow([index, *xyz, layer, source,
                             result['edge_legs'][index - 1] if index else '',
                             result['edge_legs'][index] if index < len(result['edge_legs']) else ''])
    (output / 'audit.json').write_text(json.dumps(audit, indent=2, allow_nan=False) + '\n')
    return output, audit


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    args = parser.parse_args(argv)
    if os.environ.get('D1MAX_PCT_CROSSFLOOR_CHILD') != '1':
        # The bundled GTSAM ABI must be chosen before Python loads native PCT.
        # Re-exec in a child exactly as the existing preview worker does; do
        # not mutate this process or the user's ROS/Zenoh environment.
        from .native_runtime import prepare_native_environment
        from .paths import expand_tree
        settings = validate_config(expand_tree(yaml.safe_load(args.config.read_text())))
        environment = prepare_native_environment(settings['vendor_root'])
        environment['D1MAX_PCT_CROSSFLOOR_CHILD'] = '1'
        completed = subprocess.run([sys.executable, '-m', 'd1max_pct_planner.crossfloor_route',
                                    '--config', str(args.config)],
                                   env=environment, check=False)
        return completed.returncode
    output, result = run_from_config(args.config)
    print(json.dumps({'status': result['status'], 'output_directory': str(output),
                      'length_m': result['length_m'],
                      'layer_transitions': len(result['layer_transitions']),
                      'execution_authorized': False}, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
