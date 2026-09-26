"""Inspection-only floor/stair/floor composite, never a navigation route.

The two floor legs are newly solved and rechecked with native PCT. The middle
line is measured ground from a *rejected* stair audit, not a validated path.
Run: python3 -m d1max_pct_planner.crossfloor_candidate --route-config ROUTE.yaml
     --candidate-config CANDIDATE.yaml
No ROS Path, SCAN reference, controller command, or SDK message is published.
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

from .crossfloor_route import (_check_leg, _masked_tomogram, _native_route,
                               validate_config as validate_route_config)
from .tomogram_map import TomogramMap


class CandidateError(ValueError):
    def __init__(self, code, message, **details):
        super().__init__(message)
        self.code, self.details = code, details


def _sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def validate_candidate_config(config):
    if not isinstance(config, dict):
        raise CandidateError('invalid_config', 'Candidate parameters must be a YAML mapping')
    required = {'schema_version', 'measured_stair_audit', 'output_directory',
                'max_connector_gap_m', 'max_sample_spacing_m'}
    if set(config) != required or config['schema_version'] != 1:
        raise CandidateError('invalid_config', 'Missing/unknown candidate parameters or schema version')
    for name in ('measured_stair_audit', 'output_directory'):
        if not isinstance(config[name], str) or not Path(config[name]).is_absolute():
            raise CandidateError('invalid_config', f'{name} must be an absolute path')
    for name, ceiling in (('max_connector_gap_m', 0.15), ('max_sample_spacing_m', 0.20)):
        value = config[name]
        if (isinstance(value, bool) or not isinstance(value, (float, int))
                or not np.isfinite(value) or not 0 < value <= ceiling):
            raise CandidateError('invalid_config', f'{name} must be in (0,{ceiling}] m')
    return config


def _finite_xyz(value, name):
    point = np.asarray(value, dtype=float)
    if point.shape != (3,) or not np.isfinite(point).all():
        raise CandidateError('invalid_audit', f'{name} needs finite XYZ')
    return point


def _load_rejected_audit(path, route_config, tomogram, candidate_config):
    path = Path(path).resolve(strict=True)
    audit = json.loads(path.read_text())
    if (not isinstance(audit, dict) or audit.get('schema') != 'd1max.measured_stair_connector/v1'
            or audit.get('status') != 'rejected' or audit.get('geometry_gates_passed') is not False
            or audit.get('execution_authorized') is not False
            or audit.get('route_certified_safe') is not False):
        raise CandidateError('audit_not_rejected', 'Only an explicitly rejected measured-stair audit is accepted')
    source = audit.get('source')
    if not isinstance(source, dict):
        raise CandidateError('invalid_audit', 'Stair audit has no source provenance')
    if (Path(source.get('pcd_path', '')).resolve() != Path(route_config['source_pcd']).resolve()
            or source.get('pcd_sha256') != tomogram.provenance.get('source_sha256')
            or source.get('declared_frame_id') != route_config['frame_id']):
        raise CandidateError('audit_source_mismatch', 'Stair audit PCD hash/path/frame differs from the PCT map')
    poses_path = Path(source.get('optimized_poses_path', ''))
    if not poses_path.is_file() or _sha256(poses_path) != source.get('optimized_poses_sha256'):
        raise CandidateError('audit_source_mismatch', 'Optimized trajectory source changed after stair audit')
    samples = audit.get('samples')
    if (not isinstance(samples, list) or len(samples) < 2
            or audit.get('sample_count') != len(samples)):
        raise CandidateError('invalid_audit', 'Stair audit needs at least two indexed measured samples')
    measured, failures, distances = [], [], []
    for index, sample in enumerate(samples):
        if not isinstance(sample, dict) or sample.get('sample_index') != index:
            raise CandidateError('invalid_audit', 'Stair sample indices are not contiguous')
        ground = _finite_xyz(sample.get('measured_ground_xyz'), f'sample {index} ground')
        body = _finite_xyz(sample.get('candidate_body_xyz'), f'sample {index} body')
        if np.linalg.norm(ground[:2] - body[:2]) > 1e-6:
            raise CandidateError('invalid_audit', 'Measured ground does not share the candidate station XY')
        codes = sample.get('failure_codes')
        if not isinstance(codes, list) or any(not isinstance(code, str) for code in codes):
            raise CandidateError('invalid_audit', 'Stair failure codes are malformed')
        distance = sample.get('xy_distance_m')
        if isinstance(distance, bool) or not isinstance(distance, (float, int)) or not np.isfinite(distance):
            raise CandidateError('invalid_audit', 'Stair station distance is invalid')
        measured.append(ground)
        failures.append(codes)
        distances.append(float(distance))
    measured = np.asarray(measured)
    spacings = np.linalg.norm(np.diff(measured[:, :2], axis=0), axis=1)
    # xy_distance_m indexes the original trajectory stations; projected ground
    # estimates can move laterally, so their XY chord need not equal station delta.
    station_steps = np.diff(distances)
    if (abs(distances[0]) > 1e-6 or np.any(station_steps <= 0)
            or np.any(station_steps > candidate_config['max_sample_spacing_m'] + 1e-6)
            or np.any(spacings > candidate_config['max_sample_spacing_m'] + 1e-6)):
        raise CandidateError('stair_sample_gap', 'Stair measured samples have a gap or inconsistent spacing')
    low, high = route_config['stair_roi']
    if not np.all((measured >= low - 1e-6) & (measured <= high + 1e-6)):
        raise CandidateError('stair_outside_roi', 'Measured candidate leaves the declared stair ROI')
    connectors = {}
    for label, point, portal in (('entry', measured[0], route_config['anchors']['entry']['xyz']),
                                 ('exit', measured[-1], route_config['anchors']['exit']['xyz'])):
        gap = float(np.linalg.norm(point - portal))
        connectors[label] = {'portal_xyz': portal.tolist(), 'measured_xyz': point.tolist(),
                             'gap_m': gap, 'within_limit': gap <= candidate_config['max_connector_gap_m']}
        if not connectors[label]['within_limit']:
            raise CandidateError('connector_gap', f'{label} measured ground does not meet the PCT portal',
                                 gap_m=gap, limit_m=candidate_config['max_connector_gap_m'])
    rejection = {'status': audit['status'], 'geometry_gates_passed': False,
                 'execution_authorized': False, 'route_certified_safe': False,
                 'failure_counts': audit.get('failure_counts', {}),
                 'limitations': audit.get('limitations', []),
                 'source_audit': str(path), 'source_audit_sha256': _sha256(path),
                 'source': source, 'sample_count': len(samples),
                 'measured_floor_rise_m': audit.get('measured_floor_rise_m')}
    return measured, failures, connectors, rejection


def _plan_verified_floor(tomogram, route_config, leg, from_name, to_name, factory):
    masked = _masked_tomogram(tomogram, leg, route_config)
    first = route_config['anchors'][from_name]
    last = route_config['anchors'][to_name]
    for name, anchor in ((from_name, first), (to_name, last)):
        try:
            tomogram.validate_endpoint(anchor['xyz'], anchor['layer_id'],
                                       route_config['limits']['max_selection_height_error_m'])
        except ValueError as exc:
            raise CandidateError('invalid_floor_portal', f'{name} is not on the measured PCT floor',
                                 cause_code=getattr(exc, 'code', type(exc).__name__)) from exc
    try:
        raw = factory(masked, leg, route_config).plan(
            first['xyz'], last['xyz'], first['layer_id'], last['layer_id'])
    except Exception as exc:
        raise CandidateError('floor_pct_failed', f'Native PCT failed on {leg}',
                             cause_code=getattr(exc, 'code', type(exc).__name__), cause=str(exc)) from exc
    if not isinstance(raw, dict):
        raise CandidateError('floor_pct_failed', f'Native PCT returned no path for {leg}')
    try:
        points, layers, checks = _check_leg(raw, tomogram, masked, leg, first, last, route_config)
    except ValueError as exc:
        raise CandidateError('floor_pct_rejected', f'{leg} failed original-tomogram validation',
                             cause_code=getattr(exc, 'code', type(exc).__name__), cause=str(exc)) from exc
    return {'segment_id': leg, 'label': 'verified_floor_pct', 'source': 'native_pct_newly_planned',
            'xyz': points.tolist(), 'layer_ids': layers.tolist(),
            'source_layer_ids': tomogram.source_layers[layers].astype(int).tolist(),
            'length_m': float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum()),
            'checks': checks, 'algorithm': raw.get('algorithm', 'native_pct')}


def build_candidate(route_config, candidate_config, tomogram, audit_path, floor_route_factory=None):
    """Build inspectable *segments*, not an executable joined route."""
    route = validate_route_config(route_config)
    candidate = validate_candidate_config(candidate_config)
    measured, failures, connectors, rejection = _load_rejected_audit(
        audit_path, route, tomogram, candidate)
    factory = floor_route_factory or _native_route
    lower = _plan_verified_floor(tomogram, route, 'lower_floor', 'start', 'entry', factory)
    upper = _plan_verified_floor(tomogram, route, 'upper_floor', 'exit', 'goal', factory)
    stair = {'segment_id': 'measured_stair', 'label': 'rejected_stair_candidate',
             'source': 'measured_ground_from_rejected_stair_audit',
             'xyz': measured.tolist(), 'layer_ids': [None] * len(measured),
             'source_layer_ids': [None] * len(measured),
             'sample_failure_codes': failures,
             'length_m': float(np.linalg.norm(np.diff(measured[:, :2], axis=0), axis=1).sum()),
             'validated_for_navigation': False}
    return {'schema': 'd1max.crossfloor_candidate/v1',
            'status': 'inspection_only_rejected_stair',
            'complete_validated_route': False, 'execution_authorized': False,
            'scan_handoff_allowed': False, 'controller_handoff_allowed': False,
            'frame_id': route['frame_id'], 'tomogram_path': tomogram.source,
            'tomogram_sha256': tomogram.sha256,
            'source_pcd': route['source_pcd'],
            'source_pcd_sha256': tomogram.provenance.get('source_sha256'),
            'stair_rejection': rejection, 'connector_checks': connectors,
            'inspection_segments': [lower, stair, upper],
            'caveat': 'The measured stair line failed geometry gates; the segment list is for RViz inspection only. '
                      'It is not a validated joined path, and must not be passed to SCAN or control.'}


def _write_csv(path, segments):
    with path.open('x', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['segment_id', 'verification_label', 'point_index', 'x_m', 'y_m', 'z_m',
                         'layer_id', 'source_layer_id', 'stair_failure_codes'])
        for segment in segments:
            codes = segment.get('sample_failure_codes', [[]] * len(segment['xyz']))
            for index, (xyz, layer, source_layer, failures) in enumerate(zip(
                    segment['xyz'], segment['layer_ids'], segment['source_layer_ids'], codes)):
                writer.writerow([segment['segment_id'], segment['label'], index, *xyz,
                                 '' if layer is None else layer,
                                 '' if source_layer is None else source_layer,
                                 '|'.join(failures)])


def run_from_configs(route_config_path, candidate_config_path, floor_route_factory=None):
    route_config_path = Path(route_config_path).resolve(strict=True)
    candidate_config_path = Path(candidate_config_path).resolve(strict=True)
    route_raw = yaml.safe_load(route_config_path.read_text())
    candidate_raw = yaml.safe_load(candidate_config_path.read_text())
    route = validate_route_config(route_raw)
    candidate = validate_candidate_config(candidate_raw)
    tomogram = TomogramMap(route['tomogram_path'],
                           unknown_ceiling_policy=route['unknown_ceiling_policy'],
                           max_ground_step_m=route['limits']['max_ground_step_m'])
    tomogram.verify_source(route['source_pcd'], route['frame_id'])
    # Check the rejected-audit provenance *before* allocating a new result.
    measured, failures, connectors, rejection = _load_rejected_audit(
        candidate['measured_stair_audit'], route, tomogram, candidate)
    del measured, failures, connectors
    from .official_pipeline import unique_output
    output = unique_output(candidate['output_directory'])
    base = {'created_utc': datetime.now(timezone.utc).isoformat(),
            'route_config_path': str(route_config_path),
            'route_config_sha256': _sha256(route_config_path),
            'candidate_config_path': str(candidate_config_path),
            'candidate_config_sha256': _sha256(candidate_config_path),
            'output_directory': str(output), 'stair_rejection': rejection,
            'complete_validated_route': False, 'execution_authorized': False,
            'scan_handoff_allowed': False, 'controller_handoff_allowed': False}
    try:
        result = build_candidate(route_raw, candidate_raw, tomogram,
                                 candidate['measured_stair_audit'], floor_route_factory)
    except Exception as exc:
        failure = {**base, 'status': 'failed_floor_pct_or_connector',
                   'error': {'code': getattr(exc, 'code', type(exc).__name__),
                             'message': str(exc), 'details': getattr(exc, 'details', {})}}
        (output / 'audit.json').write_text(json.dumps(failure, ensure_ascii=False, indent=2,
                                                       allow_nan=False) + '\n')
        raise
    result = {**base, **result}
    _write_csv(output / 'candidate_inspection_only.csv', result['inspection_segments'])
    (output / 'audit.json').write_text(json.dumps(result, ensure_ascii=False, indent=2,
                                                   allow_nan=False) + '\n')
    return output, result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--route-config', type=Path, required=True)
    parser.add_argument('--candidate-config', type=Path, required=True)
    args = parser.parse_args(argv)
    if os.environ.get('D1MAX_PCT_CANDIDATE_CHILD') != '1':
        from .native_runtime import prepare_native_environment
        route = validate_route_config(yaml.safe_load(args.route_config.read_text()))
        environment = prepare_native_environment(route['vendor_root'])
        environment['D1MAX_PCT_CANDIDATE_CHILD'] = '1'
        return subprocess.run([sys.executable, '-m', 'd1max_pct_planner.crossfloor_candidate',
                               '--route-config', str(args.route_config),
                               '--candidate-config', str(args.candidate_config)],
                              env=environment, check=False).returncode
    output, report = run_from_configs(args.route_config, args.candidate_config)
    print(json.dumps({'status': report['status'], 'output_directory': str(output),
                      'floor_segments_verified': 2, 'stair_status': 'rejected',
                      'complete_validated_route': False, 'execution_authorized': False},
                     ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
