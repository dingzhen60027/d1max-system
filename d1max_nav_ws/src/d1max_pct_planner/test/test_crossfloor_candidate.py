import hashlib
import json

import numpy as np
import pytest
import yaml

from d1max_pct_planner.crossfloor_candidate import (
    CandidateError, build_candidate, main, run_from_configs,
)
from d1max_pct_planner.tomogram_map import TomogramMap


def _hash(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _xyz(cell):
    return [0.2 * (cell - 20), 0., max(0., min(3., (cell - 5) * .1))]


def _route_config(tomogram, pcd, output):
    return {'schema_version': 1, 'tomogram_path': str(tomogram),
            'source_pcd': str(pcd), 'vendor_root': str(output.parent),
            'output_directory': str(output.parent / 'unused_strict'),
            'frame_id': 'd1max_loc_map',
            'anchors': {
                'start': {'xyz': _xyz(1), 'layer_id': 0},
                'entry': {'xyz': _xyz(5), 'layer_id': 0},
                'landing': {'xyz': _xyz(21), 'layer_id': 1},
                'exit': {'xyz': _xyz(35), 'layer_id': 1},
                'goal': {'xyz': _xyz(39), 'layer_id': 1}},
            'stair_roi': {'min': [_xyz(4)[0], -.1, -.1],
                          'max': [_xyz(36)[0], .1, 3.1]},
            'floor_z_ranges': {'lower': [-.1, .1], 'upper': [2.9, 3.1]}}


def _prepare(tmp_path):
    pcd = tmp_path / 'source.pcd'
    pcd.write_bytes(b'pointcloud fixture')
    poses = tmp_path / 'optimized_poses.txt'
    poses.write_text('0 0 0 0 0 0 0 0 0 0 0 0\n')
    data = np.zeros((5, 2, 41, 3), np.float32)
    data[0] = 50
    data[3:] = np.nan
    for layer, cells in ((0, range(24)), (1, range(18, 41))):
        for x in cells:
            data[0, layer, x, 1] = 0
            data[3, layer, x, 1] = _xyz(x)[2]
            data[4, layer, x, 1] = _xyz(x)[2] + 2
    tomogram = tmp_path / 'tomogram.npz'
    np.savez_compressed(tomogram, data=data, resolution=.2, center=[0., 0.],
                        slice_h0=.5, slice_dh=.5, selected_source_layers=[0, 6],
                        source_pcd=str(pcd), source_sha256=_hash(pcd), frame_id='d1max_loc_map')
    route = _route_config(tomogram, pcd, tmp_path / 'candidate')
    samples = [{'sample_index': i, 'xy_distance_m': .2 * i,
                'candidate_body_xyz': [*_xyz(cell)[:2], _xyz(cell)[2] + .475],
                'measured_ground_xyz': _xyz(cell),
                'failure_codes': ['potential_body_obstacle'] if i == 10 else []}
               for i, cell in enumerate(range(5, 36))]
    audit = {'schema': 'd1max.measured_stair_connector/v1', 'status': 'rejected',
             'geometry_gates_passed': False, 'execution_authorized': False,
             'route_certified_safe': False, 'sample_count': len(samples),
             'failure_counts': {'potential_body_obstacle': 1},
             'limitations': ['Fused PCD cannot certify clearance'],
             'measured_floor_rise_m': 3.0, 'samples': samples,
             'source': {'pcd_path': str(pcd), 'pcd_sha256': _hash(pcd),
                        'optimized_poses_path': str(poses), 'optimized_poses_sha256': _hash(poses),
                        'declared_frame_id': 'd1max_loc_map', 'frame_independently_verified': False}}
    audit_path = tmp_path / 'measured_stair_audit.json'
    audit_path.write_text(json.dumps(audit))
    candidate = {'schema_version': 1, 'measured_stair_audit': str(audit_path),
                 'output_directory': str(tmp_path / 'candidate'),
                 'max_connector_gap_m': .08, 'max_sample_spacing_m': .2}
    return route, candidate, audit, audit_path, tomogram


def _floor_factory(_map, leg, _config):
    class Route:
        def plan(self, *_args):
            cells = range(1, 6) if leg == 'lower_floor' else range(35, 40)
            layer = 0 if leg == 'lower_floor' else 1
            return {'path': [_xyz(x) for x in cells],
                    'layer_ids': [layer] * len(cells), 'algorithm': 'fixture PCT'}
    return Route()


def test_rejected_stair_remains_rejected_between_two_newly_validated_floor_legs(tmp_path):
    route, candidate, audit, audit_path, tomogram_path = _prepare(tmp_path)
    result = build_candidate(route, candidate, TomogramMap(tomogram_path),
                             audit_path, _floor_factory)
    assert [segment['label'] for segment in result['inspection_segments']] == [
        'verified_floor_pct', 'rejected_stair_candidate', 'verified_floor_pct']
    assert result['complete_validated_route'] is False
    assert result['execution_authorized'] is False
    assert result['scan_handoff_allowed'] is False
    assert result['controller_handoff_allowed'] is False
    assert result['stair_rejection']['failure_counts'] == audit['failure_counts']
    assert result['inspection_segments'][1]['layer_ids'] == [None] * audit['sample_count']
    assert result['inspection_segments'][1]['sample_failure_codes'][10] == ['potential_body_obstacle']
    assert result['connector_checks']['entry']['gap_m'] == 0
    assert result['connector_checks']['exit']['gap_m'] == 0
    assert all(segment.get('checks') for segment in
               (result['inspection_segments'][0], result['inspection_segments'][2]))


def test_stair_rejection_and_connector_discrepancy_cannot_be_promoted(tmp_path):
    route, candidate, audit, audit_path, tomogram_path = _prepare(tmp_path)
    audit['status'] = 'candidate_only'
    audit_path.write_text(json.dumps(audit))
    with pytest.raises(CandidateError) as error:
        build_candidate(route, candidate, TomogramMap(tomogram_path), audit_path, _floor_factory)
    assert error.value.code == 'audit_not_rejected'
    audit['status'] = 'rejected'
    audit['samples'][0]['measured_ground_xyz'][2] += .2
    audit_path.write_text(json.dumps(audit))
    with pytest.raises(CandidateError) as error:
        build_candidate(route, candidate, TomogramMap(tomogram_path), audit_path, _floor_factory)
    assert error.value.code == 'connector_gap'


def test_floor_leg_must_replan_and_pass_original_map_checks(tmp_path):
    route, candidate, _audit, audit_path, tomogram_path = _prepare(tmp_path)

    def unsafe_factory(_map, leg, _config):
        class Route:
            def plan(self, *_args):
                if leg == 'lower_floor':
                    return {'path': [_xyz(1), [100., 100., 0.], _xyz(5)],
                            'layer_ids': [0, 0, 0]}
                return _floor_factory(_map, leg, _config).plan()
        return Route()

    with pytest.raises(CandidateError) as error:
        build_candidate(route, candidate, TomogramMap(tomogram_path), audit_path, unsafe_factory)
    assert error.value.code == 'floor_pct_rejected'


def test_cli_artifacts_are_inspection_only_and_failed_floor_writes_no_csv(tmp_path):
    route, candidate, _audit, _audit_path, _tomogram_path = _prepare(tmp_path)
    route_file, candidate_file = tmp_path / 'route.yaml', tmp_path / 'candidate.yaml'
    route_file.write_text(yaml.safe_dump(route))
    candidate_file.write_text(yaml.safe_dump(candidate))
    output, result = run_from_configs(route_file, candidate_file, _floor_factory)
    report = json.loads((output / 'audit.json').read_text())
    assert report['status'] == 'inspection_only_rejected_stair'
    assert report['complete_validated_route'] is False
    assert report['execution_authorized'] is False
    csv_text = (output / 'candidate_inspection_only.csv').read_text()
    assert 'verified_floor_pct' in csv_text and 'rejected_stair_candidate' in csv_text
    assert 'potential_body_obstacle' in csv_text
    assert not (output / 'path.csv').exists()

    def failed_factory(_map, _leg, _config):
        class Failed:
            def plan(self, *_args):
                return None
        return Failed()

    with pytest.raises(CandidateError):
        run_from_configs(route_file, candidate_file, failed_factory)
    failure_output = tmp_path / 'candidate_001'
    failure = json.loads((failure_output / 'audit.json').read_text())
    assert failure['status'] == 'failed_floor_pct_or_connector'
    assert failure['stair_rejection']['status'] == 'rejected'
    assert failure['complete_validated_route'] is False
    assert not (failure_output / 'candidate_inspection_only.csv').exists()


def test_cli_reexecs_with_verified_native_abi_before_planning(tmp_path, monkeypatch):
    from d1max_pct_planner import crossfloor_candidate, native_runtime
    route, candidate, _audit, _audit_path, _tomogram_path = _prepare(tmp_path)
    route_file, candidate_file = tmp_path / 'route.yaml', tmp_path / 'candidate.yaml'
    route_file.write_text(yaml.safe_dump(route))
    candidate_file.write_text(yaml.safe_dump(candidate))
    monkeypatch.delenv('D1MAX_PCT_CANDIDATE_CHILD', raising=False)
    monkeypatch.setattr(native_runtime, 'prepare_native_environment',
                        lambda root: {'TEST_ROOT': root})
    called = {}

    def fake_run(command, *, env, check):
        called.update(command=command, env=env, check=check)
        return type('Result', (), {'returncode': 9})()

    monkeypatch.setattr(crossfloor_candidate.subprocess, 'run', fake_run)
    assert main(['--route-config', str(route_file), '--candidate-config', str(candidate_file)]) == 9
    assert called['command'][1:3] == ['-m', 'd1max_pct_planner.crossfloor_candidate']
    assert called['env']['D1MAX_PCT_CANDIDATE_CHILD'] == '1'
