import hashlib
import json

import numpy as np
import pytest
import yaml

from d1max_pct_planner.crossfloor_route import (
    CrossfloorError, STAIR_REFINEMENT_PROOF, _masked_tomogram, _native_route,
    main, plan_crossfloor, run_from_config, validate_config, _stationary_floor_leg,
)
from d1max_pct_planner.tomogram_map import TomogramMap


def ramp_height(x):
    return min(3.0, max(0.0, (x - 5) * 0.1))


def scene(source=None, frame='d1max_loc_map'):
    data = np.zeros((5, 2, 41, 3), dtype=np.float32)
    data[0] = 50.0
    data[3:] = np.nan
    for layer, cells in ((0, range(0, 24)), (1, range(18, 41))):
        for x in cells:
            data[0, layer, x, 1] = 0.0
            data[3, layer, x, 1] = ramp_height(x)
            data[4, layer, x, 1] = ramp_height(x) + 2.0
    payload = {'data': data, 'resolution': 0.2, 'center': np.array([0., 0.]),
               'slice_h0': .5, 'slice_dh': .5, 'selected_source_layers': [0, 6],
               'frame_id': frame, 'source_pcd': str(source) if source else '/tmp/map.pcd',
               'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest() if source else 'test'}
    return payload


def xyz(x):
    return [0.2 * (x - 20), 0., ramp_height(x)]


def config():
    return {
        'schema_version': 1,
        'tomogram_path': '/tmp/tomogram.npz',
        'source_pcd': '/tmp/map.pcd',
        'vendor_root': '/tmp',
        'output_directory': '/tmp/crossfloor_test',
        'frame_id': 'd1max_loc_map',
        'anchors': {
            'start': {'xyz': xyz(1), 'layer_id': 0},
            'entry': {'xyz': xyz(5), 'layer_id': 0},
            'landing': {'xyz': xyz(21), 'layer_id': 1},
            'exit': {'xyz': xyz(35), 'layer_id': 1},
            'goal': {'xyz': xyz(39), 'layer_id': 1},
        },
        'stair_roi': {'min': [xyz(4)[0], -0.1, -0.1],
                      'max': [xyz(36)[0], 0.1, 3.1]},
        'floor_z_ranges': {'lower': [-0.1, 0.1], 'upper': [2.9, 3.1]},
        'planning': {'path_refinement': 'visibility_c2'},
    }


def paths():
    return {
        'lower_floor': (list(map(xyz, range(1, 6))), [0] * 5),
        'stair_lower': (list(map(xyz, range(5, 22))),
                        [0 if x <= 19 else 1 for x in range(5, 22)]),
        'stair_upper': (list(map(xyz, range(21, 36))), [1] * 15),
        'upper_floor': (list(map(xyz, range(35, 40))), [1] * 5),
    }


def route_factory(route_paths):
    def make(_map, leg, _settings):
        class FakeRoute:
            def plan(self, *_args):
                points, layers = route_paths[leg]
                return {'path': points, 'layer_ids': layers, 'algorithm': 'fixture'}
        return FakeRoute()
    return make


def install_joint_refinement(monkeypatch, transform=None):
    """Replace only the optional optimizer; production map checks still run."""
    from types import SimpleNamespace
    import sys
    calls = []

    def refine(grid, values, layers, *, landing_index, **options):
        calls.append((np.array(values), np.array(layers), landing_index))
        points = np.asarray(values).copy()
        requested = points[landing_index].copy()
        points[landing_index, 0] += .02  # same observed cell on fixture platform
        # An inserted sample changes all later global indices.
        points = np.insert(points, landing_index+1,
                           (points[landing_index]+points[landing_index+1])/2., axis=0)
        ids = np.insert(np.asarray(layers), landing_index+1, layers[landing_index])
        result = dict(applied=True, reason='fixture_joint_rounding', path=points.tolist(),
            layer_ids=ids.tolist(), landing_index=landing_index,
            requested_landing_xyz=requested.tolist(),
            effective_landing_xyz=points[landing_index].tolist(), landing_displacement_m=.02,
            curve_validation=STAIR_REFINEMENT_PROOF, curve_partition_points=len(points),
            physical_safety_certified=False, span_records=[],
            stair_profile=dict(direction='up', height_change_m=3.))
        return transform(result) if transform is not None else result

    monkeypatch.setitem(sys.modules, 'd1max_pct_planner.stair_refinement',
                        SimpleNamespace(refine_stair_route=refine))
    return calls


def proven_route_factory(route_paths):
    factory = route_factory(route_paths)
    def make(*args):
        wrapped = factory(*args)
        class Proven:
            def plan(self, *values):
                return {**wrapped.plan(*values),
                    'curve_validation': 'quintic_cell_boundary_roots_and_interval_interiors',
                    'quintic_segments': 2, 'curve_partition_points': 20}
        return Proven()
    return make


def test_stair_route_is_segmented_masked_and_checked_against_original_map():
    original = TomogramMap(scene())
    result = plan_crossfloor(config(), original, route_factory(paths()))
    assert result['status'] == 'validated_offline_route'
    assert result['execution_authorized'] is False
    assert [leg['name'] for leg in result['segments']] == [
        'lower_floor', 'stair_lower', 'stair_upper', 'upper_floor']
    assert result['path_xyz'][0] == xyz(1) and result['path_xyz'][-1] == xyz(39)
    assert result['stair_profile']['height_change_m'] == 3.0
    assert result['layer_transitions']
    assert all(item['leg'].startswith('stair_') for item in result['layer_transitions'])
    assert result['whole_route_check']['checked_cells'] > 0
    assert len(result['edge_legs']) == len(result['path_xyz']) - 1


@pytest.mark.parametrize('at_entry,at_exit', [(True, False), (False, True), (True, True)])
def test_exact_floor_portals_do_not_query_native_or_manufacture_edges(at_entry, at_exit):
    raw = config()
    raw['planning']['path_refinement'] = 'none'
    if at_entry:
        raw['anchors']['start'] = raw['anchors']['entry'].copy()
    if at_exit:
        raw['anchors']['goal'] = raw['anchors']['exit'].copy()
    calls = []
    factory = route_factory(paths())
    def native(grid, leg, settings):
        delegate = factory(grid, leg, settings)
        class Native:
            def plan(self, start, goal, first, last):
                assert not (first == last and np.array_equal(start, goal)), 'zero native query'
                calls.append(leg)
                return delegate.plan(start, goal, first, last)
        return Native()
    result = plan_crossfloor(raw, TomogramMap(scene()), native)
    expected = (['lower_floor'] if not at_entry else []) + ['stair_lower', 'stair_upper']
    expected += ['upper_floor'] if not at_exit else []
    assert calls == expected
    assert [segment['name'] for segment in result['segments']] == expected
    assert result['path_xyz'][0] == raw['anchors']['start']['xyz']
    assert result['path_xyz'][-1] == raw['anchors']['goal']['xyz']
    points = np.asarray(result['path_xyz'])
    assert np.all(np.linalg.norm(np.diff(points, axis=0), axis=1) > 0)
    assert len(result['edge_legs']) == len(points)-1
    assert result['length_m'] == pytest.approx(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())
    cursor = 0
    for segment in result['segments']:
        assert segment['first_index'] == cursor < segment['last_index']
        cursor = segment['last_index']
    assert cursor == len(points)-1


def test_coincident_floor_portal_still_requires_observed_original_support():
    raw = config()
    raw['anchors']['start'] = raw['anchors']['entry'].copy()
    payload = scene()
    payload['data'][0, 0, 5, 1] = 50.
    def forbidden(*args):
        raise AssertionError('invalid portal must be rejected before native queries')
    with pytest.raises(CrossfloorError, match='entry|start') as caught:
        plan_crossfloor(raw, TomogramMap(payload), forbidden)
    assert caught.value.code == 'invalid_anchor'


@pytest.mark.parametrize('name,layer,delta', [
    ('stair_lower', 0, 0.), ('lower_floor', 1, 0.), ('lower_floor', 0, 1e-8),
])
def test_stationary_floor_exception_is_exact_not_a_new_nearby_tolerance(name, layer, delta):
    begin = dict(xyz=np.array([1., 2., 0.]), layer_id=0)
    end = dict(xyz=np.array([1.+delta, 2., 0.]), layer_id=layer)
    assert not _stationary_floor_leg(name, begin, end)


def test_joint_stair_refinement_rebuilds_all_indices_and_retains_requested_portal(monkeypatch):
    calls = install_joint_refinement(monkeypatch)
    result = plan_crossfloor(config(), TomogramMap(scene()), proven_route_factory(paths()))
    assert len(calls) == 1 and len(calls[0][0]) == 31
    assert result['stair_refinement']['applied'] is True
    portal = result['anchors']['landing']
    assert portal['requested_xyz'] == xyz(21)
    assert portal['xyz'] != xyz(21)
    assert portal['displacement_m'] == pytest.approx(.02)
    n = len(result['path_xyz'])
    assert n == 40 and len(result['edge_legs']) == n-1
    cursor = 0
    for segment in result['segments']:
        a, b = segment['first_index'], segment['last_index']
        assert a == cursor
        assert result['path_xyz'][a] == result['anchors'][segment['from']]['xyz']
        assert result['path_xyz'][b] == result['anchors'][segment['to']]['xyz']
        assert result['edge_legs'][a:b] == [segment['name']]*(b-a)
        if segment['name'].startswith('stair_'):
            assert segment['curve_validation'] == STAIR_REFINEMENT_PROOF
            assert segment['quintic_segments'] == 0  # never relabel native proof
            assert segment['native_curve_evidence']['quintic_segments'] == 2
            assert segment['checks']['original']['checked_cells'] > 0
        cursor = b
    assert cursor == n-1
    for transition in result['layer_transitions']:
        i = transition['edge']
        assert result['layer_ids'][i:i+2] == [transition['from_layer'], transition['to_layer']]
        assert result['edge_legs'][i] == transition['leg']
    assert result['source_layer_ids'] == [0 if value == 0 else 6 for value in result['layer_ids']]


@pytest.mark.parametrize('corruption', ['wall', 'wrong_landing_layer', 'old_native_proof', 'moved_entry',
                                       'wrong_landing_metadata', 'wrong_partition_count'])
def test_rejected_joint_candidate_preserves_validated_native_route(monkeypatch, corruption):
    def corrupt(result):
        if corruption == 'wall':
            result['path'][3][1] = 2.
        elif corruption == 'wrong_landing_layer':
            result['layer_ids'][result['landing_index']] = 0
        elif corruption == 'old_native_proof':
            result['curve_validation'] = 'quintic_cell_boundary_roots_and_interval_interiors'
        elif corruption == 'moved_entry':
            result['path'][0][0] += .01
        elif corruption == 'wrong_landing_metadata':
            result['effective_landing_xyz'][0] += 1.
        else:
            result['curve_partition_points'] = 0
        return result
    install_joint_refinement(monkeypatch, corrupt)
    result = plan_crossfloor(config(), TomogramMap(scene()), proven_route_factory(paths()))
    assert result['stair_refinement']['applied'] is False
    assert result['stair_refinement']['reason'] == 'refinement_rejected_keep_native'
    assert result['path_xyz'] == list(map(xyz, range(1, 40)))
    assert result['anchors']['landing'] == config()['anchors']['landing']
    assert all(segment['curve_validation'] == 'quintic_cell_boundary_roots_and_interval_interiors'
               for segment in result['segments'])


def test_disabled_refinement_never_calls_optional_optimizer(monkeypatch):
    calls = install_joint_refinement(monkeypatch)
    raw = config()
    raw['planning']['path_refinement'] = 'none'
    result = plan_crossfloor(raw, TomogramMap(scene()), route_factory(paths()))
    assert calls == []
    assert result['stair_refinement'] == dict(applied=False, reason='path_refinement_disabled')


@pytest.mark.parametrize('leg', ['lower_floor', 'stair_lower', 'stair_upper', 'upper_floor'])
def test_owned_mask_buffer_matches_previous_geometry_and_leaves_source_untouched(leg):
    original = TomogramMap(scene())
    before = original.data.copy()
    settings = validate_config(config())
    if leg.endswith('_floor'):
        lo, hi = settings['floor_z_ranges'][leg.split('_')[0]]
        keep = original.valid & (original.ground >= lo) & (original.ground <= hi)
    else:
        lo, hi = settings['stair_roi']
        ix, iy = np.indices(original.shape)
        x = original.center[0] + (ix - original.offset[0]) * original.resolution
        y = original.center[1] + (iy - original.offset[1]) * original.resolution
        xy = (x >= lo[0]) & (x <= hi[0]) & (y >= lo[1]) & (y <= hi[1])
        keep = original.valid & xy[None] & (original.ground >= lo[2]) & (original.ground <= hi[2])
    expected = before.copy()
    expected[0, ~keep] = 50.
    expected[1:3] = 0.
    expected[1, :, 1:-1, :] = expected[0, :, 2:, :] - expected[0, :, :-2, :]
    expected[2, :, :, 1:-1] = expected[0, :, :, 2:] - expected[0, :, :, :-2]
    actual = _masked_tomogram(original, leg, settings)
    np.testing.assert_array_equal(actual.data, expected)
    np.testing.assert_array_equal(actual.valid, keep)
    np.testing.assert_array_equal(original.data, before)
    assert not np.shares_memory(actual.data, original.data)
    assert actual.valid is actual.allowed and actual.allowed is actual.traversable
    np.testing.assert_array_equal(actual.source_layers, original.source_layers)
    np.testing.assert_array_equal(actual.center, original.center)
    assert actual.provenance == original.provenance
    assert actual.provenance is not original.provenance


@pytest.mark.parametrize('leg', ['stair_lower', 'stair_upper'])
def test_stair_resources_do_not_enable_corridor_quality_tradeoff(monkeypatch, leg):
    from d1max_pct_planner import tomogram_route
    calls = []
    monkeypatch.setattr(tomogram_route, 'TomogramRoute', lambda *args, **kwargs: calls.append(kwargs))
    _native_route(TomogramMap(scene()), leg, validate_config(config()))
    assert calls[0]['path_refinement'] == 'none'


def test_mask_rejects_unknown_resource_instead_of_treating_it_as_stairs():
    with pytest.raises(ValueError, match='Unknown fixed'):
        _masked_tomogram(TomogramMap(scene()), 'wrong_leg', validate_config(config()))


def test_optimizer_spacing_is_optional_bounded_and_only_in_planning():
    assert validate_config(config())['planning']['optimizer_sample_interval'] == 10
    for value in [1, 3, 100]:
        settings = config()
        settings['planning']['optimizer_sample_interval'] = value
        assert validate_config(settings)['planning']['optimizer_sample_interval'] == value
    for value in [0, -1, 101, 3.0, True, '3']:
        settings = config()
        settings['planning']['optimizer_sample_interval'] = value
        with pytest.raises(CrossfloorError, match='optimizer_sample_interval'):
            validate_config(settings)
    settings = config()
    settings['optimizer_sample_interval'] = 3
    with pytest.raises(CrossfloorError, match='configuration keys'):
        validate_config(settings)


def test_stair_is_not_allowed_to_leave_configured_roi():
    route_paths = paths()
    points, layers = route_paths['stair_lower']
    route_paths['stair_lower'] = (points[:2] + [xyz(3)] + points[2:],
                                  layers[:2] + [0] + layers[2:])
    with pytest.raises(CrossfloorError) as error:
        plan_crossfloor(config(), TomogramMap(scene()), route_factory(route_paths))
    assert error.value.code in ('segment_geometry_rejected', 'stair_outside_roi')


def test_vertical_shortcut_through_floor_slab_is_rejected():
    route_paths = paths()
    lifted = [xyz(5)[0], 0., xyz(21)[2]]
    route_paths['stair_lower'] = ([xyz(5), lifted, xyz(21)], [0, 0, 1])
    with pytest.raises(CrossfloorError) as error:
        plan_crossfloor(config(), TomogramMap(scene()), route_factory(route_paths))
    assert error.value.code in ('segment_geometry_rejected', 'stair_vertical_jump')


def test_portal_floor_and_safety_limits_are_not_guessed_or_relaxed():
    invalid = config()
    invalid['anchors']['exit']['xyz'][2] = 0.0
    with pytest.raises(CrossfloorError) as error:
        plan_crossfloor(invalid, TomogramMap(scene()), route_factory(paths()))
    assert error.value.code == 'invalid_anchor'
    invalid = config()
    invalid['limits'] = {'max_vertical_grade': 1.5}
    with pytest.raises(CrossfloorError) as error:
        validate_config(invalid)
    assert error.value.code == 'invalid_config'


def test_a_single_layer_ramp_does_not_masquerade_as_multilevel_gateway():
    source = scene()
    source['data'] = source['data'][:, :1].copy()
    source['selected_source_layers'] = [0]
    for x in range(24, 41):
        source['data'][0, 0, x, 1] = 0.0
        source['data'][3, 0, x, 1] = ramp_height(x)
        source['data'][4, 0, x, 1] = ramp_height(x) + 2.0
    settings = config()
    for anchor in settings['anchors'].values():
        anchor['layer_id'] = 0
    route_paths = {name: (points, [0] * len(points))
                   for name, (points, _layers) in paths().items()}
    with pytest.raises(CrossfloorError) as error:
        plan_crossfloor(settings, TomogramMap(source), route_factory(route_paths))
    assert error.value.code == 'no_stair_gateway'


def test_cli_artifacts_include_hashes_segments_layers_and_failure_audit(tmp_path):
    source = tmp_path / 'source.pcd'
    source.write_bytes(b'fixture source bytes')
    payload = scene(source)
    tomogram = tmp_path / 'tomogram.npz'
    np.savez_compressed(tomogram, **payload)
    settings = config()
    settings.update(tomogram_path=str(tomogram), source_pcd=str(source),
                    vendor_root=str(tmp_path), output_directory=str(tmp_path / 'route'))
    config_path = tmp_path / 'route.yaml'
    config_path.write_text(yaml.safe_dump(settings))
    output, result = run_from_config(config_path, route_factory(paths()))
    audit = json.loads((output / 'audit.json').read_text())
    assert audit['status'] == 'validated_offline_route'
    assert audit['tomogram_sha256'] == hashlib.sha256(tomogram.read_bytes()).hexdigest()
    assert audit['source_pcd_sha256'] == hashlib.sha256(source.read_bytes()).hexdigest()
    assert audit['segments'][-1]['last_index'] == len(audit['path_xyz']) - 1
    assert len((output / 'path.csv').read_text().splitlines()) == len(result['path_xyz']) + 1
    assert audit['execution_authorized'] is False
    failure_paths = paths()
    failure_paths['stair_lower'] = ([xyz(5), [xyz(5)[0], 0., xyz(21)[2]], xyz(21)], [0, 0, 1])
    with pytest.raises(CrossfloorError):
        run_from_config(config_path, route_factory(failure_paths))
    failure = json.loads((tmp_path / 'route_001' / 'audit.json').read_text())
    assert failure['status'] == 'failed' and failure['execution_authorized'] is False
    assert not (tmp_path / 'route_001' / 'path.csv').exists()


def test_cli_bootstraps_verified_native_runtime_in_a_child(tmp_path, monkeypatch):
    from d1max_pct_planner import crossfloor_route, native_runtime
    config_path = tmp_path / 'route.yaml'
    config_path.write_text(yaml.safe_dump(config()))
    monkeypatch.delenv('D1MAX_PCT_CROSSFLOOR_CHILD', raising=False)
    monkeypatch.setattr(native_runtime, 'prepare_native_environment',
                        lambda root: {'ROOT_FOR_TEST': root})
    seen = {}

    def fake_run(command, *, env, check):
        seen.update(command=command, env=env, check=check)
        return type('Result', (), {'returncode': 7})()

    monkeypatch.setattr(crossfloor_route.subprocess, 'run', fake_run)
    assert main(['--config', str(config_path)]) == 7
    assert seen['command'][1:3] == ['-m', 'd1max_pct_planner.crossfloor_route']
    assert seen['env']['D1MAX_PCT_CROSSFLOOR_CHILD'] == '1'
    assert seen['check'] is False
