import hashlib
import json

import numpy as np
import pytest
import yaml

from d1max_pct_planner.crossfloor_route import (
    CrossfloorError, _masked_tomogram, main, plan_crossfloor, run_from_config, validate_config,
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
