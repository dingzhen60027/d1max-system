import numpy as np
import pytest
import hashlib

from d1max_pct_planner.tomogram_map import TomogramMap
from d1max_pct_planner.tomogram_selection import TomogramSelection
from d1max_pct_planner.tomogram_display import surface_clouds


def fixture_map():
    data = np.zeros((5, 2, 15, 15), dtype=np.float32)
    data[3, 1] = 3
    data[4] = data[3] + 2
    return TomogramMap({'data': data, 'resolution': .2, 'center': [0., 0.],
                        'slice_h0': .5, 'slice_dh': .5,
                        'selected_source_layers': [0, 6]})


def test_ground_follow_ignores_incoming_camera_z_and_preserves_xy_orientation():
    selection = TomogramSelection(fixture_map(), anchor_z=0)
    selection.activate('start', [0, 0])
    selection.set_orientation('start', [0, 0, 1, 1])
    quat = selection.orientations['start'].copy()
    selection.set_point('start', [.24, .34, 100.])
    np.testing.assert_array_equal(selection.validate('start'), [.24, .34, 0])
    np.testing.assert_array_equal(selection.orientations['start'], quat)
    assert selection.layers['start'] == 0


def test_free_high_z_stays_visible_invalid_and_explicit_snap_recovers():
    selection = TomogramSelection(fixture_map(), anchor_z=0)
    selection.activate('start', [0, 0])
    selection.set_mode('free')
    selection.set_point('start', [.2, .4, .5])
    assert selection.validation('start')['reason_code'] == 'height_off_surface'
    np.testing.assert_array_equal(selection.points['start'], [.2, .4, .5])
    selection.set_mode('ground')
    assert not selection.validation('start')['valid']
    assert selection.snap_ground('start')
    np.testing.assert_array_equal(selection.validate('start'), [.2, .4, 0])


def test_layer_and_mode_selection_never_teleport_or_reset_existing_points():
    selection = TomogramSelection(fixture_map(), anchor_z=0)
    selection.activate('start', [0, 0])
    before, generation = selection.points['start'].copy(), selection.revision
    selection.set_mode('free')
    selection.set_active_layer(1)
    assert not selection.activate('start', [1, 1])
    np.testing.assert_array_equal(selection.points['start'], before)
    assert selection.revision == generation
    selection.set_mode('ground')
    selection.set_point('start', [.2, .4, 0])
    np.testing.assert_array_equal(selection.validate('start'), [.2, .4, 3])
    assert selection.layers['start'] == 1


def test_invalid_xy_does_not_snap_and_blocked_reason_is_specific():
    tomo = fixture_map()
    # Reconstruct so all authoritative masks follow this true cost.
    data = tomo.data.copy()
    data[0, 0, 8, 7] = 50
    source = tomo.native_payload()
    source['data'] = data
    selection = TomogramSelection(TomogramMap(source), anchor_z=0)
    selection.activate('start', [0, 0])
    selection.set_point('start', [.2, 0., 0.])
    np.testing.assert_array_equal(selection.points['start'], [.2, 0, 0])
    assert selection.validation('start')['reason_code'] == 'pct_cost_blocked'
    selection.set_point('start', [50., 40., 0.])
    assert selection.validation('start')['reason_code'] == 'outside_map'
    np.testing.assert_array_equal(selection.points['start'], [50., 40., 0.])


def test_same_xyz_new_layer_changes_generation_and_clear_removes_errors():
    source = fixture_map().native_payload()
    source['data'][3, 1] = 0
    selection = TomogramSelection(TomogramMap(source), anchor_z=0)
    selection.activate('start', [0, 0])
    revision = selection.revision
    selection.set_active_layer(1)
    assert selection.set_point('start', [0, 0, 0])
    assert selection.revision == revision + 1 and selection.layers['start'] == 1
    selection.set_point('start', [99, 99, 0])
    selection.clear()
    assert selection.layers == {'start': None, 'goal': None}
    assert selection.edit_errors == {'start': None, 'goal': None}
    assert selection.snapshot() == {'start_xyz': None, 'goal_xyz': None}


def test_request_and_result_keep_all_three_endpoint_coordinates_and_layers():
    selection = TomogramSelection(fixture_map(), mode='free', anchor_z=0)
    selection.set_point('start', [-.4, 0, .025])
    selection.set_point('goal', [.4, 0, -.025])
    request = selection.request()
    assert request['start_layer'] == request['goal_layer'] == 0
    result = {'generation': selection.revision, 'path': [request['start_xyz'], request['goal_xyz']],
              'layer_ids': [0, 0]}
    assert selection.checked_result(result, request).shape == (2, 3)
    result['path'][-1][-1] = 0
    with pytest.raises(ValueError, match='endpoint XYZ'):
        selection.checked_result(result, request)
    result['generation'] -= 1
    assert selection.checked_result(result, request) is None


def test_visualization_deduplicates_layers_without_changing_any_masks():
    source = fixture_map().native_payload()
    source['data'][3, 1] = 0
    source['data'][0, 1, 7, 7] = 50
    tomo = TomogramMap(source)
    masks = tomo.allowed.copy()
    xyz, costs, blocked = surface_clouds(tomo, [0, 1])
    assert len(xyz) == 225 and len(costs) == 225 and len(blocked) == 0
    np.testing.assert_array_equal(tomo.allowed, masks)
    _, _, red_only = surface_clouds(tomo, [1])
    np.testing.assert_array_equal(red_only, [[0, 0, 0]])
    assert surface_clouds(tomo, [])[0].shape == (0, 3)


@pytest.mark.parametrize('bad', [True, 2, -2, .5, '0'])
def test_layer_lock_rejects_invalid_ids(bad):
    with pytest.raises(ValueError):
        TomogramSelection(fixture_map()).set_active_layer(bad)


def test_source_identity_and_coordinate_frame_are_verified_before_overlay(tmp_path):
    source = fixture_map().native_payload()
    pcd = tmp_path / 'source.pcd'
    pcd.write_bytes(b'original point cloud fixture')
    source.update(source_sha256=hashlib.sha256(pcd.read_bytes()).hexdigest(), frame_id='map')
    tomo = TomogramMap(source)
    assert tomo.verify_source(pcd, 'map')['frame_id'] == 'map'
    with pytest.raises(ValueError, match='frame differs'):
        tomo.verify_source(pcd, 'odom')
    pcd.write_bytes(b'a different map')
    with pytest.raises(ValueError, match='not the tomogram source'):
        tomo.verify_source(pcd, 'map')
    with pytest.raises(ValueError, match='Rebuild tomography'):
        fixture_map().verify_source(pcd, 'map')
