import copy
import json

import numpy as np
import pytest

from d1max_pct_planner.route_restore import FULL_CURVE_VALIDATION, RESTORED_ORIGIN, restore_verified_route
from d1max_pct_planner.tomogram_map import TomogramError, TomogramMap


def fixture(tmp_path, mutate=None):
    data = np.zeros((5, 1, 9, 9), dtype=np.float32)
    data[0] = 10
    data[4] = 2
    payload = dict(data=data, resolution=.2, center=np.array([0., 0.]),
                   slice_h0=.5, slice_dh=.5, selected_source_layers=np.array([8]),
                   minimum_headroom_m=.55, source_pcd='/immutable.pcd',
                   source_sha256='abc', frame_id='map')
    old_path = tmp_path / 'old.npz'
    np.savez_compressed(old_path, **payload)
    old = TomogramMap(old_path)
    current = copy.deepcopy(payload)
    current['data'][0] = 5
    if mutate:
        mutate(current)
    new_path = tmp_path / 'new.npz'
    np.savez_compressed(new_path, **current)
    new = TomogramMap(new_path)
    route = dict(path=[[-.4, 0., 0.], [.4, 0., 0.]], layer_ids=[0, 0],
                 start_xyz=[-.4, 0., 0.], goal_xyz=[.4, 0., 0.], start_layer=0, goal_layer=0,
                 source_tomogram_sha256=old.sha256, curve_validation=FULL_CURVE_VALIDATION,
                 quintic_segments=1, extra_erosion_cells=0, cost_threshold=20.,
                 elapsed_s=.123, length_m=.8, kind='planned', generation=17)
    return new, old_path, route


def test_preserves_exact_route_and_transparent_provenance(tmp_path):
    current, old_path, route = fixture(tmp_path)
    original = copy.deepcopy(route)
    result = restore_verified_route(current, old_path, route)
    assert route == original
    assert result['path'] == route['path'] and result['layer_ids'] == route['layer_ids']
    assert result['source_tomogram_sha256'] == current.sha256
    assert result['route_origin'] == RESTORED_ORIGIN
    assert result['elapsed_s'] == 0 and result['previous_native_elapsed_s'] == .123
    assert result['revalidation']['native_replanning_executed'] is False
    assert result['revalidation']['maximum_new_minus_old_cost'] == -5
    assert 'kind' not in result and 'generation' not in result


def test_file_input_and_identical_cost_are_accepted(tmp_path):
    current, old_path, route = fixture(tmp_path, lambda p: p['data'][0].fill(10))
    route_path = tmp_path / 'route.json'
    route_path.write_text(json.dumps(route))
    assert restore_verified_route(current, old_path, route_path)['revalidation']['maximum_new_minus_old_cost'] == 0


@pytest.mark.parametrize('field,value', [('source_sha256','different'), ('source_pcd','/other.pcd'), ('frame_id','other')])
def test_source_or_frame_changes_rejected(tmp_path, field, value):
    current, old_path, route = fixture(tmp_path, lambda p: p.update({field: value}))
    with pytest.raises(TomogramError) as error:
        restore_verified_route(current, old_path, route)
    assert error.value.code == 'restore_source_mismatch'


@pytest.mark.parametrize('mutation', [
    lambda p: p.update(center=np.array([.001, 0.])),
    lambda p: p.update(resolution=.21),
    lambda p: p.update(slice_h0=.6),
    lambda p: p.update(slice_dh=.6),
    lambda p: p.update(selected_source_layers=np.array([7])),
    lambda p: p['data'].__setitem__((3,0,1,1), .001),
    lambda p: p['data'].__setitem__((4,0,1,1), 2.001),
    lambda p: p.update(open_sky_verified=np.ones((1,9,9),dtype=bool)),
])
def test_any_geometry_or_evidence_difference_rejected(tmp_path, mutation):
    current, old_path, route = fixture(tmp_path, mutation)
    with pytest.raises(TomogramError) as error:
        restore_verified_route(current, old_path, route)
    assert error.value.code == 'restore_geometry_mismatch'


@pytest.mark.parametrize('value,code', [(10.001,'restore_cost_increased'), (np.nan,'restore_cost_mask_changed'), (-1,'restore_stricter_validity')])
def test_cost_change_rejected_even_away_from_saved_route(tmp_path, value, code):
    current, old_path, route = fixture(tmp_path, lambda p: p['data'].__setitem__((0,0,0,0), value))
    with pytest.raises(TomogramError) as error:
        restore_verified_route(current, old_path, route)
    assert error.value.code == code


@pytest.mark.parametrize('mutation,code', [
    (lambda r:r.update(source_tomogram_sha256='wrong'), 'restore_map_hash_mismatch'),
    (lambda r:r.update(curve_validation='sampled_only'), 'restore_unverified_curve'),
    (lambda r:r.update(quintic_segments=0), 'restore_unverified_curve'),
    (lambda r:r.update(extra_erosion_cells=1), 'restore_unverified_curve'),
    (lambda r:r.update(start_xyz=[-.3,0,0]), 'restore_endpoint_mismatch'),
    (lambda r:r.update(goal_layer=1), 'restore_endpoint_mismatch'),
    (lambda r:r.update(path=[[np.nan,0,0],[.4,0,0]]), 'restore_invalid_route'),
])
def test_invalid_provenance_or_endpoints_rejected(tmp_path, mutation, code):
    current, old_path, route = fixture(tmp_path)
    mutation(route)
    with pytest.raises(TomogramError) as error:
        restore_verified_route(current, old_path, route)
    assert error.value.code == code


def test_rechecks_prior_path_not_only_its_success_label(tmp_path):
    current, old_path, route = fixture(tmp_path)
    route['path'] = [[-.4,0,0],[.0,0,.7],[.4,0,0]]
    route['layer_ids'] = [0,0,0]
    with pytest.raises(TomogramError):
        restore_verified_route(current, old_path, route)


def test_memory_only_current_map_rejected(tmp_path):
    current, old_path, route = fixture(tmp_path)
    current.sha256 = None
    with pytest.raises(TomogramError) as error:
        restore_verified_route(current, old_path, route)
    assert error.value.code == 'restore_missing_map_hash'
