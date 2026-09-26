import numpy as np
import pytest

from d1max_pct_planner.tomogram_map import TomogramError, TomogramMap
from d1max_pct_planner.tomogram_route import curve_partition, expand_native_curve, quintic_xy, real_unit_roots


def payload(layers=2, size=9, ceiling=True):
    data = np.zeros((5, layers, size, size), dtype=np.float32)
    for layer in range(layers):
        data[3, layer] = layer * 3.0
        data[4, layer] = layer * 3.0 + 2.0 if ceiling else np.nan
    return {'data': data, 'resolution': 0.2, 'center': np.array([12.3, -4.2]),
            'slice_h0': 0.5, 'slice_dh': 0.5,
            'selected_source_layers': np.arange(layers) * 6, 'minimum_headroom_m': 0.55}


def test_safe_npz_round_trip_and_native_centre_convention(tmp_path):
    source = payload()
    path = tmp_path / 'tomogram.npz'
    np.savez_compressed(path, **source)
    tomo = TomogramMap(path)
    assert tomo.sha256 and tomo.source == str(path)
    for x in range(tomo.nx):
        for y in range(tomo.ny):
            assert np.array_equal(tomo.index(tomo.world([x, y])), [x, y])
    assert np.array_equal(tomo.native_payload()['data'], source['data'])
    assert tomo.available_layers()[1]['source_layer'] == 6


def test_native_rint_half_tie_and_supercover_are_both_preserved():
    tomo = TomogramMap(payload(size=10))  # odd offset reveals banker's-rounding mistakes
    point = tomo.center + [0.1, 0.0]
    expected = np.rint((point - tomo.center) / tomo.resolution).astype(int) + tomo.offset
    assert np.array_equal(tomo.index(point), expected)
    assert tomo.point_cells(point) == {(5, 5), (6, 5)}


def test_scalar_supercover_matches_vector_convention_random_and_boundary_points():
    tomo = TomogramMap(payload(size=10))
    rng = np.random.default_rng(4827)
    relative = rng.uniform(-4, 4, (2000, 2))
    boundaries = np.array([[i+.5+e, j+.5-e] for i in range(-4, 4)
        for j in range(-4, 4) for e in (0., 1e-9, -1e-9, 2e-8, -2e-8)])
    for point in tomo.center+np.vstack((relative, boundaries))*tomo.resolution:
        index = np.rint((point-tomo.center)/tomo.resolution).astype(int)+tomo.offset
        continuous = (point-tomo.center)/tomo.resolution+tomo.offset
        choices = []
        for axis, value in enumerate(continuous):
            lower = np.floor(value)
            choices.append([int(lower), int(lower+1)] if abs(value-lower-.5)<1e-8
                           else [int(index[axis])])
        expected = {(x,y) for x in choices[0] for y in choices[1]}
        assert tomo.point_cells(point) == expected


@pytest.mark.parametrize('dtype',[np.float32,np.float64])
def test_fast_segment_samples_match_original_vector_interpolation(dtype):
    tomo = TomogramMap(payload(size=10))
    rng = np.random.default_rng(661)
    for _ in range(100):
        a,b = tomo.center+rng.uniform(-.75,.75,(2,2))
        a,b = a.astype(dtype),b.astype(dtype)
        for time,cells in tomo.segment_samples(a,b):
            assert cells == tomo.point_cells(np.asarray(a)+time*(np.asarray(b)-a))


def test_native_payload_optimization_keeps_input_and_hard_costs_unchanged():
    source = payload()
    source['data'][0,0,4,4] = 80
    source['data'][0,0,3,4] = np.nan
    source['data'][1,0,4,4] = np.nan
    tomo = TomogramMap(source)
    before = tomo.data.copy()
    expected = before.copy()
    expected[0,~tomo.valid] = np.maximum(np.nan_to_num(expected[0,~tomo.valid],nan=50.),50.)
    expected[1:3] = np.nan_to_num(expected[1:3],nan=0.)
    actual = tomo.native_payload()['data']
    np.testing.assert_array_equal(actual,expected)
    np.testing.assert_array_equal(tomo.data,before)


def test_same_xy_layers_are_chosen_by_real_height_not_zero_or_lowest_cost():
    source = payload()
    source['data'][0, 1] = 19
    tomo = TomogramMap(source)
    x, y = tomo.center
    assert tomo.select([x, y, 3.02])['layer_id'] == 1
    assert tomo.select([x, y, 0.02])['layer_id'] == 0
    assert tomo.select([x, y, 3.02])['xyz'] == [x, y, 3.02]
    with pytest.raises(TomogramError, match='No supported surface'):
        tomo.select([x, y, 1.5])


def test_ground_follow_changes_only_z_and_explicit_layer_can_change_floor():
    tomo = TomogramMap(payload())
    xyz = [*tomo.center + [0.03, 0.02], 0.04]
    selected = tomo.select(xyz, mode='ground_follow', preferred_layer=0)
    assert selected['xyz'] == [*xyz[:2], 0.0]
    upper = tomo.select(xyz, mode='ground_follow', preferred_layer=0, layer_lock=1)
    assert upper['xyz'] == [*xyz[:2], 3.0]
    with pytest.raises(TomogramError):
        tomo.select(xyz, mode='free_xyz', layer_lock=1)


def test_no_support_never_projects_xy_or_silently_jumps_to_roof():
    source = payload()
    source['data'][3, 0, 4, 4] = np.nan
    tomo = TomogramMap(source)
    with pytest.raises(TomogramError):
        tomo.select([*tomo.center, 0.0], mode='ground_follow', preferred_layer=0)
    assert len(tomo.sample_surfaces(tomo.center)) == 1
    assert tomo.sample_surfaces(tomo.center)[0]['ground_z'] == 3.0


def test_missing_previous_slice_can_follow_same_physical_surface_only():
    source = payload()
    source['data'][3, 0, 4, 4] = np.nan
    source['data'][3, 1] = 0.10
    tomo = TomogramMap(source)
    result = tomo.select([*tomo.center, 0.0], mode='ground_follow', preferred_layer=0)
    assert result['layer_id'] == 1
    source['data'][3, 1] = 0.29
    with pytest.raises(TomogramError, match='Previous surface ended'):
        TomogramMap(source).select([*tomo.center, 0.0], mode='ground_follow', preferred_layer=0)


def test_unknown_ceiling_is_never_fabricated_or_called_open_sky():
    tomo = TomogramMap(payload(ceiling=False))
    info = tomo.select([*tomo.center, 0.0])
    assert info['ceiling_z'] is None and info['headroom_m'] is None
    assert info['ceiling_state'] == 'unobserved_above'
    assert np.isnan(tomo.native_payload()['data'][4]).all()
    strict = TomogramMap(payload(ceiling=False), unknown_ceiling_policy='reject')
    assert not strict.valid.any()
    with pytest.raises(TomogramError):
        strict.validate_endpoint([*strict.center, 0.0])


def test_explicit_verified_open_sky_is_distinct_from_missing_ceiling():
    source = payload(ceiling=False)
    source['open_sky_verified'] = np.ones_like(source['data'][3], dtype=bool)
    tomo = TomogramMap(source, unknown_ceiling_policy='reject')
    assert tomo.select([*tomo.center, 0.0])['ceiling_state'] == 'verified_open_sky'


def test_low_ceiling_is_blocked_for_display_selection_and_native_map():
    source = payload()
    source['data'][4, 0, 4, 4] = 0.4
    tomo = TomogramMap(source)
    assert not tomo.valid[0, 4, 4]
    assert tomo.native_payload()['data'][0, 0, 4, 4] > 20
    with pytest.raises(TomogramError, match='not traversable') as error:
        tomo.validate_endpoint([*tomo.center, 0], layer_id=0)
    assert error.value.code == 'insufficient_headroom'
    display = tomo.display_points(layer_id=0)
    assert not np.any(np.all(np.isclose(display['xyz'][:, :2], tomo.center), axis=1))


def test_inclusive_official_threshold_and_no_extra_erosion():
    source = payload()
    source['data'][0, 0, 4, 4] = 20
    source['data'][0, 0, 5, 4] = 20.001
    tomo = TomogramMap(source)
    assert tomo.valid[0, 4, 4] and not tomo.valid[0, 5, 4]
    assert tomo.valid[0, 3, 4]


def test_duplicate_surfaces_are_only_deduplicated_for_display_and_selection():
    source = payload()
    source['data'][3, 1] = 0
    tomo = TomogramMap(source)
    assert len(tomo.sample_surfaces(tomo.center)) == 1
    assert tomo.sample_surfaces(tomo.center)[0]['equivalent_layers'] == [0, 1]
    assert len(tomo.sample_surfaces(tomo.center, deduplicate=False)) == 2
    assert tomo.native_payload()['data'].shape[1] == 2
    assert len(tomo.display_points()['xyz']) == 81
    assert len(tomo.display_points(deduplicate=False)['xyz']) == 162


def test_boundary_endpoint_checks_both_cells():
    source = payload()
    source['data'][0, 0, 5, 4] = 50
    tomo = TomogramMap(source)
    with pytest.raises(TomogramError, match='cell boundary'):
        tomo.validate_endpoint([*(tomo.center + [0.1, 0]), 0], 0)


def test_supercover_checks_whole_segment_not_just_endpoints():
    source = payload()
    source['data'][0, 0, 4, 4] = 50
    tomo = TomogramMap(source)
    path = [[*tomo.world([2, 4]), 0], [*tomo.world([6, 4]), 0]]
    with pytest.raises(TomogramError, match='Optimized curve'):
        tomo.validate_path(path, [0, 0])


def test_boundary_line_does_not_sneak_past_blocked_neighbor():
    source = payload()
    source['data'][0, 0, 5, 4] = 50
    tomo = TomogramMap(source)
    path = [[*(tomo.center + [.1, -.4]), 0], [*(tomo.center + [.1, .4]), 0]]
    with pytest.raises(TomogramError):
        tomo.validate_path(path, [0, 0])


def test_layer_transition_rejects_vertical_jump_accepts_same_surface():
    source = payload()
    tomo = TomogramMap(source)
    path = [[*tomo.world([2, 4]), 0], [*tomo.world([6, 4]), 3]]
    with pytest.raises(TomogramError):
        tomo.validate_path(path, [0, 1])
    source['data'][3, 1] = 0
    tomo = TomogramMap(source)
    path[-1][2] = 0
    report = tomo.validate_path(path, [0, 1])
    assert report['layer_transitions'] == 1 and report['boundary_samples'] > 0


def test_floating_path_ground_step_and_unknown_rejected():
    source = payload()
    tomo = TomogramMap(source)
    path = [[*tomo.world([2, 4]), .6], [*tomo.world([6, 4]), .6]]
    with pytest.raises(TomogramError):
        tomo.validate_path(path, [0, 0])
    source['data'][3, 0, 4, 4] = .3
    tomo = TomogramMap(source)
    with pytest.raises(TomogramError):
        tomo.validate_path([[*tomo.world([2, 4]), 0], [*tomo.world([6, 4]), 0]], [0, 0])


def test_free_endpoint_height_error_is_kept_within_tolerance():
    tomo = TomogramMap(payload())
    path = [[*tomo.world([2, 4]), .04], [*tomo.world([6, 4]), -.04]]
    assert tomo.validate_endpoint(path[0], 0)['xyz'] == path[0]
    assert tomo.validate_path(path, [0, 0])['extra_erosion_cells'] == 0


def test_initial_seed_requires_correct_height_and_does_not_change_selection_contract():
    tomo = TomogramMap(payload())
    seed = tomo.initial_seed([*tomo.center, 3.01])
    assert seed['layer_id'] == 1 and seed['ground_z'] == 3
    with pytest.raises(TomogramError):
        tomo.initial_seed([*tomo.center, 1.5])


def test_quintic_reconstruction_preserves_endpoint_and_derivatives():
    a = np.array([1, 2, 3, 4, 5, 6.])
    b = np.array([7, 8, 9, 10, 11, 12.])
    poly = quintic_xy(a, b, .5, .2, [0, 0], [0, 0])
    assert np.allclose(poly[0], a[[3, 0]] * .2)
    assert np.allclose(np.polynomial.polynomial.polyval(1, poly), b[[3, 0]] * .2)
    assert np.allclose(np.polynomial.polynomial.polyval(1, np.polynomial.polynomial.polyder(poly)),
                       b[[4, 1]] * .2 * .5)


def test_actual_quintic_overshoot_is_checked_even_when_endpoint_chord_is_safe():
    source = payload(layers=1)
    source['data'][0, 0, 4, 5] = 50
    tomo = TomogramMap(source)
    # World X stays centred; world Y bows into a blocked neighboring cell.
    # Both endpoints are at the free centre cell, so chord-only checks miss it.
    states = np.array([[4, 8, 0, 4, 0, 0], [4, -8, 0, 4, 0, 0.]])
    native = {'path': np.array([[*tomo.center, 0], [*tomo.center, 0]]),
              'layer_ids': [0, 0], 'native_states': states, 'native_sample_dt': 1.0}
    with pytest.raises(TomogramError):
        expand_native_curve(native, tomo)


def test_constant_polynomial_has_no_roots_and_partition_is_bounded():
    assert real_unit_roots(np.zeros(6)) == []
    tomo = TomogramMap(payload())
    polynomial = np.zeros((6, 2))
    polynomial[0] = tomo.center
    assert curve_partition(polynomial, tomo) == [0., .25, .5, .75, 1.]


def test_reject_malformed_layers_and_unsafe_archive_pickle(tmp_path):
    source = payload()
    source['selected_source_layers'] = [1, 1]
    with pytest.raises(TomogramError):
        TomogramMap(source)
    np.savez(tmp_path / 'unsafe.npz', data=np.array([object()], dtype=object))
    with pytest.raises(ValueError):
        TomogramMap(tmp_path / 'unsafe.npz')


def test_outside_curves_fail_before_unbounded_cell_enumeration():
    tomo = TomogramMap(payload())
    with pytest.raises(TomogramError):
        tomo.validate_path([[*tomo.center, 0], [1e12, 1e12, 0]], [0, 0])
    polynomial = np.zeros((6, 2))
    polynomial[0] = tomo.center
    polynomial[1, 0] = 1e12
    with pytest.raises(TomogramError, match='leaves the measured'):
        curve_partition(polynomial, tomo)
