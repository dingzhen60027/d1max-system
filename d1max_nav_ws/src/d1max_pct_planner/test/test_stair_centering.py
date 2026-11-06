"""Offline measured-side fitting; current map and original support remain final."""
from dataclasses import replace

import numpy as np
import pytest

from d1max_pct_planner.stair_centering import (
    CenteringPolicy, _center_transition, _centered_lateral, _minimum_structure_distance,
    center_measured_stair_path, extract_measured_flight_structure, fit_measured_sides,
)
from d1max_pct_planner.tomogram_map import TomogramMap


def flight():
    s, z = np.meshgrid(np.linspace(-2, 2, 161), np.linspace(.3, 1.2, 12), indexing='ij')
    sides = np.concatenate([np.c_[s.ravel(), np.full(s.size, sign * .8), z.ravel()]
                            for sign in (-1, 1)])
    return dict(id=1, leg='stair_lower', kind='flight_support_envelope', center_xy=[0., 0.],
        uphill_axis_xy=[1., 0.], grade=0., intercept_m=0., along_range_m=[-2., 2.],
        structure_xyz=sides)


def example(monkeypatch):
    # A native GridMap/Tomogram collision check is used below. This focused
    # fixture has a single flat slice, so gateway/profile ownership is stubbed;
    # whole real stairs + source checks are in the sealed production experiment.
    import d1max_pct_planner.crossfloor_route as route
    monkeypatch.setattr(route, '_masked_tomogram', lambda grid, *args: grid)
    monkeypatch.setattr(route, '_check_joined', lambda *args: {'fixture': 'one-flat-slice-only'})
    data = np.zeros((5, 1, 121, 61), np.float32)
    data[4] = 2.
    grid = TomogramMap(dict(data=data, resolution=.05, center=[0., 0.], slice_h0=.1, slice_dh=.5))
    x = np.linspace(-2.4, 2.4, 161)
    path = np.c_[x, .14 * np.sin(2.8 * x) + .06, np.zeros(len(x))]
    layers = np.zeros(len(path), dtype=int)
    owner = ['stair_lower'] * (len(path) - 1)
    settings = dict(limits=dict(max_ground_step_m=.17))
    return grid, path, layers, owner, settings


def test_fit_measured_bilateral_sides_and_preserve_cloud_indices():
    model = flight()
    cloud = model['structure_xyz'].copy()
    extracted = extract_measured_flight_structure(cloud, [model])
    assert len(extracted) == 1
    actual = extracted[0]
    assert np.array_equal(actual['structure_xyz'], cloud[actual['structure_source_indices']])
    fit = fit_measured_sides(actual)
    assert np.allclose(fit['center_slope_intercept'], [0., 0.], atol=1e-12)
    assert np.allclose(fit['width_at_endpoints_m'], 1.6)
    assert np.array_equal(cloud, model['structure_xyz'])


def test_no_one_sided_fallback_or_pct_contour_substitute():
    model = flight()
    model['structure_xyz'] = model['structure_xyz'][model['structure_xyz'][:, 1] > 0]
    with pytest.raises(ValueError, match='side_missing:negative'):
        fit_measured_sides(model)


def test_sparse_or_short_wall_coverage_cannot_define_whole_flight_middle():
    model = flight()
    model['structure_xyz'] = model['structure_xyz'][model['structure_xyz'][:, 0] < -.5]
    with pytest.raises(ValueError, match='side_coverage'):
        fit_measured_sides(model)


def test_floor_returns_are_not_vertical_wall_evidence():
    x, y = np.meshgrid(np.linspace(-2, 2, 81), np.linspace(-1, 1, 41))
    model = flight()
    with pytest.raises(ValueError, match='measured_structure'):
        extract_measured_flight_structure(np.c_[x.ravel(), y.ravel(), np.full(x.size, .5)], [model])


def test_structure_distance_checks_chord_not_only_vertices():
    path = np.array([[-1., 0., 0.], [1., 0., 1.]])
    structure = np.array([[0., .25, 2.]])
    assert _minimum_structure_distance(path, structure) == pytest.approx(.25)


@pytest.mark.parametrize('policy', [replace(CenteringPolicy(), smoothing_length_m=0),
    replace(CenteringPolicy(), minimum_center_error_reduction=1),
    replace(CenteringPolicy(), normal_neighbors=True),
    replace(CenteringPolicy(), maximum_normal_z=np.nan)])
def test_invalid_fit_scales_rejected(policy):
    with pytest.raises(ValueError, match='policy'):
        fit_measured_sides(flight(), policy=policy)


def test_applied_shape_keeps_z_layers_endpoints_external_span_and_input_map(monkeypatch):
    grid, path, layers, owner, settings = example(monkeypatch)
    before, map_before = path.copy(), grid.data.copy()
    source_calls = []
    result = center_measured_stair_path(grid, path, layers, owner, settings=settings, flights=[flight()],
        validate_candidate=lambda xyz, ids: source_calls.append((xyz.copy(), ids.copy())) or True)
    assert result['applied'], result['diagnostics']['candidate_rejection']
    new = result['path']
    first, last = (result['flight_spans'][0][key] for key in ('first_index', 'last_index'))
    assert np.array_equal(new[:first + 2], path[:first + 2])
    assert np.array_equal(new[last - 1:], path[last - 1:])
    assert np.array_equal(new[:, 2], path[:, 2])
    assert np.array_equal(result['layer_ids'], layers)
    assert np.array_equal(path, before) and np.array_equal(grid.data, map_before)
    assert len(source_calls) == 1 and np.array_equal(source_calls[0][0], new)
    assert result['diagnostics']['execution_authorized'] is False
    report = result['diagnostics']['flights'][0]
    assert report['after']['quality_10cm']['curvature_max_per_m'] < report['before']['quality_10cm']['curvature_max_per_m']
    assert report['after']['actual_retained_structure_distance_minimum_m'] >= report['before']['actual_retained_structure_distance_minimum_m']


@pytest.mark.parametrize('failure', ['current_obstacle', 'source_missing', 'source_exception'])
def test_current_geometry_or_source_failure_returns_entire_old_route(monkeypatch, failure):
    grid, path, layers, owner, settings = example(monkeypatch)
    if failure == 'current_obstacle':
        grid.data[0, 0] = 50.
        # Tomogram validity is materialized; reconstruct from the mutated data.
        grid = TomogramMap(dict(data=grid.data.copy(), resolution=.05, center=[0., 0.], slice_h0=.1, slice_dh=.5))
    def proof(*args):
        if failure == 'source_exception': raise ValueError('original_source_missing')
        return failure != 'source_missing'
    result = center_measured_stair_path(grid, path, layers, owner,
        settings=settings, flights=[flight()], validate_candidate=proof)
    assert not result['applied']
    assert np.array_equal(result['path'], path)
    assert result['diagnostics']['candidate_rejection'].startswith('geometry_or_source_rejected')


def test_cannot_change_owner_or_connect_nonmonotone_flight(monkeypatch):
    grid, path, layers, owner, settings = example(monkeypatch)
    path[60, 0] = path[59, 0] - .02
    with pytest.raises(ValueError, match='flight_not_monotone'):
        center_measured_stair_path(grid, path, layers, owner, settings=settings,
            flights=[flight()], validate_candidate=lambda *args: True)


def test_source_proof_required_not_assumed_from_pcd_side_fit(monkeypatch):
    grid, path, layers, owner, settings = example(monkeypatch)
    with pytest.raises(ValueError, match='route_contract'):
        center_measured_stair_path(grid, path, layers, owner,
            settings=settings, flights=[flight()], validate_candidate=None)


@pytest.mark.parametrize('side', [-1., 1.])
def test_structured_middle_is_exact_line_not_whole_flight_bow(side):
    s = np.linspace(0., 4., 161)
    old = np.full(len(s), side * .2)
    structure = np.array([[x, y] for x in s for y in (-.8, .8)])
    result, profile = _centered_lateral(s, old, [0., 0.], .5,
        structure, .6, .1, return_profile=True)
    begin, end = profile['center_along_range_m']
    center = (s >= begin) & (s <= end)
    assert end - begin >= 1.
    assert np.array_equal(result[center], np.zeros(np.count_nonzero(center)))
    assert result[[0, -1]].tolist() == old[[0, -1]].tolist()
    assert np.all(side * result >= -1e-12)
    assert np.max(abs(result)) <= .2 + 1e-12
    assert profile['kind'] == 'c2_entry_exact_middle_c2_exit'
    assert profile['sampled_curve_curvature_max_per_m'] < 1.
    for index, record in enumerate(profile['transitions']):
        from numpy.polynomial import Polynomial
        polynomial = Polynomial(record['normalized_coefficients'])
        length = record['length_m']
        assert polynomial(0.) == pytest.approx(record['start_value_m'])
        assert polynomial(1.) == pytest.approx(record['end_value_m'])
        assert polynomial.deriv()(0.) / length == pytest.approx(record['start_slope'])
        assert polynomial.deriv()(1.) / length == pytest.approx(record['end_slope'])
        middle_end, outer_end = ((1., 0.) if index == 0 else (0., 1.))
        assert polynomial.deriv(2)(middle_end) / length ** 2 == pytest.approx(0., abs=1e-10)
        assert polynomial.deriv(3)(outer_end) / length ** 3 == pytest.approx(0., abs=1e-10)


def test_structured_center_uses_sloping_measured_middle_and_preserves_entry_tangent():
    s = np.linspace(-2., 2., 161)
    line = np.array([.03, .01])
    old = np.polyval(line, s) + .15
    structure = np.array([[x, np.polyval(line, x) + y] for x in s for y in (-.8, .8)])
    result, profile = _centered_lateral(s, old, line, .5, structure, .64, .1,
        endpoint_slopes=(-.12, .02), return_profile=True)
    begin, end = profile['center_along_range_m']
    middle = (s >= begin) & (s <= end)
    assert np.array_equal(result[middle], np.polyval(line, s[middle]))
    assert profile['transitions'][0]['start_slope'] == -.12
    assert profile['transitions'][1]['end_slope'] == .02


def test_structured_center_cannot_hide_real_middle_structure():
    s = np.linspace(0., 4., 161)
    old = np.full(len(s), .2)
    # An observed projection onto the fitted middle is a real obstruction,
    # not permission to make the middle green or to reuse a bowed old path.
    structure = np.array([[x, y] for x in s for y in (-.8, .8)] + [[2., 0.]])
    with pytest.raises(ValueError, match='structured_center_no_feasible_transition'):
        _centered_lateral(s, old, [0., 0.], .5, structure, .2, .1,
            return_profile=True)


@pytest.mark.parametrize('middle_at_end', [True, False])
def test_transition_free_outer_second_derivative_minimizes_spatial_jerk(middle_at_end):
    from numpy.polynomial import Polynomial
    polynomial = _center_transition(.23, -.04, -.27, .03, 1.3,
                                     middle_at_end=middle_at_end)
    assert polynomial(0.) == pytest.approx(.23)
    assert polynomial(1.) == pytest.approx(-.04)
    assert polynomial.deriv()(0.) / 1.3 == pytest.approx(-.27)
    assert polynomial.deriv()(1.) / 1.3 == pytest.approx(.03)
    middle_end, outer_end = ((1., 0.) if middle_at_end else (0., 1.))
    assert polynomial.deriv(2)(middle_end) == pytest.approx(0., abs=1e-10)
    assert polynomial.deriv(3)(outer_end) == pytest.approx(0., abs=1e-10)
    # The only free quintic coefficient preserves both values/slopes and the
    # affine end's second derivative. Its two-sided perturbation must increase
    # the exact integral, not merely a sampled low-order roughness score.
    direction = Polynomial([0., 0., 1., -3., 3., -1.])
    if not middle_at_end:
        direction = direction(Polynomial([1., -1.]))
    def energy(curve):
        integrated = (curve.deriv(3) ** 2).integ()
        return integrated(1.) - integrated(0.)
    for delta in [-1., -.1, -.01, .01, .1, 1.]:
        assert energy(polynomial + delta * direction) > energy(polynomial)
