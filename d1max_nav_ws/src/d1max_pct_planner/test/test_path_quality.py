import json

import numpy as np
import pytest

from d1max_pct_planner.path_quality import path_quality


def xyz(x, y):
    return np.column_stack((x, y, np.zeros(len(x))))


def test_straight_line_has_unit_ratio_no_turn_or_lateral_error():
    path = np.array([[0., 0., 0.], [3., 4., 0.]])
    before = path.copy()
    result = path_quality(path)
    assert np.array_equal(path, before)
    assert result['xy_length_m'] == pytest.approx(5)
    assert result['xy_chord_m'] == pytest.approx(5)
    assert result['length_chord_ratio'] == pytest.approx(1)
    assert result['lateral_deviation_max_m'] < 1e-12
    assert result['lateral_deviation_range_m'] < 1e-12
    assert result['total_abs_turn_rad'] < 1e-12
    assert result['curvature_max_per_m'] < 1e-11
    assert result['interior_excluding_1m']['available']
    assert result['resampled_points'] == 51


def test_circle_arc_curvature_matches_inverse_radius():
    radius = 3.
    angle = np.linspace(0, np.pi/2, 2001)
    result = path_quality(xyz(radius*np.cos(angle), radius*np.sin(angle)), spacing_m=.05)
    assert result['xy_length_m'] == pytest.approx(radius*np.pi/2, abs=1e-5)
    assert result['length_chord_ratio'] == pytest.approx(np.pi/(2*np.sqrt(2)), abs=1e-5)
    assert result['lateral_deviation_max_m'] == pytest.approx(radius*(1-1/np.sqrt(2)), abs=1e-5)
    assert result['curvature_p95_per_m'] == pytest.approx(1/radius, abs=.005)
    assert result['curvature_max_per_m'] == pytest.approx(1/radius, abs=.02)
    assert result['total_abs_turn_rad'] == pytest.approx(np.pi/2, abs=.03)


def test_closed_circle_has_undefined_chord_ratio_and_includes_closure_turn():
    angle = np.linspace(0, 2*np.pi, 4001)
    result = path_quality(xyz(2*np.cos(angle), 2*np.sin(angle)), .05)
    assert result['closed_xy']
    assert result['length_chord_ratio'] is None
    assert result['lateral_deviation_max_m'] is None
    assert result['total_abs_turn_rad'] == pytest.approx(2*np.pi, abs=1e-6)
    assert result['curvature_p95_per_m'] == pytest.approx(.5, abs=.01)


def test_snaking_line_exposes_more_distance_lateral_range_and_turning():
    x = np.linspace(0, 12, 2001)
    y = .4*np.sin(2*np.pi*x/3)
    result = path_quality(xyz(x, y))
    assert result['length_chord_ratio'] > 1.10
    assert result['lateral_deviation_max_m'] == pytest.approx(.4, abs=1e-5)
    assert result['lateral_deviation_range_m'] == pytest.approx(.8, abs=1e-5)
    assert result['total_abs_turn_rad'] > 8
    assert result['curvature_p95_per_m'] > 1
    assert result['interior_excluding_1m']['total_abs_turn_rad'] < result['total_abs_turn_rad']


def test_nonuniform_collinear_samples_and_duplicates_do_not_change_polyline_metrics():
    coarse = np.array([[0., 0., 0.], [3., 0., 0.], [3., 2., 0.], [6., 2., 0.]])
    dense = np.array([[0., 0., 0.], [.001, 0., 0.], [2.9, 0., 0.], [3., 0., 0.],
                      [3., 0., 2.], [3., .07, 0.], [3., 2., 0.], [3., 2., 0.],
                      [3.01, 2., 0.], [6., 2., 0.]])
    a, b = path_quality(coarse), path_quality(dense)
    for key in ('xy_length_m', 'xy_chord_m', 'length_chord_ratio', 'lateral_deviation_max_m',
                'lateral_deviation_range_m', 'total_abs_turn_rad', 'curvature_p95_per_m', 'curvature_max_per_m'):
        assert a[key] == pytest.approx(b[key], abs=1e-10)
    assert b['xyz_length_m'] > a['xyz_length_m']


def test_pure_z_motion_is_not_misreported_as_smooth_xy_navigation():
    result = path_quality([[2., 3., 0.], [2., 3., 1.], [2., 3., 2.]])
    assert result['status'] == 'no_xy_motion'
    assert result['xy_length_m'] == 0
    assert result['xyz_length_m'] == 2
    assert result['length_chord_ratio'] is None
    assert result['total_abs_turn_rad'] is None
    assert result['curvature_max_per_m'] is None


def test_single_vertex_has_no_xy_curvature():
    result = path_quality([[1., 2., 3.]])
    assert result['status'] == 'no_xy_motion'
    assert result['resampled_points'] == 1
    assert result['curvature_samples'] == 0


def test_short_nonzero_line_keeps_both_exact_endpoints():
    result = path_quality([[0., 0., 0.], [1e-12, 0., 0.]])
    assert result['resampled_points'] == 2
    assert not result['closed_xy']
    assert result['length_chord_ratio'] == 1
    assert result['curvature_max_per_m'] is None
    assert result['total_abs_turn_rad'] == 0
    assert result['status'] == 'insufficient_xy_heading_samples'


def test_tiny_closed_polyline_warns_if_resampling_cannot_resolve_it():
    result = path_quality([[0., 0., 0.], [.01, 0., 0.], [.01, .01, 0.], [0., 0., 0.]], 1.)
    assert result['xy_length_m'] > 0
    assert result['status'] == 'insufficient_xy_heading_samples'
    assert result['total_abs_turn_rad'] is None
    assert result['curvature_max_per_m'] is None


def test_endpoint_trim_does_not_replace_or_hide_full_path_metrics():
    result = path_quality([[0., 0., 0.], [.3, 0., 0.], [.3, 5., 0.], [.6, 5., 0.]])
    assert result['total_abs_turn_rad'] == pytest.approx(np.pi, abs=1e-8)
    assert result['curvature_max_per_m'] > 10
    assert result['interior_excluding_1m']['total_abs_turn_rad'] < 1e-9
    assert result['interior_excluding_1m']['curvature_max_per_m'] < 1e-9


def test_exact_multiple_spacing_does_not_add_nearly_duplicate_endpoint():
    result = path_quality([[0., 0., 0.], [1., 0., 0.]], .1)
    assert result['resampled_points'] == 11
    assert result['curvature_max_per_m'] == 0


@pytest.mark.parametrize('path', [[], [1., 2., 3.], [[0., 0.]], [[0., 0., np.nan]],
                                [[0., np.inf, 0.]], [['bad', 0, 0]]])
def test_bad_paths_rejected(path):
    with pytest.raises(ValueError):
        path_quality(path)


@pytest.mark.parametrize('spacing', [0, -1, np.nan, np.inf, True, 'bad'])
def test_bad_spacing_rejected(spacing):
    with pytest.raises(ValueError):
        path_quality([[0, 0, 0], [1, 0, 0]], spacing)


def test_excessively_dense_resampling_is_rejected_before_allocation():
    with pytest.raises(ValueError, match='one million'):
        path_quality([[0., 0., 0.], [2., 0., 0.]], 1e-9)


def test_results_are_strict_json_serializable():
    for path in ([[0, 0, 0]], [[0, 0, 0], [3, 4, 0]], [[0, 0, 0], [1, 0, 0], [0, 0, 0]]):
        result = path_quality(path)
        assert json.loads(json.dumps(result, allow_nan=False)) == result
        assert not result['safety_checked']


def test_finite_large_coordinates_do_not_overflow_chord_normalization():
    result = path_quality([[0., 0., 0.], [1e200, 0., 0.]], spacing_m=1e200)
    assert result['xy_length_m'] == 1e200
    assert result['length_chord_ratio'] == 1
    assert result['lateral_deviation_max_m'] == 0
    json.dumps(result, allow_nan=False)
