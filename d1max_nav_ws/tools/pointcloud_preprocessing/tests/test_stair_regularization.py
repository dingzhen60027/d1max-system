import numpy as np
import pytest

from tools.pointcloud_preprocessing.stair_regularization import (
    RegularizationLimits, StairSurface, regularize_stair_support,
    connected_measured_support, extend_measured_stair_support,
)


def _flight(seed=7):
    rng = np.random.default_rng(seed)
    x, y = np.meshgrid(np.arange(-.5, .501, .04), np.arange(.05, 3.96, .04))
    points = np.c_[x.ravel(), y.ravel(), .5*y.ravel()+rng.normal(0., .009, x.size)]
    return points


def _run(points, seeds=None, surfaces=None):
    if seeds is None:
        seeds = np.ones(len(points), dtype=bool)
    if surfaces is None:
        surfaces = [StairSurface(1, 'lower', 'flight_support_envelope',
                                (-1., 0., 0.), (1., 4., 2.))]
    return regularize_stair_support(points, np.arange(len(points)), seeds, surfaces)


def test_observed_flight_is_regularized_without_moving_xy_or_inventing_steps():
    points = _flight()
    before = points.copy()
    result = _run(points)
    assert result.diagnostics['changed_points'] > 1000
    assert np.array_equal(points, before)
    assert np.array_equal(result.planning_xyz[:, :2], points[result.source_indices, :2])
    assert np.array_equal(result.source_xyz, points[result.source_indices])
    assert np.max(abs(result.planning_xyz[:, 2]-result.source_xyz[:, 2])) <= .08
    model = result.models[0]
    assert abs(model['grade'] - .5) < .01
    assert model['residual_after_rms_m'] < model['residual_before_rms_m'] * .25
    assert model['actual_tread_rise_m'] is None
    assert result.diagnostics['synthetic_points'] == 0


def test_vertical_walls_and_far_obstacle_points_are_not_support():
    points = _flight()
    y, z = np.meshgrid(np.arange(0., 4., .04), np.arange(.1, 2.6, .04))
    walls = np.c_[np.full(y.size, .80), y.ravel(), z.ravel()]
    box = np.array([[0., 2., 1.35], [0., 2.04, 1.37]])
    merged = np.r_[points, walls, box]
    seeds = np.r_[np.ones(len(points), bool), np.zeros(len(walls)+len(box), bool)]
    result = _run(merged, seeds)
    assert np.max(result.source_indices) < len(points)
    assert result.diagnostics['removed_points'] == 0


def test_missing_tread_or_unobserved_strip_is_not_filled():
    points = _flight()
    points = points[(points[:, 1] < 1.4) | (points[:, 1] > 1.8)]
    result = _run(points)
    assert not np.any((result.planning_xyz[:, 1] >= 1.4)
                      & (result.planning_xyz[:, 1] <= 1.8))
    assert len(result.source_indices) <= len(points)


def test_landing_is_separate_and_other_floor_is_not_claimed():
    x, y = np.meshgrid(np.arange(-.5, .51, .04), np.arange(4.1, 5.2, .04))
    landing = np.c_[x.ravel(), y.ravel(), 2.01+.004*np.sin(y.ravel()*8)]
    points = np.r_[_flight(), landing, landing+[0, 0, 3.]]
    spec = [StairSurface(1, 'lower', 'flight_support_envelope', (-1, 0, 0), (1, 4, 2)),
            StairSurface(2, 'landing', 'landing_plane', (-1, 4.05, 1.9), (1, 5.3, 2.1))]
    result = _run(points, surfaces=spec)
    assert set(result.surface_id) == {1, 2}
    assert result.source_xyz[:, 2].max() < 2.1
    assert np.max(result.source_indices) < len(points)-len(landing)


def test_bad_model_does_not_synthesize_nominal_staircase():
    points = _flight()
    points[:, 2] = 1.
    result = _run(points)
    assert len(result.source_indices) == 0
    assert result.diagnostics['skipped_surfaces']


def test_duplicate_source_ids_and_overlapping_ownership_are_rejected():
    points = _flight()
    with pytest.raises(ValueError, match='invalid_original'):
        regularize_stair_support(points, np.zeros(len(points), int),
                                 np.ones(len(points), bool), [])
    specs = [StairSurface(i, str(i), 'flight_support_envelope',
                          (-1, 0, 0), (1, 4, 2)) for i in (1, 2)]
    with pytest.raises(ValueError, match='overlapping'):
        _run(points, surfaces=specs)


def test_deterministic_fit_and_endpoint_blend():
    points = _flight()
    a, b = _run(points), _run(points)
    assert np.array_equal(a.planning_xyz, b.planning_xyz)
    assert np.array_equal(a.source_indices, b.source_indices)
    assert a.models == b.models
    with pytest.raises(ValueError, match='limits'):
        RegularizationLimits(maximum_displacement_m=1.)


def test_connected_return_patch_reaches_beyond_one_hop_without_enlarging_radius():
    points = np.array([[0., 0., 0.], [.15, 0., 0.], [.30, 0., 0.], [.60, 0., 0.]])
    before = points.copy()
    selected, report = connected_measured_support(points, np.ones(4, bool),
        np.array([True, False, False, False]), .16)
    assert selected.tolist() == [True, True, True, False]
    assert report['neighbour_radius_m'] == .16
    np.testing.assert_array_equal(points, before)


def test_patch_cannot_bridge_ineligible_wall_or_vertical_gap_or_missing_seed():
    points = np.array([[0., 0., 0.], [.15, 0., 0.], [.30, 0., 0.], [0., 0., 3.]])
    seeds = np.array([True, False, False, False])
    selected, _ = connected_measured_support(points, np.array([True, False, True, True]), seeds, .16)
    assert selected.tolist() == [True, False, False, False]
    selected, _ = connected_measured_support(points, np.ones(4, bool), np.zeros(4, bool), .16)
    assert not selected.any()


def test_patch_work_is_bounded_before_allocating_dense_adjacency():
    points = np.zeros((20, 3))
    with pytest.raises(ValueError, match='work_limit'):
        connected_measured_support(points, np.ones(20, bool), np.ones(20, bool), .16,
                                   maximum_neighbour_visits=10)
    with pytest.raises(ValueError, match='work_limit'):
        connected_measured_support(points, np.ones(20, bool), np.ones(20, bool), .16,
                                   maximum_points=10)


def test_fixed_model_extension_keeps_old_rows_and_uses_absolute_original_coordinates():
    points = _flight()
    fitted = _run(points)
    model = fitted.models[0]
    owners = np.full(len(points), -1, np.int32)
    # An already-proven observed strip anchors the wider connected patch.
    mapped = fitted.source_indices[points[fitted.source_indices, 1] < .65]
    owners[mapped] = 1
    preserved_identity = int(fitted.source_indices[len(fitted.source_indices)//2])
    owners[preserved_identity] = 0
    original_models = repr(fitted.models)
    extension = extend_measured_stair_support(points, np.arange(len(points)), owners, fitted.models)
    assert len(extension.source_indices) > 1000
    assert not np.any(owners[extension.source_indices] >= 0)
    assert not np.isin(preserved_identity, extension.source_indices)
    np.testing.assert_array_equal(extension.source_xyz, points[extension.source_indices])
    assert np.max(abs(extension.planning_xyz[:, 2]-extension.source_xyz[:, 2])) <= .08
    assert repr(fitted.models) == original_models
    assert extension.models[0]['grade'] == model['grade']
    no_seed = extend_measured_stair_support(points, np.arange(len(points)),
                                          np.full(len(points), -1, np.int32), fitted.models)
    assert not len(no_seed.source_indices)
    with pytest.raises(ValueError, match='surface_selection'):
        extend_measured_stair_support(points, np.arange(len(points)), owners,
                                     fitted.models, surface_ids=[99])


def test_explicit_extension_scope_does_not_change_an_unreviewed_surface():
    from copy import deepcopy
    points = _flight()
    result = _run(points)
    shifted = points + np.array([3., 0., 0.])
    another = deepcopy(result.models[0])
    another.update(id=2, name='other_flight', center_xy=(np.asarray(another['center_xy'])+[3., 0.]).tolist(),
                   scope_minimum=(np.asarray(another['scope_minimum'])+[3., 0., 0.]).tolist(),
                   scope_maximum=(np.asarray(another['scope_maximum'])+[3., 0., 0.]).tolist())
    owners = np.full(len(points)*2, -1, np.int32)
    anchors = result.source_indices[points[result.source_indices, 1] < .65]
    owners[anchors] = 1
    owners[anchors+len(points)] = 2
    extension = extend_measured_stair_support(np.r_[points, shifted], np.arange(len(owners)), owners,
                                             [result.models[0], another], surface_ids=[1])
    assert len(extension.source_indices) > 1000
    assert np.all(extension.source_indices < len(points))
    assert extension.diagnostics['selected_surface_ids'] == [1]


def test_initial_landing_keeps_original_one_hop_selection():
    x, y = np.meshgrid(np.arange(-.5, .51, .04), np.arange(.05, 1.96, .04))
    points = np.c_[x.ravel(), y.ravel(), 1.+.002*np.sin(y.ravel()*3)]
    seeds = points[:, 1] < .45
    result = _run(points, seeds, [StairSurface(2, 'landing', 'landing_plane',
                                              (-1., 0., .9), (1., 2., 1.1))])
    assert len(result.source_indices) > 100
    assert result.source_xyz[:, 1].max() <= points[seeds, 1].max()+.16
    assert result.models[0]['support_connectivity']['policy'] == 'landing_original_one_hop_unchanged'
