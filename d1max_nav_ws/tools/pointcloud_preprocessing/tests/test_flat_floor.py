import numpy as np
import pytest

from pointcloud_preprocessing.flat_floor import condition_flat_floor, DEFAULTS


def scene():
    x, y = np.meshgrid(np.arange(0., 4.01, .04), np.arange(-.60, .601, .04), indexing='ij')
    z = -.50 + .006 * x * x
    floor = np.column_stack((x.ravel(), y.ravel(), z.ravel()))
    tx = np.arange(.2, 3.81, .30)
    trajectory = np.column_stack((tx, tx * 0, .006 * tx * tx))
    return floor, trajectory


def test_warped_floor_is_conditioned_without_new_points_or_xy_movement():
    floor, trajectory = scene()
    before = floor.copy()
    result = condition_flat_floor(floor, trajectory)
    assert np.array_equal(floor, before)
    assert result['xyz'].shape == floor.shape
    assert np.array_equal(result['xyz'][:, :2], floor[:, :2])
    assert result['conditioning_mask'].all()
    assert result['planning_mask'].all()
    assert result['floor_snap_mask'].mean() > .90
    assert np.max(np.abs(result['xyz'][:, 2] + .5)) < .02
    assert result['statistics']['removed_near_ground_points'] == 0


def test_unknown_surface_is_not_filled_or_moved():
    floor, trajectory = scene()
    unknown = np.array([[2., 1.5, -.48], [20., 20., -1.]])
    points = np.vstack((floor, unknown))
    result = condition_flat_floor(points, trajectory)
    assert np.array_equal(result['xyz'][-2:], unknown)
    assert result['keep_mask'][-2:].all()
    assert not result['planning_mask'][-2:].any()
    assert not result['conditioning_mask'][-2:].any()
    assert np.isnan(result['floor_z'][-2:]).all()


def test_isolated_near_floor_return_removed_with_underlying_observed_support():
    floor, trajectory = scene()
    spike = np.array([[1., 0., -.50 + .006 + .085]])
    result = condition_flat_floor(np.vstack((floor, spike)), trajectory)
    assert result['near_ground_candidate_mask'][-1]
    assert not result['keep_mask'][-1]
    assert result['keep_mask'][:-1].all()


def test_wall_and_low_box_are_protected_and_keep_relative_height():
    floor, trajectory = scene()
    wy, wh = np.meshgrid(np.arange(-.40, .401, .04), np.arange(.02, .821, .04))
    wall = np.column_stack((np.full(wy.size, 2.), wy.ravel(), -.50 + .006 * 4 + wh.ravel()))
    bx, by = np.meshgrid(np.arange(.8, 1.121, .04), np.arange(-.12, .121, .04))
    box = np.column_stack((bx.ravel(), by.ravel(), -.50 + .006 * bx.ravel() ** 2 + .10))
    points = np.vstack((floor, wall, box))
    result = condition_flat_floor(points, trajectory)
    assert result['keep_mask'][len(floor):].all()
    assert result['conditioning_mask'][len(floor):].all()
    assert not result['floor_snap_mask'][len(floor) + len(wall):].any()
    assert np.min(result['xyz'][-len(box):, 2]) > -.43
    # Complete XY columns move together, preserving obstacle height exactly.
    ids = np.flatnonzero(~result['floor_snap_mask'] & result['conditioning_mask'])
    np.testing.assert_allclose(result['xyz'][ids, 2] + .50,
                               points[ids, 2] - result['floor_z'][ids], atol=1e-12)


def test_structure_above_protects_even_sparse_wall_roots():
    floor, trajectory = scene()
    extra = np.array([[1., .10, -.50 + .006 + h] for h in [.08, .24, .34, .44]])
    result = condition_flat_floor(np.vstack((floor, extra)), trajectory)
    assert result['near_ground_candidate_mask'][-4]
    assert result['near_ground_protected_mask'][-4]
    assert result['keep_mask'][-4:].all()


def test_ceiling_does_not_protect_isolated_low_noise():
    floor, trajectory = scene()
    extra = np.array([[1., .10, -.50 + .006 + h] for h in [.08, 2.0, 2.1, 2.2]])
    result = condition_flat_floor(np.vstack((floor, extra)), trajectory)
    assert not result['keep_mask'][-4]
    assert result['keep_mask'][-3:].all()


def test_below_floor_returns_are_excluded_only_from_planning_not_deleted():
    floor, trajectory = scene()
    extra = np.array([[1., .10, -.50 + .006 - 1.0]])
    result = condition_flat_floor(np.vstack((floor, extra)), trajectory)
    assert result['keep_mask'][-1]
    assert result['conditioning_mask'][-1]
    assert result['excluded_below_floor_mask'][-1]
    assert not result['planning_mask'][-1]
    assert result['statistics']['excluded_below_floor_points'] == 1


def test_disabled_cleanup_keeps_candidates():
    floor, trajectory = scene()
    point = np.array([[1., 0., -.50 + .006 + .08]])
    result = condition_flat_floor(np.vstack((floor, point)), trajectory,
                                  {'near_ground_cleanup_enabled': False})
    assert result['keep_mask'].all()
    assert result['near_ground_candidate_mask'][-1]


def test_indices_masks_are_original_length_and_outputs_finite():
    floor, trajectory = scene()
    permutation = np.random.default_rng(42).permutation(len(floor))
    source = floor[permutation]
    result = condition_flat_floor(source, trajectory)
    for name in ('keep_mask', 'floor_z', 'conditioning_mask', 'floor_support_mask', 'planning_mask'):
        assert result[name].shape == (len(source),)
    assert np.array_equal(result['xyz'][:, :2], source[:, :2])
    assert np.isfinite(result['xyz']).all()
    assert all(result[name].dtype == bool for name in ('keep_mask', 'conditioning_mask', 'planning_mask'))


@pytest.mark.parametrize('key,value', [
    ('bogus', 1), ('support_cell_m', 0), ('reference_z_m', np.nan),
    ('workers', 20), ('field_neighbors', 0), ('anchor_max_tilt_deg', 90),
    ('near_ground_cleanup_enabled', 1), ('floor_snap_halfwidth_m', .20),
    ('near_ground_max_m', .20),
])
def test_invalid_configuration_rejected(key, value):
    floor, trajectory = scene()
    with pytest.raises(ValueError):
        condition_flat_floor(floor, trajectory, {key: value})


def test_nonfinite_input_rejected():
    floor, trajectory = scene()
    floor[0, 0] = np.nan
    with pytest.raises(ValueError, match='finite'):
        condition_flat_floor(floor, trajectory)


def test_wall_only_cannot_seed_a_floor():
    y, z = np.meshgrid(np.arange(-1, 1, .04), np.arange(-.8, 1., .02))
    wall = np.column_stack((np.zeros(y.size), y.ravel(), z.ravel()))
    poses = np.column_stack((np.zeros(6), np.linspace(-.5, .5, 6), np.zeros(6)))
    with pytest.raises(ValueError, match='Insufficient reliable floor anchors'):
        condition_flat_floor(wall, poses)


def test_floor_height_never_inferred_from_trajectory_without_points():
    floor, trajectory = scene()
    with pytest.raises(ValueError, match='Insufficient reliable floor anchors'):
        condition_flat_floor(floor + [20., 20., 0.], trajectory)


def test_config_is_not_mutated():
    floor, trajectory = scene()
    config = {'reference_z_m': -.60}
    result = condition_flat_floor(floor, trajectory, config)
    assert config == {'reference_z_m': -.60}
    assert DEFAULTS['reference_z_m'] == -.50
    assert np.median(result['xyz'][:, 2]) == pytest.approx(-.60)


def test_cleanup_and_floor_snap_masks_never_overlap():
    floor, trajectory = scene()
    extras = np.array([[1., 0., -.50 + .006 + .047]])
    result = condition_flat_floor(np.vstack((floor, extras)), trajectory,
                                  {'floor_snap_halfwidth_m': .05})
    assert result['floor_snap_mask'][-1]
    assert result['keep_mask'][-1]
    assert not result['near_ground_candidate_mask'][-1]


def test_separated_height_returns_do_not_get_false_xy_only_density_support():
    floor, trajectory = scene()
    extras = np.array([[1., 0., -.50 + .006 + h] for h in [.06, .14, .22, .30]])
    result = condition_flat_floor(np.vstack((floor, extras)), trajectory,
                                  {'near_ground_max_m': .35, 'structure_above_min_m': .40})
    assert result['near_ground_candidate_mask'][-4:].all()
    assert not result['near_ground_protected_mask'][-4:].any()


def fragment_config():
    return {'near_ground_max_m': .35, 'structure_above_min_m': .40,
            'fragment_cleanup_enabled': True}


def test_dense_small_floating_fragment_can_be_removed_explicitly():
    floor, trajectory = scene()
    xyz = np.array([[1., 0., .15], [1.03, .02, .16], [1.01, -.02, .18], [1.05, 0., .14]])
    xyz[:, 2] += -.50 + .006 * xyz[:, 0] ** 2
    points = np.vstack((floor, xyz))
    previous = condition_flat_floor(points, trajectory, {'near_ground_max_m': .35, 'structure_above_min_m': .40})
    assert previous['keep_mask'][-4:].all()
    assert previous['statistics']['removed_fragment_points'] == 0
    cleaned = condition_flat_floor(points, trajectory, fragment_config())
    assert cleaned['removed_fragment_mask'][-4:].all()
    assert not cleaned['keep_mask'][-4:].any()
    assert cleaned['statistics']['removed_fragment_points'] == 4
    assert cleaned['keep_mask'][:-4].all()


def test_coherent_planar_low_box_remains_even_when_small():
    floor, trajectory = scene()
    x, y = np.meshgrid(np.arange(.94, 1.061, .06), np.arange(-.06, .061, .06))
    box = np.c_[x.ravel(), y.ravel(), -.5 + .006 * x.ravel() ** 2 + .16]
    result = condition_flat_floor(np.vstack((floor, box)), trajectory, fragment_config())
    assert result['keep_mask'][-len(box):].all()
    assert not result['removed_fragment_mask'][-len(box):].any()
    assert result['statistics']['fragment_components_protected_by_plane'] >= 1


def test_long_coherent_low_line_remains():
    floor, trajectory = scene()
    x = np.arange(.5, 1.101, .04)
    line = np.c_[x, x * 0, -.5 + .006 * x ** 2 + .16]
    result = condition_flat_floor(np.vstack((floor, line)), trajectory, fragment_config())
    assert result['keep_mask'][-len(line):].all()
    assert not result['removed_fragment_mask'][-len(line):].any()
    assert result['statistics']['fragment_components_protected_by_size'] >= 1


def test_floating_fragment_with_upward_structure_is_protected():
    floor, trajectory = scene()
    cluster = np.array([[1., 0., .15], [1.03, .02, .16], [1.01, -.02, .18], [1.05, 0., .14]])
    upper = np.array([[1., 0., .45], [1.03, .02, .55], [1.01, -.02, .65]])
    extras = np.vstack((cluster, upper))
    extras[:, 2] += -.50 + .006 * extras[:, 0] ** 2
    result = condition_flat_floor(np.vstack((floor, extras)), trajectory, fragment_config())
    assert result['keep_mask'][-len(extras):].all()
    assert not result['removed_fragment_mask'][-len(extras):].any()
    assert result['statistics']['fragment_components_protected_by_structure'] >= 1


def test_fragment_stage_never_clears_an_unobserved_floor_column():
    floor, trajectory = scene()
    cluster = np.array([[1., 1.5, .15], [1.03, 1.52, .16], [1.01, 1.48, .18], [1.05, 1.5, .14]])
    cluster[:, 2] += -.50 + .006 * cluster[:, 0] ** 2
    result = condition_flat_floor(np.vstack((floor, cluster)), trajectory, fragment_config())
    assert result['keep_mask'][-4:].all()
    assert not result['removed_fragment_mask'][-4:].any()
    assert not result['planning_mask'][-4:].any()
