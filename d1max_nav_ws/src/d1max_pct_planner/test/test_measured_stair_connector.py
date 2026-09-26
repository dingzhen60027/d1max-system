import numpy as np
import pytest

from d1max_pct_planner.measured_stair_connector import (
    ConnectorLimits, audit_measured_stair_connector,
)


def ramp_scene(*, floor=None):
    floor = floor or (lambda x: 0.3 * x)
    x = np.arange(0., 10.001, .04)
    y = np.arange(-.5, .501, .04)
    xx, yy = np.meshgrid(x, y, indexing='ij')
    cloud = np.column_stack((xx.ravel(), yy.ravel(), floor(xx).ravel()))
    pose_x = np.arange(0., 10.001, .1)
    poses = np.column_stack((pose_x, np.zeros_like(pose_x), floor(pose_x) + .5))
    return cloud, poses


def audit(cloud, poses):
    return audit_measured_stair_connector(cloud, poses,
                                          start_index=0, end_index=len(poses) - 1)


def test_dense_measured_ramp_is_only_a_candidate_never_execution_permission():
    cloud, poses = ramp_scene()
    result = audit(cloud, poses)
    assert result['status'] == 'candidate_only'
    assert result['geometry_gates_passed'] is True
    assert result['execution_authorized'] is False
    assert result['route_certified_safe'] is False
    assert result['failure_counts'] == {}
    assert result['sample_count'] == 101
    assert result['measured_floor_rise_m'] == pytest.approx(3.0, abs=.1)
    assert all(len(sample['probes']) == 5 for sample in result['samples'])
    assert all(sample['measured_ground_xyz'] is not None for sample in result['samples'])


def test_missing_measured_patch_is_not_filled_from_the_history():
    cloud, poses = ramp_scene()
    cloud = cloud[(cloud[:, 0] < 4.35) | (cloud[:, 0] > 4.85)]
    result = audit(cloud, poses)
    assert result['status'] == 'rejected'
    assert result['failure_counts']['center_ground_unobserved'] > 0
    assert result['failure_counts']['ground_profile_incomplete'] == 1
    assert any(sample['measured_ground_xyz'] is None for sample in result['samples'])


def test_missing_lateral_width_is_rejected_despite_center_support():
    cloud, poses = ramp_scene()
    cloud = cloud[(cloud[:, 0] < 4.) | (cloud[:, 0] > 6.) | (cloud[:, 1] < .1)]
    result = audit(cloud, poses)
    assert result['status'] == 'rejected'
    assert result['failure_counts']['lateral_ground_unobserved'] > 0


def test_observed_body_envelope_return_is_not_declared_free():
    cloud, poses = ramp_scene()
    obstacle = np.array([[4.0, 0., 1.55], [4.02, 0., 1.57], [4.04, 0., 1.59]])
    result = audit(np.vstack((cloud, obstacle)), poses)
    assert result['status'] == 'rejected'
    assert result['failure_counts']['potential_body_obstacle'] > 0


def test_height_discontinuity_is_rejected_not_smoothed_away():
    def floor(x):
        return 0.2 * x + np.where(x >= 5., .4, 0.)

    cloud, poses = ramp_scene(floor=floor)
    result = audit(cloud, poses)
    assert result['status'] == 'rejected'
    assert result['failure_counts']['ground_step_exceeds_limit'] > 0


def test_long_pose_gap_is_rejected_even_with_dense_pointcloud():
    cloud, poses = ramp_scene()
    result = audit(cloud, poses[[0, -1]])
    assert result['status'] == 'rejected'
    assert result['failure_counts'] == {'input_pose_gap': 1}
    assert result['sample_count'] == 0


def test_same_xy_vertical_teleport_is_rejected_before_surface_sampling():
    cloud, poses = ramp_scene()
    jumped = poses.copy()
    jumped[50, :2] = jumped[49, :2]
    jumped[50, 2] = jumped[49, 2] + 3.0
    result = audit(cloud, jumped)
    assert result['status'] == 'rejected'
    assert result['failure_counts'] == {'vertical_pose_jump': 1}


def test_inputs_and_limits_are_explicit_and_finite():
    cloud, poses = ramp_scene()
    with pytest.raises(ValueError):
        ConnectorLimits(sample_spacing_m=0.5)
    with pytest.raises(ValueError):
        audit_measured_stair_connector(cloud, poses, start_index=10, end_index=10)
    with pytest.raises(ValueError):
        audit_measured_stair_connector(np.array([[np.nan, 0., 0.]]), poses,
                                       start_index=0, end_index=1)
