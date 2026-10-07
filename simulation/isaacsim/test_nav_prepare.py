"""Preparation domains are independent of the navigation command authority."""
import pytest

from nav_prepare import quadruped_command_limits, spot_reachable_limits


@pytest.mark.parametrize('speed', [.6, .65])
def test_recalibrated_isolated_reach_does_not_raise_command_authority(speed):
    robot = dict(max_linear_speed=.23, max_angular_speed=.30,
        model_limits=dict(max_linear_speed_mps=speed, max_angular_speed_radps=.8))
    assert spot_reachable_limits(robot) == (speed, .8)
    assert quadruped_command_limits(robot) == dict(max_speed_mps=.23, max_yaw_radps=.30)


@pytest.mark.parametrize('speed', [.650001, .149999, False, '.65', float('nan'), float('inf')])
def test_reachable_domain_requires_finite_explicit_numbers_within_calibrated_cap(speed):
    robot = dict(model_limits=dict(max_linear_speed_mps=speed, max_angular_speed_radps=.8))
    with pytest.raises(ValueError): spot_reachable_limits(robot)


@pytest.mark.parametrize('field,value', [('max_linear_speed', .300001), ('max_angular_speed', .500001)])
def test_calibrated_plant_cannot_expand_original_command_hard_bounds(field, value):
    robot = dict(max_linear_speed=.23, max_angular_speed=.30,
        model_limits=dict(max_linear_speed_mps=.65, max_angular_speed_radps=.8))
    robot[field] = value
    assert spot_reachable_limits(robot) == (.65, .8)
    with pytest.raises(ValueError): quadruped_command_limits(robot)
