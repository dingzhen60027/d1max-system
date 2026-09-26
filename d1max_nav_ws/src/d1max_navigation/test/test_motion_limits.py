from copy import deepcopy
from pathlib import Path

import pytest
import yaml

from d1max_navigation.motion_limits import configured_nav2, gate_parameters, validated_motion_limits


ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def limits():
    return yaml.safe_load((ROOT / 'config/navigation_runtime.yaml').read_text())['motion_limits']


def test_commissioned_defaults_are_slow_and_below_user_ceiling(limits):
    actual = validated_motion_limits(limits)
    assert actual['max_planar'] == 1.5
    assert actual['max_forward'] == 0.3
    assert actual['max_lateral'] == 0.0
    assert gate_parameters(actual) == {'max_forward': 0.3, 'max_lateral': 0.0, 'max_yaw': 0.5}


@pytest.mark.parametrize('field,value', [
    ('max_planar', 1.50001), ('max_planar', 0), ('max_planar', -1),
    ('max_forward', 1.01), ('max_forward', -0.1), ('max_forward', 0),
    ('max_lateral', 0.01), ('max_yaw', 0.51), ('accel_forward', 0.41),
    ('accel_yaw', 0.81), ('max_planar', float('inf')),
    ('max_forward', float('nan')), ('max_forward', True), ('max_forward', '0.3'),
])
def test_invalid_or_uncommissioned_profile_refused(limits, field, value):
    limits[field] = value
    with pytest.raises(ValueError):
        validated_motion_limits(limits)


def test_lower_user_ceiling_restricts_operating_speed(limits):
    limits['max_planar'] = 0.2
    with pytest.raises(ValueError):
        validated_motion_limits(limits)


def test_missing_and_unknown_fields_fail_closed(limits):
    for raw in (None, {}, {**limits, 'typo': 1}):
        with pytest.raises(ValueError):
            validated_motion_limits(raw)


def test_all_command_layers_use_same_snapshot_without_mutating_source(limits):
    source = yaml.safe_load((ROOT / 'config/nav2.yaml').read_text())
    original = deepcopy(source)
    limits['max_forward'] = 0.2
    limits['max_yaw'] = 0.4
    generated = configured_nav2(source, limits)
    c = generated['controller_server']['ros__parameters']['FollowPath']
    s = generated['velocity_smoother']['ros__parameters']
    gate = gate_parameters(limits)
    assert c['max_speed_xy'] == c['max_vel_x'] == s['max_velocity'][0] == gate['max_forward'] == 0.2
    assert c['max_vel_theta'] == s['max_velocity'][2] == gate['max_yaw'] == 0.4
    assert s['min_velocity'][0] == 0.0
    assert source == original
