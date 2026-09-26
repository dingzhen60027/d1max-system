"""One bounded commissioned speed profile for Nav2 and its command gate.

This module is pure configuration: it neither contacts nor arms the SDK.  The
user's 1.5 m/s ceiling is not a target speed.  This release only commissions
forward commands in the SDK's documented low-speed profile (at most 1.0 m/s).
The independent command gate and SDK adapter remain the final enforcement.
"""
from copy import deepcopy
import math


USER_PLANAR_CEILING = 1.5
SDK_LOW_GEAR_FORWARD_CEILING = 1.0


def validated_motion_limits(raw):
    if not isinstance(raw, dict):
        raise ValueError('motion_limits must be an explicit configuration mapping')
    required = {'max_planar', 'max_forward', 'max_lateral', 'max_yaw',
                'accel_forward', 'accel_yaw'}
    if set(raw) != required:
        raise ValueError('motion_limits fields must be exactly: ' + ', '.join(sorted(required)))
    if any(type(value) not in (float, int) or not math.isfinite(value) for value in raw.values()):
        raise ValueError('motion_limits must contain finite numeric values')
    limits = {key: float(value) for key, value in raw.items()}
    if not 0 < limits['max_planar'] <= USER_PLANAR_CEILING:
        raise ValueError('max_planar may never exceed the user 1.5 m/s ceiling')
    if not 0 < limits['max_forward'] <= min(limits['max_planar'], SDK_LOW_GEAR_FORWARD_CEILING):
        raise ValueError('max_forward exceeds the planar or commissioned SDK low-speed bound')
    if limits['max_lateral'] != 0.0:
        raise ValueError('This commissioned navigation profile is forward-only; lateral speed must be zero')
    if not 0 < limits['max_yaw'] <= 0.5:
        raise ValueError('max_yaw exceeds the commissioned 0.5 rad/s bound')
    if not 0 < limits['accel_forward'] <= 0.4 or not 0 < limits['accel_yaw'] <= 0.8:
        raise ValueError('Acceleration exceeds the commissioned navigation bounds')
    return limits


def configured_nav2(source, raw_limits):
    """Return a private snapshot; never edit the shared Nav2 configuration."""
    limits = validated_motion_limits(raw_limits)
    result = deepcopy(source)
    controller = result['controller_server']['ros__parameters']['FollowPath']
    smoother = result['velocity_smoother']['ros__parameters']
    controller.update(min_vel_x=0.0, min_vel_y=0.0,
                      max_vel_x=limits['max_forward'], max_vel_y=0.0,
                      max_vel_theta=limits['max_yaw'], max_speed_xy=limits['max_forward'],
                      acc_lim_x=limits['accel_forward'], acc_lim_y=0.0,
                      acc_lim_theta=limits['accel_yaw'],
                      decel_lim_x=-limits['accel_forward'], decel_lim_y=0.0,
                      decel_lim_theta=-limits['accel_yaw'])
    smoother.update(max_velocity=[limits['max_forward'], 0.0, limits['max_yaw']],
                    min_velocity=[0.0, 0.0, -limits['max_yaw']],
                    max_accel=[limits['accel_forward'], 0.0, limits['accel_yaw']],
                    max_decel=[-limits['accel_forward'], 0.0, -limits['accel_yaw']])
    return result


def gate_parameters(raw_limits):
    limits = validated_motion_limits(raw_limits)
    return {key: limits[key] for key in ('max_forward', 'max_lateral', 'max_yaw')}
