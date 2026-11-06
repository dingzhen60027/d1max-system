"""One pinned body-motion profile for SCAN, the tracker and SDK calibration.

This config selects software kinematics; it cannot certify a physical velocity
scale or braking measurement. Those remain bound to the SDK's measured record.
"""
import math
from pathlib import Path
import yaml


def load(path):
    profile = yaml.safe_load(Path(path).read_text())
    if not isinstance(profile, dict) or profile.get('schema_version') != 1:
        raise ValueError('invalid_tracking_profile_schema')
    model = profile.get('motion_model')
    heading = profile.get('heading_policy')
    if model not in ('forward_yaw', 'holonomic') or heading not in ('path_tangent', 'hold_heading'):
        raise ValueError('invalid_tracking_kinematics_or_heading')
    if model == 'forward_yaw' and heading != 'path_tangent':
        raise ValueError('forward_tracking_requires_path_tangent')
    align = profile.setdefault('align_then_forward', False)
    if type(align) is not bool or align and (model != 'holonomic' or heading != 'path_tangent'):
        raise ValueError('invalid_align_then_forward_strategy')
    bounds = dict(max_speed_mps=.30, max_yaw_radps=.50,
                  max_acceleration_mps2=.35, max_yaw_acceleration_radps2=.80)
    for key, upper in bounds.items():
        value = profile.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not 0 < value <= upper:
            raise ValueError('invalid_tracking_limit:' + key)
    controller_defaults = dict(controller_frequency_hz=20., diagnostics_frequency_hz=5.,
        preview_time_s=.8, min_lookahead_distance_m=.05, max_lookahead_distance_m=.30,
        position_gain=.8, yaw_gain=1.5, heading_error_threshold_rad=.8,
        steering_enter_angle_rad=.10, steering_exit_angle_rad=.04,
        steering_error_scale_rad=.04,
        velocity_damping_gain=0., yaw_damping_gain=0.,max_jerk_mps3=0.,max_yaw_jerk_radps3=0.,
        max_input_rate_per_s=1.6,max_input_acceleration_per_s2=12.)
    controller_bounds = dict(controller_frequency_hz=(10.,50.), diagnostics_frequency_hz=(1.,10.),
        preview_time_s=(.05,2.), min_lookahead_distance_m=(.01,1.),
        max_lookahead_distance_m=(.01,1.), position_gain=(.01,3.), yaw_gain=(.01,3.),
        heading_error_threshold_rad=(.05,math.pi),
        steering_enter_angle_rad=(.001,.5), steering_exit_angle_rad=(.001,.5),
        steering_error_scale_rad=(.001,.5),
        velocity_damping_gain=(0.,2.), yaw_damping_gain=(0.,2.),
        max_jerk_mps3=(0.,2.),max_yaw_jerk_radps3=(0.,4.),
        max_input_rate_per_s=(.01,8.),max_input_acceleration_per_s2=(.01,80.))
    for key, default in controller_defaults.items():
        value = profile.setdefault(key, default)
        lo, hi = controller_bounds[key]
        if isinstance(value, bool) or not isinstance(value, (int,float)) or not math.isfinite(value) or not lo <= value <= hi:
            raise ValueError('invalid_tracking_controller:' + key)
    if (profile['steering_exit_angle_rad'] >= profile['steering_enter_angle_rad']
            or profile['min_lookahead_distance_m'] > profile['max_lookahead_distance_m']
            or profile['diagnostics_frequency_hz'] > profile['controller_frequency_hz']):
        raise ValueError('inconsistent_tracking_controller')
    return {**profile, 'kinematic_profile': 'holonomic_xyyaw' if model == 'holonomic' else 'forward_yaw'}


def command_response_parameters(tracking, record):
    """Project ONE immutable actuator description, without owning the SDK.

    The follower uses this static image only to choose its final SI command;
    the writer applies the original profile unchanged. Legacy/XY controllers
    keep their existing contract. No estimated physical gain is introduced.
    """
    parameters = dict(command_response_enabled=False,
        max_input_rate=tracking['max_input_rate_per_s'],
        max_input_acceleration=tracking['max_input_acceleration_per_s2'])
    if not tracking['align_then_forward'] or not record:
        return parameters
    from .execution_timing import validate_sdk_input_mapping
    mapping = validate_sdk_input_mapping(record)
    if not mapping or mapping['schema_version'] != 4:
        return parameters
    parameters['command_response_enabled'] = True
    for name, axis, reference in (
            ('forward_response', 'planar', record['measurements']['max_speed_mps']),
            ('yaw_response', 'yaw', record['measurements']['max_yaw_radps'])):
        parameters[name+'_reference'] = float(reference)
        for source, target in (('effective_fraction','effective_fraction'),
                               ('maximum_fraction','maximum_fraction'),
                               ('blend_demand_ratio','blend_ratio')):
            parameters[name+'_'+target] = mapping[axis][source]
    return parameters
