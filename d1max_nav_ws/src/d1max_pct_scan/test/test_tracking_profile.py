from pathlib import Path
import pytest
import yaml
from d1max_pct_scan.tracking_profile import load,command_response_parameters


def test_platform_is_holonomic_but_navigation_uses_continuous_forward_steering():
    profile=load(Path(__file__).resolve().parents[1]/'config/motion_tracking.yaml')
    assert profile['motion_model']=='holonomic' and profile['kinematic_profile']=='holonomic_xyyaw'
    assert profile['max_speed_mps']==.3
    assert profile['controller_frequency_hz']==20.
    assert profile['min_lookahead_distance_m']==.40
    assert profile['max_lookahead_distance_m']==.80
    assert profile['align_then_forward'] is True
    assert profile['steering_error_scale_rad']==.04
    assert profile['position_gain']==.8 and profile['yaw_gain']==1.2
    assert profile['heading_error_threshold_rad']==.8
    assert profile['velocity_damping_gain']==0. and profile['yaw_damping_gain']==.15
    assert profile['max_jerk_mps3']==.70 and profile['max_yaw_jerk_radps3']==2.
    assert profile['max_acceleration_mps2']==.35
    assert profile['max_yaw_acceleration_radps2']==.8
    assert profile['max_input_rate_per_s']==1.6
    assert profile['max_input_acceleration_per_s2']==12.


@pytest.mark.parametrize('changes',[{'motion_model':'mecanum_guess'},
    {'motion_model':'forward_yaw','heading_policy':'hold_heading'},
    {'max_speed_mps':float('nan')},{'max_speed_mps':.31},{'max_acceleration_mps2':True}])
def test_invalid_or_unverified_profile_is_not_launchable(tmp_path,changes):
    profile=load(Path(__file__).resolve().parents[1]/'config/motion_tracking.yaml')
    profile.update(changes);path=tmp_path/'profile.yaml';path.write_text(yaml.safe_dump(profile))
    with pytest.raises(ValueError):load(path)


def test_hold_heading_is_a_software_option_not_a_new_controller(tmp_path):
    profile=load(Path(__file__).resolve().parents[1]/'config/motion_tracking.yaml')
    profile['heading_policy']='hold_heading';profile['align_then_forward']=False
    path=tmp_path/'profile.yaml';path.write_text(yaml.safe_dump(profile))
    assert load(path)['kinematic_profile']=='holonomic_xyyaw'


@pytest.mark.parametrize('changes',[{'controller_frequency_hz':True},
    {'controller_frequency_hz':float('nan')},{'controller_frequency_hz':0},
    {'diagnostics_frequency_hz':21.},
    {'min_lookahead_distance_m':.5,'max_lookahead_distance_m':.3},
    {'preview_time_s':-1.},{'velocity_damping_gain':float('nan')},
    {'yaw_damping_gain':True},{'velocity_damping_gain':-.1},
    {'max_jerk_mps3':float('inf')},{'max_jerk_mps3':-.1},
    {'max_yaw_jerk_radps3':True},{'max_yaw_jerk_radps3':4.1},
    {'heading_error_threshold_rad':True},{'heading_error_threshold_rad':0},
    {'heading_error_threshold_rad':float('nan')},{'heading_error_threshold_rad':3.2}])
def test_controller_parameters_are_finite_and_consistent(tmp_path,changes):
    profile=load(Path(__file__).resolve().parents[1]/'config/motion_tracking.yaml')
    profile.update(changes);path=tmp_path/'profile.yaml';path.write_text(yaml.safe_dump(profile))
    with pytest.raises(ValueError):load(path)


def test_command_response_is_projected_from_same_immutable_sdk_profile():
    import json
    root=Path(__file__).resolve().parents[1]/'config'
    tracking=load(root/'motion_tracking.yaml')
    record=json.loads((root/'runtime_motion_profile.json').read_text())
    parameters=command_response_parameters(tracking,record)
    assert parameters['command_response_enabled'] is True
    for name,axis,scale in (('forward_response','planar','max_speed_mps'),
                           ('yaw_response','yaw','max_yaw_radps')):
        assert parameters[name+'_reference']==record['measurements'][scale]
        assert parameters[name+'_effective_fraction']==record['sdk_input_mapping'][axis]['effective_fraction']
        assert parameters[name+'_maximum_fraction']==record['sdk_input_mapping'][axis]['maximum_fraction']
        assert parameters[name+'_blend_ratio']==record['sdk_input_mapping'][axis]['blend_demand_ratio']
    assert parameters['max_input_rate']==tracking['max_input_rate_per_s']
    assert parameters['max_input_acceleration']==tracking['max_input_acceleration_per_s2']


def test_response_limits_never_turn_legacy_or_lateral_tracking_into_new_sdk_policy():
    import json
    root=Path(__file__).resolve().parents[1]/'config'
    tracking=load(root/'motion_tracking.yaml')
    record=json.loads((root/'runtime_motion_profile.json').read_text())
    assert command_response_parameters(tracking,None)['command_response_enabled'] is False
    tracking['align_then_forward']=False
    assert command_response_parameters(tracking,record)['command_response_enabled'] is False


@pytest.mark.parametrize('field,value',[
    ('max_input_rate_per_s',0.),('max_input_rate_per_s',True),
    ('max_input_rate_per_s',float('nan')),('max_input_acceleration_per_s2',0.),
    ('max_input_acceleration_per_s2',float('inf')),('max_input_acceleration_per_s2',True)])
def test_command_input_bounds_are_typed_finite_not_new_runtime_gates(tmp_path,field,value):
    profile=load(Path(__file__).resolve().parents[1]/'config/motion_tracking.yaml')
    profile[field]=value
    path=tmp_path/'profile.yaml';path.write_text(yaml.safe_dump(profile))
    with pytest.raises(ValueError):load(path)


@pytest.mark.parametrize('changes', [
    {'steering_enter_angle_rad':float('nan')},
    {'steering_exit_angle_rad':True},
    {'steering_enter_angle_rad':0.},
    {'steering_exit_angle_rad':.6},
    {'steering_exit_angle_rad':.2,'steering_enter_angle_rad':.1},
    {'steering_exit_angle_rad':.1,'steering_enter_angle_rad':.1},
])
def test_straight_steering_band_is_validated(tmp_path,changes):
    profile=load(Path(__file__).resolve().parents[1]/'config/motion_tracking.yaml')
    profile.update(changes);path=tmp_path/'profile.yaml';path.write_text(yaml.safe_dump(profile))
    with pytest.raises(ValueError):load(path)
