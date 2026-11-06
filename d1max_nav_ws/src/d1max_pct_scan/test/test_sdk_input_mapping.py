"""Contract checks without ROS, SDK connection or physical speed claims."""
import hashlib
import json
from pathlib import Path
import pytest
from d1max_pct_scan.execution_timing import (
    OPERATOR_BASIS, validate_runtime_profile, validate_sdk_input_mapping,
    validate_stationary)
from d1max_pct_scan.braking_model import load_model
from d1max_pct_scan.tracking_profile import load as load_tracking

CONFIG=Path(__file__).resolve().parents[1]/'config'


def profile():
    return json.loads((CONFIG/'runtime_motion_profile.json').read_text())


def legacy_v3_profile():
    """Archived V3 semantics stay explicit; the default must not use them."""
    record=profile();mapping=record['sdk_input_mapping']
    mapping['planar']['maximum_fraction']=.40
    mapping.update(schema_version=3,kind='direct_breakaway_ratio_v3',
        normalization='dominant_axis_direction_preserving_component_cap',
        smoothing=dict(startup_transition_s=1.,fraction_acceleration_per_s2=2.))
    for axis in ('planar','yaw'):
        mapping[axis].update(rise_rate_per_s=.6,fall_rate_per_s=.6)
    return record


def test_observation_is_not_a_physical_velocity_calibration(tmp_path):
    record=profile()
    assert record['parameter_basis']==OPERATOR_BASIS
    assert validate_runtime_profile(record)['physical_acceptance_verified'] is False
    assert validate_sdk_input_mapping(record)==record['sdk_input_mapping']
    assert record['provenance']['operator_observation']['quantitative_velocity_measurement'] is False
    assert record['sdk_input_mapping']['planar']['effective_fraction']==pytest.approx(.30*2**(1/16))
    assert record['sdk_input_mapping']['yaw']['effective_fraction']==.20
    assert record['sdk_input_mapping']['yaw']['maximum_fraction']==.30
    assert record['provenance']['operator_observation']['yaw_update']['below_working_fraction']=='little_or_no_turning_response'
    assert record['provenance']['operator_observation']['yaw_update']['quantitative_angular_rate_measurement'] is False
    assert record['sdk_input_mapping']['planar']['maximum_fraction']==.50
    assert record['provenance']['operator_observation']['planar_input_ceiling_update']['basis']=='operator_requested_sdk_fraction_not_metres_per_second'
    path=tmp_path/'profile.json';path.write_text(json.dumps(record))
    model=load_model(path,hashlib.sha256(path.read_bytes()).hexdigest(),'live')
    assert model['parameter_basis']==OPERATOR_BASIS
    assert model['physical_acceptance_verified'] is False
    stationary=validate_stationary(record)
    assert validate_stationary({'stationary_evidence':stationary})==stationary


@pytest.mark.parametrize('axis,key,value',[
    ('planar','effective_fraction',0),('planar','maximum_fraction',.500001),
    ('planar','maximum_fraction',.19),('yaw','effective_fraction',True),
    ('yaw','blend_demand_ratio',0),('yaw','blend_demand_ratio',1),
    ('planar','rise_rate_per_s',float('nan')),('yaw','fall_rate_per_s',float('inf')),
    ('planar','rise_rate_per_s',0),('yaw','fall_rate_per_s',1.1)])
def test_bad_numerical_mapping_not_silently_ignored(axis,key,value):
    record=profile();record['sdk_input_mapping'][axis][key]=value
    with pytest.raises(ValueError,match='sdk_input_mapping'):validate_runtime_profile(record)


@pytest.mark.parametrize('change',[
    lambda r:r.pop('sdk_input_mapping'),
    lambda r:r['sdk_input_mapping'].update(kind='other'),
    lambda r:r['sdk_input_mapping'].update(schema_version=True),
    lambda r:r['sdk_input_mapping'].update(normalization='per_axis_floor'),
    lambda r:r['sdk_input_mapping']['planar'].update(unused=1),
    lambda r:r.update(parameter_basis='vendor_nominal_and_engineering')])
def test_versioned_writer_contract_required(change):
    record=profile();change(record)
    with pytest.raises(ValueError,match='sdk_input_mapping'):validate_runtime_profile(record)


def test_legacy_profile_still_has_legacy_mapping():
    record=profile();record['parameter_basis']='vendor_nominal_and_engineering'
    record.pop('sdk_input_mapping')
    assert validate_sdk_input_mapping(record) is None
    assert validate_runtime_profile(record)['parameter_basis']==record['parameter_basis']


def test_v4_planar_ceiling_is_not_an_increase_of_yaw_legacy_or_si_speed_limits():
    record=profile()
    assert validate_sdk_input_mapping(record)['planar']['maximum_fraction']==.50
    assert record['measurements']['max_speed_mps']==.30
    assert record['sdk_input_mapping']['yaw']['maximum_fraction']==.30
    record['sdk_input_mapping']['yaw']['maximum_fraction']=.400001
    with pytest.raises(ValueError,match='sdk_input_mapping_bounds:yaw'):
        validate_sdk_input_mapping(record)
    legacy=legacy_v3_profile()
    assert validate_sdk_input_mapping(legacy)['planar']['maximum_fraction']==.40
    legacy['sdk_input_mapping']['planar']['maximum_fraction']=.50
    with pytest.raises(ValueError,match='sdk_input_mapping_bounds:planar'):
        validate_sdk_input_mapping(legacy)


def test_continuous_steering_profile_has_one_command_conditioner_not_inertia_filters():
    tracking=load_tracking(CONFIG/'motion_tracking.yaml')
    assert tracking['controller_frequency_hz']==20
    assert tracking['position_gain']==.8 and tracking['yaw_gain']==1.2
    assert tracking['velocity_damping_gain']==0. and tracking['yaw_damping_gain']==.15
    assert tracking['max_acceleration_mps2']==.35
    assert tracking['max_yaw_acceleration_radps2']==.8
    assert tracking['max_jerk_mps3']>0. and tracking['max_yaw_jerk_radps3']>0.
    assert tracking['steering_error_scale_rad']>0.


def test_continuous_mapping_has_no_hidden_time_or_inertia_state():
    record=profile()
    mapping=record['sdk_input_mapping']
    assert (mapping['schema_version'],mapping['kind'],mapping['normalization'])==(
        4,'continuous_amplitude_ratio_v4','rounded_dominant_pnorm16_direction_preserving_component_cap')
    assert 'smoothing' not in mapping
    for axis in ('planar','yaw'):
        assert set(mapping[axis])=={'effective_fraction','maximum_fraction','blend_demand_ratio'}
    assert validate_sdk_input_mapping(record)==mapping


def test_v4_old_normalization_token_is_not_silently_given_new_math():
    record=profile()
    record['sdk_input_mapping']['normalization']='dominant_axis_direction_preserving_component_cap'
    with pytest.raises(ValueError,match='sdk_input_mapping_contract'):
        validate_sdk_input_mapping(record)


def test_old_v1_keeps_its_own_fraction_cap():
    record=legacy_v3_profile();mapping=record['sdk_input_mapping']
    mapping.pop('smoothing')
    mapping.update(schema_version=1,kind='smooth_deadzone_ratio_v1',
                   normalization='dominant_axis_direction_preserving_norm_cap')
    with pytest.raises(ValueError,match='sdk_input_mapping_bounds'):
        validate_runtime_profile(record)
    for axis in ('planar','yaw'):
        mapping[axis].update(effective_fraction=.20,maximum_fraction=.30)
    assert validate_sdk_input_mapping(record)==mapping


@pytest.mark.parametrize('change',[
    lambda s:s.update(startup_transition_s=True),
    lambda s:s.update(startup_transition_s=0),
    lambda s:s.update(startup_transition_s=.1),
    lambda s:s.update(startup_transition_s=3),
    lambda s:s.update(fraction_acceleration_per_s2=1),
    lambda s:s.update(fraction_acceleration_per_s2=float('nan')),
    lambda s:s.update(extra=1),
    lambda s:s.pop('startup_transition_s')])
def test_optional_v3_smoothing_is_strict_and_respects_startup_derivatives(change):
    record=legacy_v3_profile();change(record['sdk_input_mapping']['smoothing'])
    with pytest.raises(ValueError,match='sdk_input_(smoothing|startup)'):
        validate_runtime_profile(record)


def test_optional_smoothing_cannot_change_legacy_version_semantics():
    record=legacy_v3_profile();mapping=record['sdk_input_mapping']
    assert mapping['smoothing']==dict(startup_transition_s=1.,fraction_acceleration_per_s2=2.)
    assert validate_sdk_input_mapping(record)==mapping
    mapping.update(schema_version=2,kind='smooth_deadzone_ratio_v2')
    with pytest.raises(ValueError,match='sdk_input_smoothing'):validate_sdk_input_mapping(record)
    mapping.pop('smoothing');assert validate_sdk_input_mapping(record)==mapping


@pytest.mark.parametrize('change',[
    lambda m:m.update(smoothing=dict(startup_transition_s=1.,fraction_acceleration_per_s2=2.)),
    lambda m:m['planar'].update(rise_rate_per_s=.6),
    lambda m:m['yaw'].update(fall_rate_per_s=.6)])
def test_default_mapping_rejects_legacy_filter_fields(change):
    record=profile();change(record['sdk_input_mapping'])
    with pytest.raises(ValueError,match='sdk_input_'):validate_sdk_input_mapping(record)
