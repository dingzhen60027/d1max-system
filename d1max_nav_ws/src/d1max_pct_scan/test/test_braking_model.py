import hashlib
import json
import pytest
from d1max_pct_scan.braking_model import load_model


def record(tmp_path,**changes):
    obj=dict(schema_version=3,transport_mode='isolated_mock',fixture_only=True,
        execution_timing=dict(sensor_source_age_bound_s=.2,command_pipeline_bound_s=.1,
            writer_period_s=.05,source_time_uncertainty_s=.02),
        stationary_evidence=dict(profile='general_low_speed',linear_threshold_mps=.03,angular_threshold_radps=.05,
            stationary_duration_s=1.,reentry_duration_s=.6,minimum_new_samples=3,
            mc_expected_hz=50.,mc_min_hz=40.,mc_max_hz=60.,
            measured_static_linear_bound_mps=.01,measured_static_angular_bound_radps=.02),
        model='reaction_braking_reachable_v1',measurements=dict(max_speed_mps=.3,
        max_yaw_radps=.5,reaction_bound_s=.4,stopping_distance_m=.08,stopping_yaw_rad=.2,
        stop_latency_bound_s=.5,tracking_error_bound_m=.01,heading_error_bound_rad=.03))
    obj.update(changes)
    path=tmp_path/'fixture.json';raw=json.dumps(obj).encode();path.write_bytes(raw)
    return path,hashlib.sha256(raw).hexdigest()


def test_fixture_is_file_bound_and_never_live_acceptance(tmp_path):
    path,digest=record(tmp_path)
    assert load_model(path,digest,'isolated_mock')['stopping_distance_m']==.08
    with pytest.raises(ValueError,match='fixture_cannot'):load_model(path,digest,'live')
    with pytest.raises(ValueError,match='hash_mismatch'):load_model(path,'a'*64,'isolated_mock')


@pytest.mark.parametrize('field,value',[('stopping_yaw_rad',0.),('tracking_error_bound_m',None),
    ('reaction_bound_s',float('nan')),('max_speed_mps',.31),
    ('tracking_error_bound_m',.27),('stopping_yaw_rad',1.01),('heading_error_bound_rad',.51)])
def test_missing_nonfinite_or_unlimited_model_never_authorizes(tmp_path,field,value):
    path,_=record(tmp_path);obj=json.loads(path.read_text());obj['measurements'][field]=value
    raw=json.dumps(obj).encode();path.write_bytes(raw)
    with pytest.raises(ValueError):load_model(path,hashlib.sha256(raw).hexdigest(),'isolated_mock')


def test_no_expiry_or_reaction_budget_guessing(tmp_path):
    from d1max_pct_scan.execution_timing import validate_timing
    path,_=record(tmp_path);obj=json.loads(path.read_text())
    assert validate_timing(obj,sensor_timeout_s=.2)['required_reaction_bound_s']==pytest.approx(.37)
    with pytest.raises(ValueError,match='source_age_exceeds'):validate_timing(obj,sensor_timeout_s=.5)
    obj['execution_timing']['sensor_source_age_bound_s']=.5
    with pytest.raises(ValueError,match='reaction_bound'):validate_timing(obj)
    del obj['execution_timing']
    with pytest.raises(ValueError,match='contract_missing'):validate_timing(obj)


@pytest.mark.parametrize('distance',[None,0.,True,float('nan'),1.01])
def test_holonomic_model_cannot_reuse_unmeasured_forward_stop(tmp_path,distance):
    path,_=record(tmp_path,kinematic_profile='holonomic_xyyaw')
    obj=json.loads(path.read_text());obj['measurements']['stopping_lateral_distance_m']=distance
    raw=json.dumps(obj).encode();path.write_bytes(raw)
    with pytest.raises(ValueError,match='stopping_lateral_distance_m'):
        load_model(path,hashlib.sha256(raw).hexdigest(),'isolated_mock')


def test_holonomic_fixture_has_distinct_lateral_braking_bound(tmp_path):
    path,_=record(tmp_path,kinematic_profile='holonomic_xyyaw')
    obj=json.loads(path.read_text());obj['measurements'].update(stopping_lateral_distance_m=.06,lateral_metres_per_fraction=1.)
    raw=json.dumps(obj).encode();path.write_bytes(raw)
    assert load_model(path,hashlib.sha256(raw).hexdigest(),'isolated_mock')['stopping_lateral_distance_m']==.06


def test_stationary_thresholds_are_bound_to_measured_noise_and_mc_rate(tmp_path):
    from d1max_pct_scan.execution_timing import validate_stationary
    path,_=record(tmp_path);obj=json.loads(path.read_text());p=obj['stationary_evidence']
    p.update(linear_threshold_mps=.04,measured_static_linear_bound_mps=.038,
        mc_expected_hz=100.,mc_min_hz=80.,mc_max_hz=120.)
    assert validate_stationary(obj)['linear_threshold_mps']==.04
    p['measured_static_linear_bound_mps']=.041
    with pytest.raises(ValueError,match='noise_or_frequency'):validate_stationary(obj)
    p['measured_static_linear_bound_mps']=.038;p['minimum_new_samples']=True
    with pytest.raises(ValueError,match='minimum_new_samples'):validate_stationary(obj)
    del obj['stationary_evidence']
    with pytest.raises(ValueError,match='contract_missing'):validate_stationary(obj)


def runtime_record(tmp_path,**changes):
    path,_=record(tmp_path)
    obj=json.loads(path.read_text())
    obj.update(record_kind='runtime_motion_profile',parameter_basis='vendor_nominal_and_engineering',
        physical_acceptance_verified=False,fixture_only=False,transport_mode='live',
        profile='general_low_speed',kinematic_profile='holonomic_xyyaw',robot_id='D1-Max',
        sdk_version='0.1.0-charging_v2',calibration_sha256='a'*64,robot_profile_sha256='b'*64)
    obj['measurements'].update(model='reaction_braking_reachable_v1',forward_scale_mps=1.,
        lateral_metres_per_fraction=1.,yaw_scale_radps=2.,stopping_lateral_distance_m=.08)
    policy=obj['stationary_evidence']
    policy['configured_static_linear_bound_mps']=policy.pop('measured_static_linear_bound_mps')
    policy['configured_static_angular_bound_radps']=policy.pop('measured_static_angular_bound_radps')
    policy.update(mc_expected_hz=100.,mc_min_hz=10.,mc_max_hz=250.)
    obj.update(changes)
    return obj


def write_record(tmp_path,obj):
    path=tmp_path/'model.json';raw=json.dumps(obj).encode();path.write_bytes(raw)
    return path,hashlib.sha256(raw).hexdigest()


def test_live_runtime_uses_exact_structural_profile_not_physical_checker(tmp_path,monkeypatch):
    import sys
    from types import SimpleNamespace
    def physical_check_must_not_run(*args):
        pytest.fail('runtime parameters are not a physical acceptance request')
    monkeypatch.setitem(sys.modules,'d1max_pct_scan.single_floor_session',
        SimpleNamespace(check_acceptance=physical_check_must_not_run))
    path,digest=write_record(tmp_path,runtime_record(tmp_path))
    values=load_model(path,digest,'live')
    assert values['stopping_lateral_distance_m']==.08
    assert values['record_kind']=='runtime_motion_profile'
    assert values['parameter_basis']=='vendor_nominal_and_engineering'
    assert values['physical_acceptance_verified'] is False
    with pytest.raises(ValueError,match='hash_mismatch'):load_model(path,'0'*64,'live')
    with pytest.raises(ValueError,match='requires_live'):load_model(path,digest,'isolated_mock')


@pytest.mark.parametrize('field,value',[
    ('schema_version',3.),('fixture_only',None),('fixture_only',True),
    ('physical_acceptance_verified',True),('physical_acceptance_verified',0),
    ('parameter_basis','measured'),('profile','high_speed'),('transport_mode','other'),
    ('robot_id',''),('sdk_version',None),('calibration_sha256','bad'),
    ('robot_profile_sha256','A'*64),('speed_mapping_verified',True),
    ('stop_timing_verified',True),('body_envelope_verified',True),('raw_ray_safety_verified',True)])
def test_runtime_structural_or_provenance_failure_never_loads_live(tmp_path,field,value):
    obj=runtime_record(tmp_path);obj[field]=value
    path,digest=write_record(tmp_path,obj)
    with pytest.raises(ValueError):load_model(path,digest,'live')


@pytest.mark.parametrize('field,value',[
    ('model','wrong'),('forward_scale_mps',True),('forward_scale_mps',.29),
    ('yaw_scale_radps',None),('yaw_scale_radps',.49),('lateral_metres_per_fraction',.29),
    ('stopping_lateral_distance_m',None),('reaction_bound_s',.3),
    ('tracking_error_bound_m',.26)])
def test_runtime_numerical_compatibility_slots_keep_old_limits(tmp_path,field,value):
    obj=runtime_record(tmp_path);obj['measurements'][field]=value
    path,digest=write_record(tmp_path,obj)
    with pytest.raises(ValueError):load_model(path,digest,'live')


def test_legacy_live_record_still_invokes_physical_checker(tmp_path,monkeypatch):
    import sys
    from types import SimpleNamespace
    calls=[]
    def checked(path):
        calls.append(path)
        raise ValueError('legacy_physical_evidence_missing')
    monkeypatch.setitem(sys.modules,'d1max_pct_scan.single_floor_session',SimpleNamespace(check_acceptance=checked))
    obj=runtime_record(tmp_path);obj.pop('record_kind')
    path,digest=write_record(tmp_path,obj)
    with pytest.raises(ValueError,match='legacy_physical_evidence_missing'):load_model(path,digest,'live')
    assert calls==[path]


def test_runtime_stationary_configured_bounds_preserve_source_through_revalidation(tmp_path):
    from d1max_pct_scan.execution_timing import validate_stationary
    from d1max_pct_scan.execution_safety import ExecutionSafety
    obj=runtime_record(tmp_path)
    policy=validate_stationary(obj)
    assert policy['configured_static_linear_bound_mps']==policy['measured_static_linear_bound_mps']==.01
    assert policy['configured_static_angular_bound_radps']==policy['measured_static_angular_bound_radps']==.02
    assert policy['record_kind']=='runtime_motion_profile'
    assert policy['parameter_basis']=='vendor_nominal_and_engineering'
    assert policy['physical_acceptance_verified'] is False
    assert validate_stationary({'stationary_evidence':policy})==policy
    core=ExecutionSafety('runtime-session','live',braking_model_sha256='c'*64,
        kinematic_profile='holonomic_xyyaw',stationary_policy=policy)
    assert core.stationary_policy==policy
    # Configured names must remain present, even for a normalized internal policy.
    policy.pop('configured_static_linear_bound_mps')
    with pytest.raises(ValueError,match='compatibility_bound_mismatch'):
        validate_stationary({'stationary_evidence':policy})


@pytest.mark.parametrize('field,value',[
    ('configured_static_linear_bound_mps',.031),('configured_static_linear_bound_mps',None),
    ('configured_static_angular_bound_radps',float('nan')),('configured_static_angular_bound_radps',True),
    ('measured_static_linear_bound_mps',.009),('mc_min_hz',110.),('minimum_new_samples',True)])
def test_runtime_stationary_invalid_or_conflicting_bound_rejected(tmp_path,field,value):
    from d1max_pct_scan.execution_timing import validate_stationary
    obj=runtime_record(tmp_path);obj['stationary_evidence'][field]=value
    with pytest.raises(ValueError):validate_stationary(obj)


def test_configured_stationary_names_do_not_replace_legacy_measurements(tmp_path):
    from d1max_pct_scan.execution_timing import validate_stationary
    obj=runtime_record(tmp_path);obj.pop('record_kind')
    with pytest.raises(ValueError,match='measured_static_linear_bound_mps'):validate_stationary(obj)


def test_checked_in_runtime_defaults_load_without_physical_evidence():
    from pathlib import Path
    from d1max_pct_scan.execution_timing import validate_stationary
    path=Path(__file__).resolve().parents[1]/'config'/'runtime_motion_profile.json'
    values=load_model(path,hashlib.sha256(path.read_bytes()).hexdigest(),'live')
    assert values['physical_acceptance_verified'] is False
    assert values['max_speed_mps']==.3 and values['max_yaw_radps']==.5
    policy=validate_stationary(json.loads(path.read_text()))
    assert policy['mc_expected_hz']==100. and policy['mc_max_hz']==250.


def test_runtime_timing_reserves_delivery_without_increasing_total_reaction_budget():
    from pathlib import Path
    from d1max_pct_scan.execution_timing import validate_timing
    path=Path(__file__).resolve().parents[1]/'config'/'runtime_motion_profile.json'
    record=json.loads(path.read_text())
    timing=validate_timing(record,sensor_timeout_s=.6)
    assert timing['sensor_source_age_bound_s']==.6
    # The local body contract already limits every command to 100 ms. The
    # old nominal 200 ms command allocation was never usable by the writer.
    assert timing['command_pipeline_bound_s']==.1
    assert timing['required_reaction_bound_s']==pytest.approx(1.)
    assert record['measurements']['reaction_bound_s']==1.
    assert record['physical_acceptance_verified'] is False
    for source in (.342,.444,.501,.546,.557):
        remaining=timing['sensor_source_age_bound_s']-source
        assert remaining>0.
    # The writer still needs its ORIGINAL body/command lease as well as rays;
    # the larger sensor-stage allocation does not grant a 600 ms command.
    assert timing['command_pipeline_bound_s']<=.1
