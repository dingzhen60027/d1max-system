import hashlib
import json
import pytest
from d1max_pct_scan.braking_model import load_model, reference_model_limits


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


def spot_record(tmp_path):
    path,_=record(tmp_path)
    obj=json.loads(path.read_text())
    obj['measurements'].update(max_speed_mps=.6,max_yaw_radps=.8,
        stopping_distance_m=.5,stop_latency_bound_s=3.)
    obj['isolated_platform_model']=dict(schema=1,kind='official_spot_physx',
        command_max_speed_mps=.15,command_max_yaw_radps=.30,
        reachable_max_speed_mps=.6,reachable_max_yaw_radps=.8,
        source_scope='isolated_simulation_physx_measured_model')
    return path,obj


def write_record(path,obj):
    raw=json.dumps(obj).encode();path.write_bytes(raw)
    return hashlib.sha256(raw).hexdigest()


def reference_record(tmp_path,cap=.6,observed=.532):
    path,obj=spot_record(tmp_path)
    obj['isolated_full_xyz_reference_model']=dict(schema=1,kind='official_spot_physx',
        source_scope='isolated_simulation_physx_measured_model',reference_max_speed_mps=cap,
        measured_travel_max_speed_mps=cap,observed_max_full_xyz_speed_mps=observed,evidence_sha256='b'*64)
    return path,obj


@pytest.mark.parametrize('observed',[.508039,.532,.6])
def test_explicit_xyz_domain_up_to_platform_reach_does_not_change_command_or_braking(tmp_path,observed):
    path,obj=reference_record(tmp_path,observed=observed)
    values=load_model(path,write_record(path,obj),'isolated_mock')
    assert reference_model_limits(obj,'isolated_mock')==dict(reference_max_speed_mps=.6,
        measured_travel_max_speed_mps=.6,observed_max_full_xyz_speed_mps=observed)
    assert values['max_speed_mps']==.6 and values['max_yaw_radps']==.8
    assert values['command_max_speed_mps']==.15 and values['command_max_yaw_radps']==.3
    assert values['stop_latency_bound_s']==3. and values['stopping_distance_m']==.5
    with pytest.raises(ValueError,match='cannot_authorize_real_robot'):
        load_model(path,write_record(path,obj),'live')


def test_absent_xyz_marker_preserves_legacy_without_inferring_platform_reach(tmp_path):
    path,obj=spot_record(tmp_path)
    assert reference_model_limits(obj,'isolated_mock') is None
    assert load_model(path,write_record(path,obj),'isolated_mock')['command_max_speed_mps']==.15


@pytest.mark.parametrize('field,value',[
    ('reference_max_speed_mps',.600001),('measured_travel_max_speed_mps',.600001),
    ('observed_max_full_xyz_speed_mps',.600001),('reference_max_speed_mps',0.),
    ('measured_travel_max_speed_mps',True),('observed_max_full_xyz_speed_mps',float('nan')),
    ('reference_max_speed_mps','0.6'),('schema',1.),('kind','official_go2_physx'),
    ('source_scope','live'),('evidence_sha256','B'*64),('extra',True)])
def test_xyz_reference_model_numbers_exact_marker_and_evidence_are_required(tmp_path,field,value):
    path,obj=reference_record(tmp_path);obj['isolated_full_xyz_reference_model'][field]=value
    with pytest.raises(ValueError):load_model(path,write_record(path,obj),'isolated_mock')


@pytest.mark.parametrize('field',['reference_max_speed_mps','measured_travel_max_speed_mps',
    'observed_max_full_xyz_speed_mps','evidence_sha256','schema','kind','source_scope'])
def test_incomplete_xyz_model_never_falls_back_to_legacy(tmp_path,field):
    path,obj=reference_record(tmp_path);obj['isolated_full_xyz_reference_model'].pop(field)
    with pytest.raises(ValueError):load_model(path,write_record(path,obj),'isolated_mock')


def test_xyz_domains_must_fit_same_hashed_reach_and_both_observation_and_command(tmp_path):
    path,obj=reference_record(tmp_path)
    obj['isolated_platform_model']['reachable_max_speed_mps']=.55;obj['measurements']['max_speed_mps']=.55
    with pytest.raises(ValueError,match='invalid_isolated_reference_bound'):
        load_model(path,write_record(path,obj),'isolated_mock')
    for field in ('reference_max_speed_mps','measured_travel_max_speed_mps'):
        path,obj=reference_record(tmp_path);obj['isolated_full_xyz_reference_model'][field]=.53
        with pytest.raises(ValueError,match='evidence_domain_invalid'):
            load_model(path,write_record(path,obj),'isolated_mock')
        path,obj=reference_record(tmp_path,observed=.1);obj['isolated_full_xyz_reference_model'][field]=.14
        with pytest.raises(ValueError,match='evidence_domain_invalid'):
            load_model(path,write_record(path,obj),'isolated_mock')


@pytest.mark.parametrize('observed',[.616161,.65])
def test_calibrated_dot65_record_is_exact_hashed_isolated_domain(tmp_path,observed):
    path,obj=reference_record(tmp_path,cap=.65,observed=observed)
    obj['measurements']['max_speed_mps']=.65
    obj['isolated_platform_model']['reachable_max_speed_mps']=.65
    obj['isolated_platform_model'].update(command_max_speed_mps=.30,command_max_yaw_radps=.50)
    digest=write_record(path,obj)
    values=load_model(path,digest,'isolated_mock')
    assert values['max_speed_mps']==.65
    assert values['command_max_speed_mps']==.30 and values['command_max_yaw_radps']==.50
    assert values['stopping_distance_m']==.5 and values['stop_latency_bound_s']==3.
    assert reference_model_limits(obj,'isolated_mock')['observed_max_full_xyz_speed_mps']==observed
    with pytest.raises(ValueError,match='cannot_authorize_real_robot'):
        load_model(path,digest,'live')
    with pytest.raises(ValueError,match='hash_mismatch'):
        load_model(path,'a'*64,'isolated_mock')


@pytest.mark.parametrize('field',[
    'reference_max_speed_mps','measured_travel_max_speed_mps','observed_max_full_xyz_speed_mps'])
def test_calibrated_dot65_reference_cap_remains_strict(tmp_path,field):
    import math
    path,obj=reference_record(tmp_path,cap=.65,observed=.616161)
    obj['measurements']['max_speed_mps']=.65
    obj['isolated_platform_model']['reachable_max_speed_mps']=.65
    obj['isolated_full_xyz_reference_model'][field]=math.nextafter(.65,math.inf)
    with pytest.raises(ValueError,match='invalid_isolated_reference_bound'):
        load_model(path,write_record(path,obj),'isolated_mock')


def test_old_dot60_record_cannot_accept_new_observation_or_reuse_old_hash(tmp_path):
    path,obj=reference_record(tmp_path,observed=.6)
    old_digest=write_record(path,obj)
    obj['isolated_full_xyz_reference_model']['observed_max_full_xyz_speed_mps']=.616161
    new_digest=write_record(path,obj)
    with pytest.raises(ValueError,match='hash_mismatch'):
        load_model(path,old_digest,'isolated_mock')
    with pytest.raises(ValueError,match='invalid_isolated_reference_bound'):
        load_model(path,new_digest,'isolated_mock')


@pytest.mark.parametrize('mutation', ['reach_over_cap','measurement_mismatch','invalid_evidence','fixture','transport'])
def test_calibrated_dot65_marker_measurement_and_evidence_guards(tmp_path,mutation):
    import math
    path,obj=reference_record(tmp_path,cap=.65,observed=.616161)
    obj['measurements']['max_speed_mps']=.65
    obj['isolated_platform_model']['reachable_max_speed_mps']=.65
    if mutation=='reach_over_cap':
        obj['measurements']['max_speed_mps']=math.nextafter(.65,math.inf)
        obj['isolated_platform_model']['reachable_max_speed_mps']=obj['measurements']['max_speed_mps']
    elif mutation=='measurement_mismatch':obj['measurements']['max_speed_mps']=.6
    elif mutation=='invalid_evidence':obj['isolated_full_xyz_reference_model']['evidence_sha256']=''
    elif mutation=='fixture':obj['fixture_only']=False
    else:obj['transport_mode']='live'
    with pytest.raises(ValueError):load_model(path,write_record(path,obj),'isolated_mock')


@pytest.mark.parametrize('change',['platform','fixture','transport','schema_float'])
def test_xyz_model_requires_original_platform_and_exact_isolated_fixture(tmp_path,change):
    path,obj=reference_record(tmp_path)
    if change=='platform':obj.pop('isolated_platform_model')
    elif change=='fixture':obj['fixture_only']=False
    elif change=='transport':obj['transport_mode']='live'
    else:obj['schema_version']=3.0
    with pytest.raises(ValueError):load_model(path,write_record(path,obj),'isolated_mock')


def test_spot_actual_motion_is_separate_from_command_authority(tmp_path):
    path,obj=spot_record(tmp_path)
    values=load_model(path,write_record(path,obj),'isolated_mock')
    assert values['max_speed_mps']==.6 and values['max_yaw_radps']==.8
    assert values['command_max_speed_mps']==.15 and values['command_max_yaw_radps']==.30
    assert values['stop_latency_bound_s']==3. and values['stopping_distance_m']==.5
    with pytest.raises(ValueError,match='cannot_authorize_real_robot'):
        load_model(path,write_record(path,obj),'live')
    obj['fixture_only']=False;obj['transport_mode']='live'
    with pytest.raises(ValueError,match='cannot_authorize_real_robot'):
        load_model(path,write_record(path,obj),'live')


def test_new_reference_marker_alone_cannot_authorize_live(tmp_path):
    path,obj=spot_record(tmp_path)
    obj.pop('isolated_platform_model')
    obj['isolated_full_xyz_reference_model']={'schema':1,'kind':'official_spot_physx'}
    obj['fixture_only']=False;obj['transport_mode']='live'
    with pytest.raises(ValueError,match='cannot_authorize_real_robot'):
        load_model(path,write_record(path,obj),'live')


@pytest.mark.parametrize('field,value',[
    ('schema',True),('schema',1.),('schema',2),('kind','official_go2_physx'),
    ('source_scope','live'),('command_max_speed_mps',.301),('command_max_yaw_radps',.501),
    ('command_max_speed_mps',0.),('command_max_yaw_radps',True),
    ('reachable_max_speed_mps',.601),('reachable_max_yaw_radps',.801),
    ('reachable_max_speed_mps',.59),('reachable_max_yaw_radps',.79),
    ('reachable_max_speed_mps',float('nan')),('reachable_max_yaw_radps','0.8')])
def test_spot_model_identity_numeric_bounds_and_measurement_match_are_required(tmp_path,field,value):
    path,obj=spot_record(tmp_path);obj['isolated_platform_model'][field]=value
    with pytest.raises(ValueError):load_model(path,write_record(path,obj),'isolated_mock')


def test_spot_model_cannot_relax_stop_timing_or_legacy_default(tmp_path):
    path,obj=spot_record(tmp_path)
    for field,value in [('stop_latency_bound_s',3.01),('stopping_distance_m',1.01),
            ('reaction_bound_s',1.01),('tracking_error_bound_m',.251)]:
        saved=obj['measurements'][field];obj['measurements'][field]=value
        with pytest.raises(ValueError):load_model(path,write_record(path,obj),'isolated_mock')
        obj['measurements'][field]=saved
    for field in tuple(obj['isolated_platform_model']):
        saved=obj['isolated_platform_model'].pop(field)
        with pytest.raises(ValueError):load_model(path,write_record(path,obj),'isolated_mock')
        obj['isolated_platform_model'][field]=saved
    obj['isolated_platform_model']['extra_permission']=True
    with pytest.raises(ValueError):load_model(path,write_record(path,obj),'isolated_mock')
    obj.pop('isolated_platform_model')
    with pytest.raises(ValueError,match='first_acceptance_motion_limit'):
        load_model(path,write_record(path,obj),'isolated_mock')
