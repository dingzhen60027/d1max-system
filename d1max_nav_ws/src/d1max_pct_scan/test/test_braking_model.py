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
