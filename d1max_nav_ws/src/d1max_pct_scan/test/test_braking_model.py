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
