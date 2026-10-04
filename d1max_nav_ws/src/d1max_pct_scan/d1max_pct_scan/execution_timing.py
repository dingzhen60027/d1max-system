"""One reaction budget for perception, validation and the unique SDK writer."""
import math

FIELDS=('sensor_source_age_bound_s','command_pipeline_bound_s',
        'writer_period_s','source_time_uncertainty_s')


def validate_timing(record, *, sensor_timeout_s=None, writer_period_s=.05):
    timing=record.get('execution_timing')
    if not isinstance(timing,dict):
        raise ValueError('execution_timing_contract_missing')
    for name in FIELDS:
        v=timing.get(name)
        bounds={'sensor_source_age_bound_s':(.001,.6),'command_pipeline_bound_s':(.1,.5),
                'writer_period_s':(.05,.05),'source_time_uncertainty_s':(0.,.25)}
        lo,hi=bounds[name]
        if type(v) not in (int,float) or not math.isfinite(v) or not lo<=v<=hi:
            raise ValueError('invalid_execution_timing:'+name)
    if timing['sensor_source_age_bound_s']<=0 or timing['command_pipeline_bound_s']<=0:
        raise ValueError('execution_timing_bounds_must_be_positive')
    if abs(timing['writer_period_s']-writer_period_s)>1e-9:
        raise ValueError('writer_period_does_not_match_execution_record')
    reaction=record.get('measurements',{}).get('reaction_bound_s')
    required=sum(timing[name] for name in FIELDS)
    if (type(reaction) not in (int,float) or not math.isfinite(reaction)
            or reaction+1e-12<required):
        raise ValueError('reaction_bound_does_not_cover_execution_pipeline')
    if sensor_timeout_s is not None and (type(sensor_timeout_s) not in (int,float)
            or not math.isfinite(sensor_timeout_s) or not 0<sensor_timeout_s<=timing['sensor_source_age_bound_s']):
        raise ValueError('sensor_source_age_exceeds_execution_record')
    return dict(timing,required_reaction_bound_s=required)


def validate_stationary(record):
    """Mirror the hashed SDK measurement policy; never a permission boolean."""
    policy=record.get('stationary_evidence')
    if not isinstance(policy,dict) or policy.get('profile')!='general_low_speed':
        raise ValueError('stationary_evidence_contract_missing')
    bounds=dict(linear_threshold_mps=(.001,.05),angular_threshold_radps=(.001,.1),
        stationary_duration_s=(1.,5.),reentry_duration_s=(.6,5.),
        mc_expected_hz=(10.,200.),mc_min_hz=(10.,200.),mc_max_hz=(10.,300.),
        measured_static_linear_bound_mps=(0.,.05),measured_static_angular_bound_radps=(0.,.1))
    for name,(lo,hi) in bounds.items():
        value=policy.get(name)
        if type(value) not in (int,float) or not math.isfinite(value) or not lo<=value<=hi:
            raise ValueError('invalid_stationary_evidence:'+name)
    count=policy.get('minimum_new_samples')
    if type(count) is not int or not 3<=count<=512:
        raise ValueError('invalid_stationary_evidence:minimum_new_samples')
    if (policy['measured_static_linear_bound_mps']>policy['linear_threshold_mps']
            or policy['measured_static_angular_bound_radps']>policy['angular_threshold_radps']
            or not policy['mc_min_hz']<=policy['mc_expected_hz']<=policy['mc_max_hz']):
        raise ValueError('stationary_noise_or_frequency_outside_record')
    return dict(policy)
