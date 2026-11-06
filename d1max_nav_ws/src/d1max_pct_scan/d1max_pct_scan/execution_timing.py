"""One reaction budget for perception, validation and the unique SDK writer."""
import math

FIELDS=('sensor_source_age_bound_s','command_pipeline_bound_s',
        'writer_period_s','source_time_uncertainty_s')
RUNTIME_KIND='runtime_motion_profile'
RUNTIME_BASIS='vendor_nominal_and_engineering'
OPERATOR_BASIS='vendor_nominal_engineering_and_operator_observation'
SDK_INPUT_KIND='smooth_deadzone_ratio_v1'
SDK_INPUT_KIND_V2='smooth_deadzone_ratio_v2'
SDK_INPUT_KIND_V3='direct_breakaway_ratio_v3'
SDK_INPUT_KIND_V4='continuous_amplitude_ratio_v4'


def validate_sdk_input_mapping(record):
    """SDK fractions are not measured body velocities or braking evidence.

    This version is explicitly understood by the writer and its checker.
    Old binaries reject the new basis instead of ignoring a new JSON field.
    """
    mapping=record.get('sdk_input_mapping')
    observed=record.get('parameter_basis')==OPERATOR_BASIS
    if not observed:
        if 'sdk_input_mapping' in record:
            raise ValueError('sdk_input_mapping_requires_operator_basis')
        return None
    if (not isinstance(mapping,dict)
            or set(mapping) not in ({'schema_version','kind','normalization','planar','yaw'},
                                   {'schema_version','kind','normalization','planar','yaw','smoothing'})
            or type(mapping.get('schema_version')) is not int
            or (mapping['schema_version'],mapping.get('kind'),mapping.get('normalization')) not in (
                (1,SDK_INPUT_KIND,'dominant_axis_direction_preserving_norm_cap'),
                (2,SDK_INPUT_KIND_V2,'dominant_axis_direction_preserving_component_cap'),
                (3,SDK_INPUT_KIND_V3,'dominant_axis_direction_preserving_component_cap'),
                (4,SDK_INPUT_KIND_V4,'rounded_dominant_pnorm16_direction_preserving_component_cap'))):
        raise ValueError('invalid_sdk_input_mapping_contract')
    keys={'effective_fraction','maximum_fraction','blend_demand_ratio'}
    if mapping['schema_version']<4:
        keys|={'rise_rate_per_s','fall_rate_per_s'}
    for axis in ('planar','yaw'):
        # The current V4 planar response may use the operator-requested 50%
        # ceiling. Do not silently change yaw or archived V1--V3 contracts.
        fraction_cap=(.50 if mapping['schema_version']==4 and axis=='planar'
                      else .40 if mapping['schema_version']>=2 else .30)
        values=mapping.get(axis)
        if not isinstance(values,dict) or set(values)!=keys:
            raise ValueError('invalid_sdk_input_mapping_axis:'+axis)
        if any(type(v) not in (int,float) or not math.isfinite(v) for v in values.values()):
            raise ValueError('invalid_sdk_input_mapping_number:'+axis)
        if not (0<values['effective_fraction']<=values['maximum_fraction']<=fraction_cap
                and 0<values['blend_demand_ratio']<1):
            raise ValueError('invalid_sdk_input_mapping_bounds:'+axis)
        if mapping['schema_version']<4 and not (
                0<values['rise_rate_per_s']<=1 and 0<values['fall_rate_per_s']<=1):
            raise ValueError('invalid_sdk_input_mapping_bounds:'+axis)
    if 'smoothing' in mapping:
        smoothing=mapping['smoothing']
        if (mapping['schema_version']!=3 or not isinstance(smoothing,dict)
                or set(smoothing)!={'startup_transition_s','fraction_acceleration_per_s2'}
                or any(type(v) not in (int,float) or not math.isfinite(v) for v in smoothing.values())):
            raise ValueError('invalid_sdk_input_smoothing_contract')
        duration=smoothing['startup_transition_s'];acceleration=smoothing['fraction_acceleration_per_s2']
        if not (0<duration<=2 and 0<acceleration<=4):
            raise ValueError('invalid_sdk_input_smoothing_bounds')
        for axis in ('planar','yaw'):
            values=mapping[axis]
            if (1.875*values['effective_fraction']/duration>values['rise_rate_per_s']+1e-12
                    or 5.774*values['effective_fraction']/duration**2>acceleration+1e-12):
                raise ValueError('invalid_sdk_input_startup_derivative_bounds')
    return mapping


def runtime_profile_provenance(record):
    """Validate configured-parameter provenance, not physical acceptance."""
    if (record.get('record_kind')!=RUNTIME_KIND
            or record.get('parameter_basis') not in (RUNTIME_BASIS,OPERATOR_BASIS)
            or record.get('physical_acceptance_verified') is not False):
        raise ValueError('invalid_runtime_motion_profile_provenance')
    return dict(record_kind=RUNTIME_KIND,parameter_basis=record['parameter_basis'],
        physical_acceptance_verified=False)


def validate_runtime_profile(record):
    """Validate the live runtime record's structure without an evidence gate.

    The calibration SHA is a wire-compatible identity slot. For this record
    kind it identifies the vendor document, not a physical calibration.
    """
    provenance=runtime_profile_provenance(record)
    if (type(record.get('schema_version')) is not int or record['schema_version']!=3
            or record.get('profile')!='general_low_speed'
            or record.get('fixture_only') is not False
            or record.get('transport_mode')!='live'):
        raise ValueError('invalid_runtime_motion_profile_contract')
    for key in ('robot_id','sdk_version'):
        if not isinstance(record.get(key),str) or not record[key].strip():
            raise ValueError('invalid_runtime_motion_profile_identity:'+key)
    for key in ('calibration_sha256','robot_profile_sha256'):
        value=record.get(key)
        if (not isinstance(value,str) or len(value)!=64
                or any(c not in '0123456789abcdef' for c in value)):
            raise ValueError('invalid_runtime_motion_profile_identity:'+key)
    # A configured profile must not inherit old assertions of verification.
    for key in ('speed_mapping_verified','stop_timing_verified',
                'body_envelope_verified','raw_ray_safety_verified'):
        if key in record and record[key] is not False:
            raise ValueError('runtime_motion_profile_cannot_claim_verification:'+key)
    validate_sdk_input_mapping(record)
    return provenance


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
    """Mirror the hashed SDK stop policy; never a permission boolean.

    Runtime bounds are configured, not measured. Keep their provenance and
    configured names even when supplying the existing internal numeric slots.
    """
    policy=record.get('stationary_evidence')
    if not isinstance(policy,dict) or policy.get('profile')!='general_low_speed':
        raise ValueError('stationary_evidence_contract_missing')
    if policy.get('velocity_metric','spatial_norm_3d') not in ('spatial_norm_3d','planar_xy_with_vertical_guard_v1'):
        raise ValueError('invalid_stationary_evidence:velocity_metric')
    runtime_source=record if record.get('record_kind')==RUNTIME_KIND else policy
    runtime=runtime_source.get('record_kind')==RUNTIME_KIND
    provenance=runtime_profile_provenance(runtime_source) if runtime else {}
    result=dict(policy,**provenance)
    if runtime:
        for axis,unit in (('linear','mps'),('angular','radps')):
            configured=f'configured_static_{axis}_bound_{unit}'
            compatible=f'measured_static_{axis}_bound_{unit}'
            # Revalidation by ExecutionSafety receives the normalized policy.
            # Do not let a conflicting legacy slot override its source value.
            if compatible in policy and policy[compatible]!=policy.get(configured):
                raise ValueError('stationary_compatibility_bound_mismatch:'+compatible)
            result[compatible]=policy.get(configured)
    bounds=dict(linear_threshold_mps=(.001,.05),angular_threshold_radps=(.001,.1),
        stationary_duration_s=(1.,5.),reentry_duration_s=(.6,5.),
        mc_expected_hz=(10.,200.),mc_min_hz=(10.,200.),mc_max_hz=(10.,300.),
        measured_static_linear_bound_mps=(0.,.05),measured_static_angular_bound_radps=(0.,.1))
    for name,(lo,hi) in bounds.items():
        value=result.get(name)
        if type(value) not in (int,float) or not math.isfinite(value) or not lo<=value<=hi:
            source_name=name.replace('measured_static_','configured_static_') if runtime else name
            raise ValueError('invalid_stationary_evidence:'+source_name)
    count=policy.get('minimum_new_samples')
    if type(count) is not int or not 3<=count<=512:
        raise ValueError('invalid_stationary_evidence:minimum_new_samples')
    if (result['measured_static_linear_bound_mps']>policy['linear_threshold_mps']
            or result['measured_static_angular_bound_radps']>policy['angular_threshold_radps']
            or not policy['mc_min_hz']<=policy['mc_expected_hz']<=policy['mc_max_hz']):
        raise ValueError('stationary_noise_or_frequency_outside_record')
    return result


def stationary_velocity_within(e,policy):
    """Same original MC norm/component contract as SDK and BT; never a permit."""
    raw=e.measured_linear_mps;angular=e.measured_angular_radps
    configured=policy.get('velocity_metric','spatial_norm_3d')
    if (configured not in ('spatial_norm_3d','planar_xy_with_vertical_guard_v1')
            or not math.isfinite(raw) or raw<0 or not math.isfinite(angular)
            or not 0<=angular<=policy['angular_threshold_radps']):return False
    metric=getattr(e,'velocity_metric','')
    if metric in ('','spatial_norm_3d'):return raw<=policy['linear_threshold_mps']
    if metric!='planar_xy_with_vertical_guard_v1' or metric!=configured:return False
    planar=e.measured_planar_mps;vertical=e.measured_vertical_mps
    return (math.isfinite(planar) and math.isfinite(vertical) and planar>=0 and vertical>=0
        and abs(math.hypot(planar,vertical)-raw)<=1e-6
        and max(planar,vertical)<=policy['linear_threshold_mps'])
