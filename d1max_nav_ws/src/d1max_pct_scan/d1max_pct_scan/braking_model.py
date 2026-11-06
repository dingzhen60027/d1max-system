"""Read the exact bound parameters used by native sweep and the SDK.

Runtime-profile numerical slots retain the legacy ``measurements`` spelling
for compatibility; their configured provenance is not physical acceptance.
"""
import hashlib
import json
import math
from pathlib import Path

MODEL='reaction_braking_reachable_v1'
FIELDS=('max_speed_mps','max_yaw_radps','reaction_bound_s','stopping_distance_m',
        'stopping_yaw_rad','stop_latency_bound_s','tracking_error_bound_m','heading_error_bound_rad')


def load_model(path,expected_sha256,mode):
    path=Path(path)
    if not path.is_absolute():raise ValueError('absolute_braking_record_required')
    with path.open('rb') as stream:
        raw=stream.read(1024*1024+1)
    if not raw or len(raw)>1024*1024:
        raise ValueError('braking_record_size_invalid')
    if hashlib.sha256(raw).hexdigest()!=expected_sha256:
        raise ValueError('braking_record_hash_mismatch')
    record=json.loads(raw)
    if not isinstance(record,dict):raise ValueError('braking_record_object_required')
    from .execution_timing import (RUNTIME_KIND,validate_runtime_profile,
        validate_timing,validate_stationary)
    runtime=record.get('record_kind')==RUNTIME_KIND
    provenance={}
    if mode=='isolated_mock':
        if runtime:raise ValueError('runtime_motion_profile_requires_live_transport')
        if (record.get('schema_version')!=3 or record.get('transport_mode')!=mode
                or record.get('fixture_only') is not True or record.get('model')!=MODEL):
            raise ValueError('isolated_fixture_braking_model_required')
    elif mode=='live':
        if record.get('fixture_only') or record.get('transport_mode')=='isolated_mock':
            raise ValueError('fixture_cannot_authorize_real_robot')
        if runtime:
            provenance=validate_runtime_profile(record)
        else:
            # Existing physical records retain their independent evidence and
            # SDK identity checks. A runtime profile never calls this gate.
            from .single_floor_session import check_acceptance
            check_acceptance(path)
    else:raise ValueError('explicit_braking_transport_required')
    values=record['measurements']
    if not isinstance(values,dict):raise ValueError('braking_parameters_object_required')
    profile=record.get('kinematic_profile','forward_yaw')
    if profile not in ('forward_yaw','holonomic_xyyaw'):
        raise ValueError('invalid_braking_kinematic_profile')
    if profile=='holonomic_xyyaw':
        lateral=values.get('stopping_lateral_distance_m')
        if (isinstance(lateral,bool) or not isinstance(lateral,(int,float))
                or not math.isfinite(lateral) or not 0 < lateral <= 1.):
            raise ValueError('invalid_braking_bound:stopping_lateral_distance_m')
        scale=values.get('lateral_metres_per_fraction')
        if (isinstance(scale,bool) or not isinstance(scale,(int,float))
                or not math.isfinite(scale) or scale<=0):
            raise ValueError('invalid_lateral_metres_per_fraction')
    validate_timing(record)
    validate_stationary(record)
    if mode=='live' and values.get('model')!=MODEL:
        raise ValueError('runtime_reachable_model_missing' if runtime else 'physical_reachable_model_missing')
    for field in FIELDS:
        value=values.get(field)
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or value<=0:
            raise ValueError('invalid_braking_bound:'+field)
    if values['max_speed_mps']>.30 or values['max_yaw_radps']>.50:
        raise ValueError('first_acceptance_motion_limit_exceeded')
    if runtime:
        for field,minimum in (('forward_scale_mps',values['max_speed_mps']),
                              ('yaw_scale_radps',values['max_yaw_radps'])):
            value=values.get(field)
            if (type(value) not in (int,float) or not math.isfinite(value) or value<minimum):
                raise ValueError('invalid_runtime_speed_mapping:'+field)
    if profile=='holonomic_xyyaw' and scale<values['max_speed_mps']:
        raise ValueError('invalid_lateral_metres_per_fraction')
    if (values['reaction_bound_s']>1. or values['stopping_distance_m']>1.
            or values['stop_latency_bound_s']>3.
            or values['stopping_yaw_rad']>1. or values['tracking_error_bound_m']>.25
            or values['heading_error_bound_rad']>.5):
        raise ValueError('native_reachable_model_bound_exceeded')
    return dict(values,**provenance) if runtime else values
