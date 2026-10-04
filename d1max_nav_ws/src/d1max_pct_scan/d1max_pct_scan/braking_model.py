"""Read the exact model record used by native sweep validation and the SDK."""
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
    if mode=='isolated_mock':
        if (record.get('schema_version')!=3 or record.get('transport_mode')!=mode
                or record.get('fixture_only') is not True or record.get('model')!=MODEL):
            raise ValueError('isolated_fixture_braking_model_required')
    elif mode=='live':
        if record.get('fixture_only') or record.get('transport_mode')=='isolated_mock':
            raise ValueError('fixture_cannot_authorize_real_robot')
        from .single_floor_session import check_acceptance
        check_acceptance(path)
    else:raise ValueError('explicit_braking_transport_required')
    values=record['measurements']
    from .execution_timing import validate_timing,validate_stationary
    validate_timing(record)
    validate_stationary(record)
    if mode=='live' and values.get('model')!=MODEL:
        raise ValueError('physical_reachable_model_missing')
    for field in FIELDS:
        value=values.get(field)
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or value<=0:
            raise ValueError('invalid_braking_bound:'+field)
    if values['max_speed_mps']>.30 or values['max_yaw_radps']>.50:
        raise ValueError('first_acceptance_motion_limit_exceeded')
    if (values['reaction_bound_s']>1. or values['stopping_distance_m']>1.
            or values['stop_latency_bound_s']>3.
            or values['stopping_yaw_rad']>1. or values['tracking_error_bound_m']>.25
            or values['heading_error_bound_rad']>.5):
        raise ValueError('native_reachable_model_bound_exceeded')
    return values
