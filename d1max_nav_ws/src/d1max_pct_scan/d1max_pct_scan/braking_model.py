"""Read the exact model record used by native sweep validation and the SDK."""
import hashlib
import json
import math
from pathlib import Path

MODEL='reaction_braking_reachable_v1'
FIELDS=('max_speed_mps','max_yaw_radps','reaction_bound_s','stopping_distance_m',
        'stopping_yaw_rad','stop_latency_bound_s','tracking_error_bound_m','heading_error_bound_rad')


def platform_model_limits(record, mode):
    """Separate command authority from a bounded isolated plant's motion."""
    values=record['measurements']
    if 'isolated_platform_model' not in record:
        return dict(speed_cap=.30,yaw_cap=.50,
            command_max_speed_mps=values['max_speed_mps'],
            command_max_yaw_radps=values['max_yaw_radps'])
    model=record['isolated_platform_model']
    keys={'schema','kind','command_max_speed_mps','command_max_yaw_radps',
        'reachable_max_speed_mps','reachable_max_yaw_radps','source_scope'}
    if (mode!='isolated_mock' or record.get('transport_mode')!='isolated_mock'
            or record.get('fixture_only') is not True or not isinstance(model,dict)
            or set(model)!=keys or type(model['schema']) is not int or model['schema']!=1
            or model['kind']!='official_spot_physx'
            or model['source_scope']!='isolated_simulation_physx_measured_model'):
        raise ValueError('invalid_isolated_platform_model')
    for field,cap in (('command_max_speed_mps',.30),('command_max_yaw_radps',.50),
            ('reachable_max_speed_mps',.60),('reachable_max_yaw_radps',.80)):
        value=model[field]
        if type(value) not in (int,float) or not math.isfinite(value) or not 0<value<=cap:
            raise ValueError('invalid_isolated_platform_bound:'+field)
    if (model['reachable_max_speed_mps']!=values['max_speed_mps']
            or model['reachable_max_yaw_radps']!=values['max_yaw_radps']
            or model['command_max_speed_mps']>model['reachable_max_speed_mps']
            or model['command_max_yaw_radps']>model['reachable_max_yaw_radps']):
        raise ValueError('isolated_platform_measurement_mismatch')
    return dict(speed_cap=.60,yaw_cap=.80,
        command_max_speed_mps=model['command_max_speed_mps'],
        command_max_yaw_radps=model['command_max_yaw_radps'])


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
    if ('isolated_platform_model' in record or 'isolated_full_xyz_reference_model' in record) and mode!='isolated_mock':
        raise ValueError('isolated_platform_cannot_authorize_real_robot')
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
    limits=platform_model_limits(record,mode)
    if values['max_speed_mps']>limits['speed_cap'] or values['max_yaw_radps']>limits['yaw_cap']:
        raise ValueError('first_acceptance_motion_limit_exceeded')
    if (values['reaction_bound_s']>1. or values['stopping_distance_m']>1.
            or values['stop_latency_bound_s']>3.
            or values['stopping_yaw_rad']>1. or values['tracking_error_bound_m']>.25
            or values['heading_error_bound_rad']>.5):
        raise ValueError('native_reachable_model_bound_exceeded')
    return dict(values,command_max_speed_mps=limits['command_max_speed_mps'],
        command_max_yaw_radps=limits['command_max_yaw_radps'])
