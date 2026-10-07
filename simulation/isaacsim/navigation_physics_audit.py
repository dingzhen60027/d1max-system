"""Offline audit of sealed normal-navigation native 500 Hz measurements.

The result covers recorded PhysX states, not hardware or continuous collision
freedom. It never reconstructs an intervening state from ROS publications.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import struct
from pathlib import Path

DT_NS = 2_000_000
RAW_FILE = 'policy_component_state_500hz.jsonl'


def _require(condition, reason):
    if not condition:
        raise ValueError(reason)


def _integer(value, minimum=0):
    return type(value) is int and value >= minimum


def _vector(row, name, length):
    value = row.get(name)
    _require(isinstance(value, list) and len(value) == length
        and all(type(v) in (int, float) and math.isfinite(v) for v in value),
        'invalid_native_vector:' + name)
    return value


def _yaw(quaternion):
    w, x, y, z = quaternion
    # Normalize only the orientation calculation, retaining original evidence.
    norm = math.hypot(w, x, y, z)
    _require(norm > 0., 'invalid_native_quaternion')
    w, x, y, z = (v / norm for v in quaternion)
    return math.atan2(2. * (w*z + x*y), 1. - 2. * (y*y + z*z))


def _domain(rows, anchor, bounds, yaw_bound):
    peak = max(rows, key=lambda r: math.hypot(*r['linear_velocity_world']))
    speed = math.hypot(*peak['linear_velocity_world'])
    yaw_peak = max(abs(r['angular_velocity_world'][2]) for r in rows)
    return dict(sample_count=len(rows), max_full_xyz_speed_mps=speed,
        peak_source_ns=anchor+peak['source_ns'], peak_source_tick=peak['source_tick'],
        peak_raw_linear_velocity_world=peak['linear_velocity_world'],
        max_abs_world_yaw_rate_radps=yaw_peak,
        max_full_angular_speed_radps=max(math.hypot(*r['angular_velocity_world']) for r in rows),
        full_angular_norm_is_diagnostic_only=True, speed_bounds=bounds,
        yaw_bound_radps=yaw_bound,
        exceeded_speed_domains=[name for name, limit in bounds.items() if speed > limit],
        yaw_domain_exceeded=yaw_peak > yaw_bound)


def _final_stop(rows, anchor, measurements, policy):
    # END observes the last real step; its command was not a new consumption.
    begin = rows[:-1]
    nonzero = [i for i, row in enumerate(begin) if any(v != 0. for v in row['highlevel_command'])]
    result = dict(verified=False, scope='final actual zero-consumption episode; native full XYZ and angular norm',
        transition_exercised=bool(nonzero), linear_threshold_mps=policy['linear_threshold_mps'],
        angular_threshold_radps=policy['angular_threshold_radps'],
        stationary_duration_s=policy['stationary_duration_s'],
        stop_latency_bound_s=measurements['stop_latency_bound_s'],
        stopping_distance_bound_m=measurements['stopping_distance_m'],
        stopping_yaw_bound_rad=measurements['stopping_yaw_rad'])
    if not nonzero or nonzero[-1]+1 >= len(begin):
        return dict(result, reason='no_final_zero_consumption_after_nonzero')
    first_zero = nonzero[-1]+1
    origin = rows[first_zero]
    result.update(last_nonzero_consumption_source_ns=anchor+begin[nonzero[-1]]['source_ns'],
        zero_consumption_source_ns=anchor+origin['source_ns'])
    duration_ns = round(policy['stationary_duration_s']*1e9)
    stationary_start = None
    confirmation = None
    post_confirmation_motion_samples = 0
    first_post_confirmation_motion = None
    xy_path = yaw_path = 0.
    previous = origin
    previous_yaw = _yaw(previous['quaternion_wxyz'])
    for row in rows[first_zero:]:
        xy_path += math.hypot(row['position'][0]-previous['position'][0], row['position'][1]-previous['position'][1])
        yaw = _yaw(row['quaternion_wxyz'])
        yaw_path += abs(math.atan2(math.sin(yaw-previous_yaw), math.cos(yaw-previous_yaw)))
        previous, previous_yaw = row, yaw
        low = (math.hypot(*row['linear_velocity_world']) <= policy['linear_threshold_mps']
            and math.hypot(*row['angular_velocity_world']) <= policy['angular_threshold_radps'])
        if confirmation is not None and not low:
            post_confirmation_motion_samples += 1
            if first_post_confirmation_motion is None:
                first_post_confirmation_motion = anchor+row['source_ns']
        stationary_start = (row['source_ns'] if stationary_start is None else stationary_start) if low else None
        if confirmation is None and stationary_start is not None and row['source_ns']-stationary_start >= duration_ns:
            latency = (row['source_ns']-origin['source_ns'])/1e9
            passed = (latency <= measurements['stop_latency_bound_s']
                and xy_path <= measurements['stopping_distance_m'] and yaw_path <= measurements['stopping_yaw_rad'])
            confirmation = dict(result, verified=passed, reason='recorded_stop_within_original_bounds' if passed else 'recorded_stop_outside_original_bounds',
                stationary_window_start_source_ns=anchor+stationary_start,
                confirmation_source_ns=anchor+row['source_ns'], confirmation_latency_s=latency,
                extra_xy_path_m=xy_path, extra_absolute_yaw_path_rad=yaw_path,
                actual_zero_tail_duration_s=(rows[-1]['source_ns']-origin['source_ns'])/1e9)
    if confirmation is not None:
        confirmation.update(initial_stationary_window_verified=confirmation['verified'],
            maintained_stationarity_to_end=post_confirmation_motion_samples==0,
            post_confirmation_motion_samples=post_confirmation_motion_samples,
            first_post_confirmation_motion_source_ns=first_post_confirmation_motion,
            zero_tail_total_xy_path_m=xy_path, zero_tail_total_absolute_yaw_path_rad=yaw_path)
        if post_confirmation_motion_samples:
            confirmation.update(verified=False,reason='recorded_stop_then_zero_tail_motion')
        return confirmation
    return dict(result, reason='final_zero_tail_has_no_complete_stationary_window',
        extra_xy_path_m=xy_path, extra_absolute_yaw_path_rad=yaw_path,
        actual_zero_tail_duration_s=(rows[-1]['source_ns']-origin['source_ns'])/1e9)


def audit_navigation_physics(session, scene_bytes, model_bytes, manifest, raw_bytes,
                             *, expected_epoch, action_interval_ns=None):
    """Return ``verified``/``errors`` without changing any raw record.

    ``action_interval_ns`` is an optional inclusive pair of original *anchored*
    source stamps. The final stop remains a separately named full-epoch gate.
    The caller supplies epoch from the original physics ready.json, not a row.
    """
    result = dict(schema=1, kind='sealed_normal_navigation_500hz_physics_audit',
        verified=False, errors=[], scope='isolated_simulation_native_PhysX_500Hz_BEGIN_and_actual_final_END',
        velocity_domain_scope='instantaneous full XYZ admission; not a future full XYZ reachability guarantee',
        hardware_acceptance=False, continuous_collision_claim=False,
        raw_records_modified=False, raw_sha256=hashlib.sha256(raw_bytes).hexdigest())
    try:
        from d1max_pct_scan.braking_model import FIELDS, reference_model_limits, platform_model_limits
        from d1max_pct_scan.execution_timing import validate_timing, validate_stationary
        scene, model = json.loads(scene_bytes), json.loads(model_bytes)
        _require(isinstance(session, dict) and isinstance(scene, dict) and isinstance(model, dict)
            and isinstance(manifest, dict), 'audit_input_not_object')
        _require(session.get('simulation_backend') == 'isaacsim_physx'
            and session.get('simulation_clock') == 'isaac_fixed_anchor_v1'
            and session.get('transport_mode') == 'isolated_mock'
            and session.get('physical_acceptance') is False
            and isinstance(session.get('id'), str) and bool(session['id'])
            and model.get('session_id') == session['id'], 'isolated_navigation_session_mismatch')
        bridge = session['isaac_bridge_contract']
        anchor = bridge.get('clock_anchor_ns')
        _require(_integer(anchor, 1) and isinstance(expected_epoch, str) and bool(expected_epoch)
            and bridge.get('source_clock') == 'isaac_physics_time', 'navigation_clock_or_epoch_invalid')
        inputs = session.get('input_hashes')
        model_path, scene_path = session.get('execution_braking_model_record'), bridge.get('scene_config')
        _require(isinstance(inputs, dict) and isinstance(model_path, str) and Path(model_path).is_absolute()
            and isinstance(scene_path, str) and Path(scene_path).is_absolute(), 'absolute_sealed_inputs_required')
        model_sha, scene_sha = hashlib.sha256(model_bytes).hexdigest(), hashlib.sha256(scene_bytes).hexdigest()
        _require(0 < len(model_bytes) <= 1024*1024
            and model_sha == session.get('execution_braking_model_sha256') == inputs.get(model_path), 'sealed_model_hash_mismatch')
        _require(scene_sha == bridge.get('scene_sha256') == inputs.get(scene_path)
            == manifest.get('scene_sha256'), 'sealed_scene_hash_mismatch')
        robot = scene.get('robot', {})
        _require(robot.get('record_full_physics_history') is True
            and robot.get('kind') == 'official_spot_physx', 'explicit_normal_navigation_full_history_required')
        limits = reference_model_limits(model, 'isolated_mock')
        _require(limits is not None, 'sealed_full_xyz_reference_marker_required')
        platform = platform_model_limits(model, 'isolated_mock')
        validate_timing(model)
        policy = validate_stationary(model)
        _require(session.get('stationary_evidence') == model['stationary_evidence'], 'session_stationary_record_mismatch')
        _require((policy['linear_threshold_mps'], policy['angular_threshold_radps'], policy['stationary_duration_s'])
            == (.03, .05, 1.), 'original_stop_window_contract_required')
        values = model['measurements']
        _require(all(type(values.get(name)) in (int, float) and math.isfinite(values[name]) and values[name] > 0 for name in FIELDS), 'invalid_braking_measurements')
        for name, cap in (('reaction_bound_s', 1.), ('stopping_distance_m', 1.), ('stopping_yaw_rad', 1.),
                ('stop_latency_bound_s', 3.), ('tracking_error_bound_m', .25), ('heading_error_bound_rad', .5)):
            _require(values[name] <= cap, 'native_braking_bound_exceeded:'+name)
        commands = [platform['command_max_speed_mps'], platform['command_max_yaw_radps']]
        _require(commands == [session.get('max_speed_mps'), session.get('max_yaw_radps')]
            == [robot.get('max_linear_speed'), robot.get('max_angular_speed')]
            == manifest.get('command_limits'), 'sealed_command_authority_mismatch')
        n = manifest.get('actual_source_ticks')
        _require(_integer(n, 1) and type(manifest.get('schema')) is int and manifest.get('schema') == 1
            and manifest.get('kind') == 'actual_500hz_pre_policy_history'
            and manifest.get('physics_hz') == 500 and manifest.get('policy_hz') == 50
            and manifest.get('source_tick_ns') == DT_NS and manifest.get('duration_ns') == n*DT_NS
            and manifest.get('first_source_tick') == 0 and manifest.get('last_source_tick') == n-1
            and manifest.get('consecutive_tick_invocations_checked') is True
            and manifest.get('component_measurement_count') == n+1
            and manifest.get('component_last_source_ns') == n*DT_NS
            and manifest.get('component_measurements_file') == RAW_FILE
            and manifest.get('component_measurements_sha256') == result['raw_sha256']
            and all(_integer(manifest.get(k)) for k in ('physics_hz', 'policy_hz', 'source_tick_ns',
                'duration_ns', 'first_source_tick', 'last_source_tick', 'component_measurement_count',
                'component_last_source_ns', 'observation_count')), 'native_history_manifest_or_hash_mismatch')
        rows = [json.loads(line) for line in raw_bytes.splitlines() if line.strip()]
        _require(len(rows) == n+1, 'native_history_count_mismatch')
        first_native = rows[0].get('native_physics_tick')
        first_counter = rows[0].get('policy_counter')
        _require(_integer(first_native) and _integer(first_counter), 'native_first_counter_invalid')
        input_limits = robot.get('policy_input_limits', {})
        _require(all(type(input_limits.get(k)) in (int, float) and math.isfinite(input_limits[k])
            and 0 < input_limits[k] <= cap for k, cap in (('linear', .6), ('angular', .5))),
            'sealed_policy_input_limits_missing')
        # The original official input tensor is float32. Compare its exact
        # representation of the sealed cap, not an arbitrary epsilon.
        input_bounds = {k: struct.unpack('f', struct.pack('f', input_limits[k]))[0]
            for k in ('linear', 'angular')}
        for i, row in enumerate(rows):
            _require(isinstance(row, dict) and type(row.get('schema')) is int and row.get('schema') == 1
                and row.get('kind') == 'actual_full_body_component_measurement'
                and row.get('phase') == 'source'
                and _integer(row.get('source_tick')) and row['source_tick'] == i
                and _integer(row.get('source_ns')) and row['source_ns'] == i*DT_NS
                and _integer(row.get('native_physics_tick')) and row['native_physics_tick'] == first_native+i
                and _integer(row.get('policy_counter')) and row['policy_counter'] == first_counter+i
                and row.get('session_id') == session['id'] and row.get('epoch') == expected_epoch
                and row.get('clock_anchor_ns') == anchor, 'native_tick_or_identity_mismatch:'+str(i))
            _require(row.get('acquisition_phase') == ('actual_final_PhysX_END' if i == n else 'actual_pre_policy_PhysX_BEGIN'), 'native_phase_mismatch:'+str(i))
            if i < n:
                _require(row.get('mode') == 'navigation_wire' and isinstance(row.get('authority'), dict)
                    and row['authority'].get('policy_input_override') is None
                    and row['authority'].get('replay_original_authority') is None, 'normal_navigation_consumption_required:'+str(i))
            for name, count in (('position', 3), ('quaternion_wxyz', 4), ('linear_velocity_world', 3),
                    ('angular_velocity_world', 3), ('official_policy_input', 3), ('highlevel_command', 2)):
                _vector(row, name, count)
            _yaw(row['quaternion_wxyz'])
            _require(all(abs(v) <= bound for v, bound in zip(row['highlevel_command'], commands)), 'actual_highlevel_command_outside_seal')
            internal = row['official_policy_input']
            _require(abs(internal[0]) <= input_bounds['linear'] and internal[1] == 0.
                and abs(internal[2]) <= input_bounds['angular'], 'actual_policy_input_outside_seal')
            for name, vector in (('full_linear_speed', 'linear_velocity_world'), ('full_angular_speed', 'angular_velocity_world')):
                _require(type(row.get(name)) in (int, float) and math.isfinite(row[name])
                    and row[name] == math.hypot(*row[vector]), 'native_derived_speed_mismatch:'+name)
        bounds = dict(reachable_max_speed_mps=values['max_speed_mps'],
            reference_max_speed_mps=limits['reference_max_speed_mps'],
            measured_travel_max_speed_mps=limits['measured_travel_max_speed_mps'])
        result.update(session_id=session['id'], epoch=expected_epoch, clock_anchor_ns=anchor,
            scene_sha256=scene_sha, model_sha256=model_sha, actual_source_ticks=n,
            duration_ns=n*DT_NS, sample_count=len(rows), complete_native_500hz=True,
            native_first_tick=first_native, native_final_tick=first_native+n,
            policy_first_counter=first_counter, policy_final_counter=first_counter+n,
            manifest_policy_hz=manifest['policy_hz'], manifest_observation_count=manifest['observation_count'],
            observation_count_scope='original count includes bootstrap; not a reconstructed source-only inference gate',
            all_epoch=_domain(rows, anchor, bounds, values['max_yaw_radps']))
        if action_interval_ns is not None:
            _require(isinstance(action_interval_ns, (tuple, list)) and len(action_interval_ns) == 2
                and all(_integer(v, anchor) for v in action_interval_ns)
                and action_interval_ns[0] <= action_interval_ns[1] <= anchor+n*DT_NS, 'action_interval_source_invalid')
            selected = [r for r in rows if action_interval_ns[0] <= anchor+r['source_ns'] <= action_interval_ns[1]]
            _require(bool(selected), 'action_interval_has_no_native_samples')
            result['action_interval'] = dict(source_interval_ns=list(action_interval_ns), **_domain(selected, anchor, bounds, values['max_yaw_radps']))
        for name in ('all_epoch', 'action_interval'):
            domain = result.get(name)
            if domain and domain['exceeded_speed_domains']:
                result['errors'].append(name+':measured_full_xyz_speed_outside_sealed_domain')
            if domain and domain['yaw_domain_exceeded']:
                result['errors'].append(name+':measured_world_yaw_rate_outside_sealed_domain')
        result['final_stop'] = _final_stop(rows, anchor, values, policy)
        if not result['final_stop']['verified']:
            result['errors'].append('final_stop:'+result['final_stop']['reason'])
        result['verified'] = not result['errors']
    except (ValueError, TypeError, KeyError, IndexError, AttributeError, OverflowError) as error:
        result['errors'].append(str(error))
    return result


def audit_navigation_physics_files(session_path, *, action_interval_ns=None):
    """Read original sealed paths and original ready epoch from a run folder."""
    path = Path(session_path)
    try:
        session = json.loads(path.read_bytes())
        physics = path.parent/'physics'
        return audit_navigation_physics(session,
            Path(session['isaac_bridge_contract']['scene_config']).read_bytes(),
            Path(session['execution_braking_model_record']).read_bytes(),
            json.loads((physics/'policy_history_manifest.json').read_bytes()),
            (physics/RAW_FILE).read_bytes(),
            expected_epoch=json.loads((physics/'ready.json').read_bytes())['epoch'],
            action_interval_ns=action_interval_ns)
    except (OSError, ValueError, KeyError, TypeError) as error:
        return dict(schema=1, kind='sealed_normal_navigation_500hz_physics_audit',
            verified=False, errors=[str(error)], hardware_acceptance=False, continuous_collision_claim=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', required=True, type=Path)
    parser.add_argument('--action-source-start-ns', type=int)
    parser.add_argument('--action-source-end-ns', type=int)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    interval = None
    if args.action_source_start_ns is not None or args.action_source_end_ns is not None:
        interval = (args.action_source_start_ns, args.action_source_end_ns)
    result = audit_navigation_physics_files(args.session, action_interval_ns=interval)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    return 0 if result['verified'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
