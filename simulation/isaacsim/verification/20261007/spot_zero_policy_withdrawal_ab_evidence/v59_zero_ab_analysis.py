#!/usr/bin/env python3
"""Private read-only v59 frozen-command delayed-v3 comparison.

No state interpolation, runtime mutation, replay launch or acceptance override.
Exact input withdrawal and measured physical stop are separate results.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import math
import struct
import sys
from pathlib import Path

DT = 2_000_000
FROZEN_SHA = '6a9d175c01b09708fddf7327c08e7732a7b97790fcb66e320c65e12cf64f466c'
ORIGINAL_SHA = '23eac6beca478fe6e5b0497f37000c6d5367ebe3701c55b5674fc703c27bbbd1'
BASELINE_SHA = '4ae9721022718d5358ccc4b2a82d79c1cf7ca145c037d9e4a8f220eb4b4ab895'
SIX = ('position', 'quaternion_wxyz', 'linear_velocity_world',
       'angular_velocity_world', 'official_policy_input', 'highlevel_command')


def require(ok, why):
    if not ok:
        raise ValueError(why)


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda: f.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def load(path):
    return json.loads(Path(path).read_bytes())


def read_rows(path):
    h, rows = hashlib.sha256(), []
    with Path(path).open('rb') as f:
        for line in f:
            h.update(line)
            require(bool(line.strip()), 'blank_jsonl_line:' + str(path))
            row = json.loads(line)
            require(isinstance(row, dict), 'non_object_row:' + str(path))
            rows.append(row)
    return rows, h.hexdigest()


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def vector(row, key, size):
    v = row.get(key)
    require(isinstance(v, list) and len(v) == size and all(finite(x) for x in v), 'invalid_vector:' + key)
    return v


def identity(row, ctx):
    return all(row.get(k) == ctx[k] for k in ('session_id', 'epoch', 'clock_anchor_ns'))


def difference(a, b, fields, count):
    require(len(a) >= count and len(b) >= count, 'comparison_prefix_missing')
    first = None
    by_field = dict.fromkeys(fields, 0)
    for i in range(count):
        for field in fields:
            if a[i].get(field) != b[i].get(field):
                by_field[field] += 1
                if first is None:
                    first = dict(row_index=i, source_tick=a[i].get('source_tick'),
                        source_ns=a[i].get('source_ns'), field=field,
                        baseline=a[i].get(field), alternative=b[i].get(field))
    return dict(sample_count=count, fields=list(fields), all_fields_exactly_equal=first is None,
        different_row_counts_by_field=by_field, first_difference=first)


def canonical_hash(rows):
    h = hashlib.sha256()
    for row in rows:
        h.update(json.dumps(row, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()+b'\n')
    return h.hexdigest()


def witness(row):
    return {k: row.get(k) for k in ('source_tick','source_ns','native_physics_tick',
        'policy_counter','acquisition_phase', *SIX)}


def audit_run(path, *, ctx_base, scene, scene_sha, frozen_sha, frozen, n, activation, alt,
              domain_bounds, yaw_bound, measurements, stationary, normal=False):
    manifest = load(path/'policy_history_manifest.json')
    ready, summary = load(path/'ready.json'), load(path/'summary.json')
    ctx = dict(ctx_base, epoch=ready['epoch'])
    count = manifest.get('actual_source_ticks')
    require(type(count) is int and count > 0 and (count == n or normal), 'actual_tick_count_mismatch')
    require(manifest.get('schema') == 1 and manifest.get('kind') == 'actual_500hz_pre_policy_history'
        and manifest.get('scene_sha256') == scene_sha and manifest.get('physics_hz') == 500
        and manifest.get('policy_hz') == 50 and manifest.get('source_tick_ns') == DT
        and manifest.get('duration_ns') == count*DT and manifest.get('first_source_tick') == 0
        and manifest.get('last_source_tick') == count-1
        and manifest.get('consecutive_tick_invocations_checked') is True
        and manifest.get('component_measurement_count') == count+1
        and manifest.get('component_last_source_ns') == count*DT
        and manifest.get('component_measurements_file') == 'policy_component_state_500hz.jsonl', 'manifest_500hz_invalid')
    limits = [scene['robot']['max_linear_speed'], scene['robot']['max_angular_speed']]
    require(manifest.get('command_limits') == limits, 'manifest_command_limits_mismatch')
    rows, raw_sha = read_rows(path/'policy_component_state_500hz.jsonl')
    require(len(rows) == count+1 and raw_sha == manifest.get('component_measurements_sha256'), 'raw_count_or_hash_mismatch')
    if normal:
        require(raw_sha == ORIGINAL_SHA, 'original_v59_raw_hash_mismatch')
    elif not alt:
        require(raw_sha == BASELINE_SHA, 'baseline_raw_hash_mismatch')
    first_native, first_counter = rows[0].get('native_physics_tick'), rows[0].get('policy_counter')
    require(type(first_native) is int and type(first_counter) is int
        and first_native == 1010 and first_counter == 1000, 'original_bootstrap_counter_changed')
    input_limits = scene['robot']['policy_input_limits']
    inp_bounds = [struct.unpack('f', struct.pack('f', input_limits[k]))[0] for k in ('linear', 'angular')]
    mode = 'navigation_wire' if normal else 'isolated_frozen_commands'
    for i, row in enumerate(rows):
        require(row.get('schema') == 1 and row.get('kind') == 'actual_full_body_component_measurement'
            and row.get('phase') == 'source' and type(row.get('source_tick')) is int
            and row['source_tick'] == i and type(row.get('source_ns')) is int and row['source_ns'] == i*DT
            and row.get('native_physics_tick') == first_native+i and row.get('policy_counter') == first_counter+i
            and identity(row, ctx), 'raw_source_identity_or_counter_mismatch:' + str(i))
        require(row.get('acquisition_phase') == ('actual_final_PhysX_END' if i == count else 'actual_pre_policy_PhysX_BEGIN'), 'raw_acquisition_phase_mismatch:'+str(i))
        for name, size in (('position',3),('quaternion_wxyz',4),('linear_velocity_world',3),
                ('angular_velocity_world',3),('official_policy_input',3),('highlevel_command',2)):
            vector(row, name, size)
        require(math.hypot(*row['quaternion_wxyz']) > 0, 'zero_quaternion')
        require(row.get('full_linear_speed') == math.hypot(*row['linear_velocity_world'])
            and row.get('full_angular_speed') == math.hypot(*row['angular_velocity_world']), 'derived_norm_mismatch')
        require(abs(row['highlevel_command'][0]) <= limits[0] and abs(row['highlevel_command'][1]) <= limits[1], 'command_authority_exceeded')
        require(abs(row['official_policy_input'][0]) <= inp_bounds[0]
            and row['official_policy_input'][1] == 0 and abs(row['official_policy_input'][2]) <= inp_bounds[1], 'policy_input_domain_exceeded')
        if i < count:
            require(row.get('mode') == mode and isinstance(row.get('authority'), dict)
                and row['authority'].get('policy_input_override') is None, 'consumption_mode_or_override_mismatch')
            if not normal:
                event = frozen.at_tick(i)
                require(row['highlevel_command'] == event['command']
                    and row['authority'].get('replay_original_authority') == event['authority'], 'frozen_ZOH_command_or_original_authority_mismatch:'+str(i))
    events, event_sha = read_rows(path/'policy_command_events.jsonl')
    require(event_sha == manifest.get('command_events_sha256') and len(events) == manifest.get('command_event_count')
        and manifest.get('command_events_file') == 'policy_command_events.jsonl', 'command_event_manifest_mismatch')
    require(bool(events) and events[0]['source_tick'] == 0, 'initial_consumption_event_missing')
    previous_tick = -1
    for event in events:
        tick = event.get('source_tick')
        require(type(tick) is int and previous_tick < tick < count and event.get('source_ns') == tick*DT
            and event.get('native_physics_tick') == first_native+tick
            and event.get('policy_counter') == first_counter+tick and identity(event, ctx)
            and event.get('mode') == mode and event.get('acquisition_phase') == 'actual_pre_policy_PhysX_BEGIN'
            and event.get('command') == rows[tick]['highlevel_command'] and event.get('authority') == rows[tick]['authority'], 'command_event_continuity_or_identity_mismatch')
        previous_tick = tick
    observations, obs_sha = read_rows(path/'policy_observations.jsonl')
    require(len(observations) == manifest.get('observation_count'), 'observation_manifest_count_mismatch')
    previous_counter = -10
    source_obs = 0
    for obs in observations:
        counter = obs.get('inference_policy_counter')
        require(type(counter) is int and counter == previous_counter+10 and counter % 10 == 0
            and obs.get('policy_counter') == counter and obs.get('native_physics_tick') == counter+10
            and obs.get('kind') == 'official_single_observation_return' and obs.get('schema') == 1
            and obs.get('acquisition_phase') == 'actual_pre_policy_PhysX_BEGIN' and obs.get('mode') == mode
            and obs.get('observation_dtype') == 'torch.float32' and identity(obs, ctx), 'official_50hz_counter_or_identity_mismatch')
        vector(obs,'observation',48)
        if counter < first_counter:
            require(obs.get('phase') == 'bootstrap', 'bootstrap_phase_changed')
        else:
            tick = counter-first_counter
            require(obs.get('phase') == 'source' and tick < count
                and obs.get('source_tick') == tick and obs.get('source_ns') == tick*DT, '50hz_original_source_phase_mismatch')
            source_obs += 1
        previous_counter = counter
    require(source_obs == sum((first_counter+i)%10 == 0 for i in range(count)), 'missing_50hz_inference')
    component, velocity = manifest.get('component_experiment'), manifest.get('velocity_feedback_experiment')
    if not normal:
        require(isinstance(component,dict) and isinstance(velocity,dict), 'component_experiment_manifest_missing')
        for experiment in (component,velocity):
            require(identity(experiment,ctx) and experiment.get('source_scene_sha256') == scene_sha
                and experiment.get('frozen_command_sha256') == frozen_sha
                and experiment.get('source_scene_bytes_modified') is False and experiment.get('navigation_authorization') is False
                and experiment.get('actual_source_begin_ns') == 0
                and experiment.get('actual_source_begin_native_tick') == first_native
                and experiment.get('actual_source_begin_policy_counter') == first_counter, 'component_experiment_identity_or_phase_mismatch')
        require(component.get('requested') is False and component.get('runtime_local_override') is None
            and component.get('source_spec_stall_recovery') == scene['robot'].get('stall_recovery')
            and component.get('source_resolved_stall_recovery_enabled') == scene['robot']['stall_recovery']['enabled'], 'extra_memory_configuration_factor')
        require(velocity.get('source_mode') == 'spot_monotone_measured_v2', 'source_v2_required')
        pc = summary.get('policy_component',{})
        require(pc.get('enabled') is True and pc.get('external_udp_disabled') is True
            and pc.get('navigation_authorization') is False and pc.get('frozen_command_sha256') == frozen_sha
            and pc.get('velocity_feedback_experiment') == velocity
            and pc.get('action_memory_experiment') == component
            and pc.get('internal_policy_linear_speed') is None and pc.get('desired_navigation_linear_speed') is None
            and pc.get('internal_policy_motion_frames') == 0, 'external_competitor_or_calibration_enabled')
        if alt:
            require(velocity.get('requested') is True and velocity.get('applied') is True
                and velocity.get('requested_mode') == velocity.get('runtime_mode') == velocity.get('runtime_local_override') == 'spot_monotone_measured_v3'
                and velocity.get('activation_source_tick') == velocity.get('actual_activation_source_tick') == activation
                and velocity.get('actual_activation_source_ns') == activation*DT
                and velocity.get('actual_activation_native_tick') == first_native+activation
                and velocity.get('actual_activation_policy_counter') == first_counter+activation
                and velocity.get('actual_activation_phase') == 'actual_pre_policy_PhysX_BEGIN', 'delayed_v3_activation_not_original_BEGIN')
        else:
            require(velocity.get('requested') is False and velocity.get('applied') is False
                and velocity.get('runtime_mode') == 'spot_monotone_measured_v2'
                and velocity.get('runtime_local_override') is None, 'baseline_not_original_v2')
    actor_rows, actor_sha = read_rows(path/'dynamic_actor_readings.jsonl')
    actor_ids = sorted(a['id'] for a in scene['dynamic_actors'] if a.get('enabled') is True)
    last_source = -1
    for row in actor_rows:
        sim = row.get('sim_time_ns')
        require(type(sim) is int and last_source < sim <= count*DT
            and row.get('registry_sha256') == ready.get('dynamic_actor_registry_sha256')
            and sorted(row.get('samples',{})) == actor_ids, 'actor_source_or_registered_identity_mismatch')
        for actual in row['samples'].values():
            require(actual.get('present') is True, 'actual_actor_missing')
            vector(actual,'position',3); vector(actual,'orientation_xyzw',4); vector(actual,'linear_velocity',3)
        last_source=sim
    require(ready.get('dynamic_actor_count') == len(actor_ids) == 6, 'all_six_actual_actors_required')
    link_rows, link_sha = read_rows(path/'robot_link_readings.jsonl')
    for row in link_rows:
        require(row.get('session_id') == ctx['session_id'] and row.get('clock_anchor_ns') == ctx['clock_anchor_ns']
            and type(row.get('sim_time_ns')) is int and isinstance(row.get('colliders'),list)
            and len(row['colliders']) == 13, 'actual_link_snapshot_identity_mismatch')
        vector(row,'pose',7)
    collision_path = path/'collision_audit.json'
    collision = load(collision_path)
    require(collision.get('scope') == 'sampled_actual_link_geometry_static_and_actor'
        and collision.get('continuous_collision_proof') is False and collision.get('session_id') == ctx['session_id']
        and collision.get('spec_sha256') == scene_sha and sorted(collision.get('actor_ids',[])) == actor_ids, 'collision_audit_context_mismatch')
    examples = collision.get('possible_overlap_examples',[])
    first_overlap = min((e for e in examples if e.get('kind') == 'dynamic'),
        key=lambda e:e['sim_time_ns'],default=None)
    from navigation_physics_audit import _domain, _final_stop
    zero_checks=[0,0];zero_violations=[0,0];zero_first=[None,None]
    for row in rows[activation:count]:
        for axis,input_axis in ((0,0),(1,2)):
            if row['highlevel_command'][axis] == 0:
                zero_checks[axis] += 1
                if row['official_policy_input'][input_axis] != 0:
                    zero_violations[axis] += 1
                    if zero_first[axis] is None:
                        zero_first[axis]=witness(row)
    stop = _final_stop(rows, ctx['clock_anchor_ns'], measurements, stationary)
    result=dict(path=str(path), epoch=ctx['epoch'], validation_verified=True,
        raw_file_sha256=raw_sha, manifest_sha256=sha(path/'policy_history_manifest.json'),
        summary_sha256=sha(path/'summary.json'), ready_sha256=sha(path/'ready.json'),
        begin_sample_count=count, actual_END_count=1, actual_END=witness(rows[-1]),
        duration_source_ns=count*DT, original_bootstrap_native_tick=first_native,
        original_policy_counter=first_counter, source_50hz_observation_count=source_obs,
        bootstrap_50hz_observation_count=len(observations)-source_obs,
        observations_sha256=obs_sha, command_events_count=len(events),command_events_sha256=event_sha,
        component_experiment=component, velocity_feedback_experiment=velocity,
        recorded_recovery_count=manifest.get('recovery_count'),
        full_epoch_domain=_domain(rows,ctx['clock_anchor_ns'],domain_bounds,yaw_bound),
        activation_to_END_domain=_domain(rows[activation:],ctx['clock_anchor_ns'],domain_bounds,yaw_bound),
        activation_begin=witness(rows[activation]),
        zero_axis_policy_withdrawal=dict(scope='consumed original BEGIN ticks from activation through last real step; END is observation only',
            begin_checks_by_axis=zero_checks,nonzero_policy_input_on_zero_command_by_axis=zero_violations,
            every_zero_axis_input_exactly_zero=zero_violations==[0,0],first_violation_by_axis=zero_first),
        measured_final_zero_episode=stop, actor_actual_raw_sha256=actor_sha,
        actor_canonical_rows_sha256=canonical_hash(actor_rows), actor_sample_packets=len(actor_rows),
        actor_ids=actor_ids, robot_links_raw_sha256=link_sha,
        collision_audit=dict(path=str(collision_path),sha256=sha(collision_path),completed=collision.get('completed'),
            sample_count=collision.get('sample_count'),possible_overlap_counts=collision.get('possible_overlap_counts'),
            possible_overlap_sample_counts=collision.get('possible_overlap_sample_counts'),
            first_recorded_dynamic_possible_overlap=first_overlap,
            contact_force_measured=False, continuous_collision_claim=False,
            interpretation='sampled possible AABB overlap is unresolved; neither contact nor a unique force cause is established'))
    return result, rows, actor_rows, link_rows


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--session',type=Path,required=True)
    p.add_argument('--baseline',type=Path,required=True)
    p.add_argument('--alternative',type=Path,required=True)
    p.add_argument('--frozen',type=Path,required=True)
    p.add_argument('--activation-tick',type=int,default=36428)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--repo',type=Path,default=Path('/home/eric/wjg/d1max-system'))
    args=p.parse_args()
    sys.path[:0]=[str(args.repo/'simulation/isaacsim'),str(args.repo/'d1max_nav_ws/src/d1max_pct_scan')]
    from d1max_pct_scan.braking_model import reference_model_limits, platform_model_limits
    from d1max_pct_scan.execution_timing import validate_timing, validate_stationary
    from policy_history import FrozenCommands
    result=dict(schema=1,kind='private_v59_delayed_v3_frozen_full_scene_comparison',
        evidence_verified=False, errors=[], activation_source_tick=args.activation_tick,
        exact_zero_input_is_not_physical_stop=True,hardware_acceptance=False,
        navigation_acceptance=False, continuous_collision_claim=False, raw_records_modified=False,
        causal_scope='one isolated frozen-command delayed controller-factor comparison; no unique PI, neural-policy or contact-force attribution',
        limitations=['Neither run is a new navigation Action acceptance.',
            'Actors and commands retain original source time and paths; epochs intentionally differ.',
            'Possible AABB overlap is sampled unresolved geometry and does not prove contact force.',
            'Frozen prefix excludes original last two ticks (4ms), without new ZERO or synthetic state.',
            'Future full XYZ speed guarantee is not inferred from the instantaneous .65 admission domain.'])
    try:
        session_path=args.session/'session.json' if args.session.is_dir() else args.session
        session=load(session_path);bridge=session['isaac_bridge_contract']
        scene_path=Path(bridge['scene_config']);model_path=Path(session['execution_braking_model_record'])
        scene,model=load(scene_path),load(model_path);scene_sha,model_sha=sha(scene_path),sha(model_path)
        require(session.get('simulation_backend') == 'isaacsim_physx' and session.get('simulation_clock') == 'isaac_fixed_anchor_v1'
            and session.get('transport_mode') == 'isolated_mock' and session.get('physical_acceptance') is False
            and bridge.get('source_clock') == 'isaac_physics_time' and type(bridge.get('clock_anchor_ns')) is int,
            'original_isolated_source_session_required')
        require(scene_sha == bridge['scene_sha256'] == session['input_hashes'].get(str(scene_path))
            and model_sha == session['execution_braking_model_sha256'] == session['input_hashes'].get(str(model_path))
            and model.get('session_id') == session['id'], 'original_sealed_rawbytes_mismatch')
        require(scene['robot'].get('kind') == 'official_spot_physx' and scene['robot'].get('record_full_physics_history') is True
            and scene['robot'].get('velocity_feedback_mode') == 'spot_monotone_measured_v2'
            and scene['physics']['frequency_hz'] == 500, 'original_v2_full_history_required')
        platform=platform_model_limits(model,'isolated_mock');reference=reference_model_limits(model,'isolated_mock')
        require(reference is not None,'isolated_reference_marker_required')
        validate_timing(model);stationary=validate_stationary(model)
        require(session['stationary_evidence'] == model['stationary_evidence']
            and (stationary['linear_threshold_mps'],stationary['angular_threshold_radps'],stationary['stationary_duration_s']) == (.03,.05,1.), 'original_stationary_contract_changed')
        require([platform['command_max_speed_mps'],platform['command_max_yaw_radps']]
            == [session['max_speed_mps'],session['max_yaw_radps']]
            == [scene['robot']['max_linear_speed'],scene['robot']['max_angular_speed']], 'original_command_authority_changed')
        frozen_value=load(args.frozen);frozen_sha=sha(args.frozen)
        require(frozen_sha == FROZEN_SHA,'known_frozen_input_hash_mismatch')
        n=frozen_value['actual_source_ticks'];require(n == 43410 and args.activation_tick == 36428,'this_comparison_domain_mismatch')
        frozen=FrozenCommands(frozen_value,scene_sha,n,[session['max_speed_mps'],session['max_yaw_radps']])
        original_dir=session_path.parent/'physics'
        require(sha(original_dir/'policy_history_manifest.json') == frozen_value['original_history_manifest_sha256']
            and sha(original_dir/'policy_command_events.jsonl') == frozen_value['original_command_events_sha256'], 'frozen_original_manifest_or_command_hash_mismatch')
        prefix=frozen_value['prefix_metadata']
        require(prefix.get('explicit_prefix') is True and prefix.get('original_source_tick_count') == 43412
            and prefix.get('selected_source_tick_count') == n and prefix.get('excluded_tail_tick_count') == 2
            and prefix.get('source_origin_ns') == 0 and prefix.get('timestamps_modified') is False
            and prefix.get('original_records_modified') is False,'original_prefix_scope_mismatch')
        bounds=dict(platform_instantaneous_admission=model['isolated_platform_model']['reachable_max_speed_mps'],
            full_xyz_reference=reference['reference_max_speed_mps'],
            measured_travel=reference['measured_travel_max_speed_mps'])
        yaw_bound=model['isolated_platform_model']['reachable_max_yaw_radps']
        require(bounds == dict.fromkeys(bounds,.65) and yaw_bound == .8,
            'original_velocity_domains_changed')
        ctx=dict(session_id=session['id'],clock_anchor_ns=bridge['clock_anchor_ns'])
        shared=dict(ctx_base=ctx,scene=scene,scene_sha=scene_sha,frozen_sha=frozen_sha,frozen=frozen,n=n,
            activation=args.activation_tick,domain_bounds=bounds,yaw_bound=yaw_bound,
            measurements=model['measurements'],stationary=stationary)
        original,original_rows,_,_=audit_run(original_dir,alt=False,normal=True,**shared)
        baseline,base_rows,base_actors,base_links=audit_run(args.baseline,alt=False,**shared)
        alternative,alt_rows,alt_actors,alt_links=audit_run(args.alternative,alt=True,**shared)
        original_baseline=difference(original_rows,base_rows,SIX,n)
        pre=difference(base_rows,alt_rows,SIX,args.activation_tick)
        all_commands=difference(base_rows,alt_rows,('highlevel_command',),n)
        all_commands['END_last_consumed_command_exactly_equal']=base_rows[-1]['highlevel_command']==alt_rows[-1]['highlevel_command']
        require(len(base_actors)==len(alt_actors),'actual_actor_packet_count_mismatch')
        first_actor_diff=next((dict(packet_index=i,source_ns=a.get('sim_time_ns'),baseline=a,alternative=b)
            for i,(a,b) in enumerate(zip(base_actors,alt_actors)) if a!=b),None)
        link_limit=args.activation_tick*DT
        base_prefix=[r for r in base_links if r['sim_time_ns'] < link_limit]
        alt_prefix=[r for r in alt_links if r['sim_time_ns'] < link_limit]
        first_link_diff=next((dict(packet_index=i,source_ns=a.get('sim_time_ns'))
            for i,(a,b) in enumerate(zip(base_prefix,alt_prefix)) if a!=b),None)
        result.update(original_session_path=str(session_path), original_session_sha256=sha(session_path),
            scene_path=str(scene_path),scene_rawbytes_sha256=scene_sha,model_path=str(model_path),model_rawbytes_sha256=model_sha,
            frozen_path=str(args.frozen),frozen_sha256=frozen_sha,original_prefix_metadata=prefix,
            excluded_original_tail_source_duration_s=2*DT/1e9,
            original=original,baseline=baseline,alternative=alternative,
            comparisons=dict(original_v59_to_baseline_begin=original_baseline,
                baseline_to_v3_before_activation=pre,all_original_source_BEGIN_commands=all_commands,
                all_actual_actor_packets=dict(packet_count=len(base_actors),full_rows_exactly_equal=first_actor_diff is None,
                    first_difference=first_actor_diff,canonical_baseline_sha256=canonical_hash(base_actors),
                    canonical_alternative_sha256=canonical_hash(alt_actors),normalization='sorted JSON key serialization only; no field omitted or value changed'),
                actual_13_link_geometry_before_activation=dict(baseline_packet_count=len(base_prefix),
                    alternative_packet_count=len(alt_prefix),full_rows_exactly_equal=len(base_prefix)==len(alt_prefix) and first_link_diff is None,
                    first_difference=first_link_diff,baseline_canonical_sha256=canonical_hash(base_prefix),alternative_canonical_sha256=canonical_hash(alt_prefix))),
            same_controller_source_snapshot_required=True,
            changed_factor='runtime servo.mode at real BEGIN tick36428 only; source scene remains v2',
            source_hashes={str(p):sha(p) for p in [Path(__file__),args.repo/'simulation/isaacsim/navigation_physics_audit.py',
                args.repo/'simulation/isaacsim/policy_history.py',args.repo/'simulation/isaacsim/full_scene_replay.py',
                args.repo/'simulation/isaacsim/scene.py',args.repo/'simulation/isaacsim/quadruped.py',
                args.repo/'d1max_nav_ws/src/d1max_pct_scan/d1max_pct_scan/braking_model.py']})
        require(original_baseline['all_fields_exactly_equal'] and pre['all_fields_exactly_equal']
            and all_commands['all_fields_exactly_equal'] and all_commands['END_last_consumed_command_exactly_equal']
            and first_actor_diff is None and len(base_prefix)==len(alt_prefix) and first_link_diff is None,
            'single_factor_equality_not_verified')
        result['evidence_verified']=True
    except (ValueError,KeyError,TypeError,OSError,IndexError) as error:
        result['errors'].append(str(error))
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(output=str(args.output),sha256=sha(args.output),evidence_verified=result['evidence_verified'],errors=result['errors'])))
    return 0 if result['evidence_verified'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
