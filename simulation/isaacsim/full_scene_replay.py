#!/usr/bin/env python3
"""Freeze real pre-policy events or launch a bounded original-scene component.

This invokes the existing scene implementation, native sensors and full world;
it starts no navigation/bridge/SDK process and the scene disables all UDP in
component mode. A previous 50Hz trajectory cannot substitute for this history.
"""
from __future__ import annotations
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import subprocess

from policy_history import DT_NS, FrozenCommands, check_component_options, finite_command

HERE = Path(__file__).resolve().parent


def validate_action_memory_experiment(enabled, frozen_file, policy_speed=None, navigation_speed=None):
    if type(enabled) is not bool:
        raise ValueError('action_memory_experiment_requires_explicit_bool')
    if enabled and (frozen_file is None or policy_speed is not None or navigation_speed is not None):
        raise ValueError('action_memory_experiment_requires_exclusive_frozen_commands')


def validate_velocity_feedback_experiment(mode, frozen_file, policy_speed=None,
                                          navigation_speed=None, action_memory_recovery=False,
                                          activation_tick=0, frames=None):
    """One local controller factor on the same complete frozen command input."""
    if mode is not None and (mode != 'spot_monotone_measured_v3' or frozen_file is None
                            or policy_speed is not None or navigation_speed is not None
                            or action_memory_recovery):
        raise ValueError('velocity_feedback_experiment_requires_exclusive_frozen_v3_replay')
    if (type(activation_tick) is not int or activation_tick < 0
            or (mode is None and activation_tick != 0)
            or (mode is not None and frames is not None and activation_tick >= frames)):
        raise ValueError('velocity_feedback_activation_requires_original_in_range_source_tick')


def apply_velocity_feedback_experiment(servo, context, manifest):
    """Switch only the explicit mode at an original source BEGIN boundary.

    Bootstrap and all preceding physical steps keep v2. This never changes
    clocks, counters, filter, action memory, command events or actor targets.
    The normal v3 step withdraws zero axes after this switch.
    """
    if not manifest['requested'] or context['phase'] != 'source':
        return
    tick = context['source_tick']
    activation = manifest['activation_source_tick']
    if manifest.get('applied', False) or tick < activation:
        return
    if (tick != activation or type(tick) is not int or context['source_ns'] != tick*DT_NS
            or context['acquisition_phase'] != 'actual_pre_policy_PhysX_BEGIN'
            or any(type(context[key]) is not int or context[key] < 0
                   for key in ('native_physics_tick', 'policy_counter'))
            or servo.mode != 'spot_monotone_measured_v2'):
        raise RuntimeError('velocity_feedback_activation_source_or_original_mode_mismatch')
    servo.mode = 'spot_monotone_measured_v3'
    manifest.update(applied=True, runtime_mode=servo.mode,
        actual_activation_source_tick=tick, actual_activation_source_ns=context['source_ns'],
        actual_activation_native_tick=context['native_physics_tick'],
        actual_activation_policy_counter=context['policy_counter'],
        actual_activation_phase=context['acquisition_phase'])


def component_runtime_robot(robot, *, action_memory_recovery=False, frozen_file=None,
                            policy_speed=None, navigation_speed=None, velocity_feedback_mode=None):
    """Replay the original memory configuration or opt in to one intervention."""
    from stall_recovery import enabled_for
    validate_action_memory_experiment(action_memory_recovery, frozen_file, policy_speed, navigation_speed)
    validate_velocity_feedback_experiment(velocity_feedback_mode, frozen_file, policy_speed,
        navigation_speed, action_memory_recovery)
    if robot.get('kind') != 'official_spot_physx':
        raise ValueError('full_scene_component_requires_official_spot')
    source_enabled = enabled_for(robot)
    if source_enabled and (frozen_file is None or action_memory_recovery
                           or policy_speed is not None or navigation_speed is not None):
        raise ValueError('enabled_original_memory_recovery_requires_unmodified_frozen_replay')
    runtime = copy.deepcopy(robot)
    if action_memory_recovery:
        runtime['stall_recovery'] = dict(schema=1, enabled=True, isolated_fixture=True)
    if velocity_feedback_mode is not None:
        if (robot.get('velocity_feedback_mode') != 'spot_monotone_measured_v2'
                or robot.get('velocity_feedback', True) is not True):
            raise ValueError('velocity_feedback_experiment_requires_original_enabled_v2')
        runtime['velocity_feedback_mode'] = velocity_feedback_mode
    return runtime


def freeze_history(directory, output, *, source_ticks=None):
    directory, output = Path(directory), Path(output)
    if output.exists():
        raise ValueError('frozen_command_output_already_exists')
    manifest = json.loads((directory/'policy_history_manifest.json').read_text())
    path = directory/'policy_command_events.jsonl'
    raw = path.read_bytes()
    count = manifest.get('actual_source_ticks')
    if (manifest.get('kind') != 'actual_500hz_pre_policy_history' or manifest.get('physics_hz') != 500
            or manifest.get('consecutive_tick_invocations_checked') is not True
            or type(count) is not int or count <= 0
            or manifest.get('first_source_tick') != 0 or manifest.get('last_source_tick') != count-1
            or manifest.get('duration_ns') != count*DT_NS
            or hashlib.sha256(raw).hexdigest() != manifest.get('command_events_sha256')):
        raise ValueError('incomplete_or_changed_actual_policy_history')
    events = [json.loads(line) for line in raw.splitlines()]
    if len(events) != manifest.get('command_event_count'):
        raise ValueError('command_event_count_mismatch')
    # Validate the entire original stream before selecting any prefix: a
    # malformed/overridden excluded tail is still changed original evidence.
    previous = None
    limits = manifest.get('command_limits', [.15,.3])
    for event in events:
        tick = event.get('source_tick')
        if (type(tick) is not int or not 0 <= tick < count or event.get('source_ns') != tick*DT_NS
                or (previous is None and tick != 0) or (previous is not None and tick <= previous)):
            raise ValueError('invalid_original_command_event_tick')
        finite_command(event['command'], limits)
        if not isinstance(event.get('authority'), dict):
            raise ValueError('invalid_original_command_authority')
        if event['authority'].get('policy_input_override') is not None:
            raise ValueError('internal_policy_override_is_not_navigation_command_history')
        previous = tick
    if not events:
        raise ValueError('missing_original_initial_command')
    if source_ticks is None:
        if count % 10:
            raise ValueError('complete_history_requires_aligned_ticks_or_explicit_prefix')
        selected_count = count
    else:
        if type(source_ticks) is not int or not 0 < source_ticks <= count or source_ticks % 10:
            raise ValueError('invalid_explicit_original_source_prefix')
        selected_count = source_ticks
    selected_events = [event for event in events if event['source_tick'] < selected_count]
    value = dict(schema=1, kind='frozen_actual_500hz_pre_policy_commands', physics_hz=500,
        command_limits=limits,
        scene_sha256=manifest['scene_sha256'], actual_source_ticks=selected_count, duration_ns=selected_count*DT_NS,
        original_history_manifest_sha256=hashlib.sha256((directory/'policy_history_manifest.json').read_bytes()).hexdigest(),
        original_command_events_sha256=hashlib.sha256(raw).hexdigest(), events=selected_events,
        prefix_metadata=dict(kind='original_actual_500hz_source_prefix' if source_ticks is not None else 'original_complete_500hz_source_history',
            explicit_prefix=source_ticks is not None, original_source_tick_count=count,
            selected_source_tick_count=selected_count, excluded_tail_tick_count=count-selected_count,
            source_origin_ns=0, excluded_events_count=len(events)-len(selected_events),
            timestamps_modified=False, original_records_modified=False),
        navigation_authorization=False,
        semantics='exact actual command/authority changes consumed at BEGIN on real 500Hz ticks; causal ZOH')
    FrozenCommands(value, manifest['scene_sha256'], selected_count)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(value, separators=(',', ':'), allow_nan=False)+'\n')
    return value


def scene_command(session, result, *, frozen_file=None, policy_speed=None, navigation_speed=None,motion_frames=0, frames=None,
                  action_memory_recovery=False, velocity_feedback_mode=None, velocity_feedback_start_tick=0,
                  isaac_root='/home/eric/isaacsim'):
    session, result = Path(session).resolve(), Path(result).resolve()
    if result.exists():
        raise ValueError('component_result_already_exists')
    record = json.loads((session/'session.json').read_text())
    contract = record['isaac_bridge_contract']
    scene = Path(contract['scene_config']).resolve(strict=True)
    scene_bytes = scene.read_bytes()
    if hashlib.sha256(scene_bytes).hexdigest() != contract['scene_sha256']:
        raise ValueError('original_sealed_scene_changed')
    spec = json.loads(scene_bytes)
    if spec['robot'].get('kind') != 'official_spot_physx' or spec.get('physics',{}).get('frequency_hz') != 500:
        raise ValueError('component_requires_original_official_500hz_spot_scene')
    component_runtime_robot(spec['robot'], action_memory_recovery=action_memory_recovery,
        frozen_file=frozen_file, policy_speed=policy_speed, navigation_speed=navigation_speed,
        velocity_feedback_mode=velocity_feedback_mode)
    if frozen_file is not None:
        frozen_file = Path(frozen_file).resolve(strict=True)
        value = json.loads(frozen_file.read_text())
        frames = value['actual_source_ticks'] if frames is None else frames
        FrozenCommands(value, contract['scene_sha256'], frames,
            [float(spec['robot'].get('max_linear_speed',.25)),float(spec['robot'].get('max_angular_speed',.4))])
    check_component_options(frames, frozen_file, policy_speed, motion_frames,navigation_speed=navigation_speed,
        command_limits=[float(spec['robot'].get('max_linear_speed',.25)),float(spec['robot'].get('max_angular_speed',.4))])
    validate_velocity_feedback_experiment(velocity_feedback_mode, frozen_file, policy_speed,
        navigation_speed, action_memory_recovery, velocity_feedback_start_tick, frames)
    if frozen_file is None and policy_speed is None and navigation_speed is None:
        raise ValueError('explicit_frozen_commands_or_policy_calibration_required')
    command = [str(Path(isaac_root)/'python.sh'), str(HERE/'scene.py'), '--headless',
        '--scene-file', str(scene), '--scene-sha256', contract['scene_sha256'],
        '--result-dir', str(result), '--test-frames', str(frames),
        '--session-id', record['id'], '--clock-anchor-ns', str(contract['clock_anchor_ns'])]
    prior = record.get('static_collision_prior_contract')
    if prior:
        command += ['--static-prior-geometry-sha256', prior['static_prior_geometry_sha256']]
        if prior.get('body_envelope_attestation_required'):
            command += ['--body-envelope-json', json.dumps(prior['body_envelope'], separators=(',', ':'))]
    if frozen_file is not None:
        command += ['--frozen-command-file', str(frozen_file)]
        if action_memory_recovery:
            command += ['--test-action-memory-recovery']
        if velocity_feedback_mode is not None:
            command += ['--test-velocity-feedback-mode', velocity_feedback_mode]
            command += ['--test-velocity-feedback-start-tick', str(velocity_feedback_start_tick)]
    elif policy_speed is not None:
        command += ['--test-policy-linear-speed', str(policy_speed), '--test-policy-motion-frames', str(motion_frames)]
    else:
        command += ['--test-navigation-linear-speed',str(navigation_speed),'--test-policy-motion-frames',str(motion_frames)]
    return command


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    commands=parser.add_subparsers(dest='action', required=True)
    freeze=commands.add_parser('freeze')
    freeze.add_argument('--history-dir',type=Path,required=True)
    freeze.add_argument('--output',type=Path,required=True)
    freeze.add_argument('--source-ticks',type=int,
        help='Explicit unchanged original prefix; positive multiple of ten, no larger than the complete history')
    for name in ('command','launch'):
        child=commands.add_parser(name)
        child.add_argument('--session',type=Path,required=True)
        child.add_argument('--result-dir',type=Path,required=True)
        child.add_argument('--frozen-command-file',type=Path)
        child.add_argument('--test-action-memory-recovery',action='store_true',
            help='Isolated memory-only experiment on frozen real commands; baseline defaults disabled')
        child.add_argument('--test-velocity-feedback-mode', choices=['spot_monotone_measured_v3'],
            help='Single controller factor on original frozen v2 commands; source scene stays sealed')
        child.add_argument('--test-velocity-feedback-start-tick', type=int, default=0,
            help='Original source BEGIN tick to activate the mode; bootstrap and preceding ticks stay v2')
        child.add_argument('--test-policy-linear-speed',type=float)
        child.add_argument('--test-navigation-linear-speed',type=float)
        child.add_argument('--test-policy-motion-frames',type=int,default=0)
        child.add_argument('--test-frames',type=int)
        child.add_argument('--isaac-root',default=os.environ.get('ISAAC_SIM_ROOT','/home/eric/isaacsim'))
    args=parser.parse_args()
    if args.action=='freeze':
        value=freeze_history(args.history_dir,args.output,source_ticks=args.source_ticks)
        print(json.dumps(dict(output=str(args.output),ticks=value['actual_source_ticks'],sha256=hashlib.sha256(args.output.read_bytes()).hexdigest())))
        return
    command=scene_command(args.session,args.result_dir,frozen_file=args.frozen_command_file,
        policy_speed=args.test_policy_linear_speed,motion_frames=args.test_policy_motion_frames,
        navigation_speed=args.test_navigation_linear_speed,
        action_memory_recovery=args.test_action_memory_recovery,
        velocity_feedback_mode=args.test_velocity_feedback_mode,
        velocity_feedback_start_tick=args.test_velocity_feedback_start_tick,
        frames=args.test_frames,isaac_root=args.isaac_root)
    if args.action=='command':
        print(json.dumps(command))
        return
    env=dict(os.environ)
    for key in ('PYTHONPATH','LD_LIBRARY_PATH','AMENT_PREFIX_PATH','CMAKE_PREFIX_PATH',
                'PYTHONHOME','CONDA_PREFIX','VIRTUAL_ENV','PYTHONEXE','LD_PRELOAD'):
        env.pop(key,None)
    raise SystemExit(subprocess.call(command,env=env))


if __name__=='__main__':
    main()
