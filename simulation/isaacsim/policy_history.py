"""Real pre-policy source-tick history and bounded full-scene replay helpers."""
from __future__ import annotations
import bisect
import hashlib
import json
import math
from pathlib import Path

DT_NS = 2_000_000


def full_physics_history_enabled(robot, component=False):
    """An explicit sealed option adds read-only native states in navigation."""
    value = robot.get('record_full_physics_history', False)
    if type(value) is not bool or type(component) is not bool:
        raise ValueError('full_physics_history_requires_explicit_bool')
    return component or value


def observe_official_return(compute, command, observer, counter, context):
    """The original single tensor return is also the inference input."""
    observation = compute(command)
    if observer is not None:
        observer(observation, counter, context)
    return observation


def command_domain(limits):
    if (len(limits) != 2 or any(type(v) not in (int,float) or not math.isfinite(v) or v <= 0 for v in limits)
            or limits[0] > .3 or limits[1] > .5):
        raise ValueError('invalid_sealed_highlevel_command_domain')
    return [float(v) for v in limits]


def finite_command(value, limits=(.15,.3)):
    if len(value) != 2 or any(not math.isfinite(float(v)) for v in value):
        raise ValueError('invalid_history_command')
    value = [float(v) for v in value]
    limits=command_domain(limits)
    if abs(value[0]) > limits[0] or abs(value[1]) > limits[1]:
        raise ValueError('history_command_outside_original_limits')
    return value


def check_component_options(test_frames, frozen_file, policy_speed, motion_frames,
                            test_linear=0., test_angular=0., control_file=None,
                            navigation_speed=None,command_limits=(.15,.3)):
    if frozen_file is not None:
        if not test_frames or test_frames % 10 or policy_speed is not None or navigation_speed is not None or test_linear or test_angular or control_file:
            raise ValueError('frozen_replay_requires_exclusive_bounded_component_mode')
    if policy_speed is not None:
        if (not math.isfinite(policy_speed) or not 0 <= policy_speed <= .6 or not test_frames
                or test_frames % 10 or type(motion_frames) is not int or not 0 < motion_frames < test_frames
                or motion_frames % 10 or frozen_file is not None or navigation_speed is not None or test_linear or test_angular or control_file):
            raise ValueError('policy_calibration_requires_exclusive_motion_and_true_zero_tail')
    if navigation_speed is not None:
        limits=command_domain(command_limits)
        if (not math.isfinite(navigation_speed) or not 0 <= navigation_speed <= limits[0]
                or not test_frames or test_frames % 10 or type(motion_frames) is not int
                or not 0 < motion_frames < test_frames or motion_frames % 10
                or test_frames-motion_frames < 2000 or frozen_file is not None or policy_speed is not None
                or test_linear or test_angular or control_file):
            raise ValueError('navigation_servo_calibration_requires_sealed_domain_and_four_second_true_zero_tail')
    elif motion_frames and policy_speed is None:
        raise ValueError('policy_motion_frames_requires_internal_policy_speed')


class PolicyHistory:
    """Sparse exact changes, with a checked invocation on every source tick."""
    def __init__(self, result_dir, scene_sha256, component=False, command_limits=(.15,.3)):
        self.directory = Path(result_dir)
        self.scene_sha256 = scene_sha256
        self.command_limits = command_domain(command_limits)
        self.events = (self.directory/'policy_command_events.jsonl').open('w')
        self.observations = (self.directory/'policy_observations.jsonl').open('w')
        self.recovery = (self.directory/'policy_recovery_events.jsonl').open('w')
        self.component = (self.directory/'policy_component_state_500hz.jsonl').open('w') if component else None
        self.component_count = 0
        self.component_last_ns = None
        self.previous_tick = None
        self.last_key = None
        self.tick_count = self.event_count = self.observation_count = self.recovery_count = 0

    @staticmethod
    def write(stream, row):
        stream.write(json.dumps(row, separators=(',', ':'), allow_nan=False)+'\n')

    def command(self, context, command, authority):
        tick = context['source_tick']
        if type(tick) is not int or tick < 0 or (self.previous_tick is not None and tick != self.previous_tick+1):
            raise ValueError('nonconsecutive_actual_policy_source_ticks')
        if self.previous_tick is None and tick != 0:
            raise ValueError('missing_first_actual_policy_tick')
        if context['source_ns'] != tick*DT_NS:
            raise ValueError('policy_tick_source_mismatch')
        command = finite_command(command,self.command_limits)
        key = dict(command=command, authority=authority)
        if key != self.last_key:
            self.write(self.events, dict(schema=1, kind='actual_pre_policy_command_or_authority_change',
                **context, **key))
            self.event_count += 1
            self.last_key = key
        self.previous_tick = tick
        self.tick_count += 1

    def observation(self, observation, policy_counter, context):
        # This observer receives the official single compute return; no robot
        # getters, inference, action setters, or data-dependent policy calls.
        value = observation.detach().cpu().reshape(-1).tolist()
        if len(value) != 48 or any(not math.isfinite(v) for v in value) or not isinstance(context, dict):
            raise ValueError('invalid_official_policy_observation')
        self.write(self.observations, dict(schema=1, kind='official_single_observation_return',
            **context, inference_policy_counter=policy_counter, observation=value,
            observation_dtype=str(observation.dtype)))
        self.observation_count += 1

    def memory_event(self, event):
        self.write(self.recovery, dict(schema=1, kind='isolated_stall_action_memory_intervention', **event))
        self.recovery_count += 1

    def component_state(self, context, command, policy_input, position, quaternion_wxyz, linear, angular):
        if self.component is None:
            raise ValueError('component_measurement_not_enabled')
        source = context['source_ns']
        if type(source) is not int or source < 0 or (self.component_last_ns is not None and source != self.component_last_ns+DT_NS):
            raise ValueError('component_actual_measurement_source_gap')
        vectors = [list(map(float, v)) for v in (position,quaternion_wxyz,linear,angular,policy_input)]
        if [len(v) for v in vectors] != [3,4,3,3,3] or any(not math.isfinite(item) for v in vectors for item in v):
            raise ValueError('invalid_component_actual_measurement')
        position,quaternion_wxyz,linear,angular,policy_input = vectors
        self.write(self.component, dict(schema=1,kind='actual_full_body_component_measurement',**context,
            highlevel_command=finite_command(command,self.command_limits),official_policy_input=policy_input,
            position=position,quaternion_wxyz=quaternion_wxyz,linear_velocity_world=linear,angular_velocity_world=angular,
            full_linear_speed=math.hypot(*linear),full_angular_speed=math.hypot(*angular)))
        self.component_last_ns = source
        self.component_count += 1

    def close(self):
        for stream in (self.events, self.observations, self.recovery):
            stream.close()
        if self.component is not None:
            self.component.close()
        record = dict(schema=1, kind='actual_500hz_pre_policy_history', scene_sha256=self.scene_sha256,
            physics_hz=500, policy_hz=50, source_tick_ns=DT_NS,
            command_limits=self.command_limits,
            actual_source_ticks=self.tick_count, duration_ns=self.tick_count*DT_NS,
            first_source_tick=0 if self.tick_count else None, last_source_tick=self.previous_tick,
            consecutive_tick_invocations_checked=True, command_event_count=self.event_count,
            observation_count=self.observation_count, recovery_count=self.recovery_count,
            component_measurement_count=self.component_count,component_last_source_ns=self.component_last_ns,
            component_measurements_file='policy_component_state_500hz.jsonl' if self.component is not None else None,
            component_measurements_sha256=(hashlib.sha256((self.directory/'policy_component_state_500hz.jsonl').read_bytes()).hexdigest()
                if self.component is not None else None),
            component_measurement_cadence=('every real500Hz BEGIN plus final measured END'
                if self.component is not None else 'disabled; no extra body getter'),
            command_events_file='policy_command_events.jsonl',
            command_events_sha256=hashlib.sha256((self.directory/'policy_command_events.jsonl').read_bytes()).hexdigest(),
            semantics='actual BEGIN command consumed on every physical tick; sparse exact command/authority changes; no sampled-state reconstruction')
        (self.directory/'policy_history_manifest.json').write_text(json.dumps(record, indent=2)+'\n')
        return record


class FrozenCommands:
    def __init__(self, value, scene_sha256, test_frames, command_limits=None):
        self.command_limits = command_domain(value.get('command_limits',[.15,.3]))
        if command_limits is not None and self.command_limits != command_domain(command_limits):
            raise ValueError('frozen_command_sealed_limits_mismatch')
        if (value.get('schema') != 1 or value.get('kind') != 'frozen_actual_500hz_pre_policy_commands'
                or value.get('physics_hz') != 500 or value.get('scene_sha256') != scene_sha256
                or type(value.get('actual_source_ticks')) is not int
                or value['actual_source_ticks'] != test_frames or test_frames <= 0 or test_frames % 10
                or value.get('duration_ns') != test_frames*DT_NS):
            raise ValueError('frozen_command_domain_mismatch')
        self.value = value
        self.events = value['events']
        self.ticks = []
        for event in self.events:
            tick = event['source_tick']
            if (type(tick) is not int or not 0 <= tick < test_frames
                    or (self.ticks and tick <= self.ticks[-1]) or event['source_ns'] != tick*DT_NS):
                raise ValueError('invalid_frozen_command_tick')
            finite_command(event['command'],self.command_limits)
            self.ticks.append(tick)
        if not self.ticks or self.ticks[0] != 0:
            raise ValueError('missing_frozen_initial_command')

    @classmethod
    def load(cls, path, scene_sha256, test_frames, command_limits=None):
        data = Path(path).read_bytes()
        if len(data) > 32*1024*1024:
            raise ValueError('frozen_command_resource_limit')
        return cls(json.loads(data), scene_sha256, test_frames,command_limits)

    def at_tick(self, tick):
        if type(tick) is not int or not 0 <= tick < self.value['actual_source_ticks']:
            raise ValueError('frozen_command_outside_domain')
        return self.events[bisect.bisect_right(self.ticks, tick)-1]
