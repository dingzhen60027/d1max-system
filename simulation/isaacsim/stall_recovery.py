"""Optional isolated Spot action-memory experiment; no motion authorization."""
from __future__ import annotations

import math
from numbers import Real

DT_NS = 2_000_000
DECIMATION = 10
MIN_FORWARD_MPS = .05


def enabled_for(robot):
    value = robot.get('stall_recovery', {'schema': 1, 'enabled': False, 'isolated_fixture': False})
    if (not isinstance(value, dict) or set(value) != {'schema', 'enabled', 'isolated_fixture'}
            or type(value['schema']) is not int or value['schema'] != 1
            or type(value['enabled']) is not bool or type(value['isolated_fixture']) is not bool):
        raise ValueError('invalid_stall_recovery_config')
    if value['enabled'] and (robot.get('kind') != 'official_spot_physx' or not value['isolated_fixture']):
        raise ValueError('stall_recovery_requires_explicit_isolated_spot_fixture')
    return value['enabled']


def vector(value, size):
    if (len(value) != size or any(isinstance(item, bool) or not isinstance(item, Real)
            or not math.isfinite(float(item)) for item in value)):
        raise ValueError('invalid_stall_measurement')
    return [float(item) for item in value]


class StallRecovery:
    """One intervention per qualified episode on an existing inference boundary.

    All full XYZ/angular samples must be consecutive real 500 Hz source ticks.
    The unchanged forward demand must remain at least .05m/s; tiny goal demand
    is not a stall. Missing ticks break the window. Counter/source/identity
    faults permanently withdraw memory intervention from this object. This is
    an isolated experiment, not evidence of navigation or physical acceptance.
    """
    def __init__(self):
        self.previous_ns = None
        self.previous_tick = self.previous_native_tick = self.previous_counter = None
        self.source_identity = None
        self.qualified = False
        self.episode = 0
        self.episode_start_ns = None
        self.qualification_start_ns = None
        self.stationary_start_ns = None
        self.consumed = False
        self.fault = None

    def _fail(self, reason):
        if self.fault is None:
            self.fault = reason
        self.qualified = False
        self.qualification_start_ns = self.stationary_start_ns = None
        raise ValueError('stall_guard_fault:' + self.fault)

    def observe(self, source_ns, command, linear, angular, policy_counter, *, tick_context=None):
        if self.fault is not None:
            self._fail(self.fault)
        try:
            command, linear, angular = vector(command, 2), vector(linear, 3), vector(angular, 3)
        except (ValueError, TypeError, OverflowError):
            self._fail('invalid_stall_measurement')
        if (type(source_ns) is not int or source_ns < 0
                or type(policy_counter) is not int or policy_counter < 0):
            self._fail('invalid_stall_source')
        context = tick_context
        if (not isinstance(context, dict) or context.get('phase') != 'source'
                or context.get('acquisition_phase') != 'actual_pre_policy_PhysX_BEGIN'
                or any(type(context.get(k)) is not int or context[k] < 0
                    for k in ('source_tick', 'source_ns', 'native_physics_tick', 'policy_counter'))
                or context['source_ns'] != source_ns or context['source_tick'] * DT_NS != source_ns
                or context['policy_counter'] != policy_counter
                or any(not isinstance(context.get(k), str) or not context[k]
                    for k in ('epoch', 'session_id'))
                or type(context.get('clock_anchor_ns')) is not int or context['clock_anchor_ns'] <= 0):
            self._fail('invalid_stall_context')
        identity = {k: context[k] for k in ('epoch', 'session_id', 'clock_anchor_ns')}
        if self.source_identity is not None and identity != self.source_identity:
            self._fail('stall_context_identity_changed')
        source_tick, native_tick = context['source_tick'], context['native_physics_tick']
        gap = False
        if self.previous_ns is not None:
            steps = source_tick - self.previous_tick
            if steps <= 0 or source_ns <= self.previous_ns:
                self._fail('stall_source_rollback')
            if (native_tick - self.previous_native_tick != steps
                    or policy_counter - self.previous_counter != steps):
                self._fail('stall_counter_source_mismatch')
            gap = steps != 1
        if self.source_identity is None:
            self.source_identity = identity
        self.previous_ns, self.previous_tick = source_ns, source_tick
        self.previous_native_tick, self.previous_counter = native_tick, policy_counter
        if command[0] < MIN_FORWARD_MPS:
            self.qualified = False
            self.stationary_start_ns = self.episode_start_ns = self.qualification_start_ns = None
            self.consumed = False
            return None
        if not self.qualified:
            self.qualified = True
            self.episode += 1
            self.episode_start_ns = self.qualification_start_ns = source_ns
            self.consumed = False
        if gap:
            self.qualification_start_ns = source_ns
            self.stationary_start_ns = None
        full_linear, full_angular = math.hypot(*linear), math.hypot(*angular)
        if full_linear > .03 or full_angular > .05:
            self.stationary_start_ns = None
            return None
        if self.stationary_start_ns is None:
            self.stationary_start_ns = source_ns
        if (self.consumed or source_ns - self.qualification_start_ns < 1_000_000_000
                or source_ns - self.stationary_start_ns < 1_000_000_000
                or policy_counter % DECIMATION):
            return None
        self.consumed = True
        return dict(source_ns=source_ns, episode=self.episode,
            episode_start_ns=self.episode_start_ns, stationary_start_ns=self.stationary_start_ns,
            qualification_start_ns=self.qualification_start_ns,
            qualification_min_forward_mps=MIN_FORWARD_MPS,
            source_identity=dict(self.source_identity), source_tick=source_tick,
            native_physics_tick=native_tick,
            continuous_static_duration_ns=source_ns-self.stationary_start_ns,
            command=command, full_linear_norm=full_linear, full_angular_norm=full_angular,
            policy_counter=policy_counter)
