"""Single-use mode transactions producing intents, NEVER SDK calls.

RobotState has no vendor timestamp in SDK 0.1.0. Its callback sequence/time is
explicit receipt evidence only. Motion stop evidence uses original OnMcData
timestamps through StopVerifier. A transport adapter must not invent raw source
time or assign a late ACK to a new transaction after an ambiguous one.
"""
from dataclasses import dataclass
from .stop_verifier import StopVerifier, StopPolicy, finite


@dataclass(frozen=True)
class ModeCapability:
    name: str
    sdk_mode: int
    motion_status: int
    required_speed_level: int = 1
    mode_switch_verified: bool = False
    zero_hold_verified: bool = False
    move_verified: bool = False
    speed_mapping_verified: bool = False
    evidence_id: str = ''

    def __post_init__(self):
        if (self.name, self.sdk_mode, self.motion_status) not in (('general', 1, 5), ('stair', 3, 7)):
            raise ValueError('unsupported_mode_capability')
        if self.required_speed_level not in (1, 2, 3):
            raise ValueError('invalid_speed_level')
        for key in ('mode_switch_verified', 'zero_hold_verified', 'move_verified', 'speed_mapping_verified'):
            if type(getattr(self, key)) is not bool:
                raise ValueError('capability_requires_explicit_boolean')
        if any((self.mode_switch_verified, self.zero_hold_verified, self.move_verified,
                self.speed_mapping_verified)) and not self.evidence_id:
            raise ValueError('capability_requires_evidence_id')

    @property
    def motion_capable(self):
        return all((self.mode_switch_verified, self.zero_hold_verified,
                    self.move_verified, self.speed_mapping_verified, self.evidence_id))


def documented_capabilities():
    # API existence does not certify current firmware behavior or speed units.
    return {p.name: p for p in (ModeCapability('general', 1, 5), ModeCapability('stair', 3, 7))}


@dataclass(frozen=True)
class RequestEffect:
    kind: str
    transaction_id: str
    mode_epoch: int
    mode: str
    sdk_mode: int = 0


@dataclass(frozen=True)
class RobotModeState:
    sdk_session: str
    callback_sequence: int
    callback_time: float
    sport_mode: int
    motion_status: int
    speed_level: int
    control_source: int = 2
    software_emergency_status: int = 1
    hardware_emergency_status: int = 1
    head_direction: int = 1


@dataclass(frozen=True)
class ModePolicy:
    stop_timeout_s: float = 3.
    ack_timeout_s: float = 3.
    state_timeout_s: float = 4.
    robot_state_receipt_age_s: float = 2.
    confirmations: int = 2

    def __post_init__(self):
        if any(not finite(v) or not .1 <= v <= 30. for v in (
                self.stop_timeout_s, self.ack_timeout_s, self.state_timeout_s,
                self.robot_state_receipt_age_s)) or type(self.confirmations) is not int or not 2 <= self.confirmations <= 10:
            raise ValueError('invalid_mode_policy')


class ModeTransaction:
    def __init__(self, profiles=None, *, policy=None, stop_policy=None):
        self.profiles = dict(documented_capabilities() if profiles is None else profiles)
        if set(self.profiles) != {'general', 'stair'} or any(k != v.name for k, v in self.profiles.items()):
            raise ValueError('general_and_stair_profiles_required')
        self.policy = policy or ModePolicy()
        self.stop = StopVerifier(stop_policy or StopPolicy())
        self.phase, self.reason = 'idle', 'no_transaction'
        self.transaction_id = self.sdk_session = self.clock_epoch = self.target = self.current_mode = ''
        self.mode_epoch = 0
        self.started = self.submitted = self.acknowledged = None
        self.last_callback_sequence = self.dispatch_callback_sequence = -1
        self.last_robot_time = None
        self.matches = 0
        self.effects = []
        self._identities = set()

    def _effect(self, kind, mode='', sdk_mode=0):
        self.effects.append(RequestEffect(kind, self.transaction_id, self.mode_epoch, mode, sdk_mode))

    def take_effects(self):
        values, self.effects = tuple(self.effects), []
        return values

    def _fail(self, reason):
        self.phase, self.reason = 'needs_review', reason
        # Remove an unsubmitted mode/stop intent; cancellation cannot recall an
        # SDK command already sent. Never synthesize reverse mode or zero hold.
        self.effects = [effect for effect in self.effects if effect.kind == 'revoke_permission']
        if not self.effects:
            self._effect('revoke_permission')
        self.stop.invalidate(reason)
        self.matches = 0

    def begin(self, *, transaction_id, target_mode, current_mode, sdk_session, clock_epoch,
              now, raw_mc_cutoff=0, owned=False, initial_state=None):
        if self.phase == 'needs_review':
            return False
        if self.phase not in ('idle', 'ready'):
            return False
        if (not transaction_id or transaction_id in self._identities or len(self._identities) >= 1024 or not sdk_session or not clock_epoch
                or target_mode not in self.profiles or current_mode not in self.profiles or not finite(now)):
            return False
        self._identities.add(transaction_id)
        self.transaction_id, self.target, self.current_mode = transaction_id, target_mode, current_mode
        self.sdk_session, self.clock_epoch = sdk_session, clock_epoch
        self.mode_epoch += 1
        self.started = now
        self.submitted = self.acknowledged = self.last_robot_time = None
        self.last_callback_sequence = self.dispatch_callback_sequence = -1
        self.matches = 0
        self.effects.clear()
        self._effect('revoke_permission')
        self.stop.begin(transaction_id=transaction_id, sdk_session=sdk_session,
                        clock_epoch=clock_epoch, now=now, raw_cutoff=raw_mc_cutoff)
        if not owned:
            self._fail('control_not_owned')
        elif (initial_state is None or initial_state.sdk_session != sdk_session
                or type(initial_state.callback_sequence) is not int or initial_state.callback_sequence < 0
                or not finite(initial_state.callback_time)
                or not 0 <= now-initial_state.callback_time <= self.policy.robot_state_receipt_age_s
                or initial_state.control_source != 2
                or initial_state.software_emergency_status != 1 or initial_state.hardware_emergency_status != 1
                or initial_state.sport_mode != self.profiles[current_mode].sdk_mode
                or initial_state.motion_status != self.profiles[current_mode].motion_status
                or initial_state.head_direction != 1):
            self._fail('initial_robot_mode_not_confirmed')
        elif not self.profiles[current_mode].zero_hold_verified:
            self._fail('current_mode_zero_hold_unverified')
        elif not self.profiles[target_mode].mode_switch_verified:
            self._fail('target_mode_switch_unverified')
        elif not self.stop.policy.physical_acceptance_verified:
            self._fail('stop_policy_not_physically_accepted')
        else:
            self.last_callback_sequence, self.last_robot_time = initial_state.callback_sequence, initial_state.callback_time
            self.phase, self.reason = 'waiting_stop', 'waiting_stop_submission_and_mc'
            self._effect('request_zero_hold', current_mode, self.profiles[current_mode].sdk_mode)
        return True

    def stop_submitted(self, transaction_id, now):
        return self.phase == 'waiting_stop' and self.stop.mark_submitted(transaction_id, now)

    def observe_mc(self, sample, now):
        if self.phase not in ('waiting_stop', 'waiting_dispatch', 'waiting_ack', 'confirming_state', 'ready'):
            return False
        if sample.sdk_session != self.sdk_session or sample.clock_epoch != self.clock_epoch:
            self._fail('mc_context_changed')
            return False
        proven = self.stop.observe(sample, now)
        if self.phase == 'waiting_stop' and proven:
            self.phase, self.reason = 'waiting_dispatch', 'single_mode_intent_ready'
            self.dispatch_callback_sequence = self.last_callback_sequence
            self._effect('request_set_mode', self.target, self.profiles[self.target].sdk_mode)
        return proven

    def dispatch_ready(self, transaction_id, now):
        # Future sole SDK writer must check this under its dispatch lock just
        # before sending the intent, then report submission or failure.
        return (self.phase == 'waiting_dispatch' and transaction_id == self.transaction_id
                and finite(now) and self.stop.confirmed(now)
                and self.last_robot_time is not None
                and 0 <= now-self.last_robot_time <= self.policy.robot_state_receipt_age_s)

    def mode_submitted(self, transaction_id, now):
        if self.phase != 'waiting_dispatch' or transaction_id != self.transaction_id:
            return False
        if not self.dispatch_ready(transaction_id, now):
            self._fail('mode_submission_without_fresh_preconditions')
            return False
        self.submitted = now
        self.dispatch_callback_sequence = self.last_callback_sequence
        self.phase, self.reason = 'waiting_ack', 'waiting_mode_ack'
        return True

    def submission_failed(self, transaction_id):
        if transaction_id == self.transaction_id and self.phase not in ('idle', 'needs_review'):
            self._fail('sdk_write_outcome_unknown')

    def mode_ack(self, *, sdk_session, sdk_mode, callback_time, now):
        # Vendor OnMode has no request ID. Never retry an ambiguous transaction.
        if self.phase != 'waiting_ack':
            return False
        if (sdk_session != self.sdk_session or sdk_mode != self.profiles[self.target].sdk_mode
                or not finite(callback_time) or not finite(now)
                or callback_time < self.submitted or not 0 <= now-callback_time <= self.policy.robot_state_receipt_age_s
                or now-self.submitted > self.policy.ack_timeout_s):
            self._fail('mode_ack_late_or_mismatched')
            return False
        self.acknowledged = callback_time
        self.phase, self.reason = 'confirming_state', 'waiting_new_robot_mode_states'
        self.matches = 0
        return True

    def robot_state(self, state, now):
        if self.phase in ('idle', 'needs_review'):
            return False
        if (state.sdk_session != self.sdk_session or state.control_source != 2
                or state.software_emergency_status != 1 or state.hardware_emergency_status != 1):
            self._fail('robot_ownership_or_emergency_changed')
            return False
        if (type(state.callback_sequence) is not int or state.callback_sequence <= self.last_callback_sequence
                or not finite(state.callback_time) or not finite(now)
                or not 0 <= now-state.callback_time <= self.policy.robot_state_receipt_age_s
                or self.last_robot_time is not None and state.callback_time <= self.last_robot_time):
            return False
        self.last_callback_sequence, self.last_robot_time = state.callback_sequence, state.callback_time
        if self.phase in ('waiting_stop', 'waiting_dispatch'):
            current = self.profiles[self.current_mode]
            if state.sport_mode != current.sdk_mode or state.motion_status != current.motion_status:
                self._fail('current_mode_changed_before_dispatch')
            return False
        p = self.profiles[self.target]
        matches = (state.sport_mode == p.sdk_mode and state.motion_status == p.motion_status
                   and state.speed_level == p.required_speed_level and state.head_direction == 1)
        if self.phase == 'ready' and not matches:
            self._fail('confirmed_robot_mode_changed')
            return False
        if self.phase != 'confirming_state':
            return False
        if state.callback_time <= self.acknowledged or state.callback_sequence <= self.dispatch_callback_sequence:
            return False
        self.matches = self.matches+1 if matches else 0
        if self.matches >= self.policy.confirmations:
            self.phase, self.reason = 'ready', 'mode_confirmed_not_motion_permission'
            self._effect('mode_ready', self.target, p.sdk_mode)
            return True
        return False

    def cancel(self, reason='mode_transaction_cancelled'):
        if self.phase != 'idle':
            self._fail(reason)

    def control_lost(self):
        self.cancel('control_lost')

    def tick(self, now):
        if self.phase in ('idle', 'needs_review'):
            return
        if not finite(now) or now < self.started:
            self._fail('clock_invalid')
        elif self.phase in ('waiting_stop', 'waiting_dispatch') and now-self.started > self.policy.stop_timeout_s:
            self._fail('stop_or_dispatch_timeout')
        elif self.phase == 'waiting_ack' and now-self.submitted > self.policy.ack_timeout_s:
            self._fail('mode_ack_timeout')
        elif self.phase == 'confirming_state' and now-self.acknowledged > self.policy.state_timeout_s:
            self._fail('mode_state_confirmation_timeout')
        elif self.phase == 'ready' and (self.last_robot_time is None or now-self.last_robot_time > self.policy.robot_state_receipt_age_s):
            self._fail('mode_state_receipt_stale')
        elif self.phase in ('waiting_ack', 'confirming_state', 'ready') and (
                self.stop.last_source is None or now-self.stop.last_source > self.stop.policy.max_source_age_seconds):
            self._fail('mc_source_stale')

    def status(self, now):
        return dict(transaction_id=self.transaction_id, phase=self.phase, reason=self.reason,
            mode_epoch=self.mode_epoch, target_mode=self.target,
            mode_confirmed=self.phase == 'ready', motion_authorized=False,
            capability_verified=bool(self.target and self.profiles[self.target].motion_capable),
            robot_state_time_basis='sdk_callback_receipt_no_vendor_stamp',
            stop=self.stop.status(now), adapter_wired=False)
