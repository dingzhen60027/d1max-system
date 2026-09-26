"""ROS-independent, latched execution lease for the live planning pipeline.

Planning/display success is not motion permission. This module never owns an
SDK client, changes a robot mode, or fabricates calibration acceptance.
"""
from dataclasses import dataclass
import math


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def fresh(stamp, now, limit, future=.05):
    return finite(stamp) and finite(now) and -future <= now-stamp <= limit


@dataclass(frozen=True)
class MotionConfig:
    session_id: str
    map_version_id: str
    frame_id: str = 'd1max_loc_map'
    body_height: float = .55
    max_speed: float = .30
    max_yaw: float = .50
    single_floor_height_span: float = .25
    body_height_calibrated: bool = False
    collision_envelope_validated: bool = False
    vertical_envelope_validated: bool = False
    sdk_speed_mapping_validated: bool = False

    def __post_init__(self):
        if not self.session_id or not self.map_version_id or self.frame_id != 'd1max_loc_map':
            raise ValueError('explicit live map/session required')
        for name, ceiling in (('body_height', 1.), ('max_speed', .30),
                              ('max_yaw', .50), ('single_floor_height_span', .30)):
            value = getattr(self, name)
            if not finite(value) or not 0 < value <= ceiling:
                raise ValueError('invalid execution limit: '+name)
        if any(type(getattr(self, field)) is not bool for field in self.acceptance_fields):
            raise ValueError('acceptance values must be explicit booleans')

    @property
    def acceptance_fields(self):
        return ('body_height_calibrated', 'collision_envelope_validated',
                'vertical_envelope_validated', 'sdk_speed_mapping_validated')

    def blockers(self):
        return [field for field in self.acceptance_fields if not getattr(self, field)]


class ExecutionLease:
    """One explicit execution per reference generation. No automatic re-arm."""
    def __init__(self, config):
        self.c = config
        self.generation = 0
        self.task_generation = 0
        self.consumed_generation = 0
        self.goal = None
        self.reference_stamp = 0.
        self.reference_reason = 'waiting_reference'
        self.phase = 'locked'
        self.reason = 'not_started'
        self.issued_at = 0.
        self.started_at = 0.
        self.arm_confirmed_at = 0.
        self.arm_confirmed_wall = 0.
        self.context = None
        self.completion_started_at = 0.
        self.samples = {}
        self.order = {}
        self.permit_seq = 0
        self.mismatch_since = None
        self.matched_admission = None
        self.handover_active = False
        self.pair_wait_active = False
        self.pending_candidate_id = -1
        self.admission_highwater = -1
        self.admission_identity = None
        self.disarm_requested = False

    @property
    def active(self):
        return self.phase in ('arming', 'waiting_trajectory', 'executing', 'verifying_completion')

    def reference(self, *, session, generation, frame, stamp, points, now):
        if session != self.c.session_id or type(generation) is not int or generation < self.generation:
            return False
        if generation == self.generation:
            # Same-generation retransmissions must not mutate a committed task.
            if not points and self.goal is not None:
                self.stop('reference_cancelled')
                self.goal = None
            return False
        if self.active:
            self.stop('reference_replaced')
        self.generation, self.goal = generation, None
        self.reference_reason = 'reference_invalid'
        if (generation < 1 or frame != self.c.frame_id or not fresh(stamp, now, 2.)
                or not 2 <= len(points) <= 20000
                or any(len(p) != 3 or not all(finite(v) and abs(v) < 10000 for v in p) for p in points)):
            return False
        # An entire global reference must stay on this level. Small per-step
        # deltas must never turn a staircase into an approved flat route.
        heights = [p[2] for p in points]
        if max(heights)-min(heights) > self.c.single_floor_height_span:
            self.reference_reason = 'stairs_or_cross_floor_execution_not_supported'
            return False
        self.reference_stamp = stamp
        self.goal = [*points[-1][:2], points[-1][2]+self.c.body_height]
        self.reference_reason = ''
        return True

    def observe(self, kind, value, now, wall):
        if not isinstance(value, dict):
            self.samples.pop(kind, None)
            if self.active and kind in ('admission', 'gate'):
                self.stop('malformed_'+kind)
            return False
        field = 'received_at_unix' if kind == 'admission' else 'stamp' if kind == 'tracker' else 'wall_time'
        stamp = value.get(field)
        if not fresh(stamp, wall, .6) or stamp <= self.order.get(kind, -math.inf):
            return False  # replayed heartbeats do not extend receipt freshness
        if kind in ('admission', 'tracker') and value.get('session_id') != self.c.session_id:
            return False
        if kind == 'admission':
            sequence = value.get('sequence')
            if type(sequence) is not int or sequence <= self.order.get('admission_sequence', 0):
                return False
            self.order['admission_sequence'] = sequence
        if kind == 'gate' and (value.get('navigation_session_id') != self.c.session_id
                               or value.get('map_version_id') != self.c.map_version_id):
            return False
        self.order[kind] = stamp
        self.samples[kind] = (dict(value), now)
        # Revocation is an event, not just the latest depth-one sample. A later
        # accepted heartbeat must not erase a failure before the 20 Hz tick.
        if self.active and kind == 'admission':
            problem = self.admission_reason(now, wall)
            if problem:
                self.admission_failed(problem, value, now)
            else:
                self.admission_highwater = value['trajectory_id']
                self.admission_identity = self.curve_identity(value)
                if value.get('handover_pending') is True:
                    if self.mismatch_since is None:
                        self.mismatch_since = now
                    self.pair_wait_active = True
                    self.pending_candidate_id = max(self.pending_candidate_id, value['pending_candidate_id'])
        elif (self.phase in ('executing', 'waiting_trajectory') and kind == 'gate'
              and value.get('armed') is not True):
            self.stop('motion_gate_revoked: '+str(value.get('reason', 'stale_or_missing')))
        return True

    def sample(self, kind, now, wall, ttl=.35):
        item = self.samples.get(kind)
        if not item or not fresh(item[1], now, ttl, future=0.):
            return {}
        field = 'received_at_unix' if kind == 'admission' else 'stamp' if kind == 'tracker' else 'wall_time'
        return item[0] if fresh(item[0].get(field), wall, ttl) else {}

    def admission_reason(self, now, wall):
        a = self.sample('admission', now, wall)
        context = a.get('localization_context')
        if (a.get('schema') != 1 or a.get('execution_mode') != 'execution'
                or a.get('motion_authorized') is not False or a.get('frame_id') != self.c.frame_id
                or a.get('lease_timeout_sec') != .35
                or not isinstance(context, list) or len(context) != 3
                or context[0] != self.c.session_id or type(context[1]) is not int or context[1] < 1
                or not isinstance(context[2], str) or not context[2]
                or (self.active and context != self.context)
                or a.get('valid') is not True or a.get('generation') != self.generation
                or type(a.get('trajectory_id')) is not int or a['trajectory_id'] < 0
                or not fresh(a.get('source_stamp'), wall, 2.)
                or not fresh(a.get('debug_stamp'), wall, 2.)):
            return 'local_trajectory_not_admitted: '+str(a.get('reason', 'stale_or_missing'))
        if (type(a.get('handover_pending', False)) is not bool
                or (a.get('handover_pending') is True and
                    (type(a.get('pending_candidate_id')) is not int
                     or a['pending_candidate_id'] <= a['trajectory_id']))):
            return 'invalid_handover_pending_identity'
        if self.active and a['trajectory_id'] < self.admission_highwater:
            return 'admission_trajectory_id_regressed'
        if (self.active and a['trajectory_id'] == self.admission_highwater
                and self.admission_identity != self.curve_identity(a)):
            return 'admission_trajectory_identity_mutated'
        if (self.matched_admission is not None
                and a['trajectory_id'] < self.matched_admission[0]['trajectory_id']):
            return 'admission_trajectory_id_regressed'
        if (self.matched_admission is not None
                and a['trajectory_id'] == self.matched_admission[0]['trajectory_id']
                and any(a[key] != self.matched_admission[0][key]
                        for key in ('source_stamp', 'debug_stamp'))):
            return 'admission_trajectory_identity_mutated'
        return ''

    @staticmethod
    def curve_identity(a):
        # Pairing state and heartbeat receipt may change; the accepted curve's
        # source evidence, including its native collision proof, may not.
        return (a.get('source_stamp'), a.get('debug_stamp'), a.get('predecessor_id', -1),
                a.get('predecessor_safe', False), a.get('predecessor_check_stamp', 0.))

    def admission_failed(self, problem, admission, now):
        if (self.phase == 'executing' and admission.get('generation') == self.task_generation
                and admission.get('localization_context') == self.context
                and admission.get('reason') == 'native_completed'):
            self.phase, self.reason = 'verifying_completion', 'waiting_measured_goal_confirmation'
            self.completion_started_at = now
            self.disarm_requested = True
            self.matched_admission = None
        elif self.phase != 'verifying_completion':
            self.stop(problem)

    def spline_admitted(self, *, session, generation, frame, trajectory_id, source_stamp, now, wall):
        """An exact accepted candidate, not merely a fresh same-generation ID."""
        a = self.sample('admission', now, wall)
        return (self.phase in ('waiting_trajectory', 'executing')
                and not self.admission_reason(now, wall)
                and a.get('handover_pending') is not True
                and a['trajectory_id'] >= self.pending_candidate_id
                and session == self.c.session_id and generation == self.generation
                and frame == self.c.frame_id and type(trajectory_id) is int
                and trajectory_id == a.get('trajectory_id')
                and fresh(source_stamp, wall, 2.)
                and source_stamp >= self.arm_confirmed_wall
                and abs(source_stamp-a['source_stamp']) <= 1e-6)

    def candidate_pending(self, generation, trajectory_id, now):
        # The typed curve and its admission use different ROS topics. A curve
        # may arrive first even when bridge pairing was atomic; record that
        # evidence locally so an old matching heartbeat cannot keep motion on.
        a = self.samples.get('admission', ({},))[0]
        if (self.phase == 'executing' and generation == self.generation
                and type(trajectory_id) is int and trajectory_id > a.get('trajectory_id', -1)):
            if self.mismatch_since is None:
                self.mismatch_since = now
            self.pair_wait_active = True
            self.pending_candidate_id = max(self.pending_candidate_id, trajectory_id)

    def safe_handover(self, admission, tracker, now, wall):
        """Bounded overlap backed by a native *old-curve* collision recheck.

        A new accepted curve alone says nothing about whether its predecessor
        is safe. Never infer overlap permission from IDs or keep an old command
        alive by refreshing its original admission/source timestamps.
        """
        if self.phase != 'executing' or self.matched_admission is None:
            return False
        old, receipt = self.matched_admission
        return (self.mismatch_since is not None and 0 <= now-self.mismatch_since <= .15
                and tracker.get('active') is True and tracker.get('reason') == 'tracking'
                and tracker.get('generation') == self.generation
                and tracker.get('frame_id') == self.c.frame_id
                and tracker.get('trajectory_id') == old['trajectory_id']
                and old['generation'] == admission['generation'] == self.generation
                and old['localization_context'] == admission['localization_context'] == self.context
                and admission['trajectory_id'] > old['trajectory_id']
                and admission.get('predecessor_safe') is True
                and type(admission.get('predecessor_id')) is int
                and admission['predecessor_id'] == old['trajectory_id']
                and fresh(admission.get('predecessor_check_stamp'), wall, .15)
                and admission['predecessor_check_stamp'] >= old['source_stamp']
                and admission['source_stamp'] >= self.arm_confirmed_wall
                and fresh(receipt, now, .35, future=0.)
                and fresh(old.get('received_at_unix'), wall, .35)
                and fresh(old.get('source_stamp'), wall, 2.)
                and fresh(old.get('debug_stamp'), wall, 2.))

    def ready_reason(self, now, wall):
        if self.c.blockers():
            return 'acceptance_required: '+', '.join(self.c.blockers())
        if self.goal is None:
            return self.reference_reason or 'waiting_reference'
        if self.generation <= self.consumed_generation:
            return 'new_reference_required_after_stop'
        reason = self.admission_reason(now, wall)
        if not reason and self.sample('admission', now, wall).get('handover_pending') is True:
            return 'waiting_native_handover_pair'
        return reason

    def begin(self, now, wall):
        if self.active:
            return False, 'already_executing'
        reason = self.ready_reason(now, wall)
        if reason:
            self.reason = reason
            return False, reason
        self.context = self.sample('admission', now, wall)['localization_context']
        self.phase, self.reason = 'arming', 'waiting_sdk_arm'
        self.issued_at, self.started_at = wall, now
        self.arm_confirmed_at = self.arm_confirmed_wall = 0.
        self.task_generation = self.generation
        self.consumed_generation = self.generation
        self.mismatch_since = None
        self.matched_admission = None
        self.handover_active = False
        self.pair_wait_active = False
        self.pending_candidate_id = -1
        self.admission_highwater = -1
        self.admission_identity = None
        self.disarm_requested = False
        return True, self.reason

    def stop(self, reason, *, finished=False):
        self.phase = 'finished' if finished else 'locked'
        self.reason = reason
        self.consumed_generation = max(self.consumed_generation, self.generation)
        self.disarm_requested = True
        self.matched_admission = None
        self.handover_active = False
        self.pair_wait_active = False
        self.pending_candidate_id = -1
        self.admission_highwater = -1
        self.admission_identity = None

    def task(self):
        return dict(session_id=self.c.session_id, generation=self.task_generation,
                    active=self.active, frame_id=self.c.frame_id,
                    issued_at=self.issued_at, target_xyz=self.goal or [0., 0., 0.])

    def permit(self, now, wall):
        self.permit_seq += 1
        return dict(schema=1, session_id=self.c.session_id, map_version_id=self.c.map_version_id,
                    generation=max(1, self.generation), seq=self.permit_seq,
                    allow=self.active and self.phase != 'verifying_completion', received_at_unix=wall)

    def step(self, now, wall):
        zero = (0., 0., 0.)
        if not self.active:
            return zero, True
        t = self.sample('tracker', now, wall, .25)
        if (t.get('generation') == self.task_generation and t.get('frame_id') == self.c.frame_id
                and t.get('finished') is True and t.get('reason') == 'goal_reached'
                and t.get('stamp', 0.) >= self.issued_at):
            self.stop('goal_reached', finished=True)
            return zero, True
        if self.phase == 'verifying_completion':
            if now-self.completion_started_at > .25:
                self.stop('native_completed_without_measured_goal_confirmation')
            return zero, True
        problem = self.admission_reason(now, wall)
        if problem:
            a = self.sample('admission', now, wall)
            self.admission_failed(problem, a, now)
            return zero, True
        g = self.sample('gate', now, wall, .6)
        if self.phase == 'arming':
            if g.get('armed') is True and g.get('wall_time', 0.) >= self.issued_at:
                self.phase, self.reason = 'waiting_trajectory', 'waiting_new_native_trajectory'
                self.arm_confirmed_at, self.arm_confirmed_wall = now, wall
            elif now-self.started_at > 1.5:
                self.stop('sdk_arm_not_confirmed: '+str(g.get('arm_block_reason', 'missing_gate_status')))
            return zero, True
        if g.get('armed') is not True:
            self.stop('motion_gate_revoked: '+str(g.get('reason', 'stale_or_missing')))
            return zero, True
        a = self.sample('admission', now, wall)
        # This deadline begins with the first unpaired native candidate, not
        # the eventual paired admission or tracker ACK. Neither a later ID nor
        # a same-ID heartbeat can renew it. Even a late matching ACK must stop.
        if (self.mismatch_since is not None and (self.pair_wait_active or self.handover_active)
                and now-self.mismatch_since > .15):
            self.stop('native_handover_pair_timeout' if self.pair_wait_active else 'tracker_handover_timeout')
            return zero, True
        if a.get('handover_pending') is True or a['trajectory_id'] < self.pending_candidate_id:
            if self.mismatch_since is None:
                self.mismatch_since = now
            self.pair_wait_active = True
            self.reason = 'waiting_native_handover_pair'
            return zero, True
        matching = (t.get('generation') == self.generation and t.get('active') is True
                    and t.get('frame_id') == self.c.frame_id and t.get('reason') == 'tracking'
                    and t.get('trajectory_id') == a.get('trajectory_id')
                    and a.get('source_stamp', 0.) >= self.arm_confirmed_wall)
        if not matching:
            if self.mismatch_since is None:
                self.mismatch_since = now
            if self.safe_handover(a, t, now, wall):
                self.handover_active = True
                self.reason = 'safe_trajectory_handover'
                return self.tracker_command(t)
            # No native predecessor evidence means ZERO, even if the new
            # trajectory is accepted. This includes collision-triggered swaps.
            endpoint_wait = (t.get('generation') == self.generation and t.get('active') is True
                and t.get('reason') == 'local_segment_finished_waiting_replan')
            deadline = 2. if self.phase == 'waiting_trajectory' else 1.5 if endpoint_wait else .2
            if ((self.phase == 'waiting_trajectory' and now-self.arm_confirmed_at > deadline)
                    or (self.phase != 'waiting_trajectory' and now-self.mismatch_since > deadline)):
                self.stop('tracker_not_current: '+str(t.get('reason', 'stale_or_missing')))
            return zero, True
        self.mismatch_since = None
        self.handover_active = False
        self.pair_wait_active = False
        self.pending_candidate_id = -1
        self.phase, self.reason = 'executing', 'tracking'
        output = self.tracker_command(t)
        if self.active:
            a, receipt = self.samples['admission']
            self.matched_admission = (dict(a), receipt)
        return output

    def tracker_command(self, t):
        command = t.get('command', {})
        if not isinstance(command, dict):
            self.stop('invalid_tracker_velocity')
            return (0., 0., 0.), True
        v = tuple(command.get(k) for k in ('x', 'y', 'yaw'))
        if (not all(finite(x) for x in v) or not 0 <= v[0] <= self.c.max_speed
                or abs(v[1]) > 1e-9 or abs(v[2]) > self.c.max_yaw
                or math.hypot(*v[:2]) > 1.5 or type(t.get('execution_frozen')) is not bool):
            self.stop('invalid_tracker_velocity')
            return (0., 0., 0.), True
        return v, t['execution_frozen']
