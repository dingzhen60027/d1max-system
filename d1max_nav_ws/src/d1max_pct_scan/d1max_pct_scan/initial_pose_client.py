"""Bounded UI client state; the BT owner decides retirement and admission.

No estimator or task state is owned here. In particular PREPARE does not write
the mailbox; only an exact COMMIT reply can release one immutable seed command.
"""
from copy import deepcopy
import math


TRANSACTION = 'bt_initial_pose_transaction_v1'


class InitialPoseClient:
    def __init__(self):
        self.pending = None
        self.started = 0.
        self.operation = 0
        self.reason = ''
        self.current = None
        self.phase = 'idle'
        self.feedback_session = ''
        self.submitted = False
        self.outcome_source_ns = 0
        self.applied_identity = None

    @staticmethod
    def source_ns(stamp):
        return stamp.sec*1000000000 + stamp.nanosec

    def begin(self, request, command, monotonic):
        if not math.isfinite(monotonic):
            raise ValueError('初值事务时钟无效')
        if request.request_id != command['id'] or request.session_id != command['session_id']:
            raise ValueError('初值事务身份不匹配')
        if self.current is not None and (request.request_id == self.current.request_id
                or self.source_ns(request.body_pose.header.stamp)
                   <= self.source_ns(self.current.body_pose.header.stamp)):
            raise ValueError('初值意图时间或标识未更新，请重新给初值')
        # One latest immutable intent, never a coordinate-based duplicate gate.
        # Replacing UI state does not retire tasks or authorize application;
        # the new PREPARE still goes through the existing BT owner.
        self.pending = deepcopy((request, command))
        self.current = deepcopy(request)
        self.started = monotonic
        self.operation = request.PREPARE
        self.feedback_session = request.session_id
        self.phase = 'waiting_retirement'
        self.reason = '等待旧任务退役'
        self.submitted = False
        self.outcome_source_ns = 0
        self.applied_identity = None

    def owns(self, request):
        expected = self.current
        return (request is not None and expected is not None
                and request.request_id == expected.request_id
                and request.session_id == expected.session_id
                and request.source_map_sha256 == expected.source_map_sha256
                and request.body_pose == expected.body_pose)

    def matches(self, request):
        return (self.pending is not None and self.owns(request)
                and request.operation == self.operation)

    def request(self, monotonic):
        if self.pending is None:
            return None
        if not math.isfinite(monotonic) or not 0 <= monotonic-self.started < 10.:
            self.fail('初值事务已过期，未提交定位；可重新给初值', phase='timeout')
            return None
        request = deepcopy(self.pending[0])
        request.operation = self.operation
        return request

    def receive(self, request, response, monotonic):
        # Check identity BEFORE expiration/failure: an old RPC may not mutate
        # the replacement's pending state, even when its reply is an error.
        if not self.matches(request) or not self.request(monotonic):
            return None
        expected = self.pending[0]
        if response is None or response.schema_version != 1:
            self.fail('初值接口版本不匹配')
            return None
        if not response.accepted:
            if response.reason == 'initial_pose_retirement_pending':
                self.operation = request.PREPARE
                self.phase, self.reason = 'waiting_retirement', '等待旧任务退役'
                return None
            self.fail(response.reason)
            return None
        if request.operation == request.PREPARE:
            if response.application_authorized:
                self.fail('初值准入回执状态不一致')
            elif response.ready:
                self.operation = request.COMMIT
                self.phase, self.reason = 'committing', '旧任务已退役，等待初值提交回执'
            else:
                self.phase, self.reason = 'waiting_retirement', '等待旧任务退役'
            return None
        if request.operation != request.COMMIT or not response.ready or not response.application_authorized:
            self.fail('初值应用未获正式授权')
            return None
        command = deepcopy(self.pending[1])
        command['initial_pose_transaction'] = TRANSACTION
        command['source_map_sha256'] = expected.source_map_sha256
        command['intent_source_stamp_ns'] = str(expected.body_pose.header.stamp.sec*1000000000
                                               + expected.body_pose.header.stamp.nanosec)
        self.pending = None
        self.phase = 'awaiting_localization'
        self.reason = '已提交机身初值；等待对应定位结果'
        return command

    def fail(self, reason, *, request=None, phase='rejected'):
        # Unknown service result is not a known rejection or physical stop.
        # Never write a seed or restore authorization. The owner retains its
        # own timeout/isolation fence until the corresponding outcome.
        if request is not None and (not self.owns(request) or request.operation != self.operation):
            return
        self.pending = None
        self.phase = phase
        self.reason = reason

    def reject_input(self, reason, session_id):
        # A malformed newer gesture does not withdraw a valid pending intent.
        self.feedback_session = session_id
        self.phase, self.reason = 'rejected', str(reason)

    def waiting_service(self):
        if self.pending is not None:
            self.phase, self.reason = 'waiting_service', '等待行为树初值准入服务'

    def mark_submitted(self, request):
        if self.owns(request) and request.operation == request.COMMIT:
            self.submitted = True

    def observe_outcome(self, outcome, *, map_version_id, now_ns):
        """Correlate existing typed feedback for display only, never authority."""
        expected = self.current
        if (expected is None or not self.submitted or self.phase == 'complete'
                or outcome.schema_version != 1 or outcome.session_id != expected.session_id
                or outcome.request_id != expected.request_id
                or outcome.map_version_id != map_version_id
                or outcome.intent_source_stamp != expected.body_pose.header.stamp):
            return False
        stamp = outcome.source_stamp
        source = self.source_ns(stamp)
        if (stamp.sec < 0 or not 0 <= stamp.nanosec < 1000000000
                or source < self.source_ns(expected.body_pose.header.stamp)
                or source <= self.outcome_source_ns or source > now_ns+50000000):
            return False
        if outcome.accepted and outcome.applied:
            if outcome.localization_epoch <= 0 or not outcome.localization_seed_id:
                return False
            self.applied_identity = (expected.session_id, outcome.localization_epoch,
                                     outcome.localization_seed_id)
            self.phase, self.reason = 'awaiting_localization', '初值已接收，等待对应定位匹配确认'
        elif not outcome.accepted and not outcome.applied:
            self.phase, self.reason = 'rejected', '初值被拒绝：' + outcome.reason
        else:
            self.phase, self.reason = 'uncertain', '初值应用结果待核对：' + outcome.reason
        self.outcome_source_ns = source
        return True

    def observe_ready(self, identity, *, ready, source_ns):
        if (self.phase == 'awaiting_localization' and self.applied_identity is not None
                and ready and identity == self.applied_identity
                and source_ns >= self.outcome_source_ns):
            self.phase, self.reason = 'complete', ''

    def feedback(self):
        if self.phase in ('idle', 'complete'):
            return None
        return dict(session_id=self.feedback_session, phase=self.phase, reason=self.reason)
