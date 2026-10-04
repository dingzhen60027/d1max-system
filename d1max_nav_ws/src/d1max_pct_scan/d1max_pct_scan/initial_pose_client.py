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

    def begin(self, request, command, monotonic):
        if self.pending is not None:
            raise ValueError('初值正在退役或提交，请等待当前事务')
        if not math.isfinite(monotonic):
            raise ValueError('初值事务时钟无效')
        if request.request_id != command['id'] or request.session_id != command['session_id']:
            raise ValueError('初值事务身份不匹配')
        self.pending = deepcopy((request, command))
        self.started = monotonic
        self.operation = request.PREPARE
        self.reason = '等待旧任务退役'

    def request(self, monotonic):
        if self.pending is None:
            return None
        if not math.isfinite(monotonic) or not 0 <= monotonic-self.started < 10.:
            self.fail('初值事务已过期，未提交定位')
            return None
        request = deepcopy(self.pending[0])
        request.operation = self.operation
        return request

    def receive(self, request, response, monotonic):
        if self.pending is None or not self.request(monotonic):
            return None
        expected = self.pending[0]
        if (request.request_id != expected.request_id or request.session_id != expected.session_id
                or request.body_pose != expected.body_pose or request.operation != self.operation):
            return None  # Late or foreign replies never publish a seed.
        if response.schema_version != 1:
            self.fail('初值接口版本不匹配')
            return None
        if not response.accepted:
            if response.reason == 'initial_pose_retirement_pending':
                self.operation = request.PREPARE
                return None
            self.fail(response.reason)
            return None
        if request.operation == request.PREPARE:
            if response.application_authorized:
                self.fail('初值准入回执状态不一致')
            elif response.ready:
                self.operation = request.COMMIT
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
        self.reason = '已提交机身初值；等待对应定位结果'
        return command

    def fail(self, reason):
        # Unknown service result is not a known rejection or physical stop.
        # Never write a seed or restore authorization. The owner retains its
        # own timeout/isolation fence until the corresponding outcome.
        self.pending = None
        self.reason = reason
