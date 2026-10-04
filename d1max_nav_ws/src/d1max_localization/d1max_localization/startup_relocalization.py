"""Pure startup state/identity/time contract, separate from coarse registration.

Never authorizes motion and never retries a global seed after a verified
localization has existed. A manual seed always wins over an in-flight result.
"""
from collections import deque
from dataclasses import dataclass
import math
import uuid
import numpy as np
from scipy.spatial.transform import Rotation
from .math_utils import Pose3, compose, inverse, interpolate_pose, pose_innovation


@dataclass(frozen=True)
class StartupConfig:
    stationary_window_s: float = .6
    stationary_speed_mps: float = .08
    stationary_angular_rps: float = .12
    max_source_age_s: float = .5
    max_propagation_m: float = .15
    max_propagation_rad: float = .10
    confirmation_timeout_s: float = 15.
    retry_limit: int = 2

    def __post_init__(self):
        for key, value in vars(self).items():
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError('invalid_startup_parameter:' + key)
        if (self.stationary_window_s < .5 or self.stationary_speed_mps > .15
                or self.stationary_angular_rps > .2 or self.max_source_age_s > .5
                or self.max_propagation_m > .25 or self.max_propagation_rad > .2
                or self.confirmation_timeout_s > 30 or type(self.retry_limit) is not int or self.retry_limit > 3):
            raise ValueError('unbounded_startup_policy')


class StartupRelocalization:
    def __init__(self, session_id, enabled=True, config=StartupConfig()):
        if not session_id:
            raise ValueError('missing_localization_session')
        self.session_id, self.enabled, self.config = session_id, enabled, config
        self.state = 'waiting_lio' if enabled else 'disabled'
        self.reason = self.state
        self.epoch = 0
        self.history = deque(maxlen=2048)
        self.request = None
        self.seed_id = None
        self.seed_at = None
        self.ever_seeded = False
        self.attempts = 0
        self.map_sha256 = None
        self.result = None

    def observe(self, epoch, stamp, pose, linear, angular):
        if epoch < self.epoch or not math.isfinite(stamp) or stamp <= 0:
            return
        self.reset_epoch(epoch)
        if self.history and stamp <= self.history[-1][0]:
            return
        self.history.append((stamp, pose, float(np.linalg.norm(linear)), float(np.linalg.norm(angular))))

    def reset_epoch(self,epoch):
        if epoch<=self.epoch:return
        self.history.clear();self.request=None
        if self.ever_seeded:self.state=self.reason='manual_required_after_local_reset'
        self.epoch=epoch

    def stationary(self, now):
        c = self.config
        values = [v for v in self.history if now - v[0] <= c.stationary_window_s + .3]
        if (len(values) < 3 or now - values[-1][0] > .3
                or values[-1][0] - values[0][0] < c.stationary_window_s
                or any(b[0] - a[0] > .25 for a, b in zip(values, values[1:]))):
            return False
        drift = pose_innovation(values[-1][1], values[0][1])
        return (all(v[2] <= c.stationary_speed_mps and v[3] <= c.stationary_angular_rps for v in values)
                and math.hypot(drift.translation_xy, drift.translation_z) <= .06 and drift.rotation <= .08)

    def capture(self, source_stamp, now):
        if self.attempts>=self.config.retry_limit:
            self.fail('global_search_attempt_limit_manual_available')
            return None
        if (not self.enabled or self.ever_seeded or self.request is not None
                or self.state == 'failed' or not self.map_sha256
                or not 0 <= now - source_stamp <= self.config.max_source_age_s or not self.stationary(now)):
            return None
        source_pose = interpolate_pose([(round(t * 1e9), p) for t, p, _, _ in self.history],
            round(source_stamp * 1e9), 250000000)
        if source_pose is None:
            return None  # Wait for a bracketing sample; never use latest TF.
        self.attempts += 1
        self.request = dict(request_id=uuid.uuid4().hex, session_id=self.session_id,
            epoch=self.epoch, source_stamp=source_stamp, local_pose=source_pose, map_sha256=self.map_sha256)
        self.state = self.reason = 'searching'
        return self.request

    def accept(self, result, now):
        r = self.request
        if (r is None or self.ever_seeded or r['epoch'] != self.epoch
                or result.get('request_id') != r['request_id']
                or result.get('map_sha256') != self.map_sha256):
            return None
        self.request = None
        self.result = {k: v for k, v in result.items() if k != 'kind'}
        if not result.get('accepted'):
            self.fail(result.get('reason', 'global_match_rejected'))
            return None
        if not self.history or not 0 <= now - self.history[-1][0] <= .3 or not self.stationary(now):
            self.fail('local_state_not_fresh_or_stationary')
            return None
        current = self.history[-1][1]
        delta = pose_innovation(current, r['local_pose'])
        if (math.hypot(delta.translation_xy, delta.translation_z) > self.config.max_propagation_m
                or delta.rotation > self.config.max_propagation_rad):
            self.fail('moved_during_global_search')
            return None
        try:
            matrix = np.asarray(result['candidate']['transform'], dtype=float)
            if (matrix.shape != (4, 4) or not np.isfinite(matrix).all()
                    or not np.allclose(matrix[3], (0, 0, 0, 1), atol=1.e-6)
                    or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1.e-5)
                    or not np.isclose(np.linalg.det(matrix[:3, :3]), 1., atol=1.e-5)):
                raise ValueError('not_a_rigid_transform')
            at_scan = Pose3(tuple(matrix[:3, 3]), tuple(Rotation.from_matrix(matrix[:3, :3]).as_quat()))
        except (ValueError, KeyError, TypeError):
            self.fail('invalid_global_candidate')
            return None
        # Source scan lives in TRACKING, not body/ground. Apply LIO motion once,
        # using the paired source sample; do not apply body-height/yaw offsets.
        return compose(compose(at_scan, inverse(r['local_pose'])), current)

    def mark_seed(self, seed_id, now, source):
        self.request = None
        self.ever_seeded = True
        self.seed_id, self.seed_at = seed_id, now
        self.state, self.reason = 'confirming', source + '_seed_requires_continuous_verification'

    def verification(self, active_seed, confirmed_seed, pose_valid, now):
        if self.state != 'confirming':
            return
        if active_seed == self.seed_id == confirmed_seed and pose_valid:
            self.state = self.reason = 'ready'
        elif now - self.seed_at > self.config.confirmation_timeout_s:
            self.fail('initial_pose_confirmation_timeout_manual_available')

    def fail(self, reason):
        self.request = None
        self.state, self.reason = 'failed', reason

    def retry(self):
        if not self.enabled or self.ever_seeded or self.state != 'failed' or self.attempts >= self.config.retry_limit:
            return False
        self.request = None
        self.state = self.reason = 'waiting_lio'
        return True

    def status(self):
        return dict(schema=1, state=self.state, reason=self.reason, attempts=self.attempts,
            automatic_initialization_only=True, session_id=self.session_id, local_epoch=self.epoch,
            map_sha256=self.map_sha256, seed_id=self.seed_id,
            request_id=self.request['request_id'] if self.request else None,
            manual_initial_pose_available=True, result=self.result)
