"""Source-stamped MC stop evidence, independent of ROS, SDK and task ownership.

Thresholds are engineering defaults, NOT a measured braking/holding guarantee.
The caller must retain raw MotionData.time_stamp and supply a validated mapping
of that source clock. Republishing an old sample cannot renew this evidence.
"""
from dataclasses import dataclass
import math


def finite(value):
    return type(value) in (float, int) and math.isfinite(value)


@dataclass(frozen=True)
class StopPolicy:
    linear_threshold_mps: float = .03
    angular_threshold_radps: float = .05
    dwell_seconds: float = .30
    max_source_age_seconds: float = .25
    max_sample_gap_seconds: float = .20
    minimum_samples: int = 3
    physical_acceptance_verified: bool = False

    def __post_init__(self):
        for key in ('linear_threshold_mps', 'angular_threshold_radps', 'dwell_seconds',
                    'max_source_age_seconds', 'max_sample_gap_seconds'):
            value = getattr(self, key)
            if not finite(value) or not 0 < value <= 2.:
                raise ValueError('invalid_stop_policy:'+key)
        if (type(self.minimum_samples) is not int or not 3 <= self.minimum_samples <= 1000
                or type(self.physical_acceptance_verified) is not bool):
            raise ValueError('invalid_stop_policy')


@dataclass(frozen=True)
class MotionSample:
    sdk_session: str
    clock_epoch: str
    raw_stamp_ns: int
    source_time: float
    received_time: float
    velocity_body: tuple
    angular_velocity_body: tuple
    source_clock_verified: bool = False


class StopVerifier:
    """Nonzero blocked != stop submitted != measured stop confirmed."""
    def __init__(self, policy=None):
        self.policy = policy or StopPolicy()
        self.transaction_id = ''
        self.sdk_session = self.clock_epoch = ''
        self.requested_at = self.submitted_at = None
        self.raw_cutoff = self.last_raw = 0
        self.first_source = self.last_source = self.last_received = None
        self.samples = 0
        self.reason = 'stop_not_requested'

    def begin(self, *, transaction_id, sdk_session, clock_epoch, now, raw_cutoff=0):
        if (not transaction_id or not sdk_session or not clock_epoch or not finite(now)
                or type(raw_cutoff) is not int or raw_cutoff < 0):
            raise ValueError('invalid_stop_context')
        self.transaction_id, self.sdk_session, self.clock_epoch = transaction_id, sdk_session, clock_epoch
        self.requested_at, self.submitted_at = now, None
        self.raw_cutoff = self.last_raw = raw_cutoff
        self.first_source = self.last_source = self.last_received = None
        self.samples = 0
        self.reason = 'nonzero_blocked'

    def mark_submitted(self, transaction_id, now):
        # This is a successful transport-submission event, not RequestEffect
        # construction and not a robot stopped ACK.
        if (transaction_id != self.transaction_id or self.requested_at is None
                or not finite(now) or now < self.requested_at or self.submitted_at is not None):
            return False
        self.submitted_at = now
        self.first_source = None
        self.samples = 0
        self.reason = 'stop_submitted'
        return True

    def invalidate(self, reason):
        self.first_source = None
        self.samples = 0
        self.reason = reason

    def observe(self, sample, now):
        p = self.policy
        if self.requested_at is None or self.submitted_at is None:
            return False
        if (sample.sdk_session != self.sdk_session or sample.clock_epoch != self.clock_epoch
                or sample.source_clock_verified is not True):
            self.invalidate('stop_source_context_unverified')
            return False
        values = (*sample.velocity_body, *sample.angular_velocity_body)
        if (len(sample.velocity_body) != 3 or len(sample.angular_velocity_body) != 3
                or any(not finite(v) for v in (*values, sample.source_time, sample.received_time, now))
                or type(sample.raw_stamp_ns) is not int or sample.raw_stamp_ns <= 0):
            self.invalidate('stop_sample_invalid')
            return False
        if sample.raw_stamp_ns <= self.last_raw:
            # Ignore duplicates/reordering, but never advance last source time.
            self.reason = 'stop_duplicate_or_out_of_order'
            return False
        if (sample.source_time <= self.submitted_at
                or self.last_source is not None and sample.source_time <= self.last_source
                or not 0 <= now-sample.source_time <= p.max_source_age_seconds
                or not 0 <= now-sample.received_time <= p.max_source_age_seconds
                or sample.received_time < sample.source_time):
            self.invalidate('stop_source_stale_or_invalid')
            return False
        if (self.last_source is not None and
                abs((sample.source_time-self.last_source)-(sample.raw_stamp_ns-self.last_raw)*1e-9) > .02):
            self.invalidate('stop_source_clock_mapping_discontinuous')
            return False
        if self.last_source is not None and sample.source_time-self.last_source > p.max_sample_gap_seconds:
            self.invalidate('stop_sample_gap')
        self.last_source, self.last_received, self.last_raw = sample.source_time, sample.received_time, sample.raw_stamp_ns
        if (math.sqrt(sum(v*v for v in sample.velocity_body)) > p.linear_threshold_mps
                or math.sqrt(sum(v*v for v in sample.angular_velocity_body)) > p.angular_threshold_radps):
            self.invalidate('robot_still_moving')
            return False
        if self.first_source is None:
            self.first_source = sample.source_time
        self.samples += 1
        confirmed = self.confirmed(now)
        self.reason = 'measured_stop_confirmed' if confirmed else 'waiting_stationary_dwell'
        return confirmed

    def confirmed(self, now):
        return (finite(now) and self.first_source is not None and self.last_source is not None
            and self.samples >= self.policy.minimum_samples
            and self.last_source-self.first_source >= self.policy.dwell_seconds
            and 0 <= now-self.last_source <= self.policy.max_source_age_seconds
            and 0 <= now-self.last_received <= self.policy.max_source_age_seconds)

    def status(self, now):
        confirmed = self.confirmed(now)
        return dict(nonzero_blocked=self.requested_at is not None,
            stop_submitted=self.submitted_at is not None, measured_stop_confirmed=confirmed,
            physical_acceptance_verified=self.policy.physical_acceptance_verified,
            reason='measured_stop_confirmed' if confirmed else self.reason,
            source_stamp_ns=self.last_raw, stationary_samples=self.samples)
