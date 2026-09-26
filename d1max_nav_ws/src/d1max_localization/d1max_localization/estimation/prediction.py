"""Bounded scan-end posterior -> real-IMU propagation. Never a second LIO.

The posterior (including biases, gravity and acceleration scale) is immutable.
Each evaluation replays only newer IMU samples. Delayed LIO updates therefore
replace the anchor without mixing old/new integration histories or feeding a
prediction back into the scan matcher. Covariance growth is conservative and
explicitly approximate, not a second independent statistical measurement.
"""

from collections import deque
from dataclasses import dataclass
import math
from ..math_utils import Pose3, quaternion_multiply, rotate_vector
from .contracts import (
    MotionState,
    covariance,
    epoch_header,
    fresh,
    positive_time,
    sample_pose,
    vector,
)


def rotation_step(omega, dt):
    norm = math.sqrt(sum(x * x for x in omega))
    if norm < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    half = norm * dt * 0.5
    return (*(x * math.sin(half) / norm for x in omega), math.cos(half))


@dataclass(frozen=True)
class PredictionLimits:
    max_horizon: float = 0.25
    max_extrapolation: float = 0.025
    # Total time without a new IMU endpoint, including max_extrapolation.
    # Default disables the additional kinematic tail for existing deployments.
    max_coast: float = 0.025
    coast_accel_sigma: float = 2.0  # m/s^2 unresolved world acceleration
    coast_angular_accel_sigma: float = 1.0  # rad/s^2 unresolved angular acceleration
    max_imu_gap: float = 0.05
    soft_imu_gap: float = 0.015
    max_gap_rotation: float = 0.08
    history_sec: float = 2.0


class InertialPredictor:
    def __init__(
        self, limits=PredictionLimits(), odom="d1max_loc_odom", tracking="d1max_loc_tracking"
    ):
        for key, value in vars(limits).items():
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError("invalid prediction limit " + key)
        if (
            not limits.max_extrapolation <= limits.max_imu_gap <= 0.1
            or limits.max_extrapolation > 0.025
            or not limits.max_extrapolation <= limits.max_coast <= 0.1
            or not 2.0 <= limits.coast_accel_sigma <= 20.0
            or not 1.0 <= limits.coast_angular_accel_sigma <= 20.0
            or limits.soft_imu_gap > limits.max_imu_gap
            or limits.max_horizon > 0.5
            or limits.history_sec > 5.0
        ):
            raise ValueError("unbounded inertial prediction")
        self.limits = limits
        self.odom = odom
        self.tracking = tracking
        self.imu = deque(maxlen=1024)
        self.epoch = 0
        self.valid = False
        self.fault = ""
        self.snapshot = None
        self.snapshot_publication = self.snapshot_receipt = None
        self.reason = "waiting_lio"
        self.imu_received = 0
        self.rejected = 0
        self.last_evaluation = 0.0
        self.degraded = False
        self.gap_stats = self.empty_gap_stats()
        self.prediction_mode = "unavailable"
        self.coast_stats = self.empty_coast_stats()

    @staticmethod
    def empty_coast_stats():
        return dict(unsupported_sec=0.0, imu_hold_sec=0.0, duration_sec=0.0,
                    rotation_rad=0.0, rejected_reason="")

    @staticmethod
    def empty_gap_stats():
        # Per evaluation/posterior, not cumulative: replaying the same posterior
        # must not inflate counters or uncertainty on each timer tick.
        return dict(count=0, max_sec=0.0, max_rotation_rad=0.0,
                    integrated_sec=0.0, rejected_reason="")

    def push_imu(self, stamp, acceleration, angular, now):
        stamp = positive_time(stamp)
        a = vector(acceleration, bound=200.0)
        w = vector(angular, bound=30.0)
        if not fresh(stamp, now, 0.2):
            self.rejected += 1
            return False
        if self.imu and stamp <= self.imu[-1][0]:
            if stamp < self.imu[-1][0] - 0.05:
                self.fault = "imu_clock_reset"
            self.rejected += 1
            return False
        self.imu.append((stamp, a, w))
        self.imu_received += 1
        while len(self.imu) > 2 and stamp - self.imu[1][0] > self.limits.history_sec:
            self.imu.popleft()
        return True

    def accept(self, value, now):
        epoch, valid = epoch_header(value, now)
        if epoch < self.epoch:
            return False
        snapshot = None
        if valid:
            if value["frame"] != self.odom or value["child_frame"] != self.tracking:
                raise ValueError("snapshot frame mismatch")
            stamp = positive_time(int(value["stamp_ns"]) * 1e-9)
            if not fresh(stamp, now, self.limits.max_horizon):
                return False
            inertial = value["inertial"]
            if inertial.get("schema") != 1:
                raise ValueError("missing inertial posterior")
            pose = sample_pose(value)
            velocity = vector(inertial["world_velocity"], bound=20.0)
            gyro_bias = vector(inertial["gyro_bias"], bound=2.0)
            accel_bias = vector(inertial["accel_bias"], bound=10.0)
            gravity = vector(inertial["gravity"], bound=12.0)
            scale = float(inertial["accel_scale"])
            if not 0.8 <= scale <= 1.2 or not 8.5 < math.sqrt(sum(g * g for g in gravity)) < 11.0:
                raise ValueError("invalid gravity or acceleration scale")
            pc = covariance(value["pose_covariance"], (0.0004,) * 3 + (0.0001,) * 3)
            tc = covariance(value["twist_covariance"], (0.0025,) * 3 + (0.0004,) * 3)
            snapshot = (stamp, pose, velocity, gyro_bias, accel_bias, gravity, scale, pc, tc)
            if epoch == self.epoch and self.snapshot and stamp <= self.snapshot[0]:
                return False
        if epoch > self.epoch:
            self.epoch = epoch
            self.snapshot = None
            self.snapshot_publication = self.snapshot_receipt = None
            self.fault = ""
            # Preserve recent independently received IMU samples. Only samples
            # after the NEW posterior stamp will be used, never the old state.
        self.valid = valid
        if value["fault"]:
            self.fault = value.get("reason", "lio_fault")
        if snapshot:
            self.snapshot = snapshot
            self.snapshot_publication = float(value["received_at_unix"])
            self.snapshot_receipt = now
        self.reason = value.get("reason", "waiting_lio")
        return True

    def timing(self, now):
        """Original posterior/IMU times, not refreshed by prediction ticks."""
        source = self.snapshot[0] if self.snapshot else None
        imu = self.imu[-1][0] if self.imu else None
        return {
            "posterior_stamp_sec": source,
            "posterior_age_sec": now - source if source is not None else None,
            "posterior_processing_sec": (self.snapshot_publication - source
                                         if source is not None else None),
            "posterior_transport_sec": (self.snapshot_receipt - self.snapshot_publication
                                        if self.snapshot_receipt is not None else None),
            "posterior_receipt_age_sec": (now - self.snapshot_receipt
                                          if self.snapshot_receipt is not None else None),
            "imu_stamp_sec": imu,
            "imu_age_sec": now - imu if imu is not None else None,
            "propagation_limit_sec": self.limits.max_horizon,
        }

    def evaluate(self, now):
        self.degraded = False
        self.gap_stats = self.empty_gap_stats()
        self.prediction_mode = "unavailable"
        self.coast_stats = self.empty_coast_stats()
        if self.last_evaluation and now < self.last_evaluation - 0.05:
            self.fault = "host_clock_reset"
        self.last_evaluation = now
        if self.fault:
            self.reason = self.fault
            return None
        if not self.valid or self.snapshot is None:
            self.reason = "waiting_lio"
            return None
        t, pose, velocity, bg, ba, gravity, scale, pc, tc = self.snapshot
        if not fresh(t, now, self.limits.max_horizon, future=0.0):
            self.reason = "lio_stale"
            return None
        unsupported = max(0.0, now - self.imu[-1][0]) if self.imu else 0.0
        self.coast_stats["unsupported_sec"] = unsupported
        if not self.imu or unsupported > self.limits.max_coast + 1e-6:
            self.coast_stats["rejected_reason"] = "duration" if self.imu else "missing_imu"
            self.reason = "imu_stale"
            return None
        values = list(self.imu)
        left = next((i for i in range(len(values) - 1, -1, -1) if values[i][0] <= t), None)
        if left is None:
            self.reason = "imu_start_uncovered"
            return None
        p = pose.position
        q = pose.orientation
        v = velocity
        current = t
        angular = (0.0, 0.0, 0.0)
        gap_position_variance = gap_velocity_variance = gap_angle_variance = 0.0

        def advance(a, w, dt):
            nonlocal p, q, v, angular
            angular = tuple(w[i] - bg[i] for i in range(3))
            mid = quaternion_multiply(q, rotation_step(angular, dt * 0.5))
            force = tuple(a[i] * scale - ba[i] for i in range(3))
            rotated = rotate_vector(mid, force)
            acc = tuple(rotated[i] + gravity[i] for i in range(3))
            p = tuple(p[i] + v[i] * dt + 0.5 * acc[i] * dt * dt for i in range(3))
            v = tuple(v[i] + acc[i] * dt for i in range(3))
            q = quaternion_multiply(q, rotation_step(angular, dt))

        for i in range(left, len(values) - 1):
            ta, aa, wa = values[i]
            tb, ab, wb = values[i + 1]
            if tb <= current:
                continue
            end = min(tb, now)
            if end <= current:
                break
            gap = tb - ta
            turn = gap * max(math.sqrt(sum(x * x for x in wa)), math.sqrt(sum(x * x for x in wb)))
            if gap > self.limits.soft_imu_gap:
                self.gap_stats["count"] += 1
                self.gap_stats["max_sec"] = max(self.gap_stats["max_sec"], gap)
                self.gap_stats["max_rotation_rad"] = max(self.gap_stats["max_rotation_rad"], turn)
            if (
                # Source stamps are converted through float seconds; tolerate
                # at most 1us at the configured supported-interval boundary.
                gap > self.limits.max_imu_gap + 1e-6
                or gap > self.limits.soft_imu_gap
                and turn > self.limits.max_gap_rotation
            ):
                self.gap_stats["rejected_reason"] = (
                    "duration" if gap > self.limits.max_imu_gap + 1e-6 else "rotation"
                )
                self.reason = "imu_gap"
                return None
            if gap > self.limits.soft_imu_gap:
                self.degraded = True
                dt = end - current
                self.gap_stats["integrated_sec"] += dt
                # Approximate missing-motion envelope, not fabricated IMU or
                # an independent observation. Assume at least 2m/s² and .2rad/s
                # unresolved dynamics; endpoint changes can only increase it.
                # Carry the added velocity uncertainty to evaluation time.
                accel_sigma = max(2.0, scale * math.sqrt(sum((ab[j] - aa[j])**2 for j in range(3))))
                gyro_sigma = max(0.2, math.sqrt(sum((wb[j] - wa[j])**2 for j in range(3))))
                gap_velocity_variance += (accel_sigma * dt)**2
                gap_position_variance += (accel_sigma * dt * (0.5 * dt + now - end))**2
                gap_angle_variance += (gyro_sigma * dt)**2
            fraction = ((current + end) * 0.5 - ta) / gap
            a = tuple(aa[j] + fraction * (ab[j] - aa[j]) for j in range(3))
            w = tuple(wa[j] + fraction * (wb[j] - wa[j]) for j in range(3))
            advance(a, w, end - current)
            current = end
            if current >= now:
                break
        tail = max(0.0, now - current)
        coast = 0.0
        if tail:
            # The entire unsupported interval, not just the kinematic part,
            # has a rotation budget. A fast turn cannot coast unconditionally.
            angular = tuple(values[-1][2][i] - bg[i] for i in range(3))
            turn = unsupported * math.sqrt(sum(w * w for w in angular))
            self.coast_stats["rotation_rad"] = turn
            if turn > self.limits.max_gap_rotation + 1e-9:
                self.coast_stats["rejected_reason"] = "rotation"
                self.reason = "coast_rotation"
                return None
            # Old IMU acceleration is used for at most the ORIGINAL 25ms
            # deadline after that real endpoint, never for the whole coast.
            hold = min(tail, max(0.0, values[-1][0] + self.limits.max_extrapolation - current))
            if hold:
                advance(values[-1][1], values[-1][2], hold)
            coast = tail - hold
            self.coast_stats["imu_hold_sec"] = hold
            self.coast_stats["duration_sec"] = coast
            if coast > 0.0:
                # World velocity and bias-corrected body angular rate at the
                # end of the IMU hold. This advances the state; it is NOT an
                # old pose republished under a new timestamp or a fake IMU.
                p = tuple(p[i] + v[i] * coast for i in range(3))
                q = quaternion_multiply(q, rotation_step(angular, coast))
        coasting = coast > 1e-9
        self.degraded = self.degraded or coasting
        # Explicit diagonal process-noise envelope, conservative/approximate
        # rather than another independent sensor observation. No unsupported
        # acceleration measurement is integrated during the kinematic tail.
        coast_position_variance = 0.25 * self.limits.coast_accel_sigma**2 * coast**4
        coast_velocity_variance = self.limits.coast_accel_sigma**2 * coast**2
        coast_angle_variance = 0.25 * self.limits.coast_angular_accel_sigma**2 * coast**4
        coast_angular_variance = self.limits.coast_angular_accel_sigma**2 * coast**2
        horizon = now - t
        pc = list(pc)
        tc = list(tc)
        for i in range(3):
            pc[7 * i] += (0.04 * horizon * horizon + 0.25 * horizon**4
                          + gap_position_variance + coast_position_variance)
            pc[7 * (i + 3)] += (0.0025 * horizon * horizon
                                + gap_angle_variance + coast_angle_variance)
            tc[7 * i] += horizon * horizon + gap_velocity_variance + coast_velocity_variance
            tc[7 * (i + 3)] += 0.0025 * horizon + coast_angular_variance
            if self.gap_stats["count"]:
                tc[7 * (i + 3)] += 0.04  # .2rad/s missing-motion floor
        if not all(math.isfinite(x) for x in (*p, *q, *v, *angular)):
            self.fault = self.reason = "prediction_nonfinite"
            return None
        self.prediction_mode = "coasting" if coasting else "imu_propagation"
        self.reason = ("predicting_coast" if coasting else
                       "predicting_degraded_imu_gap" if self.degraded else "predicting")
        return MotionState(now, Pose3(p, q), v, angular, pc, tc, t, values[-1][0], unsupported)
