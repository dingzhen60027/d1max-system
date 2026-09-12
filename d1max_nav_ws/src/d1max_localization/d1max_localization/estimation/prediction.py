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
            not limits.max_extrapolation <= limits.max_imu_gap <= 0.05
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
        self.reason = "waiting_lio"
        self.imu_received = 0
        self.rejected = 0
        self.last_evaluation = 0.0

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
            self.fault = ""
            # Preserve recent independently received IMU samples. Only samples
            # after the NEW posterior stamp will be used, never the old state.
        self.valid = valid
        if value["fault"]:
            self.fault = value.get("reason", "lio_fault")
        if snapshot:
            self.snapshot = snapshot
        self.reason = value.get("reason", "waiting_lio")
        return True

    def evaluate(self, now):
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
        if not self.imu or now - self.imu[-1][0] > self.limits.max_extrapolation:
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
            gap = tb - ta
            turn = gap * max(math.sqrt(sum(x * x for x in wa)), math.sqrt(sum(x * x for x in wb)))
            if (
                gap > self.limits.max_imu_gap
                or gap > self.limits.soft_imu_gap
                and turn > self.limits.max_gap_rotation
            ):
                self.reason = "imu_gap"
                return None
            end = min(tb, now)
            if end <= current:
                break
            fraction = ((current + end) * 0.5 - ta) / gap
            a = tuple(aa[j] + fraction * (ab[j] - aa[j]) for j in range(3))
            w = tuple(wa[j] + fraction * (wb[j] - wa[j]) for j in range(3))
            advance(a, w, end - current)
            current = end
            if current >= now:
                break
        extrapolation = max(0.0, now - current)
        if extrapolation:
            if extrapolation > self.limits.max_extrapolation:
                self.reason = "imu_stale"
                return None
            advance(values[-1][1], values[-1][2], extrapolation)
        horizon = now - t
        pc = list(pc)
        tc = list(tc)
        for i in range(3):
            pc[7 * i] += 0.04 * horizon * horizon + 0.25 * horizon**4
            pc[7 * (i + 3)] += 0.0025 * horizon * horizon
            tc[7 * i] += horizon * horizon
            tc[7 * (i + 3)] += 0.0025 * horizon
        if not all(math.isfinite(x) for x in (*p, *q, *v, *angular)):
            self.fault = self.reason = "prediction_nonfinite"
            return None
        self.reason = "predicting"
        return MotionState(now, Pose3(p, q), v, angular, pc, tc, t, values[-1][0], extrapolation)
