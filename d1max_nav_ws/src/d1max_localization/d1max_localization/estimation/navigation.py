"""Pure navigation admission and aligned history. No ROS/SDK side effects."""

from collections import deque
from dataclasses import dataclass
import math
from ..math_utils import Pose3, compose, inverse, interpolate_pose, pose_innovation
from .contracts import (
    MotionState,
    covariance,
    epoch_header,
    fresh,
    positive_time,
    sample_pose,
    vector,
)


@dataclass(frozen=True)
class NavigationLimits:
    local_timeout: float = 0.08
    correction_timeout: float = 0.75
    contract_timeout: float = 0.6
    filter_timeout: float = 0.08
    max_prediction_horizon: float = 0.25
    max_imu_age: float = 0.05
    max_history_gap: float = 0.08
    max_map_error: float = 0.5
    max_map_angle: float = 0.3
    max_correction_step: float = 0.10
    max_correction_angle: float = 0.08
    max_speed: float = 4.0
    max_angular_rate: float = 5.0
    reset_timeout: float = 2.0
    warmup_sec: float = 0.15


class NavigationState:
    def __init__(
        self,
        limits=NavigationLimits(),
        odom="d1max_loc_odom",
        tracking="d1max_loc_tracking",
        map_frame="d1max_loc_map",
    ):
        for key, value in vars(limits).items():
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError("invalid navigation limit " + key)
        if (
            limits.max_prediction_horizon > 0.5
            or limits.max_imu_age > 0.1
            or limits.reset_timeout > 5.0
        ):
            raise ValueError("unbounded navigation settings")
        self.limits = limits
        self.odom = odom
        self.tracking = tracking
        self.map_frame = map_frame
        self.epoch = 0
        self.local = deque(maxlen=256)
        self.filtered = deque(maxlen=256)
        self.local_valid = False
        self.map_contract = None
        self.contract_at = 0.0
        self.fault = ""
        self.reason = "waiting_local"
        self.filter_key = None
        self.reset_stamp = 0.0
        self.reset_ack_at = 0.0
        self.filter_fault = ""
        self.last_output = 0.0
        self.last_global = None
        self.last_local = None
        self.local_sequence = 0

    def push_local(self, value, now):
        epoch, valid = epoch_header(value, now, self.limits.local_timeout)
        if epoch < self.epoch:
            return False
        state = None
        if valid:
            if value["frame"] != self.odom or value["child_frame"] != self.tracking:
                raise ValueError("local frame mismatch")
            stamp = positive_time(int(value["stamp_ns"]) * 1e-9)
            source = positive_time(int(value["source_stamp_ns"]) * 1e-9)
            imu = positive_time(int(value["imu_stamp_ns"]) * 1e-9)
            extrapolation = float(value["extrapolation_sec"])
            if (
                not fresh(stamp, now, self.limits.local_timeout)
                or not fresh(source, now, self.limits.max_prediction_horizon)
                or not fresh(imu, now, self.limits.max_imu_age)
                or source > stamp
                or not 0 <= extrapolation <= 0.05
            ):
                return False
            state = MotionState(
                stamp,
                sample_pose(value),
                vector(value["world_velocity"], bound=20.0),
                vector(value["angular"], bound=20.0),
                covariance(value["pose_covariance"]),
                covariance(value["twist_covariance"]),
                source,
                imu,
                extrapolation,
            )
            if epoch == self.epoch and self.local and stamp <= self.local[-1].stamp:
                return False
        if epoch > self.epoch:
            self.epoch = epoch
            self.local.clear()
            self.filtered.clear()
            self.map_contract = None
            self.contract_at = 0.0
            self.filter_key = None
            self.reset_stamp = self.reset_ack_at = 0.0
            self.fault = self.filter_fault = ""
            self.last_global = self.last_local = None
            self.last_output = 0.0
        self.local_valid = valid
        if value["fault"]:
            self.fault = value.get("reason", "local_fault")
        if state:
            if self.fault:
                return False
            if self.local:
                previous = self.local[-1]
                dt = state.stamp - previous.stamp
                delta = pose_innovation(state.pose, previous.pose)
                if (
                    math.hypot(delta.translation_xy, delta.translation_z)
                    > 0.03 + self.limits.max_speed * dt
                    or delta.rotation > 0.03 + self.limits.max_angular_rate * dt
                ):
                    self.fault = "local_pose_jump"
                    self.local_valid = False
                    return False
            self.local.append(state)
            self.local_sequence += 1
        return True

    def local_ready(self, now):
        return (
            not self.fault
            and self.local_valid
            and bool(self.local)
            and fresh(self.local[-1].stamp, now, self.limits.local_timeout)
            and fresh(self.local[-1].source_stamp, now, self.limits.max_prediction_horizon)
            and fresh(self.local[-1].imu_stamp, now, self.limits.max_imu_age)
        )

    def accept_map(self, value, now):
        epoch, valid = epoch_header(value, now, self.limits.contract_timeout)
        arrival = float(value["received_at_unix"])
        if epoch != self.epoch or arrival <= self.contract_at:
            return False
        result = dict(value)
        if valid:
            if value["frame"] != self.map_frame or value["child_frame"] != self.tracking:
                raise ValueError("map frame mismatch")
            if (
                not isinstance(value.get("seed_id"), str)
                or not value["seed_id"]
                or value.get("confirmations", 0) < 3
            ):
                raise ValueError("unverified map seed")
            stamp = positive_time(int(value["stamp_ns"]) * 1e-9)
            if not fresh(stamp, now, self.limits.correction_timeout):
                return False
            result["pose"] = sample_pose(value)
            result["covariance"] = covariance(
                value["covariance"], (0.0225, 0.0225, 0.0625, 0.0144, 0.0144, 0.0225)
            )
            result["anchor"] = sample_pose(value["anchor"])
            result["stamp"] = stamp
        self.map_contract = result
        self.contract_at = arrival
        return True

    def map_ready(self, now):
        c = self.map_contract
        return bool(
            c
            and c["valid"]
            and c["epoch"] == self.epoch
            and fresh(self.contract_at, now, self.limits.contract_timeout)
            and fresh(c["stamp"], now, self.limits.correction_timeout)
        )

    def key(self):
        return (
            (self.epoch, self.map_contract["seed_id"])
            if self.map_contract and self.map_contract["valid"]
            else None
        )

    def begin_reset(self, now):
        if (
            not self.local_ready(now)
            or not self.map_ready(now)
            or self.filter_fault == "filter_reset_unknown"
        ):
            return None
        key = self.key()
        if key == self.filter_key:
            return None
        self.filter_fault = ""
        self.filter_key = key
        self.reset_stamp = self.local[-1].stamp
        self.reset_ack_at = 0.0
        self.filtered.clear()
        self.last_global = self.last_local = None
        return key, self.reset_stamp, compose(self.map_contract["anchor"], self.local[-1].pose)

    def reset_ack(self, key, stamp, now):
        if key != self.filter_key or stamp != self.reset_stamp or key != self.key():
            return False
        self.reset_ack_at = now
        return True

    def push_filtered(self, stamp, pose, pose_cov, linear, angular, now):
        if (
            not self.reset_ack_at
            or self.filter_key != self.key()
            or stamp <= self.reset_stamp
            or not fresh(stamp, now, self.limits.filter_timeout)
        ):
            return False
        if self.filtered and stamp <= self.filtered[-1][0]:
            return False
        self.filtered.append(
            (
                stamp,
                pose,
                covariance(pose_cov),
                vector(linear, bound=20.0),
                vector(angular, bound=20.0),
            )
        )
        return True

    def local_at(self, stamp):
        pose = interpolate_pose(
            [(round(s.stamp * 1e9), s.pose) for s in self.local],
            round(stamp * 1e9),
            round(self.limits.max_history_gap * 1e9),
        )
        if pose is None:
            return None
        left = None
        for state in self.local:
            if state.stamp == stamp:
                return state
            if state.stamp > stamp:
                if left is None:
                    return None
                ratio = (stamp - left.stamp) / (state.stamp - left.stamp)
                blend = lambda a, b: tuple(x + (y - x) * ratio for x, y in zip(a, b))
                return MotionState(
                    stamp,
                    pose,
                    blend(left.world_velocity, state.world_velocity),
                    blend(left.angular, state.angular),
                    state.pose_covariance,
                    state.twist_covariance,
                    min(left.source_stamp, state.source_stamp),
                    min(left.imu_stamp, state.imu_stamp),
                    max(left.extrapolation, state.extrapolation),
                )
            left = state
        return None

    def output(self, now):
        if not self.local_ready(now):
            self.reason = self.fault or "waiting_local"
            return None
        if not self.map_ready(now):
            self.reason = "waiting_map"
            return None
        if self.filter_fault:
            self.reason = self.filter_fault
            return None
        if (
            not self.reset_ack_at
            or now - self.reset_ack_at < self.limits.warmup_sec
            or self.filter_key != self.key()
            or len(self.filtered) < 2
        ):
            self.reason = "initializing_filter"
            return None
        if not fresh(self.filtered[-1][0], now, self.limits.filter_timeout):
            self.reason = "filter_stale"
            return None
        stamp = min(self.local[-1].stamp, self.filtered[-1][0])
        if stamp <= self.last_output:
            return None
        local = self.local_at(stamp)
        pose = interpolate_pose(
            [(round(t * 1e9), p) for t, p, *_ in self.filtered],
            round(stamp * 1e9),
            round(self.limits.max_history_gap * 1e9),
        )
        if local is None or pose is None:
            self.reason = "waiting_aligned_history"
            return None
        if not fresh(local.imu_stamp, now, self.limits.max_imu_age) or not fresh(
            local.source_stamp, now, self.limits.max_prediction_horizon
        ):
            self.reason = "aligned_history_stale"
            return None
        expected = compose(self.map_contract["anchor"], local.pose)
        error = pose_innovation(pose, expected)
        if (
            math.hypot(error.translation_xy, error.translation_z) > self.limits.max_map_error
            or error.rotation > self.limits.max_map_angle
        ):
            self.filter_fault = self.reason = "global_alignment_error"
            return None
        if self.last_global is not None:
            prediction = compose(self.last_global, compose(inverse(self.last_local), local.pose))
            correction = pose_innovation(pose, prediction)
            if (
                math.hypot(correction.translation_xy, correction.translation_z)
                > self.limits.max_correction_step
                or correction.rotation > self.limits.max_correction_angle
            ):
                self.filter_fault = self.reason = "global_correction_jump"
                return None
        self.last_output = stamp
        self.last_global = pose
        self.last_local = local.pose
        self.reason = "tracking"
        # Latest filter marginal with conservative floors. Interpolation is not
        # an additional independent observation or a calibrated error guarantee.
        pc = self.filtered[-1][2]
        pc = covariance(pc, (0.0225, 0.0225, 0.0625, 0.0144, 0.0144, 0.0225))
        return local, pose, pc
