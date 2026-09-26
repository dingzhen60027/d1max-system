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
from .continuity import continuous_local, resume_local, correct_pose, residual_covariance


@dataclass(frozen=True)
class NavigationLimits:
    local_timeout: float = 0.08
    correction_timeout: float = 0.75
    contract_timeout: float = 0.6
    filter_timeout: float = 0.08
    # The EKF and local motion callbacks are asynchronous. A delayed EKF
    # sample may be aligned against real local history, while a previously
    # verified correction may be held only within the map validity window.
    filter_input_timeout: float = 0.30
    alignment_hold_timeout: float = 0.60
    max_prediction_horizon: float = 0.25
    max_imu_age: float = 0.05
    max_coast: float = 0.025
    max_history_gap: float = 0.08
    max_map_error: float = 0.5
    max_map_angle: float = 0.3
    max_correction_step: float = 0.10
    max_correction_angle: float = 0.08
    correction_speed: float = 0.15
    correction_angular_speed: float = 0.10
    correction_time_constant: float = 0.30
    local_correction_speed: float = 0.15
    local_correction_angular_speed: float = 0.10
    max_local_residual: float = 0.25
    max_local_angle: float = 0.15
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
            or limits.filter_input_timeout < limits.filter_timeout
            or limits.filter_input_timeout > limits.alignment_hold_timeout
            or limits.alignment_hold_timeout > min(
                limits.contract_timeout, limits.correction_timeout, 0.75
            )
            or not 0.025 <= limits.max_coast <= limits.max_imu_age
            or limits.max_coast > 0.1
            or max(limits.correction_speed, limits.local_correction_speed) > 0.5
            or max(limits.correction_angular_speed, limits.local_correction_angular_speed) > 0.5
            or limits.correction_time_constant > 2.0
            or limits.max_local_residual > limits.max_map_error
            or limits.max_local_angle > limits.max_map_angle
            or limits.reset_timeout > 5.0
        ):
            raise ValueError("unbounded navigation settings")
        self.limits = limits
        self.odom = odom
        self.tracking = tracking
        self.map_frame = map_frame
        self.epoch = 0
        self.local = deque(maxlen=256)
        self.raw_history = deque(maxlen=256)
        self.filtered = deque(maxlen=256)
        self.local_valid = False
        self.map_contract = None
        self.contract_at = 0.0
        self.map_notice_at = 0.0
        self.map_soft_unavailable = False
        self.fault = ""
        self.reason = "waiting_local"
        self.filter_key = None
        self.reset_stamp = 0.0
        self.reset_ack_at = 0.0
        self.filter_fault = ""
        self.last_output = 0.0
        self.last_global = None
        self.last_local = None
        self.last_output_key = None
        self.output_attempt_reason = "waiting_local"
        self.local_sequence = 0
        self.raw_local = None
        self.alignment_target = None  # (original EKF stamp, map_from_odom, covariance)
        self.alignment_barrier = 0.0
        self.local_residual = self.map_residual = (0.0, 0.0)
        self.output_resume_count = 0

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
                # Integration spans source -> target, not source -> receipt.
                # A valid 247ms prediction does not become a 252ms prediction
                # after 5ms transport. Target TTL and current IMU age remain
                # independent checks; neither timestamp is ever refreshed.
                or not fresh(source, stamp, self.limits.max_prediction_horizon, future=0.)
                or not fresh(imu, now, self.limits.max_imu_age)
                or source > stamp
                or not 0 <= extrapolation <= self.limits.max_coast + 1e-6
            ):
                return False
            if extrapolation > 0.025 + 1e-6:
                # Longer support is an explicit kinematic prediction, never
                # ordinary measured IMU propagation with a relaxed timeout.
                if (value.get("prediction_mode") != "coasting"
                    or value.get("degraded") is not True
                    or value.get("reason") != "predicting_coast"
                    or abs(extrapolation - max(0., stamp-imu)) > 1e-5):
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
        same_epoch = epoch == self.epoch
        if epoch > self.epoch:
            self.epoch = epoch
            self.local.clear()
            self.raw_history.clear()
            self.filtered.clear()
            self.map_contract = None
            self.contract_at = 0.0
            self.map_notice_at = 0.0
            self.map_soft_unavailable = False
            self.filter_key = None
            self.reset_stamp = self.reset_ack_at = 0.0
            self.fault = self.filter_fault = ""
            self.last_global = self.last_local = None
            self.last_output = 0.0
            self.last_output_key = None
            self.raw_local = None
            self.alignment_target = None
            self.alignment_barrier = 0.0
            self.local_residual = self.map_residual = (0.0, 0.0)
        # No new estimate is possible while waiting for the next real IMU or
        # scan-end posterior. These non-fault statuses do not revoke a previous
        # real sample; local_ready still checks its ORIGINAL target and IMU TTL.
        # An invalid snapshot, clock fault, new epoch or any other reason does.
        pending_source = (
            same_epoch
            and not valid
            and not value["fault"]
            and value.get("reason") in ("imu_stale", "lio_stale")
        )
        if not pending_source:
            self.local_valid = valid
            if not valid:
                self.last_output_key = None
                # A missing local sample stops output immediately. It does
                # not by itself change the odom frame. The old alignment's
                # original source time continues to age while unavailable.
                if value["fault"]:
                    self.alignment_target = None
                    self.alignment_barrier = now
        if value["fault"]:
            self.fault = value.get("reason", "local_fault")
        if state:
            if self.fault:
                return False
            if self.raw_local is not None:
                previous = self.raw_local
                dt = state.stamp - previous.stamp
                delta = pose_innovation(state.pose, previous.pose)
                if (
                    math.hypot(delta.translation_xy, delta.translation_z)
                    > 0.03 + self.limits.max_speed * dt
                    or delta.rotation > 0.03 + self.limits.max_angular_rate * dt
                ):
                    self.fault = "local_pose_jump"
                    self.local_valid = False
                    self.last_output_key = None
                    return False
            raw = state
            if self.local:
                try:
                    if state.stamp-self.local[-1].stamp > self.limits.max_prediction_horizon:
                        state, self.local_residual = resume_local(
                            self.local[-1], self.raw_local, state, self.limits)
                        self.output_resume_count += 1
                        # resume_local preserves the same odom coordinates
                        # using a new measured LIO endpoint; do not require
                        # an unrelated same-rate EKF correction to resume.
                        self.last_output_key = None
                    else:
                        state, self.local_residual = continuous_local(self.local[-1], state, self.limits)
                except ValueError as error:
                    self.fault = str(error)
                    self.local_valid = False
                    self.last_output_key = None
                    return False
            self.raw_local = raw
            self.raw_history.append(raw)
            self.local.append(state)
            self.local_sequence += 1
        return True

    def local_ready(self, now):
        return (
            not self.fault
            and self.local_valid
            and bool(self.local)
            and fresh(self.local[-1].stamp, now, self.limits.local_timeout)
            and fresh(self.local[-1].source_stamp, self.local[-1].stamp,
                      self.limits.max_prediction_horizon, future=0.)
            and fresh(self.local[-1].imu_stamp, now, self.limits.max_imu_age)
        )

    def local_timing(self, now):
        """Keep propagation, retained-sample age and current IMU age distinct."""
        if not self.local:
            return {}
        state = self.local[-1]
        return {
            "target_stamp_sec": state.stamp,
            "posterior_stamp_sec": state.source_stamp,
            "imu_stamp_sec": state.imu_stamp,
            "propagation_sec": state.stamp - state.source_stamp,
            "target_age_sec": now - state.stamp,
            "posterior_age_sec": now - state.source_stamp,
            "imu_age_sec": now - state.imu_stamp,
            "propagation_limit_sec": self.limits.max_prediction_horizon,
            "target_timeout_sec": self.limits.local_timeout,
            "imu_timeout_sec": self.limits.max_imu_age,
        }

    def accept_map(self, value, now):
        epoch, valid = epoch_header(value, now, self.limits.contract_timeout)
        arrival = float(value["received_at_unix"])
        if epoch != self.epoch or arrival <= self.map_notice_at:
            return False
        # A same-seed, non-fault invalid status is not a new correction. Keep
        # the previous verified contract only until its ORIGINAL source and
        # receipt deadlines; a new seed or a fault still revokes immediately.
        previous = self.map_contract
        same_verified_seed = bool(
            previous and previous["valid"]
            and value.get("seed_id") == previous.get("seed_id")
            and value.get("confirmations", 0) >= 3
        )
        if not valid and not value["fault"] and same_verified_seed:
            self.map_notice_at = arrival
            self.map_soft_unavailable = True
            return True
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
            if same_verified_seed and stamp < previous["stamp"]:
                return False
        old_key = self.key()
        self.map_contract = result
        self.contract_at = arrival
        self.map_notice_at = arrival
        self.map_soft_unavailable = False
        if not valid or self.key() != old_key:
            self.last_output_key = None
            self.alignment_target = None
            self.alignment_barrier = now
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
        self.alignment_target = None
        self.map_residual = (0.0, 0.0)
        self.last_output_key = None
        return key, self.reset_stamp, compose(self.map_contract["anchor"], self.raw_local.pose)

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
            or not fresh(stamp, now, self.limits.filter_input_timeout)
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

    def local_at(self, stamp, *, raw=False):
        history = self.raw_history if raw else self.local
        pose = interpolate_pose(
            [(round(s.stamp * 1e9), s.pose) for s in history],
            round(stamp * 1e9),
            round(self.limits.max_history_gap * 1e9),
        )
        if pose is None:
            return None
        left = None
        for state in history:
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

    def _admission_reason(self, now):
        if not self.local_ready(now):
            return self.fault or "waiting_local"
        if not self.map_ready(now):
            return "waiting_map"
        if self.filter_fault:
            return self.filter_fault
        if (
            not self.reset_ack_at
            or now - self.reset_ack_at < self.limits.warmup_sec
            or self.filter_key != self.key()
            or len(self.filtered) < 2
        ):
            return "initializing_filter"
        if not fresh(self.filtered[-1][0], now, self.limits.alignment_hold_timeout):
            return "filter_stale"
        return ""

    def output_ready(self, now):
        """Whether the last actual output is usable, without publishing it again."""
        return bool(
            not self._admission_reason(now)
            and self.last_output_key is not None
            and self.last_output_key == self.key()
            and self.alignment_target is not None
            and fresh(self.alignment_target[0], now, self.limits.alignment_hold_timeout)
            and fresh(self.last_output, now, self.limits.local_timeout)
            and fresh(self.last_output, now, self.limits.filter_timeout)
        )

    def _no_output(self, reason, now):
        self.output_attempt_reason = reason
        self.reason = "tracking" if self.output_ready(now) else reason
        return None

    def update_alignment(self, now):
        """The asynchronous EKF supplies a correction, not the output clock.

        A target is computed at ONE historical instant. Only a fresh admitted
        target may subsequently be composed with a new local motion estimate.
        We never restamp a filtered pose or require two 50Hz clocks in phase.
        """
        sample = next((value for value in reversed(self.filtered)
                       if value[0] <= self.local[-1].stamp), None)
        if sample is None:
            return "waiting_aligned_history"
        stamp, pose, pc, *_ = sample
        if stamp <= self.alignment_barrier:
            return "waiting_new_alignment"
        if not fresh(stamp, now, self.limits.filter_input_timeout):
            return "aligned_output_stale"
        if self.alignment_target and stamp <= self.alignment_target[0]:
            return ""
        local = self.local_at(stamp)
        if local is None:
            return "waiting_aligned_history"
        # This state belongs to the selected EKF time, not the newest live
        # state. Validate its propagation support at that time. Current target
        # and IMU freshness remain mandatory in local_ready. The posterior
        # integration interval is bounded against that target, not recharged
        # with transport/history latency at each consumer callback.
        if not fresh(local.imu_stamp, stamp, self.limits.max_imu_age) or not fresh(
            local.source_stamp, stamp, self.limits.max_prediction_horizon
        ):
            return "aligned_history_stale"
        raw_local = self.local_at(stamp, raw=True)
        if raw_local is None:
            return "waiting_raw_history"
        # PCD's anchor belongs to original LIO posterior coordinates/estimates,
        # not the rate-limited public estimate. Never use smoothing lag as ICP
        # innovation. The output target BELOW is relative to public odometry.
        expected = compose(self.map_contract["anchor"], raw_local.pose)
        error = pose_innovation(pose, expected)
        if (
            math.hypot(error.translation_xy, error.translation_z) > self.limits.max_map_error
            or error.rotation > self.limits.max_map_angle
        ):
            self.filter_fault = "global_alignment_error"
            self.last_output_key = None
            return self.filter_fault
        if self.alignment_target is not None:
            # Check RAW corrections, not accumulated smoothing lag. Otherwise
            # a valid but deliberately gradual correction trips its own gate.
            prediction = compose(self.alignment_target[1], local.pose)
            correction = pose_innovation(pose, prediction)
            if (
                math.hypot(correction.translation_xy, correction.translation_z)
                > self.limits.max_correction_step
                or correction.rotation > self.limits.max_correction_angle
            ):
                self.filter_fault = "global_correction_jump"
                self.last_output_key = None
                return self.filter_fault
        self.alignment_target = (stamp, compose(pose, inverse(local.pose)), pc)
        return ""

    def output(self, now):
        admission = self._admission_reason(now)
        if admission:
            return self._no_output(admission, now)
        alignment_reason = self.update_alignment(now)
        if self.filter_fault:
            return self._no_output(self.filter_fault, now)
        if not self.alignment_target or not fresh(
            self.alignment_target[0], now, self.limits.alignment_hold_timeout
        ):
            return self._no_output(alignment_reason or "alignment_stale", now)
        local = self.local[-1]
        stamp = local.stamp
        if stamp <= self.last_output:
            return self._no_output("waiting_local_sample", now)
        _, target, pc = self.alignment_target
        raw_pose = pose = compose(target, local.pose)
        self.map_residual = (0.0, 0.0)
        if self.last_global is not None:
            prediction = compose(self.last_global, compose(inverse(self.last_local), local.pose))
            pose, self.map_residual = correct_pose(
                prediction, raw_pose, min(stamp-self.last_output, self.limits.local_timeout),
                self.limits.correction_speed, self.limits.correction_angular_speed,
                self.limits.correction_time_constant,
            )
            if (self.map_residual[0] > self.limits.max_map_error
                    or self.map_residual[1] > self.limits.max_map_angle):
                self.filter_fault = self.reason = "global_correction_diverged"
                self.last_output_key = None
                self.output_attempt_reason = self.reason
                return None
        self.last_output = stamp
        self.last_global = pose
        self.last_local = local.pose
        self.last_output_key = self.key()
        self.reason = "tracking"
        self.output_attempt_reason = "new_output"
        # Conservative bound, NOT independent fusion of two correlated poses.
        # Add an isotropic upper bound on the local 6D marginal; row-sum norm
        # bounds every eigenvalue and remains valid under frame rotation.
        pc = covariance(pc, (0.0225, 0.0225, 0.0625, 0.0144, 0.0144, 0.0225))
        uncertainty = max(sum(abs(v) for v in local.pose_covariance[6*i:6*i+6]) for i in range(6))
        for i in range(6):
            pc[7*i] += uncertainty
        age = max(0.0, stamp-self.alignment_target[0])
        for i in range(3):
            pc[7*i] += (0.2*age)**2
            pc[7*(i+3)] += (0.05*age)**2
        pc = residual_covariance(pc, *self.map_residual)
        return local, pose, pc
