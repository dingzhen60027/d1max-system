#!/usr/bin/env python3
"""Read-only localization replay instrumentation; never commands or seeds a robot.

Run in the same isolated ROS/Zenoh environment as the replay:
  python3 observe_replay.py --output /absolute/new/run/metrics --duration 180

SIGINT/SIGTERM finalize the report. No ground-truth accuracy is inferred from
ICP residual, output smoothness, or local-odometry low-motion intervals.
"""
from __future__ import annotations

import argparse
from collections import Counter
import csv
import json
import math
import operator
from pathlib import Path
import signal
import time


PREFIX = "/d1max/localization/"
# The matcher currently has no per-field freshness/quality-valid flag. These
# reasons occur only AFTER its current attempt evaluates nearest-neighbour
# inliers/RMSE. Earlier exits retain old/default values and must not enter a
# residual distribution. Unknown future reasons are conservatively excluded.
QUALITY_EVALUATED_REASONS = frozenset({
    "点云有效重叠不足或残差过大", "匹配完成时扫描已过期", "匹配偏移或残差超出门限",
    "匹配已连续确认", "候选匹配确认中",
})


def uint8_value(value):
    """ROS Humble IDL `byte` is bytes; other generators may expose an int."""
    if isinstance(value, (bytes, bytearray, memoryview)):
        if len(value) != 1:
            raise ValueError("Expected exactly one ROS byte")
        return bytes(value)[0]
    number = operator.index(value)
    if not 0 <= number <= 255:
        raise ValueError("ROS uint8 out of range")
    return number


def diagnostic_record(status, stamp):
    return {"name": status.name, "level": uint8_value(status.level),
            "message": status.message, "stamp": stamp,
            "values": {value.key: value.value for value in status.values}}


def percentile(values, fraction):
    values = sorted(v for v in values if math.isfinite(v))
    if not values:
        return None
    index = (len(values) - 1) * fraction
    left, right = math.floor(index), math.ceil(index)
    return values[left] + (values[right] - values[left]) * (index - left)


def distribution(values):
    finite = [float(v) for v in values if math.isfinite(float(v))]
    return {"n": len(finite), "median": percentile(finite, .5),
            "p95": percentile(finite, .95), "max": max(finite, default=None)}


def distance(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def numeric(value):
    try:
        number = float(value)
        return number if math.isfinite(number) else None
    except (TypeError, ValueError):
        return None


def matcher_quality_evaluated(record):
    values = record.get("values", {})
    return (all((numeric(values.get(key)) or 0.) > 0.
                for key in ("attempts", "source_points", "elapsed_ms"))
            and record.get("message") in QUALITY_EVALUATED_REASONS)


def clean_json(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {str(k): clean_json(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean_json(v) for v in value]
    return value


class Recorder:
    """Pure-Python collector, also usable without ROS for deterministic checks."""

    def __init__(self, output, status_timeout=1., low_linear=.03, low_angular=.03,
                 low_motion_seconds=5.):
        self.output = Path(output).expanduser().resolve()
        self.output.mkdir(parents=True, exist_ok=True)
        # Never append a new run onto an existing metrics result.
        self.events = (self.output / "metrics.jsonl").open("x", buffering=1)
        self.trajectory = (self.output / "trajectory.csv").open("x", newline="", buffering=1)
        self.csv = csv.writer(self.trajectory)
        self.csv.writerow(["topic", "elapsed_wall_s", "received_unix", "header_unix",
                           "frame", "child_frame", "x", "y", "z", "qx", "qy", "qz", "qw",
                           "vx", "vy", "vz", "wx", "wy", "wz"])
        self.started = time.monotonic()
        self.started_unix = time.time()
        self.status_timeout = status_timeout
        self.low_linear, self.low_angular = low_linear, low_angular
        self.low_motion_seconds = low_motion_seconds
        self.odom = {"local": [], "global": []}
        self.statuses, self.frontends, self.matcher = [], [], []
        self.rates, self.last = {}, {}
        self.verified = 0
        self.raw = 0
        self.invalid = Counter()
        self.last_matcher_key = None
        self.closed = False

    def record(self, kind, data, elapsed=None):
        elapsed = time.monotonic() - self.started if elapsed is None else elapsed
        self.events.write(json.dumps(clean_json({"kind": kind, "elapsed_wall_s": elapsed,
                          "received_unix": self.started_unix + elapsed, "data": data}),
                          ensure_ascii=False, allow_nan=False) + "\n")
        return elapsed

    def text(self, kind, text):
        try:
            data = json.loads(text)
            if not isinstance(data, dict):
                raise ValueError("JSON object required")
        except (ValueError, TypeError):
            self.invalid[kind] += 1
            return
        elapsed = self.record(kind, data)
        self.rates.setdefault(kind, []).append(elapsed)
        self.last[kind] = data
        if kind == "status":
            self.statuses.append((elapsed, data))
        elif kind == "lio_status":
            self.frontends.append((elapsed, data))
        elif kind == "verified":
            self.verified += 1

    def diagnostic(self, data):
        elapsed = self.record("diagnostic", data)
        if data["name"] == "d1max_localization/matcher":
            value = data["values"]
            # Diagnostics repeat the previous result; avoid treating repeats as
            # independent registration attempts in residual distributions.
            key = (value.get("attempts"), value.get("elapsed_ms"),
                   value.get("inlier_ratio"), value.get("inlier_rmse_m"), data["message"])
            if key != self.last_matcher_key:
                self.matcher.append((elapsed, data))
                self.last_matcher_key = key

    def odometry(self, kind, value):
        elapsed = time.monotonic() - self.started
        flat = list(value["position"]) + list(value["orientation"]) + list(value["linear"]) + list(value["angular"])
        if not all(math.isfinite(v) for v in flat + [value["stamp"]]):
            self.invalid[kind + "_odom"] += 1
            return
        self.odom[kind].append((elapsed, value))
        self.record(kind + "_odom", value, elapsed)
        self.csv.writerow([kind, elapsed, self.started_unix + elapsed, value["stamp"],
                           value["frame"], value["child_frame"], *flat])

    @staticmethod
    def rate_summary(times, elapsed):
        gaps = [b - a for a, b in zip(times, times[1:])]
        active_time = times[-1] - times[0] if len(times) > 1 else 0.
        return {"messages": len(times), "active_received_hz": (len(times) - 1) / active_time if active_time > 0 else None,
                "whole_observation_hz": len(times) / elapsed if elapsed > 0 else None,
                "reception_gap_s": distribution(gaps), "gaps_over_0_1_s": sum(g > .1 for g in gaps),
                "gaps_over_0_5_s": sum(g > .5 for g in gaps), "gaps_over_1_s": sum(g > 1 for g in gaps),
                "last_message_age_s": elapsed - times[-1] if times else None}

    def status_summary(self, elapsed):
        first_lock = next((t for t, v in self.statuses if v.get("localized") is True), None)
        buckets, after = Counter(), Counter()
        localized = 0.
        for index, (at, value) in enumerate(self.statuses):
            end = self.statuses[index + 1][0] if index + 1 < len(self.statuses) else elapsed
            fresh_end = min(end, at + self.status_timeout)
            state = str(value.get("state", "unknown"))
            buckets[state] += max(0., fresh_end - at)
            buckets["status_stale"] += max(0., end - fresh_end)
            if first_lock is not None and end > first_lock:
                live_duration = max(0., fresh_end - max(at, first_lock))
                after[state] += live_duration
                after["status_stale"] += max(0., end - max(fresh_end, first_lock))
                if value.get("localized") is True:
                    localized += live_duration
        if self.statuses:
            buckets["before_first_status"] = self.statuses[0][0]
        else:
            buckets["before_first_status"] = elapsed
        duration = elapsed - first_lock if first_lock is not None else None
        return {"first_localized_elapsed_s": first_lock, "time_by_state_s": dict(buckets),
                "after_first_lock_s": duration, "after_first_lock_state_s": dict(after),
                "time_weighted_localized_fraction_after_first_lock": localized / duration if duration and duration > 0 else None,
                "status_freshness_timeout_s": self.status_timeout,
                "last_status": self.last.get("status")}

    def odom_summary(self, kind, elapsed):
        rows = self.odom[kind]
        speed, angular, steps, step_velocity, source_gaps, source_age = [], [], [], [], [], []
        frame_transitions, nonpositive_stamps = 0, 0
        for received, value in rows:
            speed.append(distance(value["linear"], (0, 0, 0)))
            angular.append(distance(value["angular"], (0, 0, 0)))
            source_age.append(self.started_unix + received - value["stamp"])
        for (_, previous), (_, current) in zip(rows, rows[1:]):
            dt = current["stamp"] - previous["stamp"]
            if (current["frame"], current["child_frame"]) != (previous["frame"], previous["child_frame"]):
                frame_transitions += 1
                continue
            if dt <= 0:
                nonpositive_stamps += 1
                continue
            step = distance(previous["position"], current["position"])
            steps.append(step)
            step_velocity.append(step / dt)
            source_gaps.append(dt)
        return {**self.rate_summary([t for t, _ in rows], elapsed), "linear_speed_m_s": distribution(speed),
                "angular_speed_rad_s": distribution(angular), "consecutive_pose_step_m": distribution(steps),
                "pose_difference_speed_m_s": distribution(step_velocity), "source_stamp_gap_s": distribution(source_gaps),
                "source_stamp_age_at_reception_s": distribution(source_age), "frame_changes": frame_transitions,
                "nonpositive_source_stamp_intervals": nonpositive_stamps,
                "trajectory_length_m_including_corrections_and_gaps": sum(steps),
                "first": rows[0][1] if rows else None, "last": rows[-1][1] if rows else None}

    def low_motion_intervals(self):
        ranges, candidate = [], []
        for at, value in self.odom["local"]:
            low = distance(value["linear"], (0, 0, 0)) <= self.low_linear and distance(value["angular"], (0, 0, 0)) <= self.low_angular
            if candidate and (at - candidate[-1][0] > .2 or not low):
                if candidate[-1][0] - candidate[0][0] >= self.low_motion_seconds:
                    ranges.append(candidate)
                candidate = []
            if low:
                candidate.append((at, value))
        if candidate and candidate[-1][0] - candidate[0][0] >= self.low_motion_seconds:
            ranges.append(candidate)
        results = []
        for rows in ranges:
            start, end = rows[0][0], rows[-1][0]
            global_rows = [(t, v) for t, v in self.odom["global"] if start <= t <= end]
            item = {"start_elapsed_s": start, "end_elapsed_s": end, "duration_s": end - start,
                    "local_endpoint_displacement_m": distance(rows[0][1]["position"], rows[-1][1]["position"])}
            if len(global_rows) > 1:
                item["global_endpoint_displacement_m"] = distance(global_rows[0][1]["position"], global_rows[-1][1]["position"])
                item["global_max_excursion_from_first_m"] = max(distance(global_rows[0][1]["position"], v["position"]) for _, v in global_rows)
            results.append(item)
        return {"interpretation": "Candidate low-motion intervals inferred ONLY from local odometry twist, not independently verified stationary ground truth.",
                "linear_threshold_m_s": self.low_linear, "angular_threshold_rad_s": self.low_angular,
                "minimum_interval_s": self.low_motion_seconds, "max_allowed_reception_gap_s": .2, "intervals": results}

    def summary(self, elapsed=None):
        elapsed = time.monotonic() - self.started if elapsed is None else elapsed
        epochs, faults, previous_epoch, previous_fault = [], [], None, None
        for at, value in self.frontends:
            epoch, fault = value.get("epoch"), value.get("fault")
            if isinstance(epoch, int) and epoch > 0 and epoch != previous_epoch:
                epochs.append({"elapsed_s": at, "epoch": epoch})
                previous_epoch = epoch
            if fault and fault != previous_fault:
                faults.append({"elapsed_s": at, "fault": fault})
            previous_fault = fault
        quality = {}
        quality_results = [v for _, v in self.matcher if matcher_quality_evaluated(v)]
        for name in ("inlier_ratio", "inlier_rmse_m", "elapsed_ms", "information_ratio"):
            values = [numeric(v["values"].get(name)) for v in quality_results]
            quality[name] = distribution([v for v in values if v is not None])
        return clean_json({"schema": 1, "started_unix": self.started_unix, "elapsed_wall_s": elapsed,
            "read_only": True, "ground_truth_available": False,
            "limitations": ["ICP inlier RMSE is registration residual, NOT localization position accuracy.",
                            "Availability is wall-time weighted with stale statuses counted unavailable.",
                            "Header timestamps are supplied by the replay pipeline; their age is not clock calibration proof.",
                            "Pose steps include map corrections and unobserved motion over gaps, not all steps are faults.",
                            "Matcher distributions use changed diagnostic results, not complete per-attempt ground truth.",
                            "Matcher quality excludes unattempted, empty-source and uncompleted quality evaluation; absent samples are null, not zero error."],
            "localization": self.status_summary(elapsed),
            "streams": {k: self.rate_summary(v, elapsed) for k, v in self.rates.items()},
            "odometry": {k: self.odom_summary(k, elapsed) for k in self.odom},
            "frontend": {"epoch_changes_including_initial": epochs, "resets_after_first_epoch": max(0, len(epochs) - 1), "fault_events": faults},
            "matcher": {"changed_diagnostic_results": len(self.matcher), "reason_observations": dict(Counter(v["message"] for _, v in self.matcher)),
                        "quality_evaluated_results": len(quality_results),
                        "quality_excluded_results": len(self.matcher) - len(quality_results),
                        "quality_sampling_rule": "attempts > 0, source_points > 0, elapsed_ms > 0, current reason proves current attempt evaluated inliers/RMSE",
                        "quality": quality, "raw_pose_messages": self.raw, "verified_messages": self.verified,
                        "last_diagnostic": self.matcher[-1][1] if self.matcher else None},
            "low_motion_proxy": self.low_motion_intervals(), "invalid_messages": dict(self.invalid)})

    def write_summary(self):
        report = self.summary()
        temporary = self.output / "summary.tmp"
        temporary.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
        temporary.replace(self.output / "summary.json")
        return report

    def close(self):
        if not self.closed:
            self.write_summary()
            self.events.close()
            self.trajectory.close()
            self.closed = True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--duration", type=float, default=0., help="Seconds; 0 runs until interrupted")
    parser.add_argument("--status-timeout", type=float, default=1.)
    parser.add_argument("--low-linear", type=float, default=.03)
    parser.add_argument("--low-angular", type=float, default=.03)
    parser.add_argument("--low-motion-seconds", type=float, default=5.)
    args = parser.parse_args()
    if args.duration < 0 or min(args.status_timeout, args.low_linear, args.low_angular, args.low_motion_seconds) <= 0:
        parser.error("Duration must be nonnegative; other numeric parameters must be positive")
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from diagnostic_msgs.msg import DiagnosticArray

    recorder = Recorder(args.output, args.status_timeout, args.low_linear, args.low_angular, args.low_motion_seconds)
    stop = False
    def finish(_signal, _frame):
        nonlocal stop
        stop = True
    signal.signal(signal.SIGINT, finish)
    signal.signal(signal.SIGTERM, finish)
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("localization_replay_readonly_observer")
    subscriptions = []
    for kind, suffix in (("status", "status"), ("lio_status", "lio/status"),
                         ("navigation_status", "navigation/status"), ("verified", "fused_icp/pose_raw/verified")):
        subscriptions.append(node.create_subscription(String, PREFIX + suffix,
                             lambda msg, name=kind: recorder.text(name, msg.data), qos_profile_sensor_data))

    def on_odom(message, kind):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        v, w = message.twist.twist.linear, message.twist.twist.angular
        recorder.odometry(kind, {"stamp": message.header.stamp.sec + message.header.stamp.nanosec * 1e-9,
            "frame": message.header.frame_id, "child_frame": message.child_frame_id,
            "position": [p.x, p.y, p.z], "orientation": [q.x, q.y, q.z, q.w],
            "linear": [v.x, v.y, v.z], "angular": [w.x, w.y, w.z]})
    for kind in ("local", "global"):
        subscriptions.append(node.create_subscription(Odometry, PREFIX + "odometry/" + kind,
                             lambda msg, name=kind: on_odom(msg, name), qos_profile_sensor_data))

    def on_raw(message):
        recorder.raw += 1
        p = message.pose.pose.position
        elapsed = recorder.record("raw_pose", {"stamp": message.header.stamp.sec + message.header.stamp.nanosec * 1e-9,
                                   "frame": message.header.frame_id, "position": [p.x, p.y, p.z]})
        recorder.rates.setdefault("raw_pose", []).append(elapsed)
    subscriptions.append(node.create_subscription(PoseWithCovarianceStamped, PREFIX + "fused_icp/pose_raw", on_raw, qos_profile_sensor_data))

    def on_diagnostic(message):
        for status in message.status:
            try:
                recorder.diagnostic(diagnostic_record(status,
                    message.header.stamp.sec + message.header.stamp.nanosec * 1e-9))
            except (TypeError, ValueError):
                recorder.invalid["diagnostic"] += 1
    subscriptions.append(node.create_subscription(DiagnosticArray, "/diagnostics", on_diagnostic, qos_profile_sensor_data))
    next_report = time.monotonic()
    print(f"READ-ONLY localization observer -> {recorder.output}", flush=True)
    try:
        while rclpy.ok() and not stop:
            elapsed = time.monotonic() - recorder.started
            if args.duration and elapsed >= args.duration:
                break
            rclpy.spin_once(node, timeout_sec=.1)
            if time.monotonic() >= next_report:
                report = recorder.write_summary()
                current = recorder.last.get("status", {})
                counts = "/".join(str(len(recorder.odom[k])) for k in ("local", "global"))
                print(f"t={elapsed:.1f}s state={current.get('state', 'no_status')} localized={current.get('localized')} "
                      f"odom(local/global)={counts} verified={recorder.verified} "
                      f"after_lock_availability={report['localization']['time_weighted_localized_fraction_after_first_lock']}", flush=True)
                next_report = time.monotonic() + 5.
    finally:
        recorder.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print(f"Saved {recorder.output / 'summary.json'}", flush=True)


if __name__ == "__main__":
    main()
