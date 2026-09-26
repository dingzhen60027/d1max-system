#!/usr/bin/env python3
"""Read-only cross-floor loop audit for an existing Faster-LIO + SC-PGO run.

Usage:
    python3 audit_cross_floor_loops.py --run /path/to/maps/runs/RUN
    python3 audit_cross_floor_loops.py --run RUN1 --run RUN2 --json

Only the run's sc_pgo/{odom_poses,optimized_poses,times,loop_events} exports are
read. In-progress files are supported: unfinished final lines are ignored and
missing/lagging exports are reported as provisional. This is a trajectory
diagnostic, not surveyed proof of a floor's identity or elevation.
"""

import argparse
from collections import Counter
import csv
import io
import json
import math
from pathlib import Path


def completed_lines(path):
    if not path.is_file():
        return None, False
    data = path.read_bytes()
    if not data:
        return [], False
    lines = data.splitlines(keepends=True)
    partial = not lines[-1].endswith((b"\n", b"\r"))
    if partial:
        lines.pop()
    return [line.decode("utf-8", errors="replace").strip() for line in lines], partial


def read_poses(path, issues):
    lines, partial = completed_lines(path)
    if lines is None:
        issues.append(f"missing {path.name}")
        return [], False
    if partial:
        issues.append(f"{path.name}: incomplete final line ignored")
    poses = []
    for line_no, line in enumerate(lines, 1):
        try:
            values = [float(value) for value in line.split()]
        except ValueError:
            values = []
        if len(values) != 12 or not all(math.isfinite(value) for value in values):
            issues.append(f"{path.name}: invalid complete line {line_no}; stopped to preserve keyframe indices")
            break
        poses.append((values[3], values[7], values[11]))
    return poses, partial


def read_times(path, issues):
    lines, partial = completed_lines(path)
    if lines is None:
        return []
    if partial:
        issues.append("times.txt: incomplete final line ignored")
    times = []
    for line_no, line in enumerate(lines, 1):
        try:
            value = float(line)
        except ValueError:
            value = math.nan
        if not math.isfinite(value) or (times and value <= times[-1]):
            issues.append(f"times.txt: invalid/non-increasing line {line_no}; stopped")
            break
        times.append(value)
    return times


def read_events(path, issues):
    lines, partial = completed_lines(path)
    if lines is None:
        issues.append("missing loop_events.csv")
        return [], False
    if partial:
        issues.append("loop_events.csv: incomplete final line ignored")
    if not lines:
        return [], partial
    reader = csv.DictReader(io.StringIO("\n".join(lines) + "\n"))
    required = {"ros_time", "event", "history_keyframe", "current_keyframe", "value1", "value2"}
    if not required.issubset(reader.fieldnames or ()):
        issues.append("loop_events.csv: missing expected columns")
        return [], partial
    events = []
    for line_no, row in enumerate(reader, 2):
        try:
            item = {
                "event": row["event"],
                "history": int(row["history_keyframe"]),
                "current": int(row["current_keyframe"]),
                "ros_time": float(row["ros_time"]),
                "value1": float(row["value1"]),
                "value2": float(row["value2"]),
            }
            if not item["event"] or not all(math.isfinite(item[key]) for key in ("ros_time", "value1", "value2")):
                raise ValueError
        except (KeyError, TypeError, ValueError):
            issues.append(f"loop_events.csv: malformed complete line {line_no}; skipped")
            continue
        events.append(item)
    return events, partial


def cumulative_xy(poses):
    distances = [0.0]
    for previous, current in zip(poses, poses[1:]):
        distances.append(distances[-1] + math.hypot(current[0] - previous[0], current[1] - previous[1]))
    return distances


def pair_metrics(history, current, raw, optimized, distances, times):
    if history < 0 or current <= history or current >= len(raw):
        return None
    segment_xy = distances[current] - distances[history]
    raw_delta = tuple(raw[current][axis] - raw[history][axis] for axis in range(3))
    result = {
        "history": history, "current": current,
        "raw_delta_xyz_m": [round(value, 4) for value in raw_delta],
        "raw_xy_path_length_m": round(segment_xy, 4),
        "raw_vertical_change_per_xy_path": round(raw_delta[2] / segment_xy, 4) if segment_xy > 0.01 else None,
    }
    if current < len(optimized):
        opt_delta = tuple(optimized[current][axis] - optimized[history][axis] for axis in range(3))
        result["optimized_delta_xyz_m"] = [round(value, 4) for value in opt_delta]
        result["vertical_change_removed_m"] = round(raw_delta[2] - opt_delta[2], 4)
    if current < len(times):
        result["sensor_elapsed_sec"] = round(times[current] - times[history], 3)
    return result


def transition_like(metrics):
    if metrics is None:
        return False
    dz = abs(metrics["raw_delta_xyz_m"][2])
    xy = metrics["raw_xy_path_length_m"]
    return dz >= 2.0 and 4.0 <= xy <= 60.0 and dz / xy >= 0.08


def collapsed(metrics):
    if not transition_like(metrics) or "optimized_delta_xyz_m" not in metrics:
        return False
    raw_dz = abs(metrics["raw_delta_xyz_m"][2])
    opt_dz = abs(metrics["optimized_delta_xyz_m"][2])
    return opt_dz <= max(0.75, 0.30 * raw_dz)


def infer_transition(raw, distances, optimized, times):
    """Fallback when no accepted pair spans the climb; bounded index search."""
    best = None
    for current in range(len(raw)):
        for history in range(current - 1, max(-1, current - 601), -1):
            length = distances[current] - distances[history]
            if length > 60.0:
                break
            if length < 4.0:
                continue
            candidate = pair_metrics(history, current, raw, optimized, distances, times)
            if not transition_like(candidate):
                continue
            # Prefer a complete vertical change, then the shortest path for it.
            score = (abs(candidate["raw_delta_xyz_m"][2]), -length)
            if best is None or score > best[0]:
                best = (score, candidate)
    return best[1] if best else None


def audit(run):
    run = Path(run).expanduser().resolve()
    pgo = run / "sc_pgo" if (run / "sc_pgo").is_dir() else run
    run_root = pgo.parent if pgo.name == "sc_pgo" else run
    issues = []
    raw, raw_partial = read_poses(pgo / "odom_poses.txt", issues)
    optimized, opt_partial = read_poses(pgo / "optimized_poses.txt", issues)
    times = read_times(pgo / "times.txt", issues)
    events, events_partial = read_events(pgo / "loop_events.csv", issues)
    if len(raw) != len(optimized):
        issues.append(f"pose row counts differ ({len(raw)} raw, {len(optimized)} optimized); comparisons are provisional")
    distances = cumulative_xy(raw)
    accepted = []
    for event in events:
        if event["event"] != "accepted":
            continue
        metrics = pair_metrics(event["history"], event["current"], raw, optimized, distances, times)
        entry = {"loop_number": round(event["value1"]), "ros_time": event["ros_time"],
                 "history": event["history"], "current": event["current"],
                 "measurements": metrics, "transition_like": transition_like(metrics),
                 "height_collapsed": collapsed(metrics)}
        accepted.append(entry)
    first_floor_candidates = [entry for entry in accepted
                              if entry["history"] <= 5 and 450 <= entry["current"] <= 600]
    first_floor = min(first_floor_candidates, key=lambda entry: abs(entry["current"] - 525)) if first_floor_candidates else None
    suspicious = [entry for entry in accepted if entry["height_collapsed"]]
    ascent_pair = suspicious[0]["measurements"] if suspicious else infer_transition(raw, distances, optimized, times)
    rejects = []
    vertical_rejection_events = {"reject_vertical_transition", "reject_vertical_input"}
    generic_rejection_events = {"reject_correction", "reject_geometry"}
    if ascent_pair is not None:
        start, end = ascent_pair["history"], ascent_pair["current"]
        for event in events:
            if (event["event"] not in vertical_rejection_events | generic_rejection_events
                    or not start <= event["current"] <= end):
                continue
            raw_pair = pair_metrics(event["history"], event["current"], raw, optimized, distances, times)
            rejection = {"event": event["event"], "history": event["history"],
                         "current": event["current"], "raw_pair_delta_z_m":
                         raw_pair["raw_delta_xyz_m"][2] if raw_pair else None}
            if event["event"] in vertical_rejection_events:
                rejection.update(concentrated_excursion_evidence_m=event["value1"],
                                 icp_z_evidence_difference_m=event["value2"])
            else:
                rejection.update(value1=event["value1"], value2=event["value2"])
            rejects.append(rejection)
    vertical_rejects = [item for item in rejects if item["event"] in vertical_rejection_events]
    generic_rejects = [item for item in rejects if item["event"] in generic_rejection_events]
    complete = None
    result_path = run_root / "result.json"
    if result_path.is_file():
        try:
            complete = bool(json.loads(result_path.read_text())["complete"])
        except (OSError, ValueError, KeyError, TypeError):
            issues.append("result.json: completion state unreadable")
    return {
        "run": str(run_root), "pgo_directory": str(pgo), "run_complete": complete,
        "provisional": (complete is not True or raw_partial or opt_partial or events_partial
                        or len(raw) != len(optimized) or bool(issues)
                        or any(entry["measurements"] is None for entry in accepted)),
        "raw_pose_count": len(raw), "optimized_pose_count": len(optimized),
        "event_counts": dict(Counter(event["event"] for event in events)),
        "raw_z_range_m": round(max(pose[2] for pose in raw) - min(pose[2] for pose in raw), 4) if raw else None,
        "optimized_z_range_m": round(max(pose[2] for pose in optimized) - min(pose[2] for pose in optimized), 4) if optimized else None,
        "first_floor_closure": first_floor,
        "major_vertical_transition": ascent_pair,
        "suspected_cross_floor_accepted_loops": suspicious,
        "vertical_transition_rejections": {
            "window_keyframes": [ascent_pair["history"], ascent_pair["current"]] if ascent_pair else None,
            "counts": dict(Counter(item["event"] for item in rejects)),
            "vertical_evidence_gate_counts": dict(Counter(item["event"] for item in vertical_rejects)),
            "generic_geometry_correction_counts": dict(Counter(item["event"] for item in generic_rejects)),
            "with_abs_raw_pair_delta_z_at_least_1m": sum(
                item["raw_pair_delta_z_m"] is not None and abs(item["raw_pair_delta_z_m"]) >= 1.0
                for item in rejects),
            "recent_examples": rejects[-8:],
            "recent_vertical_evidence_examples": vertical_rejects[-8:],
            "interpretation": "reject_vertical_transition/input are explicit height-evidence gates: value1 is concentrated excursion evidence (m), value2 is |ICP z - evidence| (m). Generic reject_geometry/correction occurred during the transition window but their CSV rows do not establish which threshold failed.",
        },
        "issues": issues,
        "criteria": "Transition-like: |raw ΔZ|≥2 m over 4–60 m of travelled XY and |ΔZ|/XY≥0.08. Collapsed: |optimized ΔZ|≤max(0.75 m, 30% of |raw ΔZ|). These are diagnostic flags, not ground truth.",
    }


def print_report(report):
    print(f"\nRun: {report['run']}")
    print(f"Status: {'provisional' if report['provisional'] else 'complete'}; "
          f"keyframes raw/optimized {report['raw_pose_count']}/{report['optimized_pose_count']}; "
          f"accepted loops {report['event_counts'].get('accepted', 0)}")
    print(f"Z span raw/optimized: {report['raw_z_range_m']} / {report['optimized_z_range_m']} m")
    first = report["first_floor_closure"]
    if first and first["measurements"]:
        m = first["measurements"]
        opt = m.get("optimized_delta_xyz_m")
        print(f"First-floor closure #{first['loop_number']} {first['history']}↔{first['current']}: "
              f"raw ΔZ {m['raw_delta_xyz_m'][2]:+.2f} m over {m['raw_xy_path_length_m']:.1f} m XY, "
              f"optimized ΔZ {opt[2]:+.2f} m" if opt else "First-floor closure awaiting optimized pose rows")
    elif first:
        print("First-floor closure accepted, awaiting its pose rows")
    else:
        print("First-floor closure near 0↔525: not yet observed")
    major = report["major_vertical_transition"]
    if major:
        opt = major.get("optimized_delta_xyz_m")
        print(f"Major vertical transition {major['history']}↔{major['current']}: "
              f"raw ΔZ {major['raw_delta_xyz_m'][2]:+.2f} m over {major['raw_xy_path_length_m']:.1f} m XY"
              + (f", optimized ΔZ {opt[2]:+.2f} m" if opt else "; optimized rows pending"))
    else:
        print("Major vertical transition: not yet measurable")
    loops = report["suspected_cross_floor_accepted_loops"]
    print("Suspected accepted cross-floor collapses: " + (
        ", ".join(f"#{item['loop_number']} {item['history']}↔{item['current']}" for item in loops)
        if loops else "none measurable yet"))
    rejects = report["vertical_transition_rejections"]
    print(f"Rejections during transition window {rejects['window_keyframes']}: "
          f"height-evidence gates {rejects['vertical_evidence_gate_counts']}; "
          f"generic geometry/correction {rejects['generic_geometry_correction_counts']}. "
          f"{rejects['with_abs_raw_pair_delta_z_at_least_1m']} rejected pairs have |raw ΔZ|≥1 m")
    if rejects["recent_vertical_evidence_examples"]:
        item = rejects["recent_vertical_evidence_examples"][-1]
        print(f"Latest explicit height rejection {item['event']} "
              f"{item['history']}↔{item['current']}: excursion evidence "
              f"{item['concentrated_excursion_evidence_m']:.2f} m, "
              f"|ICP z - evidence| {item['icp_z_evidence_difference_m']:.2f} m")
    for issue in report["issues"]:
        print(f"Note: {issue}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--run", action="append", type=Path, required=True,
                        help="Run directory (repeat for several runs); sc_pgo directory is also accepted")
    parser.add_argument("--json", action="store_true", help="Print machine-readable JSON to stdout")
    args = parser.parse_args()
    reports = [audit(run) for run in args.run]
    if args.json:
        print(json.dumps(reports, ensure_ascii=False, indent=2))
    else:
        for report in reports:
            print_report(report)


if __name__ == "__main__":
    main()
