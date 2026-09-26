#!/usr/bin/env python3
"""Compare the recorded initial segment; never mutate source CSV/configuration."""
import argparse
import json
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

from analyze_result import read_state


WORKSPACE = Path(__file__).resolve().parents[2]
DEFAULT_REFERENCE = WORKSPACE / "maps/runs/20260917_frontend_no_loop_correspondence_fix_02"
DEFAULT_CURRENT = WORKSPACE / "maps/runs/20260918_235958_803162_central_imu_faster_lio_zenoh"


def pcts(values, percentiles=(0, 50, 95)):
    values = np.asarray(values)
    values = values[np.isfinite(values)]
    return np.percentile(values, percentiles).tolist() if len(values) else None


def evaluate(data, start, end, extrinsic_rotation, extrinsic_translation):
    mask = (data["t"] >= start) & (data["t"] <= end)
    d = {k: v[mask] for k, v in data.items()}
    t = d["t"]-start
    xyz = np.column_stack([d[k] for k in ("x", "y", "z")])
    grav = np.column_stack([d[k] for k in ("gx", "gy", "gz")])
    quat = np.column_stack([d[k] for k in ("qx", "qy", "qz", "qw")])
    if not all(np.isfinite(x).all() for x in (t, xyz, grav, quat)):
        raise ValueError("Nonfinite pose/gravity/quaternion in comparison interval")
    if len(t) < 2:
        raise ValueError("Need at least two poses in each comparison interval")
    r_world_imu = Rotation.from_quat(quat).as_matrix()
    r_world_front = r_world_imu @ extrinsic_rotation
    front_position = xyz + np.einsum("nij,j->ni", r_world_imu, extrinsic_translation)
    tilt = np.degrees(np.arctan2(np.linalg.norm(grav[:, :2], axis=1), -grav[:, 2]))
    roll = np.degrees(np.arctan2(r_world_front[:, 2, 1], r_world_front[:, 2, 2]))
    pitch = np.degrees(np.arcsin(np.clip(-r_world_front[:, 2, 0], -1, 1)))
    up_tilt = np.degrees(np.arccos(np.clip(r_world_front[:, 2, 2], -1, 1)))
    ba = np.column_stack([d[k] for k in ("bax", "bay", "baz")])
    bg = np.column_stack([d[k] for k in ("bgx", "bgy", "bgz")])
    result = {
        "rows": len(t), "first_last_elapsed_s": t[[0, -1]].tolist(),
        "imu_origin_delta_z_m": float(xyz[-1, 2]-xyz[0, 2]),
        "imu_origin_z_min_max_span_m": [float(xyz[:, 2].min()), float(xyz[:, 2].max()), float(np.ptp(xyz[:, 2]))],
        "front_lidar_origin_delta_z_m": float(front_position[-1, 2]-front_position[0, 2]),
        "imu_origin_xy_path_length_m": float(np.linalg.norm(np.diff(xyz[:, :2], axis=0), axis=1).sum()),
        "gravity_tilt_first_last_max_deg": [float(tilt[0]), float(tilt[-1]), float(tilt.max())],
        "front_normalized_roll_first_last_deg": roll[[0, -1]].tolist(),
        "front_normalized_pitch_first_last_deg": pitch[[0, -1]].tolist(),
        "front_normalized_up_tilt_first_last_max_deg": [float(up_tilt[0]), float(up_tilt[-1]), float(up_tilt.max())],
        "features_min_median_p95": pcts(d["features"]),
        "horizontal_features_min_median_p95": pcts(d["horizontal_features"]),
        "horizontal_feature_fraction_median": float(np.median(d["horizontal_features"]/np.maximum(d["features"], 1))),
        "residual_rmse_median_p95_max_m": pcts(d["residual_rmse"], (50, 95, 100)),
        "accel_bias_first_last_imu_axes": ba[[0, -1]].tolist(),
        "accel_bias_last_norm": float(np.linalg.norm(ba[-1])),
        "gyro_bias_first_last_imu_axes": bg[[0, -1]].tolist(),
        "gyro_bias_change_norm": float(np.linalg.norm(bg[-1]-bg[0])),
        "samples_at_elapsed_s": [], "segments": []}
    times = sorted(set([v for v in (0., 30., 60., 120., 180., end-start) if v <= end-start]))
    for at in times:
        result["samples_at_elapsed_s"].append({
            "s": at, "imu_origin_delta_z_m": float(np.interp(at, t, xyz[:, 2])-xyz[0, 2]),
            "front_lidar_origin_delta_z_m": float(np.interp(at, t, front_position[:, 2])-front_position[0, 2]),
            "gravity_tilt_deg": float(np.interp(at, t, tilt)),
            "front_normalized_roll_deg": float(np.interp(at, t, roll)),
            "front_normalized_pitch_deg": float(np.interp(at, t, pitch))})
    for lo, hi in ((0., 30.), (30., 60.), (60., 120.), (120., end-start+1e-4)):
        m = (t >= lo) & (t < hi)
        if not m.any():
            continue
        result["segments"].append({"requested_elapsed_interval_s": [lo, hi], "rows": int(m.sum()),
            "residual_rmse_median_m": float(np.median(d["residual_rmse"][m])),
            "horizontal_features_median": float(np.median(d["horizontal_features"][m])),
            "horizontal_fraction_median": float(np.median(d["horizontal_features"][m]/np.maximum(d["features"][m], 1)))})
    series = {"t": t, "header_t": d["t"], "imu_dz": xyz[:, 2]-xyz[0, 2],
              "front_dz": front_position[:, 2]-front_position[0, 2]}
    return result, series


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference", type=Path, default=DEFAULT_REFERENCE)
    parser.add_argument("--current", type=Path, default=DEFAULT_CURRENT)
    parser.add_argument("--duration-sec", type=float, default=188.9)
    args = parser.parse_args()
    if args.duration_sec <= 0:
        raise ValueError("duration must be positive")
    reference, current = args.reference.resolve(strict=True), args.current.resolve(strict=True)
    old, old_validation = read_state(reference/"frontend_state.csv")
    new, new_validation = read_state(current/"frontend_state.csv")
    start = max(old["t"][0], new["t"][0])
    # The recorded frame at nominal 188.9s is 188.9000201s after the first frame.
    # Retain that frame with an explicitly documented 100us boundary tolerance.
    tolerance = 1e-4
    end = min(old["t"][-1], new["t"][-1], start+args.duration_sec+tolerance)
    cal_path = current/"config/calibration.yaml"
    extrinsic = yaml.safe_load(cal_path.read_text())["lio_extrinsic"]
    rotation = np.asarray(extrinsic["rotation"], dtype=float).reshape(3, 3)
    translation = np.asarray(extrinsic["translation"], dtype=float)
    if abs(np.linalg.det(rotation)-1)>1e-6 or np.linalg.norm(rotation.T@rotation-np.eye(3))>1e-6:
        raise ValueError("Current extrinsic rotation must be proper SO(3)")
    old_summary, old_series = evaluate(old, start, end, np.eye(3), np.zeros(3))
    new_summary, new_series = evaluate(new, start, end, rotation, translation)
    old_manifest = json.loads((reference/"manifest.json").read_text())
    new_manifest = json.loads((current/"manifest.json").read_text())
    old_hashes, new_hashes = old_manifest.get("binary_sha256", {}), new_manifest.get("binary_sha256", {})
    common_binaries = sorted(old_hashes.keys() & new_hashes.keys())
    common_stamps = np.intersect1d(old_series["header_t"], new_series["header_t"])
    same_count = len(old_series["t"]) == len(new_series["t"])
    grid = np.arange(0, end-start, .1)
    differences = {}
    for key in ("imu_dz", "front_dz"):
        diff = np.interp(grid, new_series["t"], new_series[key])-np.interp(grid, old_series["t"], old_series[key])
        differences[key] = {"definition": "current delta-Z minus reference delta-Z on 0.1s interpolated grid",
                            "median_m": float(np.median(diff)), "min_m": float(diff.min()),
                            "max_m": float(diff.max()), "rmse_m": float(np.sqrt(np.mean(diff**2)))}
    endpoint_difference = new_summary["front_lidar_origin_delta_z_m"]-old_summary["front_lidar_origin_delta_z_m"]
    result = {
        "schema": 1, "reference_run": str(reference), "current_run": str(current),
        "requested_duration_sec": args.duration_sec, "end_boundary_tolerance_sec": tolerance,
        "common_first_last_header_sec": [float(start), float(end)], "actual_common_span_sec": float(end-start),
        "timestamp_matching": {"exact_common_timestamps": len(common_stamps),
            "reference_unmatched_timestamps": len(old_series["t"])-len(common_stamps),
            "current_unmatched_timestamps": len(new_series["t"])-len(common_stamps),
            "all_timestamps_equal_in_order": bool(same_count and np.array_equal(old_series["header_t"], new_series["header_t"])),
            "max_rowwise_stamp_difference_sec_if_equal_counts": float(np.max(np.abs(old_series["header_t"]-new_series["header_t"]))) if same_count else None},
        "transform": {"calibration_source": str(cal_path),
            "formula": "p_W_front(t)=p_W_IMU(t)+R_W_IMU(t)*t_IMU_front; R_W_front(t)=R_W_IMU(t)*R_IMU_front; compare each curve after subtracting its own first Z.",
            "current_R_IMU_front": rotation.tolist(), "current_t_IMU_front_m": translation.tolist(),
            "reference_R_IMU_front": np.eye(3).tolist(), "reference_t_IMU_front_m": [0., 0., 0.],
            "note": "Uses the current experiment's nominal lever arm, not a surveyed translation. No map/trajectory files or world coordinates are modified. Roll/pitch use common normalized-front axes; world yaw is not aligned."},
        "reference_front_imu": old_summary, "current_central_imu": new_summary,
        "current_minus_reference_front_origin_endpoint_delta_z_m": float(endpoint_difference),
        "interpolated_height_curve_differences": differences,
        "csv_validation": {"reference": old_validation, "current": new_validation},
        "software_evidence": {"reference_binary_sha256": old_hashes, "current_binary_sha256": new_hashes,
            "common_binary_paths_equal_hash": {p: old_hashes[p] == new_hashes[p] for p in common_binaries},
            "reference_replay_rate": old_manifest.get("rate"), "current_replay_rate": new_manifest.get("rate")},
        "summary": {
            "observation": "The two initial raw height and gravity-direction trajectories are very similar. Switching to central IMU with this experiment's calibration did not visibly eliminate the early downward height trend.",
            "front_origin_endpoint_dz_reference_current_m": [old_summary["front_lidar_origin_delta_z_m"], new_summary["front_lidar_origin_delta_z_m"]],
            "gravity_tilt_endpoint_reference_current_deg": [old_summary["gravity_tilt_first_last_max_deg"][1], new_summary["gravity_tilt_first_last_max_deg"][1]]},
        "limitations": [
            "This compares the Sept17 21:42 interrupted correspondence-fix run, not the Sept17 19:01 full-loop run.",
            "Matching recorded executable/build-library hashes support a shared build, but the reference lacks the installed-library hash and actual loaded-library evidence; identical runtime code cannot be established completely.",
            "Replay rates differ (reference 2.5x, current 1x), as do IMU source, IMU origin, extrinsics and time compensation. This is not a controlled single-variable experiment.",
            "The central lever arm is nominal. Reconstructed front-LiDAR positions inherit its uncertainty.",
            "Estimated Z change is not ground-truth drift. Terrain, robot motion, initialization and estimator error have not been separated.",
            "Bias components are expressed in different IMU axes and should not be compared componentwise without transformation.",
            "Only complete newline-terminated CSV rows are parsed; any live partial final row is discarded in memory and recorded in csv_validation."]}
    output = current/"analysis/initial_segment_comparison.json"
    output.parent.mkdir(exist_ok=True)
    output.write_text(json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False)+"\n")
    print(str(output))


if __name__ == "__main__":
    main()
