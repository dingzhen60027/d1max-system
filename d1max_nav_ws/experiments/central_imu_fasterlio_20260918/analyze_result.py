#!/usr/bin/env python3
"""Read-only Faster-LIO result inspection; outputs stay in RUN/analysis.

Default mode requires an already saved binary PCD. --state-only explicitly
permits an incomplete CSV snapshot and labels both artifacts as partial.
Nothing in this script runs ROS, changes a map, or assumes a flat true floor.
"""
import argparse
import csv
import io
import json
import math
from pathlib import Path

import numpy as np


def summary(x):
    x = np.asarray(x, dtype=float)
    x = x[np.isfinite(x)]
    if not len(x):
        return {"count": 0}
    return {"count": len(x), "min": float(x.min()), "median": float(np.median(x)),
            "max": float(x.max()), "mean": float(x.mean()),
            "p05": float(np.percentile(x, 5)), "p95": float(np.percentile(x, 95))}


def read_state(path):
    data = path.read_bytes()
    incomplete_tail = bool(data and not data.endswith(b"\n"))
    if incomplete_tail:
        data = data[:data.rfind(b"\n")+1]
    reader = csv.reader(io.StringIO(data.decode("utf-8")))
    header = next(reader)
    required = {"t", "x", "y", "z", "qx", "qy", "qz", "qw", "gx", "gy", "gz"}
    if not required.issubset(header):
        raise ValueError("State CSV lacks required columns: " + str(sorted(required-set(header))))
    rows, rejected = [], []
    for number, row in enumerate(reader, 2):
        try:
            if len(row) != len(header):
                raise ValueError("column count")
            rows.append([float(value) for value in row])
        except ValueError:
            rejected.append(number)
    if len(rows) < 2:
        raise ValueError("Need at least two complete state rows")
    matrix = np.asarray(rows)
    fields = {name: matrix[:, index] for index, name in enumerate(header)}
    validation = {"parsed_rows": len(rows), "malformed_complete_rows": len(rejected),
                  "malformed_line_examples": rejected[:20], "ignored_incomplete_final_line": incomplete_tail,
                  "incomplete_final_lines_dropped_in_memory": int(incomplete_tail),
                  "snapshot_bytes": len(data),
                  "nonfinite_cells_by_column": {name: int((~np.isfinite(v)).sum()) for name, v in fields.items()},
                  "all_numeric_cells_finite": bool(np.isfinite(matrix).all())}
    return fields, validation


def trajectory_metrics(fields, validation):
    xyz = np.column_stack([fields[k] for k in ("x", "y", "z")])
    gravity = np.column_stack([fields[k] for k in ("gx", "gy", "gz")])
    quaternion = np.column_stack([fields[k] for k in ("qx", "qy", "qz", "qw")])
    timestamps = fields["t"]
    valid = np.isfinite(xyz).all(axis=1)&np.isfinite(timestamps)
    indices = np.flatnonzero(valid)
    if len(indices) < 2:
        raise ValueError("Need at least two finite poses")
    first, last = indices[[0, -1]]
    adjacent = valid[:-1]&valid[1:]&(np.diff(timestamps)>0)
    delta = np.diff(xyz, axis=0)[adjacent]
    norm = np.linalg.norm(gravity, axis=1)
    gravity_valid = np.isfinite(gravity).all(axis=1)&(norm>0)
    tilt = np.full(len(xyz), np.nan)
    relative_tilt = np.full(len(xyz), np.nan)
    if gravity_valid.any():
        tilt[gravity_valid] = np.degrees(np.arctan2(np.linalg.norm(gravity[gravity_valid, :2], axis=1),
                                                   -gravity[gravity_valid, 2]))
        direction = gravity[gravity_valid]/norm[gravity_valid, None]
        relative_tilt[gravity_valid] = np.degrees(np.arccos(np.clip(direction@direction[0], -1, 1)))
    diagnostics = {}
    for column in ("features", "horizontal_features", "residual_rmse", "roll_variance", "pitch_variance", "z_variance"):
        if column in fields:
            diagnostics[column] = summary(fields[column])
    if "horizontal_features" in fields and "features" in fields:
        m = fields["features"] > 0
        diagnostics["horizontal_feature_fraction"] = summary(fields["horizontal_features"][m]/fields["features"][m])
    result = {
        "pose_count": len(timestamps), "finite_pose_count": int(valid.sum()),
        "first_last_header_sec": [float(timestamps[first]), float(timestamps[last])],
        "duration_sec": float(timestamps[last]-timestamps[first]),
        "nonpositive_timestamp_intervals": int(np.sum(np.diff(timestamps)<=0)),
        "timestamp_interval_sec": summary(np.diff(timestamps)),
        "start_xyz_m": xyz[first].tolist(), "endpoint_xyz_m": xyz[last].tolist(),
        "endpoint_minus_start_xyz_m": (xyz[last]-xyz[first]).tolist(),
        "endpoint_to_start_xy_distance_m": float(np.linalg.norm(xyz[last, :2]-xyz[first, :2])),
        "path_length_xy_m": float(np.linalg.norm(delta[:, :2], axis=1).sum()),
        "path_length_xyz_m": float(np.linalg.norm(delta, axis=1).sum()),
        "excluded_path_segments": int(len(xyz)-1-adjacent.sum()),
        "height_z_m": summary(xyz[valid, 2]),
        "height_span_m": float(np.ptp(xyz[valid, 2])),
        "height_endpoint_minus_start_m": float(xyz[last, 2]-xyz[first, 2]),
        "gravity_tilt_from_initial_world_down_z_deg": summary(tilt),
        "gravity_direction_change_from_first_gravity_deg": summary(relative_tilt),
        "gravity_first_last_xyz": gravity[np.flatnonzero(gravity_valid)[[0, -1]]].tolist() if gravity_valid.any() else None,
        "gravity_tilt_first_last_deg": tilt[gravity_valid][[0, -1]].tolist() if gravity_valid.any() else None,
        "gravity_norm": summary(norm), "quaternion_norm": summary(np.linalg.norm(quaternion, axis=1)),
        "diagnostics": diagnostics, "csv_validation": validation,
        "interpretation": "Height span and endpoint delta are raw estimated motion in this run's world frame, not ground-truth drift. Actual terrain, robot height, initial alignment and estimator error may all contribute."
    }
    return result, xyz, valid, tilt


def read_pcd(path, max_plot_points=350000):
    header = {}
    with path.open("rb") as stream:
        for _ in range(100):
            raw = stream.readline()
            if not raw:
                raise ValueError("Truncated PCD header")
            line = raw.decode("ascii").strip()
            if line and not line.startswith("#"):
                key, _, value = line.partition(" ")
                header[key] = value.split()
            if line.startswith("DATA "):
                payload_offset = stream.tell()
                break
        else:
            raise ValueError("No DATA line in PCD header")
    if header.get("DATA") != ["binary"]:
        raise ValueError("Expected map_capture binary PCD, got " + str(header.get("DATA")))
    names, sizes, types = header["FIELDS"], list(map(int, header["SIZE"])), header["TYPE"]
    counts = list(map(int, header.get("COUNT", ["1"]*len(names))))
    if not len(names) == len(sizes) == len(types) == len(counts):
        raise ValueError("Inconsistent PCD fields")
    dtype_fields = []
    for index, (name, size, kind, count) in enumerate(zip(names, sizes, types, counts)):
        if kind not in ("F", "I", "U") or size not in (1, 2, 4, 8):
            raise ValueError("Unsupported PCD field dtype")
        name = name if name not in {f[0] for f in dtype_fields} else f"padding_{index}"
        dtype_fields.append((name, "<"+{"F": "f", "I": "i", "U": "u"}[kind]+str(size), (count,)) if count > 1
                            else (name, "<"+{"F": "f", "I": "i", "U": "u"}[kind]+str(size)))
    dtype = np.dtype(dtype_fields)
    count = int(header["POINTS"][0])
    if count <= 0:
        raise ValueError("Empty saved map")
    expected_bytes = dtype.itemsize*count
    actual_bytes = path.stat().st_size-payload_offset
    if expected_bytes != actual_bytes:
        raise ValueError(f"PCD payload size mismatch: expected {expected_bytes}, actual {actual_bytes}")
    cloud = np.memmap(path, dtype=dtype, mode="r", offset=payload_offset, shape=(count,))
    lower, upper = np.full(3, np.inf), np.full(3, -np.inf)
    finite_count = 0
    for start in range(0, count, 500000):
        block = cloud[start:start+500000]
        xyz = np.column_stack([block[name] for name in ("x", "y", "z")])
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        finite_count += len(xyz)
        if len(xyz):
            lower, upper = np.minimum(lower, xyz.min(axis=0)), np.maximum(upper, xyz.max(axis=0))
    if finite_count == 0:
        raise ValueError("Saved map contains no finite XYZ points")
    stride = max(1, math.ceil(count/max_plot_points))
    sampled = cloud[::stride]
    sample = np.column_stack([sampled[name] for name in ("x", "y", "z")])
    sample = sample[np.isfinite(sample).all(axis=1)]
    result = {"path": str(path), "resolved_path": str(path.resolve()),
              "file_bytes": path.stat().st_size, "declared_points": count,
              "finite_xyz_points": finite_count, "nonfinite_xyz_points": count-finite_count,
              "bounds_min_xyz_m": lower.tolist(), "bounds_max_xyz_m": upper.tolist(),
              "extent_xyz_m": (upper-lower).tolist(),
              "plot_sample_points": len(sample), "plot_sample_stride": stride,
              "plot_sample_z_percentile_1_99_m": np.percentile(sample[:, 2], [1, 99]).tolist(),
              "sampling_note": "Counts and bounds use every point. Plot uses a deterministic stride sample; color scale clips at sample Z percentiles 1 and 99, without altering coordinates."}
    return result, sample


def floor_like_patches(sample, xyz, valid):
    from scipy.spatial import cKDTree
    tree = cKDTree(sample[:, :2])
    poses = xyz[valid]
    travelled = np.r_[0., np.cumsum(np.linalg.norm(np.diff(poses[:, :2], axis=0), axis=1))]
    ids = np.unique(np.searchsorted(travelled, np.arange(0, travelled[-1]+1e-9, 3.)))
    rng = np.random.default_rng(190918)
    output = []
    for index in ids:
        pose = poses[index]
        patch = sample[tree.query_ball_point(pose[:2], 1.)]
        patch = patch[(patch[:, 2]<pose[2]-.2)&(patch[:, 2]>pose[2]-1.)]
        item = {"path_distance_m": float(travelled[index]), "pose_xyz_m": pose.tolist(), "candidate_points": len(patch)}
        if len(patch) < 35:
            item["status"] = "insufficient_candidate_points"
            output.append(item)
            continue
        best = None
        for _ in range(120):
            p = patch[rng.choice(len(patch), 3, replace=False)]
            normal = np.cross(p[1]-p[0], p[2]-p[0])
            length = np.linalg.norm(normal)
            if length < 1e-6 or abs(normal[2])/length < np.cos(np.radians(20)):
                continue
            normal /= length
            mask = np.abs((patch-p[0])@normal)<.03
            if best is None or mask.sum()>best.sum():
                best = mask
        if best is None or best.sum()<30 or best.mean()<.35:
            item["status"] = "no_dominant_near_horizontal_plane"
        else:
            points = patch[best]
            center = points.mean(axis=0)
            _, _, vt = np.linalg.svd(points-center, full_matrices=False)
            normal = vt[-1]
            if normal[2]<0:
                normal = -normal
            xy_eigen = np.linalg.eigvalsh(np.cov(points[:, :2].T))
            if normal[2]<np.cos(np.radians(20)) or xy_eigen[0]<.015:
                item["status"] = "plane_too_steep_or_narrow"
            else:
                z = center[2] - np.dot(normal[:2], pose[:2]-center[:2])/normal[2]
                item.update(status="floor_like_plane_found", map_plane_z_at_pose_xy_m=float(z),
                            pose_minus_plane_height_m=float(pose[2]-z), normal_xyz=normal.tolist(),
                            plane_tilt_deg=float(np.degrees(np.arccos(normal[2]))),
                            inlier_count=int(best.sum()), inlier_fraction=float(best.mean()),
                            inlier_rmse_m=float(np.sqrt(np.mean(((points-center)@normal)**2))))
        output.append(item)
    found = [p for p in output if p["status"] == "floor_like_plane_found"]
    return {"method": "stride-sampled saved map; poses every 3m; XY radius 1m; height band 0.2-1.0m below estimated pose; RANSAC 3cm/20deg; no map transformation",
            "warning": "These are floor-like surfaces selected relative to the estimated trajectory. They are not surveyed floor truth, and accumulated maps may combine multiple passes. Furniture/platforms can satisfy the filter.",
            "patches_found": len(found), "patches_queried": len(output),
            "sampled_plane_z_m": summary([p["map_plane_z_at_pose_xy_m"] for p in found]),
            "pose_minus_plane_height_m": summary([p["pose_minus_plane_height_m"] for p in found]),
            "patches": output}


def plot_result(target, fields, xyz, valid, tilt, map_sample, metrics, partial):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import Normalize
    plt.rcParams.update({"font.size": 9, "axes.spines.top": False, "axes.spines.right": False})
    fig = plt.figure(figsize=(14, 8.5), constrained_layout=True)
    grid = fig.add_gridspec(2, 2, width_ratios=[1.12, 1.])
    top = fig.add_subplot(grid[:, 0])
    side = fig.add_subplot(grid[0, 1])
    time = fig.add_subplot(grid[1, 1])
    poses = xyz[valid]
    centered = poses[:, :2]-poses[0, :2]
    _, _, vt = np.linalg.svd(centered, full_matrices=False)
    principal = vt[0]
    if np.dot(centered[-1], principal)<0:
        principal = -principal
    if map_sample is not None:
        vmin, vmax = np.percentile(map_sample[:, 2], [1, 99])
        if vmax <= vmin:
            vmax = vmin+.01
        norm = Normalize(vmin=vmin, vmax=vmax, clip=True)
        colored = top.scatter(map_sample[:, 0], map_sample[:, 1], c=map_sample[:, 2], s=.18, cmap="viridis", norm=norm, rasterized=True)
        fig.colorbar(colored, ax=top, shrink=.58, label="Saved-map Z [m]; color clipped at 1/99 percentiles")
        coordinate = (map_sample[:, :2]-poses[0, :2])@principal
        side.scatter(coordinate, map_sample[:, 2], c=map_sample[:, 2], s=.18, cmap="viridis", norm=norm, rasterized=True)
    top.plot(poses[:, 0], poses[:, 1], color="#d83b40", linewidth=.9, label="Estimated trajectory")
    top.scatter(*poses[0, :2], c="#111111", s=42, marker="o", label="Start", zorder=5)
    top.scatter(*poses[-1, :2], c="#dc2626", s=55, marker="X", label="End", zorder=5)
    top.set(title="Top view: original world coordinates", xlabel="World X [m]", ylabel="World Y [m]")
    top.axis("equal")
    top.legend(loc="best", markerscale=1.)
    side.plot(centered@principal, poses[:, 2], color="#d83b40", linewidth=1., label="Estimated trajectory Z")
    side.set(title="Side view along principal XY direction", xlabel="Projected XY displacement [m]", ylabel="Original world Z [m]")
    side.legend(loc="best")
    elapsed = fields["t"]-fields["t"][np.flatnonzero(valid)[0]]
    time.plot(elapsed[valid], poses[:, 2], color="#d83b40", linewidth=1., label="Trajectory Z")
    time.set(title="Height and gravity direction over recorded time", xlabel="Time from first logged pose [s]", ylabel="Original world Z [m]")
    other = time.twinx()
    other.plot(elapsed, tilt, color="#1e75a6", linewidth=.9, alpha=.9, label="Gravity tilt")
    other.set_ylabel("Gravity tilt from initial world down-Z [deg]", color="#1e75a6")
    other.spines["right"].set_visible(True)
    time.legend([time.lines[0], other.lines[0]], ["Trajectory Z", "Gravity tilt"], loc="best")
    for ax in (top, side, time):
        ax.grid(alpha=.16)
    t = metrics["trajectory"]
    fig.suptitle(("PARTIAL STATE SNAPSHOT — " if partial else "SAVED MAP — ")+"Faster-LIO frontend diagnostics\n"+
                 f"{t['pose_count']:,} poses | {t['duration_sec']:.1f} s | XY path {t['path_length_xy_m']:.2f} m | endpoint ΔZ {t['height_endpoint_minus_start_m']:+.3f} m", fontsize=13)
    fig.supxlabel("Raw height change is not ground-truth drift. No floor flattening or map correction applied.", fontsize=10)
    fig.savefig(target, dpi=170)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--state-only", action="store_true", help="Explicit partial CSV snapshot; no saved map required")
    parser.add_argument("--ground-check", action="store_true", help="Optional floor-like local plane checks; not ground truth")
    args = parser.parse_args()
    run = args.run.resolve(strict=True)
    if not args.state_only and not (run/"scans.pcd").is_file():
        raise SystemExit("No saved scans.pcd: wait for map save, or explicitly use --state-only for a partial snapshot")
    fields, validation = read_state(run/"frontend_state.csv")
    trajectory, xyz, valid, tilt = trajectory_metrics(fields, validation)
    manifest = json.loads((run/"manifest.json").read_text()) if (run/"manifest.json").is_file() else {}
    status = json.loads((run/"status.json").read_text()) if (run/"status.json").is_file() else None
    metrics = {"schema": 1, "run": str(run), "mode": "partial_state_only" if args.state_only else "saved_map_inspection",
               "run_status_snapshot": status, "manifest_binary_sha256": manifest.get("binary_sha256"),
               "trajectory": trajectory,
               "limitations": ["A saved map alone does not establish that the full source bag completed; inspect the run status/result alongside these metrics.",
                               "Raw world Z or endpoint change is not surveyed drift; robot motion, terrain and estimator error are not separated here.",
                               "The original 20260917_190115 front-IMU run used a different frontend build. A comparison also changes IMU source and extrinsics, so it cannot isolate an IMU effect or establish equal gravity histories.",
                               "World yaw may differ between independently initialized runs; axes are not automatically aligned."]}
    sample = None
    if not args.state_only:
        metrics["saved_map"], sample = read_pcd(run/"scans.pcd")
        if args.ground_check:
            metrics["floor_like_patch_check"] = floor_like_patches(sample, xyz, valid)
    output = run/"analysis"
    output.mkdir(exist_ok=True)
    stem = "partial_state" if args.state_only else "frontend_result"
    summary_path, image_path = output/(stem+".json"), output/(stem+".png")
    plot_result(image_path, fields, xyz, valid, tilt, sample, metrics, args.state_only)
    metrics["diagnostic_image"] = str(image_path)
    summary_path.write_text(json.dumps(metrics, indent=2, ensure_ascii=False, allow_nan=False)+"\n")
    print(json.dumps({"metrics": str(summary_path), "image": str(image_path),
                      "duration_sec": trajectory["duration_sec"], "pose_count": trajectory["pose_count"],
                      "endpoint_delta_z_m": trajectory["height_endpoint_minus_start_m"]}))


if __name__ == "__main__":
    main()
