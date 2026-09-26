#!/usr/bin/env python3
"""Inspect saved LIO-SAM outputs; metrics are not ground-truth accuracy."""
import argparse
import json
from pathlib import Path
import re

import numpy as np


def pcd_summary(path):
    with path.open("rb") as f:
        header = {}
        while True:
            line = f.readline().decode("ascii").strip()
            if not line:
                raise ValueError("Missing PCD DATA marker")
            fields = line.split()
            if line.startswith("#"):
                continue
            header[fields[0]] = fields[1:]
            if fields[0] == "DATA":
                break
        if header["DATA"] != ["binary"] or header["FIELDS"] != ["x", "y", "z", "intensity"]:
            raise ValueError("Unexpected native GlobalMap schema")
        points = np.frombuffer(f.read(), dtype="<f4").reshape(-1, 4)
    expected = int(header["POINTS"][0])
    if len(points) != expected:
        raise ValueError("PCD payload count mismatch")
    return {"path": str(path), "points": len(points), "bytes": path.stat().st_size,
            "finite_xyz": bool(np.isfinite(points[:, :3]).all()),
            "min_xyz": points[:, :3].min(axis=0).tolist(), "max_xyz": points[:, :3].max(axis=0).tolist(),
            "content": "Native LIO-SAM corner/surface keyframe FEATURE map, not all raw LiDAR returns"}


def optimized_keyframes(path):
    """Read saved poses, rather than treating append-only online poses as final."""
    with path.open("rb") as f:
        header = {}
        while True:
            line = f.readline().decode("ascii").strip()
            if not line:
                raise ValueError("Missing keyframe PCD DATA marker")
            if line.startswith("#"):
                continue
            parts = line.split()
            header[parts[0]] = parts[1:]
            if parts[0] == "DATA":
                break
        names = ["x", "y", "z", "intensity", "roll", "pitch", "yaw", "time"]
        if (header["FIELDS"] != names or header["DATA"] != ["binary"]
                or header["SIZE"] != ["4"]*7+["8"]
                or header["TYPE"] != ["F"]*8 or header["COUNT"] != ["1"]*8):
            raise ValueError("Unexpected native keyframe schema")
        poses = np.frombuffer(f.read(), dtype=np.dtype([(name, "<f4" if name != "time" else "<f8") for name in names]))
    if len(poses) != int(header["POINTS"][0]) or not len(poses):
        raise ValueError("Empty/inconsistent keyframe PCD")
    xyz = np.column_stack([poses[name] for name in ("x", "y", "z")])
    return {"path": str(path), "keyframes": len(poses),
            "kind": "Final optimized keyframe poses at map export; not every scan",
            "finite_xyz": bool(np.isfinite(xyz).all()),
            "sensor_duration_s": float(poses["time"][-1]-poses["time"][0]),
            "estimated_endpoint_xyz_m": (xyz[-1]-xyz[0]).tolist(),
            "estimated_z_min_max_m": [float(xyz[:, 2].min()), float(xyz[:, 2].max())],
            "roll_min_max_deg": np.degrees([poses["roll"].min(), poses["roll"].max()]).tolist(),
            "pitch_min_max_deg": np.degrees([poses["pitch"].min(), poses["pitch"].max()]).tolist(),
            "caveat": "Endpoint and height are estimated states, not independently measured closure/accuracy error."}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    args = parser.parse_args()
    manifest = json.loads((args.run/"manifest.json").read_text())
    poses = np.loadtxt(args.run/"odometry.tum", ndmin=2)
    delta = np.diff(poses[:, 1:4], axis=0)
    dt = np.diff(poses[:, 0])
    result = {"source": str(args.run), "completion": manifest.get("completion"),
              "map": pcd_summary(args.run/"map/GlobalMap.pcd"),
              "trajectory": {"kind": "Online append-only mapping poses; historical rows are not rewritten after loop corrections",
                             "poses": len(poses), "duration_s": float(poses[-1, 0]-poses[0, 0]),
                             "average_rate_hz": float((len(poses)-1)/(poses[-1, 0]-poses[0, 0])),
                             "rate_scope": "Mapping pose output, NOT high-rate IMU preintegration",
                             "strictly_increasing": bool(np.all(dt > 0)),
                             "estimated_path_length_m": float(np.linalg.norm(delta, axis=1).sum()),
                             "estimated_endpoint_xyz_m": (poses[-1, 1:4]-poses[0, 1:4]).tolist(),
                             "max_step_m": float(np.linalg.norm(delta, axis=1).max()),
                             "quaternion_norm_min_max": [float(np.linalg.norm(poses[:, 4:8], axis=1).min()), float(np.linalg.norm(poses[:, 4:8], axis=1).max())],
                             "caveat": "Path length and maximum step include online loop-correction jumps; they are not true travel distance or pure frontend continuity metrics."},
              "optimized_keyframes": optimized_keyframes(args.run/"map/transformations.pcd"),
              "loop_marker_scope": "ICP-accepted historical constraints; marker count is neither physical laps nor a guaranteed count of already-fused factors",
              "warning_counts": {},
              "caveat": "Mapping pose may jump at loop correction; endpoint distance/height is not localization error without independent ground truth. Sensor/extrinsics/noise are not newly calibrated."}
    for name in ("imuPreintegration", "mapOptimization", "imageProjection", "adapter"):
        log = (args.run/(name+".log")).read_text(errors="replace")
        result["warning_counts"][name] = {pattern: len(re.findall(pattern, log, re.I)) for pattern in
                                         ["large velocity", "large bias", "reset imu", "not enough features", "waiting for imu", "traceback", "segmentation", "TF_NO_FRAME_ID"]}
    adapter = json.loads((args.run/"adapter.json").read_text())
    result["input_counts"] = adapter["counts"]
    output = args.run/"summary.json"
    with output.open("x") as f:
        json.dump(result, f, indent=2, allow_nan=False)
        f.write("\n")
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
