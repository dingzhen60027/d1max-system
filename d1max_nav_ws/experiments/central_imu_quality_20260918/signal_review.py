#!/usr/bin/env python3
"""Descriptive signal review of cached bag IMU arrays; never starts ROS.

Outputs are observations about this recording, not sensor accuracy estimates.
The cache is produced independently from read-only rosbag extraction.
"""
import argparse
import json
from pathlib import Path

import numpy as np


SENSORS = ("central", "front", "rear")
QUANTILES = (0, 1, 5, 50, 95, 99, 100)


def scalar_summary(x):
    x = np.asarray(x)
    x = x[np.isfinite(x)]
    if not len(x):
        return {"finite_count": 0}
    return {
        "finite_count": len(x),
        "mean": float(x.mean()),
        "std_population": float(x.std()),
        "quantiles": {str(q): float(v) for q, v in zip(QUANTILES, np.percentile(x, QUANTILES))},
    }


def vector_summary(x):
    finite = np.isfinite(x).all(axis=1)
    x = x[finite]
    if not len(x):
        return {"finite_count": 0}
    return {
        "finite_count": len(x),
        "axis_mean_xyz": x.mean(axis=0).tolist(),
        "axis_std_population_xyz": x.std(axis=0).tolist(),
        "axis_min_xyz": x.min(axis=0).tolist(),
        "axis_max_xyz": x.max(axis=0).tolist(),
        "std_vector_rss": float(np.linalg.norm(x.std(axis=0))),
        "norm_of_axis_mean": float(np.linalg.norm(x.mean(axis=0))),
        "sample_norm": scalar_summary(np.linalg.norm(x, axis=1)),
        "identical_adjacent_vectors": int(np.all(np.diff(x, axis=0) == 0, axis=1).sum()),
    }


def sensor_review(a, sensor):
    stamps = a["stamp_ns"]
    t = (stamps - stamps[0]) * 1e-9
    finish = float(t[-1])
    # All three recorded acceleration magnitudes are near 1 in the initial
    # low-motion window. That suggests g, but no driver unit metadata proves it.
    scale = 9.80665
    windows = [("first_2s", 0.0, 2.0), ("first_10s", 0.0, 10.0),
               ("first_30s", 0.0, 30.0), ("final_10s", finish - 10.0, finish + 1e-8),
               ("whole_recording", 0.0, finish + 1e-8)]
    output = {"sensor": sensor, "stamp_origin_ns": int(stamps[0]),
              "window_origin": "each sensor's first header timestamp; [start,end)",
              "assumed_accel_raw_to_inferred_m_s2": scale,
              "units_note": "Raw acceleration units are not established from the source driver. All three sensors have initial acceleration norm near 1, consistent with g; inferred_m_s2 fields assume raw acceleration is g and multiply by 9.80665. Gyro is unchanged as recorded; its cross-sensor norm scale is similar, conventionally rad/s. No unit calibration is performed.",
              "windows": {}}
    for name, start, end in windows:
        m = (t >= start) & (t < end)
        result = {"requested_interval_s": [start, end], "count": int(m.sum())}
        if m.any():
            six_axis = np.column_stack((a["accel"][m], a["gyro"][m]))
            changed = np.r_[True, np.any(np.diff(six_axis, axis=0) != 0, axis=1)]
            result["six_axis_change_events"] = {
                "count_including_first": int(changed.sum()),
                "identical_adjacent_message_count": int((~changed).sum()),
                "accel_raw": vector_summary(a["accel"][m][changed]),
                "accel_inferred_m_s2": vector_summary(a["accel"][m][changed] * scale),
                "gyro_raw": vector_summary(a["gyro"][m][changed]),
                "interpretation": "Consecutive identical six-axis rows collapsed only for this comparison. Change events are not independently established hardware update events; event-weighted statistics also differ from time-weighted statistics.",
            }
            result.update({"first_last_elapsed_s": [float(t[m][0]), float(t[m][-1])],
                           "accel_raw": vector_summary(a["accel"][m]),
                           "accel_inferred_m_s2": vector_summary(a["accel"][m] * scale),
                           "gyro_raw": vector_summary(a["gyro"][m])})
        output["windows"][name] = result
    return output


def interp_norm(a, origin_ns, grid):
    mask = np.isfinite(a["gyro"]).all(axis=1)
    t = (a["stamp_ns"][mask] - origin_ns) * 1e-9
    y = np.linalg.norm(a["gyro"][mask], axis=1)
    order = np.argsort(t, kind="stable")
    t, y = t[order], y[order]
    t, indices = np.unique(t, return_index=True)
    y = y[indices]
    right = np.searchsorted(t, grid)
    valid = (right > 0) & (right < len(t))
    clipped = np.clip(right, 1, len(t) - 1)
    # Do not fill the known source outages with invented smooth motion.
    valid &= (t[clipped] - t[clipped - 1]) <= 0.030
    result = np.interp(grid, t, y)
    result[~valid] = np.nan
    return result


def smooth(x, width=11):
    valid = np.isfinite(x)
    sums = np.convolve(np.nan_to_num(x), np.ones(width), mode="same")
    counts = np.convolve(valid.astype(float), np.ones(width), mode="same")
    result = sums / np.maximum(counts, 1)
    result[counts < width] = np.nan
    return result


def lag_diagnostics(a, b, step=0.01):
    results = []
    for offset in range(-50, 51):
        if offset > 0:
            left, right = a[:-offset], b[offset:]
        elif offset < 0:
            left, right = a[-offset:], b[:offset]
        else:
            left, right = a, b
        valid = np.isfinite(left) & np.isfinite(right)
        left, right = left[valid], right[valid]
        if len(left) < 100:
            continue
        lstd, rstd = left.std(), right.std()
        if min(lstd, rstd) < 1e-9:
            continue
        corr = float(np.mean((left - left.mean()) * (right - right.mean())) / (lstd * rstd))
        results.append({"lag_s": offset * step, "pearson": corr, "paired_samples": len(left),
                        "norm_std_a": float(lstd), "norm_std_b": float(rstd),
                        "demeaned_slope_b_on_a": corr * float(rstd / lstd)})
    if not results:
        return {"available": False}
    peak = max(results, key=lambda r: r["pearson"])
    return {"available": True, "zero_lag": next(r for r in results if r["lag_s"] == 0),
            "peak": peak, "peak_at_search_boundary": abs(peak["lag_s"]) >= 0.5}


def comparisons(arrays):
    origin = min(int(a["stamp_ns"][0]) for a in arrays.values())
    start = max((int(a["stamp_ns"][0]) - origin) * 1e-9 for a in arrays.values())
    end = min((int(a["stamp_ns"][-1]) - origin) * 1e-9 for a in arrays.values())
    output = {"origin_ns": origin, "overlap_interval_s": [start, end],
              "grid_hz": 100, "interpolation_max_source_gap_s": .03,
              "lag_search_s": [-.5, .5],
              "lag_sign": "For pair a,b, positive lag correlates a(t) with b(t+lag), so positive means b's features occur at later header timestamps.",
              "limitations": "Rotation-invariant gyro norms remove constant axis rotation, not bias, scale, vibration, flex, or clock effects. Correlation lags are descriptive diagnostics, not timestamp or extrinsic calibration. All signal statistics include actual robot motion.",
              "pairs": {}}
    if end - start < 2:
        output["available"] = False
        return output
    grid = np.arange(np.ceil(start * 100) / 100, end, .01)
    raw = {name: interp_norm(a, origin, grid) for name, a in arrays.items()}
    filtered = {name: smooth(value) for name, value in raw.items()}
    for a, b in (("front", "central"), ("rear", "central"), ("front", "rear")):
        pair = {"a": a, "b": b,
                "raw_norm": lag_diagnostics(raw[a], raw[b]),
                "smoothed_110ms_norm": lag_diagnostics(filtered[a], filtered[b]),
                "sixty_second_blocks_smoothed": []}
        for begin in np.arange(grid[0], grid[-1] - 29, 60):
            mask = (grid >= begin) & (grid < begin + 60)
            pair["sixty_second_blocks_smoothed"].append({"interval_s": [float(begin), float(min(begin + 60, grid[-1]))],
                                                        "result": lag_diagnostics(filtered[a][mask], filtered[b][mask])})
        output["pairs"][a + "_vs_" + b] = pair
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--directory", type=Path, default=Path(__file__).resolve().parent)
    args = parser.parse_args()
    arrays = {name: dict(np.load(args.directory / (name + ".npz"), allow_pickle=False)) for name in SENSORS}
    result = {"schema": 1, "source": "cached original Sept17 bag arrays; no replay",
              "interpretation_limits": ["First/final fixed windows are descriptive; no independently verified static interval or motion ground truth is available.",
                                         "Variances during walking reflect real body motion, placement, filtering and transport as well as sensor noise; smaller variance is not a hardware accuracy ranking.",
                                         "No temperature/Allan-variance/metrology analysis or calibration is performed."],
              "sensors": {name: sensor_review(a, name) for name, a in arrays.items()},
              "gyro_norm_comparisons": comparisons(arrays)}
    target = args.directory / "signal_review.json"
    target.write_text(json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False) + "\n")
    print(json.dumps({"output": str(target), "counts": {n: len(a["stamp_ns"]) for n, a in arrays.items()}}))


if __name__ == "__main__":
    main()
