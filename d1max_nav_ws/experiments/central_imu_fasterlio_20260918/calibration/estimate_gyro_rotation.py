#!/usr/bin/env python3
"""Estimate a candidate rigid rotation/time offset from cached recorded gyros.

No ROS, production configuration, bag mutation, or translation inference.
Vectors follow column-vector convention: gyro_c(t+lag)=R_cf gyro_f(t)+b.
"""
import argparse
import json
from pathlib import Path

import numpy as np


def interpolate(t, v, grid, max_gap=.020):
    order = np.argsort(t, kind="stable")
    t, v = t[order], v[order]
    t, first = np.unique(t, return_index=True)
    v = v[first]
    j = np.searchsorted(t, grid)
    valid = (j > 0) & (j < len(t))
    j = np.clip(j, 1, len(t)-1)
    valid &= (t[j] - t[j-1] <= max_gap)
    u = ((grid-t[j-1])/(t[j]-t[j-1]))[:, None]
    out = v[j-1]*(1-u)+v[j]*u
    valid &= np.isfinite(out).all(axis=1)
    return out, valid


def fit_rotation(x, y, iterations=5):
    weights = np.ones(len(x))
    for _ in range(iterations):
        mx = np.average(x, axis=0, weights=weights)
        my = np.average(y, axis=0, weights=weights)
        xc, yc = x-mx, y-my
        h = (xc*weights[:, None]).T @ yc
        u, singular, vt = np.linalg.svd(h)
        correction = np.eye(3)
        correction[2, 2] = np.linalg.det(vt.T @ u.T)
        rotation = vt.T @ correction @ u.T
        intercept = my - rotation @ mx
        residual = np.linalg.norm(y-x @ rotation.T-intercept, axis=1)
        delta = max(.004, 1.5*float(np.median(residual)))
        weights = np.minimum(1.0, delta/np.maximum(residual, 1e-12))
    return rotation, intercept, {"huber_delta": delta, "downweighted_fraction": float(np.mean(weights < 1)),
                                  "cross_covariance_singular_values_per_sample": (singular/weights.sum()).tolist()}


def residual_summary(x, y, rotation, intercept):
    r = y-x @ rotation.T-intercept
    norm = np.linalg.norm(r, axis=1)
    clipped = np.minimum(norm, .03)
    return {"count": len(x), "axis_mean": r.mean(axis=0).tolist(),
            "axis_std": r.std(axis=0).tolist(),
            "vector_rmse": float(np.sqrt(np.mean(norm**2))),
            "norm_median": float(np.median(norm)),
            "norm_p95": float(np.percentile(norm, 95)), "norm_p99": float(np.percentile(norm, 99)),
            "clipped_0p03_vector_rmse": float(np.sqrt(np.mean(clipped**2))),
            "fraction_norm_above_0p03": float(np.mean(norm > .03))}


def excitation(x):
    xc = x-x.mean(axis=0)
    covariance = xc.T@xc/len(x)
    info = np.trace(covariance)*np.eye(3)-covariance
    eig = np.linalg.eigvalsh(covariance)
    ieig = np.linalg.eigvalsh(info)
    return {"centered_gyro_covariance": covariance.tolist(),
            "covariance_eigenvalues_ascending": eig.tolist(),
            "covariance_eigenvalue_ratio_min_max": float(eig[0]/eig[-1]),
            "rotation_information_eigenvalues_ascending_per_sample": ieig.tolist(),
            "rotation_information_condition_number": float(ieig[-1]/ieig[0]),
            "note": "Full covariance rank supports multiple excited axes; the rotation information spectrum is a descriptive sensitivity measure, not a calibrated uncertainty bound."}


def rotation_angle(r):
    return float(np.degrees(np.arccos(np.clip((np.trace(r)-1)/2, -1, 1))))


def gravity_check(front, central, rf, rc, rotation):
    f = front["accel"][(rf >= 0)&(rf < 2)].mean(axis=0)
    c = central["accel"][(rc >= 0)&(rc < 2)].mean(axis=0)
    mapped = rotation @ f
    angle = float(np.degrees(np.arccos(np.clip(np.dot(mapped, c)/np.linalg.norm(mapped)/np.linalg.norm(c), -1, 1))))
    return {"window": "first two seconds of each sensor; low motion, not independently verified static ground truth",
            "front_accel_raw_mean": f.tolist(), "central_accel_raw_mean": c.tolist(),
            "rotated_front_accel_raw_mean": mapped.tolist(), "angle_degrees": angle,
            "raw_vector_difference": (c-mapped).tolist(),
            "note": "Uses measured acceleration only as an independent sign/orientation consistency check; includes acceleration biases and residual motion."}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cache", type=Path, default=Path(__file__).resolve().parents[2]/"central_imu_quality_20260918")
    args = parser.parse_args()
    front = dict(np.load(args.cache/"front.npz", allow_pickle=False))
    central = dict(np.load(args.cache/"central.npz", allow_pickle=False))
    origin = min(int(front["stamp_ns"][0]), int(central["stamp_ns"][0]))
    tf = (front["stamp_ns"]-origin)*1e-9
    tc = (central["stamp_ns"]-origin)*1e-9
    # Keep support for all tested lags. Avoid passing through long missing spans.
    begin = max(float(tf[0]), float(tc[0]))+.051
    end = min(float(tf[-1]), float(tc[-1]))-.051
    grid = np.arange(np.ceil(begin*100)/100, end, .01)
    x, valid_x = interpolate(tf, front["gyro"], grid)
    train_blocks = ((grid//60).astype(int)%2 == 0)
    profile = []
    for lag_ms in range(-50, 51):
        y, valid_y = interpolate(tc, central["gyro"], grid+lag_ms*.001)
        mask = valid_x&valid_y&train_blocks
        rotation, bias, _ = fit_rotation(x[mask], y[mask], iterations=3)
        score = residual_summary(x[mask], y[mask], rotation, bias)
        profile.append({"lag_ms": lag_ms, "train_clipped_rmse": score["clipped_0p03_vector_rmse"],
                        "train_median": score["norm_median"], "train_count": score["count"]})
    best = min(profile, key=lambda r: r["train_clipped_rmse"])
    lag_ms = best["lag_ms"]
    y, valid_y = interpolate(tc, central["gyro"], grid+lag_ms*.001)
    valid = valid_x&valid_y
    train, test = valid&train_blocks, valid&~train_blocks
    rotation_train, bias_train, train_robust = fit_rotation(x[train], y[train])
    rotation_all, bias_all, all_robust = fit_rotation(x[valid], y[valid])
    rotation_test, bias_test, test_robust = fit_rotation(x[test], y[test])
    heldout_time_sensitivity = []
    for candidate_ms in [0, 5, 10, 11, 12, 13, 14, 15, 20]:
        yc, vc = interpolate(tc, central["gyro"], grid+candidate_ms*.001)
        mask = valid_x&vc&~train_blocks
        heldout_time_sensitivity.append({"lag_ms": candidate_ms,
            "fixed_training_rotation_and_intercept_residual": residual_summary(x[mask], yc[mask], rotation_train, bias_train)})
    blocks = []
    for block_id in range(int(grid[-1]//60)+1):
        m = valid&(grid//60 == block_id)
        if m.sum() < 100:
            continue
        rb, bb, _ = fit_rotation(x[m], y[m])
        block_profile = []
        for candidate_ms in range(8, 19):
            g = grid[m]
            yc, vc = interpolate(tc, central["gyro"], g+candidate_ms*.001)
            xm, ym = x[m][vc], yc[vc]
            # Re-center only the intercept to avoid confusing slow bias changes
            # with this per-block timing diagnostic. Keep the global rotation.
            bb_candidate = (ym-xm@rotation_all.T).mean(axis=0)
            metric = residual_summary(xm, ym, rotation_all, bb_candidate)
            block_profile.append({"lag_ms": candidate_ms, "clipped_rmse": metric["clipped_0p03_vector_rmse"]})
        blocks.append({"id": block_id, "training": block_id%2 == 0,
                       "interval_s": [float(grid[m][0]), float(grid[m][-1])],
                       "fixed_training_model_residual": residual_summary(x[m], y[m], rotation_train, bias_train),
                       "independent_block_fit_angle_from_all_degrees": rotation_angle(rb@rotation_all.T),
                       "independent_block_fit_intercept": bb.tolist(),
                       "local_time_profile_fixed_all_rotation_recentered_bias": block_profile,
                       "local_time_best_lag_ms": min(block_profile, key=lambda p:p["clipped_rmse"])["lag_ms"],
                       "excitation": excitation(x[m])})
    # front adapter maps raw front IMU to normalized front frame via Rx(+90deg).
    r_front_normalized_from_raw = np.array([[1., 0., 0.], [0., 0., -1.], [0., 1., 0.]])
    result = {
        "schema": 1, "status": "offline experimental candidate, not factory or validated production calibration",
        "source_cache": str(args.cache), "origin_header_ns": origin,
        "model": "omega_c(t+lag)=R_c_from_f @ omega_f(t)+constant_intercept; column vector convention",
        "timing_sign": "positive lag means central samples carrying timestamp t+lag best match front at t; compensating central to the front timestamp convention would subtract lag from central header. Sensor filter/group delay and repeated messages may contribute, so this is not a proven clock offset.",
        "lag_ms": lag_ms, "search_grid_ms": 1, "search_range_ms": [-50, 50],
        "resampling_hz": 100, "maximum_interpolated_source_gap_s": .020,
        "validation_split": "alternating nonoverlapping 60s blocks on common header-time origin: even blocks train; odd blocks held out. Lag and training transform selected without held-out samples.",
        "R_c_from_f_raw": rotation_all.tolist(),
        "R_c_from_front_normalized": (rotation_all@r_front_normalized_from_raw.T).tolist(),
        "R_front_normalized_from_c_raw": (r_front_normalized_from_raw@rotation_all.T).tolist(),
        "constant_intercept_c_rad_s_if_input_rad_s": bias_all.tolist(),
        "det_rotation": float(np.linalg.det(rotation_all)),
        "orthogonality_error_frobenius": float(np.linalg.norm(rotation_all.T@rotation_all-np.eye(3))),
        "all_fit": all_robust,
        "train_model": {"rotation": rotation_train.tolist(), "intercept": bias_train.tolist(),
                        "robust_fit": train_robust,
                        "residual_train": residual_summary(x[train], y[train], rotation_train, bias_train),
                        "residual_heldout": residual_summary(x[test], y[test], rotation_train, bias_train),
                        "rotation_difference_from_all_fit_degrees": rotation_angle(rotation_train@rotation_all.T)},
        "all_fit_residual": residual_summary(x[valid], y[valid], rotation_all, bias_all),
        "independent_heldout_fit": {"rotation": rotation_test.tolist(), "intercept": bias_test.tolist(),
                                    "robust_fit": test_robust,
                                    "angle_from_training_fit_degrees": rotation_angle(rotation_test@rotation_train.T)},
        "heldout_time_sensitivity": heldout_time_sensitivity,
        "excitation_all": excitation(x[valid]), "excitation_train": excitation(x[train]),
        "excitation_heldout": excitation(x[test]),
        "gravity_consistency": gravity_check(front, central, tf-tf[0], tc-tc[0], rotation_all),
        "time_search_profile": profile, "temporal_blocks": blocks,
        "limitations": ["No translation is observable from this gyro-only model; no translation is estimated.",
                        "The front point-cloud frame differs from its raw IMU frame. Compose with the existing front point-to-IMU rotation before using this for points; do not apply R_c_from_f_raw directly to raw point coordinates.",
                        "Constant intercept is a difference of rotated gyro biases, not the standalone calibrated bias of either IMU.",
                        "This fit does not establish acceleration sign, unit, scale or bias calibration. Gravity check is separate.",
                        "Repeated measurements, timing jitter, mounting flex and temperature-dependent gyro biases limit precision."]}
    output = Path(__file__).resolve().parent/"gyro_rotation_candidate.json"
    output.write_text(json.dumps(result, indent=2, allow_nan=False)+"\n")
    print(json.dumps({"output": str(output), "lag_ms": lag_ms, "rotation": result["R_c_from_f_raw"],
                      "heldout_residual": result["train_model"]["residual_heldout"],
                      "gravity_angle_deg": result["gravity_consistency"]["angle_degrees"]}))


if __name__ == "__main__":
    main()
