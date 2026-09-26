#!/usr/bin/env python3
"""Independent, offline LiDAR/central-gyro diagnostic; never installs calibration.

ICP supplies R_Lprevious_Lnext in raw LiDAR axes. Its conjugate by R_C_L
should match the central gyro relative rotation. Raw rotating scans do not
have one exact pose epoch, so this is an identifiability/consistency test.
"""
import argparse
import importlib.util
import json
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
FIT_SOURCE = ROOT / "central_imu_fasterlio_20260918/calibration/estimate_gyro_rotation.py"
spec = importlib.util.spec_from_file_location("gyro_fit_helpers", FIT_SOURCE)
helpers = importlib.util.module_from_spec(spec)
spec.loader.exec_module(helpers)


def prepare(t, v):
    order = np.argsort(t, kind="stable")
    t, v = t[order], v[order]
    t, first = np.unique(t, return_index=True)
    return t, v[first]


def interval_samples(t, v, a, b, max_gap):
    """Exact endpoints + original knots, no interpolation across long gaps."""
    if not t[0] <= a < b <= t[-1]:
        return None
    left = max(0, np.searchsorted(t, a, side="right") - 1)
    right = min(len(t)-1, np.searchsorted(t, b, side="left"))
    if np.any(np.diff(t[left:right+1]) > max_gap):
        return None
    knots = np.r_[a, t[(t > a) & (t < b)], b]
    samples = np.column_stack([np.interp(knots, t, v[:, j]) for j in range(3)])
    return knots, samples


def interval_mean(t, v, a, b, max_gap):
    data = interval_samples(t, v, a, b, max_gap)
    if data is None:
        return np.full(3, np.nan)
    knots, samples = data
    return np.trapz(samples, knots, axis=0)/(b-a)


def integrate(t, v, a, b, bias, max_gap):
    knots, samples = interval_samples(t, v, a, b, max_gap)
    total = Rotation.identity()
    for dt, omega in zip(np.diff(knots), (samples[:-1]+samples[1:])*.5-bias):
        total = total * Rotation.from_rotvec(omega*dt)
    return total


def robust_score(x, y, r, b):
    """Smooth Huber cost, unlike capped residuals still penalizes large errors."""
    n = np.linalg.norm(y - x @ r.T - b, axis=1)
    delta = .02
    return float(np.mean(np.where(n <= delta, .5*n*n, delta*(n-.5*delta))))


def fit_huber(x, y, fixed_rotation=None, iterations=60):
    """Same radial Huber loss (delta=.02 rad/s) for free/fixed rotations."""
    weights = np.ones(len(x))
    for _ in range(iterations):
        mx = np.average(x, axis=0, weights=weights)
        my = np.average(y, axis=0, weights=weights)
        if fixed_rotation is None:
            u, _, vt = np.linalg.svd(((x-mx)*weights[:, None]).T @ (y-my))
            correction = np.eye(3)
            correction[2, 2] = np.linalg.det(vt.T@u.T)
            r = vt.T@correction@u.T
        else:
            r = fixed_rotation
        b = my-r@mx
        n = np.linalg.norm(y-x@r.T-b, axis=1)
        new_weights = np.minimum(1., .02/np.maximum(n, 1e-12))
        if np.max(np.abs(new_weights-weights)) < 1e-9:
            break
        weights = new_weights
    return r, b, {"huber_delta_rad_s": .02, "iterations": _+1}


def profile_fit(x, ys, lags, mask):
    profile, models = [], []
    for lag, y in zip(lags, ys):
        r, b, detail = fit_huber(x[mask], y[mask])
        profile.append({"lag_ms": int(lag), "huber_cost": robust_score(x[mask], y[mask], r, b),
                        "residual": helpers.residual_summary(x[mask], y[mask], r, b)})
        models.append((r, b, detail))
    best = int(np.argmin([p["huber_cost"] for p in profile]))
    return best, models[best], profile


def rotations_for(pairs, variant):
    if variant == "primary":
        return np.array([p["R_prev_from_next"] for p in pairs])
    key = "alternate_T_prev_from_next" if variant == "alternate" else "reverse_T_next_from_prev"
    rs = np.array([np.asarray(p[key])[:3, :3] for p in pairs])
    return rs if variant == "alternate" else rs.transpose(0, 2, 1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--motion", type=Path, default=Path(__file__).parent/"lidar_motion/lidar_rotations.json")
    parser.add_argument("--output", type=Path, default=Path(__file__).parent/"lidar_central_fit_v2.json")
    args = parser.parse_args()
    motion = json.loads(args.motion.read_text())
    pairs = [p for p in motion["pairs"] if p["accepted"]]
    central = np.load(ROOT/"central_imu_quality_20260918/central.npz", allow_pickle=False)
    origin = int(central["stamp_ns"][0])
    tc, gyro = prepare((central["stamp_ns"]-origin)*1e-9, central["gyro"])
    # Recover scan midpoints relative to int64 header rather than subtracting
    # two epoch-sized floating-point values for the whole timestamp.
    def midpoint(p):
        return (int(p["header_ns"])-origin)*1e-9 + (p["point_time_max_sec"]-p["point_time_min_sec"])*.5
    intervals = np.array([[midpoint(p["previous"]), midpoint(p["next"])] for p in pairs])
    dt = intervals[:, 1]-intervals[:, 0]
    groups = np.array([p["window_index"] for p in pairs])
    lags = np.arange(-80, 81, 2)
    r_n_l = Rotation.from_quat([-.499867275, .503186620, .497953310, .498977388]).as_matrix()
    configs = {
        "historical": ROOT/"central_imu_fasterlio_20260918/calibration.yaml",
        "device_front": ROOT/"lio_frontend_reliability_20260919/extrinsic_trials_20260919/calibrations/device_front_rotation.yaml",
    }
    current = {k: np.array(yaml.safe_load(p.read_text())["lio_extrinsic"]["rotation"]).reshape(3, 3) @ r_n_l
               for k, p in configs.items()}
    output = {
        "status": "DIAGNOSTIC ONLY: not an approved or deployed calibration",
        "source_motion": str(args.motion), "source_time_origin_ns": origin,
        "model": "mean omega_C(mid_prev+lag..mid_next+lag) = R_C_L * rotvec(R_Lprev_Lnext)/dt + b_C",
        "lag_sign": "Positive lag associates central timestamp t+lag with LiDAR effective epoch t. Uniform choice of scan epoch can shift lag; not a certified hardware offset.",
        "lag_search_ms": [-80, 80], "lag_step_ms": 2,
        "fit_loss": "Free/fixed fits and lag selection use the same vector-norm Huber loss, delta=.02 rad/s, converged IRLS; RMSE is a descriptive unweighted metric, not the selected objective.",
        "selected_pairs": [{"window": int(g), "pair": p["pair_index"], "interval_s": interval.tolist()}
                           for p, g, interval in zip(pairs, groups, intervals)],
        "current_R_C_L": {k: v.tolist() for k, v in current.items()},
        "variants": [],
        "limitations": [
            "Raw spinning scans are motion distorted; ICP is not ground truth and midpoint is approximate.",
            "ICP quality gates, reverse consistency, and voxel stability do not exclude common geometric bias.",
            "Adjacent pairs are correlated; leave-window-out, not random sample splitting, is used.",
            "Mean-rate Procrustes neglects noncommuting rotations; SO3 integration check is provided separately.",
            "No translation/accelerometer bias/scale/gravity calibration is estimated.",
            "Free gyro intercept can absorb part of registration/model error; it is not a certified IMU bias.",
            "Gap sensitivity uses distinct common pair sets per threshold; never compare those as a controlled A/B.",
        ],
    }
    for gap in [.020, .035]:
        ys = np.array([[interval_mean(tc, gyro, a+lag*.001, b+lag*.001, gap)
                        for a, b in intervals] for lag in lags])
        common = np.isfinite(ys).all(axis=(0, 2))
        for variant in ["primary", "alternate", "reverse"]:
            matrices = rotations_for(pairs, variant)
            x = Rotation.from_matrix(matrices).as_rotvec()/dt[:, None]
            item = {"variant": variant, "max_source_gap_s": gap,
                    "common_mask_across_all_lags": common.tolist(),
                    "common_pair_count": int(common.sum()),
                    "common_pairs_per_window": {str(int(g)): int(np.sum(common & (groups == g))) for g in np.unique(groups)}}
            if common.sum() < 6 or len(np.unique(groups[common])) < 3:
                item["status"] = "insufficient independently separated windows after common-support gap guard"
                output["variants"].append(item)
                continue
            best, (r, bias, detail), profile = profile_fit(x, ys, lags, common)
            item.update({"status": "fit produced for sensitivity audit only", "best_lag_ms": int(lags[best]),
                         "R_C_L": r.tolist(), "intercept_rad_s": bias.tolist(),
                         "excitation": helpers.excitation(x[common]), "profile": profile,
                         "residual": helpers.residual_summary(x[common], ys[best][common], r, bias),
                         "delta_from_current_deg": {k: helpers.rotation_angle(r@rc.T) for k, rc in current.items()}})
            item["current_fixed_rotation_profiles"] = {}
            for name, rc in current.items():
                fixed = []
                for lag, y in zip(lags, ys):
                    _, bc, _ = fit_huber(x[common], y[common], fixed_rotation=rc)
                    fixed.append({"lag_ms": int(lag), "huber_cost": robust_score(x[common], y[common], rc, bc),
                                  "residual": helpers.residual_summary(x[common], y[common], rc, bc)})
                item["current_fixed_rotation_profiles"][name] = {
                    "best": min(fixed, key=lambda z: z["huber_cost"]), "profile": fixed}
            item["leave_one_window_out"] = []
            for g in np.unique(groups[common]):
                train, test = common & (groups != g), common & (groups == g)
                bi, (rg, bg, _), _ = profile_fit(x, ys, lags, train)
                item["leave_one_window_out"].append({
                    "heldout_window": int(g), "train_count": int(train.sum()), "test_count": int(test.sum()),
                    "train_selected_lag_ms": int(lags[bi]),
                    "rotation_change_from_full_fit_deg": helpers.rotation_angle(rg@r.T),
                    "heldout_residual": helpers.residual_summary(x[test], ys[bi][test], rg, bg),
                    "train_excitation": helpers.excitation(x[train])})
            check = []
            for i in np.flatnonzero(common):
                a, b = intervals[i]+lags[best]*.001
                imu = integrate(tc, gyro, a, b, bias, gap)
                mean = Rotation.from_rotvec((ys[best][i]-bias)*dt[i])
                mapped = Rotation.from_matrix(r@matrices[i]@r.T)
                check.append({"window": int(groups[i]), "pair": pairs[i]["pair_index"],
                              "mean_vs_composed_rotation_deg": float(np.degrees((mean.inv()*imu).magnitude())),
                              "lidar_vs_composed_imu_deg": float(np.degrees((mapped.inv()*imu).magnitude()))})
            item["noncommuting_integration_check"] = check
            output["variants"].append(item)
    with args.output.open("x") as f:
        json.dump(output, f, indent=2, allow_nan=False)
        f.write("\n")
    print(json.dumps({"output": str(args.output), "variants": [
        {k: v.get(k) for k in ["variant", "max_source_gap_s", "common_pair_count", "common_pairs_per_window", "best_lag_ms", "delta_from_current_deg", "residual"]}
        for v in output["variants"]]}, indent=2))


if __name__ == "__main__":
    main()
