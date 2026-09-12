"""One YAML -> explicit node parameters. No ROS imports or hidden second config."""

import math
from dataclasses import fields
from .prediction import InertialPredictor, PredictionLimits
from .navigation import NavigationState, NavigationLimits


def navigation_parameters(config):
    pipeline = config.get("navigation_estimation", {}).get("ros__parameters", {})
    if not pipeline.get("enabled", False):
        return None
    known = {"enabled", "output_rate_hz", "body_frame", "filter_process_noise_diagonal"}
    known.update("prediction." + f.name for f in fields(PredictionLimits))
    known.update("limits." + f.name for f in fields(NavigationLimits))
    if set(pipeline) - known:
        raise ValueError(
            "unknown navigation parameter: " + ", ".join(sorted(set(pipeline) - known))
        )
    rate = pipeline.get("output_rate_hz", 50.0)
    if type(rate) not in (int, float) or not math.isfinite(rate) or not 20 <= rate <= 100:
        raise ValueError("invalid navigation output rate")
    front = config["lio_localizer"]["ros__parameters"]
    frames = {key: front[key] for key in ("map_frame", "odom_frame", "tracking_frame")}
    output = {
        **frames,
        "output_rate_hz": float(rate),
        "body_frame": pipeline.get("body_frame", "d1max_loc_base_link"),
    }
    for key in (
        "sdk_to_tracking_yaw",
        "tracking_offset_body",
        "extrinsics_verified",
        "time_alignment_verified",
        "trajectory_rate_hz",
        "trajectory_max_points",
    ):
        if key in front:
            output[key] = front[key]
    for key, value in pipeline.items():
        if key.startswith("limits."):
            output[key] = value
    prediction = {key: frames[key] for key in ("odom_frame", "tracking_frame")}
    prediction.update(
        output_rate_hz=float(rate),
        imu_frame=config["dual_lidar_adapter"]["ros__parameters"]["target_frame"],
    )
    for key, value in pipeline.items():
        if key.startswith("prediction."):
            prediction[key[len("prediction.") :]] = value
    # Validate contracts BEFORE launch, not after partially starting the pipeline.
    pl = PredictionLimits(**{k[11:]: v for k, v in pipeline.items() if k.startswith("prediction.")})
    nl = NavigationLimits(**{k[7:]: v for k, v in pipeline.items() if k.startswith("limits.")})
    InertialPredictor(pl)
    NavigationState(nl)
    if nl.max_prediction_horizon < pl.max_horizon or nl.max_imu_age < pl.max_extrapolation:
        raise ValueError("navigation admission must cover the configured prediction bounds")
    rl = config["ekf_navigation"]["ros__parameters"]
    required = {
        "publish_tf": False,
        "two_d_mode": False,
        "predict_to_current_time": True,
        "smooth_lagged_data": True,
        "use_control": False,
    }
    if any(rl.get(k) is not v for k, v in required.items()):
        raise ValueError(
            "navigation EKF must be private, full 3D, lag-aware and without control inputs"
        )
    if (
        rl.get("twist0") != "/d1max/localization/estimator/motion"
        or rl.get("pose0") != "/d1max/localization/estimator/map_pose"
        or rl.get("twist0_config") != [False] * 6 + [True] * 6 + [False] * 3
        or rl.get("pose0_config") != [True] * 6 + [False] * 9
        or any(
            k.startswith(("imu", "odom", "twist", "pose"))
            and k.split("_")[0] not in ("pose0", "twist0")
            for k in rl
        )
    ):
        raise ValueError("navigation EKF accepts only predictor twist and verified PCD pose")
    noise = pipeline.get(
        "filter_process_noise_diagonal",
        [0.03, 0.03, 0.04, 0.02, 0.02, 0.04, 0.15, 0.15, 0.15, 0.05, 0.05, 0.08, 0.2, 0.2, 0.2],
    )
    if len(noise) != 15 or any(
        type(x) not in (int, float) or not math.isfinite(x) or x <= 0 for x in noise
    ):
        raise ValueError("invalid EKF process noise")
    ekf = {
        "frequency": float(rate),
        "map_frame": frames["map_frame"],
        "odom_frame": frames["odom_frame"],
        "base_link_frame": frames["tracking_frame"],
        "world_frame": frames["map_frame"],
        "process_noise_covariance": [
            float(noise[i]) if i == j else 0.0 for i in range(15) for j in range(15)
        ],
    }
    return prediction, output, ekf
