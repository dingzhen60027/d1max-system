"""Deterministic estimator contracts. No ROS graph, SDK, services or robot."""

from copy import deepcopy
from dataclasses import replace
from pathlib import Path
import math
import numpy as np
import pytest
import yaml

from d1max_localization.math_utils import Pose3, compose, inverse, rotate_vector
from d1max_localization.estimation.contracts import MotionState, body_state, covariance
from d1max_localization.estimation.prediction import (
    InertialPredictor,
    PredictionLimits,
    rotation_step,
)
from d1max_localization.estimation.navigation import NavigationState, NavigationLimits
from d1max_localization.estimation.configuration import navigation_parameters


def diagonal(value=0.01):
    return [value if i == j else 0.0 for i in range(6) for j in range(6)]


def snapshot(stamp=10.0, epoch=1):
    return dict(
        schema=1,
        epoch=epoch,
        valid=True,
        fault=False,
        received_at_unix=stamp,
        stamp_ns=str(round(stamp * 1e9)),
        frame="d1max_loc_odom",
        child_frame="d1max_loc_tracking",
        position=[0.0, 0.0, 0.0],
        orientation=[0.0, 0.0, 0.0, 1.0],
        pose_covariance=diagonal(),
        twist_covariance=diagonal(),
        inertial=dict(
            schema=1,
            world_velocity=[0.0, 0.0, 0.0],
            gyro_bias=[0.0, 0.0, 0.0],
            accel_bias=[0.0, 0.0, 0.0],
            gravity=[0.0, 0.0, -9.81],
            accel_scale=1.0,
        ),
    )


def local(stamp, epoch=1, x=0.0):
    return dict(
        schema=1,
        epoch=epoch,
        valid=True,
        fault=False,
        received_at_unix=stamp,
        stamp_ns=str(round(stamp * 1e9)),
        source_stamp_ns=str(round((stamp - 0.04) * 1e9)),
        imu_stamp_ns=str(round(stamp * 1e9)),
        extrapolation_sec=0.0,
        frame="d1max_loc_odom",
        child_frame="d1max_loc_tracking",
        position=[x, 0.0, 0.0],
        orientation=[0.0, 0.0, 0.0, 1.0],
        world_velocity=[0.0, 0.0, 0.0],
        angular=[0.0, 0.0, 0.0],
        pose_covariance=diagonal(),
        twist_covariance=diagonal(),
    )


def alignment(stamp=10.0, epoch=1, seed="seed-a"):
    return dict(
        schema=1,
        epoch=epoch,
        valid=True,
        fault=False,
        received_at_unix=stamp,
        stamp_ns=str(round(stamp * 1e9)),
        frame="d1max_loc_map",
        child_frame="d1max_loc_tracking",
        seed_id=seed,
        confirmations=3,
        position=[0.0, 0.0, 0.0],
        orientation=[0.0, 0.0, 0.0, 1.0],
        covariance=diagonal(),
        anchor=dict(position=[0.0, 0.0, 0.0], orientation=[0.0, 0.0, 0.0, 1.0]),
    )


def push_filter(core, stamp, x=0.0):
    return core.push_filtered(
        stamp,
        Pose3((x, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        diagonal(),
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        stamp,
    )


def locked():
    core = NavigationState()
    core.push_local(local(10.0), 10.0)
    core.accept_map(alignment(), 10.0)
    key, stamp, _ = core.begin_reset(10.0)
    assert core.reset_ack(key, stamp, 10.0)
    for stamp in (10.18, 10.20):
        core.push_local(local(stamp), stamp)
        push_filter(core, stamp)
    return core


def test_static_prediction_in_tilted_odom_uses_estimated_gravity_bias_and_scale():
    p = InertialPredictor()
    s = snapshot()
    q = rotation_step((0.8, -0.4, 0.1), 0.2)
    s["orientation"] = list(q)
    s["inertial"].update(
        gravity=list(rotate_vector(q, (0.0, 0.0, -9.81))),
        gyro_bias=[0.02, -0.01, 0.03],
        accel_bias=[0.1, -0.2, 0.3],
        accel_scale=1.02,
    )
    a = tuple(v / 1.02 for v in (0.1, -0.2, 10.11))
    for t in np.arange(9.995, 10.201, 0.005):
        assert p.push_imu(float(t), a, (0.02, -0.01, 0.03), float(t))
    assert p.accept(s, 10.20)
    result = p.evaluate(10.20)
    assert result is not None
    assert np.linalg.norm(result.pose.position) < 1e-9
    assert np.linalg.norm(result.world_velocity) < 1e-9
    assert result.pose.orientation == pytest.approx(q)


def test_prediction_integrates_actual_acceleration_and_gyro():
    p = InertialPredictor()
    p.accept(snapshot(), 10.0)
    for t in np.arange(9.995, 10.101, 0.005):
        p.push_imu(float(t), (2.0, 0.0, 9.81), (0.0, 0.0, 0.0), float(t))
    result = p.evaluate(10.10)
    assert result.pose.position == pytest.approx((0.01, 0.0, 0.0), abs=1e-7)
    assert result.world_velocity == pytest.approx((0.2, 0.0, 0.0), abs=1e-7)
    assert p.evaluate(10.14) is None and p.reason == "imu_stale"
    assert p.evaluate(10.26) is None and p.reason == "lio_stale"


def test_prediction_reanchors_delayed_posterior_without_double_integration():
    p = InertialPredictor()
    s = snapshot()
    s["inertial"]["world_velocity"] = [1.0, 0.0, 0.0]
    for t in np.arange(9.995, 10.201, 0.005):
        p.push_imu(float(t), (0.0, 0.0, 9.81), (0.0, 0.0, 0.0), float(t))
    p.accept(s, 10.1)
    assert p.evaluate(10.1).pose.position[0] == pytest.approx(0.1)
    s = snapshot(10.1)
    s["position"][0] = 0.1
    s["inertial"]["world_velocity"] = [1.0, 0.0, 0.0]
    p.accept(s, 10.2)
    assert p.evaluate(10.2).pose.position[0] == pytest.approx(0.2)


def test_prediction_gap_and_epoch_cannot_reuse_old_posterior():
    p = InertialPredictor()
    p.accept(snapshot(), 10.0)
    for t in (9.995, 10.0, 10.1):
        p.push_imu(t, (0.0, 0.0, 9.81), (0.0, 0.0, 0.0), t)
    assert p.evaluate(10.1) is None and p.reason == "imu_gap"
    assert p.accept(dict(schema=1, epoch=2, valid=False, fault=False, received_at_unix=10.1), 10.1)
    assert p.evaluate(10.1) is None
    assert not p.accept(snapshot(), 10.1)
    assert p.snapshot is None


def test_host_clock_backwards_latches_prediction_fault():
    p = InertialPredictor()
    p.evaluate(10.0)
    p.evaluate(9.8)
    assert p.fault == "host_clock_reset"
    p.accept(snapshot(), 10.0)
    # First epoch establishes a new LIO session, subsequent same-epoch jumps do not.
    p.evaluate(10.0)
    p.evaluate(9.8)
    p.accept(snapshot(10.1), 10.1)
    assert p.fault == "host_clock_reset" and p.evaluate(10.1) is None


@pytest.mark.parametrize(
    "field,value",
    [
        ("orientation", [0.0, 0.0, 0.0, 0.0]),
        ("pose_covariance", [float("nan")] * 36),
        ("frame", "map"),
        ("epoch", True),
    ],
)
def test_bad_snapshot_is_atomic(field, value):
    p = InertialPredictor()
    s = snapshot()
    s[field] = value
    with pytest.raises((ValueError, KeyError)):
        p.accept(s, 10.0)
    assert p.epoch == 0 and p.snapshot is None


def test_covariance_checks_symmetry_and_psd():
    m = diagonal()
    m[0] = -1.0
    with pytest.raises(ValueError):
        covariance(m)
    m = diagonal()
    m[1] = 0.1
    with pytest.raises(ValueError):
        covariance(m)
    assert covariance(diagonal(0.0), (0.1,) * 6)[0] == 0.1


def test_body_reference_removes_lever_arm_velocity():
    # Sensor one metre ahead of body, in-place yaw rotation: sensor v_y = 1.
    state = MotionState(
        10.0,
        Pose3((1.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        (0.0, 1.0, 0.0),
        (0.0, 0.0, 1.0),
        diagonal(),
        diagonal(),
        10.0,
        10.0,
    )
    pose, velocity, angular, pc, tc = body_state(
        state, Pose3((1.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    )
    assert pose.position == pytest.approx((0.0, 0.0, 0.0))
    assert velocity == pytest.approx((0.0, 0.0, 0.0))
    assert angular == pytest.approx((0.0, 0.0, 1.0))
    assert np.linalg.eigvalsh(np.array(tc).reshape(6, 6))[0] > 0
    assert tc[7] > 0.01 and pc[7] > 0.01


def test_no_map_no_global_and_bounded_filter_output():
    core = NavigationState()
    core.push_local(local(10.0), 10.0)
    assert core.output(10.0) is None
    assert core.reason == "waiting_map"
    core = locked()
    assert core.output(10.2) is not None
    assert core.output(10.3) is None and core.reason == "waiting_local"


def test_delayed_filter_output_aligned_to_common_time_without_restamping():
    core = locked()
    core.filtered.clear()
    assert push_filter(core, 10.19)
    assert push_filter(core, 10.21)
    result = core.output(10.21)
    assert result is not None and result[0].stamp == pytest.approx(10.2, abs=1e-9)
    assert core.last_output == pytest.approx(10.2, abs=1e-9)


def test_reseed_and_epoch_require_new_ack_and_never_reuse_unguarded_filter():
    core = locked()
    assert core.output(10.2)
    assert core.accept_map(alignment(10.21, seed="seed-b"), 10.21)
    assert not push_filter(core, 10.22)
    reset = core.begin_reset(10.22)
    assert reset[0] == (1, "seed-b")
    assert not core.reset_ack((1, "seed-a"), 10.0, 10.23)
    assert core.output(10.23) is None
    core.push_local(local(10.24, epoch=2), 10.24)
    assert core.map_contract is None and not core.filtered
    assert not core.reset_ack(reset[0], reset[1], 10.24)
    assert not core.accept_map(alignment(10.24), 10.24)


def test_unknown_reset_is_not_retried_even_for_new_seed():
    core = locked()
    core.filter_fault = "filter_reset_unknown"
    core.accept_map(alignment(10.21, seed="seed-b"), 10.21)
    assert core.begin_reset(10.21) is None
    assert core.output(10.21) is None and core.reason == "filter_reset_unknown"


def test_local_and_global_jump_fail_closed():
    core = locked()
    assert core.output(10.2)
    assert not core.push_local(local(10.22, x=1.0), 10.22)
    assert core.fault == "local_pose_jump" and core.output(10.22) is None
    core = locked()
    assert core.output(10.2)
    core.push_local(local(10.22), 10.22)
    push_filter(core, 10.22, x=0.2)
    assert core.output(10.22) is None and core.filter_fault == "global_correction_jump"


def test_map_expiry_not_hidden_by_fresh_filter_predictions():
    core = locked()
    for t in np.arange(10.22, 10.81, 0.02):
        t = float(t)
        core.push_local(local(t), t)
        push_filter(core, t)
    assert core.output(10.8) is None and core.reason == "waiting_map"


@pytest.mark.parametrize(
    "limits", [PredictionLimits(max_horizon=2.0), PredictionLimits(max_extrapolation=0.1)]
)
def test_unbounded_prediction_configuration_refused(limits):
    with pytest.raises(ValueError):
        InertialPredictor(limits)


def test_single_yaml_generates_rate_and_extrinsics_without_planar_duplicate_inputs():
    config = yaml.safe_load((Path(__file__).parents[1] / "config/localization.yaml").read_text())
    prediction, output, ekf = navigation_parameters(config)
    assert prediction["output_rate_hz"] == output["output_rate_hz"] == ekf["frequency"] == 50.0
    assert output["body_frame"] != ekf["base_link_frame"]
    assert (
        output["tracking_offset_body"]
        == config["lio_localizer"]["ros__parameters"]["tracking_offset_body"]
    )
    p = config["ekf_navigation"]["ros__parameters"]
    assert p["two_d_mode"] is False and p["publish_tf"] is False
    assert p["predict_to_current_time"] and p["smooth_lagged_data"]
    assert not any(k in p for k in ("imu0", "odom0", "twist1", "pose1"))
    assert p["twist0_config"] == [False] * 6 + [True] * 6 + [False] * 3
    assert p["pose0_config"] == [True] * 6 + [False] * 9
    assert len(ekf["process_noise_covariance"]) == 225
    config["navigation_estimation"]["ros__parameters"]["output_rate_hz"] = 1000.0
    with pytest.raises(ValueError):
        navigation_parameters(config)


@pytest.mark.parametrize(
    "section,key,value",
    [
        ("navigation_estimation", "prediction.max_horizn", 0.25),
        ("navigation_estimation", "prediction.max_horizon", 2.0),
        ("ekf_navigation", "publish_tf", True),
        ("ekf_navigation", "two_d_mode", True),
        ("ekf_navigation", "imu0", "/raw_imu"),
    ],
)
def test_invalid_configuration_rejected_before_starting_nodes(section, key, value):
    config = yaml.safe_load((Path(__file__).parents[1] / "config/localization.yaml").read_text())
    config[section]["ros__parameters"][key] = value
    with pytest.raises(ValueError):
        navigation_parameters(config)
