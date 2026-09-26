"""Bounded kinematic prediction under synthetic delivery gaps. No ROS nodes."""

from dataclasses import replace
import math

import pytest

from d1max_localization.estimation.prediction import (
    InertialPredictor, PredictionLimits, rotation_step,
)
from test_navigation_estimation import snapshot


def moving_predictor(*, omega=.4, acceleration=0., max_coast=.1, bias=0.):
    core = InertialPredictor(PredictionLimits(max_imu_gap=.1, max_coast=max_coast))
    posterior = snapshot()
    posterior["inertial"]["world_velocity"] = [1.2, -.3, .1]
    posterior["inertial"]["gyro_bias"] = [0., 0., bias]
    assert core.accept(posterior, 10.)
    assert core.push_imu(10., (acceleration, 0., 9.81), (0., 0., omega + bias), 10.)
    return core


@pytest.mark.parametrize("age", [.025, .05, .075, .1, .1000009])
def test_nonstatic_coast_advances_pose_with_true_source_times(age):
    core = moving_predictor()
    original_samples = list(core.imu)
    state = core.evaluate(10. + age)
    assert state is not None and not core.fault
    assert state.stamp == pytest.approx(10. + age)
    assert state.source_stamp == state.imu_stamp == 10.
    assert state.extrapolation == pytest.approx(age)
    assert state.pose.position == pytest.approx((1.2 * age, -.3 * age, .1 * age))
    assert state.world_velocity == pytest.approx((1.2, -.3, .1))
    assert state.pose.orientation == pytest.approx(rotation_step((0., 0., .4), age))
    assert state.angular == pytest.approx((0., 0., .4))
    assert core.coast_stats["unsupported_sec"] == pytest.approx(age)
    assert core.coast_stats["imu_hold_sec"] == pytest.approx(min(age, .025))
    assert core.coast_stats["duration_sec"] == pytest.approx(max(0., age - .025))
    assert core.coast_stats["rotation_rad"] == pytest.approx(.4 * age)
    assert core.degraded is (age > .025)
    assert core.prediction_mode == ("coasting" if age > .025 else "imu_propagation")
    assert core.reason == ("predicting_coast" if age > .025 else "predicting")
    assert list(core.imu) == original_samples  # no invented measurement samples


def test_old_acceleration_is_not_reused_beyond_original_25ms_hold():
    core = moving_predictor(omega=0., acceleration=2.)
    state = core.evaluate(10.075)
    hold, coast = .025, .05
    expected_velocity = 1.2 + 2. * hold
    expected_x = 1.2 * hold + .5 * 2. * hold**2 + expected_velocity * coast
    assert state.world_velocity[0] == pytest.approx(expected_velocity)
    assert state.pose.position[0] == pytest.approx(expected_x)
    assert state.pose.position[0] != pytest.approx(1.2 * .075 + .075**2)
    assert core.coast_stats["duration_sec"] == pytest.approx(coast)


def test_late_posterior_does_not_restart_old_imu_hold_deadline():
    core = moving_predictor(omega=0., acceleration=2.)
    posterior = snapshot(10.06)
    posterior["inertial"]["world_velocity"] = [1., 0., 0.]
    assert core.accept(posterior, 10.06)
    state = core.evaluate(10.075)
    assert state.pose.position == pytest.approx((.015, 0., 0.))
    assert state.world_velocity == pytest.approx((1., 0., 0.))
    assert state.imu_stamp == 10. and state.source_stamp == pytest.approx(10.06)
    assert state.extrapolation == pytest.approx(.075)
    assert core.coast_stats["imu_hold_sec"] == 0.
    assert core.coast_stats["duration_sec"] == pytest.approx(.015)


def test_coast_covariance_has_explicit_growth_and_no_timer_double_counting():
    core = moving_predictor()
    dense = moving_predictor()
    for i in range(1, 16):
        t = 10. + .005 * i
        assert dense.push_imu(t, (0., 0., 9.81), (0., 0., .4), t)
    measured = dense.evaluate(10.075)
    states = [core.evaluate(10. + t) for t in (.05, .075, .1)]
    coast = .05
    for axis in range(3):
        assert states[1].pose_covariance[7*axis] - measured.pose_covariance[7*axis] == pytest.approx(.25 * 2.**2 * coast**4)
        assert states[1].twist_covariance[7*axis] - measured.twist_covariance[7*axis] == pytest.approx(2.**2 * coast**2)
        assert states[1].pose_covariance[7*(axis+3)] - measured.pose_covariance[7*(axis+3)] == pytest.approx(.25 * coast**4)
        assert states[1].twist_covariance[7*(axis+3)] - measured.twist_covariance[7*(axis+3)] == pytest.approx(coast**2)
    for axis in range(6):
        assert states[0].pose_covariance[7*axis] < states[1].pose_covariance[7*axis] < states[2].pose_covariance[7*axis]
        assert states[0].twist_covariance[7*axis] < states[1].twist_covariance[7*axis] < states[2].twist_covariance[7*axis]
    again = core.evaluate(10.1)
    assert again.pose_covariance == states[-1].pose_covariance
    assert again.twist_covariance == states[-1].twist_covariance


@pytest.mark.parametrize("blackout", [.025, .05, .075, .1])
def test_50hz_timer_delivers_actual_new_estimates_across_batched_arrival(blackout):
    core = moving_predictor()
    # 200Hz real source messages queued during a known synthetic delivery
    # blackout. This models arrival timing, not just arithmetic source cadence.
    packets = [(10. + .005*i, max(10. + .005*i, 10. + blackout)) for i in range(1, 41)]
    pending = list(packets)
    published = []
    modes = []
    for i in range(1, 11):
        wall_now = 10. + .02 * i
        while pending and pending[0][1] <= wall_now + 1e-9:
            stamp, _ = pending.pop(0)
            assert core.push_imu(stamp, (0., 0., 9.81), (0., 0., .4), wall_now)
        state = core.evaluate(wall_now)
        assert state is not None
        elapsed = wall_now - 10.
        assert state.pose.position == pytest.approx((1.2*elapsed, -.3*elapsed, .1*elapsed))
        assert state.pose.orientation == pytest.approx(rotation_step((0., 0., .4), elapsed))
        published.append((wall_now, state.stamp, state.imu_stamp))
        modes.append(core.prediction_mode)
    assert len(published) == 10
    assert [b[0] - a[0] for a, b in zip(published, published[1:])] == pytest.approx([.02]*9)
    assert [b[1] - a[1] for a, b in zip(published, published[1:])] == pytest.approx([.02]*9)
    assert all(stamp == wall for wall, stamp, _ in published)
    if blackout >= .05:
        assert "coasting" in modes
    assert modes[-1] == "imu_propagation" and not core.degraded
    assert core.imu_received == 41 and not core.fault


@pytest.mark.parametrize("age", [.1000011, .15])
def test_coast_hard_deadline_does_not_keep_old_pose_alive(age):
    core = moving_predictor()
    assert core.evaluate(10.1) is not None
    assert core.evaluate(10. + age) is None
    assert core.prediction_mode == "unavailable"
    assert core.reason == "imu_stale" and core.coast_stats["rejected_reason"] == "duration"
    assert core.epoch == 1 and not core.fault


@pytest.mark.parametrize("omega,age", [(3.3, .025), (2., .05), (1.1, .075)])
def test_unsupported_total_rotation_budget_rejects_high_turns(omega, age):
    core = moving_predictor(omega=omega)
    assert core.evaluate(10. + age) is None
    assert core.reason == "coast_rotation"
    assert core.coast_stats["rejected_reason"] == "rotation"
    assert core.coast_stats["rotation_rad"] == pytest.approx(omega * age)
    assert core.prediction_mode == "unavailable" and not core.fault


def test_coast_rotation_uses_bias_corrected_rate():
    core = moving_predictor(omega=.5, bias=1.5)
    state = core.evaluate(10.1)
    assert state is not None and state.angular == pytest.approx((0., 0., .5))
    assert core.coast_stats["rotation_rad"] == pytest.approx(.05)


def test_coast_does_not_bypass_later_real_internal_gap_or_lio_timeout():
    core = moving_predictor()
    assert core.evaluate(10.075)
    assert core.push_imu(10.15, (0., 0., 9.81), (0., 0., .4), 10.15)
    assert core.evaluate(10.15) is None and core.reason == "imu_gap"
    assert core.gap_stats["rejected_reason"] == "duration"
    assert core.accept(snapshot(10.15), 10.15)
    assert core.push_imu(10.155, (0., 0., 9.81), (0., 0., .4), 10.155)
    assert core.evaluate(10.155) is not None and not core.degraded
    for i in range(2, 53):
        stamp = 10.15 + .005*i
        assert core.push_imu(stamp, (0., 0., 9.81), (0., 0., .4), stamp)
    assert core.evaluate(10.41) is None and core.reason == "lio_stale"


def test_new_epoch_or_invalid_posterior_and_clock_fault_revoke_coast():
    core = moving_predictor()
    assert core.evaluate(10.075)
    invalid = dict(schema=1, epoch=2, valid=False, fault=False, received_at_unix=10.08)
    assert core.accept(invalid, 10.08)
    assert core.evaluate(10.08) is None and core.reason == "waiting_lio"
    assert not core.accept(snapshot(10.08), 10.08)  # old epoch cannot revive
    assert core.accept(snapshot(10.08, epoch=2), 10.08)
    assert core.push_imu(10.08, (0., 0., 9.81), (0., 0., .4), 10.08)
    assert core.evaluate(10.09) is not None
    assert core.evaluate(9.99) is None and core.fault == "host_clock_reset"
    assert core.evaluate(10.1) is None


def test_default_keeps_original_no_coast_policy():
    core = moving_predictor(max_coast=PredictionLimits().max_coast)
    assert core.evaluate(10.024) is not None
    assert core.evaluate(10.026) is None and core.reason == "imu_stale"


@pytest.mark.parametrize("overrides", [
    {"max_coast": .101}, {"max_coast": .02}, {"max_coast": None},
    {"coast_accel_sigma": 1.9}, {"coast_accel_sigma": 21.},
    {"coast_angular_accel_sigma": .9}, {"coast_angular_accel_sigma": math.inf},
])
def test_unbounded_or_nonconservative_coast_settings_rejected(overrides):
    with pytest.raises(ValueError):
        InertialPredictor(replace(PredictionLimits(), **overrides))
