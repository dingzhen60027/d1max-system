"""Propagation duration and delivery age are different clocks; no ROS graph."""

from dataclasses import replace

import pytest

from d1max_localization.estimation.navigation import NavigationLimits, NavigationState
from d1max_localization.estimation.prediction import InertialPredictor, rotation_step
from d1max_localization.math_utils import Pose3
from test_navigation_estimation import alignment, diagonal, local, locked, pending_local, snapshot


def near_horizon_sample(stamp=10.247):
    value = local(stamp, x=1.2 * (stamp - 10.0))
    value.update(source_stamp_ns="10000000000", imu_stamp_ns="10245000000",
                 world_velocity=[1.2, 0.0, 0.0], angular=[0.0, 0.0, 0.4],
                 extrapolation_sec=max(0.0, stamp - 10.245),
                 reason="predicting", prediction_mode="imu_propagation")
    return value


def test_247ms_propagation_survives_5ms_delivery_without_restamping():
    core = NavigationState()
    value = near_horizon_sample()
    assert core.push_local(value, 10.252)
    sample = core.local[-1]
    assert sample.stamp == pytest.approx(10.247)
    assert sample.source_stamp == 10.0 and sample.imu_stamp == pytest.approx(10.245)
    assert core.local_ready(10.272)
    assert sample is core.local[-1]  # no held-pose restamping


def test_propagation_limit_is_still_enforced_at_prediction_target():
    core = NavigationState()
    assert not core.push_local(near_horizon_sample(10.251), 10.252)
    assert not core.local


def test_transport_and_current_imu_deadlines_remain_independent_and_finite():
    core = NavigationState()
    core.limits = replace(core.limits, max_imu_age=.1)
    value = near_horizon_sample()
    assert core.push_local(value, 10.252)
    assert core.local_ready(10.326)
    assert not core.local_ready(10.328)  # original target + 80ms, not receipt + 80ms
    with pytest.raises(ValueError, match="stale envelope"):
        NavigationState().push_local(value, 10.328)
    # A fresh envelope still cannot renew the enclosed target stamp.
    assert not NavigationState().push_local(dict(value, received_at_unix=10.328), 10.328)
    core.limits = replace(core.limits, max_imu_age=.05)
    assert not core.local_ready(10.296)  # actual IMU remains subject to current age


@pytest.mark.parametrize("reason", ["imu_stale", "lio_stale"])
def test_pending_prediction_cannot_revoke_or_refresh_actual_output(reason):
    core = locked()
    assert core.output(10.2)
    sample, output_stamp, sequence = core.local[-1], core.last_output, core.local_sequence
    assert core.push_local(pending_local(10.23, reason), 10.23)
    assert core.output_ready(10.23)
    assert core.output(10.23) is None
    assert core.local[-1] is sample and core.local_sequence == sequence
    assert core.last_output == output_stamp
    assert core.push_local(pending_local(10.251, reason), 10.251)
    assert not core.output_ready(10.251)  # existing IMU deadline is unchanged
    assert core.last_output == output_stamp


def test_pending_lio_without_a_previous_valid_sample_cannot_revive_output():
    core = locked()
    assert core.push_local(pending_local(10.21, "waiting_lio"), 10.21)
    assert core.push_local(pending_local(10.22, "lio_stale"), 10.22)
    assert not core.local_ready(10.22)
    assert core.output(10.22) is None


def test_timing_diagnostics_keep_original_posterior_pipeline_components():
    predictor = InertialPredictor()
    value = snapshot(10.)
    value["received_at_unix"] = 10.14
    assert predictor.accept(value, 10.145)
    assert predictor.push_imu(10.24, (0., 0., 9.81), (0., 0., 0.), 10.245)
    timing = predictor.timing(10.247)
    assert timing["posterior_processing_sec"] == pytest.approx(.14)
    assert timing["posterior_transport_sec"] == pytest.approx(.005)
    assert timing["posterior_receipt_age_sec"] == pytest.approx(.102)
    assert timing["posterior_age_sec"] == pytest.approx(.247)
    assert timing["imu_age_sec"] == pytest.approx(.007)
    assert not predictor.accept(value, 10.15)  # duplicate cannot renew receipt
    assert predictor.timing(10.267)["posterior_receipt_age_sec"] == pytest.approx(.122)
    nav = NavigationState()
    assert nav.push_local(near_horizon_sample(), 10.252)
    timing = nav.local_timing(10.272)
    assert timing["propagation_sec"] == pytest.approx(.247)
    assert timing["target_age_sec"] == pytest.approx(.025)
    assert timing["posterior_age_sec"] == pytest.approx(.272)
    assert timing["propagation_limit_sec"] == .25


def test_200ms_posterior_interval_retains_truthful_moving_output_without_new_samples():
    """Observed timing shape: 10Hz LIO can skip one scan; IMU remains 200Hz.

    Delayed posterior + timer phase yields a last 247ms prediction, delivered
    5ms later. Its 60ms successor gap is represented honestly, not filled with
    invented 50Hz samples. The actual sample stays valid within its own TTL.
    """
    predictor = InertialPredictor()
    nav = NavigationState(replace(NavigationLimits(), max_imu_age=.1))
    velocity, omega = (1.2, -.3, .1), (0., 0., .4)

    def pose(stamp):
        return Pose3(tuple(v * (stamp - 100.) for v in velocity),
                     rotation_step(omega, stamp - 100.))

    imu_index, posterior_index = 0, 0
    outputs, pending = [], 0
    for tick in range(100):
        target = 100.007 + .02 * tick
        receipt = target + .005
        while 99.895 + .005 * imu_index <= target - .006:
            stamp = 99.895 + .005 * imu_index
            assert predictor.push_imu(stamp, (0., 0., 9.81), omega, target)
            imu_index += 1
        while 99.9 + .1 * posterior_index + .1 <= target:
            source = 99.9 + .1 * posterior_index
            posterior_index += 1
            if posterior_index == 6:  # skip source 100.4, next interval is 200ms
                continue
            p = pose(source)
            value = snapshot(source)
            value.update(received_at_unix=source+.095, position=p.position,
                         orientation=p.orientation)
            value["inertial"]["world_velocity"] = velocity
            assert predictor.accept(value, target)
        state = predictor.evaluate(target)
        if state:
            value = local(state.stamp)
            value.update(position=state.pose.position, orientation=state.pose.orientation,
                         world_velocity=state.world_velocity, angular=state.angular,
                         source_stamp_ns=str(round(state.source_stamp * 1e9)),
                         imu_stamp_ns=str(round(state.imu_stamp * 1e9)),
                         extrapolation_sec=state.extrapolation)
            assert nav.push_local(value, receipt)
        else:
            assert predictor.reason == "lio_stale"
            pending += 1
            assert nav.push_local(pending_local(target, "lio_stale"), receipt)
        if tick % 5 == 0:
            assert nav.accept_map(alignment(target), receipt)
        if tick == 0:
            key, stamp, _ = nav.begin_reset(receipt)
            assert nav.reset_ack(key, stamp, receipt)
        if tick:
            assert nav.push_filtered(target, pose(target), diagonal(), (0., 0., 0.), omega, receipt)
        output = nav.output(receipt)
        if output:
            outputs.append(output)
            assert state is not None
            assert output[0].stamp == pytest.approx(target)
            assert output[1].position == pytest.approx(pose(target).position, abs=1e-5)
        if tick >= 10:
            assert nav.output_ready(receipt), (target, predictor.reason, nav.reason)
        if state is None:
            assert output is None
    assert pending == 2
    stamps = [output[0].stamp for output in outputs]
    assert all(b > a for a, b in zip(stamps, stamps[1:]))
    assert max(b-a for a, b in zip(stamps, stamps[1:])) == pytest.approx(.06)
    assert any(output[0].stamp-output[0].source_stamp == pytest.approx(.247) for output in outputs)
    assert not nav.output_ready(target + .1)
    assert not nav.fault and not nav.filter_fault
