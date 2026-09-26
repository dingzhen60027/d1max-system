"""Supported endpoint gaps, without ROS graph or synthetic IMU samples."""

import json
from unittest.mock import Mock

import pytest

from d1max_localization.estimation.prediction import InertialPredictor, PredictionLimits
from d1max_localization.lio_predictor import LioPredictor
from test_navigation_estimation import snapshot


def predictor(gap, angular=(0., 0., 0.), dense=False):
    core = InertialPredictor(PredictionLimits(max_imu_gap=.1))
    core.accept(snapshot(), 10.)
    stamps = [10., 10. + gap]
    if dense:
        stamps = [10. + gap * i / 20 for i in range(21)]
    for stamp in stamps:
        assert core.push_imu(stamp, (2., 0., 9.81), angular, stamp)
    return core


@pytest.mark.parametrize("gap", [.05, .08, .1, .1000009])
def test_supported_gap_integrates_real_dt_with_degraded_uncertainty(gap):
    core = predictor(gap)
    before = list(core.imu)
    result = core.evaluate(10. + gap)
    dense = predictor(gap, dense=True).evaluate(10. + gap)
    assert result is not None and core.degraded
    assert result.pose.position[0] == pytest.approx(gap**2)
    assert result.world_velocity[0] == pytest.approx(2 * gap)
    assert result.extrapolation == 0.
    assert core.reason == "predicting_degraded_imu_gap"
    assert core.gap_stats["count"] == 1
    assert core.gap_stats["max_sec"] == pytest.approx(gap)
    assert core.gap_stats["integrated_sec"] == pytest.approx(gap)
    assert not core.gap_stats["rejected_reason"] and not core.fault
    for i in range(6):
        assert result.pose_covariance[7*i] > dense.pose_covariance[7*i]
        assert result.twist_covariance[7*i] > dense.twist_covariance[7*i]
    again = core.evaluate(10. + gap)
    assert again.pose_covariance == result.pose_covariance
    assert core.gap_stats["count"] == 1  # timer replay is not a new gap
    assert list(core.imu) == before  # no interpolation samples inserted
    assert core.epoch == 1


@pytest.mark.parametrize("gap", [.1000011, .15])
def test_above_hard_boundary_is_rejected_without_fake_output(gap):
    core = predictor(gap)
    assert core.evaluate(10. + gap) is None
    assert core.reason == "imu_gap" and core.gap_stats["rejected_reason"] == "duration"


def test_supported_duration_but_high_rotation_is_rejected():
    core = predictor(.08, angular=(0., 0., 1.1))
    assert core.evaluate(10.08) is None
    assert core.gap_stats["max_rotation_rad"] == pytest.approx(.088)
    assert core.gap_stats["rejected_reason"] == "rotation"


def test_temporary_missing_endpoint_does_not_latch_or_change_epoch():
    core = InertialPredictor(PredictionLimits(max_imu_gap=.1))
    core.accept(snapshot(), 10.)
    core.push_imu(10., (0., 0., 9.81), (0., 0., 0.), 10.)
    assert core.evaluate(10.024) is not None
    assert core.evaluate(10.026) is None and core.reason == "imu_stale"
    assert not core.fault and core.epoch == 1
    core.push_imu(10.08, (0., 0., 9.81), (0., 0., 0.), 10.08)
    assert core.evaluate(10.08) is not None and core.degraded
    assert not core.fault and core.epoch == 1
    # An independently corrected posterior moves beyond the gap; normal quality
    # returns in the same epoch rather than permanently retaining degraded state.
    assert core.accept(snapshot(10.08), 10.08)
    core.push_imu(10.085, (0., 0., 9.81), (0., 0., 0.), 10.085)
    assert core.evaluate(10.085) is not None
    assert not core.degraded and core.reason == "predicting"
    assert core.gap_stats["count"] == 0 and core.epoch == 1


def test_long_gap_requires_a_new_real_posterior_not_more_tail_samples():
    core = predictor(.15)
    assert core.evaluate(10.15) is None
    core.push_imu(10.155, (0., 0., 9.81), (0., 0., 0.), 10.155)
    assert core.evaluate(10.155) is None
    assert core.accept(snapshot(10.15), 10.155)
    assert core.evaluate(10.155) is not None and not core.degraded


def test_backward_imu_still_latches_fault():
    core = predictor(.08)
    assert not core.push_imu(10., (0., 0., 9.81), (0., 0., 0.), 10.08)
    assert core.fault == "imu_clock_reset" and core.evaluate(10.08) is None


@pytest.mark.parametrize("limits", [
    PredictionLimits(max_imu_gap=.100001),
    PredictionLimits(max_imu_gap=.1, max_extrapolation=.03),
    PredictionLimits(max_imu_gap=.1, soft_imu_gap=.11),
])
def test_supported_gap_never_loosens_extrapolation_or_configuration_bound(limits):
    with pytest.raises(ValueError):
        InertialPredictor(limits)


def test_default_remains_50ms_and_degraded_diagnostics_are_serialized():
    assert PredictionLimits().max_imu_gap == .05
    node = LioPredictor.__new__(LioPredictor)
    node.core = predictor(.08)
    node.now_s = Mock(return_value=10.08)
    node.pub = Mock()
    node.sequence = 0
    node.last_error = ""
    node.tick()
    result = json.loads(node.pub.publish.call_args.args[0].data)
    assert result["valid"] and result["degraded"] and not result["fault"]
    assert result["imu_gap"]["count"] == 1
    assert result["reason"] == "predicting_degraded_imu_gap"
    assert result["extrapolation_sec"] == 0.
