from math import pi

import pytest

from d1max_localization.math_utils import (
    FusionGateLimits,
    FusionInnovationGate,
    Pose3,
    base_pose_to_tracking,
    normalize_quaternion,
    passes_gate,
    pose_innovation,
    tracking_pose_to_base,
)
from d1max_localization.motion_utils import StationaryGyroCorrector


def assert_pose_close(actual, expected):
    assert actual.position == pytest.approx(expected.position, abs=1.0e-9)
    assert abs(
        sum(a * b for a, b in zip(actual.orientation, expected.orientation))
    ) == pytest.approx(1.0, abs=1.0e-9)


def test_tracking_transform_round_trip():
    map_to_base = Pose3(
        position=(2.0, -1.0, 0.2),
        orientation=normalize_quaternion((0.0, 0.0, 0.3826834, 0.9238795)),
    )
    base_to_tracking = Pose3(
        position=(0.16143, 0.0, 0.12262),
        orientation=(0.0, 0.0, 0.0, 1.0),
    )
    map_to_tracking = base_pose_to_tracking(map_to_base, base_to_tracking)
    recovered = tracking_pose_to_base(map_to_tracking, base_to_tracking)
    assert_pose_close(recovered, map_to_base)


def test_tracking_offset_rotates_with_base():
    map_to_base = Pose3(
        position=(1.0, 2.0, 0.0),
        orientation=normalize_quaternion((0.0, 0.0, 1.0, 1.0)),
    )
    base_to_tracking = Pose3(
        position=(0.2, 0.0, 0.1),
        orientation=(0.0, 0.0, 0.0, 1.0),
    )
    result = base_pose_to_tracking(map_to_base, base_to_tracking)
    assert result.position == pytest.approx((1.0, 2.2, 0.1), abs=1.0e-9)


def test_gate_rejects_large_yaw_innovation():
    prediction = Pose3((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    measurement = Pose3(
        (0.1, 0.0, 0.0),
        (0.0, 0.0, 0.5**0.5, 0.5**0.5),
    )
    innovation = pose_innovation(measurement, prediction)
    assert innovation.yaw == pytest.approx(pi / 2.0)
    assert not passes_gate(innovation, 0.6, 0.35, 0.55, 0.7)


def test_invalid_quaternion_is_rejected():
    with pytest.raises(ValueError):
        normalize_quaternion((0.0, 0.0, 0.0, 0.0))


def make_fusion_gate():
    return FusionInnovationGate(
        FusionGateLimits(0.6, 0.35, 0.55, 0.7),
        FusionGateLimits(3.0, 1.0, 1.75, 1.8),
    )


def test_fusion_gate_allows_one_initial_alignment_then_becomes_strict():
    gate = make_fusion_gate()
    large_innovation = pose_innovation(
        Pose3((1.5, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        Pose3((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
    )

    first = gate.evaluate(large_innovation)
    second = gate.evaluate(large_innovation)

    assert first.accepted
    assert first.just_locked
    assert not second.accepted
    assert not second.just_locked
    assert gate.alignment_locked


def test_fusion_gate_never_falls_back_without_explicit_reset():
    gate = make_fusion_gate()
    small_innovation = pose_innovation(
        Pose3((0.1, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        Pose3((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
    )
    large_innovation = pose_innovation(
        Pose3((1.5, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        Pose3((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
    )
    assert gate.evaluate(small_innovation).just_locked
    for _ in range(10):
        assert not gate.evaluate(large_innovation).accepted
    assert gate.alignment_locked

    gate.reset()
    assert gate.evaluate(large_innovation).just_locked


def test_stationary_gyro_bias_is_removed_after_initialization():
    corrector = StationaryGyroCorrector(
        required_samples=3,
        initialization_max_gyro=0.03,
        stationary_linear_speed=0.02,
        stationary_gyro_deadband=0.015,
    )

    for _ in range(3):
        velocity, gyro, stationary = corrector.correct(
            (0.0, 0.0, 0.0), (0.001, -0.002, 0.008)
        )

    assert corrector.ready
    assert corrector.bias == pytest.approx((0.001, -0.002, 0.008))
    assert velocity == (0.0, 0.0, 0.0)
    assert gyro == pytest.approx((0.0, 0.0, 0.0))
    assert stationary


def test_real_in_place_rotation_is_not_suppressed():
    corrector = StationaryGyroCorrector(
        required_samples=1,
        initialization_max_gyro=0.03,
        stationary_linear_speed=0.02,
        stationary_gyro_deadband=0.015,
    )
    corrector.correct((0.0, 0.0, 0.0), (0.0, 0.0, 0.005))

    _, gyro, stationary = corrector.correct(
        (0.0, 0.0, 0.0), (0.0, 0.0, 1.0)
    )

    assert gyro[2] == pytest.approx(0.995)
    assert not stationary
