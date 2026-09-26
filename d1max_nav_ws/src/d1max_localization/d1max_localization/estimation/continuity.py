"""Bounded correction of a motion prediction, not a second sensor or estimator.

Only the correction is low-pass/rate limited. Actual robot motion is propagated
first, so a fast moving robot is not slowed to the correction speed. All time
steps use source stamps. Residuals are exposed and added to pose uncertainty.
"""

from dataclasses import replace
import math
from ..math_utils import Pose3, compose, inverse, interpolate_pose, pose_innovation, quaternion_multiply
from .prediction import rotation_step


def residual(predicted, measured):
    delta = pose_innovation(measured, predicted)
    return math.hypot(delta.translation_xy, delta.translation_z), delta.rotation


def correct_pose(predicted, measured, dt, speed, angular_speed, time_constant):
    """SE(3) pose correction; translation and shortest-arc rotation separately."""
    if not math.isfinite(dt) or dt <= 0:
        raise ValueError("correction requires an increasing source stamp")
    distance, angle = residual(predicted, measured)
    alpha = -math.expm1(-dt / time_constant)
    linear_alpha = min(alpha, speed * dt / max(distance, 1e-12))
    angular_alpha = min(alpha, angular_speed * dt / max(angle, 1e-12))
    position = tuple(a + linear_alpha * (b-a)
                     for a, b in zip(predicted.position, measured.position))
    # Reuse the same tested shortest-arc SLERP convention as history alignment.
    orientation = interpolate_pose(
        [(1, predicted), (1000000001, measured)],
        1 + round(angular_alpha * 1000000000), 1000000000,
    ).orientation
    pose = Pose3(position, orientation)
    return pose, residual(pose, measured)


def residual_covariance(values, translation, rotation):
    result = list(values)
    for i in range(3):
        result[7*i] += translation**2
        result[7*(i+3)] += rotation**2
    return result


def continuous_local(previous, measured, limits):
    """Smooth posterior replacement/catch-up while preserving measured motion.

    Twists retain their physical meaning; correction velocity is deliberately
    not fed back as a second motion observation. This is a bounded odometry
    presentation contract shared by navigation/TF, NOT merely RViz cosmetics.
    """
    dt = measured.stamp - previous.stamp
    if not 0 < dt <= limits.max_prediction_horizon + 1e-6:
        raise ValueError("local_continuity_gap")
    velocity = tuple((a+b)*.5 for a, b in
                     zip(previous.world_velocity, measured.world_velocity))
    angular = tuple((a+b)*.5 for a, b in zip(previous.angular, measured.angular))
    predicted = Pose3(
        tuple(p + v*dt for p, v in zip(previous.pose.position, velocity)),
        quaternion_multiply(previous.pose.orientation, rotation_step(angular, dt)),
    )
    distance, angle = residual(predicted, measured.pose)
    if distance > limits.max_local_residual or angle > limits.max_local_angle:
        raise ValueError("local_correction_diverged")
    pose, (distance, angle) = correct_pose(
        predicted, measured.pose, dt, limits.local_correction_speed,
        limits.local_correction_angular_speed, limits.correction_time_constant,
    )
    return replace(measured, pose=pose, pose_covariance=residual_covariance(
        measured.pose_covariance, distance, angle)), (distance, angle)


def resume_local(previous_public, previous_raw, measured, limits):
    """Resume the SAME odometry epoch using its measured endpoint displacement.

    The missing interval stays missing: no IMU integration across it, no
    interpolated/fabricated output samples. Preserve the prior public offset
    while applying the actual new LIO displacement. A gap is availability
    loss, not proof that the coordinate frame or estimator has failed.
    """
    predicted = compose(previous_public.pose,
                        compose(inverse(previous_raw.pose), measured.pose))
    distance, angle = residual(predicted, measured.pose)
    if distance > limits.max_local_residual or angle > limits.max_local_angle:
        raise ValueError("local_correction_diverged")
    # Do not accumulate a large correction budget while output was suspended.
    dt = min(measured.stamp-previous_public.stamp, limits.local_timeout)
    pose, remaining = correct_pose(predicted, measured.pose, dt,
        limits.local_correction_speed, limits.local_correction_angular_speed,
        limits.correction_time_constant)
    return replace(measured, pose=pose, pose_covariance=residual_covariance(
        measured.pose_covariance, *remaining)), remaining
