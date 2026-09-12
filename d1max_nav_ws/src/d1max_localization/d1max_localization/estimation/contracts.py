"""Versioned estimator boundary. No ROS, SDK, filesystem or runtime lifecycle."""

from dataclasses import dataclass
import math
import numpy as np
from ..math_utils import (
    Pose3,
    normalize_quaternion,
    quaternion_conjugate,
    rotate_vector,
    compose,
    inverse,
)


def vector(value, size=3, bound=1e6):
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise ValueError("invalid vector shape")
    if any(type(x) not in (int, float) or not math.isfinite(x) or abs(x) > bound for x in value):
        raise ValueError("invalid vector value")
    return tuple(float(x) for x in value)


def positive_time(value):
    if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
        raise ValueError("invalid time")
    return float(value)


def fresh(stamp, now, limit, future=0.01):
    return math.isfinite(stamp) and stamp > 0 and -future <= now - stamp <= limit


def covariance(values, floors=(0.0,) * 6):
    matrix = np.asarray(vector(values, 36, 1e9)).reshape(6, 6)
    if not np.allclose(matrix, matrix.T, atol=1e-7) or np.linalg.eigvalsh(matrix)[0] < -1e-7:
        raise ValueError("invalid covariance")
    matrix = (matrix + matrix.T) * 0.5
    for i, floor in enumerate(floors):
        matrix[i, i] += max(0.0, floor - matrix[i, i])
    return matrix.reshape(36).tolist()


def sample_pose(value):
    return Pose3(
        vector(value["position"]), normalize_quaternion(vector(value["orientation"], 4, 2.0))
    )


def epoch_header(value, now, max_age=0.3):
    if (
        value.get("schema") != 1
        or type(value.get("epoch")) is not int
        or not 1 <= value["epoch"] < 2**64
    ):
        raise ValueError("invalid epoch")
    if type(value.get("valid")) is not bool or type(value.get("fault")) is not bool:
        raise ValueError("invalid validity flags")
    if not fresh(positive_time(value["received_at_unix"]), now, max_age):
        raise ValueError("stale envelope")
    if value["valid"] and value["fault"]:
        raise ValueError("faulty valid sample")
    return value["epoch"], value["valid"]


@dataclass(frozen=True)
class MotionState:
    stamp: float
    pose: Pose3
    world_velocity: tuple
    angular: tuple
    pose_covariance: list
    twist_covariance: list
    source_stamp: float
    imu_stamp: float
    extrapolation: float = 0.0

    @property
    def linear(self):
        return rotate_vector(quaternion_conjugate(self.pose.orientation), self.world_velocity)


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def body_state(state, body_to_tracking):
    """Change reference point AND axes. ROS twist is in the child body frame."""
    tracking_to_body = inverse(body_to_tracking)
    pose = compose(state.pose, tracking_to_body)
    rotation = body_to_tracking.orientation
    angular = rotate_vector(rotation, state.angular)
    sensor_velocity = rotate_vector(rotation, state.linear)
    arm_velocity = cross(angular, body_to_tracking.position)
    linear = tuple(a - b for a, b in zip(sensor_velocity, arm_velocity))
    # v_body = R_body_tracking v_tracking + [r_body]x R_body_tracking omega_tracking
    r = body_to_tracking.position
    skew = np.array([[0.0, -r[2], r[1]], [r[2], 0.0, -r[0]], [-r[1], r[0], 0.0]])
    rot = np.column_stack(
        [
            rotate_vector(rotation, axis)
            for axis in ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
        ]
    )
    j = np.zeros((6, 6))
    j[:3, :3] = rot
    j[:3, 3:] = skew @ rot
    j[3:, 3:] = rot
    twist_cov = j @ np.asarray(state.twist_covariance).reshape(6, 6) @ j.T
    # Pose attitude errors are fixed/world axes, not body axes.
    arm = rotate_vector(state.pose.orientation, tracking_to_body.position)
    arm_skew = np.array([[0.0, -arm[2], arm[1]], [arm[2], 0.0, -arm[0]], [-arm[1], arm[0], 0.0]])
    j = np.eye(6)
    j[:3, 3:] = -arm_skew
    pose_cov = j @ np.asarray(state.pose_covariance).reshape(6, 6) @ j.T
    return pose, linear, angular, pose_cov.reshape(36).tolist(), twist_cov.reshape(36).tolist()
