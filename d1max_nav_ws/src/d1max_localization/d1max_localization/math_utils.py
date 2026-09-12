from dataclasses import dataclass
from math import acos, atan2, isfinite, pi, sqrt
from typing import Sequence, Tuple


Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]


@dataclass(frozen=True)
class Pose3:
    position: Vector3
    orientation: Quaternion


@dataclass(frozen=True)
class Innovation:
    translation_xy: float
    translation_z: float
    yaw: float
    rotation: float


@dataclass(frozen=True)
class FusionGateLimits:
    max_translation_xy: float
    max_translation_z: float
    max_yaw: float
    max_rotation: float


@dataclass(frozen=True)
class FusionGateDecision:
    accepted: bool
    just_locked: bool


class FusionInnovationGate:
    def __init__(
        self,
        tracking_limits: FusionGateLimits,
        alignment_limits: FusionGateLimits,
    ):
        self.tracking_limits = tracking_limits
        self.alignment_limits = alignment_limits
        self.alignment_locked = False

    def reset(self):
        self.alignment_locked = False

    def evaluate(self, innovation: Innovation) -> FusionGateDecision:
        limits = (
            self.tracking_limits
            if self.alignment_locked
            else self.alignment_limits
        )
        accepted = passes_gate(
            innovation,
            limits.max_translation_xy,
            limits.max_translation_z,
            limits.max_yaw,
            limits.max_rotation,
        )
        just_locked = accepted and not self.alignment_locked
        if just_locked:
            self.alignment_locked = True
        return FusionGateDecision(accepted=accepted, just_locked=just_locked)


def normalize_angle(angle: float) -> float:
    while angle > pi:
        angle -= 2.0 * pi
    while angle < -pi:
        angle += 2.0 * pi
    return angle


def normalize_quaternion(quaternion: Sequence[float]) -> Quaternion:
    if len(quaternion) != 4 or not all(isfinite(value) for value in quaternion):
        raise ValueError("quaternion must contain four finite values")
    norm = sqrt(sum(value * value for value in quaternion))
    if norm < 1.0e-9:
        raise ValueError("quaternion norm is zero")
    return tuple(value / norm for value in quaternion)  # type: ignore[return-value]


def quaternion_conjugate(quaternion: Quaternion) -> Quaternion:
    x, y, z, w = quaternion
    return (-x, -y, -z, w)


def quaternion_multiply(left: Quaternion, right: Quaternion) -> Quaternion:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return normalize_quaternion(
        (
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        )
    )


def rotate_vector(quaternion: Quaternion, vector: Vector3) -> Vector3:
    x, y, z, w = quaternion
    vx, vy, vz = vector
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + y * tz - z * ty,
        vy + w * ty + z * tx - x * tz,
        vz + w * tz + x * ty - y * tx,
    )


def compose(left: Pose3, right: Pose3) -> Pose3:
    rotated = rotate_vector(left.orientation, right.position)
    return Pose3(
        position=tuple(
            left.position[index] + rotated[index] for index in range(3)
        ),  # type: ignore[arg-type]
        orientation=quaternion_multiply(left.orientation, right.orientation),
    )


def inverse(pose: Pose3) -> Pose3:
    orientation = quaternion_conjugate(pose.orientation)
    position = rotate_vector(
        orientation,
        (-pose.position[0], -pose.position[1], -pose.position[2]),
    )
    return Pose3(position=position, orientation=orientation)


def tracking_pose_to_base(map_to_tracking: Pose3, base_to_tracking: Pose3) -> Pose3:
    return compose(map_to_tracking, inverse(base_to_tracking))


def base_pose_to_tracking(map_to_base: Pose3, base_to_tracking: Pose3) -> Pose3:
    return compose(map_to_base, base_to_tracking)


def yaw_from_quaternion(quaternion: Quaternion) -> float:
    x, y, z, w = quaternion
    return atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def pose_innovation(measurement: Pose3, prediction: Pose3) -> Innovation:
    dx = measurement.position[0] - prediction.position[0]
    dy = measurement.position[1] - prediction.position[1]
    dz = measurement.position[2] - prediction.position[2]
    yaw = abs(
        normalize_angle(
            yaw_from_quaternion(measurement.orientation)
            - yaw_from_quaternion(prediction.orientation)
        )
    )
    dot = abs(
        sum(
            measurement.orientation[index] * prediction.orientation[index]
            for index in range(4)
        )
    )
    rotation = 2.0 * acos(max(-1.0, min(1.0, dot)))
    return Innovation(
        translation_xy=sqrt(dx * dx + dy * dy),
        translation_z=abs(dz),
        yaw=yaw,
        rotation=rotation,
    )


def passes_gate(
    innovation: Innovation,
    max_translation_xy: float,
    max_translation_z: float,
    max_yaw: float,
    max_rotation: float,
) -> bool:
    return (
        innovation.translation_xy <= max_translation_xy
        and innovation.translation_z <= max_translation_z
        and innovation.yaw <= max_yaw
        and innovation.rotation <= max_rotation
    )


def interpolate_pose(samples, stamp_ns, max_gap_ns):
    """Linear XYZ + shortest-arc quaternion SLERP. Never extrapolate / restamp.

    samples is a strictly ordered sequence of (integer nanoseconds, Pose3).
    The limit is the entire bracketing gap, not distance to a nearest sample.
    """
    if not samples or stamp_ns <= 0 or max_gap_ns <= 0:
        return None
    left = None
    for time_ns, pose in samples:
        if time_ns == stamp_ns:
            return pose
        if time_ns > stamp_ns:
            if left is None:
                return None
            before_ns, before = left
            gap = time_ns - before_ns
            if gap <= 0 or gap > max_gap_ns:
                return None
            fraction = (stamp_ns - before_ns) / gap
            q0 = normalize_quaternion(before.orientation)
            q1 = normalize_quaternion(pose.orientation)
            dot = sum(a*b for a, b in zip(q0, q1))
            if dot < 0:
                q1 = tuple(-value for value in q1)
                dot = -dot
            from math import sin
            if dot > 0.9995:
                q = normalize_quaternion(tuple(a + fraction*(b-a) for a, b in zip(q0, q1)))
            else:
                angle = acos(max(-1., min(1., dot)))
                q = normalize_quaternion(tuple(
                    (sin((1-fraction)*angle)*a + sin(fraction*angle)*b) / sin(angle)
                    for a, b in zip(q0, q1)))
            return Pose3(tuple(a + fraction*(b-a) for a, b in zip(before.position, pose.position)), q)
        left = (time_ns, pose)
    return None
