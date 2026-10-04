"""Validate acquisition-time projection evidence without ROS initialization.

The navigation publisher owns epoch/seed and all three source times. Neither a
task heartbeat nor a receipt timestamp can turn an old pose into a new sample.
"""
from dataclasses import dataclass
import math

from .ray_projection import checked_pose, ProjectionError


def stamp_ns(stamp):
    if type(stamp.sec) is not int or type(stamp.nanosec) is not int or stamp.sec < 0 or not 0 <= stamp.nanosec < 10**9:
        raise ProjectionError('invalid_navigation_source_time')
    return stamp.sec * 10**9 + stamp.nanosec


@dataclass(frozen=True)
class AtomicProjectionState:
    identity: tuple
    source_ns: int
    posterior_ns: int
    imu_ns: int
    local_body: object
    global_body: object


def validate(message, *, session_id, map_version_id, now_ns,
             map_frame='d1max_loc_map', odom_frame='d1max_loc_odom',
             body_frame='d1max_loc_base_link'):
    if (message.schema_version != 2 or not message.usable
            or message.session_id != session_id or message.map_version_id != map_version_id
            or message.localization_epoch < 1 or not message.localization_seed_id):
        raise ProjectionError('atomic_navigation_identity_invalid')
    source, posterior, imu = map(stamp_ns, (message.source_stamp, message.posterior_stamp, message.imu_stamp))
    if (not 0 < posterior <= source or not 0 < imu <= source
            or not 0 <= now_ns-source <= 400_000_000
            or not 0 <= now_ns-posterior <= 400_000_000
            or not 0 <= now_ns-imu <= 100_000_000
            or not math.isfinite(message.extrapolation_sec)
            or not 0 <= message.extrapolation_sec <= .1
            or abs((source-imu)*1e-9-message.extrapolation_sec) > 1e-5):
        raise ProjectionError('atomic_navigation_measurement_stale_or_invalid')
    poses = []
    for odom, frame in ((message.local_odometry, odom_frame), (message.global_odometry, map_frame)):
        if (stamp_ns(odom.header.stamp) != source or odom.header.frame_id != frame
                or odom.child_frame_id != body_frame):
            raise ProjectionError('atomic_navigation_pair_frame_or_time_mismatch')
        p, q, v = odom.pose.pose.position, odom.pose.pose.orientation, odom.twist.twist
        if any(not math.isfinite(getattr(vector, axis)) for vector in (v.linear, v.angular) for axis in 'xyz'):
            raise ProjectionError('atomic_navigation_velocity_nonfinite')
        poses.append(checked_pose((p.x, p.y, p.z), (q.x, q.y, q.z, q.w)))
    return AtomicProjectionState((session_id, message.localization_epoch, message.localization_seed_id),
        source, posterior, imu, *poses)
