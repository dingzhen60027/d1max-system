"""Atomic same-source-time body pair; no TF lookup or receipt restamping."""
import math


def local_body_message(*, local, session_id, map_version_id, epoch, seed_id,
                       posterior_stamp, imu_stamp, extrapolation_sec):
    from d1max_planning_interfaces.msg import LocalNavigationState
    from .estimation_ros import stamp_time
    source_ns = local.header.stamp.sec*10**9+local.header.stamp.nanosec
    if (not session_id or not map_version_id or type(epoch) is not int or epoch < 1
            or not seed_id or local.header.frame_id != 'd1max_loc_odom'
            or local.child_frame_id != 'd1max_loc_base_link'
            or any(not math.isfinite(v) for v in (posterior_stamp,imu_stamp,extrapolation_sec))
            or not 0 < round(posterior_stamp*1e9) <= source_ns
            or not 0 < round(imu_stamp*1e9) <= source_ns
            or not 0 <= extrapolation_sec <= .1
            or abs((source_ns-round(imu_stamp*1e9))*1e-9-extrapolation_sec)>1e-5):
        raise ValueError('invalid_local_navigation_state_provenance')
    p,q,t=local.pose.pose.position,local.pose.pose.orientation,local.twist.twist
    if (any(not math.isfinite(getattr(v,a)) for v in (p,t.linear,t.angular) for a in 'xyz')
            or not all(math.isfinite(v) for v in (q.x,q.y,q.z,q.w))
            or abs(math.hypot(q.x,q.y,q.z,q.w)-1.)>.01):
        raise ValueError('invalid_local_navigation_state_geometry')
    return LocalNavigationState(schema_version=1,session_id=session_id,map_version_id=map_version_id,
        localization_epoch=epoch,localization_seed_id=seed_id,local_odometry=local,
        source_stamp=local.header.stamp,posterior_stamp=stamp_time(posterior_stamp),
        imu_stamp=stamp_time(imu_stamp),extrapolation_sec=extrapolation_sec,
        usable=True,reason='continuous_local_measurement')


def body_pair_message(*, local, global_, session_id, map_version_id, epoch, seed_id,
                      posterior_stamp, imu_stamp, extrapolation_sec):
    from d1max_planning_interfaces.msg import NavigationState
    from .estimation_ros import seconds, stamp_time
    source = seconds(local)
    source_ns = local.header.stamp.sec*1_000_000_000 + local.header.stamp.nanosec
    inputs_finite = all(math.isfinite(v) for v in (posterior_stamp,imu_stamp,extrapolation_sec))
    posterior_ns = round(posterior_stamp*1e9) if inputs_finite else 0
    imu_ns = round(imu_stamp*1e9) if inputs_finite else 0
    if (not session_id or not map_version_id or type(epoch) is not int or epoch < 1
            or not isinstance(seed_id, str) or not seed_id
            or local.header.stamp != global_.header.stamp
            or local.header.frame_id != 'd1max_loc_odom'
            or global_.header.frame_id != 'd1max_loc_map'
            or local.child_frame_id != global_.child_frame_id
            or local.child_frame_id != 'd1max_loc_base_link'
            or any(not math.isfinite(v) for v in (source, posterior_stamp, imu_stamp, extrapolation_sec))
            or not 0 < posterior_ns <= source_ns or not 0 < imu_ns <= source_ns
            or not 0 <= extrapolation_sec <= .1
            or abs(max(0.,source-imu_stamp)-extrapolation_sec) > 1e-5):
        raise ValueError('invalid_atomic_navigation_state_provenance')
    for odometry in (local, global_):
        pose, twist = odometry.pose.pose, odometry.twist.twist
        q = pose.orientation
        values = [getattr(vector, axis) for vector in (pose.position, twist.linear, twist.angular)
                  for axis in 'xyz'] + [q.x,q.y,q.z,q.w]
        if any(not math.isfinite(v) for v in values) or abs(math.hypot(q.x,q.y,q.z,q.w)-1.) > .01:
            raise ValueError('invalid_atomic_navigation_state_geometry')
    message = NavigationState()
    message.schema_version, message.session_id, message.map_version_id = 2, session_id, map_version_id
    message.localization_epoch, message.localization_seed_id = epoch, seed_id
    message.local_odometry, message.global_odometry = local, global_
    message.source_stamp = local.header.stamp
    message.posterior_stamp, message.imu_stamp = stamp_time(posterior_stamp), stamp_time(imu_stamp)
    message.extrapolation_sec = extrapolation_sec
    message.usable, message.reason = True, 'same_source_time_local_global_pair'
    return message
