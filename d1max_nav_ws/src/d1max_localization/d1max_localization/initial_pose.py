"""Initial pose contract: map->body composed once with body->tracking.

Same SE(3) composition as go2_localization.base_pose_to_tracking. No ground
estimator, guessed Z correction, map edits, TF publication or filter reset.
"""
import math
from .math_utils import Pose3, compose, normalize_quaternion

def body_to_tracking_transform(offset_body, sdk_to_tracking_yaw):
    if len(offset_body)!=3 or not all(math.isfinite(float(v)) for v in offset_body):
        raise ValueError('invalid body-to-tracking translation')
    yaw=float(sdk_to_tracking_yaw)
    if not math.isfinite(yaw):raise ValueError('invalid body-to-tracking rotation')
    # SDK velocities use R_tracking_body(+yaw), hence this pose uses its inverse.
    return Pose3(tuple(float(v) for v in offset_body),(0.,0.,math.sin(-yaw/2),math.cos(-yaw/2)))

def initial_tracking_pose(command, offset_body, sdk_to_tracking_yaw):
    values=[float(command[k]) for k in ('x','y','z','yaw')]
    if not all(math.isfinite(v) for v in values) or max(abs(v) for v in values[:2])>1e5 or abs(values[2])>100 or abs(values[3])>math.pi:
        raise ValueError('invalid initial pose')
    x,y,z,yaw=values
    pose=Pose3((x,y,z),normalize_quaternion((0.,0.,math.sin(yaw/2),math.cos(yaw/2))))
    reference=command.get('reference')
    if reference=='body':return compose(pose,body_to_tracking_transform(offset_body,sdk_to_tracking_yaw))
    if reference=='tracking':return pose
    if reference is not None:raise ValueError('unknown initial pose reference')
    # Old in-flight mailbox / offline fixtures only. New Web requests always
    # declare reference explicitly; do not reinterpret old XYZ as body XYZ.
    if command.get('heading_frame','tracking')=='body':
        angle=yaw-sdk_to_tracking_yaw
        return Pose3((x,y,z),(0.,0.,math.sin(angle/2),math.cos(angle/2)))
    return pose
