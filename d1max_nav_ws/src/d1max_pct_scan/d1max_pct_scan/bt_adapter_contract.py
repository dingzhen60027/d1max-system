"""ROS-independent validation used by the production BT action adapters."""
from dataclasses import dataclass
import hashlib
import math
import struct


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def route_fingerprint(path, frame):
    """Identity is exact geometry, not a renewable ROS publication timestamp."""
    if path.header.frame_id != frame or not 2 <= len(path.poses) <= 20000:
        raise ValueError('route_frame_or_size_invalid')
    digest = hashlib.sha256(frame.encode())
    previous, length = None, 0.
    for pose in path.poses:
        if pose.header.frame_id not in ('', frame):
            raise ValueError('route_pose_frame_invalid')
        point = tuple(float(getattr(pose.pose.position, axis)) for axis in 'xyz')
        if not all(math.isfinite(v) and abs(v) < 100000 for v in point):
            raise ValueError('route_geometry_invalid')
        if previous is not None:
            distance = math.dist(previous, point)
            if distance > 1.:
                raise ValueError('route_discontinuity')
            length += distance
        digest.update(struct.pack('<ddd', *point))
        previous = point
    if not .05 <= length <= 2000.:
        raise ValueError('route_length_invalid')
    return digest.hexdigest()


def valid_task(request, session):
    return (request.session_id == session and isinstance(request.task_id, str)
            and 0 < len(request.task_id) <= 128 and request.task_id.isascii()
            and all(c.isalnum() or c in '-_.' for c in request.task_id))


def valid_compute_request(request, session, frame):
    if (getattr(request, 'schema_version', None) != 2 or not valid_task(request, session)
            or request.goal_kind not in ('2d', '3d')
            or type(request.has_goal_yaw) is not bool
            or not math.isfinite(request.goal_yaw_tolerance_rad)
            or not .01 <= request.goal_yaw_tolerance_rad <= .5):
        return False
    frames = (frame,) if request.goal_kind == '2d' else (frame, 'd1max_multifloor_planning')
    q = request.goal.pose.orientation
    return (request.goal.header.frame_id in frames
        and all(math.isfinite(v) for v in (q.x,q.y,q.z,q.w))
        and abs(math.hypot(q.x,q.y,q.z,q.w)-1.) <= .01 and all(
        math.isfinite(getattr(request.goal.pose.position, axis))
        and abs(getattr(request.goal.pose.position, axis)) < 100000 for axis in 'xyz'))


@dataclass
class ArrivalEvidence:
    """Consecutive real, new body samples; never trajectory-clock completion."""
    samples: int = 0
    first_stamp: float = 0.
    last_stamp: float = 0.

    def reset(self):
        self.samples, self.first_stamp, self.last_stamp = 0, 0., 0.

    def observe(self, *, body, endpoint, stamp, valid, body_height, xy_tolerance,
                z_tolerance, min_samples=3, span=.2):
        if (not valid or not math.isfinite(stamp) or stamp <= 0
                or len(body) != 3 or len(endpoint) != 3
                or not all(math.isfinite(v) for v in (*body, *endpoint))
                or math.hypot(body[0]-endpoint[0], body[1]-endpoint[1]) > xy_tolerance
                or abs(body[2]-body_height-endpoint[2]) > z_tolerance):
            self.reset()
            return False
        if stamp <= self.last_stamp:
            return False
        if not self.samples or stamp-self.last_stamp > .2:
            self.samples, self.first_stamp = 0, stamp
        self.samples += 1
        self.last_stamp = stamp
        return self.samples >= min_samples and stamp-self.first_stamp >= span-1e-9
