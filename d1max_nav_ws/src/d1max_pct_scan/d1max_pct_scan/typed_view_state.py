"""Source-timed RViz state and seed validation for the single-floor graph.

These are UI checks, never control authority. Seed submission intentionally
does not require an already localized NavigationState (a circular dependency).
The localization owner still validates sensor readiness before using the seed.
"""
import math

from .atomic_projection_state import validate


class TypedViewState:
    def __init__(self, session_id, map_version_id):
        self.session_id, self.map_version_id = session_id, map_version_id
        self.state = None
        self.received = -math.inf

    def observe(self, message, *, now_ns, monotonic):
        sample = validate(message, session_id=self.session_id,
                          map_version_id=self.map_version_id, now_ns=now_ns)
        if (self.state is not None and sample.identity == self.state.identity
                and sample.source_ns <= self.state.source_ns):
            return False
        self.state, self.received = sample, monotonic
        return True

    def lease(self, *, now_ns, monotonic):
        s = self.state
        if s is None or not 0 <= monotonic-self.received <= .4:
            return 0.
        ages = ((now_ns-s.source_ns)*1e-9, (now_ns-s.posterior_ns)*1e-9,
                (now_ns-s.imu_ns)*1e-9)
        if any(v < 0 for v in ages):
            return 0.
        return max(0., min(.4-ages[0], .4-ages[1], .1-ages[2], .4-(monotonic-self.received)))


def seed_command(message, *, now, session_id, body_z, command_id):
    p, q = message.pose.pose.position, message.pose.pose.orientation
    stamp = message.header.stamp.sec + message.header.stamp.nanosec*1e-9
    values = (p.x, p.y, body_z, q.x, q.y, q.z, q.w)
    covariance = message.pose.covariance
    if (message.header.frame_id != 'd1max_loc_map' or not -.05 <= now-stamp <= 2.
            or message.header.stamp.sec < 0 or not 0 <= message.header.stamp.nanosec < 1000000000
            or not all(math.isfinite(v) for v in values)
            or max(abs(p.x), abs(p.y)) > 10000 or abs(body_z) > 100.
            or abs(math.hypot(q.x, q.y, q.z, q.w)-1.) > .01
            or len(covariance) != 36 or not all(math.isfinite(v) for v in covariance)
            or any(covariance[i*7] < 0 for i in range(6))):
        raise ValueError('初值坐标、时间或姿态无效')
    return dict(id=command_id, session_id=session_id, created_at=now,
                x=p.x, y=p.y, z=body_z, reference='body',
                yaw=math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
