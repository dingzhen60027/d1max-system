"""Independent continuous-odom admission and acquisition-history evidence.

A map correction cannot revoke a local estimate. Hard estimator identity loss
does revoke history; expired control evidence cannot authorize a newer scan.
"""
from dataclasses import dataclass
import math

from .atomic_projection_state import stamp_ns
from .ray_projection import ProjectionError, checked_pose


@dataclass(frozen=True)
class LocalProjectionState:
    identity: tuple
    source_ns: int
    posterior_ns: int
    imu_ns: int
    valid_until_ns: int
    local_body: object


def validate_local(message, *, session_id, map_version_id, now_ns,
                   odom_frame='d1max_loc_odom', body_frame='d1max_loc_base_link'):
    state = _validate_measurement(message, session_id=session_id,
        map_version_id=map_version_id, odom_frame=odom_frame, body_frame=body_frame)
    if not state.source_ns <= now_ns <= state.valid_until_ns:
        raise ProjectionError('local_navigation_measurement_stale_or_invalid')
    return state


def _validate_measurement(message, *, session_id, map_version_id,
                          odom_frame='d1max_loc_odom', body_frame='d1max_loc_base_link'):
    """Validate the producer's estimate at its unchanged source instant."""
    if (message.schema_version != 2 or not message.usable
            or message.session_id != session_id or message.map_version_id != map_version_id
            or message.localization_epoch < 1 or not message.localization_seed_id):
        raise ProjectionError('local_navigation_identity_invalid')
    source, posterior, imu = map(stamp_ns, (message.source_stamp, message.posterior_stamp, message.imu_stamp))
    deadline = stamp_ns(message.valid_until)
    if (not 0 < posterior <= source or not 0 < imu <= source
            or not 0 < deadline-source <= 100_000_000
            or source-posterior > 400_000_000
            or source-imu > 100_000_000
            or not math.isfinite(message.extrapolation_sec)
            or not 0 <= message.extrapolation_sec <= .1
            or abs((source-imu)*1e-9-message.extrapolation_sec) > 1e-5):
        raise ProjectionError('local_navigation_measurement_stale_or_invalid')
    odom = message.local_odometry
    if (stamp_ns(odom.header.stamp) != source or odom.header.frame_id != odom_frame
            or odom.child_frame_id != body_frame):
        raise ProjectionError('local_navigation_frame_or_time_mismatch')
    p, q, t = odom.pose.pose.position, odom.pose.pose.orientation, odom.twist.twist
    if any(not math.isfinite(getattr(v, axis)) for v in (t.linear, t.angular) for axis in 'xyz'):
        raise ProjectionError('local_navigation_velocity_nonfinite')
    return LocalProjectionState((session_id, message.localization_epoch, message.localization_seed_id),
        source, posterior, imu, deadline, checked_pose((p.x,p.y,p.z),(q.x,q.y,q.z,q.w)))


class _OrderedLocalInbox:
    """One event ordering policy; subclasses select their consumption contract."""
    def __init__(self):
        self.latest = None

    def accept(self, message, *, session_id, map_version_id, now_ns, monotonic):
        from .atomic_navigation_inbox import NavigationEvent,navigation_event_is_new
        try:
            if (type(message.schema_version) is not int or message.schema_version != 2
                    or not hasattr(message, 'valid_until')
                    or type(message.usable) is not bool
                    or message.session_id != session_id or message.map_version_id != map_version_id
                    or type(message.localization_epoch) is not int or message.localization_epoch < 1
                    or not isinstance(message.localization_seed_id, str)
                    or not 0 < len(message.localization_seed_id) <= 256
                    or not math.isfinite(monotonic)):
                return None
            source = stamp_ns(message.source_stamp)
            if source <= 0 or not 0 <= now_ns-source <= 400_000_000:
                return None
            identity = (session_id,message.localization_epoch,message.localization_seed_id)
            if not navigation_event_is_new(identity,source,self.latest):
                return None
            if message.usable:
                state = self.validate_state(message,session_id=session_id,map_version_id=map_version_id,now_ns=now_ns)
                reason = ''
            else:
                reason = message.reason
                if not isinstance(reason,str) or len(reason)>1024:
                    return None
                state, reason = None, reason or 'local_navigation_unavailable'
            event = NavigationEvent(identity,source,monotonic,state,reason)
        except (ValueError,TypeError,AttributeError,OverflowError):
            return None
        self.latest = event
        return event


class LocalNavigationInbox(_OrderedLocalInbox):
    """Current local control/task evidence with the original consumption lease."""
    validate_state = staticmethod(validate_local)

    def usable(self, *, now_ns, monotonic, receipt_timeout_s):
        event = self.latest
        if event is None or event.state is None:
            return False
        state = event.state
        return (0 <= monotonic-event.received <= receipt_timeout_s
                and state.source_ns <= now_ns <= state.valid_until_ns)

    def task_usable(self, *, now_ns, monotonic, receipt_timeout_s):
        """Task continuity only; NEVER a collision or motion authorization.

        Admission validates the producer's original IMU/propagation bounds
        and original consumption deadline. Expiry must hold motion, but need
        not tear down a still identifiable task.
        Explicit producer loss and exhausted source/receipt observation leases
        do pause the task. Raw posterior/IMU ages were validated at the original
        source; delivery time is not a second propagation interval. Duplicate
        receipts cannot refresh any acquisition evidence.
        """
        event = self.latest
        if event is None or event.state is None:
            return False
        state = event.state
        return (0 <= monotonic-event.received <= receipt_timeout_s
                and 0 <= now_ns-state.source_ns <= min(400_000_000, round(receipt_timeout_s*1e9)))


def validate_local_history(message, *, session_id, map_version_id, now_ns):
    """Acquisition geometry, not permission to control at delivery time.

    A valid source-timed estimate can arrive after its 100 ms control lease. It
    remains the producer's historical pose for rays acquired at that instant;
    discarding it creates artificial interpolation holes. The existing 400 ms
    input-age bound, original producer lease and IMU propagation limits remain.
    No timestamp, deadline or measured pose is refreshed here.
    """
    state = _validate_measurement(message, session_id=session_id, map_version_id=map_version_id)
    if not 0 <= now_ns-state.source_ns <= 400_000_000:
        raise ProjectionError('local_navigation_history_outside_input_window')
    return state


class LocalNavigationHistoryInbox(_OrderedLocalInbox):
    """Projector-only history, deliberately without motion/task usability APIs."""
    validate_state = staticmethod(validate_local_history)


def history_identity(inbox):
    """Only previously measured history, not current motion authorization.

    This may survive a soft gap, never a hard event or changed identity. Scan
    admission still requires its acquisition end <= the validated watermark.
    """
    event = inbox.latest
    return None if event is None or event.hard_failure else event.identity
