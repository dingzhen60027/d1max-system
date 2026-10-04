"""Only a source-audited, versioned committed route may drive native reference.

Display Path messages have no authority. Delivery ordering, task identity and
geometry identity are deliberately distinct; callbacks never restamp sensors.
"""
import math
from .source_route_ros import from_message


class RouteIngress:
    def __init__(self, session_id, map_version_id):
        if not session_id or not map_version_id:
            raise ValueError('explicit_route_session_and_map_required')
        self.session_id, self.map_version_id = session_id, map_version_id
        self.last_sequence = 0
        self.last_stamp = 0.
        self.last_stamp_ns = 0
        self.active_identity = None
        self.retired = set()

    def accept(self, message, *, now, context):
        sec, nsec = message.source_stamp.sec, message.source_stamp.nanosec
        if sec < 0 or not 0 <= nsec < 1_000_000_000:
            raise ValueError('committed_route_delivery_timestamp_invalid')
        stamp_ns = sec*1_000_000_000+nsec
        stamp = stamp_ns*1e-9
        if (message.schema_version != 2 or message.session_id != self.session_id
                or not math.isfinite(now) or not -.1 <= now-stamp <= 2.
                or not message.task_id or not message.route_id or not message.route_hash):
            raise ValueError('committed_route_delivery_invalid')
        # Source time is a freshness fact, not a delivery counter. Distinct
        # commands can share one paused-clock tick, and Unix float seconds
        # cannot represent individual nanoseconds. Cancellation of the exact
        # owner must also remain possible after a small clock regression.
        if (message.delivery_sequence <= self.last_sequence
                or message.active and stamp_ns < self.last_stamp_ns):
            return None
        source = message.snapshot
        identity = (message.task_id, message.route_id, message.route_hash)
        if (source.session_id != message.session_id or source.task_id != message.task_id
                or source.route_id != message.route_id or source.route_hash != message.route_hash
                or source.map_version_id != self.map_version_id):
            raise ValueError('committed_route_identity_mismatch')
        if source.path.header.stamp != message.source_stamp:
            raise ValueError('committed_route_delivery_stamp_mismatch')
        snapshot = from_message(source)
        if message.active:
            if identity in self.retired:
                raise ValueError('retired_committed_route_cannot_resume')
            if context != (self.session_id, source.localization_epoch, source.localization_seed_id):
                raise ValueError('committed_route_localization_context_mismatch')
            if not source.preview_ready:
                raise ValueError('committed_route_not_ready')
            if self.active_identity is not None and self.active_identity != identity:
                raise ValueError('old_committed_route_not_retired')
            self.active_identity = identity
        else:
            # An old task's cancellation cannot erase a newer committed route.
            if self.active_identity != identity:
                return None
            # Do not partially mutate the owner if the bounded session is full.
            if identity not in self.retired and len(self.retired) >= 128:
                raise ValueError('route_session_retirement_capacity_exceeded')
            self.retired.add(identity)
            self.active_identity = None
        self.last_sequence, self.last_stamp = message.delivery_sequence, stamp
        self.last_stamp_ns = stamp_ns
        return snapshot


def annotate_native_reference(target, source, *, context_sequence):
    """Keep source route semantics through the existing native Path interface."""
    target.schema_version = 2
    target.point_reference = 'ground'
    for name in ('task_id', 'route_id', 'route_hash', 'map_version_id',
                 'localization_epoch', 'localization_seed_id'):
        setattr(target, name, getattr(source, name))
    target.context_sequence = context_sequence
    # Source-map preview has no continuous-odom anchor. Empty != verified.
    target.anchor_id = ''
    target.anchor_revision = 0
    target.map_geometry_revision = context_sequence
    if not target.path.poses:
        return
    segments = {segment.segment_id: segment for segment in source.segments}
    labels = list(source.edge_segments) + [source.edge_segments[-1]]
    target.point_segment_ids = labels
    target.point_segment_kinds = [segments[label].kind for label in labels]
    target.point_required_modes = [segments[label].required_mode for label in labels]
    first = segments[labels[0]]
    target.segment_id, target.segment_kind, target.required_mode = first.segment_id, first.kind, first.required_mode


def spline_matches_route(message, source, *, context_sequence, anchor=None):
    """Match real body-center output against one explicit coordinate profile.

    Default is the current source-map preview, which has no odom anchor.
    A continuous-odom caller must supply the ACTUAL admitted ControlAnchor;
    accepting an arbitrary nonempty anchor string would fake that integration.
    This validates identity/semantics only, not native collision or freshness.
    """
    if (source is None or message.schema_version != 2
            or message.point_reference != 'body_center'
            or message.context_sequence != context_sequence
            or any(getattr(message, name) != getattr(source, name) for name in (
                'session_id', 'task_id', 'route_id', 'route_hash', 'map_version_id',
                'localization_epoch', 'localization_seed_id'))):
        return False
    parts = {part.segment_id: part for part in source.segments}
    segment = parts.get(message.segment_id)
    if (segment is None or message.segment_kind != segment.kind
            or message.required_mode != segment.required_mode):
        return False
    if anchor is None:
        return (message.frame_id == source.frame_id == 'd1max_loc_map'
                and message.anchor_id == '' and message.anchor_revision == 0
                and message.map_geometry_revision == context_sequence)
    from .control_frame_contract import ControlAnchor
    return (isinstance(anchor, ControlAnchor)
        and (anchor.context.session, anchor.context.epoch, anchor.context.seed, anchor.context.map_version)
            == (source.session_id, source.localization_epoch, source.localization_seed_id, source.map_version_id)
        and message.frame_id == 'd1max_loc_odom'
        and message.anchor_id == anchor.anchor_id
        and message.anchor_revision == anchor.revision
        and message.map_geometry_revision == anchor.revision)
