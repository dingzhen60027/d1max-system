"""Algorithm-neutral, visualization-only contracts for native local-plan debug.

No target is inferred here: every vertex comes from the native planner's actual
selected-reference / projection / local-target snapshot, never global-path
resampling or the displayed trajectory. These graphics authorize no execution.
"""
from dataclasses import dataclass
import math

import numpy as np

from .live_scan_contract import fresh

DEBUG_TIMEOUT = 2.
MAX_DEBUG_POINTS = 4096
MAX_DEBUG_COORDINATE_M = 10000.
MAX_DEBUG_ARC_M = 10000.
REFERENCE_REJECTION_PHASES = frozenset(('reference_rejected_odometry',
                                      'reference_rejected_frame', 'reference_rejected_geometry'))
INVALID_PHASES = frozenset(('reference_replaced', 'cancelled', 'failed', 'completed',
                            'emergency_stop', 'debug_overflow', 'debug_invalid',
                            'failed_reference_search', 'failed_rebound_search',
                            'failed_optimization', 'failed_final_collision',
                            'failed_current_validation',
                            'failed_dynamics', 'waiting_environment',
                            'failed_reference_geometry', 'failed_reference_search_budget',
                            'failed_reference_target_occupied', 'failed_reference_search_collision',
                            'failed_reference_start_occupied', 'failed_reference_lattice_occupied',
                            'failed_reference_outside_map',
                            'waiting_goal_reached', 'waiting_sensor_map', 'waiting_recheck',
                            'waiting_observed_space')) | REFERENCE_REJECTION_PHASES


def attempt_marker_contract(markers, *, session_id, generation, frame_id, issued_at, now):
    """Bounded native attempt graphics; never evidence of accepted planning.

    Duck-typed ROS markers keep this validator independent of ROS imports.
    Session/generation are explicit in every namespace, including DELETEALL.
    A native attempt's actual geometry and source time are preserved verbatim.
    """
    if not 1 <= len(markers) <= 8:
        raise ValueError('attempt_marker_count')
    prefix = f'local_attempt/{session_id}/{generation}/'
    source_ns, adds, point_count, lifetime = None, 0, 0, DEBUG_TIMEOUT
    for marker in markers:
        stamp_ns = marker.header.stamp.sec*1000000000+marker.header.stamp.nanosec
        stamp = stamp_ns*1e-9
        if (marker.header.frame_id != frame_id or not marker.ns.startswith(prefix)
                or marker.ns[len(prefix):] not in ('clear', 'reference', 'target', 'blocked', 'detour', 'projection')
                or not fresh(stamp, now, DEBUG_TIMEOUT) or stamp < issued_at
                or source_ns is not None and stamp_ns != source_ns
                or marker.action not in (0, 3)):
            raise ValueError('attempt_marker_context_or_time')
        source_ns = stamp_ns
        point_count += len(marker.points)
        if point_count > MAX_DEBUG_POINTS:
            raise ValueError('attempt_marker_point_budget')
        if marker.action == 3:  # DELETEALL still requires the current identity.
            continue
        if marker.type not in (2, 4, 8) or marker.frame_locked:
            raise ValueError('attempt_marker_type')
        coords = [[p.x, p.y, p.z] for p in marker.points]
        position, orientation = marker.pose.position, marker.pose.orientation
        coords.append([position.x, position.y, position.z])
        values = np.asarray(coords, dtype=float)
        color = np.asarray([marker.color.r, marker.color.g, marker.color.b, marker.color.a])
        scales = np.asarray([marker.scale.x, marker.scale.y, marker.scale.z])
        q = np.asarray([orientation.x, orientation.y, orientation.z, orientation.w])
        duration = marker.lifetime.sec+marker.lifetime.nanosec*1e-9
        if (not np.isfinite(values).all() or np.abs(values).max() > MAX_DEBUG_COORDINATE_M
                or not np.isfinite(color).all() or (color < 0).any() or (color > 1).any()
                or not np.isfinite(scales).all() or (scales < 0).any() or (scales > 1).any()
                or marker.scale.x <= 0 or not np.isfinite(q).all()
                or abs(np.linalg.norm(q)-1.) > .01 or not 0 < duration <= DEBUG_TIMEOUT
                or marker.mesh_resource or marker.text):
            raise ValueError('attempt_marker_geometry_or_style')
        # Diagnostic attempts must not be styled as an accepted green path.
        if color[1] > .7 and color[0] < .3 and color[2] < .4:
            raise ValueError('attempt_marker_must_not_look_accepted')
        adds += 1
        lifetime = min(lifetime, duration, DEBUG_TIMEOUT-max(0., now-stamp))
    return source_ns, lifetime, adds


@dataclass
class DebugSnapshot:
    session_id: str
    generation: int
    plan_id: int
    frame_id: str
    stamp: float
    valid: bool
    phase: str
    selected_reference: list
    projection: list
    local_target: list
    progress_arc_m: float
    target_arc_m: float
    matching_path_headers: bool = True
    stamp_ns: int = 0
    predecessor_id: int = 0
    predecessor_safe: bool = False
    predecessor_check_stamp: float = 0.
    checked_map_source_stamp_ns: int = 0
    checked_body_source_stamp_ns: int = 0
    checked_map_revision: int = 0
    checked_context_sequence: int = 0


class LocalDebugGate:
    """Only render accepted native diagnostics paired with that exact spline.

    Cross-topic delivery can be reordered, so retain at most one validated
    pending diagnostic. Event and plan high-water marks survive clearing; late
    success messages cannot resurrect an invalidated attempt in the same task.
    """
    def __init__(self):
        self.generation = -1
        self.reset(0)

    def reset(self, generation):
        self.generation = generation
        self.last_event_stamp = 0.
        self.last_event_stamp_ns = 0
        self.highest_plan_id = -1
        self.active = self.pending = None
        self.received_mono = -math.inf
        self.phase = 'waiting'

    def clear(self, phase):
        self.active = self.pending = None
        self.phase = phase

    def receive(self, snapshot, *, expected_session, gate, now, mono, spline_id, spline_stamp):
        if self.generation != gate.generation:
            self.reset(gate.generation)
        # A foreign/obsolete packet must neither revive nor erase newer output.
        if (not gate.ready or not gate.active or snapshot.session_id != expected_session
                or snapshot.generation != gate.generation
                or not fresh(snapshot.stamp, now, DEBUG_TIMEOUT)
                or snapshot.stamp < gate.issued_at):
            return 'ignore'
        # Keep source nanoseconds for ordering. At Unix-time magnitudes two
        # distinct ROS stamps can round to the same float seconds value.
        event_ns = snapshot.stamp_ns or round(snapshot.stamp*1000000000)
        if event_ns <= self.last_event_stamp_ns:
            return 'ignore'
        if (type(snapshot.plan_id) is not int or not 0 <= snapshot.plan_id < 2**63
                or snapshot.frame_id != gate.frame_id or type(snapshot.valid) is not bool):
            self.last_event_stamp = snapshot.stamp
            self.last_event_stamp_ns = event_ns
            if type(snapshot.plan_id) is int and 0 <= snapshot.plan_id < 2**63:
                self.highest_plan_id = max(self.highest_plan_id, snapshot.plan_id)
            self.clear('invalid_debug')
            return 'clear'
        if snapshot.valid and snapshot.plan_id <= self.highest_plan_id:
            return 'ignore'
        if not snapshot.valid and snapshot.plan_id < max(self.highest_plan_id, spline_id):
            # Native invalid events carry the last actual trajectory ID. A
            # delayed failure of N must not erase an already-accepted N+1.
            return 'ignore'
        self.last_event_stamp = snapshot.stamp
        self.last_event_stamp_ns = event_ns
        self.highest_plan_id = max(self.highest_plan_id, snapshot.plan_id)
        self.received_mono = mono
        if not snapshot.valid:
            phase = snapshot.phase if snapshot.phase in INVALID_PHASES else 'invalid_debug'
            self.clear(phase)
            return 'clear'
        try:
            if (snapshot.phase != 'accepted' or not snapshot.matching_path_headers
                    or not math.isfinite(snapshot.progress_arc_m)
                    or not math.isfinite(snapshot.target_arc_m)
                    or not 0 <= snapshot.progress_arc_m <= snapshot.target_arc_m <= MAX_DEBUG_ARC_M):
                raise ValueError('invalid_native_debug_metadata')
            local_debug_specs(selected_reference=snapshot.selected_reference,
                              projection=snapshot.projection, local_target=snapshot.local_target)
        except (TypeError, ValueError):
            self.clear('invalid_debug')
            return 'clear'
        self.active = None
        if snapshot.plan_id < spline_id:
            self.pending = None
            self.phase = 'superseded'
            return 'clear'
        self.pending, self.phase = snapshot, 'waiting_for_matching_spline'
        return self.pair(spline_id=spline_id, spline_stamp=spline_stamp, now=now, mono=mono)

    def pair(self, *, spline_id, spline_stamp, now, mono):
        # Native debug is stamped at publication; the spline carries its own
        # trajectory start time. They need not be equal. Identity comes from
        # session/generation/plan_id; both independent times must remain fresh.
        if self.active is not None and self.active.plan_id != spline_id:
            self.active = None
            self.phase = 'waiting_for_native_debug'
        if self.pending is None:
            return 'clear' if self.active is None else 'unchanged'
        if (not fresh(self.pending.stamp, now, DEBUG_TIMEOUT)
                or not 0 <= mono-self.received_mono <= DEBUG_TIMEOUT):
            self.clear('expired')
            return 'clear'
        if self.pending.plan_id < spline_id:
            self.clear('superseded')
            return 'clear'
        if self.pending.plan_id != spline_id or not fresh(spline_stamp, now, DEBUG_TIMEOUT):
            return 'pending'
        self.active, self.pending, self.phase = self.pending, None, 'accepted'
        return 'draw'

    def expire(self, *, now, mono, gate):
        if self.generation != gate.generation:
            self.reset(gate.generation)
            return True
        current = self.active or self.pending
        if current is not None and (not gate.ready or not gate.active
                or not fresh(current.stamp, now, DEBUG_TIMEOUT)
                or not 0 <= mono-self.received_mono <= DEBUG_TIMEOUT):
            self.clear('expired' if gate.ready and gate.active else 'inactive')
            return True
        return False


def local_debug_specs(*, selected_reference, projection, local_target):
    """Validate bounded real planner geometry and return ROS-independent styles."""
    if not 1 <= len(selected_reference) <= MAX_DEBUG_POINTS:
        raise ValueError('debug_reference_size_invalid')
    reference = np.asarray(selected_reference, dtype=float)
    projection, target = np.asarray(projection, dtype=float), np.asarray(local_target, dtype=float)
    if (reference.ndim != 2 or reference.shape[1] != 3
            or projection.shape != (3,) or target.shape != (3,)
            or not np.isfinite(reference).all() or not np.isfinite(projection).all()
            or not np.isfinite(target).all()):
        raise ValueError('debug_geometry_nonfinite_or_malformed')
    if any(np.abs(value).max() > MAX_DEBUG_COORDINATE_M for value in (reference, projection, target)):
        # Rendering-only abuse bound, never a navigation/collision threshold.
        raise ValueError('debug_geometry_outside_rendering_budget')
    return [
        dict(namespace='local_reference_segment', kind='LINE_STRIP', width=.03,
             color=(1., .55, .08, 1.), points=reference.tolist()),
        dict(namespace='local_reference_anchors', kind='POINTS', width=.07,
             color=(.12, .45, 1., 1.), points=reference.tolist()),
        dict(namespace='local_target', kind='SPHERE', width=.18,
             color=(1., .40, .02, 1.), position=target.tolist()),
        dict(namespace='local_reference_projection', kind='SPHERE', width=.10,
             color=(.12, .45, 1., 1.), position=projection.tolist()),
    ]
