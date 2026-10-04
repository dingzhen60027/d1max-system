"""Pure admission and geometry contracts for live, NO-MOTION SCAN previews."""
from dataclasses import dataclass
from array import array
import hashlib
import math

import numpy as np
from scipy.interpolate import BSpline

from .navigation_task_policy import LocalAction, TaskEvent, task_transition


def fresh(stamp, now, timeout, future=0.1):
    return (type(stamp) in (int, float) and math.isfinite(stamp) and stamp > 0
            and math.isfinite(now) and -future <= now - stamp <= timeout)


def perception_timeout(params):
    """Bound only the ray-map lease; never broaden pose or execution leases."""
    value = params.get('perception_timeout', .5)
    if (type(value) not in (int, float) or not math.isfinite(value)
            or not .1 <= value <= .75):
        raise ValueError('invalid_bounded_perception_timeout')
    if value > .5 and not (
            params.get('collision_policy') == 'official'
            and params.get('execution_mode', 'preview') == 'preview'
            and params.get('perception_backend') == 'per_sensor_rays'):
        raise ValueError('extended_perception_timeout_requires_official_ray_preview')
    if params.get('perception_backend') == 'per_sensor_rays':
        return float(value)
    # Keep the pre-existing deskewed-cloud adapter contract unchanged.
    return params['input_timeout']


def cloud_source_issue(*, frame_id, tracking_frame, stamp, now, timeout, context,
                       sensor_barrier, last_input_stamp, data_size, point_count,
                       max_input_points):
    """Explain the existing fail-closed cloud admission test without changing it."""
    if frame_id != tracking_frame:
        return 'wrong_frame'
    if not fresh(stamp, now, timeout):
        return 'invalid_stale_or_future_stamp'
    if context is None:
        return 'localization_context_unavailable'
    if stamp <= sensor_barrier:
        return 'before_sensor_barrier'
    if stamp <= last_input_stamp:
        return 'nonmonotonic_input_stamp'
    if data_size > 32*1024*1024 or point_count > max_input_points:
        return 'source_size_limit'
    return None


def checked_pose_status_envelope(pose, *, now, timeout=.1):
    """Validate before advancing packet order; false/fault/reset is still real status."""
    if not isinstance(pose, dict):
        raise ValueError('continuous_pose_status_missing')
    ttl, stamp = pose.get('pose_timeout_sec'), pose.get('received_at_unix')
    output = pose.get('output_stamp_sec')
    if (type(ttl) not in (int, float) or not math.isfinite(ttl)
            or not 0 < ttl <= min(.1, timeout)
            or type(pose.get('schema')) is not int or pose['schema'] != 1
            or type(pose.get('epoch')) is not int or pose['epoch'] < 0
            or type(pose.get('valid')) is not bool or type(pose.get('pose_valid')) is not bool
            or type(pose.get('reset_pending')) is not bool
            or pose.get('motion_control_enabled') is not False
            or not isinstance(pose.get('frame_id'), str) or not pose['frame_id']
            or not isinstance(pose.get('body_frame'), str) or not pose['body_frame']
            or not fresh(stamp, now, ttl, future=.01)
            or (output is not None and (type(output) not in (int, float)
                or not math.isfinite(output) or output <= 0 or output > stamp+.01))
            or (pose['valid'] and (output is None or not pose['pose_valid']))):
        raise ValueError('continuous_pose_invalid_envelope')
    return stamp


def validate_continuous_pose(pose, *, epoch, seed, now, map_frame='d1max_loc_map',
                             body_frame='d1max_loc_base_link', timeout=.5, receipt_age=None):
    """The 50 Hz source-timed lease, never a 5 Hz UI heartbeat or motion permit."""
    checked_pose_status_envelope(pose, now=now, timeout=timeout)
    ttl = pose.get('pose_timeout_sec')
    if ((receipt_age is not None and (type(receipt_age) not in (int, float)
             or not math.isfinite(receipt_age) or not 0 <= receipt_age <= ttl))
            or pose['epoch'] != epoch or pose.get('seed_id') != seed
            or pose.get('frame_id') != map_frame or pose.get('body_frame') != body_frame
            or pose.get('valid') is not True or pose.get('pose_valid') is not True
            or pose.get('fault') or pose.get('reset_pending') is not False
            or pose.get('motion_control_enabled') is not False
            or not fresh(pose.get('received_at_unix'), now, ttl, future=.01)
            or not fresh(pose.get('output_stamp_sec'), now, ttl, future=.01)):
        raise ValueError('continuous_pose_invalid_or_stale')
    return epoch, seed


def localization_identity_context(localizer, navigation, *, session_id, now, timeout=.5,
                                  map_frame='d1max_loc_map',
                                  tracking_frame='d1max_loc_tracking'):
    """Fresh confirmed identity only. This NEVER admits a pose or trajectory."""
    if (not isinstance(localizer, dict) or not isinstance(navigation, dict)
            or not session_id or localizer.get('session_id') != session_id
            or not fresh(localizer.get('wall_time'), now, timeout)
            or localizer.get('local_fault')):
        raise ValueError('localization_session_invalid_or_stale')
    epoch, seed = localizer.get('local_epoch'), localizer.get('active_seed_ns')
    embedded, frames = localizer.get('navigation'), localizer.get('frames')
    if (type(epoch) is not int or epoch < 1 or not isinstance(seed, str) or not seed
            or seed != localizer.get('confirmed_seed_ns')
            or type(localizer.get('verified_confirmations')) is not int
            or localizer['verified_confirmations'] < 3
            or not isinstance(embedded, dict) or not isinstance(frames, dict)
            or frames.get('map') != map_frame or frames.get('tracking') != tracking_frame):
        raise ValueError('localization_epoch_seed_or_frame_mismatch')
    for value in (embedded, navigation):
        if (type(value.get('epoch')) is not int or value.get('epoch') != epoch or value.get('seed_id') != seed
                or value.get('fault') or value.get('reset_pending') is True
                or not fresh(value.get('received_at_unix'), now, timeout)):
            raise ValueError('navigation_not_same_localization_context')
    return session_id, epoch, seed


def localization_context(localizer, navigation, *, pose_status=None, session_id, now, timeout=.5,
                         map_frame='d1max_loc_map', tracking_frame='d1max_loc_tracking',
                         body_frame='d1max_loc_base_link', pose_receipt_age=None):
    """Confirmed map identity + continuous pose; map-match quality is separate."""
    context = localization_identity_context(localizer, navigation, session_id=session_id,
        now=now, timeout=timeout, map_frame=map_frame, tracking_frame=tracking_frame)
    _, epoch, seed = context
    validate_continuous_pose(pose_status, epoch=epoch, seed=seed, now=now,
                             map_frame=map_frame, body_frame=body_frame, timeout=timeout,
                             receipt_age=pose_receipt_age)
    return context


def sample_shadow_spline(*, order, knots, points):
    """Same cubic DeBoor domain as the guard, but NOT collision admission.

    Reuses SciPy's BSpline implementation used by trajectory_guard. Importing
    that single-floor guard here would falsely couple visualization to an XY
    height grid. This helper only renders the real, unchanged native geometry.
    """
    pts, knots = np.asarray(points, dtype=float), np.asarray(knots, dtype=float)
    if (type(order) is not int or order != 3 or pts.ndim != 2 or pts.shape[1] != 3
            or not 4 <= len(pts) <= 10000 or knots.ndim != 1
            or len(knots) != len(pts)+order+1 or not np.isfinite(pts).all()
            or not np.isfinite(knots).all() or not np.all(np.diff(knots) > 1e-9)):
        raise ValueError('invalid_native_cubic_spline')
    begin, end = knots[order], knots[len(pts)]
    if not .01 < end-begin <= 120.:
        raise ValueError('invalid_native_spline_duration')
    spline = BSpline(knots, pts, order, extrapolate=False)
    speed_bound = float(np.linalg.norm(spline.derivative().c, axis=1).max())
    spacing = min(.05, .05/max(speed_bound, 1e-9))
    intervals = max(1, math.ceil((end-begin)/spacing))
    if intervals > 4000:
        raise ValueError('native_spline_display_budget_exceeded')
    result = np.asarray(spline(np.linspace(begin, end, intervals+1)))
    if not np.isfinite(result).all():
        raise ValueError('nonfinite_native_spline_samples')
    return result


def admissible_tagged_spline(*, session_id, generation, frame_id, trajectory_id,
                             start_time, expected_session, gate, last_id, now):
    """Only the current tagged native generation can supply displayed geometry."""
    return (gate.ready and gate.active and session_id == expected_session
            and type(generation) is int and generation == gate.generation
            and frame_id == gate.frame_id and type(trajectory_id) is int
            and trajectory_id > last_id and start_time >= max(
                gate.issued_at, getattr(gate, 'trajectory_barrier', 0.))
            and fresh(start_time, now, 2.))


def quaternion_matrix(quaternion):
    q = np.asarray(quaternion, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q) - 1) > .01:
        raise ValueError('invalid_unit_quaternion')
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def transform_xyz(points, translation, quaternion):
    xyz, translation = np.asarray(points), np.asarray(translation, dtype=float)
    if (xyz.ndim != 2 or xyz.shape[1] != 3 or not np.isfinite(xyz).all()
            or translation.shape != (3,) or not np.isfinite(translation).all()):
        raise ValueError('invalid_transform_geometry')
    # Retain the original float64 rotation/accumulation before the wire float32
    # cast; in-place translation avoids a second full float64 temporary.
    transformed = xyz @ quaternion_matrix(quaternion).T
    transformed += translation
    return np.asarray(transformed, dtype='<f4')


def decode_xyz(*, data, fields, point_step, row_step, width, height, bigendian,
               max_input_points=250000, max_output_points=100000):
    """Bounded XYZ extraction; honors fields, byte order and organized-row padding."""
    if (type(width) is not int or type(height) is not int or width < 1 or height < 1
            or width * height > max_input_points or not 12 <= point_step <= 256
            or row_step < width * point_step or len(data) != row_step * height
            or len(data) > 32 * 1024 * 1024):
        raise ValueError('invalid_or_oversize_cloud')
    names = {f[0]: f for f in fields}
    columns, layouts = [], []
    for name in ('x', 'y', 'z'):
        if name not in names:
            raise ValueError('cloud_missing_xyz')
        _, offset, datatype, count = names[name]
        if datatype not in (7, 8) or count != 1 or not 0 <= offset <= point_step-(4 if datatype == 7 else 8):
            raise ValueError('unsupported_xyz_field')
        dtype = ('>' if bigendian else '<') + ('f4' if datatype == 7 else 'f8')
        layouts.append((offset, datatype, np.dtype(dtype)))
        column = np.ndarray((height, width), dtype=dtype, buffer=data,
                            offset=offset, strides=(row_step, point_step)).reshape(-1)
        columns.append(column)
    first_offset, first_type, first_dtype = layouts[0]
    if (row_step == width * point_step
            and all(kind == first_type and offset == first_offset+i*first_dtype.itemsize
                    for i, (offset, kind, _) in enumerate(layouts))):
        # Common XYZ/XYZI/PCL layout: borrow source storage, including point
        # padding. The returned view retains its owner until transform finishes.
        xyz = np.ndarray((width*height, 3), dtype=first_dtype, buffer=data,
                         offset=first_offset, strides=(point_step, first_dtype.itemsize))
    else:
        # Mixed fields, reordered fields and padded organized rows retain their
        # original byte-order/layout semantics.
        xyz = np.column_stack(columns)
    finite = np.isfinite(xyz[:, 0])
    np.logical_and(finite, np.isfinite(xyz[:, 1]), out=finite)
    np.logical_and(finite, np.isfinite(xyz[:, 2]), out=finite)
    finite_count = int(np.count_nonzero(finite))
    if not finite_count:
        raise ValueError('cloud_has_no_finite_points')
    stride = max(1, math.ceil(finite_count / max_output_points))
    # Identical finite-first then stride selection, but gather only selected
    # points when invalid measurements occur; do not copy all finite points.
    selected = xyz[::stride] if finite_count == len(xyz) else xyz[np.flatnonzero(finite)[::stride]]
    return np.asarray(selected, dtype='<f4')


def pack_xyz(points):
    """One owned ROS uint8 payload copy, without an intermediate Python bytes."""
    contiguous = np.ascontiguousarray(points, dtype='<f4')
    payload = array('B')
    if contiguous.size:
        payload.frombytes(memoryview(contiguous).cast('B'))
    return payload


def take_latest_exact(pending, *, rejection, resolve):
    """Select newest eligible exact-TF scan; never let one missing TF block all.

    The bounded caller-owned deque contains source objects, not decoded clouds.
    ``rejection`` preserves all caller freshness/context/stamp checks. ``resolve``
    must perform exact-time lookup and return None only for unavailable TF.
    Newer eligible scans still waiting for their own TF are retained. Once a
    scan is selected, all older scans are discarded, never published backwards.
    """
    retained, dropped, selected, resolved, waits = [], {}, None, None, 0
    for item in reversed(pending):
        reason = 'superseded' if selected is not None else rejection(item)
        if reason:
            dropped[reason] = dropped.get(reason, 0) + 1
            continue
        exact = resolve(item)
        if exact is None:
            retained.append(item)
            waits += 1
        else:
            selected, resolved = item, exact
    pending.clear()
    pending.extend(reversed(retained))
    return selected, resolved, dropped, waits


@dataclass
class ReferenceGate:
    """Local following permission, distinct from the owner's immutable route.

    The strict/execution adapter still withdraws on bad input. The explicitly
    selected no-motion adapter suspends its task and needs fresh trajectory
    proof on recovery. Neither path invents a new global planning request.
    """
    frame_id: str = 'd1max_loc_map'
    body_height: float = .55
    generation: int = 0
    ready: bool = False
    active: bool = False
    barrier: float = 0.
    last_path_stamp: float = 0.
    last_owner_sequence: int = 0
    issued_at: float = 0.
    digest: str = ''
    reason: str = 'waiting_for_localization_and_cloud'
    context: tuple | None = None
    preview_paused: bool = False
    trajectory_barrier: float = 0.

    def pause_preview_reference(self, now):
        """Suspend graphics, not owner task identity; caller admits preview only."""
        return self._preview_task_transition(TaskEvent.INPUT_LOST, now)

    def resume_preview_reference(self, now, context):
        """Keep the task; require new native trajectory proof after recovery."""
        return self._preview_task_transition(TaskEvent.INPUT_RECOVERED, now, context)

    def _preview_task_transition(self, event, now, context=None):
        decision = task_transition(event, route_committed=self.active)
        if not self.active or not math.isfinite(now):
            return False
        if decision.local is LocalAction.SUSPEND:
            if self.preview_paused:
                return False
            self.ready, self.preview_paused = False, True
            self.reason = 'preview_reference_paused_for_sensor'
        elif decision.local is LocalAction.REVALIDATE:
            if not self.preview_paused or context != self.context:
                return False
            self.ready, self.preview_paused = True, False
            self.reason = 'preview_reference_resumed_waiting_new_native_plan'
        else:
            return False
        self.trajectory_barrier = max(self.trajectory_barrier, now)
        return True

    def revoke(self, now, reason, *, advance_barrier=True):
        self.generation += 1
        self.active = False
        self.preview_paused = False
        if advance_barrier:
            self.barrier = max(self.barrier, now)
        self.reason = reason

    def observe(self, valid, now, context=None, *, retain_inactive_generation=False):
        """Return True exactly when a cancel publication is required."""
        # A transient unavailable status is not evidence of a different
        # localization epoch/seed. Still revoke immediately and require a NEW
        # target after recovery; retain the last real tuple for diagnosis.
        changed = self.context is not None and context is not None and context != self.context
        if (self.ready or self.preview_paused) and (not valid or changed):
            self.ready = False
            if context is not None:
                self.context = context
            if retain_inactive_generation and not self.active and not changed:
                # There is no native task to cancel. Repeated ray ready/not-
                # ready transitions must not manufacture new generations or
                # empty Paths while the owner refresh is in flight. Preserve
                # the freshness barrier: this does not admit an old reference.
                self.barrier = max(self.barrier, now)
                self.reason = 'input_stale_or_invalid'
                return False
            self.revoke(now, 'localization_context_changed' if changed else 'input_stale_or_invalid')
            return True
        if valid and not self.ready:
            self.ready = True
            self.context = context
            self.barrier = max(self.barrier, now)
            self.reason = 'ready_requires_new_target'
        return False

    def accept(self, xyz, *, frame_id, stamp, now, body_xyz, owner_sequence=None):
        points = np.asarray(xyz, dtype=float)
        if not self.ready:
            raise ValueError('inputs_not_ready_requires_new_target')
        if owner_sequence is None:
            obsolete = stamp <= max(self.barrier, self.last_path_stamp)
        else:
            # Only the typed ingress may supply this already identity-checked
            # sequence. Equal source time is allowed; no clock is fabricated.
            obsolete = (type(owner_sequence) is not int or owner_sequence <= self.last_owner_sequence
                        or stamp < self.barrier)
        if not fresh(stamp, now, 2.) or obsolete:
            raise ValueError('obsolete_or_stale_reference')
        self.last_path_stamp = max(self.last_path_stamp, stamp)
        if owner_sequence is not None:
            self.last_owner_sequence = owner_sequence
        if (frame_id != self.frame_id or points.ndim != 2 or points.shape[1] != 3
                or not 2 <= len(points) <= 20000 or not np.isfinite(points).all()):
            raise ValueError('invalid_reference_geometry_or_frame')
        body = np.asarray(body_xyz, dtype=float)
        if body.shape != (3,) or not np.isfinite(body).all():
            raise ValueError('invalid_body_position')
        steps = np.linalg.norm(np.diff(points, axis=0), axis=1)
        if steps.max() > 1. or steps.sum() < .05 or steps.sum() > 2000.:
            raise ValueError('reference_discontinuous_or_degenerate')
        digest = hashlib.sha256(np.asarray(points, dtype='<f8').tobytes()).hexdigest()
        if self.active and digest == self.digest:
            # A confirmed owner's refresh of this exact active route is an
            # acknowledgement, not a new start. Normal progress may place the
            # robot far from route[0]. All message/context/geometry checks still
            # apply; a new or inactive route must satisfy start admission below.
            return False
        if np.linalg.norm(points[0] + [0, 0, self.body_height] - body) > 1.:
            raise ValueError('reference_start_not_near_live_body')
        self.generation += 1
        self.active, self.issued_at, self.digest = True, stamp, digest
        self.preview_paused = False
        self.reason = 'shadow_reference_active_no_motion'
        return True
