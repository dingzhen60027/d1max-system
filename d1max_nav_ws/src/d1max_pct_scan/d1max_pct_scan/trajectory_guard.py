"""Independent measured-ground admission for real SCAN output, never a planner.

Only an unchanged, approved TaggedBspline is forwarded. Rejection revokes the
task and asks the route coordinator to cancel; it does not keep an older plan
moving, snap to the map, edit the free mask, or synthesize a fallback curve.
"""
from dataclasses import dataclass
import json
import math
from pathlib import Path
import time

import numpy as np
from scipy.interpolate import BSpline
from d1max_pct_planner.measured_grid import MeasuredGrid


def sample_checked_spline(grid, *, order, knots, points, body_height=0.55,
                          height_tolerance=0.5, max_ground_step=0.15,
                          max_sample_spacing=0.025, max_sample_period=0.05):
    """Use the same degree/knots/domain as vendor evaluateDeBoorT.

    The derivative-control-point norm bounds spline speed. Sampling therefore
    limits *arc length*, not just distance between endpoints of a curved span.
    MeasuredGrid validates every supercovered cell of every resulting segment.
    This is sampled geometric admission, not robot swept-volume certification.
    """
    pts, knots = np.asarray(points, dtype=float), np.asarray(knots, dtype=float)
    if (type(order) is not int or order != 3 or pts.ndim != 2 or pts.shape[1] != 3
            or not 4 <= len(pts) <= 10000 or knots.ndim != 1
            or len(knots) != len(pts) + order + 1
            or not np.isfinite(pts).all() or not np.isfinite(knots).all()
            or not np.all(np.diff(knots) > 1e-9)):
        raise ValueError('invalid_cubic_spline')
    for name, value, ceiling in [('body_height', body_height, 1.0),
                                  ('height_tolerance', height_tolerance, 0.5),
                                  ('max_ground_step', max_ground_step, 0.15),
                                  ('max_sample_spacing', max_sample_spacing, 0.025),
                                  ('max_sample_period', max_sample_period, 0.05)]:
        if not math.isfinite(value) or not 0 < value <= ceiling:
            raise ValueError('unsafe_' + name)
    begin, end = knots[order], knots[len(pts)]
    duration = end - begin
    if not 0.01 < duration <= 120.0:
        raise ValueError('invalid_spline_duration')
    spline = BSpline(knots, pts, order, extrapolate=False)
    derivative = spline.derivative()
    speed_bound = float(np.max(np.linalg.norm(derivative.c, axis=1)))
    spacing = min(max_sample_spacing, grid.resolution / 8.0)
    period = min(max_sample_period, spacing / max(speed_bound, 1e-9))
    intervals = max(1, int(np.ceil(duration / period)))
    if intervals > 20000:
        raise ValueError('spline_validation_budget_exceeded')
    times = np.unique(np.concatenate((np.linspace(begin, end, intervals + 1),
                                      knots[(knots > begin) & (knots < end)])))
    samples = np.asarray(spline(times), dtype=float)
    if not np.isfinite(samples).all():
        raise ValueError('nonfinite_spline_evaluation')
    checked = grid.validate_path(samples, max_ground_step_m=max_ground_step)
    max_height_error = 0.0
    # Check both sample endpoints against every ground cell touched by that
    # segment. Do not merely look up ground below sparse control points.
    for first, second in zip(samples[:-1], samples[1:]):
        cells = grid.segment_cells(first[:2], second[:2])
        ground = np.array([grid.height[cell] for cell in cells]) + body_height
        error = float(max(np.max(np.abs(first[2] - ground)),
                          np.max(np.abs(second[2] - ground))))
        max_height_error = max(max_height_error, error)
        if error > height_tolerance:
            raise ValueError('spline_height_outside_measured_ground_envelope')
    return {**checked, 'sample_count': len(samples), 'duration': float(duration),
            'max_height_error_m': max_height_error,
            'arc_length_sampling_bound_m': float(speed_bound * period),
            'grid_sha256': grid.sha256}, samples


@dataclass
class TaskLease:
    generation: int = 0
    active: bool = False
    issued_at: float = 0.0
    received: float = -math.inf


class AdmissionContext:
    """Pure state machine: a failed generation cannot be revived by heartbeat."""
    def __init__(self, session_id, frame_id='d1max_loc_map', task_timeout=0.75):
        if not session_id or not frame_id or not 0 < task_timeout <= 1.0:
            raise ValueError('invalid_guard_context')
        self.session_id, self.frame_id = session_id, frame_id
        self.task_timeout = task_timeout
        self.task = TaskLease()
        self.last_id = -1
        self.cancel_latched = False

    def receive_task(self, message, ros_now, received):
        if message.get('session_id') != self.session_id:
            return False
        generation = message.get('generation')
        active = message.get('active')
        if type(active) is not bool:
            raise ValueError('invalid_task_active')
        if (type(generation) is not int or generation < 0
                or (active and generation == 0)):
            raise ValueError('invalid_task_generation')
        if generation < self.task.generation:
            return False
        if not active:
            self.task = TaskLease(generation=generation)
            return True
        issued = message.get('issued_at')
        if (type(issued) not in (int, float) or not math.isfinite(issued)
                or not math.isfinite(ros_now) or not math.isfinite(received)
                or issued <= 0 or issued > ros_now + 0.2
                or message.get('frame_id') != self.frame_id):
            raise ValueError('invalid_task_context')
        if generation == self.task.generation:
            if not self.task.active or issued != self.task.issued_at:
                return False
        else:
            if ros_now - issued > self.task_timeout:
                raise ValueError('new_task_already_stale')
            self.last_id = -1
            self.cancel_latched = False
        self.task = TaskLease(generation, True, float(issued), received)
        return True

    def ignore_reason(self, session_id, generation, trajectory_id):
        """Old output must not cancel a newer task during a planner handover."""
        if session_id != self.session_id:
            return 'unrelated_session'
        if not self.task.active:
            return 'no_active_task'
        if generation != self.task.generation:
            return 'unrelated_generation'
        if trajectory_id <= self.last_id:
            return 'obsolete_trajectory_id'
        return ''

    def check(self, session_id, generation, frame_id, trajectory_id, start_time,
              ros_now, received):
        if (not self.task.active or not math.isfinite(received)
                or not 0 <= received - self.task.received <= self.task_timeout):
            raise ValueError('task_inactive_or_stale')
        if session_id != self.session_id or generation != self.task.generation:
            raise ValueError('trajectory_session_or_generation_mismatch')
        if frame_id != self.frame_id:
            raise ValueError('trajectory_frame_mismatch')
        if trajectory_id <= self.last_id:
            raise ValueError('nonmonotonic_trajectory_id')
        if (not math.isfinite(start_time) or not math.isfinite(ros_now)
                or start_time + 1e-6 < self.task.issued_at
                or not -0.2 <= ros_now - start_time <= 5.0):
            raise ValueError('trajectory_timestamp_invalid')

    def revoke(self):
        self.task.active = False
        should_cancel = not self.cancel_latched
        self.cancel_latched = True
        return should_cancel


def json_safe(value):
    """Malformed native numbers remain explicit evidence, never invalid JSON."""
    if isinstance(value, dict):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    if isinstance(value, (float, np.floating)):
        return float(value) if math.isfinite(value) else {'nonfinite': str(value)}
    if isinstance(value, np.integer):
        return int(value)
    return value


def spline_audit_payload(message):
    """Bound malformed wire data, retain complete normal SCAN geometry."""
    raw = message.trajectory
    limit = 10000
    return {
        'session_id': message.session_id, 'generation': int(message.generation),
        'frame_id': message.frame_id, 'traj_id': int(raw.traj_id),
        'start_time': {'sec': int(raw.start_time.sec), 'nanosec': int(raw.start_time.nanosec)},
        'order': int(raw.order), 'knots': list(raw.knots[:limit + 4]),
        'pos_pts': [[float(p.x), float(p.y), float(p.z)] for p in raw.pos_pts[:limit]],
        'yaw_pts': list(raw.yaw_pts[:limit]), 'yaw_dt': float(raw.yaw_dt),
        'point_count': len(raw.pos_pts), 'knot_count': len(raw.knots),
        'truncated': len(raw.pos_pts) > limit or len(raw.knots) > limit + 4,
    }


class RejectionAudit:
    """Optional bounded rejection-only artifacts; never deletes old evidence."""
    def __init__(self, directory='', max_records=20):
        if type(max_records) is not int or not 1 <= max_records <= 100:
            raise ValueError('audit_max_records must be in [1, 100]')
        self.directory = Path(directory).expanduser().resolve() if directory else None
        self.max_records = max_records
        self.written = 0
        if self.directory is not None:
            self.directory.mkdir(parents=True, exist_ok=True)
            self.written = len(list(self.directory.glob('rejected_*.json')))

    def write(self, record):
        if self.directory is None or self.written >= self.max_records:
            return ''
        candidate = record['candidate']
        path = self.directory / (
            f"rejected_{time.time_ns()}_g{int(candidate['generation'])}_t{int(candidate['traj_id'])}.json")
        with path.open('x', encoding='utf-8') as stream:
            json.dump(json_safe(record), stream, indent=2, ensure_ascii=False, allow_nan=False)
            stream.write('\n')
        self.written += 1
        return str(path)


class GuardStatus:
    """Ignored stop splines must not erase the cause of a preceding cancel."""
    def __init__(self):
        self.last_error = None

    def rejected(self, detail):
        self.last_error = json_safe(dict(detail))

    def payload(self, current):
        return json_safe({**current, 'last_error': self.last_error})


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Empty, String
    from d1max_planning_interfaces.msg import TaggedBspline

    class TrajectoryGuard(Node):
        def __init__(self):
            super().__init__('trajectory_guard')
            p = lambda name, default: self.declare_parameter(name, default).value
            self.admission = AdmissionContext(p('session_id', ''), p('frame_id', 'd1max_loc_map'),
                                             p('task_timeout', 0.75))
            self.grid = MeasuredGrid(p('planning_grid_path', ''),
                                     minimum_clearance_m=p('minimum_clearance_m', 0.2),
                                     optimization_guard_cells=0)
            self.options = dict(body_height=p('body_height', 0.55),
                                height_tolerance=p('height_tolerance', 0.5),
                                max_ground_step=p('max_ground_step', 0.15))
            self.audit_writer = RejectionAudit(p('audit_directory', ''), p('audit_max_records', 20))
            self.status_memory = GuardStatus()
            self.output = self.create_publisher(TaggedBspline,
                p('output_topic', '/d1max/pct_scan/validated_bspline'), 1)
            self.cancel = self.create_publisher(Empty,
                p('cancel_topic', '/d1max/pct_scan/cancel'), 1)
            self.status = self.create_publisher(String,
                p('status_topic', '/d1max/pct_scan/trajectory_guard_status'), 1)
            self.create_subscription(String, p('task_topic', '/d1max/pct_scan/task'), self.on_task, 1)
            self.create_subscription(TaggedBspline,
                p('input_topic', '/d1max/pct_scan/planning/tagged_bspline'), self.on_trajectory, 1)
            self.create_timer(0.1, self.watchdog)
            self.accepted = self.rejected = self.ignored = 0
            self.last_ros_now = self.clock()

        def clock(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def emit(self, reason, **extra):
            payload = {'session_id': self.admission.session_id, 'generation': self.admission.task.generation,
                       'active': self.admission.task.active, 'reason': reason, 'stamp': self.clock(),
                       'accepted': self.accepted, 'rejected': self.rejected,
                       'ignored': self.ignored, 'audit_records_written': self.audit_writer.written, **extra}
            self.status.publish(String(data=json.dumps(self.status_memory.payload(payload), allow_nan=False)))

        def reject(self, error, candidate=None, validation_started=None):
            self.rejected += 1
            if self.admission.revoke():
                self.cancel.publish(Empty())
            # Cancel is sent before any diagnostic file I/O.
            observed = time.monotonic()
            detail = {'error': str(error), 'stamp': self.clock(),
                      'generation': self.admission.task.generation,
                      'task_issued_at': self.admission.task.issued_at,
                      'task_receipt_age_s': observed - self.admission.task.received,
                      'validation_elapsed_s': (observed - validation_started)
                          if validation_started is not None else None,
                      'audit_file': ''}
            if candidate is not None:
                native = spline_audit_payload(candidate)
                detail['trajectory_id'] = native['traj_id']
                detail['trajectory_start_time'] = native['start_time']
                record = {
                    'schema': 'd1max-rejected-scan-spline-v1',
                    'session_id': self.admission.session_id, 'rejection': detail.copy(),
                    'candidate': native, 'validation_options': self.options,
                    'grid': {'path': self.grid.source, 'sha256': self.grid.sha256,
                             'origin': self.grid.origin.tolist(), 'resolution': self.grid.resolution},
                }
                try:
                    detail['audit_file'] = self.audit_writer.write(record)
                except (OSError, ValueError, TypeError) as audit_error:
                    detail['audit_error'] = str(audit_error)
            self.status_memory.rejected(detail)
            self.get_logger().warning('SCAN trajectory rejected: ' + json.dumps(
                json_safe(detail), ensure_ascii=False, allow_nan=False))
            self.emit('rejected', error=str(error))

        def on_task(self, message):
            try:
                self.admission.receive_task(json.loads(message.data), self.clock(), time.monotonic())
            except (ValueError, TypeError, AttributeError) as error:
                self.reject(error)

        def on_trajectory(self, message):
            validation_started = time.monotonic()
            try:
                raw = message.trajectory
                ignored = self.admission.ignore_reason(message.session_id, message.generation, raw.traj_id)
                if ignored:
                    self.ignored += 1
                    self.emit('ignored', detail=ignored, trajectory_id=raw.traj_id,
                              input_generation=message.generation)
                    return
                start = raw.start_time.sec + raw.start_time.nanosec * 1e-9
                self.admission.check(message.session_id, message.generation, message.frame_id,
                                   raw.traj_id, start, self.clock(), time.monotonic())
                report, _ = sample_checked_spline(self.grid, order=int(raw.order), knots=raw.knots,
                    points=[[point.x, point.y, point.z] for point in raw.pos_pts], **self.options)
                # Validation itself is bounded but not assumed instantaneous.
                self.admission.check(message.session_id, message.generation, message.frame_id,
                                   raw.traj_id, start, self.clock(), time.monotonic())
                self.admission.last_id = raw.traj_id
                self.output.publish(message)  # Exact original geometry and timing.
                self.accepted += 1
                self.emit('accepted', trajectory_id=raw.traj_id, **report)
            except (ValueError, TypeError, OverflowError, IndexError) as error:
                self.reject(error, candidate=message, validation_started=validation_started)

        def watchdog(self):
            stamp = self.clock()
            if self.admission.task.active and (stamp < self.last_ros_now or
                    time.monotonic() - self.admission.task.received > self.admission.task_timeout):
                self.reject('clock_reversed_or_task_heartbeat_stale')
            self.last_ros_now = stamp

    rclpy.init(args=args)
    node = None
    try:
        node = TrajectoryGuard()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
