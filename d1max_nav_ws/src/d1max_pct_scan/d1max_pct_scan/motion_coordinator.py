"""Explicit RViz execution -> tracker -> collision filter -> existing SDK gate.

No SDK connection here. All queues are bounded; receipt clocks and original
trajectory stamps remain distinct. Restart always starts locked.
"""
import argparse
from collections import deque
import json
from pathlib import Path
import signal
import threading
import time

from .motion_execution import ExecutionLease, MotionConfig, fresh

PREFIX = '/d1max/live_planning/'


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import String, Bool, Empty
    from std_srvs.srv import SetBool
    from geometry_msgs.msg import Twist
    from d1max_planning_interfaces.msg import ReferencePath, TaggedBspline

    parser = argparse.ArgumentParser()
    parser.add_argument('--session', type=Path, required=True)
    parsed, ros_args = parser.parse_known_args(args)
    session = json.loads((parsed.session/'session.json').read_text())
    if session.get('mode') != 'LIVE_NAVIGATION' or session.get('motion_control_enabled') is not True:
        raise ValueError('Explicit owned motion session required')
    config = MotionConfig(session_id=session['id'], map_version_id=session['version_id'],
                          frame_id=session['frame_id'], **session['motion'])

    class Coordinator(Node):
        def __init__(self):
            super().__init__('motion_coordinator', namespace='/d1max/live_planning')
            self.core = ExecutionLease(config)
            self.last_command_stamp = 0.
            self.command_ids = deque(maxlen=128)
            self.pending_spline = None
            self.sent_spline = None
            self.arm_waiting = False
            self.arm_future = None
            self.request_serial = 0
            self.last_tick = time.monotonic()
            self.task_pub = self.create_publisher(String, PREFIX+'execution_task', 1)
            self.permit_pub = self.create_publisher(String, PREFIX+'execution_permit', 1)
            self.command_pub = self.create_publisher(Twist, PREFIX+'cmd_vel_admitted', 1)
            self.freeze_pub = self.create_publisher(Bool, PREFIX+'execution_frozen', 1)
            self.spline_pub = self.create_publisher(TaggedBspline, PREFIX+'execution_bspline', 1)
            self.status_pub = self.create_publisher(String, PREFIX+'motion_status', 1)
            self.cancel_pub = self.create_publisher(Empty, PREFIX+'cancel', 1)
            self.arm_client = self.create_client(SetBool, PREFIX+'navigation_command_gate/arm')
            self.create_subscription(ReferencePath, PREFIX+'scan_reference', self.reference, 1)
            self.create_subscription(TaggedBspline, PREFIX+'validated_tagged_bspline', self.spline, 1)
            for kind, topic in (('admission', 'execution_admission'), ('tracker', 'tracker_status'),
                                ('gate', 'command_gate_status')):
                self.create_subscription(String, PREFIX+topic,
                    lambda msg, k=kind: self.observe(k, msg), 1)
            self.create_subscription(String, PREFIX+'motion_command', self.operator, 1)
            self.create_subscription(Empty, PREFIX+'cancel', lambda _: self.stop('operator_cancelled'), 1)
            # Replacing a goal/initial pose invalidates execution immediately,
            # not only when a slow global worker finally publishes an empty path.
            from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped
            for kind, topic in ((PointStamped, 'goal3d'), (PoseStamped, 'goal'),
                                (PoseWithCovarianceStamped, 'initialpose')):
                self.create_subscription(kind, PREFIX+topic,
                    lambda _: self.stop('new_goal_or_initial_pose'), 1)
            self.create_timer(.05, self.tick)

        @staticmethod
        def message(data):
            return String(data=json.dumps(data, ensure_ascii=False, allow_nan=False))

        def reference(self, msg):
            p = msg.path
            self.core.reference(session=msg.session_id, generation=msg.generation,
                frame=p.header.frame_id, stamp=p.header.stamp.sec+p.header.stamp.nanosec*1e-9,
                points=[(p.pose.position.x, p.pose.position.y, p.pose.position.z) for p in p.poses]
                       if len(p.poses) <= 20000 else [], now=time.time())
            self.tick()

        def observe(self, kind, msg):
            try:
                value = json.loads(msg.data) if len(msg.data) <= 65536 else None
            except (ValueError, TypeError):
                value = None
            self.core.observe(kind, value, time.monotonic(), time.time())
            if self.core.disarm_requested:
                self.tick()

        def spline(self, msg):
            raw = msg.trajectory
            if (msg.session_id != config.session_id or msg.generation != self.core.generation
                    or not self.core.active or msg.frame_id != config.frame_id
                    or len(raw.pos_pts) > 10000 or len(raw.knots) > 10004):
                return
            stamp = raw.start_time.sec+raw.start_time.nanosec*1e-9
            if stamp >= self.core.issued_at and fresh(stamp, time.time(), 2.):
                if (self.sent_spline is not None and
                        (msg.generation, raw.traj_id) <= self.sent_spline):
                    return
                if (self.pending_spline is not None and
                        raw.traj_id <= self.pending_spline.trajectory.traj_id):
                    return
                self.pending_spline = msg
                self.core.candidate_pending(msg.generation, raw.traj_id, time.monotonic())
                if self.core.pair_wait_active:
                    self.tick()

        def operator(self, msg):
            try:
                value = json.loads(msg.data) if len(msg.data) <= 4096 else {}
                if (not isinstance(value, dict) or value.get('session_id') != config.session_id
                        or not fresh(value.get('stamp'), time.time(), 2.)
                        or value['stamp'] <= self.last_command_stamp
                        or not isinstance(value.get('id'), str) or not 1 <= len(value['id']) <= 64
                        or value['id'] in self.command_ids):
                    return
                self.last_command_stamp = value['stamp']
                self.command_ids.append(value['id'])
                if value.get('action') == 'stop':
                    self.stop('operator_stopped')
                    self.cancel_pub.publish(Empty())
                elif value.get('action') == 'execute' and value.get('generation') == self.core.generation:
                    preflight = self.core.sample('gate', time.monotonic(), time.time(), .6)
                    if preflight.get('preflight_ready') is not True:
                        self.core.reason = 'preflight: '+str(preflight.get('preflight_block_reason', 'waiting_gate'))
                        self.tick()
                        return
                    ok, _ = self.core.begin(time.monotonic(), time.time())
                    if ok:
                        self.pending_spline = self.sent_spline = None
                        self.arm_waiting = True
                self.tick()
            except (ValueError, TypeError, KeyError):
                return

        def stop(self, reason):
            self.core.stop(reason)
            self.arm_waiting = False
            self.pending_spline = None
            self.tick()

        def disarm(self):
            self.request_serial += 1
            self.arm_waiting = False
            self.arm_future = None
            if self.arm_client.service_is_ready():
                req = SetBool.Request()
                req.data = False
                self.arm_client.call_async(req)

        def arm_done(self, future, serial):
            if serial != self.request_serial or not self.core.active:
                return
            self.arm_future = None
            try:
                if not future.result().success:
                    self.core.stop('sdk_gate_refused: '+future.result().message)
            except Exception as exc:
                self.core.stop('sdk_gate_request_failed: '+str(exc)[:150])

        def tick(self):
            now, wall = time.monotonic(), time.time()
            if self.core.active and not 0 <= now-self.last_tick <= .25:
                self.core.stop('execution_clock_or_executor_gap')
            self.last_tick = now
            velocity, frozen = self.core.step(now, wall)
            if self.core.task_generation:
                self.task_pub.publish(self.message(self.core.task()))
            self.permit_pub.publish(self.message(self.core.permit(now, wall)))
            if self.core.disarm_requested:
                self.disarm()
                self.core.disarm_requested = False
            gate = self.core.sample('gate', now, wall, .6)
            # Wait for the permit to be observed by the final gate; do not
            # assume cross-topic/service ordering and do not retry failed arms.
            if (self.arm_waiting and self.core.active and gate.get('arm_ready') is True
                    and gate.get('execution_generation') == self.core.generation
                    and gate.get('wall_time', 0.) >= self.core.issued_at
                    and self.arm_client.service_is_ready()):
                self.arm_waiting = False
                self.request_serial += 1
                serial = self.request_serial
                req = SetBool.Request()
                req.data = True
                self.arm_future = self.arm_client.call_async(req)
                self.arm_future.add_done_callback(lambda f: self.arm_done(f, serial))
            tracker = self.core.sample('tracker', now, wall, .25)
            candidate = self.pending_spline
            if (candidate is not None and self.core.phase in ('waiting_trajectory', 'executing')
                    and gate.get('armed') is True and tracker.get('active') is True
                    and tracker.get('generation') == self.core.generation):
                identity = (candidate.generation, candidate.trajectory.traj_id)
                stamp = candidate.trajectory.start_time.sec+candidate.trajectory.start_time.nanosec*1e-9
                if (identity != self.sent_spline and self.core.spline_admitted(
                        session=candidate.session_id, generation=candidate.generation,
                        frame=candidate.frame_id, trajectory_id=candidate.trajectory.traj_id,
                        source_stamp=stamp, now=now, wall=wall)):
                    self.spline_pub.publish(candidate)
                    self.sent_spline = identity
                    self.pending_spline = None
                elif identity == self.sent_spline or not fresh(stamp, wall, 2.):
                    self.pending_spline = None
                # A candidate arriving before its exact admission is retained
                # within the original two-second source lease, never forwarded
                # merely because another same-generation curve was accepted.
            elif candidate is not None:
                stamp = candidate.trajectory.start_time.sec+candidate.trajectory.start_time.nanosec*1e-9
                if not self.core.active or not fresh(stamp, wall, 2.):
                    self.pending_spline = None
            output = Twist()
            output.linear.x, output.linear.y, output.angular.z = velocity
            self.command_pub.publish(output)
            self.freeze_pub.publish(Bool(data=frozen))
            reason = self.core.ready_reason(now, wall) if not self.core.active else self.core.reason
            if not self.core.active and not reason and gate.get('preflight_ready') is not True:
                reason = 'preflight: '+str(gate.get('preflight_block_reason', 'waiting_gate'))
            self.status_pub.publish(self.message(dict(schema=1, session_id=config.session_id,
                stamp=wall, phase=self.core.phase, reason=reason, stop_reason=self.core.reason,
                generation=self.core.generation, armed=gate.get('armed') is True,
                can_execute=not self.core.active and not reason, velocity=list(velocity),
                max_speed=config.max_speed, max_yaw=config.max_yaw, single_floor_only=True,
                gate_reason=gate.get('arm_block_reason' if self.core.active else 'preflight_block_reason', 'waiting_gate'),
                acceptance_blockers=config.blockers(), sdk_connection_owned=False)))

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = Coordinator()
        while rclpy.ok() and not stop.is_set():
            rclpy.spin_once(node, timeout_sec=.05)
    finally:
        if node is not None and rclpy.ok():
            node.stop('process_shutdown')
            # Deliver the revoke while the ROS context is still open; loss of
            # this process is independently covered by the gate's .35s lease.
            until = time.monotonic()+.2
            while rclpy.ok() and time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=.02)
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
