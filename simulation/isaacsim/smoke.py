#!/usr/bin/env python3
"""Exercise the original BT Action + route-specific confirmation in Isaac.

Only the normal user interfaces issue a goal/confirmation/cancel. Stop reports
come from the existing writer and are cross-checked against PhysX measurements.
"""
import argparse
from collections import Counter, deque
import json
import math
from pathlib import Path
import time


def navigation_goal(session, xyz, source_stamp):
    from d1max_navigation_bt_interfaces.action import Navigate
    command=Navigate.Goal(schema_version=2, goal_kind='3d', has_goal_yaw=False,
        goal_yaw_tolerance_rad=.15)
    command.goal.header.frame_id=session['frame_id']
    command.goal.header.stamp=source_stamp
    command.goal.pose.position.x, command.goal.pose.position.y, command.goal.pose.position.z=xyz
    command.goal.pose.orientation.w=1.
    return command


def expected_task_result(case, result, cancellation_requested, action_status):
    # ROS Action terminal states: SUCCEEDED=4, CANCELED=5. A failed timeout
    # after requesting cancellation is not a successful cancellation test.
    if case == 'goal':
        return result.get('success') is True and action_status == 4
    return (cancellation_requested and action_status == 5
        and result.get('success') is False
        and result.get('reason') in ('action_cancelled', 'user_cancelled'))


def drain_imu_witness(observer, spin_once, *, budget=1., monotonic=time.monotonic):
    """Fence measured states, then read their exact independent IMU witnesses.

    State and IMU subscriptions can deliver callbacks in either order. Checking
    a continuously growing state list immediately after its final callback
    races the IMU subscription. A fixed evidence window keeps every internal
    missing sample a failure, while giving already-published messages a bounded
    receipt wait. It neither invents source stamps nor extends plant leases.
    """
    samples = list(observer.samples)
    started = monotonic()
    deadline = started + budget
    while True:
        states = [sample for sample in samples if observer.first_imu_ns is not None
            and sample['imu_stamp_ns'] >= observer.first_imu_ns]
        missing = sorted({sample['imu_stamp_ns'] for sample in states}
            - observer.actual_imu_times)
        matched = bool(states) and not missing
        if matched or monotonic() >= deadline:
            return dict(matched=matched, state_count=len(states),
                captured_state_count=len(samples),
                fence_source_stamp_ns=max((sample['source_stamp_ns'] for sample in samples), default=None),
                missing_imu_source_stamps_ns=missing,
                receipt_wait_budget_s=budget, receipt_wait_elapsed_s=monotonic()-started)
        spin_once(observer, timeout_sec=min(.02, max(0., deadline-monotonic())))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=90.)
    parser.add_argument('--goal', type=float, nargs=3)
    parser.add_argument('--case', choices=('goal', 'cancel', 'preview_cancel'), default='goal')
    parser.add_argument('--cancel-after', type=float, default=.25, help='measured travel in metres before cancelling')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    from d1max_pct_scan.isolated_zenoh import validate_environment
    validate_environment()
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from rclpy.action import ActionClient
    from d1max_planning_interfaces.msg import (LocalNavigationState, MotionDemand,
        ExecutionPermit, StopReport, ExecutionCommitAck, MotionValidation,
        TrajectoryValidation, RouteProgress)
    from d1max_navigation_bt_interfaces.action import Navigate
    from d1max_navigation_bt_interfaces.srv import ConfirmExecution
    from std_msgs.msg import String
    from sensor_msgs.msg import Imu
    session_dir = args.session.resolve(strict=True)
    session = json.loads((session_dir/'session.json').read_text())
    if session.get('simulation_backend') != 'isaacsim_physx' or session.get('transport_mode') != 'isolated_mock':
        raise ValueError('isaac_isolated_session_required')
    begin = time.monotonic()
    initial = session['simulation_initial_pose']
    goal = args.goal or [initial[0]+1.2, initial[1], session['simulation_goal'][2]]

    class Observer(Node):
        def __init__(self):
            super().__init__('d1max_isaac_smoke_observer', parameter_overrides=[Parameter('use_sim_time', value=True)])
            self.counts = Counter()
            self.status = {}
            self.component_status = {}
            self.bridge = {}
            self.geometry_ready = False
            self.stop = None
            self.motion_execution_id = ''
            self.motion_nonzero = 0
            self.max_measured_speed = 0.
            self.position = None
            self.start_position = None
            self.distance = 0.
            self.samples = deque(maxlen=18000)
            self.events = deque(maxlen=2000)
            self.latest_applied = None
            self.first_imu_ns = None
            self.actual_imu_times = set()
            self.imu_samples = deque(maxlen=18000)
            self.action = ActionClient(self, Navigate, '/d1max/live_planning/bt/navigate')
            self.confirm = self.create_client(ConfirmExecution, '/d1max/live_planning/bt/confirm_execution')
            self.goal_future = None
            self.goal_handle = None
            self.result_future = None
            self.confirm_future = None
            self.cancel_future = None
            self.result = None
            self.error = ''
            self.confirmed = False
            self.next_confirmation_wall = 0.
            self.cancellation_requested = False
            self.preview_ready_wall = None
            self.preview_proof = None
            self.action_result_status = None
            # Read-only causal trace: preserve exact received wire times and
            # identities. This cannot publish or refresh a control/proof lease.
            self.control_trace_path = session_dir/'control_trace.jsonl'
            self.control_trace = self.control_trace_path.open('x')
            self.last_component_trace = {}
            self.create_subscription(LocalNavigationState, '/d1max/localization/navigation/local_state', self.state, 10)
            imu_qos=QoSProfile(depth=512, reliability=qos_profile_sensor_data.reliability,
                durability=qos_profile_sensor_data.durability)
            self.create_subscription(Imu, '/d1max/localization/imu', self.imu, imu_qos)
            prefix = '/d1max/live_planning/'
            self.create_subscription(MotionDemand, prefix+'execution/applied_motion', self.applied, 10)
            self.create_subscription(ExecutionPermit, prefix+'execution/permit', self.permit, 10)
            self.create_subscription(StopReport, prefix+'execution/stop_report', self.stop_report, 10)
            self.create_subscription(ExecutionCommitAck, prefix+'execution/commit_ack', self.commit, 10)
            for name, kind, topic in (
                    ('demand', MotionDemand, 'execution/demand'),
                    ('safe_demand', MotionDemand, 'execution/safe_demand'),
                    ('motion_validation', MotionValidation, 'execution/motion_validation'),
                    ('trajectory_validation', TrajectoryValidation, 'execution/validation')):
                self.create_subscription(kind, prefix+topic,
                    lambda m,n=name:self.trace_message(n,m), 64)
            self.create_subscription(RouteProgress, prefix+'execution/route_progress',
                self.route_progress, 64)
            self.create_subscription(String, prefix+'bt/status', self.bt_status,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.create_subscription(String, '/d1max/isaacsim/status', self.bridge_status, 10)
            for name, topic in [('global', 'global_status'), ('scan', 'native_local_attempt_debug'),
                                ('projector', 'ray_projector_status'), ('rays', 'rays_status'),
                                ('safety', 'execution/safety_status'), ('tracker', 'tracker_status')]:
                self.create_subscription(String, prefix+topic, lambda m,n=name:self.component(n,m), 10)

        def event(self, key, **data):
            self.events.append(dict(elapsed_s=time.monotonic()-begin, key=key, **data))

        def trace(self, kind, data):
            self.control_trace.write(json.dumps(dict(kind=kind,
                receipt_monotonic_ns=time.monotonic_ns(),
                source_clock_ns=self.get_clock().now().nanoseconds,
                data=data), separators=(',', ':'))+'\n')
            self.control_trace.flush()

        def trace_message(self, kind, message):
            data = {}
            for field in ('execution_id', 'trajectory_id', 'control_epoch',
                    'sequence', 'permit_sequence', 'validation_sequence',
                    'trajectory_validation_sequence', 'motion_validation_sequence',
                    'demand_sequence', 'hold', 'valid', 'allowed', 'revoked', 'phase', 'reason',
                    'geometry_committed', 'handoff_id', 'applied', 'accepted'):
                if hasattr(message, field):
                    data[field] = getattr(message, field)
            for field in ('source_stamp', 'body_source_stamp', 'valid_until',
                    'demand_source_stamp', 'demand_valid_until', 'demand_body_source_stamp',
                    'front_ray_source_stamp', 'rear_ray_source_stamp',
                    'check_begin', 'check_end', 'safety_source_stamp', 'applied_at'):
                if hasattr(message, field):
                    stamp = getattr(message, field)
                    data[field+'_ns'] = stamp.sec*10**9+stamp.nanosec
            if hasattr(message, 'velocity'):
                data['velocity'] = [message.velocity.linear.x, message.velocity.angular.z]
            self.trace(kind, data)

        def route_progress(self, message):
            p, q = message.odom_body_pose.pose.position, message.odom_body_pose.pose.orientation
            a, b = message.map_from_odom.position, message.map_from_odom.orientation
            version = message.version
            self.trace('route_progress', dict(
                sequence=message.sequence, frame_id=message.frame_id,
                body_source_stamp_ns=message.body_source_stamp.sec*10**9+message.body_source_stamp.nanosec,
                anchor_source_stamp_ns=message.anchor_source_stamp.sec*10**9+message.anchor_source_stamp.nanosec,
                reference_generation=version.reference_generation, segment_id=version.segment_id,
                anchor_id=version.anchor_id, anchor_revision=version.anchor_revision,
                context_sequence=version.context_sequence, map_geometry_revision=version.map_geometry_revision,
                position=[p.x,p.y,p.z], orientation_xyzw=[q.x,q.y,q.z,q.w],
                map_from_odom_position=[a.x,a.y,a.z], map_from_odom_orientation_xyzw=[b.x,b.y,b.z,b.w],
                edge_index=message.edge_index, measured_arc_m=message.measured_arc_m,
                confirmed_arc_m=message.confirmed_arc_m, cross_track_m=message.cross_track_m,
                body_reference_height_m=message.body_reference_height_m))

        def state(self, message):
            if message.session_id != session['id'] or message.map_version_id != session['version_id']:
                self.error = 'foreign_measured_state'
                return
            self.counts['measured_state'] += 1
            if not message.usable:
                self.error = message.reason
                return
            p = message.local_odometry.pose.pose.position
            q = message.local_odometry.pose.pose.orientation
            position = [p.x, p.y, p.z]
            if self.start_position is None:
                self.start_position = position
            if self.position is not None:
                self.distance += math.hypot(p.x-self.position[0], p.y-self.position[1])
            self.position = position
            v, w = message.local_odometry.twist.twist.linear, message.local_odometry.twist.twist.angular
            linear = math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)
            angular = math.sqrt(w.x*w.x+w.y*w.y+w.z*w.z)
            self.max_measured_speed = max(self.max_measured_speed, linear)
            self.samples.append(dict(elapsed_s=time.monotonic()-begin,
                source_stamp_ns=message.source_stamp.sec*10**9+message.source_stamp.nanosec,
                imu_stamp_ns=message.imu_stamp.sec*10**9+message.imu_stamp.nanosec,
                position=position, orientation_xyzw=[q.x,q.y,q.z,q.w],
                linear_mps=linear, angular_radps=angular))

        def imu(self, message):
            self.counts['native_imu_received'] += 1
            ns = message.header.stamp.sec*10**9+message.header.stamp.nanosec
            if self.first_imu_ns is None:
                self.first_imu_ns = ns
            self.actual_imu_times.add(ns)
            acceleration, angular = message.linear_acceleration, message.angular_velocity
            q = message.orientation
            self.imu_samples.append(dict(source_stamp_ns=ns, frame=message.header.frame_id,
                orientation_xyzw=[q.x,q.y,q.z,q.w],
                acceleration_mps2=[acceleration.x,acceleration.y,acceleration.z],
                angular_velocity_radps=[angular.x,angular.y,angular.z]))

        def applied(self, message):
            self.trace_message('applied', message)
            self.counts['applied_motion'] += 1
            self.latest_applied = dict(vx=message.velocity.linear.x, wz=message.velocity.angular.z,
                hold=message.hold, reason=message.reason, execution_id=message.execution_id)
            if abs(message.velocity.linear.x) > 1e-6 or abs(message.velocity.angular.z) > 1e-6:
                self.motion_nonzero += 1
                self.motion_execution_id = message.execution_id

        def permit(self, message):
            self.trace_message('permit', message)
            self.counts['permit'] += 1
            self.geometry_ready = message.geometry_committed and not message.revoked

        def stop_report(self, message):
            self.counts['stop_report'] += 1
            self.stop = dict(execution_id=message.execution_id, stop_submitted=message.stop_submitted,
                measured_stop_confirmed=message.measured_stop_confirmed,
                physical_acceptance_verified=message.physical_acceptance_verified,
                stationary_samples=message.stationary_samples,
                measured_linear_mps=message.measured_linear_mps,
                measured_angular_radps=message.measured_angular_radps,
                reason=message.reason)

        def commit(self, message):
            self.trace_message('commit_ack', message)
            self.counts['commit_ack'] += 1
            self.event('writer_commit', applied=message.applied, reason=message.reason,
                execution_id=message.execution_id)

        def bt_status(self, message):
            self.status = json.loads(message.data)

        def bridge_status(self, message):
            self.bridge = json.loads(message.data)
            if self.bridge.get('fault'):
                self.error = self.bridge['fault']

        def component(self, name, message):
            try:
                self.component_status[name] = json.loads(message.data)
                if name in ('tracker', 'safety'):
                    fields = ('reason', 'permit_reject_reason', 'prepare_reason',
                        'handoff_reason', 'maneuver_phase', 'trajectory_id',
                        'installation_sequence', 'writer_commit_sequence',
                        'finished', 'preparation_pending', 'execution_frozen', 'active')
                    data = {key:self.component_status[name].get(key) for key in fields
                        if key in self.component_status[name]}
                    if data != self.last_component_trace.get(name):
                        self.trace(name, data)
                        self.last_component_trace[name] = data
            except json.JSONDecodeError:
                self.component_status[name] = message.data

        def tick(self):
            if self.error or self.result is not None:
                return
            if self.goal_future is None:
                if (self.counts['measured_state'] < 20 or min(self.bridge.get('ray_scans', [0,0])) < 2
                        or not self.action.server_is_ready()
                        or self.status.get('session_id') != session['id']
                        or self.status.get('lifecycle_active') is not True
                        or self.status.get('initial_pose_transaction_blocked') is not False
                        or self.status.get('dependency_health_reason') != ''):
                    return
                command = navigation_goal(session, goal, self.get_clock().now().to_msg())
                self.goal_future = self.action.send_goal_async(command)
                self.event('goal_sent', xyz=goal)
            if self.goal_future is not None and self.goal_future.done() and self.goal_handle is None:
                self.goal_handle = self.goal_future.result()
                if not self.goal_handle.accepted:
                    self.error = 'BT_goal_rejected'
                    return
                self.result_future = self.goal_handle.get_result_async()
                self.event('goal_accepted')
            if self.confirm_future is not None and self.confirm_future.done():
                response = self.confirm_future.result()
                if response.accepted:
                    if not self.confirmed:
                        self.event('route_confirmed', reason=response.reason, execution_authorized=response.execution_authorized)
                    self.confirmed = True
                elif response.reason in ('waiting_matching_native_and_tracker_preparation', 'waiting_single_sdk_writer'):
                    self.event('confirmation_wait', reason=response.reason)
                    self.confirm_future = None
                    self.next_confirmation_wall = time.monotonic()+.5
                else:
                    self.error = 'BT_confirmation_rejected:'+response.reason
                    return
            if (args.case != 'preview_cancel' and not self.confirmed
                    and self.confirm_future is None and self.geometry_ready
                    and time.monotonic() >= self.next_confirmation_wall
                    and all(self.status.get(k) for k in ('task_id', 'route_id', 'route_hash'))
                    and self.confirm.service_is_ready()):
                request = ConfirmExecution.Request(schema_version=2, session_id=session['id'],
                    task_id=self.status['task_id'], route_id=self.status['route_id'], route_hash=self.status['route_hash'],
                    request_id='isaac-smoke-'+session['id'])
                self.confirm_future = self.confirm.call_async(request)
                self.event('route_confirmation_requested', route_hash=request.route_hash)
            if (args.case == 'cancel' and self.confirmed and not self.cancellation_requested
                    and self.distance >= args.cancel_after and self.goal_handle is not None):
                self.cancel_future = self.goal_handle.cancel_goal_async()
                self.cancellation_requested = True
                self.event('cancel_requested', measured_distance_m=self.distance)
            if args.case == 'preview_cancel' and self.goal_handle is not None:
                global_status = self.component_status.get('global', {})
                if (self.preview_ready_wall is None
                        and all(self.status.get(k) for k in ('task_id', 'route_id', 'route_hash'))
                        and global_status.get('route_committed') is True
                        and (global_status.get('active_goal') or {}).get('xyz') == goal):
                    self.preview_ready_wall = time.monotonic()
                    self.preview_proof = {k: self.status[k] for k in ('task_id', 'route_id', 'route_hash')}
                    self.event('route_preview_ready', **self.preview_proof)
                if (self.preview_ready_wall is not None and not self.cancellation_requested
                        and time.monotonic()-self.preview_ready_wall >= .75):
                    self.cancel_future = self.goal_handle.cancel_goal_async()
                    self.cancellation_requested = True
                    self.event('preview_cancel_requested', measured_distance_m=self.distance)
            if self.result_future is not None and self.result_future.done():
                response = self.result_future.result()
                self.action_result_status = response.status
                result = response.result
                self.result = dict(success=result.success, reason=result.reason,
                    retirement_confirmed=result.retirement_confirmed,
                    physical_stop_confirmed=result.physical_stop_confirmed)
                self.event('BT_result', **self.result)

    rclpy.init()
    observer = Observer()
    end_state_at = None
    try:
        while time.monotonic()-begin < args.duration:
            rclpy.spin_once(observer, timeout_sec=.02)
            observer.tick()
            if observer.result is not None or observer.error:
                if end_state_at is None:
                    end_state_at = time.monotonic()
                if time.monotonic()-end_state_at > 1.5:
                    break
        if observer.result is None and not observer.error:
            observer.error = 'smoke_wall_timeout'
        if observer.goal_handle is not None and observer.result is None:
            # Bounded cancellation through the same Action before supervisor
            # cleanup. A failed smoke never leaves an intentional active task.
            future = observer.goal_handle.cancel_goal_async()
            deadline = time.monotonic()+5.
            while time.monotonic() < deadline and not future.done():
                rclpy.spin_once(observer, timeout_sec=.05)
        evidence_samples = list(observer.samples)
        evidence_fence_elapsed = time.monotonic()-begin
        imu_evidence = drain_imu_witness(observer, rclpy.spin_once)
        imu_evidence['fence_elapsed_s'] = evidence_fence_elapsed
        result, stop = observer.result or {}, observer.stop or {}
        stopped_samples = [sample for sample in evidence_samples if
            evidence_fence_elapsed-sample['elapsed_s'] <= 1.]
        measured_stationary = (len(stopped_samples) >= 3 and
            all(sample['linear_mps'] <= .03 and sample['angular_radps'] <= .05 for sample in stopped_samples))
        writer_stop = (stop.get('measured_stop_confirmed') is True
            and stop.get('physical_acceptance_verified') is False
            and stop.get('execution_id') == observer.motion_execution_id
            and bool(observer.motion_execution_id))
        imu_source_matching = imu_evidence['matched']
        heights = [sample['position'][2] for sample in observer.samples]
        expected_height = float(session['body_height'])+float(session['simulation_goal'][2])
        body_height_consistent = bool(heights) and max(abs(height-expected_height) for height in heights) <= .03
        expected_result = expected_task_result(args.case, result, observer.cancellation_requested,
            observer.action_result_status)
        execution_evidence = (writer_stop and observer.motion_nonzero > 0 and observer.distance > .05)
        if args.case == 'preview_cancel':
            execution_evidence = (observer.preview_proof is not None and not observer.confirmed
                and observer.motion_nonzero == 0 and observer.max_measured_speed <= .03)
        passed = (not observer.error and expected_result and result.get('retirement_confirmed') is True
            and result.get('physical_stop_confirmed') is False and execution_evidence and measured_stationary
            and imu_source_matching
            and body_height_consistent)
        report = dict(schema=1, passed=bool(passed), case=args.case, session_id=session['id'],
            test_scope=('original_BT_PCT_route_preview_cancellation_PhysX_sensor_fixture'
                if args.case == 'preview_cancel' else
                'original_BT_PCT_SCAN_tracker_safety_mock_writer_PhysX_wheel_fixture'),
            physical_acceptance=False, localization='groundtruth_fixture_not_LIO_localization_validation',
            goal=goal, result=observer.result, error=observer.error, counts=dict(observer.counts),
            action_result_status=observer.action_result_status, preview_proof=observer.preview_proof,
            execution_motion_test=args.case != 'preview_cancel',
            bridge=observer.bridge, stop=observer.stop, measured_stationary=measured_stationary,
            measured_distance_m=observer.distance, measured_final_position=observer.position,
            measured_max_speed_mps=observer.max_measured_speed, nonzero_applied_count=observer.motion_nonzero,
            latest_applied=observer.latest_applied, bt_status=observer.status,
            component_status=observer.component_status,
            control_trace=str(observer.control_trace_path),
            imu_source_matching=imu_source_matching,
            imu_evidence=imu_evidence,
            stationarity_evidence=dict(fence_elapsed_s=evidence_fence_elapsed,
                window_s=1., sample_count=len(stopped_samples)),
            body_height_consistent=body_height_consistent,
            expected_body_reference_z=expected_height,
            measured_body_z_range=[min(heights),max(heights)] if heights else None,
            native_imu_samples=list(observer.imu_samples),
            events=list(observer.events), measured_samples=list(observer.samples))
        output = args.output or session_dir/'isaac_smoke_report.json'
        output.write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps(dict(report=str(output), passed=report['passed'], error=observer.error,
            result=observer.result, measured_distance_m=observer.distance, physical_acceptance=False)))
        return 0 if passed else 1
    finally:
        observer.control_trace.close()
        observer.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
