#!/usr/bin/env python3
"""Real Navigator callbacks; fixture planners/SDK facts, loopback-only Zenoh.

This is a protocol regression, not physical stopping acceptance. No vendor SDK
is loaded and no robot command is sent. Its old execution deliberately owes a
stop: new preparation must proceed after software retirement, without a grant.
"""
import argparse
import copy
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import threading
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--binary', required=True)
    parser.add_argument('--router', required=True)
    parser.add_argument('--tree', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--expect-old-block', action='store_true')
    args = parser.parse_args()
    out = Path(args.output).resolve()
    out.mkdir(parents=True, exist_ok=False)
    with socket.socket() as endpoint:
        endpoint.bind(('127.0.0.1', 0))
        port = endpoint.getsockname()[1]
    base = {'scouting': {'multicast': {'enabled': False}, 'gossip': {'enabled': False}},
            'timestamping': {'enabled': True, 'drop_future_timestamp': False}}
    for role in ('router', 'client'):
        config = dict(base, mode=role, listen={'endpoints': [f'tcp/127.0.0.1:{port}'] if role == 'router' else []},
                      connect={'endpoints': [] if role == 'router' else [f'tcp/127.0.0.1:{port}']})
        (out / f'{role}.json5').write_text(json.dumps(config))
    os.environ.update(RMW_IMPLEMENTATION='rmw_zenoh_cpp', ROS_DOMAIN_ID='229',
                      ZENOH_ROUTER_CONFIG_URI=str(out / 'router.json5'),
                      ZENOH_SESSION_CONFIG_URI=str(out / 'client.json5'),
                      D1MAX_OFFLINE_ZENOH_TEST='1', D1MAX_NAV_ISOLATED='1',
                      D1MAX_NAV_TRANSPORT='isolated_mock', D1MAX_NAV_ISOLATION_TOKEN='b' * 32)
    os.environ.pop('ZENOH_CONFIG_OVERRIDE', None)
    import rclpy
    from rclpy.action import ActionClient, ActionServer, CancelResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from lifecycle_msgs.srv import ChangeState
    from lifecycle_msgs.msg import Transition
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String
    from d1max_navigation_bt_interfaces.action import Navigate, ComputeRoute, FollowRoute
    from d1max_navigation_bt_interfaces.msg import NavigationHealth, RouteSegment
    from d1max_planning_interfaces.msg import (LocalNavigationState, SDKExecutionState,
        TaskExecutionLease, LocalExecutionStatus, SDKExecutionRetirement, StopReport)
    from d1max_planning_interfaces.srv import ExecutionGrant

    prefix, session, sdk_session = '/d1max/live_planning/', 'replacement_ros_probe', 'sdk_replacement_probe'
    processes, logs, checks, statuses, leases, computed, followed, retired = [], [], [], [], [], [], [], []
    snapshots, grants, latest_leases, stop_boundaries = {}, {}, {}, {}
    release_workers, shutdown = threading.Event(), threading.Event()
    node = executor = None
    first_task = ''
    local_sequence = sdk_sequence = retirement_sequence = 0

    def wait_for(predicate, reason, timeout=8):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            if any(p.poll() is not None for p in processes):
                raise AssertionError('process exited: ' + reason)
            time.sleep(.01)
        raise AssertionError(reason + '; last=' + str(statuses[-1:]))

    def start(command, filename):
        handle = (out / filename).open('w')
        logs.append(handle)
        process = subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT)
        processes.append(process)

    try:
        start([args.router], 'router.log')
        time.sleep(.4)
        rclpy.init()
        node = rclpy.create_node('replacement_protocol_fixture')
        executor = MultiThreadedExecutor(num_threads=6)
        executor.add_node(node)
        threading.Thread(target=executor.spin, daemon=True).start()
        durable = QoSProfile(depth=8, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        health_pub = node.create_publisher(NavigationHealth, prefix + 'bt/health', 5)
        local_pub = node.create_publisher(LocalNavigationState, '/d1max/localization/navigation/local_state', 5)
        local_status_pub = node.create_publisher(LocalExecutionStatus, prefix + 'execution/local_status', 10)
        sdk_pub = node.create_publisher(SDKExecutionState, prefix + 'execution/sdk_state', 10)
        retirement_pub = node.create_publisher(SDKExecutionRetirement, prefix + 'execution/retirement', 10)
        stop_pub = node.create_publisher(StopReport, prefix + 'execution/stop_report', 10)
        node.create_subscription(String, prefix + 'bt/status', lambda m: statuses.append(json.loads(m.data)), durable)

        def stamp(ns=None):
            return rclpy.time.Time(nanoseconds=ns or time.time_ns()).to_msg()

        def on_lease(value):
            leases.append(value)
            latest_leases[value.version.task_id] = copy.deepcopy(value)
            if value.revoked:
                stop_boundaries.setdefault(value.version.task_id, time.time_ns())
        node.create_subscription(TaskExecutionLease, prefix + 'execution/task_lease', on_lease, 10)

        def grant(request, response):
            response.version, response.execution_id = request.version, request.execution_id
            response.control_epoch, response.request_id = request.control_epoch, request.request_id
            response.sdk_session, response.transport_mode = request.sdk_session, request.transport_mode
            response.accepted, response.sdk_arm_generation = True, 1
            grants.setdefault(request.version.task_id, []).append(copy.deepcopy(request))
            return response
        grant_service = node.create_service(ExecutionGrant, prefix + 'execution/grant', grant)

        def compute(handle):
            request = handle.request
            computed.append(request.task_id)
            result = ComputeRoute.Result()
            result.schema_version = 2
            result.success = result.retirement_confirmed = True
            result.reason, result.route_id = 'fixture_route', request.task_id + ':route'
            result.localization_epoch, result.localization_seed_id = 1, 'seed'
            result.route.header.frame_id = 'd1max_loc_map'
            first = PoseStamped()
            first.header.frame_id, first.pose.orientation.w = 'd1max_loc_map', 1.
            result.route.poses = [first, copy.deepcopy(request.goal)]
            s = result.snapshot
            s.schema_version, s.session_id, s.task_id, s.route_id = 2, session, request.task_id, result.route_id
            s.route_hash = s.source_map_sha256 = s.tomogram_sha256 = s.conditioning_sha256 = 'a' * 64
            s.map_version_id, s.frame_id, s.point_reference, s.direction = 'map', 'd1max_loc_map', 'ground', 'forward'
            s.localization_epoch, s.localization_seed_id, s.path = 1, 'seed', copy.deepcopy(result.route)
            s.source_layer_ids = s.layer_ids = [0, 0]
            s.point_floor_ids, s.edge_segments = ['floor', 'floor'], ['corridor']
            seg = RouteSegment()
            seg.segment_id, seg.kind, seg.floor_id = 'corridor', 'floor', 'floor'
            seg.end_index, seg.required_mode, seg.direction, seg.execution_eligible = 1, 'general', 'forward', True
            s.segments, s.geometry_evidence_json = [seg], '{}'
            s.preview_ready = s.execution_eligible = True
            snapshots[request.task_id] = copy.deepcopy(s)
            handle.succeed()
            return result

        def follow(handle):
            task = handle.request.task_id
            assert not handle.request.execution_confirmed
            followed.append(task)
            while not shutdown.is_set():
                if handle.is_cancel_requested and (task != first_task or release_workers.is_set()):
                    break
                feedback = FollowRoute.Feedback()
                feedback.phase, feedback.reason = 'trajectory_valid', 'fixture_preview'
                handle.publish_feedback(feedback)
                time.sleep(.03)
            result = FollowRoute.Result()
            result.schema_version, result.retirement_confirmed, result.reason = 2, True, 'fixture_retired'
            retired.append(task)
            if handle.is_cancel_requested:
                handle.canceled()
            else:
                handle.abort()
            return result

        group = ReentrantCallbackGroup()
        servers = [ActionServer(node, ComputeRoute, prefix + 'bt/compute_route', compute, callback_group=group),
                   ActionServer(node, FollowRoute, prefix + 'bt/follow_route', follow, callback_group=group,
                                cancel_callback=lambda h: CancelResponse.ACCEPT)]

        def evidence():
            nonlocal local_sequence, sdk_sequence, retirement_sequence
            while not shutdown.is_set():
                now = time.time_ns()
                health = NavigationHealth()
                health.schema_version, health.stamp, health.evidence_source_stamp = 2, stamp(now), stamp(now)
                health.session_id, health.map_version_id = session, 'map'
                health.ready = health.local_control_ready = health.global_planning_ready = True
                health.localization_epoch, health.localization_seed_id = 1, 'seed'
                health_pub.publish(health)
                body = LocalNavigationState()
                body.schema_version, body.session_id, body.map_version_id = 2, session, 'map'
                body.localization_epoch, body.localization_seed_id = 1, 'seed'
                body.source_stamp = body.posterior_stamp = body.imu_stamp = stamp(now)
                body.valid_until = stamp(now + 100000000)
                body.local_odometry.header.stamp = stamp(now)
                body.local_odometry.header.frame_id = 'd1max_loc_odom'
                body.local_odometry.child_frame_id = 'd1max_loc_base_link'
                body.local_odometry.pose.pose.orientation.w, body.usable = 1., True
                local_pub.publish(body)
                for task, s in list(snapshots.items()):
                    if task not in followed:
                        continue
                    local_sequence += 1
                    status = LocalExecutionStatus()
                    status.schema_version, status.sequence, status.version.schema_version = 1, local_sequence, 3
                    for key in ('session_id', 'task_id', 'route_id', 'route_hash', 'map_version_id', 'localization_epoch', 'localization_seed_id'):
                        setattr(status.version, key, getattr(s, key))
                    lease = latest_leases.get(task)
                    if lease:
                        status.execution_id, status.control_epoch = lease.execution_id, lease.control_epoch
                    status.source_stamp = status.body_source_stamp = stamp(now)
                    status.body_valid_until = stamp(now + 100000000)
                    status.trajectory_id, status.commit_sequence = 1, 1
                    status.software_retired, status.ready = task in retired, task not in retired
                    status.phase, status.transport_mode = ('retired' if task in retired else 'preview'), 'isolated_mock'
                    local_status_pub.publish(status)
                    if task in grants:
                        request = grants[task][-1]
                        sdk_sequence += 1
                        sdk = SDKExecutionState()
                        sdk.version, sdk.execution_id, sdk.control_epoch = request.version, request.execution_id, request.control_epoch
                        sdk.sdk_session, sdk.transport_mode, sdk.sdk_arm_generation = sdk_session, 'isolated_mock', 1
                        sdk.sequence, sdk.source_stamp = sdk_sequence, stamp(now)
                        sdk.control_owned = sdk.general_low_speed_confirmed = True
                        sdk.grant_ready, sdk.phase = task not in stop_boundaries, ('ready' if task not in stop_boundaries else 'stopping')
                        sdk_pub.publish(sdk)
                        if task in stop_boundaries and release_workers.is_set():
                            retirement_sequence += 1
                            report = SDKExecutionRetirement()
                            report.schema_version, report.version, report.execution_id = 1, request.version, request.execution_id
                            report.control_epoch, report.sdk_session = request.control_epoch, sdk_session
                            report.sdk_arm_generation, report.sequence = 1, retirement_sequence
                            report.grant_request_id, report.source_stamp = request.confirmation_id, stamp(now)
                            report.transport_mode = 'isolated_mock'
                            report.nonzero_blocked = report.software_retired = True
                            report.motion_never_submitted = False
                            retirement_pub.publish(report)
                time.sleep(.02)

        threading.Thread(target=evidence, daemon=True).start()
        start([args.binary, '--ros-args', '-p', f'session_id:={session}', '-p', 'preview_only:=false',
               '-p', 'execution_transport_mode:=isolated_mock', '-p', f'expected_sdk_session:={sdk_session}',
               # Accelerate only this isolated diagnostic timeout; production
               # stop limits and all source/identity predicates are unchanged.
               '-p', 'execution_blocked_timeout_s:=2.0',
               '-p', f'tree_xml:={args.tree}'], 'navigator.log')
        lifecycle = node.create_client(ChangeState, '/d1max_navigation_bt/change_state')
        assert lifecycle.wait_for_service(timeout_sec=8)
        for transition in (Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
            request = ChangeState.Request()
            request.transition.id = transition
            future = lifecycle.call_async(request)
            wait_for(future.done, 'lifecycle transition')
            assert future.result().success
        client = ActionClient(node, Navigate, prefix + 'bt/navigate')
        assert client.wait_for_server(timeout_sec=5)

        def goal(x):
            old = statuses[-1].get('task_id', '') if statuses else ''
            request = Navigate.Goal()
            request.schema_version, request.goal_kind = 2, '3d'
            request.goal.header.frame_id, request.goal.header.stamp = 'd1max_loc_map', stamp()
            request.goal.pose.position.x, request.goal.pose.orientation.w = float(x), 1.
            request.goal_yaw_tolerance_rad, request.request_execution = .15, True
            future = client.send_goal_async(request)
            wait_for(future.done, 'goal accepted')
            handle = future.result()
            assert handle.accepted
            wait_for(lambda: statuses and statuses[-1].get('task_id') not in ('', old), 'new current task')
            return handle, statuses[-1]['task_id']

        handle_a, first_task = goal(5)
        wait_for(lambda: first_task in grants and any(l.version.task_id == first_task and l.allowed for l in leases), 'first task authority')
        _, discarded = goal(6)
        _, newest = goal(7)
        time.sleep(.4)
        assert computed == [first_task] and followed == [first_task]
        assert first_task in stop_boundaries and first_task not in retired
        checks.append('unretired_workers_prevent_new_compute_and_follow')
        release_workers.set()
        wait_for(lambda: first_task in retired, 'first Follow retired')
        if args.expect_old_block:
            time.sleep(1.)
            assert computed == followed == [first_task]
            assert set(grants) == {first_task}
            checks.append('baseline_reproduces_software_retired_but_new_planning_blocked_by_stop')
        else:
            wait_for(lambda: newest in followed, 'newest prepared without old physical stop')
            assert discarded not in computed
            assert set(grants) == {first_task}
            assert not any(l.version.task_id == newest and l.allowed for l in leases)
            checks.append('newest_only_compute_and_follow_preview_before_old_stop_no_new_grant')
            wait_for(lambda: time.time_ns() - stop_boundaries[first_task] > 2300000000,
                     'old physical-stop diagnostic budget elapsed')
            time.sleep(.15)
            assert statuses[-1]['active'] and newest not in retired and set(grants) == {first_task}
            checks.append('expired_old_stop_diagnostic_does_not_fail_current_preparation')
            _, final = goal(8)
            wait_for(lambda: final in followed and newest in retired, 'preview replacement retires before newest Follow')
            assert set(grants) == {first_task}
            checks.append('preview_can_be_replaced_while_old_motion_stop_pending')
            # A delayed targeted Action cancellation for A must not affect D.
            future = handle_a.cancel_goal_async()
            wait_for(future.done, 'late old cancel reply')
            time.sleep(.1)
            assert final not in retired and statuses[-1]['task_id'] == final
            checks.append('late_old_cancel_and_statuses_do_not_cancel_newest')
            request = grants[first_task][0]
            def report(request, sequence):
                value = StopReport()
                value.version, value.execution_id, value.control_epoch = request.version, request.execution_id, request.control_epoch
                value.sdk_session, value.sdk_arm_generation, value.stop_request_id = sdk_session, 1, request.confirmation_id
                value.sequence, value.source_stamp = sequence, stamp()
                value.nonzero_blocked = value.stop_submitted = value.measured_stop_confirmed = True
                value.mc_clock_epoch, value.mc_raw_stamp_ns = 'isolated_clock', time.time_ns()
                value.stationary_samples, value.stationary_duration_sec = 100, 1.
                value.measured_linear_mps = value.measured_angular_radps = .001
                value.velocity_metric, value.time_basis, value.transport_mode = 'spatial_norm_3d', 'isolated_simulated_source_clock', 'isolated_mock'
                return value
            wait_for(lambda: time.time_ns() - stop_boundaries[first_task] > 1200000000, 'post-stop fixture interval')
            invalid = report(request, 1)
            invalid.stop_request_id = 'foreign-stop'
            stop_pub.publish(invalid)
            invalid = report(request, 2)
            invalid.source_stamp = stamp(stop_boundaries[first_task] + 100000000)
            stop_pub.publish(invalid)
            time.sleep(.3)
            assert set(grants) == {first_task}
            checks.append('foreign_and_pre_stop_stationary_intervals_do_not_authorize')
            valid = report(request, 3)
            stop_pub.publish(valid)
            wait_for(lambda: final in grants and any(l.version.task_id == final and l.allowed for l in leases), 'exact old stop releases newest grant')
            assert len(grants[final]) == 1 and newest not in grants and discarded not in grants
            checks.append('matching_old_stop_releases_only_newest_execution_once')
            stop_pub.publish(valid)
            time.sleep(.3)
            assert final not in retired and statuses[-1]['task_id'] == final and len(grants[final]) == 1
            checks.append('old_terminal_replay_does_not_complete_or_regrant_new_task')
            # Repeated real action transitions exercise task-map reclamation
            # and old SDK debt across more than a one-off A/B swap.
            for cycle in range(6):
                old = final
                _, skipped = goal(10 + 2 * cycle)
                _, final = goal(11 + 2 * cycle)
                wait_for(lambda: old in retired and final in followed, 'repeated replacement preview')
                assert final not in grants and skipped not in grants
                assert statuses[-1]['task_id'] == final
                wait_for(lambda: old in stop_boundaries and time.time_ns() - stop_boundaries[old] > 1150000000,
                         'repeated old task post-stop interval')
                completed = report(grants[old][0], 3)
                stop_pub.publish(completed)
                wait_for(lambda: final in grants and any(l.version.task_id == final and l.allowed for l in leases),
                         'repeated newest execution authority')
                stop_pub.publish(completed)
                time.sleep(.1)
                assert len(grants[final]) == 1 and skipped not in grants and final not in retired
            checks.append('six_further_motion_cancel_newest_goal_cycles_no_regrant_or_retirement_leak')
        result = dict(status='PASS', baseline=args.expect_old_block, checks=checks, computed=computed,
                      followed=followed, retired=retired, grants={k: len(v) for k, v in grants.items()},
                      scope='real Navigator ROS/lifecycle/action callbacks; fixture planners and isolated SDK facts; no robot')
        (out / 'result.json').write_text(json.dumps(result, indent=2))
        print(json.dumps(result))
    except BaseException as error:
        (out / 'failure.json').write_text(json.dumps(dict(error=str(error), checks=checks, computed=computed,
            followed=followed, retired=retired, grants={k: len(v) for k, v in grants.items()}, statuses=statuses[-10:]), indent=2))
        raise
    finally:
        shutdown.set()
        for process in reversed(processes):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
        if executor:
            executor.shutdown(timeout_sec=2)
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for handle in logs:
            handle.close()


if __name__ == '__main__':
    main()
