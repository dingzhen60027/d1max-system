#!/usr/bin/env python3
"""Real Monitor/ROS task writer + fake SDK, no hardware or production router.

The test supplies typed BT authority and controller samples, not a planner or
physical acceptance. It checks the production IPC→writer→Grant handoff.
"""
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--binary', required=True)
    parser.add_argument('--router', required=True)
    parser.add_argument('--profile', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--move-delay-ms', type=int, default=5)
    parser.add_argument('--expect-zero-failure', action='store_true')
    parser.add_argument('--terminal-stop-scenario', choices=('none', 'next_task', 'manual', 'never_stopped',
        'stair_next', 'stair_manual', 'stair_vertical', 'stair_planar', 'stair_angular'), default='none')
    args = parser.parse_args()
    binary_hash = hashlib.sha256(Path(args.binary).read_bytes()).hexdigest()
    output = Path(args.output); output.mkdir(parents=True, exist_ok=False)
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 0)); port = probe.getsockname()[1]
    base = {'scouting': {'multicast': {'enabled': False}, 'gossip': {'enabled': False}},
            'timestamping': {'enabled': True, 'drop_future_timestamp': False}}
    for name, value in [('router', dict(base, mode='router', connect={'endpoints': []}, listen={'endpoints': [f'tcp/127.0.0.1:{port}']})),
                        ('client', dict(base, mode='client', listen={'endpoints': []}, connect={'endpoints': [f'tcp/127.0.0.1:{port}']}))]:
        (output / f'{name}.json5').write_text(json.dumps(value))
    os.environ.update(RMW_IMPLEMENTATION='rmw_zenoh_cpp', ROS_DOMAIN_ID='231',
                      ZENOH_ROUTER_CONFIG_URI=str(output / 'router.json5'),
                      ZENOH_SESSION_CONFIG_URI=str(output / 'client.json5'),
                      D1MAX_SHARED_MONITOR_TEST='1', D1MAX_MOCK_MOVE_DELAY_MS=str(args.move_delay_ms))
    if args.expect_zero_failure: os.environ['D1MAX_MOCK_FAIL_POST_ESTOP_ZERO'] = '1'
    else: os.environ.pop('D1MAX_MOCK_FAIL_POST_ESTOP_ZERO', None)
    bias_file = output / 'mc_bias.txt'
    bias_file.write_text('0\n')
    if args.terminal_stop_scenario != 'none': os.environ['D1MAX_MOCK_MC_BIAS_FILE'] = str(bias_file)
    else: os.environ.pop('D1MAX_MOCK_MC_BIAS_FILE', None)
    os.environ.pop('ZENOH_CONFIG_OVERRIDE', None)
    # Keep the real numerical runtime policy, but bind it to this SDK-free
    # fixture. No physical acceptance field is promoted to true.
    profile = json.loads(Path(args.profile).read_text())
    assert profile['physical_acceptance_verified'] is False
    profile['robot_id'] = 'in_process_fake_robot'
    profile['sdk_version'] = 'shared-monitor-test'
    profile['provenance']['test_scope'] = 'fake SDK only; never deploy'
    record = output / 'fake_runtime_parameters.json'
    record.write_text(json.dumps(profile, sort_keys=True))
    digest = hashlib.sha256(record.read_bytes()).hexdigest()
    routing = process = node = conn = None
    checks = []; events = []; states = []; grants = []; stops = []; retirements = []

    def write_report(status, **extra):
        report = dict(status=status, scope='production Monitor and typed ROS writer; fake robot only',
                      checks=checks, grants=grants, physical_acceptance=False,
                      move_write_delay_ms=args.move_delay_ms, injected_zero_failure=args.expect_zero_failure,
                      terminal_stop_scenario=args.terminal_stop_scenario, binary_sha256=binary_hash,
                      stop_evidence=[dict(execution=s.execution_id, confirmed=s.measured_stop_confirmed,
                          stamp_ns=s.source_stamp.sec*1000000000+s.source_stamp.nanosec, sequence=s.sequence,
                          samples=s.stationary_samples, duration=s.stationary_duration_sec,
                          raw3d=s.measured_linear_mps, metric=s.velocity_metric,
                          planar=s.measured_planar_mps, vertical=s.measured_vertical_mps,
                          angular=s.measured_angular_radps) for s in stops], **extra)
        (output/'result.json').write_text(json.dumps(report, indent=2))
        print(json.dumps({key: value for key, value in report.items() if key != 'stop_evidence'}))
    router_log = (output / 'router.log').open('w'); monitor_log = (output / 'monitor.log').open('w')
    try:
        routing = subprocess.Popen([args.router], stdout=router_log, stderr=subprocess.STDOUT)
        time.sleep(.4); assert routing.poll() is None
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from builtin_interfaces.msg import Time
        from std_msgs.msg import String
        from d1max_planning_interfaces.msg import SDKExecutionState, TaskExecutionLease, LocalMotionCommand, StopReport, SDKExecutionRetirement
        from d1max_planning_interfaces.srv import ExecutionGrant
        rclpy.init(); node = rclpy.create_node('shared_monitor_handoff_test')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(SDKExecutionState, '/d1max/live_planning/execution/sdk_state', lambda m: states.append(m), 10)
        node.create_subscription(StopReport, '/d1max/live_planning/execution/stop_report', lambda m: stops.append(m), 10)
        node.create_subscription(SDKExecutionRetirement, '/d1max/live_planning/execution/retirement', lambda m: retirements.append(m), 10)
        client = node.create_client(ExecutionGrant, '/d1max/live_planning/execution/grant')
        leases = node.create_publisher(TaskExecutionLease, '/d1max/live_planning/execution/task_lease', 10)
        commands = node.create_publisher(LocalMotionCommand, '/d1max/live_planning/execution/local_command', 10)
        acknowledgements = node.create_publisher(String, '/d1max_sdk_bridge/manual_handoff_ack', qos)
        with tempfile.TemporaryDirectory(prefix='shared-handoff-probe-') as temporary:
            ipc = str(Path(temporary) / 'manual.sock')
            params = dict(robot_ip='127.0.0.1', execution_task_writer_enabled='true', auto_take_control_on_available='false',
                          shared_estop_ipc_path=ipc, execution_acceptance_record=str(record),
                          execution_robot_id=profile['robot_id'], execution_sdk_version=profile['sdk_version'],
                          execution_calibration_sha256=profile['calibration_sha256'], execution_robot_profile_sha256=profile['robot_profile_sha256'],
                          execution_mode_policy='fixed_stair', execution_stair_runtime_parameter_source='general_runtime_reuse')
            command = [args.binary, '--ros-args']
            for key, value in params.items(): command += ['-p', f'{key}:={value}']
            process = subprocess.Popen(command, stdout=monitor_log, stderr=subprocess.STDOUT)
            until = time.monotonic() + 10
            while not Path(ipc).exists():
                assert process.poll() is None and time.monotonic() < until, 'monitor startup'
                time.sleep(.02)
            conn = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM); conn.connect(ipc); conn.setblocking(False)
            buffer = bytearray(); messages = []; latest = {}; sequence = 0; session = ''
            pump_extra = None

            def pump():
                nonlocal latest, session
                rclpy.spin_once(node, timeout_sec=.005)
                if pump_extra: pump_extra()
                try:
                    data = conn.recv(65536)
                    assert data, 'IPC unexpectedly closed'; buffer.extend(data)
                except BlockingIOError: pass
                while b'\n' in buffer:
                    raw, _, rest = buffer.partition(b'\n'); buffer[:] = rest
                    value = json.loads(raw); messages.append(value)
                    if value.get('type') in ('hello', 'state'):
                        latest = value; session = value['sdk_session']
                assert process.poll() is None, 'monitor exited'

            def wait(predicate, timeout=10):
                end = time.monotonic() + timeout
                while time.monotonic() < end:
                    pump()
                    if predicate(): return
                raise AssertionError(f'timeout latest={latest} SDK={states[-1] if states else None}')

            def ipc_command(op, expect_ok=True, wait_result=True, **fields):
                nonlocal sequence
                sequence += 1; request = str(sequence)
                conn.sendall((json.dumps(dict(type='command', version=1, sdk_session=session,
                                             request=request, op=op, **fields)) + '\n').encode())
                if not wait_result: return request
                wait(lambda: any(m.get('type') == 'result' and m.get('request') == request for m in messages))
                result = next(m for m in messages if m.get('type') == 'result' and m.get('request') == request)
                assert result['ok'] == expect_ok, result
                return result

            def stamp(ns=None):
                ns = time.time_ns() if ns is None else ns
                return Time(sec=ns // 1000000000, nanosec=ns % 1000000000)

            def grant(task, epoch, *, source=None, activate=True):
                request = ExecutionGrant.Request(); version = request.version
                version.schema_version = 3; version.session_id = 'handoff_test_navigation'
                version.task_id = task; version.route_id = task + '_route'; version.route_hash = 'a' * 64
                version.map_version_id = 'fixture_map'; version.localization_epoch = 1; version.localization_seed_id = 'seed'
                request.execution_id = task + '_execute'; request.control_epoch = epoch
                request.confirmation_id = task + '_confirm'; request.request_id = task + '_' + str(time.time_ns())
                request.sdk_session = session; request.acceptance_record_sha256 = digest
                request.source_stamp = stamp(source); request.activate = activate; request.transport_mode = 'live'
                future = client.call_async(request); wait(future.done)
                result = future.result(); grants.append(dict(task=task, accepted=result.accepted, reason=result.reason))
                return request, result

            def ack(event, **replace):
                value = dict(schema=1, sdk_session=event['sdk_session'], lease=event['lease']); value.update(replace)
                acknowledgements.publish(String(data=json.dumps(value)))
                end = time.monotonic() + .15
                while time.monotonic() < end: pump()

            def admit(task, epoch, *, allow_zero_drain=False):
                # MC reporting is configured asynchronously after ownership.
                # Only initial telemetry readiness may retry here; a stuck
                # manual_control_busy is exactly the regression under test.
                end = time.monotonic() + 5
                while True:
                    request, response = grant(task, epoch)
                    if response.accepted: return request, response
                    temporary = ('mc_capture_unavailable', 'robot_state_unavailable')
                    if allow_zero_drain: temporary += ('manual_control_busy',)
                    assert response.reason in temporary, response
                    assert time.monotonic() < end, response
                    until = time.monotonic() + .1
                    while time.monotonic() < until: pump()

            conn.sendall(b'{"type":"hello","version":1,"client":"real_ros_handoff_probe"}\n')
            wait(lambda: latest.get('state', {}).get('connected') and latest['state']['present'])
            wait(lambda: client.service_is_ready())
            ipc_command('manual_take'); wait(lambda: latest.get('manual', {}).get('phase') == 'manual')
            lease = latest['manual']['lease']
            ipc_command('posture', lease=lease, mode=3, action=3)
            ipc_command('speed', lease=lease, level=1)
            ipc_command('jog', lease=lease, generation='1', key='w', percent=30)
            wait(lambda: latest['state']['actual_forward'] > .2)
            ipc_command('release_jog', lease=lease, generation='1')
            ipc_command('manual_release', lease=lease)
            wait(lambda: latest['manual']['phase'] == 'idle')
            # Subscriber starts AFTER takeover/handback: this proves event
            # delivery does not depend on BT being present during manual setup.
            node.create_subscription(String, '/d1max_sdk_bridge/manual_handoff', lambda m: events.append(json.loads(m.data)), qos)
            wait(lambda: bool(events)); event = events[-1]
            assert event['lease'] == lease; checks.append('late_subscriber_gets_retained_takeover')
            _, denied = grant('first', 1)
            assert denied.reason == 'manual_handoff_waiting_task_fence', denied
            ack(event, sdk_session='wrong_sdk')
            _, denied = grant('first', 1)
            assert denied.reason == 'manual_handoff_waiting_task_fence', denied
            ack(event); checks.append('wrong_ack_rejected_matching_ack_accepted')
            # All telemetry is from the fake SDK through actual Monitor worker.
            first, admitted = admit('first', 1)
            assert admitted.accepted, admitted
            checks.append('normal_manual_release_allows_real_ros_grant')
            last_sent = 0.; sample = 0

            def publish_old_navigation():
                nonlocal last_sent, sample
                now = time.monotonic()
                if now - last_sent < .05: return
                last_sent = now; sample += 1; source = time.time_ns()
                lease_msg = TaskExecutionLease(schema_version=1, version=first.version,
                    execution_id=first.execution_id, control_epoch=first.control_epoch, confirmation_id=first.confirmation_id,
                    sdk_session=session, sdk_arm_generation=admitted.sdk_arm_generation, sequence=sample,
                    source_stamp=stamp(source), valid_until=stamp(source+300000000), allowed=True, phase='tracking', transport_mode='live')
                cmd = LocalMotionCommand(schema_version=1, version=first.version, execution_id=first.execution_id,
                    control_epoch=first.control_epoch, sdk_session=session, sdk_arm_generation=admitted.sdk_arm_generation,
                    lease_sequence=sample, sequence=sample, source_stamp=stamp(source), body_source_stamp=stamp(source),
                    body_valid_until=stamp(source+100000000), valid_until=stamp(source+100000000),
                    body_frame='d1max_loc_base_link', trajectory_id=1, transport_mode='live')
                cmd.velocity.linear.x = .20; leases.publish(lease_msg); commands.publish(cmd)

            pump_extra = publish_old_navigation
            wait(lambda: latest['state']['actual_forward'] > .15)
            checks.append('typed_navigation_command_reaches_unique_sdk_writer')
            if args.terminal_stop_scenario != 'none':
                pump_extra = None
                negative = args.terminal_stop_scenario in ('never_stopped', 'stair_vertical', 'stair_planar', 'stair_angular')
                fixed_stair_case = args.terminal_stop_scenario.startswith('stair_')
                if negative or fixed_stair_case:
                    bias = {'stair_planar': 4, 'stair_angular': 5}.get(args.terminal_stop_scenario, 3 if negative else 2)
                    bias_file.write_text(f'{bias}\n')
                    end = time.monotonic()+.35
                    while time.monotonic()<end: pump()
                first_id = first.execution_id
                _, revoked = grant('first', 1, activate=False)
                assert revoked.accepted, revoked
                wait(lambda: any(r.execution_id == first_id and r.software_retired and
                                 not r.motion_never_submitted for r in retirements))
                if negative:
                    end = time.monotonic()+2.
                    while time.monotonic()<end: pump()
                    assert not any(s.execution_id == first_id and s.measured_stop_confirmed for s in stops)
                    _, blocked = grant('second', 2)
                    assert not blocked.accepted and blocked.reason == 'prior_stop_unconfirmed', blocked
                    request = ipc_command('manual_take', wait_result=False)
                    wait(lambda: latest['manual']['phase'] == 'acquiring')
                    end = time.monotonic()+1.
                    while time.monotonic()<end: pump()
                    assert latest['manual']['phase'] != 'manual', latest
                    assert not any(m.get('type') == 'result' and m.get('request') == request and m.get('ok') for m in messages)
                    checks.append('no_completed_stop_neither_next_navigation_nor_manual_admitted')
                else:
                    if args.terminal_stop_scenario == 'stair_manual':
                        ipc_command('manual_take', wait_result=False)
                        wait(lambda: latest['manual']['phase'] == 'manual')
                    wait(lambda: any(s.execution_id == first_id and s.measured_stop_confirmed for s in stops))
                    certificate = next(s for s in stops if s.execution_id == first_id and s.measured_stop_confirmed)
                    if fixed_stair_case:
                        assert certificate.velocity_metric == 'planar_xy_with_vertical_guard_v1'
                        assert certificate.measured_linear_mps > .03
                        assert certificate.measured_planar_mps < .03 and certificate.measured_vertical_mps < .03
                        assert 'MOCK_CMD MCBias 2.' in (output/'monitor.log').read_text()
                        checks.append('configured_fixed_stair_first_stop_keeps_raw_norm_and_independent_xyz_guards')
                    else:
                        checks.append('nonzero_execution_genuinely_stopped_before_bias')
                        bias_file.write_text('1\n')
                        end = time.monotonic()+1.
                        while time.monotonic()<end: pump()
                        assert 'MOCK_CMD MCBias 1.' in (output/'monitor.log').read_text()
                    if args.terminal_stop_scenario in ('next_task', 'stair_next'):
                        first, admitted = grant('second', 2)
                        assert admitted.accepted, admitted
                        sample = 0; last_sent = 0.; pump_extra = publish_old_navigation
                        wait(lambda: latest['state']['actual_forward'] > .15)
                        assert not any(s.execution_id == first.execution_id and s.measured_stop_confirmed for s in stops)
                        checks.append('completed_stop_not_reopened_by_noise_next_task_moves_without_inheriting_stop')
                        pump_extra = None
                    else:
                        if args.terminal_stop_scenario != 'stair_manual':
                            ipc_command('manual_take'); wait(lambda: latest['manual']['phase'] == 'manual')
                        manual_lease = latest['manual']['lease']
                        ipc_command('jog', lease=manual_lease, generation='1', key='w', percent=30)
                        wait(lambda: latest['state']['actual_forward'] > .2)
                        ipc_command('release_jog', lease=manual_lease, generation='1')
                        checks.append('completed_stop_not_reopened_by_noise_manual_takeover_moves')
                    assert certificate.stationary_duration_sec >= 1.
                write_report('PASS'); return
            ipc_command('manual_take')
            wait(lambda: latest['manual']['phase'] == 'manual')
            second_lease = latest['manual']['lease']; assert second_lease != lease
            wait(lambda: events[-1]['lease'] == second_lease)
            second_event = events[-1]
            # Keep replaying old navigation commands during takeover and
            # posture changes. No old identity may become active again.
            ipc_command('posture', lease=second_lease, mode=3, action=4)
            ipc_command('posture', lease=second_lease, mode=3, action=3)
            end = time.monotonic()+.4
            while time.monotonic()<end: pump()
            assert latest['state']['actual_forward'] == 0
            assert not states[-1].fault_latched
            _, retired = grant('first', 2)
            assert retired.reason == 'manual_takeover_retired_task', retired
            checks.append('moving_navigation_retired_before_manual_posture_old_commands_fenced')
            pump_extra = None
            ack(event)
            _, denied = grant('second', 2)
            assert denied.reason == 'manual_handoff_waiting_task_fence', denied
            ack(second_event)
            ipc_command('jog', lease=second_lease, generation='1', key='w', percent=30)
            wait(lambda: latest['state']['actual_forward'] > .2)
            ipc_command('heartbeat', lease=second_lease, generation='1', key='w', percent=30)
            _, busy = grant('second', 2)
            assert busy.reason == 'manual_control_busy' and busy.definitely_not_admitted, busy
            ipc_command('release_jog', lease=second_lease, generation='1')
            second, admitted2 = admit('second', 2, allow_zero_drain=True)
            assert admitted2.accepted, admitted2
            ipc_command('heartbeat', expect_ok=False, lease=second_lease, generation='1', key='w', percent=30)
            checks.append('held_manual_not_stolen_release_key_auto_hands_back_old_heartbeat_rejected')
            checks.append('old_ack_cannot_release_new_takeover_fresh_task_can_start')
            # No second navigation motion needed: manual setup can retire a
            # granted-but-never-moving execution without a fake stop witness.
            ipc_command('manual_take'); wait(lambda: latest['manual']['phase'] == 'manual')
            third_lease = latest['manual']['lease']; wait(lambda: events[-1]['lease'] == third_lease)
            third_event = events[-1]
            ipc_command('jog', lease=third_lease, generation='1', key='w', percent=30)
            wait(lambda: latest['state']['actual_forward'] > .2)
            ipc_command('emergency', on=True)
            wait(lambda: latest['state']['soft'] == 2 and not latest['state']['estop_pending'])
            # Leave ON long enough for the original manual write to retire.
            # An immediate OFF can hide an Idle-with-zero-debt scheduling bug.
            wait(lambda: latest['manual']['phase'] == 'idle')
            ipc_command('emergency', on=False)
            wait(lambda: latest['state']['soft'] == 1 and not latest['state']['estop_pending'])
            ack(third_event)
            # Direct new navigation grant, NOT another manual_take workaround.
            if args.expect_zero_failure:
                end = time.monotonic() + 1.5
                while time.monotonic() < end: pump()
                assert 'MOCK_CMD MoveFailure' in (output/'monitor.log').read_text(), 'fixture: zero failure must reach the writer before testing a new grant; use --move-delay-ms 700'
                third, admitted3 = grant('after_estop', 3)
                assert not admitted3.accepted and admitted3.reason in ('manual_control_busy', 'writer_fault_latched'), admitted3
                checks.append('failed_post_estop_zero_never_authorizes_navigation')
            else:
                third, admitted3 = admit('after_estop', 3, allow_zero_drain=True)
                assert admitted3.accepted, admitted3
                checks.append('held_manual_estop_release_allows_new_navigation_without_retake')
            assert latest['state']['actual_forward'] == 0
            write_report('PASS')
    except Exception as error:
        write_report('FAIL', error=str(error))
        raise
    finally:
        if conn: conn.close()
        if process and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try: process.wait(timeout=10)
            except subprocess.TimeoutExpired: process.kill(); process.wait()
        if node: node.destroy_node(); rclpy.shutdown()
        if routing:
            routing.send_signal(signal.SIGINT)
            try: routing.wait(timeout=5)
            except subprocess.TimeoutExpired: routing.kill(); routing.wait()
        router_log.close(); monitor_log.close()


if __name__ == '__main__': main()
