#!/usr/bin/env python3
"""Isolated ROS/Zenoh motion-interface smoke; every robot input is TEST_ONLY.

Runs the real coordinator, tracker and command gate, with a fake scoped SDK arm
service and an explicitly fake collision pass-through. No SDK library, monitor,
robot connection, localization or native planner is started. This verifies wire
contracts and fail-closed execution, not collision geometry or robot behavior.
"""
import argparse
from datetime import datetime
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
import uuid

import yaml

WS = Path(__file__).resolve().parents[3]
PREFIX = '/d1max/live_planning/'
PORT, DOMAIN = 17472, '229'
FRAME, BASE, ODOM = 'd1max_loc_map', 'd1max_loc_base_link', 'd1max_loc_odom'


def prepare(directory):
    from d1max_pct_scan.motion_execution import MotionConfig
    from d1max_pct_scan.motion_stack import parameters
    sid = 'TEST_ONLY_' + uuid.uuid4().hex
    motion = dict(body_height=.55, max_speed=.30, max_yaw=.50, single_floor_height_span=.25)
    motion.update({name: True for name in MotionConfig(sid, 'TEST_ONLY_map').acceptance_fields})
    session = dict(mode='LIVE_NAVIGATION', motion_control_enabled=True, id=sid,
                   version_id='TEST_ONLY_map', frame_id=FRAME, motion=motion,
                   fixture_kind='OFFLINE_SYNTHETIC_INTERFACES_ONLY', robot_connected=False)
    (directory/'session.json').write_text(json.dumps(session, indent=2))
    (directory/'motion.yaml').write_text(yaml.safe_dump(parameters(session, WS)))
    scout = {'multicast': {'enabled': False}, 'gossip': {'enabled': False}}
    common = dict(scouting=scout, timestamping={'enabled': True, 'drop_future_timestamp': False})
    for name, value in (
        ('client', dict(common, mode='client', connect={
            'endpoints': [f'tcp/127.0.0.1:{PORT}'], 'exit_on_failure': True})),
        ('router', dict(common, mode='router', listen={
            'endpoints': [f'tcp/127.0.0.1:{PORT}'], 'exit_on_failure': True},
            connect={'endpoints': []})),
    ):
        (directory/(name+'.json5')).write_text(json.dumps(value, indent=2))
    return session


def environment(directory):
    result = dict(os.environ)
    for key in ('ZENOH_CONFIG_OVERRIDE', 'ZENOH_SESSION_CONFIG', 'ZENOH_ROUTER_CONFIG'):
        result.pop(key, None)
    result.update(RMW_IMPLEMENTATION='rmw_zenoh_cpp', ROS_DOMAIN_ID=DOMAIN,
        ZENOH_SESSION_CONFIG_URI=str(directory/'client.json5'),
        ZENOH_ROUTER_CONFIG_URI=str(directory/'router.json5'),
        ROS_LOG_DIR=str(directory/'ros_logs'), PYTHONUNBUFFERED='1')
    sources = [str(WS/'src'/package) for package in
               ('d1max_pct_scan', 'd1max_navigation', 'd1max_pct_planner')]
    result['PYTHONPATH'] = os.pathsep.join(sources+[result.get('PYTHONPATH', '')])
    return result


def run(directory, session):
    # Bind-test prevents accidentally reusing a different router on this port.
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', PORT))
    env = environment(directory)
    os.environ.update(env)
    for key in ('ZENOH_CONFIG_OVERRIDE', 'ZENOH_SESSION_CONFIG', 'ZENOH_ROUTER_CONFIG'):
        os.environ.pop(key, None)
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from ament_index_python.packages import get_package_prefix
    from geometry_msgs.msg import Point, PoseStamped, Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String, Bool
    from std_srvs.srv import SetBool
    from d1max_planning_interfaces.msg import ReferencePath, TaggedBspline

    started, deadline = time.monotonic(), time.monotonic()+90.
    processes, streams, expected_exits = {}, [], set()
    report = dict(schema=1, kind='OFFLINE_SYNTHETIC_INTERFACES_ONLY', session_id=session['id'],
        robot_connected=False, sdk_library_loaded=False, transport='rmw_zenoh_cpp',
        loopback_port=PORT, ros_domain_id=DOMAIN, tests={}, process_cleanup={},
        collision_fixture='TEST_ONLY Twist pass-through; native collision behavior is not tested here')
    node = None

    def spawn(name, command):
        log = (directory/(name+'.log')).open('w')
        streams.append(log)
        processes[name] = subprocess.Popen(command, cwd=WS, env=env, stdout=log,
                                           stderr=subprocess.STDOUT, start_new_session=True)
        return processes[name]

    def alive():
        for name, process in processes.items():
            if name not in expected_exits and process.poll() is not None:
                raise RuntimeError(f'{name} exited unexpectedly: {process.returncode}; see {directory/name}.log')

    class Fixture(Node):
        def __init__(self):
            super().__init__('TEST_ONLY_motion_fixture')
            self.statuses, self.arm_calls, self.packets, self.forwarded, self.tasks = {}, [], [], [], []
            self.freeze = None
            self.sdk_session = uuid.uuid4().hex[:16]
            self.sdk_prefix = '/d1max/monitor/s_'+self.sdk_session
            self.sdk_armed, self.arm_generation, self.ownership = False, 0, True
            self.generation, self.plan_id, self.reference_stamp = 0, 0, 0.
            self.source_stamp, self.admission_sequence = 0., 0
            self.admission_enabled, self.auto_splines = True, False
            self.next_spline = math.inf
            topics = {'admission': PREFIX+'execution_admission', 'operator': PREFIX+'motion_command',
                'localization': '/d1max/localization/status', 'robot': '/d1max_sdk_bridge/robot_state',
                'behavior': '/d1max_sdk_bridge/behavior_state', 'mc': '/d1max_sdk_bridge/speed_report_status',
                'monitor': '/d1max/monitor/status'}
            self.json_pubs = {key: self.create_publisher(String, topic, 5) for key, topic in topics.items()}
            self.reference_pub = self.create_publisher(ReferencePath, PREFIX+'scan_reference', 1)
            self.spline_pub = self.create_publisher(TaggedBspline, PREFIX+'validated_tagged_bspline', 1)
            self.global_odom = self.create_publisher(Odometry, '/d1max/localization/odometry/global', qos_profile_sensor_data)
            self.local_odom = self.create_publisher(Odometry, '/d1max/localization/odometry/local', qos_profile_sensor_data)
            self.scan_pub = self.create_publisher(LaserScan, PREFIX+'safety_scan', qos_profile_sensor_data)
            self.collision_pub = self.create_publisher(Twist, PREFIX+'cmd_vel_collision_checked', 1)
            self.create_subscription(Twist, PREFIX+'cmd_vel_admitted', self.collision_pub.publish, 1)
            self.create_subscription(String, self.sdk_prefix+'/navigation_velocity', self.velocity, 20)
            self.create_subscription(TaggedBspline, PREFIX+'execution_bspline',
                lambda msg: self.forwarded.append(dict(wall=time.time(), id=msg.trajectory.traj_id,
                    generation=msg.generation, source=msg.trajectory.start_time.sec+msg.trajectory.start_time.nanosec*1e-9)), 10)
            self.create_subscription(String, PREFIX+'execution_task',
                lambda msg: self.tasks.append(json.loads(msg.data)), 5)
            self.create_subscription(Bool, PREFIX+'execution_frozen', lambda msg: setattr(self, 'freeze', msg.data), 1)
            for name in ('motion_status', 'tracker_status', 'command_gate_status'):
                self.create_subscription(String, PREFIX+name,
                    lambda msg, key=name: self.statuses.update({key: json.loads(msg.data)}), 5)
            self.create_service(SetBool, self.sdk_prefix+'/navigation_arm', self.arm)
            self.create_timer(.05, self.tick)

        def publish(self, key, value):
            self.json_pubs[key].publish(String(data=json.dumps(value, allow_nan=False)))

        def arm(self, request, response):
            if request.data:
                self.arm_generation += 1
            self.sdk_armed = bool(request.data)
            self.arm_calls.append(dict(allow=request.data, wall=time.time(), generation=self.arm_generation))
            response.success = True
            response.message = json.dumps(dict(sdk_session=self.sdk_session,
                arm_generation=self.arm_generation, armed=self.sdk_armed, test_only=True))
            return response

        def velocity(self, message):
            value = json.loads(message.data)
            value['observed_wall'] = time.time()
            value['fixture_was_armed'] = self.sdk_armed
            self.packets.append(value)

        def tick(self):
            stamp = self.get_clock().now().to_msg()
            wall = stamp.sec+stamp.nanosec*1e-9
            for pub, frame in ((self.global_odom, FRAME), (self.local_odom, ODOM)):
                odom = Odometry()
                odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp, frame, BASE
                odom.pose.pose.position.z, odom.pose.pose.orientation.w = .55, 1.
                pub.publish(odom)
            scan = LaserScan()
            scan.header.stamp, scan.header.frame_id = stamp, BASE
            scan.angle_min, scan.angle_increment = -math.pi, 2*math.pi/720
            scan.angle_max, scan.range_min, scan.range_max = math.pi-scan.angle_increment, .15, 12.
            scan.ranges = [5.]*720
            self.scan_pub.publish(scan)
            self.publish('localization', dict(session_id=session['id'], map_version_id=session['version_id'],
                wall_time=wall, state='tracking', localized=True, navigation_ready=True, local_fault='',
                frames={'map': FRAME}, navigation={'valid': True, 'fault': ''},
                calibration={'extrinsics_verified': True, 'time_alignment_verified': True}, test_only=True))
            self.publish('robot', dict(received_at_unix=wall, software_emergency_status=1,
                hardware_emergency_status=1, control_source=2 if self.ownership else 1,
                sport_mode=1, motion_status=5, speed_level=1, head_direction=1, test_only=True))
            self.publish('behavior', dict(telemetry_only=False, ready_for_navigation=True,
                sdk_has_control=self.ownership, replay_latched=False, fault_latched=False,
                requires_review=False, received_at_unix=wall, test_only=True))
            self.publish('mc', dict(source='sdk_mc', callback='OnMcData', stream_fresh=True,
                rate_ok=True, received_at_unix=wall, test_only=True))
            self.publish('monitor', dict(mode='monitor', session=self.sdk_session,
                service_prefix=self.sdk_prefix, wall_time=wall, motion_control_enabled=True,
                navigation_arm_generation=self.arm_generation, navigation_armed=self.sdk_armed, test_only=True))
            if self.auto_splines and time.monotonic() >= self.next_spline:
                self.new_spline()
            if self.generation and self.source_stamp and self.admission_enabled:
                self.admission_sequence += 1
                self.publish('admission', dict(schema=1, session_id=session['id'], execution_mode='execution',
                    generation=self.generation, trajectory_id=self.plan_id, valid=True, reason='TEST_ONLY_accepted',
                    sequence=self.admission_sequence, frame_id=FRAME, issued_at=self.reference_stamp,
                    source_stamp=self.source_stamp, debug_stamp=self.source_stamp, stamp=wall,
                    received_at_unix=wall, lease_timeout_sec=.35,
                    localization_context=[session['id'], 1, 'TEST_ONLY_seed'], motion_authorized=False))

        def new_reference(self):
            self.generation += 1
            message = ReferencePath()
            message.session_id, message.generation = session['id'], self.generation
            message.path.header.frame_id = FRAME
            message.path.header.stamp = self.get_clock().now().to_msg()
            self.reference_stamp = message.path.header.stamp.sec+message.path.header.stamp.nanosec*1e-9
            for x in (0., .5, 1., 1.5, 2.):
                pose = PoseStamped()
                pose.header = message.path.header
                pose.pose.position.x, pose.pose.orientation.w = x, 1.
                message.path.poses.append(pose)
            self.reference_pub.publish(message)
            self.new_spline()

        def new_spline(self):
            self.plan_id += 1
            message = TaggedBspline()
            message.session_id, message.generation, message.frame_id = session['id'], self.generation, FRAME
            raw = message.trajectory
            raw.traj_id, raw.order = self.plan_id, 3
            raw.start_time = self.get_clock().now().to_msg()
            self.source_stamp = raw.start_time.sec+raw.start_time.nanosec*1e-9
            raw.pos_pts = [Point(x=(i-1)*.05, y=0., z=.55) for i in range(23)]
            raw.knots = [(i-3)*.2 for i in range(27)]
            self.spline_pub.publish(message)
            self.next_spline = time.monotonic()+.7

        def operator(self, action):
            self.publish('operator', dict(session_id=session['id'], generation=self.generation,
                action=action, id=uuid.uuid4().hex, stamp=time.time()))

        def pump(self, seconds):
            until = min(deadline, time.monotonic()+seconds)
            while time.monotonic() < until:
                alive()
                rclpy.spin_once(self, timeout_sec=.01)

        def wait(self, condition, seconds, label):
            until = min(deadline, time.monotonic()+seconds)
            while time.monotonic() < until:
                alive()
                rclpy.spin_once(self, timeout_sec=.01)
                if condition():
                    report['tests'][label] = dict(passed=True, elapsed_s=time.monotonic()-started)
                    return
            raise AssertionError(label+': '+json.dumps(self.statuses, ensure_ascii=False))

        def nonzero(self, since=0):
            return [p for p in self.packets[since:] if any(abs(p[k]) > 1e-8 for k in ('x', 'y', 'yaw'))]

        def start_execution(self, label):
            self.auto_splines, self.admission_enabled = False, True
            self.new_reference()
            self.wait(lambda: self.statuses.get('motion_status', {}).get('can_execute') is True
                      and self.statuses['motion_status'].get('generation') == self.generation,
                      3., label+'_ready')
            self.operator('execute')
            before = len(self.packets)
            self.wait(lambda: self.sdk_armed and self.statuses.get('motion_status', {}).get('phase') == 'waiting_trajectory',
                      3., label+'_arm_ack')
            ack = self.arm_calls[-1]['wall']
            self.pump(.15)
            assert not self.nonzero(before), 'old pre-arm spline produced nonzero command'
            assert not any(x['generation'] == self.generation for x in self.forwarded), 'pre-arm spline reached tracker'
            self.new_spline()
            new_source = self.source_stamp
            assert new_source >= ack
            self.auto_splines = True
            self.wait(lambda: bool(self.nonzero(before)), 3., label+'_bounded_velocity')
            first = self.nonzero(before)[0]
            assert first['observed_wall'] >= ack and first['fixture_was_armed']
            assert any(x['generation'] == self.generation and x['source'] >= ack for x in self.forwarded)

        def no_resume(self, label):
            count = len([a for a in self.arm_calls if a['allow']])
            before = len(self.packets)
            self.new_spline()
            self.auto_splines, self.admission_enabled = True, True
            self.pump(.75)
            assert len([a for a in self.arm_calls if a['allow']]) == count
            assert not self.nonzero(before) and self.freeze is True
            report['tests'][label] = {'passed': True}

    try:
        router = Path(get_package_prefix('rmw_zenoh_cpp'))/'lib/rmw_zenoh_cpp/rmw_zenohd'
        spawn('router', [str(router)])
        until = time.monotonic()+4.
        while time.monotonic() < until:
            alive()
            try:
                with socket.create_connection(('127.0.0.1', PORT), timeout=.1):
                    break
            except OSError:
                time.sleep(.05)
        else:
            raise TimeoutError('private router did not start')
        rclpy.init(args=[])
        node = Fixture()
        config_args = ['--ros-args', '-r', '__ns:=/d1max/live_planning', '--params-file', str(directory/'motion.yaml')]
        tracker = WS/'build/d1max_trajectory_tracker/trajectory_tracker'
        if not tracker.is_file():
            raise RuntimeError('Build actual trajectory_tracker before this smoke')
        spawn('tracker', [str(tracker), *config_args, '-r', '__node:=trajectory_tracker'])
        spawn('gate', [sys.executable, '-c', 'from d1max_navigation.command_gate import main; main()',
                       *config_args, '-r', '__node:=navigation_command_gate'])
        spawn('coordinator', [sys.executable, '-m', 'd1max_pct_scan.motion_coordinator', '--session', str(directory)])
        node.wait(lambda: all(k in node.statuses for k in ('motion_status', 'tracker_status', 'command_gate_status')),
                  12., 'actual_nodes_discovered')
        node.new_reference()
        node.pump(.6)
        assert not any(a['allow'] for a in node.arm_calls) and not node.nonzero() and node.freeze is True
        report['tests']['no_execute_no_arm_or_nonzero'] = {'passed': True}

        node.start_execution('operator_stop')
        node.operator('stop')
        node.wait(lambda: not node.sdk_armed and node.statuses.get('motion_status', {}).get('phase') == 'locked',
                  2., 'operator_stop_disarms')
        node.no_resume('operator_stop_recovery_does_not_resume')

        node.start_execution('admission_loss')
        node.admission_enabled, node.auto_splines = False, False
        node.wait(lambda: not node.sdk_armed and node.statuses.get('motion_status', {}).get('phase') == 'locked',
                  2., 'admission_loss_disarms')
        assert 'local_trajectory_not_admitted' in node.statuses['motion_status']['stop_reason']
        node.no_resume('admission_recovery_does_not_resume')

        node.start_execution('ownership_loss')
        node.ownership = False
        node.wait(lambda: not node.sdk_armed and node.statuses.get('motion_status', {}).get('phase') == 'locked',
                  2., 'ownership_loss_disarms')
        node.ownership = True
        node.no_resume('ownership_recovery_does_not_resume')

        node.start_execution('coordinator_exit')
        expected_exits.add('coordinator')
        processes['coordinator'].kill()
        node.wait(lambda: not node.sdk_armed and processes['coordinator'].poll() is not None,
                  2., 'abrupt_coordinator_exit_revokes_sdk_by_lease')
        node.pump(.3)
        for packet in node.packets:
            assert packet['sdk_session'] == node.sdk_session
            assert packet['navigation_session'] == session['id'] and packet['map_version_id'] == session['version_id']
            assert math.hypot(packet['x'], packet['y']) <= .30+1e-9 <= 1.5
            assert abs(packet['y']) <= 1e-9 and abs(packet['yaw']) <= .50+1e-9
            assert all(math.isfinite(packet[k]) for k in ('x', 'y', 'yaw'))
        report.update(passed=True, sdk_arm_calls=node.arm_calls, sdk_packets=node.packets,
                      forwarded_splines=node.forwarded, final_status=node.statuses)
    except Exception as exc:
        report.update(passed=False, error=repr(exc), final_status=node.statuses if node else {})
        raise
    finally:
        for name in ('coordinator', 'tracker', 'gate'):
            process = processes.get(name)
            if process and process.poll() is None:
                process.send_signal(signal.SIGINT)
        if node is not None and rclpy.ok():
            until = time.monotonic()+.3
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=.01)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for name, process in reversed(list(processes.items())):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=3.)
            except subprocess.TimeoutExpired:
                process.terminate()
                try:
                    process.wait(timeout=2.)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=2.)
            report['process_cleanup'][name] = {'pid': process.pid, 'returncode': process.returncode}
        for stream in streams:
            stream.close()
        report['elapsed_s'] = time.monotonic()-started
        (directory/'report.json').write_text(json.dumps(report, indent=2, ensure_ascii=False))
        print(json.dumps({'passed': report.get('passed', False), 'report': str(directory/'report.json'),
                          'tests': len(report['tests']), 'elapsed_s': report['elapsed_s']}, ensure_ascii=False))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prepare-only', action='store_true')
    args = parser.parse_args()
    directory = WS/'log/offline_motion_chain_smoke'/(datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8])
    directory.mkdir(parents=True)
    session = prepare(directory)
    if args.prepare_only:
        print(json.dumps({'prepared': str(directory), 'started_processes': False, 'robot_connected': False}))
    else:
        run(directory, session)


if __name__ == '__main__':
    main()
