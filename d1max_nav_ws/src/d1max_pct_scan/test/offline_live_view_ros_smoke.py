#!/usr/bin/env python3
"""Explicit ROS smoke (not pytest): TEST_ONLY LiveView, no SDK or localizer.

Run with the workspace ROS/Zenoh environment sourced. Uses private loopback
17467/domain224 and kills only its own process groups. Deadline: 12 s of ROS
interaction, plus at most 4 s startup/cleanup. No full live_session is started.
"""
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
import uuid

from offline_live_chain_smoke import WS, FRAME, PREFIX, PORT, isolated_environment


def main():
    started = time.monotonic()
    directory = WS/'log/offline_live_view_smoke'/(
        time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8])
    directory.mkdir(parents=True)
    sid = 'TEST_ONLY_'+uuid.uuid4().hex[:12]
    session = dict(id=sid, frame_id=FRAME, initial_body_z=.55,
                   mode='OFFLINE_SYNTHETIC_TEST_ONLY', motion_control_enabled=False)
    (directory/'session.json').write_text(json.dumps(session))
    common = {'scouting': {'multicast': {'enabled': False}, 'gossip': {'enabled': False}},
              'timestamping': {'enabled': True, 'drop_future_timestamp': False}}
    (directory/'client.json5').write_text(json.dumps(dict(common, mode='client',
        connect={'endpoints': [f'tcp/127.0.0.1:{PORT}'], 'exit_on_failure': True})))
    (directory/'router.json5').write_text(json.dumps(dict(common, mode='router',
        listen={'endpoints': [f'tcp/127.0.0.1:{PORT}'], 'exit_on_failure': True},
        connect={'endpoints': []})))
    with socket.socket() as check:
        check.bind(('127.0.0.1', PORT))
    env = isolated_environment(directory)
    os.environ.update(env)
    for key in ('ZENOH_CONFIG_OVERRIDE', 'ZENOH_SESSION_CONFIG', 'ZENOH_ROUTER_CONFIG'):
        os.environ.pop(key, None)
    import rclpy
    from rclpy.node import Node
    from ament_index_python.packages import get_package_prefix
    from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String, Bool, Empty
    from visualization_msgs.msg import InteractiveMarkerUpdate, InteractiveMarkerFeedback, Marker

    children, streams, node = [], [], None
    report = dict(kind='OFFLINE_SYNTHETIC_TEST_ONLY', passed=False, tests={},
                  session_id=sid, no_sdk=True, no_localizer=True, no_control=True,
                  domain=224, router_port=PORT)
    def spawn(name, args):
        stream = (directory/(name+'.log')).open('w')
        streams.append(stream)
        child = subprocess.Popen(args, cwd=WS, env=env, stdout=stream,
                                 stderr=subprocess.STDOUT, start_new_session=True)
        children.append((name, child))
        return child

    try:
        router = Path(get_package_prefix('rmw_zenoh_cpp'))/'lib/rmw_zenoh_cpp/rmw_zenohd'
        spawn('router', [str(router)])
        router_deadline = time.monotonic()+2.
        while True:
            try:
                with socket.create_connection(('127.0.0.1', PORT), timeout=.1):
                    break
            except OSError:
                if time.monotonic() > router_deadline:
                    raise TimeoutError('private router did not start')
                time.sleep(.05)
        rclpy.init(args=[])
        node = Node('TEST_ONLY_live_view_probe', enable_rosout=False)
        status = node.create_publisher(String, '/d1max/localization/status', 10)
        scan = node.create_publisher(String, PREFIX+'scan_bridge_status', 10)
        odom = node.create_publisher(Odometry, '/d1max/localization/odometry/global', qos_profile_sensor_data)
        seed = node.create_publisher(PoseWithCovarianceStamped, PREFIX+'initialpose', 10)
        place = node.create_publisher(PointStamped, PREFIX+'place_goal3d', 10)
        activate = node.create_publisher(Empty, PREFIX+'activate_goal3d', 10)
        feedback = node.create_publisher(InteractiveMarkerFeedback, '/d1max_live_goal/feedback', 10)
        received = {'markers': [], 'goals': [], 'freeze': [], 'status': [], 'body': [], 'diagnostics': []}
        node.create_subscription(InteractiveMarkerUpdate, '/d1max_live_goal/update',
                                 lambda m: received['markers'].extend(m.markers), 10)
        node.create_subscription(PointStamped, PREFIX+'goal3d', lambda m: received['goals'].append(m), 10)
        node.create_subscription(Bool, PREFIX+'execution_frozen', lambda m: received['freeze'].append(m.data), 10)
        node.create_subscription(Marker, PREFIX+'view_status', lambda m: received['status'].append(m.text), 10)
        node.create_subscription(Marker, PREFIX+'body_marker', lambda m: received['body'].append(m), 1)
        node.create_subscription(String, PREFIX+'diagnostics',
                                 lambda m: received['diagnostics'].append(json.loads(m.data)), 1)
        acknowledgement = {}
        localized = False
        publish_body = False
        epoch = 1
        def publish_status():
            wall = time.time()
            navigation = dict(epoch=epoch, seed_id=f'seed{epoch}', valid=localized,
                received_at_unix=wall, navigation_ready=False, fault='', target_hz=50.,
                global_observed_hz=50., calibration=dict(extrinsics_verified=False,
                    time_alignment_verified=False))
            status.publish(String(data=json.dumps(dict(
                session_id=sid, wall_time=wall, initial_pose_ready=True,
                state='tracking' if localized else 'waiting_initial_pose', localized=localized,
                navigation=navigation, verified_confirmations=3 if localized else 0,
                local_epoch=epoch, active_seed_ns=f'seed{epoch}' if localized else None,
                confirmed_seed_ns=f'seed{epoch}' if localized else None,
                command_result=acknowledgement))))
            if publish_body:
                message = Odometry()
                message.header.frame_id, message.header.stamp = FRAME, node.get_clock().now().to_msg()
                message.child_frame_id = 'd1max_loc_base_link'
                message.pose.pose.position.x, message.pose.pose.position.y, message.pose.pose.position.z = 4.9, 22.2, .55
                message.pose.pose.orientation.w = 1.
                odom.publish(message)
        node.create_timer(.1, publish_status)
        spawn('view', [sys.executable, '-m', 'd1max_pct_scan.live_view', '--session', str(directory)])
        deadline = time.monotonic()+16.
        def wait(check, label):
            while time.monotonic() < deadline:
                failed = [(name, p.poll()) for name, p in children if p.poll() is not None]
                if failed:
                    raise RuntimeError('owned child exited: '+repr(failed))
                rclpy.spin_once(node, timeout_sec=.02)
                if check():
                    report['tests'][label] = True
                    return
            raise TimeoutError(label)
        wait(lambda: len(received['freeze']) >= 2 and received['status'], 'construct_and_publish_state')
        assert all(received['freeze']), 'execution must remain frozen'
        activate.publish(Empty())
        wait(lambda: any(m.name == 'goal' for m in received['markers']), 'toolbar_activation_without_cloud_hit')
        handle = received['markers'][-1]
        assert set(('move_x', 'move_y', 'move_z', 'rotate_x', 'rotate_y', 'rotate_z')) <= {c.name for c in handle.controls}
        assert not received['goals'], 'activating the tool must not submit any goal'
        report['tests']['moveit_xyz_rotation_controls_and_no_autosubmit'] = True
        wait(lambda: bool(received['diagnostics']), 'read_only_diagnostics_contract_published')
        assert received['diagnostics'][-1]['stages']['localization']['label'] == '请给初值'
        request = PoseWithCovarianceStamped()
        request.header.frame_id, request.header.stamp = FRAME, node.get_clock().now().to_msg()
        request.pose.pose.position.x, request.pose.pose.position.y = 4.9, 22.2
        request.pose.pose.orientation.w = 1.
        seed.publish(request)
        mailbox = directory/'initial_pose.json'
        wait(mailbox.is_file, 'initial_pose_only_written_to_test_mailbox')
        command = json.loads(mailbox.read_text())
        assert command['session_id'] == sid and command['reference'] == 'body'
        assert command['x'] == 4.9 and command['y'] == 22.2 and command['z'] == .55
        acknowledgement.update(id=command['id'], accepted=True, message='TEST_ONLY_mailbox_ack')
        goal = PointStamped()
        goal.header.frame_id, goal.header.stamp = FRAME, node.get_clock().now().to_msg()
        goal.point.x, goal.point.y, goal.point.z = 5.2, 25.2, -.57
        place.publish(goal)
        wait(lambda: any(m.name == 'goal' for m in received['markers']), 'interactive_goal_marker_published')
        marker = next(m for m in reversed(received['markers']) if m.name == 'goal')
        assert not received['goals'], 'placing a marker must not submit a goal'
        menu = next(m for m in marker.menu_entries if '规划到此处' in m.title)
        # Humble InteractiveMarkerServer protects a newly created marker from
        # another client for its first second; mimic a real user reading a menu.
        feedback_after = time.monotonic()+1.05
        wait(lambda: time.monotonic() >= feedback_after, 'interactive_server_client_handoff')
        event = InteractiveMarkerFeedback()
        event.header.frame_id, event.header.stamp = FRAME, node.get_clock().now().to_msg()
        event.client_id, event.marker_name = 'TEST_ONLY_probe', 'goal'
        event.event_type, event.menu_entry_id = event.MENU_SELECT, menu.id
        event.pose = marker.pose
        feedback.publish(event)
        denied_until = time.monotonic()+.3
        wait(lambda: time.monotonic() >= denied_until, 'unlocalized_commit_checked')
        assert not received['goals'], 'unlocalized editor must not submit a goal'
        report['tests']['unlocalized_commit_denied'] = True
        def snapshot():
            path = directory/'view_status.json'
            return json.loads(path.read_text()) if path.is_file() else {}
        wait(lambda: 'TEST_ONLY_mailbox_ack' in (snapshot().get('seed_feedback') or ''),
             'fresh_seed_feedback_snapshot')
        localized = publish_body = True
        wait(lambda: received['diagnostics'][-1].get('preview_ready') is True,
             'fresh_localization_allows_explicit_goal_commit')
        event.header.stamp = node.get_clock().now().to_msg()
        feedback.publish(event)
        wait(lambda: bool(received['goals']), 'menu_commit_publishes_goal3d')
        published = received['goals'][-1]
        assert published.header.frame_id == FRAME
        assert [getattr(published.point, k) for k in 'xyz'] == [5.2, 25.2, -.57]
        assert snapshot()['scan_status'] == {}, 'no SCAN node was started'
        scan.publish(String(data=json.dumps(dict(session_id=sid, received_at_unix=time.time(),
                         reason='TEST_ONLY_one_status', motion_enabled=False))))
        wait(lambda: snapshot().get('scan_status', {}).get('reason') == 'TEST_ONLY_one_status',
             'fresh_scan_status_displayed')
        wait(lambda: snapshot().get('scan_status') == {}, 'expired_scan_status_cleared')
        previous_snapshot = snapshot().get('received_at_unix', 0.)
        scan.publish(String(data=json.dumps(dict(session_id=sid,
                         received_at_unix=time.time()-5., reason='TEST_ONLY_STALE_SOURCE'))))
        wait(lambda: snapshot().get('received_at_unix', 0.) > previous_snapshot,
             'snapshot_after_stale_source_received')
        assert snapshot().get('scan_status') == {}, 'late source timestamp was treated as fresh'
        assert not any('TEST_ONLY_STALE_SOURCE' in text for text in received['status'])
        report['tests']['stale_source_status_not_revived_by_new_receipt'] = True
        wait(lambda: bool(received['diagnostics']), 'read_only_diagnostics_contract_published')
        diag = received['diagnostics'][-1]
        assert diag['session_id'] == sid and diag['motion_enabled'] is False
        assert diag['stages']['local']['tone'] != 'ready'
        localized = publish_body = True
        wait(lambda: received['diagnostics'][-1].get('preview_ready') is True,
             'localized_preview_ready_without_hardware_admission')
        diag = received['diagnostics'][-1]
        assert diag['stages']['localization']['label'] == '已定位'
        assert diag['navigation_admission']['ready'] is False
        assert any('外参' in reason for reason in diag['navigation_admission']['blockers'])
        assert any('时间对齐' in reason for reason in diag['navigation_admission']['blockers'])
        report['tests']['unverified_calibration_is_explicit_not_endless_confirmation'] = True
        wait(lambda: received['body'] and received['body'][-1].action == Marker.ADD,
             'fresh_measured_body_pose_shown')
        body = received['body'][-1]
        assert body.header.frame_id == FRAME and body.type == Marker.LINE_LIST
        assert len(body.points) == 6 and len(body.colors) == 6
        assert all(color.a == 1. for color in body.colors)
        assert all(not text for text in received['status']), 'world status captions must stay cleared'
        assert [getattr(body.pose.position, k) for k in 'xyz'] == [4.9, 22.2, .55]
        assert 0 < body.lifetime.sec+body.lifetime.nanosec*1e-9 <= .5
        publish_body = False
        epoch += 1
        wait(lambda: received['body'][-1].action == Marker.DELETE, 'new_epoch_deletes_previous_body')
        before = len(received['body'])
        obsolete = Odometry()
        obsolete.header = body.header
        obsolete.child_frame_id = 'd1max_loc_base_link'
        obsolete.pose.pose = body.pose
        odom.publish(obsolete)
        check_after = time.monotonic()+.15
        wait(lambda: time.monotonic() >= check_after, 'old_epoch_body_packet_received')
        assert not any(m.action == Marker.ADD for m in received['body'][before:])
        report['tests']['old_epoch_body_cannot_reappear'] = True
        publish_body = True
        wait(lambda: received['body'][-1].action == Marker.ADD, 'new_epoch_fresh_body_visible')
        publish_body = False
        wait(lambda: received['body'][-1].action == Marker.DELETE, 'stale_body_pose_deleted')
        assert not any(m.action == Marker.ADD and m.lifetime.sec > 0 for m in received['body'])
        report['snapshot'] = snapshot()
        report['marker_control_names'] = [m.name for m in marker.controls]
        report['published_goals'] = len(received['goals'])
        assert not node.get_publishers_info_by_topic('/cmd_vel')
        report['tests']['no_cmd_vel_publisher'] = True
        report['passed'] = True
    except BaseException as exc:
        report['error'] = type(exc).__name__+': '+str(exc)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()
        for _, child in reversed(children):
            try:
                os.killpg(child.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        limit = time.monotonic()+2.
        for _, child in reversed(children):
            try:
                child.wait(timeout=max(.01, limit-time.monotonic()))
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(child.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                child.wait(timeout=1.)
        for stream in streams:
            stream.close()
        view_log = (directory/'view.log').read_text() if (directory/'view.log').is_file() else ''
        report['tests']['no_shutdown_rclerror'] = 'RCLError' not in view_log and 'Traceback' not in view_log
        if not report['tests']['no_shutdown_rclerror']:
            report['passed'] = False
            report['error'] = 'LiveView shutdown emitted an exception; inspect view.log'
        report['all_owned_children_exited'] = all(p.poll() is not None for _, p in children)
        report['elapsed_s'] = time.monotonic()-started
        (directory/'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print(json.dumps(dict(passed=report['passed'], report=str(directory/'report.json'),
                         tests=report['tests'], error=report.get('error')), ensure_ascii=False, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
