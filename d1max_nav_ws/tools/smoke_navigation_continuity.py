#!/usr/bin/env python3
"""Real ROS/EKF continuity smoke; synthetic inputs, private loopback Zenoh only.

Run with the project's ROS environment and overlay sourced. No SDK, driver,
frontend, mapping, command publisher or physical robot is involved. Production
navigation_parameters/localization.yaml are copied without relaxed limits.
"""
from collections import Counter, deque
import hashlib
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

import numpy as np
import yaml

WS = Path(__file__).resolve().parents[1]
PORT = 17467
PREFIX = '/d1max/localization/'
ACTIVE_SEC = 22.0
OUTAGE_SEC = 2.0
WARMUP_SEC = 3.0
HOLDS = [(6., 6.075), (11., 11.075), (16., 16.075)]


def diagonal(value=.01):
    return [value if i == j else 0. for i in range(6) for j in range(6)]


def timing(samples, begin, end):
    rows = [row for row in samples if begin <= row[0] < end]
    if len(rows) < 2:
        return {'count': len(rows)}
    arrival = np.asarray([r[0] for r in rows])
    stamp = np.asarray([r[1] for r in rows])
    gaps = np.diff(arrival)
    return dict(count=len(rows), received_hz=float((len(rows)-1)/(arrival[-1]-arrival[0])),
                timestamp_hz=float((len(rows)-1)/(stamp[-1]-stamp[0])),
                callback_gap_max_sec=float(gaps.max()),
                callback_gap_p95_sec=float(np.percentile(gaps, 95)),
                callback_gap_p99_sec=float(np.percentile(gaps, 99)),
                stamp_gap_max_sec=float(np.diff(stamp).max()),
                stamps_strictly_increasing=bool(np.all(np.diff(stamp) > 0)),
                x_strictly_increasing=bool(np.all(np.diff([r[2] for r in rows]) > 0)),
                max_position_error_m=max(r[3] for r in rows))


def main():
    with socket.socket() as check:
        check.bind(('127.0.0.1', PORT))  # refuse to share or kill an existing router
    directory = WS/'log/offline_navigation_continuity'/(
        time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8])
    directory.mkdir(parents=True)
    print('ARTIFACTS='+str(directory), flush=True)
    common = {'scouting': {'multicast': {'enabled': False}, 'gossip': {'enabled': False}},
              'timestamping': {'enabled': True, 'drop_future_timestamp': False}}
    (directory/'router.json5').write_text(json.dumps(dict(common, mode='router',
        listen={'endpoints': [f'tcp/127.0.0.1:{PORT}'], 'exit_on_failure': True},
        connect={'endpoints': []})))
    (directory/'client.json5').write_text(json.dumps(dict(common, mode='client',
        connect={'endpoints': [f'tcp/127.0.0.1:{PORT}'], 'exit_on_failure': True})))
    environment = dict(os.environ)
    for key in ('ZENOH_CONFIG_OVERRIDE', 'ZENOH_SESSION_CONFIG', 'ZENOH_ROUTER_CONFIG',
                'ROS_LOCALHOST_ONLY'):
        environment.pop(key, None)
        os.environ.pop(key, None)
    environment.update(RMW_IMPLEMENTATION='rmw_zenoh_cpp', ROS_DOMAIN_ID='224',
        ZENOH_SESSION_CONFIG_URI=str(directory/'client.json5'),
        ZENOH_ROUTER_CONFIG_URI=str(directory/'router.json5'),
        ROS_LOG_DIR=str(directory/'roslogs'), RUST_LOG='warn',
        PYTHONPATH=str(WS/'src/d1max_localization')+os.pathsep+environment.get('PYTHONPATH', ''))
    os.environ.update(environment)
    sys.path.insert(0, str(WS/'src/d1max_localization'))
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from ament_index_python.packages import get_package_prefix
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
    from d1max_localization.estimation.configuration import navigation_parameters
    from d1max_localization.initial_pose import body_to_tracking_transform
    from d1max_localization.math_utils import Pose3, compose, inverse

    config_path = WS/'src/d1max_localization/config/localization.yaml'
    config_bytes = config_path.read_bytes()
    config = yaml.safe_load(config_bytes)
    prediction, output, ekf = navigation_parameters(config)
    ekf = {**config['ekf_navigation']['ros__parameters'], **ekf}
    parameters = {'lio_predictor': {'ros__parameters': prediction},
                  'navigation_output': {'ros__parameters': output},
                  'ekf_navigation': {'ros__parameters': ekf}}
    params_path = directory/'production_navigation_parameters.yaml'
    params_path.write_text(yaml.safe_dump(parameters))
    body_inverse = inverse(body_to_tracking_transform(output['tracking_offset_body'], output['sdk_to_tracking_yaw']))
    children, streams, node = [], [], None
    result = dict(passed=False, kind='SYNTHETIC_ROS_CONTINUITY_NO_HARDWARE',
                  no_robot_connection=True, motion_control_enabled=False,
                  domain=224, port=PORT, config_path=str(config_path),
                  config_sha256=hashlib.sha256(config_bytes).hexdigest(),
                  active_sec=ACTIVE_SEC, outage_sec=OUTAGE_SEC, warmup_sec=WARMUP_SEC,
                  stimulus={'imu_hz': 200, 'lio_hz': 10, 'lio_delay_sec': .1,
                            'alignment_hz': 5, 'speed_mps': 1., 'yaw_rate_radps': .05,
                            'imu_delivery_holds_sec': HOLDS, 'samples_discarded': 0}, tests={})
    def spawn(name, command):
        stream = (directory/(name+'.log')).open('w'); streams.append(stream)
        process = subprocess.Popen(command, cwd=WS, env=environment, stdout=stream,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        children.append((name, process))
    try:
        spawn('router', [str(Path(get_package_prefix('rmw_zenoh_cpp'))/'lib/rmw_zenoh_cpp/rmw_zenohd')])
        deadline = time.monotonic()+3.
        while True:
            try:
                with socket.create_connection(('127.0.0.1', PORT), timeout=.1):
                    break
            except OSError:
                if time.monotonic() > deadline:
                    raise TimeoutError('isolated router did not start')
                time.sleep(.05)
        # Installed console shims can point at a stale workspace. Import current
        # source main directly, with real ROS args and actual installed EKF.
        for name in ('lio_predictor', 'navigation_output'):
            spawn(name, [sys.executable, '-c',
                f'from d1max_localization.{name} import main; main()',
                '--ros-args', '--params-file', str(params_path)])
        spawn('ekf_navigation', [str(Path(get_package_prefix('robot_localization'))/'lib/robot_localization/ekf_node'),
            '--ros-args', '-r', '__node:=ekf_navigation', '--params-file', str(params_path),
            '-r', 'odometry/filtered:='+PREFIX+'estimator/odometry_raw',
            '-r', 'set_pose:='+PREFIX+'estimator/set_pose'])
        rclpy.init(args=[])
        node = Node('TEST_ONLY_navigation_continuity', enable_rosout=False)
        imu_pub = node.create_publisher(Imu, PREFIX+'imu',
            QoSProfile(depth=128, reliability=ReliabilityPolicy.BEST_EFFORT))
        lio_pub = node.create_publisher(String, PREFIX+'lio/local_sample', 20)
        map_pub = node.create_publisher(String, PREFIX+'map_alignment', 10)
        seen = {'local': [], 'global': [], 'private_ekf': [], 'display_pose': []}
        statuses, predictions, tf_rows = [], [], []
        origin = time.time()+.2
        def truth(stamp):
            t = stamp-origin
            return Pose3((t, 0., 0.), (0., 0., math.sin(.025*t), math.cos(.025*t)))
        def odom_callback(kind, message):
            stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
            expected = truth(stamp)
            if kind != 'private_ekf':
                expected = compose(expected, body_inverse)
            p = message.pose.pose.position
            error = math.dist((p.x, p.y, p.z), expected.position)
            seen[kind].append((time.time(), stamp, p.x, error,
                               message.header.frame_id, message.child_frame_id))
        for kind, topic in [('local', 'odometry/local'), ('global', 'odometry/global'),
                            ('private_ekf', 'estimator/odometry_raw')]:
            node.create_subscription(Odometry, PREFIX+topic,
                                     lambda msg, kind=kind: odom_callback(kind, msg), 200)
        def display_callback(msg):
            stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            p = msg.pose.position
            seen['display_pose'].append((time.time(), stamp, p.x,
                math.dist((p.x, p.y, p.z), truth(stamp).position)))
        node.create_subscription(PoseStamped, PREFIX+'pose', display_callback, 100)
        node.create_subscription(String, PREFIX+'navigation/status',
            lambda msg: statuses.append((time.time(), json.loads(msg.data))), 100)
        node.create_subscription(String, PREFIX+'prediction/local_sample',
            lambda msg: predictions.append((time.time(), json.loads(msg.data))), 100)
        def tf_callback(msg):
            for item in msg.transforms:
                tf_rows.append((time.time(), item.header.stamp.sec+item.header.stamp.nanosec*1e-9,
                                item.header.frame_id, item.child_frame_id))
        node.create_subscription(TFMessage, '/tf', tf_callback, 200)
        deadline = time.monotonic()+4.
        while any(pub.get_subscription_count() < 1 for pub in (imu_pub, lio_pub, map_pub)):
            if time.monotonic() > deadline:
                raise TimeoutError('source subscribers not ready')
            rclpy.spin_once(node, timeout_sec=.01)
        origin = time.time()
        next_imu, next_lio, next_map = -40, 0, 0
        pending = deque()
        published = Counter()
        while time.time()-origin < ACTIVE_SEC+OUTAGE_SEC:
            now = time.time()
            elapsed = now-origin
            if any(process.poll() is not None for _, process in children):
                raise RuntimeError('owned ROS child exited prematurely')
            if elapsed < ACTIVE_SEC:
                while next_imu*.005 <= elapsed:
                    pending.append(origin+next_imu*.005)
                    next_imu += 1
                holding = any(start <= elapsed < end for start, end in HOLDS)
                if not holding:
                    while pending:
                        stamp = pending.popleft()
                        msg = Imu(); msg.header.frame_id = prediction['imu_frame']
                        ns = round(stamp*1e9)
                        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 10**9)
                        msg.linear_acceleration.z = 9.81; msg.angular_velocity.z = .05
                        imu_pub.publish(msg); published['imu'] += 1
                if next_lio*.1 <= elapsed:
                    stamp = origin+next_lio*.1-.1; pose = truth(stamp)
                    value = dict(schema=1, epoch=1, valid=True, fault=False,
                        received_at_unix=now, stamp_ns=str(round(stamp*1e9)),
                        frame=prediction['odom_frame'], child_frame=prediction['tracking_frame'],
                        position=pose.position, orientation=pose.orientation,
                        pose_covariance=diagonal(), twist_covariance=diagonal(),
                        inertial=dict(schema=1, world_velocity=[1.,0.,0.],
                            gyro_bias=[0.,0.,0.], accel_bias=[0.,0.,0.],
                            gravity=[0.,0.,-9.81], accel_scale=1.))
                    lio_pub.publish(String(data=json.dumps(value))); next_lio += 1
                    published['lio'] += 1
                if next_map*.2 <= elapsed:
                    stamp = origin+next_map*.2-.1; pose = truth(stamp)
                    value = dict(schema=1, epoch=1, valid=True, fault=False,
                        received_at_unix=now, stamp_ns=str(round(stamp*1e9)),
                        frame=output['map_frame'], child_frame=output['tracking_frame'],
                        seed_id='TEST_ONLY_SYNTHETIC_SEED', confirmations=3,
                        position=pose.position, orientation=pose.orientation, covariance=diagonal(),
                        anchor=dict(position=[0.,0.,0.], orientation=[0.,0.,0.,1.]))
                    map_pub.publish(String(data=json.dumps(value))); next_map += 1
                    published['map_alignment'] += 1
            rclpy.spin_once(node, timeout_sec=.0005)
        begin, stop = origin+WARMUP_SEC, origin+ACTIVE_SEC
        result['published'] = dict(published)
        result['outputs'] = {key: timing(rows, begin, stop) for key, rows in seen.items()}
        steady_status = [value for received, value in statuses if begin <= received < stop]
        steady_prediction = [value for received, value in predictions if begin <= received < stop]
        result['status'] = dict(count=len(steady_status),
            valid_fraction=sum(bool(v['valid']) for v in steady_status)/max(1,len(steady_status)),
            states=dict(Counter(v['state'] for v in steady_status)),
            qualities=dict(Counter(v.get('quality') for v in steady_status)),
            faults=dict(Counter(str(v['fault']) for v in steady_status if v['fault'])),
            last_active=steady_status[-1] if steady_status else {}, last=statuses[-1][1] if statuses else {})
        result['prediction'] = dict(count=len(steady_prediction),
            valid_fraction=sum(bool(v['valid']) for v in steady_prediction)/max(1,len(steady_prediction)),
            modes=dict(Counter(v.get('prediction_mode') for v in steady_prediction)),
            reasons=dict(Counter(v.get('reason') for v in steady_prediction)),
            faults=sum(bool(v['fault']) for v in steady_prediction),
            maximum_unsupported_sec=max((v.get('coast', {}).get('unsupported_sec', 0.) for v in steady_prediction), default=0.))
        result['delivery_windows'] = [{
            'hold_elapsed_sec': [start,end],
            'global': timing(seen['global'], origin+start-.1, origin+end+.2),
            'prediction_modes': dict(Counter(v.get('prediction_mode') for received,v in predictions
                if origin+start-.1 <= received < origin+end+.2)),
        } for start,end in HOLDS]
        global_stamps = {row[1] for row in seen['global']}
        local_stamps = {row[1] for row in seen['local']}
        tf_keys = [(row[1], row[2], row[3]) for row in tf_rows]
        edges = {(row[2],row[3]) for row in tf_rows}
        expected_edges = {(output['map_frame'],output['odom_frame']),
                          (output['odom_frame'],output['body_frame'])}
        tf_key_set = set(tf_keys)
        result['tf'] = dict(count=len(tf_rows), edges=sorted(edges),
            all_global_stamps_have_both_tf=all(all((stamp,*edge) in tf_key_set for edge in expected_edges) for stamp in global_stamps),
            unique_dynamic_samples=len(tf_keys)==len(set(tf_keys)),
            publisher_endpoints=sorted({p.node_name for p in node.get_publishers_info_by_topic('/tf')}),
            ekf_publish_tf_config=ekf['publish_tf'],
            authority_evidence='EKF publish_tf=false plus unique observed edges/stamps; '
                'Humble rclpy callback has no publisher GID. An advertised endpoint is not proof of publishing.')
        result['cutoff'] = dict(
            local_last_receipt_after_stop_sec=seen['local'][-1][0]-stop if seen['local'] else None,
            global_last_receipt_after_stop_sec=seen['global'][-1][0]-stop if seen['global'] else None,
            tf_last_receipt_after_stop_sec=tf_rows[-1][0]-stop if tf_rows else None,
            no_public_output_after_350ms=all(not any(r[0] > stop+.35 for r in seen[k]) for k in ('local','global')),
            no_tf_after_350ms=not any(r[0] > stop+.35 for r in tf_rows))
        topics = [name for name,_ in node.get_topic_names_and_types()]
        result['tests'] = dict(
            local_40_to_60hz=40 < result['outputs']['local'].get('received_hz', 0) < 60,
            global_40_to_60hz=40 < result['outputs']['global'].get('received_hz', 0) < 60,
            public_max_callback_gap_under_100ms=all(result['outputs'][k].get('callback_gap_max_sec',math.inf)<.1 for k in ('local','global')),
            strictly_monotonic_public_stamps_and_x=all(result['outputs'][k].get('stamps_strictly_increasing',False)
                and result['outputs'][k].get('x_strictly_increasing',False) for k in ('local','global')),
            continuous_valid_after_warmup=result['status']['valid_fraction']==1.,
            no_estimation_fault=not result['status']['faults'] and result['prediction']['faults']==0,
            coast_exercised=result['prediction']['modes'].get('coasting',0)>0,
            public_pose_error_under_10cm=all(result['outputs'][k].get('max_position_error_m',math.inf)<.1 for k in ('local','global')),
            same_public_local_global_stamps=bool(global_stamps) and global_stamps.issubset(local_stamps),
            correct_tf_edges=edges==expected_edges,
            unique_tf_and_common_stamps=result['tf']['unique_dynamic_samples'] and result['tf']['all_global_stamps_have_both_tf'],
            private_ekf_tf_disabled_and_unique_public_tf=ekf['publish_tf'] is False
                and 'navigation_output' in result['tf']['publisher_endpoints']
                and result['tf']['unique_dynamic_samples'],
            long_outage_stops_public=result['cutoff']['no_public_output_after_350ms'] and result['cutoff']['no_tf_after_350ms'],
            long_outage_invalid=bool(statuses) and not statuses[-1][1]['valid'],
            no_hardware_admission=all(not v['navigation_ready'] and not v['motion_control_enabled'] for _,v in statuses),
            no_command_topics=not any('cmd_vel' in name or 'navigate_to_pose' in name for name in topics))
        result['passed'] = all(result['tests'].values())
        (directory/'navigation_status.jsonl').write_text(''.join(json.dumps(dict(arrival=t, **v))+'\n' for t,v in statuses))
        (directory/'prediction_status.jsonl').write_text(''.join(json.dumps(dict(arrival=t, **v))+'\n' for t,v in predictions))
        (directory/'output_samples.json').write_text(json.dumps(seen))
    except BaseException as error:
        result['error'] = type(error).__name__+': '+str(error)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()
        for _, process in reversed(children):
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        for _, process in reversed(children):
            try:
                process.wait(timeout=3.)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL); process.wait(timeout=1.)
        for stream in streams:
            stream.close()
        result['owned_processes'] = {name: dict(pid=p.pid, returncode=p.returncode) for name,p in children}
        result['all_owned_children_exited'] = all(p.poll() is not None for _,p in children)
        result['clean_shutdown_no_traceback'] = all(
            not any(token in (directory/(name+'.log')).read_text() for token in ('Traceback','RCLError'))
            for name,_ in children)
        with socket.socket() as check:
            result['port_closed'] = check.connect_ex(('127.0.0.1', PORT)) != 0
        result['passed'] &= result['all_owned_children_exited'] and result['clean_shutdown_no_traceback'] and result['port_closed']
        (directory/'report.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
    print(json.dumps({**result, 'report': str(directory/'report.json')}, ensure_ascii=False, indent=2), flush=True)
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
