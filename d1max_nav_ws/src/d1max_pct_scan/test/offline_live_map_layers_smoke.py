#!/usr/bin/env python3
"""Real static terrain publisher on isolated loopback Zenoh; no robot nodes."""
import json
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
from d1max_pct_planner.paths import expand_tree
from offline_live_chain_smoke import WS, FRAME, PREFIX, PORT, isolated_environment


def main():
    directory = WS/'log/offline_live_map_layers_smoke'/(
        time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8])
    directory.mkdir(parents=True)
    session = expand_tree(yaml.safe_load((WS/'src/d1max_pct_scan/config/live_visualization.yaml').read_text()))
    session.update(id='TEST_ONLY_'+uuid.uuid4().hex[:12], mode='OFFLINE_STATIC_DISPLAY_TEST_ONLY')
    (directory/'session.json').write_text(json.dumps(session))
    common = {'scouting': {'multicast': {'enabled': False}, 'gossip': {'enabled': False}},
              'timestamping': {'enabled': True, 'drop_future_timestamp': False}}
    (directory/'client.json5').write_text(json.dumps(dict(common, mode='client',
        connect={'endpoints': [f'tcp/127.0.0.1:{PORT}'], 'exit_on_failure': True})))
    (directory/'router.json5').write_text(json.dumps(dict(common, mode='router',
        listen={'endpoints': [f'tcp/127.0.0.1:{PORT}'], 'exit_on_failure': True}, connect={'endpoints': []})))
    with socket.socket() as check:
        check.bind(('127.0.0.1', PORT))
    environment = isolated_environment(directory)
    os.environ.update(environment)
    for key in ('ZENOH_CONFIG_OVERRIDE', 'ZENOH_SESSION_CONFIG', 'ZENOH_ROUTER_CONFIG'):
        os.environ.pop(key, None)
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from ament_index_python.packages import get_package_prefix
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import String
    children, streams, node = [], [], None
    result = {'passed': False, 'tests': {}, 'domain': 224, 'port': PORT,
              'kind': 'OFFLINE_STATIC_DISPLAY_TEST_ONLY', 'no_robot_connection': True}
    def spawn(name, arguments):
        stream = (directory/(name+'.log')).open('w'); streams.append(stream)
        child = subprocess.Popen(arguments, cwd=WS, env=environment, stdout=stream,
                                 stderr=subprocess.STDOUT, start_new_session=True)
        children.append((name, child))
    try:
        spawn('router', [str(Path(get_package_prefix('rmw_zenoh_cpp'))/'lib/rmw_zenoh_cpp/rmw_zenohd')])
        deadline = time.monotonic()+2.
        while True:
            try:
                with socket.create_connection(('127.0.0.1', PORT), timeout=.1):
                    break
            except OSError:
                if time.monotonic() > deadline:
                    raise TimeoutError('Private router startup')
                time.sleep(.05)
        rclpy.init(args=[])
        node = Node('TEST_ONLY_static_terrain_probe', enable_rosout=False)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        received = {}
        for name in ('traversable_surface', 'blocked_surface'):
            node.create_subscription(PointCloud2, PREFIX+name,
                lambda msg, name=name: received.update({name: msg}), qos)
        node.create_subscription(String, PREFIX+'map_layer_status',
            lambda msg: received.update(metadata=json.loads(msg.data)), qos)
        spawn('map_layers', [sys.executable, '-m', 'd1max_pct_scan.live_map_layers', '--session', str(directory)])
        deadline = time.monotonic()+40.
        while len(received) < 3:
            if any(child.poll() is not None for _, child in children):
                raise RuntimeError('Static terrain child exited before publishing')
            if time.monotonic() > deadline:
                raise TimeoutError('Static terrain publish')
            rclpy.spin_once(node, timeout_sec=.05)
        metadata = received['metadata']
        assert metadata['session_id'] == session['id'] and metadata['frame_id'] == FRAME
        assert metadata['motion_enabled'] is False and metadata['estimated_inverse_not_tf']
        result['tests']['actual_node_initializes_and_publishes_metadata'] = True
        arrays = {}
        for name, count_key in [('traversable_surface', 'traversable_cells'), ('blocked_surface', 'blocked_cells')]:
            msg = received[name]
            assert msg.header.frame_id == FRAME and msg.width == metadata[count_key] and msg.width > 1000
            assert msg.point_step == 16 and msg.height == 1 and len(msg.data) == msg.width*16
            records = np.frombuffer(msg.data, dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('rgb','<u4')])
            assert np.isfinite(np.column_stack([records[key] for key in ('x','y','z')])).all()
            arrays[name] = records
        assert len(np.unique(arrays['traversable_surface']['rgb'])) > 2
        assert np.all(arrays['blocked_surface']['rgb'] == (220 << 16) | (68 << 8) | 68)
        assert np.any(arrays['traversable_surface']['z'] < 0) and np.any(arrays['traversable_surface']['z'] > 3.)
        assert not node.get_publishers_info_by_topic('/cmd_vel')
        result['tests'].update(real_dual_floor_xyz_rgb_published=True, true_cost_and_blocked_colors=True,
                               no_cmd_vel_publisher=True)
        result['metadata'] = metadata
        result['passed'] = True
    except BaseException as error:
        result['error'] = type(error).__name__+': '+str(error)
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
        for _, child in reversed(children):
            try:
                child.wait(timeout=3.)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL); child.wait(timeout=1.)
        for stream in streams:
            stream.close()
        result['all_owned_children_exited'] = all(child.poll() is not None for _, child in children)
        log = (directory/'map_layers.log').read_text() if (directory/'map_layers.log').is_file() else ''
        result['tests']['clean_shutdown_no_traceback'] = 'Traceback' not in log and 'RCLError' not in log
        result['passed'] &= result['all_owned_children_exited'] and result['tests']['clean_shutdown_no_traceback']
        (directory/'report.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
    print(json.dumps({**result, 'report': str(directory/'report.json')}, ensure_ascii=False, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
