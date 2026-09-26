"""Offline integration: real Zenoh + ROS messages + rosbag, loopback only.

Run from map_manager with the D1 ROS environment sourced. No robot, SDK, mapping
or production bag is touched. Every recording uses a new mkdtemp directory.
"""
import json
import os
import signal
import socket
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import yaml

APP = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP))
from backend.bags.common import atomic_json
from backend.bags.runtime import BagRecorder
from backend.bags.library import BagLibrary
from backend.bags.worker import stop_child


def publish(directory):
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from rosidl_runtime_py.utilities import get_message
    from sensor_msgs.msg import PointField
    import struct
    config = yaml.safe_load((directory / 'recording.yaml').read_text())
    rclpy.init()
    node = rclpy.create_node('d1max_bag_offline_fixture')
    streams = []
    for item in config['topics']:
        cls = get_message(item['type'])
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL if item.get('durability') == 'transient_local' else DurabilityPolicy.VOLATILE)
        streams.append((node.create_publisher(cls, item['name'], qos), cls))
    def send():
        for publisher, cls in streams:
            msg = cls()
            if hasattr(msg, 'header'):
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.header.frame_id = 'offline_test_sensor'
            if hasattr(msg, 'point_step'):
                msg.height = 1; msg.width = 1; msg.point_step = 12; msg.row_step = 12
                msg.fields = [PointField(name=name, offset=i*4, datatype=7, count=1) for i, name in enumerate(('x', 'y', 'z'))]
                msg.data = struct.pack('<fff', 1, 2, 3)
            publisher.publish(msg)
    node.create_timer(.02, send)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


def until(predicate, timeout=15):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(.1)
    raise AssertionError('Offline integration timed out')


def main():
    project = APP.parents[1]
    root = Path(tempfile.mkdtemp(prefix='d1max-bag-offline-'))
    print('Fixtures:', root, flush=True)
    # Select an available loopback port; multicast and gossip disabled everywhere.
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0)); port = sock.getsockname()[1]
    endpoint = f'tcp/127.0.0.1:{port}'
    router_config = root / 'router.json5'
    atomic_json(router_config, {'mode': 'router', 'listen': {'endpoints': [endpoint]},
        'scouting': {'multicast': {'enabled': False}, 'gossip': {'enabled': False}}})
    client_config = root / 'client.json5'
    atomic_json(client_config, {'mode': 'client', 'connect': {'endpoints': [endpoint], 'exit_on_failure': True},
        'scouting': {'multicast': {'enabled': False}, 'gossip': {'enabled': False}},
        'timestamping': {'enabled': True, 'drop_future_timestamp': False}})
    environment = {**os.environ, 'ROS_DOMAIN_ID': '184', 'RMW_IMPLEMENTATION': 'rmw_zenoh_cpp',
                   'ZENOH_SESSION_CONFIG_URI': str(client_config), 'ZENOH_ROUTER_CONFIG_URI': str(router_config),
                   'ROS_LOG_DIR': str(root / 'ros_logs')}
    router_bin = project / 'd1max_ros2/local/opt/ros/humble/lib/rmw_zenoh_cpp/rmw_zenohd'
    router_log = (root / 'router.log').open('w')
    router = subprocess.Popen([str(router_bin)], env=environment, stdout=router_log, stderr=subprocess.STDOUT)
    publisher, recorder = None, None
    checks = []
    try:
        def router_ready():
            try:
                with socket.create_connection(('127.0.0.1', port), timeout=.2): return True
            except OSError: return False
        until(router_ready, 5)
        config = yaml.safe_load((APP / 'config/recording.yaml').read_text())
        config['transport'].update(endpoint=endpoint, domain_id=184)
        config['timing'].update(prepare_timeout_sec=5, sensor_timeout_sec=3, finalize_timeout_sec=5)
        config['storage'].update(cache_size_mib=1, split_size_mib=10)
        (root / 'recording.yaml').write_text(yaml.safe_dump(config))
        recorder = BagRecorder(APP, project, Path('/home/dndx/d1max_nav_ws'), root / 'runtime', root / 'bags')
        recorder.config = lambda: config
        library = BagLibrary([root / 'bags'], root / 'catalog', recorder.active_path)

        # Start with no publishers: must fail, never create a fake successful bag.
        recorder.start('QA disconnected', ['core'])
        until(lambda: not recorder.snapshot()['busy'], 12)
        assert recorder.snapshot()['status'] == 'failed', recorder.snapshot()
        assert not library.list()
        checks.append('no sensors -> bounded failure, no empty successful bag')

        publisher_log = (root / 'publisher.log').open('w')
        publisher = subprocess.Popen([sys.executable, str(Path(__file__)), '--publish', str(root)],
            env=environment, stdout=publisher_log, stderr=subprocess.STDOUT)
        recorder.start('QA complete', ['core', 'motion', 'cameras'])
        until(lambda: recorder.snapshot()['status'] == 'recording', 15)
        try:
            recorder.start('duplicate', ['core'])
            raise AssertionError('duplicate accepted')
        except RuntimeError:
            pass
        time.sleep(1.5)
        recorder.stop(); recorder.stop()
        until(lambda: not recorder.snapshot()['busy'], 12)
        assert recorder.snapshot()['status'] == 'complete', recorder.snapshot()
        saved = library.list()[0]
        assert saved['message_count'] > 0, saved
        details = library.detail(saved['id'])
        assert all(t['count'] > 0 for t in details['topics']), details
        checks.append('real ROS/Zenoh -> sqlite bag, every selected stream counted, duplicate rejected, repeated stop safe')

        # Web process recreation: recover the same worker, do not create another.
        recorder.start('QA recover close', ['core'])
        until(lambda: recorder.snapshot()['status'] == 'recording', 15)
        replacement = BagRecorder(APP, project, Path('/home/dndx/d1max_nav_ws'), root / 'runtime', root / 'bags')
        replacement.config = lambda: config
        replacement.recover()
        assert replacement.snapshot()['busy']
        replacement.close()
        assert not recorder.snapshot()['busy']
        assert recorder.snapshot()['status'] == 'complete', recorder.snapshot()
        checks.append('Web recreation adopts verified worker; close flushes and reaps recording/probe')

        recorder.start('QA disconnect during recording', ['core'])
        until(lambda: recorder.snapshot()['status'] == 'recording', 15)
        stop_child(publisher); publisher = None
        until(lambda: not recorder.snapshot()['busy'], 12)
        assert recorder.snapshot()['status'] == 'incomplete', recorder.snapshot()
        assert '断流' in recorder.snapshot()['error']
        checks.append('sensor loss -> automatic graceful stop, retained data explicitly incomplete')

        recorder.start('QA cancel waiting', ['core'])
        recorder.stop()
        until(lambda: not recorder.snapshot()['busy'], 8)
        assert recorder.snapshot()['status'] == 'cancelled', recorder.snapshot()
        checks.append('cancel during preparation leaves no orphan')
        atomic_json(root / 'report.json', {'checks': checks, 'success': True, 'root': str(root)})
        for check in checks: print('PASS', check, flush=True)
    finally:
        if recorder: recorder.close()
        stop_child(publisher)
        stop_child(router)
        router_log.close()


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == '--publish':
        publish(Path(sys.argv[2]))
    else:
        main()
