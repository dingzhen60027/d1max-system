#!/usr/bin/env python3
"""Bounded real-bag sensor -> production LIO capture; no actuation or PCD matching.

Importing this module does not initialize ROS or start a process. --prepare-only
checks inputs and writes a unique audit directory without starting ROS. Full
capture owns private Zenoh 226/17469 and only router/adapter/LIO/bag processes.
"""
from __future__ import annotations

import argparse
from collections import OrderedDict
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import signal
import socket
import subprocess
import threading
import time

import numpy as np
import yaml

WS = Path('/home/dndx/d1max_nav_ws')
DEFAULT_BAG = WS / 'bags/slam_raw_20260923_010248_eb79d8'
DEFAULT_CONFIG = WS / 'src/d1max_localization/config/localization.yaml'
DOMAIN = '226'
PORT = 17469
ENDPOINT = f'tcp/127.0.0.1:{PORT}'
TOPICS = ['/front_lidar', '/rear_lidar', '/front_lidar/imu']
PREFIX = '/d1max/localization/'


def sha256(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as source:
        for block in iter(lambda: source.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def json_value(value):
    """Normalize ROS/Python scalar storage without inventing measurement data.

    Humble exposes some uint8 fields as one-byte bytes rather than int. Reject
    arbitrary binary payloads: the JSON logs must never accidentally serialize
    a full point cloud or turn a numeric diagnostic into a string.
    """
    if isinstance(value, (bytes, bytearray)):
        if len(value) != 1:
            raise ValueError('Only one-byte ROS uint8 values are allowed in JSON')
        return value[0]
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, np.ndarray):
        return json_value(value.tolist())
    if isinstance(value, dict):
        return {str(key): json_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_value(item) for item in value]
    return value


def save_json(path, value):
    temp = path.with_suffix(path.suffix + '.tmp')
    temp.write_text(json.dumps(json_value(value), ensure_ascii=False, indent=2, allow_nan=False))
    temp.replace(path)


def transport_config(mode):
    return {'mode': mode,
            'connect': {'endpoints': [ENDPOINT] if mode == 'client' else []},
            'listen': {'endpoints': [ENDPOINT] if mode == 'router' else []},
            'scouting': {'multicast': {'enabled': False}, 'gossip': {'enabled': False}}}


def validate_isolation(env):
    if env.get('RMW_IMPLEMENTATION') != 'rmw_zenoh_cpp' or env.get('ROS_DOMAIN_ID') != DOMAIN:
        raise ValueError('Requires private Zenoh domain 226')
    for key in ('ZENOH_CONFIG_OVERRIDE', 'ZENOH_SESSION_CONFIG', 'ZENOH_ROUTER_CONFIG'):
        if env.get(key):
            raise ValueError(f'Unexpected transport override: {key}')
    for kind in ('client', 'router'):
        key = 'ZENOH_SESSION_CONFIG_URI' if kind == 'client' else 'ZENOH_ROUTER_CONFIG_URI'
        actual = json.loads(Path(env[key]).read_text())
        if actual != transport_config(kind):
            raise ValueError('Nonisolated transport configuration rejected')


def stamp_ns(msg):
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


def vector3(value):
    return [float(value.x), float(value.y), float(value.z)]


def odom_record(msg):
    p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
    return {'stamp_ns': stamp_ns(msg), 'frame': msg.header.frame_id,
            'child_frame': msg.child_frame_id, 'position': vector3(p),
            'orientation': [float(q.x), float(q.y), float(q.z), float(q.w)],
            'linear_child_frame': vector3(v.linear), 'angular_child_frame': vector3(v.angular),
            'pose_covariance': list(msg.pose.covariance), 'twist_covariance': list(msg.twist.covariance)}


def cloud_arrays(msg, max_points):
    """Read actual XYZ(I) respecting PointCloud2 row padding and endianness.

    Deterministic record-stride sampling, no voxel filling or ray generation.
    The sample index and all count reductions are retained for auditing.
    """
    if max_points < 1 or int(msg.width) * int(msg.height) > 2_000_000:
        raise ValueError('Invalid point limit or unexpectedly huge cloud')
    byteorder = '>' if msg.is_bigendian else '<'
    formats = {7: byteorder + 'f4', 8: byteorder + 'f8'}
    fields = {f.name: f for f in msg.fields}
    names = ['x', 'y', 'z'] + (['intensity'] if 'intensity' in fields else [])
    for name in names:
        f = fields.get(name)
        if (f is None or f.count != 1 or f.datatype not in formats
                or f.offset + np.dtype(formats[f.datatype]).itemsize > msg.point_step):
            raise ValueError('Unsupported XYZ(I) schema')
    if msg.row_step < msg.width * msg.point_step or len(msg.data) < msg.row_step * msg.height:
        raise ValueError('Malformed PointCloud2 data/stride')
    dtype = np.dtype({'names': names, 'formats': [formats[fields[n].datatype] for n in names],
                      'offsets': [fields[n].offset for n in names], 'itemsize': msg.point_step})
    rows = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                      strides=(msg.row_step, msg.point_step))
    values = np.column_stack([rows[n].ravel() for n in names])
    finite = np.flatnonzero(np.all(np.isfinite(values[:, :3]), axis=1))
    selected = finite[::max(1, math.ceil(len(finite) / max_points))]
    result = {'xyz': np.asarray(values[selected, :3], dtype=np.float32),
              'original_point_index': selected.astype(np.uint32)}
    if len(names) == 4:
        result['intensity'] = np.asarray(values[selected, 3], dtype=np.float32)
    return result, {'original_points': int(msg.width * msg.height), 'finite_xyz_points': len(finite),
                    'retained_points': len(selected), 'fields': names,
                    'sampling': 'deterministic_stride_of_finite_original_records'}


class CaptureWriter:
    """One background worker; no compression or file I/O in ROS callbacks."""
    def __init__(self, out, max_points, max_bytes):
        self.out, self.max_points, self.max_bytes = out, max_points, max_bytes
        self.queue = queue.Queue(maxsize=4096)
        self.cloud_slots = threading.BoundedSemaphore(8)
        self.error = None
        self.saved_clouds = 0
        self.bytes = 0
        self.thread = threading.Thread(target=self.work, name='capture-file-writer', daemon=True)
        (out / 'clouds').mkdir()
        self.thread.start()

    def submit(self, kind, value):
        if self.error:
            raise RuntimeError(self.error)
        cloud_slot = kind == 'sample'
        if cloud_slot and not self.cloud_slots.acquire(blocking=False):
            self.error = 'capture_cloud_writer_backlog; stopping instead of silently dropping'
            raise RuntimeError(self.error)
        try:
            self.queue.put_nowait((kind, value))
        except queue.Full:
            if cloud_slot:
                self.cloud_slots.release()
            self.error = 'capture_writer_queue_full; acquisition not silently dropped'
            raise RuntimeError(self.error)

    def work(self):
        streams = {}
        try:
            while True:
                item = self.queue.get()
                try:
                    if item is None:
                        return
                    kind, value = item
                    if kind == 'sample':
                        cloud, odom, state = value
                        arrays, details = cloud_arrays(cloud, self.max_points)
                        rel = f'clouds/{self.saved_clouds:05d}_{stamp_ns(cloud)}.npz'
                        target = self.out / rel
                        np.savez_compressed(target, **arrays, stamp_ns=np.int64(stamp_ns(cloud)))
                        self.bytes += target.stat().st_size
                        value = {'cloud_file': rel, 'cloud_sha256': sha256(target),
                                 'stamp_ns': stamp_ns(cloud), 'frame': cloud.header.frame_id,
                                 'cloud': details, 'odometry': odom, 'local_sample': state,
                                 'pairing': 'exact_same_scan_end_nanoseconds_no_interpolation',
                                 'acceleration': None,
                                 'acceleration_note': 'Not published in native odometry; retain real adapted IMU and inertial posterior separately, never zero-fill.'}
                        self.saved_clouds += 1
                    if kind not in streams:
                        streams[kind] = (self.out / (kind + '.jsonl')).open('x')
                    text = json.dumps(json_value(value), ensure_ascii=False, allow_nan=False) + '\n'
                    streams[kind].write(text)
                    self.bytes += len(text.encode('utf-8'))
                    if self.bytes > self.max_bytes:
                        raise RuntimeError('capture_size_limit_reached')
                finally:
                    if item is not None and item[0] == 'sample':
                        self.cloud_slots.release()
                    self.queue.task_done()
        except Exception as error:
            self.error = f'{type(error).__name__}: {error}'
        finally:
            for stream in streams.values():
                stream.close()

    def close(self):
        # Bounded shutdown even when the worker failed. No queue.join deadlock.
        if self.thread.is_alive():
            self.queue.put(None, timeout=2.)
        self.thread.join(timeout=15.)
        if self.thread.is_alive():
            raise RuntimeError('capture_writer_did_not_finish')
        if self.error:
            raise RuntimeError(self.error)


class ExactPairs:
    def __init__(self, writer, period_ns):
        self.writer, self.period_ns = writer, period_ns
        self.odom, self.state, self.cloud = OrderedDict(), OrderedDict(), OrderedDict()
        self.last_selected = None
        self.counts = {'odom': 0, 'state': 0, 'cloud': 0, 'selected_cloud': 0,
                       'paired': 0, 'unmatched_evicted': 0, 'invalid_state': 0, 'stamp_regression': 0}

    def add(self, kind, stamp, value):
        self.counts[kind] += 1
        if kind == 'cloud':
            if self.last_selected is not None:
                if stamp < self.last_selected:
                    self.counts['stamp_regression'] += 1
                    return
                if stamp - self.last_selected < self.period_ns:
                    return
            self.last_selected = stamp
            self.counts['selected_cloud'] += 1
        target = getattr(self, kind)
        target[stamp] = value
        while len(target) > (8 if kind == 'cloud' else 128):
            target.popitem(last=False)
            if kind == 'cloud':
                self.counts['unmatched_evicted'] += 1
        for key in list(self.cloud):
            if key not in self.odom or key not in self.state:
                continue
            cloud, odom, state = self.cloud.pop(key), self.odom[key], self.state[key]
            if not state.get('valid') or state.get('fault'):
                self.counts['invalid_state'] += 1
                continue
            if (cloud.header.frame_id != odom['child_frame']
                    or state['frame'] != odom['frame'] or state['child_frame'] != odom['child_frame']):
                raise ValueError('Cloud/odometry/local_sample coordinate contract mismatch')
            self.writer.submit('sample', (cloud, odom, state))
            self.counts['paired'] += 1


def prepare(args):
    bag, config, out = args.bag.resolve(), args.config.resolve(), args.output.resolve()
    if not 5 <= args.duration <= 60 or not 0.5 <= args.cloud_hz <= 10:
        raise ValueError('Require duration 5..60 s and cloud sampling 0.5..10 Hz')
    if not 1000 <= args.max_points <= 100000:
        raise ValueError('Require max_points 1000..100000')
    if out.exists() or out.parent != WS / 'log/scan_bag_replay':
        raise ValueError('Require NEW unique exact directory under /home/dndx/d1max_nav_ws/log/scan_bag_replay')
    if os.environ.get('RMW_IMPLEMENTATION') != 'rmw_zenoh_cpp':
        raise ValueError('Source the D1 ROS environment first; middleware must be Zenoh')
    for key in ('ZENOH_CONFIG_OVERRIDE', 'ZENOH_SESSION_CONFIG', 'ZENOH_ROUTER_CONFIG'):
        if os.environ.get(key):
            raise ValueError(f'Unexpected transport override: {key}')
    metadata = yaml.safe_load((bag / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    available = {x['topic_metadata']['name']: x['topic_metadata']['type'] for x in metadata['topics_with_message_count']}
    expected = {'/front_lidar': 'sensor_msgs/msg/PointCloud2', '/rear_lidar': 'sensor_msgs/msg/PointCloud2',
                '/front_lidar/imu': 'sensor_msgs/msg/Imu'}
    if any(available.get(t) != typ for t, typ in expected.items()):
        raise ValueError('Required genuine sensor topics absent/wrong type')
    source = yaml.safe_load(config.read_text())
    lio = source['/d1max/localization/lio/laserMapping']['ros__parameters']
    adapter = source['dual_lidar_adapter']['ros__parameters']
    if (lio['publish.tf_enabled'] or lio['pcd_save.pcd_save_en'] or not lio['localization.enabled']
            or adapter['input_clock_mode'] != 'estimate_shared_epoch'
            or adapter['input_imu_topic'] != '/front_lidar/imu'
            or adapter['front_topic'] != '/front_lidar' or adapter['rear_topic'] != '/rear_lidar'):
        raise ValueError('Production config is not the expected no-TF/no-map-save front-IMU LIO contract')
    with socket.socket() as probe:
        if probe.connect_ex(('127.0.0.1', PORT)) == 0:
            raise ValueError('Private replay port already in use; refusing reuse')
    out.mkdir(parents=True)
    (out / 'localization.yaml').write_bytes(config.read_bytes())
    for kind in ('router', 'client'):
        save_json(out / (kind + '.json5'), transport_config(kind))
    env = dict(os.environ, RMW_IMPLEMENTATION='rmw_zenoh_cpp', ROS_DOMAIN_ID=DOMAIN,
               ZENOH_SESSION_CONFIG_URI=str(out / 'client.json5'),
               ZENOH_ROUTER_CONFIG_URI=str(out / 'router.json5'), ROS_LOG_DIR=str(out / 'ros_logs'))
    env.pop('ROS_LOCALHOST_ONLY', None)
    validate_isolation(env)
    db_stats = []
    for rel in metadata['relative_file_paths']:
        path = (bag / rel).resolve()
        if path.parent != bag or not path.is_file():
            raise ValueError('Invalid bag storage path')
        st = path.stat()
        db_stats.append({'file': rel, 'size': st.st_size, 'mtime_ns': st.st_mtime_ns})
    manifest = {'schema': 1, 'mode': 'REAL_BAG_PRODUCTION_FRONTEND_CAPTURE_NO_MOTION',
                'bag': str(bag), 'metadata_sha256': sha256(bag / 'metadata.yaml'), 'database_stats': db_stats,
                'production_config': str(config), 'production_config_sha256': sha256(config),
                'capture_script_sha256': sha256(Path(__file__)), 'topics': TOPICS,
                'duration_limit_seconds': args.duration, 'playback_rate': 1.0,
                'cloud_sampling_hz': args.cloud_hz, 'max_points_per_cloud': args.max_points,
                'max_output_data_bytes': 512 * 1024 * 1024, 'domain': DOMAIN, 'endpoint': ENDPOINT,
                'records_are_ground_truth': False, 'localization_map_matching_included': False,
                'planner_or_controller_included': False, 'robot_commands_sent': False,
                'clock_policy': 'No /clock; production single shared epoch shift. Original sensor headers logged separately; intervals retained.',
                'imu_source': 'front_lidar_imu_PRODUCTION_not_central_IMU_mapping_experiment',
                'velocity_policy': 'Native child-frame odometry twist and world_velocity posterior preserved, never zeroed.',
                'cloud_policy': 'Deskewed combined dual lidar in tracking frame; no per-return original lidar origin retained. Do not call single-origin rays exact dual-LiDAR free-space evidence.'}
    save_json(out / 'manifest.json', manifest)
    return out, env, manifest


def capture(args):
    out, env, manifest = prepare(args)
    print('ARTIFACTS=' + str(out), flush=True)
    if args.prepare_only:
        print('Prepared only; no ROS processes launched.', flush=True)
        return 0
    os.environ.update(env)
    validate_isolation(os.environ)
    from ament_index_python.packages import get_package_prefix
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from diagnostic_msgs.msg import DiagnosticArray
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu, PointCloud2
    from std_msgs.msg import String

    children, streams = {}, []
    stopped = threading.Event()
    old_handlers = {s: signal.signal(s, lambda *_: stopped.set()) for s in (signal.SIGINT, signal.SIGTERM)}
    node = writer = None
    result = {'passed': False, 'robot_commands_sent': False, 'reason': 'starting'}
    def executable(package, name):
        path = Path(get_package_prefix(package)) / 'lib' / package / name
        manifest.setdefault('binaries', {})[str(path)] = sha256(path)
        return str(path)
    def start(name, argv):
        stream = (out / (name + '.log')).open('x')
        streams.append(stream)
        child = subprocess.Popen(argv, cwd=out, env=env, stdout=stream,
                                 stderr=subprocess.STDOUT, start_new_session=True)
        children[name] = child
        save_json(out / 'processes.json', {k: {'pid': p.pid, 'pgid': p.pid, 'argv': p.args} for k, p in children.items()})
        return child
    def spin_for(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline and not stopped.is_set():
            if node is None:
                time.sleep(.05)
            else:
                rclpy.spin_once(node, timeout_sec=.02)
            if writer and writer.error:
                raise RuntimeError(writer.error)
            for name, child in children.items():
                if child.poll() is not None:
                    raise RuntimeError(f'{name} exited prematurely: {child.returncode}')
    try:
        start('router', [executable('rmw_zenoh_cpp', 'rmw_zenohd')])
        spin_for(.7)
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = rclpy.create_node('isolated_real_bag_capture')
        writer = CaptureWriter(out, args.max_points, manifest['max_output_data_bytes'])
        pairs = ExactPairs(writer, round(1e9 / args.cloud_hz))
        counts, epochs = {}, set()
        qos = QoSProfile(depth=256, reliability=ReliabilityPolicy.BEST_EFFORT)
        def odometry(msg):
            record = odom_record(msg)
            writer.submit('odometry', record)
            pairs.add('odom', record['stamp_ns'], record)
        def sample(msg):
            record = json.loads(msg.data)
            writer.submit('local_sample', record)
            epochs.add(record.get('epoch'))
            if record.get('valid') and 'stamp_ns' in record:
                pairs.add('state', int(record['stamp_ns']), record)
        def cloud(msg):
            pairs.add('cloud', stamp_ns(msg), msg)
        def imu(msg):
            writer.submit('adapted_imu', {'stamp_ns': stamp_ns(msg), 'frame': msg.header.frame_id,
                          'acceleration': vector3(msg.linear_acceleration),
                          'angular_velocity': vector3(msg.angular_velocity),
                          'acceleration_covariance': list(msg.linear_acceleration_covariance),
                          'angular_velocity_covariance': list(msg.angular_velocity_covariance)})
        def raw_header(topic, msg):
            counts[topic] = counts.get(topic, 0) + 1
            writer.submit('raw_sensor_headers', {'topic': topic, 'stamp_ns': stamp_ns(msg),
                          'frame': msg.header.frame_id, 'received_monotonic_ns': time.monotonic_ns()})
        def status(msg):
            writer.submit('status', json.loads(msg.data))
        def diagnostics(msg):
            for item in msg.status:
                if item.name == 'd1max_localization/input_clock':
                    writer.submit('input_clock', {'level': item.level, 'message': item.message,
                                  'values': {v.key: v.value for v in item.values}})
        subscriptions = [
            node.create_subscription(Odometry, PREFIX + 'lio/odometry', odometry, qos),
            node.create_subscription(String, PREFIX + 'lio/local_sample', sample, qos),
            node.create_subscription(PointCloud2, PREFIX + 'lio/deskewed', cloud, QoSProfile(depth=8, reliability=ReliabilityPolicy.BEST_EFFORT)),
            node.create_subscription(Imu, PREFIX + 'imu', imu, qos),
            node.create_subscription(String, PREFIX + 'lio/status', status, qos),
            node.create_subscription(DiagnosticArray, '/diagnostics', diagnostics, qos),
        ]
        for topic in TOPICS:
            typ = Imu if topic.endswith('/imu') else PointCloud2
            raw_qos = qos if typ == Imu else QoSProfile(depth=8, reliability=ReliabilityPolicy.BEST_EFFORT)
            subscriptions.append(node.create_subscription(typ, topic, lambda m, t=topic: raw_header(t, m), raw_qos))
        params = str(out / 'localization.yaml')
        start('adapter', [executable('d1max_localization', 'dual_lidar_adapter'), '--ros-args', '--params-file', params])
        start('lio', [executable('faster_lio', 'run_mapping_online'), '--ros-args', '--params-file', params,
              '-r', '__ns:=/d1max/localization/lio', '-r', '__node:=laserMapping',
              '-r', 'Odometry:=odometry', '-r', 'cloud_registered_body:=deskewed'])
        save_json(out / 'manifest.json', manifest)
        spin_for(1.5)
        player = start('player', ['ros2', 'bag', 'play', str(args.bag.resolve()), '--rate', '1.0',
               '--disable-keyboard-controls', '--read-ahead-queue-size', '1000', '--topics', *TOPICS])
        begin = time.monotonic()
        spin_for(args.duration)
        result.update(reason='operator_stop' if stopped.is_set() else 'duration_limit',
                      capture_elapsed_seconds=time.monotonic() - begin, raw_received=counts,
                      pair_counts=dict(pairs.counts), local_epochs=sorted(x for x in epochs if x is not None),
                      unmatched_selected_clouds=len(pairs.cloud),
                      passed=(not stopped.is_set() and pairs.counts['paired'] >= 5 and all(counts.get(t, 0) > 0 for t in TOPICS)))
    except Exception as error:
        result.update(reason=f'{type(error).__name__}: {error}', passed=False)
    finally:
        # Terminate OWNED process groups only. Never pkill/stop live services.
        for name in ['player', 'lio', 'adapter', 'router']:
            child = children.get(name)
            if child is None:
                continue
            try:
                os.killpg(child.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                child.wait(timeout=5.)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(child.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                child.wait(timeout=3.)
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()
        if writer:
            try:
                writer.close()
            except Exception as error:
                result.update(passed=False, writer_error=str(error))
            result.update(saved_clouds=writer.saved_clouds, output_data_bytes=writer.bytes)
        result['exit_codes'] = {name: p.returncode for name, p in children.items()}
        result['owned_processes_surviving'] = [name for name, p in children.items() if p.poll() is None]
        with socket.socket() as probe:
            result['private_port_released'] = probe.connect_ex(('127.0.0.1', PORT)) != 0
        if result['owned_processes_surviving'] or not result['private_port_released']:
            result['passed'] = False
        for stream in streams:
            stream.close()
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)
        save_json(out / 'result.json', result)
        print(json.dumps(result, ensure_ascii=False, allow_nan=False), flush=True)
    return 0 if result['passed'] else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, default=DEFAULT_BAG)
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=60.)
    parser.add_argument('--cloud-hz', type=float, default=5.)
    parser.add_argument('--max-points', type=int, default=100000)
    parser.add_argument('--prepare-only', action='store_true')
    return capture(parser.parse_args())


if __name__ == '__main__':
    raise SystemExit(main())
