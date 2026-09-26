"""Supervise one isolated, unmodified-bag Faster-LIO IMU-chain experiment.

Only owned child process groups are stopped. Source/configuration snapshots and
partial output survive failures. An optional RViz window remains after success.
"""
import argparse
from collections import Counter
from copy import deepcopy
import csv
from datetime import datetime
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry, Path as RosPath
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import UInt32
from std_srvs.srv import Trigger
import yaml


WS = Path('/home/dndx/d1max_nav_ws')
ROOT = Path(__file__).resolve().parent
IMU_PROFILES = {
    'central': ('/imu_driver/imu_central', 'central_imu'),
    'front_internal': ('/front_lidar/imu', 'front_imu_raw'),
}


def imu_profile(source, calibration):
    """Explicit source selection; keep original central behavior as the default."""
    topic, frame = IMU_PROFILES[source]
    if (calibration.get('input_topic') != topic or calibration.get('frame_id') != frame
            or calibration.get('output_topic') != '/d1max/slam/imu'):
        raise ValueError(f'{source} requires {topic} -> /d1max/slam/imu, frame {frame}')
    return ['/front_lidar', '/rear_lidar', topic]


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n')


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def actual_frontend_binary():
    """Record the domain-isolated live executable and its actually mapped library."""
    nodes = []
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue
        try:
            executable = (proc / 'exe').resolve(strict=True)
            if executable.name != 'run_mapping_online':
                continue
            if b'ROS_DOMAIN_ID=219' not in (proc / 'environ').read_bytes().split(b'\0'):
                continue
            paths = set()
            for line in (proc / 'maps').read_text().splitlines():
                fields = line.split(maxsplit=5)
                if len(fields) == 6 and 'libfaster_lio_lib.so' in fields[5]:
                    paths.add(Path(fields[5]).resolve(strict=True))
            if len(paths) != 1:
                raise RuntimeError(f'Expected one actually mapped Faster-LIO library for PID {proc.name}')
            nodes.append(dict(pid=int(proc.name), executable=str(executable),
                executable_sha256=sha256(executable),
                loaded_libraries=[dict(path=str(path), sha256=sha256(path)) for path in sorted(paths)]))
        except (PermissionError, FileNotFoundError):
            continue
    if len(nodes) != 1:
        raise RuntimeError(f'Expected one live Faster-LIO node in experiment domain; found {len(nodes)}')
    return dict(method='Read live /proc/PID/exe and /proc/PID/maps before playback', nodes=nodes)


def read_pgo_state(directory):
    """Read externally observable progress; the binary has no queue-status API."""
    row_counts, fingerprints = {}, {}
    for name in ('times.txt', 'odom_poses.txt', 'optimized_poses.txt', 'loop_events.csv'):
        path = directory / name
        if not path.is_file():
            row_counts[name] = 0
            fingerprints[name] = None
            continue
        stat = path.stat()
        fingerprints[name] = [stat.st_size, stat.st_mtime_ns]
        # All these writers terminate complete records with a newline.
        row_counts[name] = sum(bool(line.strip()) for line in path.read_text().splitlines())
    events = Counter()
    pending = set()
    event_file = directory / 'loop_events.csv'
    if event_file.is_file():
        with event_file.open() as stream:
            for row in csv.DictReader(stream):
                if not all(row.get(key) is not None for key in
                           ('event', 'history_keyframe', 'current_keyframe', 'value2')):
                    continue
                event = row['event']
                pair = (row['history_keyframe'], row['current_keyframe'])
                events[event] += 1
                if event == 'candidate_queued':
                    pending.add(pair)
                elif event in ('accepted', 'candidate_superseded', 'geometry_hold') or event.startswith('reject_'):
                    pending.discard(pair)
    return dict(keyframes=row_counts['times.txt'], odometry_rows=row_counts['odom_poses.txt'],
                optimized_rows=row_counts['optimized_poses.txt'], loop_events=dict(events),
                accepted_loops=events['accepted'], pending_candidates=sorted(pending),
                file_fingerprints=fingerprints)


def pcd_info(path):
    with path.open('rb') as stream:
        header = {}
        for _ in range(80):
            line = stream.readline().decode('ascii').strip()
            if not line or line.startswith('#'):
                continue
            key, _, value = line.partition(' ')
            header[key] = value.strip()
            if key == 'DATA':
                break
        else:
            raise RuntimeError(f'Missing PCD header: {path}')
        header_size = stream.tell()
    points = int(header.get('POINTS', '0'))
    if points <= 0 or path.stat().st_size <= header_size:
        raise RuntimeError(f'Empty saved map: {path}')
    return dict(path=str(path.resolve()), points=points, size_bytes=path.stat().st_size)


def validate_pgo_outputs(directory):
    state = read_pgo_state(directory)
    if not (state['keyframes'] > 0 and state['keyframes'] == state['odometry_rows'] == state['optimized_rows']):
        raise RuntimeError(f'PGO trajectory row counts disagree: {state}')
    for name, columns in [('times.txt', 1), ('odom_poses.txt', 12), ('optimized_poses.txt', 12)]:
        values = np.loadtxt(directory / name, ndmin=2)
        if values.shape != (state['keyframes'], columns) or not np.isfinite(values).all():
            raise RuntimeError(f'Invalid PGO trajectory: {name}')
    if state['pending_candidates']:
        raise RuntimeError('PGO still has queued/unresolved loop candidates')
    state['map'] = pcd_info(directory / 'optimized_map.pcd')
    state['loop_closure_succeeded'] = state['accepted_loops'] > 0
    state['note'] = 'A saved optimized map and completed replay do not imply an accepted or geometrically validated loop.'
    return state


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--config', type=Path, default=WS / 'src/d1max_slam/config/faster_lio_airy96.yaml')
    parser.add_argument('--calibration', type=Path, default=ROOT / 'calibration.yaml')
    parser.add_argument('--imu-source', choices=sorted(IMU_PROFILES), default='central',
                        help='Explicit experimental input source; raw vector axes are retained')
    parser.add_argument('--with-loop', action='store_true', help='Also run the installed SC-PGO backend')
    parser.add_argument('--save-keyframe-scans', action='store_true',
                        help='Save dense SC-PGO keyframe PCDs; requires --with-loop')
    parser.add_argument('--pgo-config', type=Path, default=WS / 'src/d1max_slam/config/sc_pgo_d1max.yaml')
    parser.add_argument('--loop-detection-frequency', type=float,
                        help='Experimental loop-check frequency override; keeps keyframe admission unchanged')
    parser.add_argument('--rate', type=float, default=1.0)
    parser.add_argument('--duration', type=float, default=0., help='Odometry sensor seconds; 0 is entire bag')
    parser.add_argument('--cloud-selection', choices=['dual', 'paired_front', 'paired_rear'], default='dual')
    parser.add_argument('--input-window', type=float, default=0.,
                        help='Exact paired-cloud header window, seconds from first pair; no loops or duration allowed')
    parser.add_argument('--no-rviz', action='store_true', help='Disable the default RViz visualization')
    parser.add_argument('--exit-after-save', action='store_true', help='Close owned RViz after saving successfully')
    args = parser.parse_args()
    calibration = yaml.safe_load(args.calibration.read_text())
    raw_topics = imu_profile(args.imu_source, calibration)
    if args.imu_source != 'central' and args.with_loop:
        raise ValueError('Alternative IMU source trials are frontend-only; no loop backend')
    if args.save_keyframe_scans and not args.with_loop:
        raise ValueError('--save-keyframe-scans requires --with-loop')
    if not math.isfinite(args.input_window) or args.input_window < 0:
        raise ValueError('Input window must be finite and nonnegative')
    if args.input_window and (args.duration or args.with_loop):
        raise ValueError('Paired-cloud window is a frontend-only experiment, independent of odometry duration')
    use_selector = args.input_window > 0 or args.cloud_selection != 'dual'
    cloud_topic = '/d1max/slam/selected_points' if use_selector else '/d1max/slam/points'
    if not math.isfinite(args.rate) or not math.isfinite(args.duration) or args.rate <= 0 or args.duration < 0:
        raise ValueError('Need finite positive rate and nonnegative duration')
    if args.loop_detection_frequency is not None and (not args.with_loop or
            not math.isfinite(args.loop_detection_frequency) or args.loop_detection_frequency <= 0):
        raise ValueError('Loop detection frequency requires --with-loop and a finite positive value')
    if os.environ.get('RMW_IMPLEMENTATION') != 'rmw_zenoh_cpp' or os.environ.get('ROS_DOMAIN_ID') != '219':
        raise RuntimeError('Use run.sh: fixed local Zenoh, isolated ROS domain 219')
    robot_dir = Path(os.environ['D1MAX_ROS2_DIR'])
    for key, filename in [('ZENOH_SESSION_CONFIG_URI', 'zenoh-local.json5'),
                          ('ZENOH_ROUTER_CONFIG_URI', 'zenoh-router-local.json5')]:
        expected = robot_dir / 'foxglove_d1max/config' / filename
        if Path(os.environ.get(key, '')).resolve() != expected.resolve() or not expected.is_file():
            raise RuntimeError(f'{key} must use the existing local-only configuration')
    lock = (ROOT / 'run.lock').open('a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit() or int(proc.name) == os.getpid():
            continue
        try:
            if b'ROS_DOMAIN_ID=219' not in (proc / 'environ').read_bytes().split(b'\0'):
                continue
            executable = (proc / 'exe').resolve().name
            command = (proc / 'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')
            if (executable in ('run_mapping_online', 'dual_lidar_adapter', 'map_capture_node', 'alaserPGO')
                    or 'central_imu_adapter.py' in command or 'ros2 bag play ' in command):
                raise RuntimeError(f'Existing experiment process in domain 219: PID {proc.name}; left untouched')
        except (PermissionError, FileNotFoundError):
            pass
    bag = args.bag.resolve(strict=True)
    metadata = yaml.safe_load((bag / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    bag_duration = metadata['duration']['nanoseconds'] / 1e9
    expected_counts = {t['topic_metadata']['name']: t['message_count'] for t in metadata['topics_with_message_count']}
    if any(expected_counts.get(t, 0) == 0 for t in raw_topics):
        raise RuntimeError('Both LiDAR streams and the selected raw IMU are required')
    source_stats = {n: [(bag / n).stat().st_size, (bag / n).stat().st_mtime_ns]
                    for n in metadata['relative_file_paths']}
    if not (ROOT / 'central_imu_adapter.py').is_file():
        raise RuntimeError('Experiment central_imu_adapter.py is missing')
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    suffix = '_central_imu_faster_lio_sc_pgo_zenoh' if args.with_loop else '_central_imu_faster_lio_zenoh'
    if args.imu_source == 'front_internal':
        suffix = '_front_internal_imu_faster_lio_zenoh'
    out = (args.output or WS / 'maps/runs' / (timestamp + suffix)).resolve()
    experiment_runs = ROOT.parent / 'lio_frontend_reliability_20260919/runs'
    if out.exists() or out.parent not in (WS / 'maps/runs', experiment_runs):
        raise RuntimeError('Use a fresh output directory directly under maps/runs or the reliability experiment runs')
    out.parent.mkdir(parents=True, exist_ok=True)
    out.mkdir()
    for directory in ('config', 'legacy_logs_before', 'legacy_logs_after'):
        (out / directory).mkdir()
    for filename in ('traj.txt', 'imu_.txt'):
        source = WS / 'src/faster_lio/Log' / filename
        if source.is_file():
            shutil.copy2(source, out / 'legacy_logs_before' / filename)
    for source, destination in [
        (ROOT / 'mapping.launch.py', 'mapping.launch.py'), (ROOT / 'central_imu_adapter.py', 'central_imu_adapter.py'),
        (ROOT / 'run.py', 'run.py'), (ROOT / 'run.sh', 'run.sh'),
        (args.calibration, 'calibration.yaml'),
        (args.config, 'frontend_source.yaml'),
        (WS / 'src/d1max_slam/config/dual_lidar.yaml', 'dual_lidar.yaml'),
        (bag / 'metadata.yaml', 'source_metadata.yaml'),
        (Path(os.environ['ZENOH_SESSION_CONFIG_URI']), 'zenoh-local.json5'),
        (Path(os.environ['ZENOH_ROUTER_CONFIG_URI']), 'zenoh-router-local.json5'),
    ]:
        shutil.copy2(source, out / 'config' / destination)
    if (ROOT / 'calibration').is_dir():
        shutil.copytree(ROOT / 'calibration', out / 'config/calibration')
    if use_selector:
        shutil.copy2(ROOT.parent / 'lio_frontend_reliability_20260919/cloud_subset.py', out / 'config/cloud_subset.py')
    frontend = yaml.safe_load(args.config.read_text())
    params = frontend['laserMapping']['ros__parameters']
    params['use_sim_time'] = False
    params['diagnostics'] = {'state_log_path': str(out / 'frontend_state.csv')}
    params.setdefault('publish', {}).update(scan_bodyframe_pub_en=True, scan_effect_pub_en=False,
                                          path_publish_en=True, world_frame='camera_init', body_frame=calibration['frame_id'])
    params.setdefault('common', {}).update(lid_topic=cloud_topic, imu_topic='/d1max/slam/imu', time_sync_en=False)
    params.setdefault('preprocess', {}).update(lidar_type=2, scan_line=192, time_scale=1.0)
    params.setdefault('mapping', {}).update(extrinsic_est_en=False,
        extrinsic_R=calibration['lio_extrinsic']['rotation'], extrinsic_T=calibration['lio_extrinsic']['translation'])
    params.setdefault('pcd_save', {})['pcd_save_en'] = False
    params['path_save_en'] = True
    (out / 'config/frontend.yaml').write_text(yaml.safe_dump(frontend, sort_keys=False))
    adapter = yaml.safe_load((out / 'config/dual_lidar.yaml').read_text())
    adapter['dual_lidar_adapter']['ros__parameters'].update(
        input_imu_topic='/central_experiment/unused_front_imu',
        output_imu_topic='/central_experiment/unused_imu', use_sim_time=False, lidar_mode='dual')
    (out / 'config/dual_lidar.yaml').write_text(yaml.safe_dump(adapter, sort_keys=False))
    if args.with_loop:
        shutil.copy2(args.pgo_config, out / 'config/sc_pgo_source.yaml')
        pgo_config = yaml.safe_load(args.pgo_config.read_text())
        pgo_params = pgo_config['/**']['ros__parameters']
        pgo_params.update(body_frame='central_imu', odom_frame='camera_init', map_frame='map',
                          publish_tf=True, use_sim_time=False, use_gps=False,
                          use_current_stamp_for_aft_pgo_odom=False,
                          save_keyframe_scans=args.save_keyframe_scans,
                          save_directory=str(out / 'sc_pgo'))
        if args.loop_detection_frequency is not None:
            pgo_params['loop_closure_frequency'] = args.loop_detection_frequency
        (out / 'config/sc_pgo.yaml').write_text(yaml.safe_dump(pgo_config, sort_keys=False))
    rviz = yaml.safe_load((WS / 'src/d1max_slam/rviz/faster_lio.rviz').read_text())
    for display in rviz['Visualization Manager']['Displays']:
        if display.get('Class') == 'rviz_default_plugins/PointCloud2':
            display['Topic']['Durability Policy'] = 'Transient Local'
            display['Decay Time'] = 0
    if args.with_loop:
        rviz['Visualization Manager']['Global Options']['Fixed Frame'] = 'map'
        displays = rviz['Visualization Manager']['Displays']
        frontend_display = next(d for d in displays if d.get('Class') == 'rviz_default_plugins/PointCloud2')
        optimized_display = deepcopy(frontend_display)
        frontend_display.update(Name='Frontend accumulated map', Value=False, Enabled=False)
        optimized_display.update(Name='PGO optimized map', Value=True, Enabled=True)
        optimized_display['Topic'].update(Value='/d1max/pgo/map', **{'Durability Policy': 'Volatile'})
        displays.insert(displays.index(frontend_display), optimized_display)
        # Keep one yellow current scan; Decay Time=0 replaces it per message.
        live_topic = '/d1max/faster_lio/cloud_registered'
        displays[:] = [display for display in displays
                       if not (display.get('Class') == 'rviz_default_plugins/PointCloud2'
                               and display.get('Topic', {}).get('Value') == live_topic)]
        live_display = deepcopy(frontend_display)
        live_display.update(Name='Live Registered Cloud', Value=True, Enabled=True,
                            Color='255; 176; 0', **{
                                'Color Transformer': 'FlatColor', 'Decay Time': 0,
                                'Queue Size': 3, 'Size (Pixels)': 2, 'Size (m)': 0.025,
                            })
        live_display['Topic'].update(Value=live_topic, Depth=3, **{
            'Reliability Policy': 'Best Effort', 'Durability Policy': 'Volatile',
            'History Policy': 'Keep Last',
        })
        displays.insert(displays.index(frontend_display), live_display)
        for display in displays:
            if display.get('Class') == 'rviz_default_plugins/Path':
                display['Name'] = 'PGO optimized path'
                display['Color'] = '30; 220; 120'
                display['Line Width'] = 0.015
                display['Topic']['Value'] = '/d1max/pgo/path'
        rviz['Visualization Manager']['Views']['Current']['Target Frame'] = 'map'
    (out / 'config/view.rviz').write_text(yaml.safe_dump(rviz, sort_keys=False))
    binaries = [WS / 'build/faster_lio/libfaster_lio_lib.so',
                WS / 'install/faster_lio/lib/libfaster_lio_lib.so',
                WS / 'install/faster_lio/lib/faster_lio/run_mapping_online',
                WS / 'install/d1max_slam/lib/d1max_slam/dual_lidar_adapter',
                WS / 'install/d1max_slam/lib/d1max_slam/map_capture_node']
    if args.with_loop:
        binaries.append(WS / 'install/sc_pgo/lib/sc_pgo/alaserPGO')
    manifest = dict(bag=str(bag), output=str(out), rmw='rmw_zenoh_cpp', domain=219,
        topics=raw_topics, rate=args.rate, requested_duration=args.duration, duration_sec=bag_duration,
        imu_source=args.imu_source, imu_input_topic=raw_topics[2], imu_frame=calibration['frame_id'],
        raw_bag_unmodified=True, duplicate_imu_values_preserved=True, loop_closure_enabled=args.with_loop,
        backend='faster_lio+sc_pgo' if args.with_loop else 'faster_lio',
        save_keyframe_scans=args.save_keyframe_scans,
        pgo_source_config=str(args.pgo_config.resolve()) if args.with_loop else None,
        loop_detection_frequency_override=args.loop_detection_frequency,
        height_lock_enabled=False,
        cloud_selection=args.cloud_selection, paired_cloud_window_sec=args.input_window,
        clock_policy='No /clock playback; use_sim_time=false. Bag/raw LiDAR stamps remain unchanged; central IMU output stamp adds the configured experimental offset.',
        central_timestamp_offset_sec=float(calibration['timestamp_offset_sec']),
        source_config=str(args.config.resolve()), source_stats=source_stats,
        expected_raw_counts={topic: expected_counts[topic] for topic in raw_topics},
        binary_sha256={str(p): sha256(p) for p in binaries},
        snapshot_sha256={str(p.relative_to(out / 'config')): sha256(p)
                         for p in (out / 'config').rglob('*') if p.is_file()},
        trajectory_policy='frontend_odometry.tum is authoritative; native gflags trajectory argument is not parsed by this binary; legacy logs are backed up and copied after shutdown.',
        rviz_enabled=not args.no_rviz)
    write_json(out / 'manifest.json', manifest)
    print(f'RUN_DIRECTORY={out}', flush=True)
    children, handles, subscriptions = {}, [], []
    node = None
    save_client = None
    frontend_save_response = None
    cancelled = False
    trajectory = []
    counts = Counter()
    last_stamps = {}
    first_stamps = {}
    last_data = time.monotonic()
    pgo_observed = dict(path_messages=0, keyframes=0, accepted_loop_count_topic=0,
                        last_path_stamp_ns=None, last_path_wall=None)
    trajectory_file = (out / 'frontend_odometry.tum').open('x', buffering=1)
    trajectory_file.write('# t x y z qx qy qz qw\n')
    result = dict(complete=False, loop_closure_enabled=args.with_loop, height_lock_enabled=False,
                  ground_truth_accuracy_validated=False)

    def start(name, command):
        handle = (out / f'{name}.log').open('xb')
        handles.append(handle)
        child = subprocess.Popen(command, cwd=out, stdout=handle, stderr=subprocess.STDOUT, start_new_session=True)
        children[name] = child
        write_json(out / 'processes.json', {n: {'pid': p.pid, 'command': p.args} for n, p in children.items()})
        return child

    def stop(name, timeout=20):
        child = children.get(name)
        if child is None or child.poll() is not None:
            return
        os.killpg(child.pid, signal.SIGINT)
        try:
            child.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            result.setdefault('forced_cleanup', []).append(name)
            os.killpg(child.pid, signal.SIGTERM)
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait(timeout=5)

    def cancel(_sig, _frame):
        nonlocal cancelled
        cancelled = True

    def status(stage):
        record = dict(stage=stage, observed_counts=dict(counts),
                      sensor_seconds=trajectory[-1][0] - trajectory[0][0] if trajectory else 0,
                      xyz=trajectory[-1][1:4] if trajectory else None)
        if args.with_loop:
            record['pgo'] = dict(pgo_observed)
        write_json(out / 'status.json', record)
        print(json.dumps(record), flush=True)

    def save_map(label, timeout=45):
        nonlocal frontend_save_response
        if frontend_save_response is not None:
            return frontend_save_response
        response = {'success': False, 'message': 'Map service unavailable'}
        if node is not None and save_client is not None and save_client.service_is_ready():
            future = save_client.call_async(Trigger.Request())
            deadline = time.monotonic() + timeout
            while not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.05)
            saved = future.result() if future.done() else None
            response = dict(success=bool(saved and saved.success), message=saved.message if saved else 'Map save timed out')
        frontend_save_response = response
        write_json(out / f'{label}_save_response.json', response)
        return response

    def check_pipeline():
        for name in ('frontend', 'central_adapter') + (('pgo',) if args.with_loop else ()) + (('cloud_selector',) if use_selector else ()):
            if children[name].poll() is not None:
                raise RuntimeError(f'{name} exited unexpectedly: {children[name].returncode}')

    def drain_pgo():
        # No queue/finalization API is exposed by this binary. These are
        # bounded, observable stability checks, not proof of empty queues.
        minimum_wait, stable_window, maximum_wait = 15., 5., 120.
        begin = stable_since = time.monotonic()
        previous = None
        latest = None
        last_status = begin
        while time.monotonic() - begin < maximum_wait:
            if cancelled:
                raise KeyboardInterrupt()
            check_pipeline()
            until = time.monotonic() + .5
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=.02)
            latest = read_pgo_state(out / 'sc_pgo')
            signature = (latest, pgo_observed['keyframes'],
                         pgo_observed['accepted_loop_count_topic'],
                         last_stamps.get('/d1max/faster_lio/odometry'),
                         last_stamps.get('/d1max/faster_lio/cloud_registered_body'))
            now = time.monotonic()
            if signature != previous:
                stable_since, previous = now, signature
            rows_agree = (latest['keyframes'] > 0 and latest['keyframes'] == latest['odometry_rows']
                          == latest['optimized_rows'] == pgo_observed['keyframes'])
            path_fresh = pgo_observed['last_path_wall'] is not None and now - pgo_observed['last_path_wall'] < 5
            counts_agree = latest['accepted_loops'] == pgo_observed['accepted_loop_count_topic']
            if (now - begin >= minimum_wait and now - stable_since >= stable_window
                    and now - last_data >= stable_window and rows_agree and path_fresh
                    and counts_agree and not latest['pending_candidates']):
                evidence = dict(passed=True, wait_sec=now - begin, stable_sec=now - stable_since,
                    minimum_wait_sec=minimum_wait, maximum_wait_sec=maximum_wait,
                    method='Heuristic observable stability; no direct input/ICP queue API is available.',
                    state=latest, observed=dict(pgo_observed))
                write_json(out / 'pgo_drain.json', evidence)
                return evidence
            if now - last_status > 10:
                status('draining_pgo')
                last_status = now
        evidence = dict(passed=False, wait_sec=time.monotonic() - begin, state=latest,
                        observed=dict(pgo_observed),
                        method='Heuristic observable stability timed out; no direct queue API is available.')
        write_json(out / 'pgo_drain.json', evidence)
        raise RuntimeError('PGO did not reach the bounded observable-stability criteria')

    signal.signal(signal.SIGINT, cancel)
    signal.signal(signal.SIGTERM, cancel)
    try:
        try:
            with socket.create_connection(('127.0.0.1', 7447), timeout=.5):
                pass
        except OSError:
            router = robot_dir / 'local/opt/ros/humble/lib/rmw_zenoh_cpp/rmw_zenohd'
            start('router', [str(router)])
            deadline = time.monotonic() + 8
            while True:
                try:
                    with socket.create_connection(('127.0.0.1', 7447), timeout=.2):
                        break
                except OSError:
                    if time.monotonic() > deadline or children['router'].poll() is not None:
                        raise RuntimeError('Existing local-only Zenoh configuration failed to start')
                    time.sleep(.1)
        # Keep the supervisor's cancellation handler: an immediate context
        # shutdown would prevent saving and orderly child-group cleanup.
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = rclpy.create_node('central_imu_frontend_observer')
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        collision_topics = raw_topics + ['/d1max/slam/points', '/d1max/slam/imu', '/d1max/faster_lio/odometry']
        if use_selector:
            collision_topics += [cloud_topic]
        if args.with_loop:
            collision_topics += ['/d1max/pgo/odometry', '/d1max/pgo/path', '/d1max/pgo/map']
        if any(node.count_publishers(topic) for topic in collision_topics):
            raise RuntimeError('An existing publisher occupies experiment domain 219; existing processes left untouched')

        def count_message(topic):
            def callback(message):
                counts[topic] += 1
                last_stamps[topic] = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
                first_stamps.setdefault(topic, last_stamps[topic])
            return callback

        def on_odom(message):
            nonlocal last_data
            p, q = message.pose.pose.position, message.pose.pose.orientation
            stamp_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
            row = [stamp_ns / 1e9, p.x, p.y, p.z, q.x, q.y, q.z, q.w]
            if not all(math.isfinite(value) for value in row):
                raise RuntimeError('Nonfinite frontend odometry')
            trajectory.append(row)
            trajectory_file.write(f'{stamp_ns // 1_000_000_000}.{stamp_ns % 1_000_000_000:09d} ' +
                                  ' '.join(f'{value:.15g}' for value in row[1:]) + '\n')
            counts['/d1max/faster_lio/odometry'] += 1
            last_stamps['/d1max/faster_lio/odometry'] = stamp_ns
            last_data = time.monotonic()

        for topic in raw_topics + ['/d1max/slam/points', '/d1max/slam/imu', '/d1max/faster_lio/cloud_registered'] + ([cloud_topic] if use_selector else []):
            kind = Imu if topic in (raw_topics[2], '/d1max/slam/imu') else PointCloud2
            subscriptions.append(node.create_subscription(kind, topic, count_message(topic), qos_profile_sensor_data))
        subscriptions.append(node.create_subscription(Odometry, '/d1max/faster_lio/odometry', on_odom, 100))
        if args.with_loop:
            def on_pgo_path(message):
                pgo_observed.update(path_messages=pgo_observed['path_messages'] + 1,
                    keyframes=len(message.poses), last_path_wall=time.monotonic(),
                    last_path_stamp_ns=int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec))

            subscriptions.append(node.create_subscription(RosPath, '/d1max/pgo/path', on_pgo_path, 2))
            subscriptions.append(node.create_subscription(UInt32, '/d1max/pgo/loop_count',
                lambda message: pgo_observed.update(accepted_loop_count_topic=int(message.data)),
                QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
            subscriptions.append(node.create_subscription(PointCloud2, '/d1max/faster_lio/cloud_registered_body',
                count_message('/d1max/faster_lio/cloud_registered_body'), qos_profile_sensor_data))
        save_client = node.create_client(Trigger, '/d1max/slam/save')
        start('central_adapter', ['/usr/bin/python3', str(out / 'config/central_imu_adapter.py'),
                                  '--config', str(out / 'config/calibration.yaml'),
                                  '--audit-file', str(out / 'central_imu_adapter.json')])
        if use_selector:
            start('cloud_selector', ['/usr/bin/python3', str(out / 'config/cloud_subset.py'),
                  '--selection', args.cloud_selection, '--window', str(args.input_window),
                  '--audit-file', str(out / 'paired_scans.csv')])
        start('frontend', ['ros2', 'launch', str(out / 'config/mapping.launch.py'),
                          f'output_dir:={out}', f'frontend_config:={out}/config/frontend.yaml',
                          f'adapter_config:={out}/config/dual_lidar.yaml',
                          f'calibration_config:={out}/config/calibration.yaml', f'cloud_topic:={cloud_topic}'])
        if args.with_loop:
            command = [str(WS / 'install/sc_pgo/lib/sc_pgo/alaserPGO'), '--ros-args',
                       '-r', '__node:=d1max_sc_pgo', '--params-file', str(out / 'config/sc_pgo.yaml')]
            for old, new in [('aft_mapped_to_init', 'faster_lio/odometry'),
                             ('velodyne_cloud_registered_local', 'faster_lio/cloud_registered_body'),
                             ('aft_pgo_odom', 'pgo/odometry'), ('aft_pgo_path', 'pgo/path'),
                             ('aft_pgo_map', 'pgo/map'), ('loop_scan_local', 'pgo/loop_scan'),
                             ('loop_submap_local', 'pgo/loop_submap'), ('pgo_loop_count', 'pgo/loop_count'),
                             ('repub_odom', 'pgo/repub_odom')]:
                command.extend(['-r', f'/{old}:=/d1max/{new}'])
            start('pgo', command)
        if not args.no_rviz:
            start('rviz', ['/opt/ros/humble/lib/rviz2/rviz2', '-d', str(out / 'config/view.rviz')])
        status('starting')
        deadline = time.monotonic() + 30
        # Observer subscriptions also count, so require the observer plus the actual consumer.
        ready_topics = raw_topics + ['/d1max/slam/points', '/d1max/slam/imu']
        if use_selector:
            ready_topics += [cloud_topic]
        if args.with_loop:
            ready_topics += ['/d1max/faster_lio/cloud_registered_body', '/d1max/faster_lio/odometry']
        while not (all(node.count_subscribers(topic) >= 2 for topic in ready_topics)
                   and save_client.service_is_ready()):
            if cancelled:
                raise KeyboardInterrupt()
            check_pipeline()
            if time.monotonic() > deadline:
                raise RuntimeError('Mapping pipeline did not become ready')
            rclpy.spin_once(node, timeout_sec=.1)
        write_json(out / 'runtime_binary_check.json', actual_frontend_binary())
        player = start('playback', ['ros2', 'bag', 'play', str(bag), '--rate', str(args.rate),
            '--read-ahead-queue-size', '400', '--disable-keyboard-controls', '--topics', *raw_topics])
        begin = last_data = time.monotonic()
        last_status = 0.
        segment = False
        while player.poll() is None:
            if cancelled:
                raise KeyboardInterrupt()
            rclpy.spin_once(node, timeout_sec=.01)
            elapsed = trajectory[-1][0] - trajectory[0][0] if trajectory else 0.
            paired_topic = '/d1max/slam/points'
            if (args.input_window and paired_topic in first_stamps and
                    last_stamps[paired_topic] - first_stamps[paired_topic] >= round((args.input_window + 1.) * 1e9)):
                segment = True
                stop('playback')
                break
            if args.duration and elapsed >= args.duration:
                segment = True
                stop('playback')
                break
            check_pipeline()
            if time.monotonic() - last_data > 45:
                raise RuntimeError('No frontend odometry for 45 seconds')
            if time.monotonic() - begin > bag_duration / args.rate + 180:
                raise RuntimeError('Playback exceeded time guard')
            if time.monotonic() - last_status > 10:
                status('mapping')
                last_status = time.monotonic()
        if not segment and player.returncode:
            raise RuntimeError(f'Playback failed: {player.returncode}')
        status('draining')
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            if cancelled:
                raise KeyboardInterrupt()
            check_pipeline()
            rclpy.spin_once(node, timeout_sec=.02)
        if args.with_loop:
            result['pgo_drain'] = drain_pgo()
        status('saving')
        response = save_map('final')
        if not response['success']:
            raise RuntimeError(response['message'])
        result['save_message'] = response['message']
        if args.with_loop:
            # This binary exports optimized_map.pcd exactly once on SIGINT.
            stop('pgo', timeout=55)
            if children['pgo'].returncode != 0 or 'pgo' in result.get('forced_cleanup', []):
                raise RuntimeError('SC-PGO failed its graceful final save/shutdown')
            pgo_summary = validate_pgo_outputs(out / 'sc_pgo')
            result.update(pgo=pgo_summary, accepted_loops=pgo_summary['accepted_loops'],
                          loop_closure_succeeded=pgo_summary['loop_closure_succeeded'])
            write_json(out / 'pgo_result.json', pgo_summary)
        if len(trajectory) < 2:
            raise RuntimeError('Insufficient odometry')
        span = trajectory[-1][0] - trajectory[0][0]
        result.update(segment_only=segment, poses=len(trajectory), sensor_seconds=span,
            bag_coverage_fraction=span / bag_duration,
            endpoint_delta_xyz=(np.asarray(trajectory[-1][1:4]) - trajectory[0][1:4]).tolist())
        if not segment and span / bag_duration < .98:
            raise RuntimeError('Incomplete bag coverage')
        if source_stats != {n: [(bag / n).stat().st_size, (bag / n).stat().st_mtime_ns] for n in source_stats}:
            raise RuntimeError('Source bag changed')
        if not (out / 'scans.pcd').is_file() or (out / 'scans.pcd').stat().st_size < 100:
            raise RuntimeError('Saved map is missing or empty')
        if args.with_loop:
            result['frontend_map'] = pcd_info(out / 'scans.pcd')
        result.update(complete=True, map_path=str((out / 'scans.pcd').resolve()))
    except KeyboardInterrupt:
        result['error'] = 'Interrupted'
    except Exception as exception:
        result['error'] = str(exception)
        print(f'FAILED: {exception}', flush=True)
    finally:
        stop('playback')
        if not result['complete']:
            try:
                result['partial_save'] = save_map('partial', timeout=20)
            except Exception as exception:
                result['partial_save'] = {'success': False, 'message': str(exception)}
        stop('central_adapter')
        stop('cloud_selector')
        if args.with_loop:
            stop('pgo', timeout=55)
            if 'pgo' in children:
                result['pgo_observed'] = dict(pgo_observed)
                result['pgo_final_state'] = read_pgo_state(out / 'sc_pgo')
                result['accepted_loops'] = result['pgo_final_state']['accepted_loops']
                result['loop_closure_succeeded'] = result['accepted_loops'] > 0
                result['pgo_map_exists'] = (out / 'sc_pgo/optimized_map.pcd').is_file()
        stop('frontend', timeout=40)
        for filename in ('traj.txt', 'imu_.txt'):
            source = WS / 'src/faster_lio/Log' / filename
            if source.is_file():
                shutil.copy2(source, out / 'legacy_logs_after' / filename)
            backup = out / 'legacy_logs_before' / filename
            if backup.is_file():
                shutil.copy2(backup, source)
                result.setdefault('legacy_logs_restored', []).append(str(source))
        trajectory_file.close()
        result.update(observed_counts=dict(counts), last_observed_stamp_ns=last_stamps,
                      observed_count_caveat='Observer best-effort receipt counts, not proof of algorithm consumption; compare frontend_state.csv.',
                      child_returncodes={name: child.poll() for name, child in children.items()},
                      source_unchanged=source_stats == {n: [(bag / n).stat().st_size, (bag / n).stat().st_mtime_ns] for n in source_stats})
        write_json(out / 'result.json', result)
        status('complete' if result['complete'] else 'failed')
        print(json.dumps(result, ensure_ascii=False), flush=True)
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
        if result['complete'] and not cancelled and not args.exit_after_save:
            viewer = children.get('rviz')
            while viewer is not None and viewer.poll() is None and not cancelled:
                time.sleep(.5)
        stop('rviz', timeout=5)
        stop('router', timeout=5)
        for handle in handles:
            handle.close()
    return 0 if result['complete'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
