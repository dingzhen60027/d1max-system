"""Isolated, supervised original-bag test. No SDK, commands, or global cleanup."""
import argparse
import collections
import csv
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

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry, Path as RosPath
from std_msgs.msg import UInt32
from std_srvs.srv import Trigger
import yaml

WS = Path('/home/dndx/d1max_nav_ws')
ROOT = Path(__file__).resolve().parent


def write_json(path, value):
    tmp = path.with_suffix(path.suffix + '.tmp')
    tmp.write_text(json.dumps(value, indent=2, ensure_ascii=False) + '\n')
    tmp.replace(path)


def pcd_info(path):
    header = {}
    with path.open('rb') as f:
        for _ in range(40):
            line = f.readline().decode('ascii').strip()
            if line and not line.startswith('#'):
                key, _, value = line.partition(' ')
                header[key] = value
            if line.startswith('DATA '):
                break
        else:
            raise RuntimeError(f'Invalid PCD: {path}')
        payload_start = f.tell()
    count = int(header['POINTS'])
    if count <= 0 or path.stat().st_size <= payload_start:
        raise RuntimeError(f'Empty PCD: {path}')
    if header['DATA'] == 'binary':
        stride = sum(int(s)*int(c) for s,c in zip(header['SIZE'].split(),header['COUNT'].split()))
        if path.stat().st_size-payload_start != stride*count:
            raise RuntimeError(f'Truncated PCD: {path}')
    return {'path': str(path), 'points': count, 'bytes': path.stat().st_size}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--rate', type=float, default=1.0)
    parser.add_argument('--lidar-mode', choices=('dual', 'front'), default='dual')
    parser.add_argument('--save-keyframes', action='store_true',
                        help='Keep downsampled body keyframes for offline loop verification')
    args = parser.parse_args()
    bag, out = args.bag.resolve(), args.output.resolve()
    if not (bag/'metadata.yaml').is_file() or args.rate <= 0:
        raise RuntimeError('Need a finalized bag and positive playback rate')
    if out.parent != WS/'maps'/'runs' or out.exists():
        raise RuntimeError('Use a NEW run directory directly under maps/runs')
    if os.environ.get('RMW_IMPLEMENTATION') != 'rmw_zenoh_cpp':
        raise RuntimeError('D1 Max requires rmw_zenoh_cpp; middleware changes are forbidden')
    if os.environ.get('ROS_DOMAIN_ID') != '217':
        raise RuntimeError('This offline test must remain isolated in ROS domain 217')
    transport_configs = [Path(os.environ[key]) for key in
                         ('ZENOH_SESSION_CONFIG_URI', 'ZENOH_ROUTER_CONFIG_URI')]
    if not all(p.is_file() for p in transport_configs):
        raise RuntimeError('Missing existing local Zenoh configuration; no middleware fallback')
    # Reuse the original local Zenoh transport. Do not start a router, attach
    # to the robot, or silently choose another middleware when it is absent.
    try:
        with socket.create_connection(('127.0.0.1', 7447), timeout=1):
            pass
    except OSError as exc:
        raise RuntimeError('Local Zenoh router is not running at 127.0.0.1:7447; nothing started') from exc
    lock = (ROOT/'run.lock').open('a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    # Never kill another mapping task just to acquire resources.
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue
        try:
            executable = (proc/'exe').resolve().name
            if executable in ('run_mapping_online', 'alaserPGO', 'dual_lidar_adapter'):
                raise RuntimeError(f'Existing mapping process {proc.name}: {executable}')
        except (FileNotFoundError, PermissionError):
            pass
    metadata = yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    playback_topics = ['/front_lidar', '/front_lidar/imu']
    if args.lidar_mode == 'dual':
        playback_topics.insert(1, '/rear_lidar')
    topic_counts = {t['topic_metadata']['name']: t['message_count']
                    for t in metadata['topics_with_message_count']}
    if any(topic_counts.get(t, 0) <= 0 for t in playback_topics):
        raise RuntimeError('Bag is missing a required sensor; never silently fall back to front-only')
    duration = metadata['duration']['nanoseconds']/1e9
    source_stats = {n: [ (bag/n).stat().st_size, (bag/n).stat().st_mtime_ns ] for n in metadata['relative_file_paths']}
    out.mkdir()
    (out/'sc_pgo').mkdir()
    (out/'config').mkdir()
    (out/'legacy_logs_before').mkdir()
    # This legacy binary ignores its gflags trajectory argument; preserve its
    # previous generated logs before it reuses its built-in Log/ directory.
    for name in ('traj.txt', 'imu_.txt'):
        source = WS/'src/faster_lio/Log'/name
        if source.is_file():
            shutil.copy2(source, out/'legacy_logs_before'/name)
    for source in [WS/'src/d1max_slam/launch/mapping.launch.py',
                   WS/'src/d1max_slam/config/faster_lio_airy96.yaml',
                   WS/'src/d1max_slam/config/dual_lidar.yaml',
                   WS/'src/d1max_slam/config/sc_pgo_d1max.yaml', ROOT/'run.py', ROOT/'run.sh', *transport_configs]:
        shutil.copy2(source, out/'config'/source.name)
    shutil.copy2(bag/'metadata.yaml', out/'config/source_metadata.yaml')
    if args.save_keyframes:
        pgo_config_path = out/'config/sc_pgo_d1max.yaml'
        pgo_config = yaml.safe_load(pgo_config_path.read_text())
        pgo_config['/**']['ros__parameters']['save_keyframe_scans'] = True
        pgo_config_path.write_text(yaml.safe_dump(pgo_config, sort_keys=False))
    binaries = [WS/'install/faster_lio/lib/faster_lio/run_mapping_online',
                WS/'install/sc_pgo/lib/sc_pgo/alaserPGO',
                WS/'install/d1max_slam/lib/d1max_slam/dual_lidar_adapter',
                WS/'install/faster_lio/lib/libfaster_lio_lib.so']
    # Symlink-installed executables can resolve the build-tree library through
    # RPATH. Keep both hashes, not only the separately installed .so copy.
    build_library = WS/'build/faster_lio/libfaster_lio_lib.so'
    if build_library.is_file():
        binaries.append(build_library)
    manifest = {'bag':str(bag), 'output':str(out), 'duration_sec':duration, 'rate':args.rate,
                'backend':'faster_lio+sc_pgo', 'lidar_mode':args.lidar_mode,
                'topics':playback_topics, 'raw_bag_unmodified':True,
                'save_keyframes':args.save_keyframes,
                'calibration':'existing mapping.launch.py historical transforms; NOT newly calibrated',
                'rmw':os.environ['RMW_IMPLEMENTATION'], 'domain':217,
                'source_stats':source_stats,
                'binary_sha256':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in binaries}}
    write_json(out/'manifest.json', manifest)
    rviz_config = yaml.safe_load((WS/'src/d1max_slam/rviz/d1max_pgo.rviz').read_text())
    for display in rviz_config['Visualization Manager']['Displays']:
        if display.get('Class') == 'rviz_default_plugins/PointCloud2':
            display['Size (Pixels)'] = 1
        if display.get('Name') == 'Live Registered Cloud':
            display['Decay Time'] = 0
        if display.get('Class') == 'rviz_default_plugins/Path':
            display['Line Width'] = 0.015
    (out/'config/view.rviz').write_text(yaml.safe_dump(rviz_config, sort_keys=False))
    rclpy.init()
    node = rclpy.create_node('d1max_bag_test_observer')
    children = {}
    handles = []
    cancelled = False
    state = {'stage':'starting', 'odom_messages':0, 'first_sensor_stamp':None,
             'last_sensor_stamp':None, 'keyframes':0, 'accepted_loops':0}
    trajectory = (out/'frontend_odometry.tum').open('w', buffering=1)
    trajectory.write('# timestamp x y z qx qy qz qw\n')
    last_odom_wall = time.monotonic()

    def on_odom(msg):
        nonlocal last_odom_wall
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        values = [stamp,p.x,p.y,p.z,q.x,q.y,q.z,q.w]
        if not all(math.isfinite(v) for v in values):
            raise RuntimeError('Nonfinite frontend odometry')
        trajectory.write(' '.join(f'{v:.9f}' for v in values)+'\n')
        state['odom_messages'] += 1
        if state['first_sensor_stamp'] is None:
            state['first_sensor_stamp'] = stamp
        state['last_sensor_stamp'] = stamp
        state['latest_xyz'] = [p.x,p.y,p.z]
        last_odom_wall = time.monotonic()

    subscriptions = [
        node.create_subscription(Odometry, '/d1max/faster_lio/odometry', on_odom, 100),
        node.create_subscription(RosPath, '/d1max/pgo/path', lambda m:state.update(keyframes=len(m.poses)), 2),
        node.create_subscription(UInt32, '/d1max/pgo/loop_count', lambda m:state.update(accepted_loops=m.data),
                                 QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL))]
    save_client = node.create_client(Trigger, '/d1max/slam/save')

    def status(stage=None):
        if stage:
            state['stage'] = stage
            print(stage, flush=True)
        state['updated_wall'] = time.time()
        state['processed_span_sec'] = (state['last_sensor_stamp'] or 0)-(state['first_sensor_stamp'] or 0)
        write_json(out/'status.json', state)

    def start(name, command):
        handle = (out/f'{name}.log').open('wb')
        handles.append(handle)
        children[name] = subprocess.Popen(command, cwd=out, stdout=handle,
            stderr=subprocess.STDOUT, start_new_session=True)
        write_json(out/'processes.json', {k:{'pid':v.pid,'argv':v.args} for k,v in children.items()})
        return children[name]

    def stop(name, timeout=40):
        child = children.get(name)
        if child is None or child.poll() is not None:
            return
        child.send_signal(signal.SIGINT)
        try:
            child.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            os.killpg(child.pid, signal.SIGTERM)
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait(timeout=5)
            raise RuntimeError(f'{name} needed forced cleanup; results not certified complete')

    def tick(seconds, allow_cancel=True):
        until=time.monotonic()+seconds
        while time.monotonic()<until:
            if cancelled and allow_cancel:
                raise KeyboardInterrupt()
            rclpy.spin_once(node, timeout_sec=min(0.1,max(0.0,until-time.monotonic())))

    def request_save():
        if not save_client.service_is_ready():
            return {'success':False,'message':'Map save service unavailable'}
        future = save_client.call_async(Trigger.Request())
        until = time.monotonic()+35
        while not future.done() and time.monotonic()<until:
            tick(0.1, allow_cancel=False)
        result = future.result() if future.done() else None
        report={'success':bool(result and result.success),'message':result.message if result else 'save timeout'}
        write_json(out/'save_response.json', report)
        return report

    def cancel(_signum,_frame):
        nonlocal cancelled
        cancelled=True
    signal.signal(signal.SIGINT,cancel)
    signal.signal(signal.SIGTERM,cancel)
    complete=False
    try:
        status()
        start('frontend',['ros2','launch','d1max_slam','mapping.launch.py','backend:=faster_lio',
                          f'lidar_mode:={args.lidar_mode}','use_rviz:=false',f'output_dir:={out}'])
        pgo_command=[str(binaries[1]),'--ros-args','-r','__node:=d1max_sc_pgo',
                     '--params-file',str(out/'config/sc_pgo_d1max.yaml'),
                     '-p',f'save_directory:={out}/sc_pgo','-p','use_current_stamp_for_aft_pgo_odom:=false']
        for old,new in [('aft_mapped_to_init','faster_lio/odometry'),
                        ('velodyne_cloud_registered_local','faster_lio/cloud_registered_body'),
                        ('aft_pgo_odom','pgo/odometry'),('aft_pgo_path','pgo/path'),
                        ('aft_pgo_map','pgo/map'),('loop_scan_local','pgo/loop_scan'),
                        ('loop_submap_local','pgo/loop_submap'),('pgo_loop_count','pgo/loop_count')]:
            pgo_command += ['-r',f'/{old}:=/d1max/{new}']
        start('pgo',pgo_command)
        start('rviz',['/opt/ros/humble/lib/rviz2/rviz2','-d',str(out/'config/view.rviz')])
        deadline=time.monotonic()+25
        while not (save_client.service_is_ready() and
                   node.count_subscribers('/front_lidar')>0 and
                   (args.lidar_mode != 'dual' or node.count_subscribers('/rear_lidar')>0) and
                   node.count_subscribers('/front_lidar/imu')>0 and
                   node.count_subscribers('/d1max/slam/points')>0 and
                   node.count_subscribers('/d1max/faster_lio/cloud_registered_body')>0):
            if time.monotonic()>deadline:
                raise RuntimeError('Mapping nodes did not become ready within 25 s')
            for name in ('frontend','pgo'):
                if children[name].poll() is not None:
                    raise RuntimeError(f'{name} exited during startup')
            tick(0.2)
        tick(2)
        player=start('playback',['ros2','bag','play',str(bag),'--rate',str(args.rate),
            '--read-ahead-queue-size','2000','--disable-keyboard-controls',
            '--topics',*playback_topics])
        playback_start=time.monotonic()
        last_odom_wall=playback_start
        status('mapping')
        while player.poll() is None:
            tick(1)
            status()
            if any(children[n].poll() is not None for n in ('frontend','pgo')):
                raise RuntimeError('A mapping process exited during playback')
            if time.monotonic()-last_odom_wall>35:
                raise RuntimeError('No frontend odometry for 35 s')
            if time.monotonic()-playback_start>duration/args.rate+180:
                raise RuntimeError('Playback exceeded duration guard')
        if player.returncode:
            raise RuntimeError(f'Playback failed: {player.returncode}')
        status('draining')
        tick(12)
        status('saving')
        if not request_save()['success']:
            raise RuntimeError('Frontend map save failed')
        # PGO writes its FINAL optimized map on a graceful SIGINT.
        stop('pgo',60)
        stop('frontend',40)
        trajectory.flush()
        source_traj=WS/'src/faster_lio/Log/traj.txt'
        if source_traj.is_file():
            shutil.copy2(source_traj,out/'faster_lio_trajectory.txt')
        frontend_map=pcd_info(out/'scans.pcd')
        pgo_map=pcd_info(out/'sc_pgo/optimized_map.pcd')
        status()
        coverage=state['processed_span_sec']/duration
        if coverage<0.98 or state['odom_messages']<100:
            raise RuntimeError(f'Incomplete trajectory: coverage={coverage:.3f}')
        if source_stats != {n:[(bag/n).stat().st_size,(bag/n).stat().st_mtime_ns] for n in source_stats}:
            raise RuntimeError('Original bag changed during test')
        with (out/'sc_pgo/loop_events.csv').open() as f:
            events=collections.Counter(r['event'] for r in csv.DictReader(f))
        write_json(out/'result.json',{'complete':True,'source_bag':str(bag),
            'frontend_map':frontend_map,'pgo_map':pgo_map,'trajectory_coverage':coverage,
            'odom_messages':state['odom_messages'],'keyframes':state['keyframes'],
            'accepted_loops':events.get('accepted',0),'loop_events':dict(events),
            'geometric_accuracy_validated':False,
            'note':'A saved optimized map does not imply any loop was accepted.'})
        complete=True
        status('complete')
        print(f'Saved both maps: {out}',flush=True)
    except KeyboardInterrupt:
        status('interrupted')
    except Exception as exc:
        state['error']=str(exc)
        status('failed')
        print(f'FAILED: {exc}',flush=True)
    finally:
        for name in ('playback',):
            try: stop(name,10)
            except Exception as exc: print(exc,flush=True)
        if not complete:
            try: request_save()
            except Exception as exc: print(exc,flush=True)
        for name in ('pgo','frontend'):
            try: stop(name,40)
            except Exception as exc: print(exc,flush=True)
        trajectory.close()
        node.destroy_node()
        rclpy.shutdown()
        # Keep only RViz alive for inspection after successful automatic save.
        # Closing RViz during playback never cancels the bag or its exports.
        if complete and not cancelled:
            rviz=children.get('rviz')
            while rviz and rviz.poll() is None and not cancelled:
                time.sleep(0.5)
        try: stop('rviz',5)
        except Exception as exc: print(exc,flush=True)
        for handle in handles: handle.close()
    return 0 if complete else 1


if __name__ == '__main__':
    raise SystemExit(main())
