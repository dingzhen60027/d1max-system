"""Supervised frontend-only ROS/Zenoh experiment. No PGO or robot connection.

The unmodified bag is played through the production adapter and frontend.
Each run has its own configuration, state log, raw trajectory and saved map.
Only children created here are signalled. Existing results are never replaced.
"""
import argparse
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
import open3d as o3d
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger
import yaml

WS = Path('/home/dndx/d1max_nav_ws')
ROOT = Path(__file__).resolve().parent


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--config', type=Path, default=WS/'src/d1max_slam/config/faster_lio_airy96.yaml')
    parser.add_argument('--rate', type=float, default=2.)
    parser.add_argument('--duration', type=float, default=0., help='Sensor seconds, zero means entire bag')
    args = parser.parse_args()
    out = args.output.resolve()
    if out.exists() or out.parent != WS/'maps/runs' or args.rate <= 0 or args.duration < 0:
        raise RuntimeError('Need new maps/runs directory and valid rate/duration')
    if os.environ.get('RMW_IMPLEMENTATION') != 'rmw_zenoh_cpp' or os.environ.get('ROS_DOMAIN_ID') != '219':
        raise RuntimeError('Frontend tests require local Zenoh in isolated domain 219')
    lock = (ROOT/'run.lock').open('a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit(): continue
        try:
            if (proc/'exe').resolve().name in ('run_mapping_online','dual_lidar_adapter','alaserPGO'):
                environment = (proc/'environ').read_bytes().split(b'\0')
                if b'ROS_DOMAIN_ID=219' in environment:
                    raise RuntimeError(f'Existing mapping process {proc.name} in test domain; not touched')
        except (PermissionError, FileNotFoundError): pass
    bag = args.bag.resolve()
    metadata = yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    duration = metadata['duration']['nanoseconds']/1e9
    topics = ['/front_lidar','/rear_lidar','/front_lidar/imu']
    counts = {t['topic_metadata']['name']:t['message_count'] for t in metadata['topics_with_message_count']}
    if any(counts.get(t,0) == 0 for t in topics): raise RuntimeError('Both LiDARs and front IMU are required')
    source_stats = {n:[(bag/n).stat().st_size,(bag/n).stat().st_mtime_ns] for n in metadata['relative_file_paths']}
    out.mkdir()
    (out/'config').mkdir(); (out/'body_samples').mkdir(); (out/'legacy_logs_before').mkdir()
    for name in ('traj.txt','imu_.txt'):
        p = WS/'src/faster_lio/Log'/name
        if p.is_file(): shutil.copy2(p,out/'legacy_logs_before'/name)
    config = yaml.safe_load(args.config.read_text())
    params = config['laserMapping']['ros__parameters']
    params['diagnostics'] = {'state_log_path':str(out/'frontend_state.csv')}
    params['publish']['scan_bodyframe_pub_en'] = True
    params['publish']['scan_effect_pub_en'] = False
    params['publish']['path_publish_en'] = False
    params['path_save_en'] = True
    (out/'config/frontend.yaml').write_text(yaml.safe_dump(config,sort_keys=False))
    shutil.copy2(WS/'src/d1max_slam/launch/mapping.launch.py',out/'config/mapping.launch.py')
    shutil.copy2(WS/'src/d1max_slam/config/dual_lidar.yaml',out/'config/dual_lidar.yaml')
    binaries = [WS/'build/faster_lio/libfaster_lio_lib.so',WS/'install/faster_lio/lib/faster_lio/run_mapping_online',
                WS/'install/d1max_slam/lib/d1max_slam/dual_lidar_adapter']
    write_json(out/'manifest.json',dict(bag=str(bag),rmw=os.environ['RMW_IMPLEMENTATION'],domain=219,
        topics=topics,rate=args.rate,requested_duration=args.duration,source_stats=source_stats,
        loop_closure_enabled=False,height_lock_enabled=False,source_config=str(args.config.resolve()),
        binary_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in binaries}))
    children = {}; handles = []; node = None; cancelled = False; subscriptions = []
    result = {'complete':False,'loop_closure_enabled':False,'height_lock_enabled':False}

    def start(name, command):
        handle = (out/f'{name}.log').open('wb'); handles.append(handle)
        children[name] = subprocess.Popen(command,cwd=out,stdout=handle,stderr=subprocess.STDOUT,start_new_session=True)
        return children[name]

    def stop(name):
        p = children.get(name)
        if p is None or p.poll() is not None: return
        os.killpg(p.pid,signal.SIGINT)
        try: p.wait(timeout=12)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid,signal.SIGTERM)
            try: p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid,signal.SIGKILL); p.wait(timeout=5)
                result.setdefault('forced_cleanup',[]).append(name)

    def cancel(_sig,_frame):
        nonlocal cancelled
        cancelled = True

    signal.signal(signal.SIGINT,cancel); signal.signal(signal.SIGTERM,cancel)
    try:
        try:
            with socket.create_connection(('127.0.0.1',7447),timeout=.5): pass
        except OSError:
            router = Path(os.environ['D1MAX_ROS2_DIR'])/'local/opt/ros/humble/lib/rmw_zenoh_cpp/rmw_zenohd'
            start('router',[str(router)])
            until = time.monotonic()+8
            while True:
                try:
                    with socket.create_connection(('127.0.0.1',7447),timeout=.2): break
                except OSError:
                    if time.monotonic()>until: raise RuntimeError('Local Zenoh router failed')
                    time.sleep(.1)
        rclpy.init(); node = rclpy.create_node('frontend_validation_observer')
        trajectory = []; poses = {}; sampled = []; pending = []
        last_sample = -math.inf; last_cloud = None; first_stamp = None; last_data = time.monotonic()

        def on_odom(m):
            nonlocal first_stamp,last_data
            t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
            p=m.pose.pose.position; q=m.pose.pose.orientation
            row=[t,p.x,p.y,p.z,q.x,q.y,q.z,q.w]
            if not all(math.isfinite(v) for v in row): raise RuntimeError('Nonfinite odometry')
            trajectory.append(row); poses[round(t,5)] = row
            if first_stamp is None: first_stamp=t
            last_data=time.monotonic()

        def on_cloud(m):
            nonlocal last_cloud,last_sample
            t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
            last_cloud=m
            if len(sampled)+len(pending)<9 or t-last_sample>=30.:
                pending.append(m); last_sample=t

        def save_cloud(m):
            t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
            names=['x','y','z']; offsets=[next(f.offset for f in m.fields if f.name==n) for n in names]
            data=np.ndarray((m.height,m.width),dtype=np.dtype(dict(names=names,formats=['<f4']*3,
                offsets=offsets,itemsize=m.point_step)),buffer=m.data,strides=(m.row_step,m.point_step)).ravel()
            xyz=np.column_stack([data[n] for n in names]); xyz=xyz[np.isfinite(xyz).all(1)&(np.linalg.norm(xyz,axis=1)<25)]
            c=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz)).voxel_down_sample(.12)
            name=f'{len(sampled):04d}.pcd'
            if not o3d.io.write_point_cloud(str(out/'body_samples'/name),c): raise RuntimeError('Cloud save failed')
            sampled.append({'stamp':t,'file':name})

        subscriptions.append(node.create_subscription(Odometry,'/d1max/faster_lio/odometry',on_odom,100))
        subscriptions.append(node.create_subscription(PointCloud2,'/d1max/faster_lio/cloud_registered_body',on_cloud,10))
        save_client=node.create_client(Trigger,'/d1max/slam/save')
        start('frontend',['ros2','launch','d1max_slam','mapping.launch.py','backend:=faster_lio',
            'lidar_mode:=dual','use_rviz:=false',f'output_dir:={out}',f'faster_lio_config:={out}/config/frontend.yaml'])
        until=time.monotonic()+25
        while not (all(node.count_subscribers(t)>0 for t in topics) and node.count_subscribers('/d1max/slam/points')>0 and save_client.service_is_ready()):
            if cancelled: raise KeyboardInterrupt()
            if time.monotonic()>until or children['frontend'].poll() is not None: raise RuntimeError('Frontend failed to become ready')
            rclpy.spin_once(node,timeout_sec=.1)
        player=start('playback',['ros2','bag','play',str(bag),'--rate',str(args.rate),'--read-ahead-queue-size','400',
                                '--disable-keyboard-controls','--topics',*topics])
        begin=time.monotonic(); last_data=begin; last_status=0.; segment=False
        while player.poll() is None:
            if cancelled: raise KeyboardInterrupt()
            rclpy.spin_once(node,timeout_sec=.02)
            while pending: save_cloud(pending.pop(0))
            elapsed=0. if not trajectory else trajectory[-1][0]-trajectory[0][0]
            if args.duration and elapsed>=args.duration:
                segment=True; stop('playback'); break
            if children['frontend'].poll() is not None: raise RuntimeError('Frontend exited unexpectedly')
            if time.monotonic()-last_data>35: raise RuntimeError('No frontend odometry for 35 seconds')
            if time.monotonic()-begin>duration/args.rate+180: raise RuntimeError('Playback exceeded time guard')
            if time.monotonic()-last_status>10:
                status=dict(stage='mapping',sensor_seconds=elapsed,poses=len(trajectory),xyz=trajectory[-1][1:4] if trajectory else None)
                write_json(out/'status.json',status); print(json.dumps(status),flush=True); last_status=time.monotonic()
        if not segment and player.returncode: raise RuntimeError(f'Playback failed: {player.returncode}')
        until=time.monotonic()+8
        while time.monotonic()<until:
            rclpy.spin_once(node,timeout_sec=.05)
            while pending: save_cloud(pending.pop(0))
        if last_cloud is not None: save_cloud(last_cloud)
        future=save_client.call_async(Trigger.Request()); until=time.monotonic()+40
        while not future.done() and time.monotonic()<until: rclpy.spin_once(node,timeout_sec=.05)
        saved=future.result() if future.done() else None
        if not saved or not saved.success: raise RuntimeError('Frontend map save failed')
        result['save_message']=saved.message
        stop('frontend')
        for s in sampled: s['pose']=poses.get(round(s['stamp'],5))
        write_json(out/'body_samples/index.json',sampled)
        np.savetxt(out/'frontend_odometry.tum',np.asarray(trajectory),fmt='%.12f',header='t x y z qx qy qz qw')
        span=trajectory[-1][0]-trajectory[0][0]
        if not segment and span/duration < .98: raise RuntimeError('Incomplete bag coverage')
        if source_stats!={n:[(bag/n).stat().st_size,(bag/n).stat().st_mtime_ns] for n in source_stats}:
            raise RuntimeError('Source bag changed')
        result.update(complete=True,segment_only=segment,poses=len(trajectory),sensor_seconds=span,
            endpoint_delta_xyz=(np.array(trajectory[-1][1:4])-trajectory[0][1:4]).tolist(),
            ground_truth_accuracy_validated=False)
    except KeyboardInterrupt:
        result['error']='Interrupted'
    except Exception as exc:
        result['error']=str(exc); print('FAILED',exc,flush=True)
    finally:
        stop('playback'); stop('frontend')
        if node is not None: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        stop('router')
        for h in handles: h.close()
        write_json(out/'result.json',result)
        print(json.dumps(result,ensure_ascii=False),flush=True)
    return 0 if result['complete'] else 1


if __name__=='__main__':
    raise SystemExit(main())
