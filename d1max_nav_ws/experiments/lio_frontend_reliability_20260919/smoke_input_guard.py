#!/usr/bin/env python3
"""Local synthetic-input integration test, NOT a mapping accuracy benchmark."""
import argparse
from array import array
import json
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import tempfile
import time

import numpy as np
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2, PointField
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import yaml

WS = Path('/home/dndx/d1max_nav_ws')
ROOT = Path(__file__).resolve().parent


def cloud(stamp_ns):
    data = np.zeros(256, dtype={'names': ['x','y','z','intensity','ring','time'],
                    'formats': ['<f4','<f4','<f4','<f4','<u2','<f4'],
                    'offsets': [0,4,8,12,16,20], 'itemsize': 24})
    data['x'], data['y'] = np.tile(np.linspace(1, 3, 16),16), np.repeat(np.linspace(-2,2,16),16)
    data['z'], data['intensity'] = -1., 10.
    data['ring'], data['time'] = np.arange(256)%16, np.linspace(0, 95, 256)
    msg = PointCloud2()
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
    msg.header.frame_id = 'synthetic_lidar'
    msg.height, msg.width, msg.point_step, msg.row_step = 1, 256, 24, 256*24
    msg.fields = [PointField(name=n, offset=o, count=1, datatype=d) for n,o,d in
                  [('x',0,7),('y',4,7),('z',8,7),('intensity',12,7),('ring',16,4),('time',20,7)]]
    msg.is_dense, msg.data = True, array('B', data.tobytes())
    return msg


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=['observe','fail_closed'], required=True)
    args = parser.parse_args()
    if os.environ.get('RMW_IMPLEMENTATION') != 'rmw_zenoh_cpp' or os.environ.get('ROS_DOMAIN_ID') != '220':
        raise RuntimeError('Synthetic test requires local Zenoh and isolated ROS domain 220')
    binary = ROOT/'install/faster_lio/lib/faster_lio/run_mapping_online'
    if not binary.is_file():
        raise RuntimeError('Build isolated overlay first')
    # Legacy LIO logs are shared by all binaries built from this source tree.
    for entry in Path('/proc').iterdir():
        try:
            if entry.name.isdigit() and (entry/'exe').resolve().name == 'run_mapping_online':
                raise RuntimeError('Another LIO is running; refusing shared legacy-log collision')
        except (FileNotFoundError, PermissionError):
            pass
    out = Path(tempfile.mkdtemp(prefix='guard_'+args.mode+'_', dir=ROOT))
    legacy = WS/'src/faster_lio/Log'
    for filename in ['traj.txt','imu_.txt']:
        if (legacy/filename).is_file():
            shutil.copy2(legacy/filename, out/filename)
    config = yaml.safe_load((WS/'src/d1max_slam/config/faster_lio_airy96.yaml').read_text())
    p = config['laserMapping']['ros__parameters']
    p['common'].update(lid_topic='/lio_guard_test/points', imu_topic='/lio_guard_test/imu', time_sync_en=False)
    p['preprocess'].update(lidar_type=2, time_scale=1., scan_line=192)
    p['mapping'].update(extrinsic_est_en=False, extrinsic_T=[0.,0.,0.], extrinsic_R=[1.,0.,0.,0.,1.,0.,0.,0.,1.])
    p['mapping']['imu_continuity'] = dict(mode=args.mode, warning_gap_sec=.015, maximum_gap_sec=.030)
    p['pcd_save']['pcd_save_en'], p['path_save_en'] = False, False
    p['diagnostics'] = dict(input_log_path=str(out/'input.csv'))
    (out/'config.yaml').write_text(yaml.safe_dump(config))
    processes, streams, statuses, poses = [], [], [], []
    result = dict(mode=args.mode, synthetic=True, accuracy_test=False, passed=False, output=str(out))
    node = None

    def start(name, command):
        stream = (out/(name+'.log')).open('x')
        streams.append(stream)
        child = subprocess.Popen(command, cwd=out, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(child)
        return child

    def spin(seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.005)

    try:
        try:
            with socket.create_connection(('127.0.0.1',7447),timeout=.2):
                pass
        except OSError:
            router = Path(os.environ['D1MAX_ROS2_DIR'])/'local/opt/ros/humble/lib/rmw_zenoh_cpp/rmw_zenohd'
            start('router',[str(router)])
            time.sleep(.5)
        rclpy.init(args=[])
        node = rclpy.create_node('synthetic_lio_guard_test')
        imu_pub = node.create_publisher(Imu,'/lio_guard_test/imu',1000)
        cloud_pub = node.create_publisher(PointCloud2,'/lio_guard_test/points',20)
        status_sub = node.create_subscription(String,'/mapping_input_status',
            lambda msg: statuses.append(json.loads(msg.data)), QoSProfile(depth=20,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        odom_sub = node.create_subscription(Odometry,'/Odometry', lambda msg: poses.append(msg.header.stamp),100)
        child = start('lio',[str(binary),'--ros-args','--params-file',str(out/'config.yaml')])
        deadline = time.monotonic()+15
        while not (imu_pub.get_subscription_count() and cloud_pub.get_subscription_count()):
            spin(.05)
            if time.monotonic()>deadline or child.poll() is not None:
                raise RuntimeError('Synthetic pipeline not ready')
        loaded = (Path('/proc')/str(child.pid)/'maps').read_text()
        libraries = sorted({line.split()[-1] for line in loaded.splitlines() if 'libfaster_lio_lib.so' in line})
        result['loaded_lio_libraries'] = libraries
        if not libraries or any(not str(Path(p).resolve()).startswith(str(ROOT/'install')) for p in libraries):
            raise RuntimeError('Smoke test did not load the isolated candidate library')
        last_imu_ns = 999_995_000_000

        def publish_scan(index, gap=False):
            nonlocal last_imu_ns
            begin = 1_000_000_000_000+index*100_000_000
            end = begin+100_000_000
            next_ns = end if gap else last_imu_ns+5_000_000
            while next_ns <= end:
                imu = Imu()
                imu.header.stamp.sec,imu.header.stamp.nanosec=divmod(next_ns,1_000_000_000)
                imu.linear_acceleration.z=9.81
                imu_pub.publish(imu)
                last_imu_ns=next_ns
                next_ns+=5_000_000
            spin(.01)
            cloud_pub.publish(cloud(begin))
            spin(.04)

        for i in range(30):
            publish_scan(i)
        spin(.3)
        if not any(s.get('checked_windows',0)>10 and not s['mapping_halted'] for s in statuses):
            raise RuntimeError('No healthy pre-fault integration windows')
        if len(poses)<2:
            raise RuntimeError('No ordinary odometry before injection')
        publish_scan(30,gap=True)
        spin(.3)
        after_gap=len(poses)
        for i in range(31,36):
            publish_scan(i)
        spin(1.2)
        fault_seen=any(s['reason']=='imu_gap' for s in statuses)
        if args.mode=='fail_closed':
            passed=fault_seen and statuses[-1]['mapping_halted'] and len(poses)==after_gap
        else:
            passed=fault_seen and not statuses[-1]['mapping_halted'] and len(poses)>after_gap
        result.update(passed=bool(passed),fault_seen=fault_seen,poses_after_gap=after_gap,
                      final_poses=len(poses),last_status=statuses[-1])
        if not passed:
            raise RuntimeError('Fault/continuation behavior did not match explicit mode')
    except Exception as error:
        result['error']=repr(error)
    finally:
        for child in reversed(processes):
            if child.poll() is None:
                os.killpg(child.pid,signal.SIGINT)
                try:
                    child.wait(timeout=12)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid,signal.SIGTERM)
                    child.wait(timeout=5)
            result.setdefault('returncodes',[]).append(child.returncode)
        if node is not None:
            node.destroy_node()
            rclpy.try_shutdown()
        for filename in ['traj.txt','imu_.txt']:
            if (out/filename).is_file():
                shutil.copy2(out/filename,legacy/filename)
        for stream in streams:
            stream.close()
        (out/'statuses.json').write_text(json.dumps(statuses,indent=2))
        (out/'result.json').write_text(json.dumps(result,indent=2))
        print(json.dumps(result,indent=2))
    return 0 if result['passed'] else 1


if __name__=='__main__':
    raise SystemExit(main())
