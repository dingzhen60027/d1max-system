"""Re-run the actual SC-PGO node on saved BODY keyframes, never the whole bag.

Local Zenoh only. No SDK, no robot commands, no automatic process cleanup.
Outputs are NEW and separate from the source run.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry, Path as RosPath
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import UInt32
from scipy.spatial.transform import Rotation
import yaml


def pcd_message(path, stamp):
    with path.open('rb') as stream:
        fields = {}
        while True:
            line = stream.readline().decode('ascii').strip()
            if not line:
                raise RuntimeError(f'Invalid PCD header: {path}')
            if line.startswith('#'):
                continue
            key, _, value = line.partition(' ')
            fields[key] = value.split()
            if key == 'DATA':
                break
        if fields['DATA'] != ['binary'] or fields['FIELDS'] != ['x','y','z','intensity']:
            raise RuntimeError('Expected uncompressed XYZI body keyframe')
        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = 'body'
        msg.height = 1
        msg.width = int(fields['POINTS'][0])
        msg.point_step = 16
        msg.row_step = 16*msg.width
        msg.is_dense = True
        msg.fields = [PointField(name=name, offset=4*i, datatype=PointField.FLOAT32, count=1)
                      for i,name in enumerate(['x','y','z','intensity'])]
        msg.data = stream.read()
        if len(msg.data) != msg.row_step:
            raise RuntimeError('Truncated body keyframe')
        return msg


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('source', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    source, output = args.source.resolve(), args.output.resolve()
    if os.environ.get('RMW_IMPLEMENTATION') != 'rmw_zenoh_cpp' or os.environ.get('ROS_DOMAIN_ID') != '217':
        raise RuntimeError('Use the existing local Zenoh transport in offline domain 217')
    if output.parent != source or output.exists():
        raise RuntimeError('Output must be a NEW direct child of the source run')
    original = source/'sc_pgo'
    poses = np.loadtxt(original/'odom_poses.txt').reshape(-1,3,4)
    stamps = np.loadtxt(original/'times.txt')
    if len(poses) != len(stamps) or not all((original/f'Scans/{i:06d}.pcd').is_file() for i in range(len(poses))):
        raise RuntimeError('Incomplete source keyframes')
    binary = Path('/home/dndx/d1max_nav_ws/install/sc_pgo/lib/sc_pgo/alaserPGO')
    config_path = Path('/home/dndx/d1max_nav_ws/src/d1max_slam/config/sc_pgo_d1max.yaml')
    config = yaml.safe_load(config_path.read_text())
    params = config['/**']['ros__parameters']
    params.update(save_directory=str(output), keyframe_meter_gap=0.0, keyframe_deg_gap=0.0,
                  loop_closure_frequency=15.0, use_current_stamp_for_aft_pgo_odom=False,
                  save_keyframe_scans=False)
    output.mkdir()
    (output/'effective_config.yaml').write_text(yaml.safe_dump(config,sort_keys=False))
    (output/'manifest.json').write_text(json.dumps({'source':str(original),
        'input':'unchanged body keyframes and frontend poses', 'rmw':os.environ['RMW_IMPLEMENTATION'],
        'binary_sha256':hashlib.sha256(binary.read_bytes()).hexdigest()},indent=2)+'\n')
    # Never overwrite/copy huge source scans. Independent auditing can follow the link.
    (output/'input_scans').symlink_to(original/'Scans', target_is_directory=True)
    log = (output/'backend.log').open('wb')
    command = [str(binary),'--ros-args','--params-file',str(output/'effective_config.yaml'),
               '-r','__node:=d1max_sc_pgo_reprocess']
    for old, new in [('aft_mapped_to_init','/d1max/reprocess/odom'),
                     ('velodyne_cloud_registered_local','/d1max/reprocess/cloud'),
                     ('aft_pgo_odom','/d1max/pgo/odometry'),('aft_pgo_path','/d1max/pgo/path'),
                     ('aft_pgo_map','/d1max/pgo/map'),('pgo_loop_count','/d1max/pgo/loop_count')]:
        command += ['-r', f'/{old}:={new}']
    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
    rclpy.init()
    node = rclpy.create_node('d1max_keyframe_reprocessor')
    odom_pub = node.create_publisher(Odometry,'/d1max/reprocess/odom',20)
    cloud_pub = node.create_publisher(PointCloud2,'/d1max/reprocess/cloud',20)
    state = {'stage':'starting','sent':0,'keyframes':0,'loops':0}
    def path_callback(msg):
        state['keyframes']=len(msg.poses)
        if msg.poses:
            p=msg.poses[-1].pose.position
            state['optimized_xyz']=[p.x,p.y,p.z]
    subscriptions = [node.create_subscription(RosPath,'/d1max/pgo/path',path_callback,10),
        node.create_subscription(UInt32,'/d1max/pgo/loop_count',lambda m:state.update(loops=m.data),
            QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL))]
    def spin_for(seconds):
        until=time.monotonic()+seconds
        while time.monotonic()<until:
            if process.poll() is not None:
                raise RuntimeError('SC-PGO exited; inspect backend.log')
            rclpy.spin_once(node,timeout_sec=.02)
    try:
        deadline=time.monotonic()+15
        while not odom_pub.get_subscription_count() or not cloud_pub.get_subscription_count():
            if time.monotonic()>deadline: raise RuntimeError('SC-PGO not ready')
            spin_for(.1)
        for i,(matrix,t) in enumerate(zip(poses,stamps)):
            odom=Odometry()
            ns=int(round(t*1e9));odom.header.stamp.sec=ns//1000000000;odom.header.stamp.nanosec=ns%1000000000
            odom.header.frame_id='camera_init';odom.child_frame_id='body'
            odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z=matrix[:,3].tolist()
            q=Rotation.from_matrix(matrix[:,:3]).as_quat()
            odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w=q.tolist()
            odom_pub.publish(odom)
            cloud_pub.publish(pcd_message(original/f'Scans/{i:06d}.pcd',odom.header.stamp))
            state.update(stage='reprocessing',sent=i+1)
            # Slow the final approach enough for independent geometrical confirmations.
            spin_for(.8 if i>len(poses)-55 else .15)
            deadline=time.monotonic()+10
            while state['sent']-state['keyframes']>12:
                if time.monotonic()>deadline: raise RuntimeError('Backend is not keeping up')
                spin_for(.1)
            (output/'status.json').write_text(json.dumps(state,indent=2)+'\n')
        spin_for(15)
        state['stage']='saving'
        process.send_signal(signal.SIGINT);process.wait(timeout=40)
        if process.returncode or not (output/'optimized_map.pcd').is_file():
            raise RuntimeError('SC-PGO did not save successfully')
        final=np.atleast_2d(np.loadtxt(output/'optimized_poses.txt'))
        if len(final)!=len(poses): raise RuntimeError('Dropped source keyframes')
        # Scans was created empty by the node; use an audit symlink under a separate name.
        state.update(stage='complete',keyframes=len(final))
        (output/'status.json').write_text(json.dumps(state,indent=2)+'\n')
        print(json.dumps(state,indent=2))
    finally:
        node.destroy_node();rclpy.shutdown()
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:process.wait(timeout=30)
            except subprocess.TimeoutExpired:process.terminate();process.wait(timeout=10)
        log.close()


if __name__=='__main__':
    main()
