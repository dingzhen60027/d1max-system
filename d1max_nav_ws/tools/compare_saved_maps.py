#!/usr/bin/env python3
"""Read-only saved-map comparison on a private Zenoh router. No robot topics."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import yaml
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField

ROOT = Path('/home/dndx/d1max_nav_ws')
RUN = ROOT / 'maps/runs/20260919_125722_855277_central_imu_faster_lio_sc_pgo_zenoh'
OUT = ROOT / 'log/map_comparison/central_imu_20260919'
SOURCES = [('RAW - Faster-LIO before loop', RUN / 'd1max_map_20260919_131415.pcd'),
           ('PGO - SC-PGO after 4 loops', RUN / 'sc_pgo/optimized_map.pcd')]


def read_xyz(path):
    with path.open('rb') as stream:
        header = {}
        while True:
            line = stream.readline()
            if not line:
                raise ValueError('Missing PCD DATA')
            words = line.decode('ascii').strip().split()
            if not words or words[0].startswith('#'):
                continue
            header[words[0]] = words[1:]
            if words[0] == 'DATA':
                break
        fields = header['FIELDS']
        if (header['DATA'] != ['binary'] or set(header['TYPE']) != {'F'}
                or set(header['SIZE']) != {'4'} or set(header['COUNT']) != {'1'}):
            raise ValueError('Expected binary float32 scalar PCD')
        count = int(header['POINTS'][0])
        data = np.frombuffer(stream.read(), dtype='<f4')
        if data.size != count * len(fields):
            raise ValueError('PCD payload size mismatch')
        xyz = np.ascontiguousarray(data.reshape(count, len(fields))[:, [fields.index(v) for v in ('x', 'y', 'z')]])
        return xyz[np.isfinite(xyz).all(axis=1)]


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    clouds = [read_xyz(path) for _, path in SOURCES]
    low = np.minimum.reduce([c.min(axis=0) for c in clouds])
    high = np.maximum.reduce([c.max(axis=0) for c in clouds])
    center = (low + high) / 2
    distance = float(np.linalg.norm(high-low) * 1.15)
    router = OUT / 'router.json5'
    client = OUT / 'client.json5'
    base = {'scouting': {'multicast': {'enabled': False}, 'gossip': {'enabled': False}}}
    router.write_text(json.dumps(dict(base, mode='router', listen={'endpoints': ['tcp/127.0.0.1:7462']}, connect={'endpoints': []})))
    client.write_text(json.dumps(dict(base, mode='client', connect={'endpoints': ['tcp/127.0.0.1:7462'], 'exit_on_failure': True})))
    os.environ.update(RMW_IMPLEMENTATION='rmw_zenoh_cpp', ROS_DOMAIN_ID='24', ZENOH_SESSION_CONFIG_URI=str(client))
    for key in ('ZENOH_CONFIG_OVERRIDE', 'ZENOH_SESSION_CONFIG'):
        os.environ.pop(key, None)
    router_env = dict(os.environ, ZENOH_ROUTER_CONFIG_URI=str(router))
    router_exe = '/home/dndx/智元四足机器人D1 Max二次开发文档资料包v0.1.0/d1max_ros2/local/opt/ros/humble/lib/rmw_zenoh_cpp/rmw_zenohd'
    children = [subprocess.Popen([router_exe], env=router_env)]
    node = None
    try:
        time.sleep(.5)
        if children[0].poll() is not None:
            raise RuntimeError('Private comparison router failed')
        rclpy.init()
        node = rclpy.create_node('saved_map_comparison')
        publishers, messages, viewers = [], [], []
        evidence = []
        for i, ((label, source), xyz) in enumerate(zip(SOURCES, clouds)):
            topic = '/d1max/saved_comparison/' + ('raw' if i == 0 else 'pgo')
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
            publishers.append(node.create_publisher(PointCloud2, topic, qos))
            msg = PointCloud2()
            msg.header.frame_id = 'comparison_map'
            msg.height, msg.width = 1, len(xyz)
            msg.fields = [PointField(name=n, offset=k*4, datatype=PointField.FLOAT32, count=1) for k, n in enumerate(('x','y','z'))]
            msg.point_step, msg.row_step = 12, len(xyz)*12
            msg.is_dense = True
            msg.data = xyz.astype('<f4', copy=False).tobytes()
            messages.append(msg)
            display = {'Class': 'rviz_default_plugins/PointCloud2', 'Name': label, 'Enabled': True,
                'Position Transformer': 'XYZ', 'Color Transformer': 'AxisColor', 'Axis': 'Z',
                'Autocompute Value Bounds': False, 'Min Value': float(low[2]), 'Max Value': float(high[2]),
                'Use Fixed Frame': True, 'Style': 'Points', 'Size (Pixels)': 2, 'Decay Time': 0,
                'Topic': {'Value': topic, 'Depth': 1, 'Durability Policy': 'Transient Local', 'Reliability Policy': 'Reliable', 'History Policy': 'Keep Last'}}
            cfg = {'Panels': [{'Class':'rviz_common/Displays','Name':'Displays'}],
                'Visualization Manager': {'Class':'', 'Name':'root', 'Enabled':True,
                    'Global Options': {'Fixed Frame':'comparison_map','Background Color':'8; 12; 16','Frame Rate':20},
                    'Displays':[display, {'Class':'rviz_default_plugins/Grid','Name':'Z=0 reference (not fitted ground)', 'Enabled':True,'Plane':'XY','Cell Size':5,'Plane Cell Count':50,'Alpha':.25,'Color':'130; 130; 130'}],
                    'Tools':[{'Class':'rviz_default_plugins/Interact'},{'Class':'rviz_default_plugins/MoveCamera'},{'Class':'rviz_default_plugins/FocusCamera'}],
                    'Views':{'Current':{'Class':'rviz_default_plugins/Orbit','Name':label,'Target Frame':'comparison_map','Distance':distance,'Pitch':.50,'Yaw':.80,'Focal Point':dict(zip(('X','Y','Z'),map(float,center)))}}},
                'Window Geometry':{'Width':1000,'Height':800,'X':i*850,'Y':50}}
            config_path = OUT / ('01_RAW.rviz' if i == 0 else '02_PGO.rviz')
            config_path.write_text(yaml.safe_dump(cfg, sort_keys=False))
            viewers.append(subprocess.Popen(['/opt/ros/humble/lib/rviz2/rviz2','-d',str(config_path),'--ros-args','-r','__node:=saved_map_view_'+str(i)]))
            children.append(viewers[-1])
            evidence.append({'label':label, 'source':str(source), 'points':len(xyz), 'topic':topic, 'rviz':str(config_path)})
        def publish():
            for pub, msg in zip(publishers, messages):
                msg.header.stamp = node.get_clock().now().to_msg()
                pub.publish(msg)
        publish()
        node.create_timer(5., publish)
        (OUT/'comparison.json').write_text(json.dumps({'maps':evidence,'height_color_range_m':[float(low[2]),float(high[2])], 'coordinate_changes':False,'router':'127.0.0.1:7462'},indent=2))
        print(json.dumps(evidence), flush=True)
        while rclpy.ok() and any(p.poll() is None for p in viewers):
            if children[0].poll() is not None:
                raise RuntimeError('Comparison router exited')
            rclpy.spin_once(node, timeout_sec=.2)
    finally:
        for process in reversed(children):
            if process.poll() is None:
                process.terminate()
        for process in reversed(children):
            try:
                process.wait(timeout=4)
            except subprocess.TimeoutExpired:
                process.kill()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
