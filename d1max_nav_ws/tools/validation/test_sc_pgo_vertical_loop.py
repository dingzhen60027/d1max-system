"""Isolated synthetic ROS regression. Requires Zenoh, domain 218, no robot."""
import json
import math
import os
from pathlib import Path
import random
import signal
import struct
import subprocess
import tempfile
import time

import rclpy
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from validate_sc_pgo_loop import (
    ClosedLoopValidator, cloud_message, make_environment, make_square_poses,
    quaternion_from_yaw,
)


def main():
    if os.environ.get('RMW_IMPLEMENTATION') != 'rmw_zenoh_cpp' or os.environ.get('ROS_DOMAIN_ID') != '218':
        raise RuntimeError('Synthetic test requires Zenoh and isolated domain 218')
    output = Path(tempfile.mkdtemp(prefix='d1max-large-loop-regression-'))
    log = (output/'backend.log').open('wb')
    workspace = Path(__file__).resolve().parents[2]
    process = subprocess.Popen([
        str(workspace/'install/sc_pgo/lib/sc_pgo/alaserPGO'), '--ros-args',
        '--params-file', str(workspace/'src/d1max_slam/config/sc_pgo_d1max.yaml'),
        '-p', f'save_directory:={output}', '-p', 'loop_closure_frequency:=15.0',
        '-p', 'keyframe_meter_gap:=0.0', '-p', 'keyframe_deg_gap:=0.0',
        '-p', 'loop_min_time_separation:=0.0', '-p', 'map_publish_frequency:=0.2',
        '-p', 'loop_confirmation_count:=1', '-p', 'loop_min_keyframe_separation:=60',
        '-p', 'loop_accept_cooldown_keyframes:=5',
        '-r', '/aft_mapped_to_init:=/validation/odom',
        '-r', '/velodyne_cloud_registered_local:=/validation/cloud',
        '-r', '/aft_pgo_path:=/validation/pgo/path',
        '-r', '/pgo_loop_count:=/validation/pgo/loop_count'], stdout=log, stderr=subprocess.STDOUT)
    rclpy.init()
    node = ClosedLoopValidator()
    try:
        if not node.wait_for_backend():
            raise RuntimeError('Test backend unavailable')
        environment = make_environment()
        # The existing helper contains vertical walls ONLY, which leave
        # altitude weakly observable. This indoor height-regression fixture
        # must include the measured floor, as the real dual-LiDAR bag does.
        rng = random.Random(1709)
        environment += [(rng.uniform(-4,18), rng.uniform(-4,18), -.8, 5.)
                        for _ in range(10000)]
        poses = make_square_poses()
        for i, true_pose in enumerate(poses):
            alpha = i/(len(poses)-1)
            # Genuine vertical motion is deliberately present. Only the
            # accumulated error should be corrected, never all z coordinates.
            true_z = 1.2*math.sin(math.pi*alpha)
            stamp = node.get_clock().now().to_msg()
            odom = Odometry()
            odom.header = Header(stamp=stamp, frame_id='camera_init')
            odom.child_frame_id = 'body'
            odom.pose.pose.position.x = true_pose[0]+2*alpha
            odom.pose.pose.position.y = true_pose[1]-1.2*alpha
            odom.pose.pose.position.z = true_z-4.6*alpha
            q = quaternion_from_yaw(true_pose[2]+math.radians(8)*alpha)
            odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = q
            cloud = cloud_message(environment, true_pose, stamp)
            data = bytearray(cloud.data)
            for offset in range(8, len(data), cloud.point_step):
                struct.pack_into('<f', data, offset, struct.unpack_from('<f', data, offset)[0]-true_z)
            cloud.data = bytes(data)
            node.odom_pub.publish(odom); node.cloud_pub.publish(cloud)
            deadline = time.monotonic()+(.7 if i>len(poses)-20 else .16)
            while time.monotonic()<deadline:
                rclpy.spin_once(node, timeout_sec=.01)
        deadline = time.monotonic()+15
        while time.monotonic()<deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        final = node.latest_path.poses[-1].pose if node.latest_path else None
        error = math.inf if final is None else math.sqrt(final.position.x**2+final.position.y**2+final.position.z**2)
        observed_frames = len(node.latest_path.poses) if node.latest_path else 0
        peak_z = max((p.pose.position.z for p in node.latest_path.poses), default=0.) if node.latest_path else 0.
        report = {'passed': bool(node.loop_count>0 and error<0.30 and
                                observed_frames==len(poses) and peak_z>.5),
                  'loops':node.loop_count, 'published_frames':len(poses),
                  'optimized_frames':observed_frames, 'retained_height_peak_m':peak_z,
                  'closure_error_m':error, 'output':str(output), 'rmw':os.environ['RMW_IMPLEMENTATION']}
        (output/'test_result.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report,indent=2))
        return 0 if report['passed'] else 1
    finally:
        node.destroy_node(); rclpy.shutdown()
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try: process.wait(timeout=20)
            except subprocess.TimeoutExpired:
                process.terminate(); process.wait(timeout=10)
        log.close()


if __name__ == '__main__':
    raise SystemExit(main())
