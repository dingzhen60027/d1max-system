#!/usr/bin/env python3
"""Bounded, read-only check of recorded AIRY beam cones versus device DIFOP.

This is not an extrinsic calibration. The cone equation is independent of
scene geometry and rotation around the scanner's native Z axis is unobservable.
"""
import argparse
import json
import math
import sqlite3
import struct
from pathlib import Path

import numpy as np
import yaml
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2


ROOT = Path('/home/dndx/d1max_nav_ws')
BAG = ROOT/'bags/slam_raw_20260917_171716_fe8f38'
HERE = Path(__file__).resolve().parent
RXY = math.hypot(.0075, .00664)
RZ = .04532


def decode_angles(payload, offset):
    return np.array([(-1 if payload[offset+3*i] else 1)*
                     struct.unpack_from('>H', payload, offset+3*i+1)[0]*.01
                     for i in range(96)])


def cone_angle_residual(points, ring, vertical, horizontal):
    # rho^2 = s^2 + Rxy^2 + 2*s*Rxy*cos(horizontal_adjustment),
    # where s is range*cos(vertical_angle), not the measured radial distance.
    rho = np.linalg.norm(points[:, :2], axis=1)
    h = np.deg2rad(horizontal[ring])
    s = -RXY*np.cos(h) + np.sqrt(np.maximum(rho*rho-RXY*RXY*np.sin(h)**2, 0))
    return np.arctan2(points[:, 2]-RZ, s)-np.deg2rad(vertical[ring])


def stats(residual):
    degrees = np.rad2deg(residual)
    return {'rmse_deg': float(np.sqrt(np.mean(degrees**2))),
            'median_deg': float(np.median(degrees)),
            'abs_p95_deg': float(np.quantile(np.abs(degrees), .95)),
            'abs_max_deg': float(np.max(np.abs(degrees)))}


def read_cloud(db, topic_id, target):
    row = db.execute('SELECT timestamp,data FROM messages WHERE topic_id=? AND timestamp>=? ORDER BY timestamp LIMIT 1',
                     (topic_id, target)).fetchone()
    msg = deserialize_message(row[1], PointCloud2)
    fields = {f.name: f for f in msg.fields}
    dtype = np.dtype({'names': ['x','y','z','ring'],
                      'formats': ['<f4','<f4','<f4','<u2'],
                      'offsets': [fields[k].offset for k in ['x','y','z','ring']],
                      'itemsize': msg.point_step})
    arr = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                     strides=(msg.row_step, msg.point_step)).reshape(-1)
    points = np.column_stack([arr[k] for k in ['x','y','z']]).astype(float)
    ring = arr['ring'].astype(int)
    ranges = np.linalg.norm(points, axis=1)
    valid = np.isfinite(points).all(axis=1) & (ranges>.7) & (ranges<30) & (ring<96)
    return points[valid], ring[valid], {
        'record_ns': row[0], 'header_ns': msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,
        'frame_id': msg.header.frame_id, 'raw_count': len(points),
        'selected_count': int(valid.sum())}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=HERE/'beam_frame_report.json')
    args = parser.parse_args()
    metadata = yaml.safe_load((BAG/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    db = sqlite3.connect((BAG/metadata['relative_file_paths'][0]).as_uri()+'?mode=ro', uri=True)
    topics = dict(db.execute('SELECT name,id FROM topics'))
    factory_path = BAG/'snapshots/extrinsics_20260917.json'
    factory = json.loads(factory_path.read_text())
    report = {'bag': str(BAG), 'factory_snapshot': str(factory_path), 'ros_initialized': False,
              'cloud_message_count': 4,
              'primary_sources': [
                  'https://github.com/RoboSense-LiDAR/rs_driver/blob/main/src/rs_driver/driver/decoder/decoder_RSAIRY.hpp',
                  'https://github.com/RoboSense-LiDAR/rs_driver/blob/main/src/rs_driver/driver/decoder/chan_angles.hpp',
                  'https://github.com/RoboSense-LiDAR/rs_driver/blob/main/src/rs_driver/driver/decoder/decoder_mech.hpp'],
              'model': {'lens_radial_offset_m': RXY, 'lens_z_offset_m': RZ,
                        'vertical_array_offset': 468, 'horizontal_array_offset': 756,
                        'ring_mapping': 'rank of vertical angle in increasing order, as official toUserChan'},
              'limitations': [
                  'Ray cones cannot observe arbitrary yaw around the scanner native Z axis.',
                  'This tests recorded cloud coordinates, not physical LiDAR-to-IMU calibration.',
                  'Matching the factory cone model cannot prove the internal IMU axes were not separately transformed.',
                  'Sampling four messages is not a full-session validation.'], 'sensors': []}
    for topic, ip in [('/front_lidar','192.168.1.200'),('/rear_lidar','192.168.2.200')]:
        packet = next(p for p in factory['raw_capture']['difop'] if p['src']==ip)
        payload = bytes.fromhex(packet['hex'])
        assert len(payload)==1248 and payload[:8].hex()=='a5ff005a11115555'
        v, h = decode_angles(payload,468), decode_angles(payload,756)
        indices = np.argsort(v)
        v,h = v[indices],h[indices]
        assert np.all(np.diff(v)>0), 'Duplicate elevations need explicit toUserChan handling'
        device = next(p for p in factory['factory_devices'] if p['src']==ip)
        R_imu_lidar = Rotation.from_quat(device['quaternion_xyzw']).as_matrix()
        t_imu_lidar = np.array(device['translation_m'])
        sensor = {'topic':topic,'serial_number':device['sn_hex'],
                  'difop_install_mode':payload[289],
                  'difop_pitch_yaw_roll_deg':list(struct.unpack_from('>fff',payload,1124)),
                  'vertical_angles_deg':v.tolist(), 'frames':[]}
        for seconds in [.65,1.5]:
            points,ring,info=read_cloud(db,topics[topic],metadata['starting_time']['nanoseconds_since_epoch']+int(seconds*1e9))
            # Broadly spread deterministic sample only for the small diagnostic fit.
            sel=np.arange(0,len(points),max(1,len(points)//12000))
            pp,rr=points[sel],ring[sel]
            def residual(params):
                R=Rotation.from_rotvec([params[0],params[1],0]).as_matrix()
                recovered=(pp-params[2:5])@R
                return cone_angle_residual(recovered,rr,v,h)
            fit=least_squares(residual,np.zeros(5),loss='linear',max_nfev=30,
                              xtol=1e-12,gtol=1e-12,ftol=1e-12)
            frame={**info,'requested_s':seconds,
                   'native_identity_cone_residual':stats(cone_angle_residual(points,ring,v,h)),
                   'if_cloud_were_transformed_by_DIFOP_imu_extrinsic_residual':
                       stats(cone_angle_residual((points-t_imu_lidar)@R_imu_lidar,ring,v,h)),
                   'fitted_raw_to_recorded_roll_pitch_rotvec_deg':np.rad2deg(fit.x[:2]).tolist(),
                   'fitted_raw_to_recorded_translation_m':fit.x[2:5].tolist(),
                   'fitted_cone_residual':stats(fit.fun),
                   'fit_success':bool(fit.success),'fit_sample_count':len(pp),
                   'fit_jacobian_singular_values':np.linalg.svd(fit.jac,compute_uv=False).tolist()}
            sensor['frames'].append(frame)
            print(json.dumps({'topic':topic,**frame},indent=2),flush=True)
        report['sensors'].append(sensor)
    db.close()
    output=args.output
    with output.open('x') as stream:
        json.dump(report,stream,indent=2)
    print(output)


if __name__=='__main__':
    main()
