#!/usr/bin/env python3
"""Offline global registration on PCD/NPZ, no ROS/SDK or motion publishers.

Use --scan for independent sensor evidence. --crop-center is a registration
fixture cut from the map; explicitly reported as NOT independent sensor proof.
"""
import argparse
from dataclasses import asdict
import json
import os
from pathlib import Path
import sys
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map',required=True)
    parser.add_argument('--output',required=True)
    parser.add_argument('--cache',required=True)
    parser.add_argument('--scan')
    parser.add_argument('--bag')
    parser.add_argument('--topic',default='/d1max/localization/lio/deskewed')
    parser.add_argument('--static-raw',action='store_true',help='Explicitly allow static, identity-frame raw rays only')
    parser.add_argument('--crop-center',nargs=3,type=float)
    parser.add_argument('--yaw',type=float,default=.8)
    args=parser.parse_args()
    if sum(bool(v) for v in (args.scan,args.crop_center,args.bag))!=1:parser.error('choose --scan OR --crop-center OR --bag')
    for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):os.environ[key]='2'
    sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'src/d1max_localization'))
    import numpy as np
    import open3d as o3d
    from scipy.spatial.transform import Rotation
    from d1max_localization.global_registration import RegistrationConfig,file_sha256
    from d1max_localization.relocalization_worker import RegistrationWorker
    config=RegistrationConfig()
    truth=None
    if args.bag:
        import sqlite3
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import PointCloud2
        from nav_msgs.msg import Odometry
        from tf2_msgs.msg import TFMessage
        from d1max_localization.relocalization_cloud import pointcloud_xyz
        paths=sorted(Path(args.bag).glob('*.db3'))
        if len(paths)!=1:raise ValueError('probe_requires_single_sqlite_segment')
        connection=sqlite3.connect('file:'+str(paths[0].resolve())+'?mode=ro',uri=True)
        query='select m.data from messages m join topics t on m.topic_id=t.id where t.name=? order by m.timestamp'
        row=connection.execute(query+' limit 1 offset 2',(args.topic,)).fetchone()
        if row is None:raise ValueError('recorded_query_topic_missing')
        message=deserialize_message(row[0],PointCloud2)
        point_frame=message.header.frame_id
        if args.static_raw:
            if args.topic!='/d1max/localization/perception/rays_raw' or point_frame!='d1max_loc_lidar':
                raise ValueError('static_raw_requires_known_normalized_lidar_frame')
            identity=False
            for (blob,) in connection.execute(query,('/tf_static',)):
                for t in deserialize_message(blob,TFMessage).transforms:
                    if (t.header.frame_id=='d1max_loc_tracking' and t.child_frame_id==point_frame):
                        p,q=t.transform.translation,t.transform.rotation
                        identity=(abs(p.x)+abs(p.y)+abs(p.z)+abs(q.x)+abs(q.y)+abs(q.z)+abs(q.w-1)<1e-8)
            if not identity:raise ValueError('static_lidar_tracking_identity_not_recorded')
            odometry=[]
            for (blob,) in connection.execute(query,('/d1max/localization/lio/odometry',)):
                m=deserialize_message(blob,Odometry);v=m.twist.twist
                odometry.append([m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z])
                if np.linalg.norm([v.linear.x,v.linear.y,v.linear.z])>.08 or np.linalg.norm([v.angular.x,v.angular.y,v.angular.z])>.12:
                    raise ValueError('recorded_robot_not_stationary_raw_scan_not_valid_for_this_probe')
            if len(odometry)<3 or np.max(np.linalg.norm(np.asarray(odometry)-odometry[0],axis=1))>.06:
                raise ValueError('insufficient_recorded_static_motion_evidence')
        elif point_frame!='d1max_loc_tracking':raise ValueError('deskewed_query_not_in_tracking_frame')
        points=pointcloud_xyz(message,config.max_scan_points)
        combined=1
        if args.static_raw:
            # The live LIO adapter combines both lidars. A single rays_raw
            # packet is only ONE lidar, so compare a short static dual-lidar
            # window, never a motion-distorted, arbitrarily accumulated cloud.
            timestamp=message.header.stamp.sec+message.header.stamp.nanosec*1e-9
            clouds=[points]
            for (blob,) in connection.execute(query+' limit 8 offset 3',(args.topic,)):
                other=deserialize_message(blob,PointCloud2)
                at=other.header.stamp.sec+other.header.stamp.nanosec*1e-9
                if not 0<=at-timestamp<=.15:continue
                if other.header.frame_id!=point_frame:raise ValueError('recorded_static_window_frame_changed')
                clouds.append(pointcloud_xyz(other,config.max_scan_points))
            points=np.concatenate(clouds);combined=len(clouds)
        connection.close()
        evidence=dict(type='recorded_static_raw_scan' if args.static_raw else 'recorded_deskewed_scan',
            independent_sensor=True,bag=str(Path(args.bag).resolve()),bag_sha256=file_sha256(paths[0]),
            topic=args.topic,frame_id=point_frame,stamp_ns=message.header.stamp.sec*1000000000+message.header.stamp.nanosec,
            static_only_no_deskew=args.static_raw,recorded_identity_transform_verified=args.static_raw,
            combined_clouds=combined,maximum_static_window_s=.15 if args.static_raw else 0.)
    elif args.scan:
        if args.scan.endswith('.npz'):
            with np.load(args.scan,allow_pickle=False) as data:points=data['points'].copy()
        else:points=np.asarray(o3d.io.read_point_cloud(args.scan).points)
        evidence=dict(type='recorded_sensor_cloud',scan_sha256=file_sha256(args.scan),independent_sensor=True)
    else:
        source=np.asarray(o3d.io.read_point_cloud(args.map).points)
        center=np.asarray(args.crop_center)
        rotation=Rotation.from_euler('z',args.yaw).as_matrix()
        selected=source[np.linalg.norm(source-center,axis=1)<8.]
        points=(selected-center)@rotation
        truth=np.eye(4);truth[:3,:3]=rotation;truth[:3,3]=center
        evidence=dict(type='map_crop_fixture',independent_sensor=False,truth=truth.tolist())
    # Same bounded downsampling as a production query, not a fabricated pose.
    points=np.asarray(o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points)).voxel_down_sample(.12).points)
    if len(points)>config.max_scan_points:raise ValueError('offline_query_exceeds_live_budget')
    worker=RegistrationWorker(args.map,args.cache,config)
    started=time.monotonic();result=None;index=None
    try:
        while result is None:
            event=worker.poll()
            if event:
                if event['kind']=='index_ready':
                    index=event;worker.submit(points,'offline-query')
                else:result=event
            time.sleep(.02)
    finally:worker.close()
    record=dict(schema=1,mode='OFFLINE_NO_ROS_NO_SDK',source_map_sha256=file_sha256(args.map),
        config=asdict(config),query_points=len(points),evidence=evidence,index=index,result=result,
        total_elapsed_s=time.monotonic()-started,physical_acceptance=False,continuous_matcher_verified=False)
    if truth is not None and result.get('accepted'):
        actual=np.asarray(result['candidate']['transform'])
        record['fixture_error']=dict(position_m=float(np.linalg.norm(actual[:3,3]-truth[:3,3])),
            rotation_rad=float(Rotation.from_matrix(actual[:3,:3]@truth[:3,:3].T).magnitude()))
    destination=Path(args.output);destination.parent.mkdir(parents=True,exist_ok=True)
    destination.write_text(json.dumps(record,indent=2,allow_nan=False))
    print(json.dumps({k:record.get(k) for k in ('query_points','total_elapsed_s','fixture_error')}))
    print(json.dumps(dict(accepted=result.get('accepted',False),reason=result.get('reason'),index=index)))
    return 0 if result.get('accepted') else 2


if __name__=='__main__':raise SystemExit(main())
