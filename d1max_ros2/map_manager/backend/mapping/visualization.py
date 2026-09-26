"""Isolated replay display adapter for the EXISTING Foxglove layout; not localization."""
import argparse
import json
import struct
from pathlib import Path
import time
import numpy as np
import open3d as o3d
import yaml
from .display_session import active_session


class SessionChanged(Exception):
    pass


def select_map(manifest):
    ready = {a['file'] for a in manifest.get('artifacts', []) if a.get('status') == 'complete'}
    # A no-loop export is not a corrected map. Display ONE explicit stage only.
    if 'optimized.pcd' in ready and (manifest.get('loop_counts') or {}).get('gnc_inliers', 0) > 0:
        return 'optimized.pcd'
    return 'raw.pcd' if 'raw.pcd' in ready else None


def read_live_cloud(path):
    # Native x86 writer atomically replaces the complete file; no partial points.
    with Path(path).open('rb') as stream:
        header=stream.read(24)
        if len(header)!=24 or header[:8]!=b'D1MG0001':raise ValueError('Invalid live map header')
        count,stamp=struct.unpack('<Qd',header[8:])
        if count>2000000 or not np.isfinite(stamp):raise ValueError('Invalid live map limits')
        data=stream.read(count*12+1)
        if len(data)!=count*12:raise ValueError('Incomplete live map payload')
    xyz=np.frombuffer(data,dtype='<f4').reshape(-1,3)
    if not np.isfinite(xyz).all():raise ValueError('Nonfinite live map')
    return xyz,stamp


def cloud_xyz(message, stride=4):
    fields = {f.name:f for f in message.fields}
    arrays=[]
    for key in ('x','y','z'):
        f=fields[key]
        if f.datatype != 7:
            raise ValueError('Expected float32 XYZ')
        arrays.append(np.ndarray((message.height,message.width),dtype='>f4' if message.is_bigendian else '<f4',
            buffer=message.data,offset=f.offset,strides=(message.row_step,message.point_step)).ravel()[::stride])
    xyz=np.column_stack(arrays)
    return xyz[np.isfinite(xyz).all(axis=1)]


def main():
    import rclpy
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2,PointField
    from geometry_msgs.msg import PoseStamped,TransformStamped
    from nav_msgs.msg import Path as RosPath
    from std_msgs.msg import String
    from rosgraph_msgs.msg import Clock
    from tf2_ros import StaticTransformBroadcaster
    from scipy.spatial.transform import Rotation
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session',type=Path)
    parser.add_argument('--rate',type=float,default=1.)
    parser.add_argument('--saved-map',action='store_true',help='Explicit result review; default is native live SLAM only')
    parser.add_argument('--follow-active',action='store_true')
    args=parser.parse_args()
    session=args.session.resolve(); config=yaml.safe_load((session/'task.yaml').read_text())
    bag=Path(config['input']['bag_path'])
    metadata=yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    if not 0 < args.rate <= 4:raise ValueError('rate must be 0..4')
    rclpy.init(); node=rclpy.create_node('d1max_mola_offline_view',enable_rosout=False,start_parameter_services=False)
    latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
    maps=node.create_publisher(PointCloud2,'/d1max/localization/map_cloud',latched)
    trajectory=node.create_publisher(RosPath,'/d1max/localization/trajectory',latched)
    front=node.create_publisher(PointCloud2,'/front_lidar',1)
    rear=node.create_publisher(PointCloud2,'/rear_lidar',1)
    clock=node.create_publisher(Clock,'/clock',1)
    status=node.create_publisher(String,'/d1max/mola/replay_status',1)
    # Existing D1 sensor display extrinsics; do not publish these into the live ROS graph.
    front_q=[-.499867275,.503186620,.497953310,.498977388]
    rear_q=[1.,0.,0.,0.]
    broadcaster=StaticTransformBroadcaster(node)
    transforms=[]
    for parent,child,xyz,quat in [('d1max_lidar','rslidar_head',[0.,0.,0.],front_q),
        ('rslidar_head','rslidar_tail',[0.,0.,-.7323],rear_q)]:
        tf=TransformStamped();tf.header.frame_id=parent;tf.child_frame_id=child
        tf.header.stamp=node.get_clock().now().to_msg()
        tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z=xyz
        tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w=quat
        transforms.append(tf)
    broadcaster.sendTransform(transforms)
    def pointcloud(xyz,frame):
        m=PointCloud2();m.header.frame_id=frame;m.header.stamp=node.get_clock().now().to_msg()
        m.width=len(xyz);m.height=1;m.is_dense=True;m.point_step=12;m.row_step=12*len(xyz)
        m.fields=[PointField(name=k,offset=i*4,datatype=7,count=1) for i,k in enumerate(('x','y','z'))]
        m.data=np.asarray(xyz,dtype='<f4').tobytes();return m
    map_message=None; path_message=None; loaded=None; poses=None; mode='waiting_for_native_slam'
    live_stamp=None; mapping_status='starting'; native_status={}
    def refresh_map():
        nonlocal map_message,path_message,loaded,poses,mode,live_stamp,mapping_status,native_status
        try:
            if args.follow_active and active_session(Path(__file__).resolve().parents[2])!=session:
                maps.publish(pointcloud(np.empty((0,3)),'d1max_loc_map'))
                empty=RosPath();empty.header.frame_id='d1max_loc_map';trajectory.publish(empty)
                raise SessionChanged()
            manifest=json.loads((session/'manifest.json').read_text())
            mapping_status=manifest.get('status','unknown')
            filename=select_map(manifest) if args.saved_map else None
            path=session/filename if filename else session/'live/global.bin'
            signature=(str(path),path.stat().st_mtime_ns) if path.is_file() else None
            if signature and signature!=loaded:
                if args.saved_map:
                    if not filename:return
                    c=o3d.io.read_point_cloud(str(path)).remove_non_finite_points().voxel_down_sample(.12)
                    xyz=np.asarray(c.points)
                    tum=session/('optimized_lc_final.tum' if filename=='optimized.pcd' else 'trajectory.tum')
                    poses=np.atleast_2d(np.loadtxt(tum)) if tum.is_file() else None
                    mode='saved_result_review'
                else:
                    xyz,live_stamp=read_live_cloud(path)
                    tum=session/'live/trajectory.txt'
                    rows=np.atleast_2d(np.loadtxt(tum)) if tum.is_file() and tum.stat().st_size else None
                    poses=None
                    if rows is not None and rows.shape[1]==7:
                        rows=rows[rows[:,0]<=live_stamp]
                        if len(rows):poses=np.column_stack((rows[:,:4],Rotation.from_euler('ZYX',rows[:,4:7]).as_quat()))
                    status_path=session/'live/status.json'
                    if status_path.is_file():native_status=json.loads(status_path.read_text())
                    mode='native_live_global'
                step=max(1,int(np.ceil(len(xyz)/450000)))
                map_message=pointcloud(xyz[::step],'d1max_loc_map')
                path_message=RosPath();path_message.header.frame_id='d1max_loc_map'
                if poses is not None:
                    poses=poses[np.argsort(poses[:,0])]
                    for row in poses[::max(1,len(poses)//6000)]:
                        p=PoseStamped();p.header.frame_id='d1max_loc_map'
                        p.pose.position.x,p.pose.position.y,p.pose.position.z=map(float,row[1:4])
                        p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z,p.pose.orientation.w=map(float,row[4:8])
                        path_message.poses.append(p)
                loaded=signature
                print(json.dumps({'map':str(path),'display_points':len(xyz[::step]),'source_stamp':live_stamp,'mode':mode}),flush=True)
            if map_message is not None:
                map_message.header.stamp=node.get_clock().now().to_msg();maps.publish(map_message)
                path_message.header.stamp=map_message.header.stamp;trajectory.publish(path_message)
            status.publish(String(data=json.dumps({'mode':mode,'replay_latched':True,'session':session.name,
                'mapping_stage':manifest.get('stage'),'mapping_status':manifest.get('status'),
                'control_enabled':False,'layout_changed':False,'overlay':False,
                'map_file':str(path),'final_pcd_loaded':args.saved_map,'native':native_status,
                'loop_counts':manifest.get('loop_counts'),'quality':manifest.get('quality'),
                'left':'incremental native SLAM global map','right':'sensor replay following SLAM time'})))
        except (OSError,ValueError,KeyError) as exc:
            print('map refresh: '+str(exc),flush=True)
    node.create_timer(.5,refresh_map)
    try:
        while rclpy.ok():
            reader=rosbag2_py.SequentialReader()
            reader.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id=metadata['storage_identifier']),rosbag2_py.ConverterOptions('',''))
            reader.set_filter(rosbag2_py.StorageFilter(topics=['/front_lidar','/rear_lidar']))
            first_time=None;wall_start=time.monotonic();last={};frames=0
            while reader.has_next() and rclpy.ok():
                topic,data,t=reader.read_next()
                if first_time is None:first_time=t
                message=deserialize_message(data,PointCloud2)
                source_time=message.header.stamp.sec+message.header.stamp.nanosec*1e-9
                if args.saved_map:
                    due=wall_start+(t-first_time)*1e-9/args.rate
                    while time.monotonic()<due and rclpy.ok():rclpy.spin_once(node,timeout_sec=max(0.,min(.05,due-time.monotonic())))
                else:
                    # Do not invent a trajectory/replay speed independent of SLAM.
                    # Wait for the estimator's actual processed source timestamp.
                    while rclpy.ok() and (live_stamp is None or source_time>live_stamp+.11):
                        rclpy.spin_once(node,timeout_sec=.05)
                    if live_stamp is not None and source_time<live_stamp-.5:continue
                if time.monotonic()-last.get(topic,0)<.19:continue
                last[topic]=time.monotonic()
                clock.publish(Clock(clock=message.header.stamp))
                xyz=cloud_xyz(message)
                (front if topic=='/front_lidar' else rear).publish(pointcloud(xyz,message.header.frame_id))
                frames+=1
                if frames%100==0:print(f'replay frames={frames} elapsed={(t-first_time)*1e-9:.1f}s mode={mode}',flush=True)
                rclpy.spin_once(node,timeout_sec=0)
            if not args.saved_map:
                print('End of input: retaining the live global map, no loop or final-PCD substitution.',flush=True)
                while rclpy.ok():rclpy.spin_once(node,timeout_sec=.2)
            print('Saved-map review replay reached end; repeating display only.',flush=True)
    except SessionChanged:
        return 75  # Supervisor replaces only this publisher, not bridge/router or layout.
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':raise SystemExit(main())
