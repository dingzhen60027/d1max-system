"""Idealized PCD observation + planar body model, guarded to loopback-only Zenoh.

This is not physical sensor/occlusion/terrain simulation and never connects SDK.
"""
import json
from array import array
import math
import os
import signal
import time
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial import cKDTree
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist, TransformStamped, PoseStamped
from nav_msgs.msg import Odometry, Path as RosPath
from sensor_msgs.msg import PointCloud2, PointField, LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from visualization_msgs.msg import Marker
from d1max_navigation.cloud_to_scan import project_points


def simulation_scan_parameters(body_height):
    # Same provisional geometry as d1max_robot.yaml: ignore only the first
    # 0.10 m above the assumed floor, NOT 0.35 m as the old torso-only slice did.
    # This is an engineering assumption, not calibrated clearance or permission
    # to cross objects below it. The simulator never authorizes real motion.
    ground_exclusion = .10
    if type(body_height) not in (int, float) or not math.isfinite(body_height) or not ground_exclusion < body_height <= 1.:
        raise ValueError('Invalid simulated body reference height')
    return {'min_height': ground_exclusion-body_height, 'max_height': .8}


def read_xyz(path):
    with Path(path).open('rb') as f:
        h={}
        while True:
            line=f.readline()
            if not line:raise ValueError('Invalid PCD header')
            words=line.decode('ascii').split()
            if words and not words[0].startswith('#'):h[words[0]]=words[1:]
            if words[:1]==['DATA']:break
        if h['DATA']!=['binary'] or h['FIELDS']!=['x','y','z','intensity'] or h['SIZE']!=['4']*4:raise ValueError('Expected binary XYZI')
        a=np.frombuffer(f.read(),dtype='<f4').reshape(-1,4)
        if len(a)!=int(h['POINTS'][0]):raise ValueError('PCD truncated')
        return a[np.isfinite(a[:,:3]).all(1),:3].copy()


def check_isolation():
    cfg=yaml.safe_load(Path(os.environ['ZENOH_SESSION_CONFIG_URI']).read_text())
    if (os.environ.get('RMW_IMPLEMENTATION')!='rmw_zenoh_cpp' or os.environ.get('ROS_DOMAIN_ID')!='24'
        or cfg.get('connect',{}).get('endpoints')!=['tcp/127.0.0.1:7464']
        or cfg.get('scouting',{}).get('multicast',{}).get('enabled') is not False
        or cfg.get('scouting',{}).get('gossip',{}).get('enabled') is not False
        or os.environ.get('ZENOH_CONFIG_OVERRIDE')):raise ValueError('Offline isolation required')


class Simulator(Node):
    def __init__(self):
        super().__init__('pct_scan_simulator');check_isolation()
        self.cfg=yaml.safe_load(Path(self.declare_parameter('config','').value).read_text())
        self.scan_slice=simulation_scan_parameters(self.cfg['body_height'])
        self.session=self.declare_parameter('session_id','').value
        self.xyz=read_xyz(self.cfg['map_pcd']);voxel=self.cfg['sensor_voxel']
        _,ids=np.unique(np.floor(self.xyz/voxel).astype(int),axis=0,return_index=True);self.xyz=self.xyz[ids]
        self.tree=cKDTree(self.xyz[:,:2]);self.ground=read_xyz(self.cfg['ground_pcd']);self.gt=cKDTree(self.ground[:,:2])
        grid=np.load(self.cfg['planning_grid']);self.free=grid['free'];self.origin=grid['origin'];self.res=float(grid['resolution'])
        self.x,self.y=self.cfg['initial_xy'];self.yaw=self.cfg['initial_yaw'];self.v=self.w=0.
        self.command=(0.,0.);self.received=0.;self.tick_at=time.monotonic();self.distance=0.;self.collisions=0;self.ticks=0
        self.obstacle=False;self.sensor_enabled=True;self.obstacle_center=None
        self.create_subscription(Twist,'/d1max/pct_scan/cmd_vel_safe',self.cmd,1)
        self.global_pub=self.create_publisher(Odometry,'/d1max/localization/odometry/global',qos_profile_sensor_data)
        self.local_pub=self.create_publisher(Odometry,'/d1max/localization/odometry/local',qos_profile_sensor_data)
        self.sensor_pub=self.create_publisher(Odometry,'/d1max/pct_scan/sensor_pose',qos_profile_sensor_data)
        self.cloud_pub=self.create_publisher(PointCloud2,'/d1max/pct_scan/cloud_map',qos_profile_sensor_data)
        self.scan_pub=self.create_publisher(LaserScan,'/d1max/pct_scan/scan',qos_profile_sensor_data)
        self.status_pub=self.create_publisher(String,'/d1max/pct_scan/simulation_status',1)
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub=self.create_publisher(PointCloud2,'/d1max/pct_scan/map_cloud',qos)
        self.mark_pub=self.create_publisher(Marker,'/d1max/pct_scan/simulation_marker',qos)
        self.trail_pub=self.create_publisher(RosPath,'/d1max/pct_scan/simulated_trail',qos)
        self.trail=RosPath();self.trail.header.frame_id=self.cfg['map_frame']
        self.tf=TransformBroadcaster(self);self.stf=StaticTransformBroadcaster(self)
        tf=TransformStamped();tf.header.stamp=self.get_clock().now().to_msg();tf.header.frame_id=self.cfg['map_frame'];tf.child_frame_id=self.cfg['odom_frame'];tf.transform.rotation.w=1.;self.stf.sendTransform(tf)
        self.create_service(SetBool,'/d1max/pct_scan/simulation/sensors',self.sensors)
        self.create_service(SetBool,'/d1max/pct_scan/simulation/obstacle',self.obstacle_service)
        self.map_pub.publish(self.cloud_message(self.xyz))
        self.create_timer(.02,self.tick);self.create_timer(.1,self.clouds)
        # Static transient-local map is sent once. Re-serializing the entire
        # building every second stalls the 50 Hz pose/TF loop.

    def cloud_message(self,xyz):
        m=PointCloud2();m.header.stamp=self.get_clock().now().to_msg();m.header.frame_id=self.cfg['map_frame'];m.height=1;m.width=len(xyz)
        m.fields=[PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,n in enumerate(('x','y','z'))]
        m.point_step=12;m.row_step=12*len(xyz);m.is_dense=True;m.data=array('B',np.asarray(xyz,dtype='<f4').tobytes());return m

    def floor(self,x,y):
        _,ids=self.gt.query([x,y],k=8);return float(np.median(self.ground[ids,2]))

    def cmd(self,msg):
        vals=[msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z]
        if not np.isfinite(vals).all() or any(abs(v)>1e-6 for v in vals[1:5]):self.command=(0.,0.);return
        self.command=(float(np.clip(vals[0],-self.cfg['max_speed'],self.cfg['max_speed'])),float(np.clip(vals[5],-self.cfg['max_yaw_rate'],self.cfg['max_yaw_rate'])));self.received=time.monotonic()

    def sensors(self,req,res):
        self.sensor_enabled=req.data;res.success=True;res.message='synthetic sensor publishing '+str(req.data);return res

    def obstacle_service(self,req,res):
        self.obstacle=req.data;self.obstacle_center=(self.x+math.cos(self.yaw)*.75,self.y+math.sin(self.yaw)*.75)
        res.success=True;res.message=json.dumps({'enabled':self.obstacle,'center':self.obstacle_center});return res

    def tick(self):
        now=time.monotonic();dt=min(.05,now-self.tick_at);self.tick_at=now
        v,w=self.command if now-self.received<.25 else (0.,0.)
        nx=self.x+math.cos(self.yaw)*v*dt;ny=self.y+math.sin(self.yaw)*v*dt
        cell=np.floor((np.array([nx,ny])-self.origin)/self.res).astype(int)
        valid=(cell>=0).all() and (cell<self.free.shape).all() and self.free[tuple(cell)]
        if self.obstacle and math.hypot(nx-self.obstacle_center[0],ny-self.obstacle_center[1])<.45:valid=False
        if not valid and abs(v)>1e-6:v=0.;nx,ny=self.x,self.y;self.collisions+=1
        self.distance+=math.hypot(nx-self.x,ny-self.y);self.x,self.y=nx,ny;self.yaw=math.atan2(math.sin(self.yaw+w*dt),math.cos(self.yaw+w*dt));self.v,self.w=v,w
        z=self.floor(self.x,self.y)+self.cfg['body_height'];stamp=self.get_clock().now().to_msg();self.pose_stamp=stamp
        od=Odometry();od.header.stamp=stamp;od.header.frame_id=self.cfg['map_frame'];od.child_frame_id=self.cfg['base_frame']
        od.pose.pose.position.x=self.x;od.pose.pose.position.y=self.y;od.pose.pose.position.z=z
        od.pose.pose.orientation.z=math.sin(self.yaw/2);od.pose.pose.orientation.w=math.cos(self.yaw/2);od.twist.twist.linear.x=v;od.twist.twist.angular.z=w
        if self.sensor_enabled:
            self.global_pub.publish(od);self.sensor_pub.publish(od);od.header.frame_id=self.cfg['odom_frame'];self.local_pub.publish(od)
            tf=TransformStamped();tf.header=od.header;tf.child_frame_id=self.cfg['base_frame'];tf.transform.translation.x=self.x;tf.transform.translation.y=self.y;tf.transform.translation.z=z;tf.transform.rotation=od.pose.pose.orientation;self.tf.sendTransform(tf)
        self.ticks+=1
        if self.ticks%10==0:
            m=String();m.data=json.dumps({'mode':'PCD_SOFTWARE_SIMULATION','session_id':self.session,'robot_connected':False,'real_motion_enabled':False,'wall_time':time.time(),'pose':[self.x,self.y,z,self.yaw],'velocity':[v,w],'distance_m':self.distance,'collision_stops':self.collisions,'sensors_enabled':self.sensor_enabled,'obstacle_enabled':self.obstacle});self.status_pub.publish(m)
            marker=Marker();marker.header.stamp=stamp;marker.header.frame_id=self.cfg['map_frame'];marker.ns='simulation';marker.id=0;marker.type=Marker.ARROW;marker.action=Marker.ADD;marker.pose=od.pose.pose;marker.scale.x=.8;marker.scale.y=.35;marker.scale.z=.25;marker.color.r=1.;marker.color.g=.4;marker.color.a=1.;self.mark_pub.publish(marker)
            if not self.trail.poses or math.hypot(self.x-self.trail.poses[-1].pose.position.x,self.y-self.trail.poses[-1].pose.position.y)>.02:
                pose=PoseStamped();pose.header=marker.header;pose.pose=od.pose.pose;self.trail.header=marker.header
                self.trail.poses.append(pose);self.trail.poses=self.trail.poses[-5000:];self.trail_pub.publish(self.trail)

    def clouds(self):
        if not self.sensor_enabled or not hasattr(self,'pose_stamp'):return
        ids=self.tree.query_ball_point([self.x,self.y],self.cfg['sensor_range']);xyz=self.xyz[ids]
        if self.obstacle:
            angles=np.linspace(0,2*np.pi,80);heights=np.linspace(self.floor(self.x,self.y)+.1,self.floor(self.x,self.y)+1.,12)
            ring=np.array([[self.obstacle_center[0]+.15*math.cos(a),self.obstacle_center[1]+.15*math.sin(a),h] for a in angles for h in heights]);xyz=np.vstack([xyz,ring])
        cloud=self.cloud_message(xyz);cloud.header.stamp=self.pose_stamp;self.cloud_pub.publish(cloud)
        c,s=math.cos(self.yaw),math.sin(self.yaw);z=self.floor(self.x,self.y)+self.cfg['body_height']
        ranges=project_points(xyz,[-c*self.x-s*self.y,s*self.x-c*self.y,-z],[0,0,-math.sin(self.yaw/2),math.cos(self.yaw/2)],**self.scan_slice,range_min=.15,range_max=6.,bins=720)
        m=LaserScan();m.header.stamp=self.pose_stamp;m.header.frame_id=self.cfg['base_frame'];m.angle_min=-math.pi;m.angle_increment=2*math.pi/720;m.angle_max=m.angle_min+719*m.angle_increment;m.range_min=.15;m.range_max=6.;m.scan_time=.1;m.ranges=ranges.tolist();self.scan_pub.publish(m)


def main(args=None):
    check_isolation();rclpy.init(args=args);node=Simulator()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:node.destroy_node();rclpy.try_shutdown()
