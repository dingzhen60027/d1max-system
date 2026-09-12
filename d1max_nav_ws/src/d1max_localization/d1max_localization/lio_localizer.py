"""LIO + verified PCD localization. SDK telemetry is never integrated as position."""
from copy import deepcopy
from collections import deque
from dataclasses import fields
import json
import math
import os
from pathlib import Path
import queue
import signal
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry, Path as RosPath
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, TransformStamped
from diagnostic_msgs.msg import DiagnosticArray
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from .lio_fusion import LioLimits, LioMapState, rotate_pose_covariance
from .math_utils import Pose3, normalize_quaternion, compose
from .initial_pose import initial_tracking_pose

PREFIX='/d1max/localization/'
def seconds(message):
    return message.header.stamp.sec+message.header.stamp.nanosec*1e-9
def pose_from(p):
    xyz=(p.position.x,p.position.y,p.position.z)
    if not all(math.isfinite(v) for v in xyz):raise ValueError('nonfinite pose')
    return Pose3(xyz,normalize_quaternion((p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w)))
def fill(p,value):
    p.position.x,p.position.y,p.position.z=value.position
    p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=value.orientation

class LioLocalizer(Node):
    def __init__(self):
        super().__init__('lio_localizer')
        defaults={'session_dir':'','map_frame':'d1max_loc_map','odom_frame':'d1max_loc_odom',
          'tracking_frame':'d1max_loc_tracking','lidar_frame':'d1max_loc_lidar',
          'sdk_to_tracking_yaw':1.56646,'tracking_offset_body':[.4043,0.,-.0377],
          'extrinsics_verified':False,'time_alignment_verified':False,
          'trajectory_rate_hz':5.,'trajectory_max_points':1800,'navigation_output_enabled':False}
        self.p={k:self.declare_parameter(k,v).value for k,v in defaults.items()}
        if not 0<self.p['trajectory_rate_hz']<=20 or not 10<=self.p['trajectory_max_points']<=10000:
            raise ValueError('Invalid bounded trajectory configuration')
        limits={f.name:self.declare_parameter('limits.'+f.name,getattr(LioLimits(),f.name)).value for f in fields(LioLimits)}
        self.core=LioMapState(LioLimits(**limits))
        self.directory=Path(self.p['session_dir']).resolve()
        self.session=json.loads((self.directory/'session.json').read_text())
        self.navigation={};self.navigation_at=0.;self.last_verified=None
        self.alignment_pub=self.create_publisher(String,PREFIX+'map_alignment',10)
        self.local_pub=self.global_pub=self.pose_pub=self.path_pub=None
        if not self.p['navigation_output_enabled']:
            self.local_pub=self.create_publisher(Odometry,PREFIX+'odometry/local',20)
            self.global_pub=self.create_publisher(Odometry,PREFIX+'odometry/global',20)
            self.pose_pub=self.create_publisher(PoseStamped,PREFIX+'pose',10)
            self.path_pub=self.create_publisher(RosPath,PREFIX+'trajectory',2)
        self.icp_pub=self.create_publisher(PoseWithCovarianceStamped,PREFIX+'icp_pose',10)
        self.seed_pub=self.create_publisher(PoseWithCovarianceStamped,PREFIX+'fused_icp/initialpose',10)
        self.prediction_pub=self.create_publisher(PoseWithCovarianceStamped,PREFIX+'fused_icp/prediction',20)
        self.status_pub=self.create_publisher(String,PREFIX+'status',10)
        self.tf=TransformBroadcaster(self) if not self.p['navigation_output_enabled'] else None
        self.static_tf=StaticTransformBroadcaster(self)
        sensor=TransformStamped();sensor.header.stamp=self.get_clock().now().to_msg()
        sensor.header.frame_id=self.p['tracking_frame'];sensor.child_frame_id=self.p['lidar_frame']
        sensor.transform.rotation.w=1.;self.static_tf.sendTransform(sensor)
        self.frontend={};self.frontend_at=0.;self.matcher={};self.input_clock={}
        self.imu_at=0.;self.cloud_at=0.;self.sdk_at=0.;self.robot_at=0.;self.head_direction=0
        self.speed_report={};self.speed_report_at=0.;self.last_error='';self.command_result=None
        self.last_command=None;self.last_output=0.;self.last_path=0.;self.pose_times=deque(maxlen=60)
        self.active_seed=None;self.confirmed_seed=None;self.verified_confirmations=0
        self.epoch_changed_at=0.
        self.path=deque(maxlen=int(self.p['trajectory_max_points']))
        self.subscriptions_=[
          self.create_subscription(String,PREFIX+'lio/local_sample',self.on_local_sample,20),
          self.create_subscription(String,PREFIX+'lio/status',self.on_frontend,10),
          self.create_subscription(String,PREFIX+'fused_icp/pose_raw/verified',self.on_verified,20),
          self.create_subscription(String,PREFIX+'navigation/status',self.on_navigation,10),
          self.create_subscription(Imu,PREFIX+'imu',lambda m:setattr(self,'imu_at',seconds(m)),qos_profile_sensor_data),
          self.create_subscription(PointCloud2,PREFIX+'points',lambda m:setattr(self,'cloud_at',seconds(m)),qos_profile_sensor_data),
          self.create_subscription(String,'/d1max_sdk_bridge/velocity',self.on_mc,10),
          self.create_subscription(String,'/d1max_sdk_bridge/robot_state',self.on_robot,10),
          self.create_subscription(String,'/d1max_sdk_bridge/speed_report_status',self.on_speed,10),
          self.create_subscription(DiagnosticArray,'/diagnostics',self.on_diagnostic,10)]
        self.saved=queue.Queue(maxsize=1);self.writer_stop=threading.Event()
        self.writer=threading.Thread(target=self.write_status,name='localization-status-writer',daemon=True);self.writer.start()
        self.timer=self.create_timer(.2,self.tick)

    def now_s(self):return self.get_clock().now().nanoseconds*1e-9
    def fresh(self,stamp,age=.3):return self.core.valid_time(stamp,self.now_s(),age)
    def front_ready(self):
        return (self.core.stream_valid and self.frontend.get('epoch')==self.core.local_epoch
                and self.frontend.get('ready') is True and not self.frontend.get('fault') and self.fresh(self.frontend_at,.6))
    def seed_ready(self):
        local_ready=(not self.p.get('navigation_output_enabled') or
            self.fresh(self.navigation_at,.6) and self.navigation.get('epoch')==self.core.local_epoch and self.navigation.get('local_fresh') is True)
        return local_ready and self.front_ready() and self.core.local_fresh(self.now_s()) and self.head_direction==1 and self.fresh(self.robot_at,1.5)
    def on_navigation(self,message):
        try:
            value=json.loads(message.data);stamp=float(value['received_at_unix'])
            if (value.get('schema')==1 and self.fresh(stamp,.6) and stamp>self.navigation_at
                and value.get('epoch',0)>=self.core.local_epoch):self.navigation=value;self.navigation_at=stamp
        except (ValueError,KeyError,TypeError):pass

    def publish_alignment(self):
        if not self.p.get('navigation_output_enabled'):return
        if self.core.local_epoch == 0:return
        now=self.now_s();m=self.last_verified
        valid=bool(m is not None and self.front_ready() and self.core.tracking(now)
                   and self.confirmed_seed==self.active_seed and self.verified_confirmations>=3)
        value={'schema':1,'epoch':self.core.local_epoch,'valid':valid,'fault':bool(self.core.fault),
               'received_at_unix':now,'seed_id':self.active_seed,'confirmations':self.verified_confirmations}
        if valid:
            pose=pose_from(m.pose.pose);anchor=self.core.anchor
            value.update(frame=self.p['map_frame'],child_frame=self.p['tracking_frame'],
                         stamp_ns=str(m.header.stamp.sec*1000000000+m.header.stamp.nanosec),
                         position=pose.position,orientation=pose.orientation,covariance=list(m.pose.covariance),
                         anchor={'position':anchor.position,'orientation':anchor.orientation})
        message=String();message.data=json.dumps(value,allow_nan=False);self.alignment_pub.publish(message)
    def on_frontend(self,m):
        try:
            value=json.loads(m.data)
            if value.get('backend')!='faster_lio' or value.get('epoch',0)<self.core.local_epoch:return
            self.frontend=value;self.frontend_at=self.now_s()
            # Fault authority travels with epoch-tagged samples on one ordered
            # channel. A delayed status must not fault a newly initialized epoch.
        except (ValueError,AttributeError):pass
    def on_mc(self,m):
        try:
            v=json.loads(m.data)
            if v.get('source')=='sdk_mc' and v.get('frame')=='sdk_body' and self.fresh(float(v['received_at_unix'])):
                self.sdk_at=float(v['received_at_unix'])
        except (ValueError,KeyError,TypeError):pass
    def on_robot(self,m):
        try:
            v=json.loads(m.data);t=float(v['received_at_unix'])
            if self.fresh(t,1.5):self.robot_at=t;self.head_direction=v.get('head_direction',0)
        except (ValueError,KeyError,TypeError):pass
    def on_speed(self,m):
        try:
            v=json.loads(m.data);t=float(v['received_at_unix'])
            if v.get('source')=='sdk_mc' and self.fresh(t,1.):
                self.speed_report={key:v.get(key) for key in ('source','state','observed_hz','rate_ok','acknowledged','attempts','max_attempts','write_error')}
                self.speed_report_at=t
        except (ValueError,KeyError,TypeError):pass
    def on_diagnostic(self,m):
        if not self.fresh(seconds(m),1.):return
        for status in m.status:
            value={p.key:p.value for p in status.values};value['message']=status.message
            if status.name=='d1max_localization/matcher':self.matcher=value
            elif status.name=='d1max_localization/input_clock':self.input_clock=value
    def pose_message(self,pose,stamp):
        m=PoseWithCovarianceStamped();m.header.frame_id=self.p['map_frame']
        m.header.stamp=rclpy.time.Time(seconds=stamp).to_msg();fill(m.pose.pose,pose)
        for i in (0,7,14,21,28,35):m.pose.covariance[i]=.1
        return m
    def transform(self,parent,child,pose,stamp):
        t=TransformStamped();t.header.frame_id=parent;t.child_frame_id=child;t.header.stamp=stamp
        t.transform.translation.x,t.transform.translation.y,t.transform.translation.z=pose.position
        t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w=pose.orientation
        return t
    def on_local_sample(self,message):
        try:
            value=json.loads(message.data)
            if value.get('schema')!=1 or not self.fresh(float(value['received_at_unix']),.3):return
            epoch=value['epoch'];valid=value['valid']
            if type(epoch) is not int or not 1<=epoch<2**64 or type(valid) is not bool:return
            if type(value.get('fault')) is not bool:return
            if valid:
                if value['fault'] or value['frame']!=self.p['odom_frame'] or value['child_frame']!=self.p['tracking_frame']:return
                for key,size in (('position',3),('orientation',4),('linear',3),('angular',3),('pose_covariance',36),('twist_covariance',36)):
                    if len(value[key])!=size or not all(math.isfinite(x) for x in value[key]):return
                m=Odometry();m.header.frame_id=value['frame'];m.child_frame_id=value['child_frame']
                m.header.stamp=rclpy.time.Time(nanoseconds=int(value['stamp_ns'])).to_msg()
                if not self.fresh(seconds(m)):return
                fill(m.pose.pose,Pose3(tuple(value['position']),normalize_quaternion(value['orientation'])))
                m.pose.covariance=list(map(float,value['pose_covariance']));m.twist.covariance=list(map(float,value['twist_covariance']))
                m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.linear.z=map(float,value['linear'])
                m.twist.twist.angular.x,m.twist.twist.angular.y,m.twist.twist.angular.z=map(float,value['angular'])
            previous=self.core.local_epoch
            if not self.core.accept_epoch(epoch,valid):return
            if epoch!=previous:
                self.epoch_changed_at=self.now_s()
                self.active_seed=None;self.confirmed_seed=None;self.verified_confirmations=0
                self.path.clear();self.pose_times.clear();self.last_output=0.;self.last_path=0.
                if not self.p.get('navigation_output_enabled'):
                    empty=RosPath();empty.header.frame_id=self.p['map_frame'];empty.header.stamp=self.get_clock().now().to_msg();self.path_pub.publish(empty)
                else:self.last_verified=None
                if previous:
                    self.last_error='局部里程计已重新初始化，请重新提交地图初值'
                    if self.last_command:self.command_result={'id':self.last_command,'accepted':False,'message':self.last_error}
            if not valid:
                if value['fault']:self.core.fault=value.get('reason','lio_fault')
                self.publish_alignment()
                return
            self.on_local(m)
        except (ValueError,KeyError,TypeError,OverflowError):
            self.last_error='局部里程计样本格式无效'
    def on_local(self,m):
        if m.header.frame_id!=self.p['odom_frame'] or m.child_frame_id!=self.p['tracking_frame']:return
        try:
            t=seconds(m);pose=pose_from(m.pose.pose)
            if not self.core.push_local(t,pose,self.now_s()):return
        except (ValueError,OverflowError):return
        if not self.p.get('navigation_output_enabled'):self.local_pub.publish(m)
        prediction=self.core.prediction(t)
        if prediction is not None:self.prediction_pub.publish(self.pose_message(prediction,t))
        if self.p.get('navigation_output_enabled'):return
        if not self.front_ready():return
        self.tf.sendTransform(self.transform(self.p['odom_frame'],self.p['tracking_frame'],pose,m.header.stamp))
        if not self.core.tracking(self.now_s()) or t<=self.last_output:return
        # The map correction and local pose are combined at exactly the same source time.
        global_pose=compose(self.core.anchor,pose)
        out=deepcopy(m);out.header.frame_id=self.p['map_frame'];fill(out.pose.pose,global_pose)
        out.pose.covariance=rotate_pose_covariance(m.pose.covariance,self.core.anchor.orientation)
        # No uncalibrated tight covariance, and no reuse of body covariance as a map guarantee.
        for index,floor in ((0,.0225),(7,.0225),(14,.0625),(21,.0144),(28,.0144),(35,.0225)):
            out.pose.covariance[index]=max(floor,out.pose.covariance[index])
        self.global_pub.publish(out)
        p=PoseStamped();p.header=deepcopy(out.header);p.pose=deepcopy(out.pose.pose);self.pose_pub.publish(p)
        self.tf.sendTransform(self.transform(self.p['map_frame'],self.p['odom_frame'],self.core.anchor,m.header.stamp))
        self.last_output=t;self.pose_times.append(t)
        if t-self.last_path>=1./self.p['trajectory_rate_hz']:
            self.path.append(p);path=RosPath();path.header=p.header;path.poses=list(self.path);self.path_pub.publish(path);self.last_path=t
    def on_icp(self,m):
        if m.header.frame_id!=self.p['map_frame'] or not self.front_ready():return
        try:
            if self.core.accept(seconds(m),pose_from(m.pose.pose),self.now_s()):
                self.icp_pub.publish(m);self.last_error=''
            else:self.last_error='地图校正未通过时间或创新量门限'
        except (ValueError,OverflowError):self.last_error='地图校正无效'
    def publish_seed(self,pose,stamp):
        m=self.pose_message(pose,stamp)
        self.active_seed=str(m.header.stamp.sec*1000000000+m.header.stamp.nanosec)
        self.confirmed_seed=None;self.verified_confirmations=0
        self.seed_pub.publish(m)
        if self.p.get('navigation_output_enabled'):self.last_verified=None;self.publish_alignment()
    def on_verified(self,message):
        try:
            value=json.loads(message.data)
            if not self.core.current_verification(value,self.active_seed):
                self.last_error='忽略旧轮次或尚未连续确认的匹配';return
            m=PoseWithCovarianceStamped();m.header.frame_id=value['frame']
            m.header.stamp=rclpy.time.Time(nanoseconds=int(value['stamp_ns'])).to_msg()
            fill(m.pose.pose,Pose3(tuple(value['position']),normalize_quaternion(value['orientation'])))
            covariance=value['covariance']
            if len(covariance)!=36 or not all(math.isfinite(x) for x in covariance):return
            m.pose.covariance=list(map(float,covariance))
            previous=self.core.accepted;self.on_icp(m)
            if self.core.accepted>previous:
                self.confirmed_seed=self.active_seed;self.verified_confirmations=value['confirmations']
                if self.p.get('navigation_output_enabled'):self.last_verified=deepcopy(m);self.publish_alignment()
        except (ValueError,KeyError,TypeError,IndexError,OverflowError):self.last_error='地图校验消息无效'
    def read_command(self):
        path=self.directory/'initial_pose.json'
        if not path.exists():return
        try:
            command=json.loads(path.read_text());identifier=command['id']
            if identifier==self.last_command:return
            self.last_command=identifier
            if command['session_id']!=self.session['id'] or not 0<=time.time()-command['created_at']<=5:raise ValueError('初值已过期或会话不匹配')
            if command['created_at']<self.epoch_changed_at:raise ValueError('局部里程计轮次已变化，请重新提交初值')
            if not self.seed_ready():raise ValueError('等待 LIO 初始化和新鲜机头前向状态')
            pose=initial_tracking_pose(command,self.p['tracking_offset_body'],self.p['sdk_to_tracking_yaw'])
            stamp=self.now_s();self.core.seed(pose,stamp);self.publish_seed(pose,stamp)
            self.path.clear()
            # Explicitly clear old verified trajectory on reseed.
            if not self.p.get('navigation_output_enabled'):
                empty=RosPath();empty.header.frame_id=self.p['map_frame'];empty.header.stamp=self.get_clock().now().to_msg();self.path_pub.publish(empty)
            self.command_result={'id':identifier,'accepted':True,'message':'初值仅作为地图匹配种子，等待连续确认'}
        except (ValueError,KeyError,TypeError,OSError) as e:
            self.command_result={'id':self.last_command,'accepted':False,'message':str(e)}
    def tick(self):
        self.read_command()
        now=self.now_s();front=self.front_ready()
        if front:
            recovery=self.core.begin_recovery(now)
            if recovery is not None:
                self.publish_seed(recovery,now)
                self.last_error='地图匹配失锁，正在限制范围内重新确认'
        tracking=front and self.core.tracking(now)
        if self.core.fault:state='fault'
        elif not self.fresh(self.imu_at) or not self.fresh(self.cloud_at,.6):state='waiting_sensors'
        elif self.frontend.get('recovering'):state='recovering_local'
        elif not front:state='calibrating' if self.frontend.get('initializing',True) else 'degraded'
        elif self.core.seed_time is None:state='waiting_initial_pose'
        elif tracking:state='tracking'
        elif self.core.last_correction is None:state='acquiring'
        elif self.core.recovery:state='relocalizing'
        else:state='lost'
        self.publish_alignment()
        navigation_current=(self.p.get('navigation_output_enabled') and self.fresh(self.navigation_at,.6)
                            and self.navigation.get('epoch')==self.core.local_epoch)
        navigation=self.navigation if navigation_current else {}
        if self.p.get('navigation_output_enabled') and tracking:
            tracking=navigation.get('valid') is True and navigation.get('seed_id')==self.active_seed
            if not tracking:state='navigation_fault' if navigation.get('fault') else 'filter_initializing'
        age=now-self.core.last_correction if self.core.last_correction is not None else None
        hz=0.
        if len(self.pose_times)>1 and self.fresh(self.pose_times[-1],.3):
            hz=(len(self.pose_times)-1)/(self.pose_times[-1]-self.pose_times[0])
        if self.p.get('navigation_output_enabled'):hz=navigation.get('global_observed_hz',0.)
        warnings=[]
        if not self.p['extrinsics_verified']:warnings.append('外参未完成实机验收')
        if not self.p['time_alignment_verified']:warnings.append('传感器时间对齐未完成实机验收')
        if not self.fresh(self.sdk_at):warnings.append('MC 速度未就绪；不使用状态速度降级，不影响独立 LIO 运动估计')
        if self.core.fault:warnings.append('局部 LIO 已停止可信输出：'+self.core.fault)
        if self.frontend.get('recovering'):warnings.append('IMU 缺口后重新初始化；请保持静止，完成后重新给地图初值')
        if self.head_direction!=1:warnings.append('初值提交要求机头前向；当前头尾切换状态不符合，程序不会自动切换')
        if navigation.get('fault'):warnings.append('导航输出已暂停：'+navigation['fault'])
        status={'session_id':self.session['id'],'map_version_id':self.session['version_id'],'wall_time':time.time(),
          'state':state,'localized':tracking,'navigation_ready':bool(tracking and navigation.get('navigation_ready')),
          'local_backend':'faster_lio','fusion_backend':'robot_localization_ekf' if self.p.get('navigation_output_enabled') else 'se3_map_alignment',
          'navigation':navigation,
          'initial_pose_ready':self.seed_ready(),'frontend':self.frontend,'frontend_ready':front,
          'local_epoch':self.core.local_epoch,'local_fault':self.core.fault,
          'active_seed_ns':self.active_seed,'confirmed_seed_ns':self.confirmed_seed,'verified_confirmations':self.verified_confirmations,
          'sensors':{'sdk':self.fresh(self.sdk_at),'imu':self.fresh(self.imu_at),'cloud':self.fresh(self.cloud_at,.6)},
          'sdk_source':'mc_stream' if self.fresh(self.sdk_at) else 'missing',
          'speed_report':self.speed_report if self.fresh(self.speed_report_at,1.) else {},
          'gyro_bias_samples':self.frontend.get('initialization_samples',0),
          'gyro_bias_required':self.frontend.get('initialization_required',200),'gyro_bias':self.frontend.get('gyro_bias'),
          'local_ekf_fresh':(navigation.get('local_fresh',False) if self.p.get('navigation_output_enabled') else front and self.core.local_fresh(now)),
          'global_ekf_fresh':tracking,
          'correction_age':age,'head_direction':self.head_direction,
          'fusion':{'accepted':str(self.core.accepted),'rejected':str(self.core.rejected),
                    'alignment_locked':str(tracking).lower(),'message':state},
          'recovery':{'attempts':self.core.recovery_count,'max_attempts':self.core.limits.recovery_attempts},
          'input_clock':self.input_clock,'matcher':self.matcher,'warnings':warnings,'last_error':self.last_error,
          'command_result':self.command_result,'output_observed_hz':hz,'calibration':{
            'extrinsics_verified':self.p['extrinsics_verified'],'time_alignment_verified':self.p['time_alignment_verified']},
          'frames':{'map':self.p['map_frame'],'odom':self.p['odom_frame'],'tracking':self.p['tracking_frame']}}
        data=json.dumps(status,ensure_ascii=False,allow_nan=False)
        m=String();m.data=data;self.status_pub.publish(m)
        try:self.saved.get_nowait()
        except queue.Empty:pass
        self.saved.put_nowait(data)
    def write_status(self):
        while not self.writer_stop.is_set():
            try:data=self.saved.get(timeout=.2)
            except queue.Empty:continue
            try:
                temporary=self.directory/'status.tmp';temporary.write_text(data);os.replace(temporary,self.directory/'status.json')
            except OSError:pass
    def close(self):
        self.timer.cancel();self.writer_stop.set();self.writer.join(timeout=2.)

def main(args=None):
    rclpy.init(args=args);node=LioLocalizer()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        # Launch and its owning process group may both deliver SIGINT during stop.
        signal.signal(signal.SIGINT,signal.SIG_IGN)
        node.close();node.destroy_node();rclpy.try_shutdown()
