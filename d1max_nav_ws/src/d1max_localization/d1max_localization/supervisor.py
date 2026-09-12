"""Sensor admission, verified TF, session health and Web initial-pose mailbox.

No SDK socket, service client, ownership request, cmd_vel or movement interface.
"""
from copy import deepcopy
import json
import math
import os
from collections import deque
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistWithCovarianceStamped, PoseWithCovarianceStamped, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path as RosPath
from sensor_msgs.msg import Imu, PointCloud2
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from .sensor_policy import finite_vector, valid_stamp, tracking_twist, GyroBias3, health_state
from .math_utils import Pose3, normalize_quaternion, compose, inverse, rotate_vector, interpolate_pose
from .initial_pose import initial_tracking_pose

PREFIX='/d1max/localization/'

def stamp_seconds(message):
    return message.header.stamp.sec+message.header.stamp.nanosec*1e-9

def pose3(pose):
    position=(pose.position.x,pose.position.y,pose.position.z)
    if not all(math.isfinite(v) for v in position):raise ValueError('pose position is not finite')
    return Pose3(position,normalize_quaternion((pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w)))

class Supervisor(Node):
    def __init__(self):
        super().__init__('localization_supervisor')
        defaults={'session_dir':'','map_frame':'d1max_loc_map','odom_frame':'d1max_loc_odom','tracking_frame':'d1max_loc_tracking',
            'sdk_state_topic':'/d1max_sdk_bridge/robot_state','sdk_velocity_topic':'/d1max_sdk_bridge/velocity',
            'allow_sdk_state_velocity_fallback':False,  # legacy config accepted, never used
            'gyro_bias_samples':100,'gyro_bias_max_rate':.04,'stationary_speed':.05,'stationary_angular_rate':.025,'gyro_deadband':.01,
            'max_sdk_speed':3.,'max_gyro':4.,'imu_timeout':.3,'cloud_timeout':.6,'sdk_fast_timeout':.3,'sdk_state_timeout':1.5,
            'sdk_to_tracking_yaw':1.56646,'tracking_offset_body':[.4043,0.,-.0377],'extrinsics_verified':False,
            'lidar_frame':'d1max_loc_lidar','output_rate_hz':30.,'trajectory_rate_hz':5.,
            'mc_time_offset_sec':0.,'time_alignment_verified':False,'velocity_stddev_z':.30,
            'velocity_stddev_xy':.20,'fallback_velocity_stddev_xy':.45,'angular_velocity_stddev':.04,
            'max_output_tilt_error_deg':10.,'max_output_icp_sync_sec':.20,'max_output_icp_age_sec':.75}
        self.p={key:self.declare_parameter(key,value).value for key,value in defaults.items()}
        for key in ('max_output_tilt_error_deg','max_output_icp_sync_sec','max_output_icp_age_sec','output_rate_hz','trajectory_rate_hz'):
            if not math.isfinite(self.p[key]) or self.p[key]<=0:raise ValueError(key+' must be finite and positive')
        for key in ('stationary_speed','stationary_angular_rate'):
            if not math.isfinite(self.p[key]) or not 0<self.p[key]<=.1:
                raise ValueError(key+' must be finite and within (0, 0.1]')
        if self.p['max_output_tilt_error_deg']>=90:raise ValueError('output tilt limit must be less than 90 degrees')
        if not math.isfinite(self.p['mc_time_offset_sec']) or abs(self.p['mc_time_offset_sec'])>.25:
            raise ValueError('MC timing correction must be finite and within 250 ms')
        if self.p['output_rate_hz']>100 or self.p['trajectory_rate_hz']>self.p['output_rate_hz']:
            raise ValueError('invalid output / trajectory rates')
        self.directory=Path(self.p['session_dir']).resolve()
        if not self.directory.is_dir():raise ValueError('session_dir must be created by the launcher')
        self.session=json.loads((self.directory/'session.json').read_text())
        self.bias=GyroBias3(self.p['gyro_bias_samples'],self.p['gyro_bias_max_rate'],self.p['stationary_speed'],self.p['gyro_deadband'])
        self.sdk_fast=None;self.sdk_state=None;self.last_fast=0.;self.last_fast_received=0.;self.last_state=0.;self.last_imu=0.;self.last_cloud=0.
        self.head_direction=0;self.head_stamp=0.
        self.last_imu_stamp=0.;self.last_cloud_stamp=0.;self.gyro=None;self.seeded=False;self.aligned=False
        self.diagnostic={};self.diagnostic_time=0.;self.input_clock={};self.matcher={};self.local=deque(maxlen=200);self.global_odom=None;self.global_history=deque(maxlen=200)
        self.accepted_icp=deque(maxlen=100);self.output_guard={'ok':False,'reason':'等待已接受的 ICP 姿态校验'}
        self.speed_report={};self.speed_report_at=0.
        self.mc_epoch=None;self.mc_source_ns=0;self.stationary_metric=None;self.last_path_stamp=0.;self.output_samples=deque(maxlen=100)
        self.last_pose_pub_stamp=0.;self.seed_time=0.;self.last_command=None;self.command_result=None;self.rejections={};self.last_error=''
        self.twist_pub=self.create_publisher(TwistWithCovarianceStamped,PREFIX+'body_twist',10)
        self.imu_pub=self.create_publisher(Imu,PREFIX+'body_imu',50)
        self.initial_pub=self.create_publisher(PoseWithCovarianceStamped,PREFIX+'initialpose',10)
        self.pose_pub=self.create_publisher(PoseStamped,PREFIX+'pose',10)
        self.path_pub=self.create_publisher(RosPath,PREFIX+'trajectory',2)
        self.status_pub=self.create_publisher(String,PREFIX+'status',2)
        self.tf=TransformBroadcaster(self);self.trajectory=RosPath();self.trajectory.header.frame_id=self.p['map_frame']
        self.static_tf=StaticTransformBroadcaster(self)
        sensor_tf=TransformStamped();sensor_tf.header.stamp=self.get_clock().now().to_msg()
        sensor_tf.header.frame_id=self.p['tracking_frame'];sensor_tf.child_frame_id=self.p['lidar_frame']
        sensor_tf.transform.rotation.w=1.
        self.static_tf.sendTransform(sensor_tf)  # normalized LiDAR and tracking coincide; no URDF
        self.subscriptions_=[
            self.create_subscription(String,self.p['sdk_velocity_topic'],lambda m:self.on_sdk(m,True),10),
            self.create_subscription(String,self.p['sdk_state_topic'],lambda m:self.on_sdk(m,False),10),
            self.create_subscription(String,'/d1max_sdk_bridge/speed_report_status',self.on_speed_report,10),
            self.create_subscription(Imu,PREFIX+'imu',self.on_imu,qos_profile_sensor_data),
            self.create_subscription(PointCloud2,PREFIX+'points',self.on_cloud,qos_profile_sensor_data),
            self.create_subscription(Odometry,PREFIX+'odometry/local',self.on_local,20),
            self.create_subscription(Odometry,PREFIX+'odometry/global',self.on_global,20),
            self.create_subscription(PoseWithCovarianceStamped,PREFIX+'icp_pose',self.on_accepted_icp,20),
            self.create_subscription(DiagnosticArray,'/diagnostics',self.on_diagnostic,10),
        ]
        self.timer=self.create_timer(.2,self.tick)
        self.output_timer=self.create_timer(1./self.p['output_rate_hz'],self.output_tick)

    def reject(self,source,error):
        self.rejections[source]=self.rejections.get(source,0)+1;self.last_error=str(error)

    def now_seconds(self):return self.get_clock().now().nanoseconds*1e-9

    def on_sdk(self,message,fast):
        try:
            value=json.loads(message.data);stamp=float(value['received_at_unix'])
            if not valid_stamp(stamp,self.now_seconds(),self.p['sdk_fast_timeout'] if fast else self.p['sdk_state_timeout']):raise ValueError('SDK timestamp stale or clock not synchronized')
            if not fast:
                self.head_direction=value.get('head_direction',0);self.head_stamp=stamp
                if self.head_direction!=1:self.sdk_fast=None
                return  # RobotState is status only, NEVER a velocity fallback.
            if self.head_direction!=1 or not valid_stamp(self.head_stamp,self.now_seconds(),self.p['sdk_state_timeout']):
                self.sdk_fast=None;self.sdk_state=None
                raise ValueError('机身前进方向未知或切换为尾部，暂停速度融合；请核对前向状态')
            if value.get('source')!='sdk_mc' or value.get('frame')!='sdk_body':raise ValueError('定位只接受 OnMcData 机身速度')
            linear=finite_vector(value['v_body'],self.p['max_sdk_speed'])
            angular=finite_vector(value['omega_body'],self.p['max_gyro'])
            if len(linear)!=3 or len(angular)!=3:raise ValueError('MC vectors must have 3 axes')
            source=value['source_timestamp_ns']
            if not isinstance(source,str) or not source.isdecimal():raise ValueError('MC timestamp must be a decimal ns string')
            source=int(source)
            if not 0<source<2**64:raise ValueError('invalid MC source timestamp')
            epoch=(value['session'],value['generation'])
            if not isinstance(epoch[0],str) or not epoch[0] or type(epoch[1]) is not int or epoch[1]<0:raise ValueError('invalid MC session')
            if epoch==self.mc_epoch and source<=self.mc_source_ns:raise ValueError('MC timestamp repeated or reversed')
            aligned=float(value['stamp_unix'])
            if value.get('clock_mode')!='source_delta_host_anchor' or not valid_stamp(aligned,self.now_seconds(),self.p['sdk_fast_timeout']):
                raise ValueError('MC aligned timestamp stale or invalid')
            if abs(aligned-stamp)>=self.p['sdk_fast_timeout']:raise ValueError('MC source / receipt clock mismatch')
            aligned+=self.p['mc_time_offset_sec']
            if not valid_stamp(aligned,self.now_seconds(),self.p['sdk_fast_timeout']):raise ValueError('calibrated MC stamp outside admission window')
            if aligned<=self.last_fast:return
            velocity=(linear[0],linear[1],angular[2])
            covariance=self.p['velocity_stddev_xy']
            converted,_=tracking_twist(linear,angular,self.p['sdk_to_tracking_yaw'],self.p['tracking_offset_body'])
            output=TwistWithCovarianceStamped();output.header.stamp=rclpy.time.Time(seconds=aligned).to_msg();output.header.frame_id=self.p['tracking_frame']
            output.twist.twist.linear.x,output.twist.twist.linear.y,output.twist.twist.linear.z=converted
            output.twist.covariance[0]=covariance**2;output.twist.covariance[7]=covariance**2
            output.twist.covariance[14]=self.p['velocity_stddev_z']**2
            # MC omega is used for the full 3D lever arm, not fused a second time.
            for i in (21,28,35):output.twist.covariance[i]=1e6
            self.twist_pub.publish(output)
            if self.mc_epoch is not None and epoch!=self.mc_epoch:
                self.bias.reset();self.seeded=False;self.aligned=False
                self.accepted_icp.clear();self.local.clear();self.global_history.clear()
                self.output_guard={'ok':False,'reason':'MC 会话变化，需重新初始化定位'}
            # Linear speed (m/s) and angular rate (rad/s) have separate limits.
            # This only gates gyro-bias calibration; published MC velocity is untouched.
            self.stationary_metric=(math.hypot(*linear) if math.hypot(*angular)<=self.p['stationary_angular_rate'] else None)
            self.sdk_fast=velocity;self.last_fast=aligned;self.last_fast_received=stamp;self.mc_epoch=epoch;self.mc_source_ns=source
        except (ValueError,KeyError,TypeError,OverflowError) as error:self.reject('sdk',error)

    def sdk_sample(self):
        now=self.now_seconds()
        if self.head_direction!=1 or not valid_stamp(self.head_stamp,now,self.p['sdk_state_timeout']):return None,'missing'
        # Match admission's bounded future tolerance for approximate clock alignment.
        # Receipt freshness additionally prevents a future stamp from extending a dead stream.
        if (self.sdk_fast is not None and valid_stamp(self.last_fast,now,self.p['sdk_fast_timeout'])
                and valid_stamp(self.last_fast_received,now,self.p['sdk_fast_timeout'])):return self.sdk_fast,'mc_stream'
        return None,'missing'

    def on_speed_report(self,message):
        try:
            value=json.loads(message.data);stamp=float(value['received_at_unix'])
            if not valid_stamp(stamp,self.now_seconds(),1.):return
            if value.get('source')!='sdk_mc':raise ValueError('MC status source mismatch')
            hz=float(value.get('observed_hz',0.))
            if not math.isfinite(hz) or hz<0:raise ValueError('invalid speed stream frequency')
            self.speed_report={'source':'sdk_mc','state':str(value.get('state','unknown')),'observed_hz':hz,
                               'rate_ok':value.get('rate_ok') is True,'acknowledged':value.get('acknowledged') is True,
                               'clock_approximate':value.get('clock_approximate') is True}
            self.speed_report_at=stamp
        except (ValueError,TypeError,KeyError) as error:self.reject('speed_report',error)

    def on_imu(self,message):
        if message.header.frame_id!=self.p['lidar_frame']:
            self.reject('imu','unexpected normalized IMU frame');return
        stamp=stamp_seconds(message)
        if not valid_stamp(stamp,self.now_seconds(),self.p['imu_timeout']):self.reject('imu','IMU clock skew or stale data');return
        if stamp<=self.last_imu_stamp:
            if stamp<self.last_imu_stamp-.5:
                self.bias.reset();self.seeded=False;self.aligned=False;self.last_imu_stamp=0.;self.last_cloud_stamp=0.;self.local.clear();self.global_history.clear();self.last_pose_pub_stamp=0.
                self.accepted_icp.clear();self.output_guard={'ok':False,'reason':'时钟回跳，等待重新初始化'}
            return
        try:gyro=finite_vector([message.angular_velocity.x,message.angular_velocity.y,message.angular_velocity.z],self.p['max_gyro'])
        except ValueError as error:self.reject('imu',error);return
        self.last_imu_stamp=stamp;self.last_imu=stamp
        sample,_=self.sdk_sample();speed=self.stationary_metric if sample else None
        corrected=self.bias.update(gyro,speed)
        if corrected is None:return
        self.gyro=corrected
        output=Imu();output.header=deepcopy(message.header);output.header.frame_id=self.p['tracking_frame']
        output.orientation_covariance[0]=-1.;output.linear_acceleration_covariance[0]=-1.
        output.angular_velocity.x,output.angular_velocity.y,output.angular_velocity.z=corrected
        variance=self.p['angular_velocity_stddev']**2
        output.angular_velocity_covariance=[variance,0.,0.,0.,variance,0.,0.,0.,variance]
        self.imu_pub.publish(output)

    def on_cloud(self,message):
        stamp=stamp_seconds(message)
        if not valid_stamp(stamp,self.now_seconds(),self.p['cloud_timeout']):self.reject('cloud','LiDAR clock skew or stale data');return
        if stamp>self.last_cloud_stamp:self.last_cloud=stamp;self.last_cloud_stamp=stamp

    def on_local(self,message):
        if message.header.frame_id!=self.p['odom_frame'] or message.child_frame_id!=self.p['tracking_frame']:return
        if not self.local or stamp_seconds(message)>stamp_seconds(self.local[-1]):self.local.append(message)

    def on_global(self,message):
        if message.header.frame_id!=self.p['map_frame'] or message.child_frame_id!=self.p['tracking_frame']:return
        if not self.global_history or stamp_seconds(message)>stamp_seconds(self.global_history[-1]):
            self.global_odom=message;self.global_history.append(message)

    def on_accepted_icp(self,message):
        stamp=stamp_seconds(message)
        if not self.seeded or stamp<=self.seed_time or message.header.frame_id!=self.p['map_frame']:return
        if not valid_stamp(stamp,self.now_seconds(),self.p['max_output_icp_age_sec']):return
        try:pose3(message.pose.pose)
        except ValueError as error:self.reject('accepted_icp',error);return
        if not self.accepted_icp or stamp>stamp_seconds(self.accepted_icp[-1]):self.accepted_icp.append(message)

    def on_diagnostic(self,message):
        stamp=stamp_seconds(message)
        if not valid_stamp(stamp,self.now_seconds(),1.):return
        for status in message.status:
            if status.name=='d1max_localization/matcher':
                self.matcher={v.key:v.value for v in status.values};self.matcher['message']=status.message
            if status.name=='d1max_localization/input_clock':
                self.input_clock={v.key:v.value for v in status.values};self.input_clock['message']=status.message
            if status.name=='d1max_localization/icp_fusion':
                self.diagnostic={v.key:v.value for v in status.values};self.diagnostic['message']=status.message
                self.diagnostic_time=self.now_seconds()
                if self.seeded and stamp>self.seed_time:self.aligned=self.diagnostic.get('alignment_locked')=='true'

    def read_command(self,can_seed):
        path=self.directory/'initial_pose.json'
        if not path.exists():return
        try:
            command=json.loads(path.read_text());identifier=command['id']
            if identifier==self.last_command:return
            self.last_command=identifier
            if command['session_id']!=self.session['id']:raise ValueError('wrong localization session')
            if not 0<=time.time()-command['created_at']<=5:raise ValueError('initial pose request expired')
            if not can_seed:raise ValueError('等待新鲜雷达、IMU 和 SDK 数据及静止标定完成')
            pose=initial_tracking_pose(command,self.p['tracking_offset_body'],self.p['sdk_to_tracking_yaw'])
            output=PoseWithCovarianceStamped();output.header.stamp=self.get_clock().now().to_msg();output.header.frame_id=self.p['map_frame']
            output.pose.pose.position.x,output.pose.pose.position.y,output.pose.pose.position.z=pose.position
            output.pose.pose.orientation.x,output.pose.pose.orientation.y,output.pose.pose.orientation.z,output.pose.pose.orientation.w=pose.orientation
            output.pose.covariance[0]=.25;output.pose.covariance[7]=.25;output.pose.covariance[14]=.25;output.pose.covariance[35]=.25
            self.initial_pub.publish(output)
            self.seeded=True;self.aligned=False;self.seed_time=self.now_seconds();self.diagnostic={};self.trajectory.poses=[]
            self.accepted_icp.clear();self.output_guard={'ok':False,'reason':'新初值，等待已接受的 ICP 姿态校验'}
            self.command_result={'id':identifier,'accepted':True,'message':'机身初值按固定外参转换一次，仅作为匹配种子；等待连续匹配确认',
                                 'reference':command.get('reference','legacy'),'tracking_position':list(pose.position),'tracking_orientation':list(pose.orientation)}
        except (ValueError,TypeError,KeyError,OSError) as error:
            self.command_result={'id':self.last_command,'accepted':False,'message':str(error)}

    def publish_verified_pose(self):
        self.output_guard={'ok':False,'reason':''}
        try:
            message=self.global_odom
            if message is None or not self.local:raise ValueError('等待全局 / 本地 EKF 数据')
            # Both odometry trajectories are evaluated at one common, covered
            # timestamp. ICP is transported to it by local odometry before comparison.
            target_ns=min(self.message_ns(message),self.message_ns(self.local[-1]))
            stamp=target_ns*1e-9
            if not valid_stamp(stamp,self.now_seconds(),.3):raise ValueError('EKF 数据已过期')
            gap_ns=int(self.p['max_output_icp_sync_sec']*1e9)
            local_samples=[(self.message_ns(v),pose3(v.pose.pose)) for v in self.local]
            global_samples=[(self.message_ns(v),pose3(v.pose.pose)) for v in self.global_history]
            global_pose=interpolate_pose(global_samples,target_ns,gap_ns)
            local_pose=interpolate_pose(local_samples,target_ns,gap_ns)
            if global_pose is None or local_pose is None:raise ValueError('全局 / 本地 EKF 缺少同时间插值数据')
            eligible=[v for v in self.accepted_icp if self.message_ns(v)<=target_ns]
            if not eligible:raise ValueError('等待已接受的 ICP 姿态校验')
            icp=eligible[-1];icp_ns=self.message_ns(icp)
            if not valid_stamp(icp_ns*1e-9,self.now_seconds(),self.p['max_output_icp_age_sec']):
                raise ValueError('ICP 姿态已过期，暂停可信 TF 输出')
            local_at_icp=interpolate_pose(local_samples,icp_ns,gap_ns)
            if local_at_icp is None:raise ValueError('ICP 时刻缺少本地里程计，暂停可信 TF 输出')
            expected=compose(pose3(icp.pose.pose),compose(inverse(local_at_icp),local_pose))
            self.output_guard['icp_sync_sec']=(target_ns-icp_ns)*1e-9
            self.output_guard['time_pairing']='interpolated_and_motion_compensated'
            up_global=rotate_vector(global_pose.orientation,(0.,0.,1.))
            up_icp=rotate_vector(expected.orientation,(0.,0.,1.))
            tilt=math.degrees(math.acos(max(-1.,min(1.,sum(a*b for a,b in zip(up_global,up_icp))))))
            self.output_guard['tilt_error_deg']=tilt
            if tilt>self.p['max_output_tilt_error_deg']:
                raise ValueError(f'全局 EKF 与 ICP 倾角不一致（{tilt:.1f}°），已阻止 TF / 定位输出')
            delta=compose(global_pose,inverse(local_pose))
            message=deepcopy(message);message.header.stamp=rclpy.time.Time(nanoseconds=target_ns).to_msg()
            message.pose.pose.position.x,message.pose.pose.position.y,message.pose.pose.position.z=global_pose.position
            message.pose.pose.orientation.x,message.pose.pose.orientation.y,message.pose.pose.orientation.z,message.pose.pose.orientation.w=global_pose.orientation
        except ValueError as error:
            self.output_guard['reason']=str(error);self.reject('output_guard',error)
            return False
        self.output_guard['ok']=True
        if stamp<=self.last_pose_pub_stamp:return True
        tf=TransformStamped();tf.header=message.header;tf.child_frame_id=self.p['odom_frame']
        tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z=delta.position
        tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w=delta.orientation
        self.tf.sendTransform(tf)
        pose=PoseStamped();pose.header=message.header;pose.pose=message.pose.pose;self.pose_pub.publish(pose)
        if stamp-self.last_path_stamp>=1./self.p['trajectory_rate_hz']:
            self.trajectory.header.stamp=message.header.stamp;self.trajectory.poses.append(pose);self.trajectory.poses=self.trajectory.poses[-1500:]
            self.path_pub.publish(self.trajectory);self.last_path_stamp=stamp
        self.last_pose_pub_stamp=stamp
        self.output_samples.append(self.now_seconds())
        return True

    @staticmethod
    def message_ns(message):
        return message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec

    def output_tick(self):
        now=self.now_seconds()
        if (self.seeded and self.aligned and self.bias.bias is not None and self.sdk_sample()[0] is not None
            and valid_stamp(self.last_imu,now,self.p['imu_timeout'])
            and valid_stamp(self.last_cloud,now,self.p['cloud_timeout'])):
            self.publish_verified_pose()

    def tick(self):
        now=self.now_seconds();sdk,sdk_source=self.sdk_sample()
        fresh={'sdk':sdk is not None,'imu':valid_stamp(self.last_imu,now,self.p['imu_timeout']),
               'cloud':valid_stamp(self.last_cloud,now,self.p['cloud_timeout'])}
        local_fresh=bool(self.local) and valid_stamp(stamp_seconds(self.local[-1]),now,.3)
        global_fresh=self.global_odom is not None and valid_stamp(stamp_seconds(self.global_odom),now,.5)
        try:correction_age=float(self.diagnostic.get('last_correction_age_sec','nan'))+max(0,now-self.diagnostic_time)
        except ValueError:correction_age=math.inf
        # Diagnostics are only 1 Hz. Prefer the actual accepted measurement
        # stamps, otherwise a healthy 10 Hz ICP stream appears stale between reports.
        if self.accepted_icp:
            correction_age=now-stamp_seconds(self.accepted_icp[-1])
        if not math.isfinite(correction_age):correction_age=None
        sensors_ready=all(fresh.values())
        self.read_command(sensors_ready and self.bias.bias is not None and local_fresh)
        state=health_state(sensors_ready,self.bias.bias is not None,self.seeded,self.aligned,correction_age,local_fresh,global_fresh)
        if state=='tracking' and not self.publish_verified_pose():state='degraded'
        warnings=[]
        if self.output_guard.get('reason'):warnings.append(self.output_guard['reason'])
        if not self.p['time_alignment_verified']:warnings.append('MC / 雷达相对时间未完成实机标定')
        if not self.p['extrinsics_verified']:warnings.append('机身速度到前雷达使用 CAD 近似外参，未完成实机标定，仅作定位调试')
        if sdk_source=='missing':warnings.append('等待 OnMcData 速度流；不使用 1 Hz 状态速度')
        speed_report=self.speed_report if valid_stamp(self.speed_report_at,now,1.) else {}
        if sdk_source=='mc_stream' and not speed_report.get('rate_ok'):
            warnings.append('MC 速度已收到，实收频率待核对')
        if sdk_source=='mc_stream' and speed_report.get('clock_approximate'):
            warnings.append('MC 时间按源时间间隔与本机接收时刻对齐；非硬件同步')
        if self.input_clock.get('approximate')=='true':warnings.append('传感器使用共享时差估计，仅供调试；不是硬件时钟同步，未校准网络单向延迟')
        hz=0.
        if len(self.output_samples)>1 and now-self.output_samples[-1]<.3:
            span=self.output_samples[-1]-self.output_samples[0]
            if span>0:hz=(len(self.output_samples)-1)/span
        status={'session_id':self.session['id'],'map_version_id':self.session['version_id'],'wall_time':time.time(),'state':state,
                'localized':state=='tracking','navigation_ready':False,'sensors':fresh,'sdk_source':sdk_source,'speed_report':speed_report,'gyro_bias_samples':self.bias.count,
                'gyro_bias':self.bias.bias,'local_ekf_fresh':local_fresh,'global_ekf_fresh':global_fresh,'correction_age':correction_age,'head_direction':self.head_direction,
                'fusion':self.diagnostic,'input_clock':self.input_clock,'matcher':self.matcher,'output_guard':self.output_guard,'warnings':warnings,'rejections':self.rejections,'last_error':self.last_error,
                'command_result':self.command_result,'timing':{'mc_offset_sec':self.p['mc_time_offset_sec'],'verified':self.p['time_alignment_verified']},'output_rate_limit_hz':self.p['output_rate_hz'],'output_observed_hz':hz,'calibration':{'extrinsics_verified':self.p['extrinsics_verified'],'time_alignment_verified':self.p['time_alignment_verified']},'frames':{'map':self.p['map_frame'],'odom':self.p['odom_frame'],'tracking':self.p['tracking_frame']}}
        data=json.dumps(status,ensure_ascii=False,allow_nan=False)
        temporary=self.directory/'status.tmp';temporary.write_text(data);os.replace(temporary,self.directory/'status.json')
        message=String();message.data=data;self.status_pub.publish(message)

def main(args=None):
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(args=args,signal_handler_options=SignalHandlerOptions.NO);node=Supervisor()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()
