#!/usr/bin/env python3
"""Isolated real-component navigation graph with a control-driven robot fixture.

Only the transport/plant/sensors are simulated. Navigation goals go through the
real Action, native PCT, native GridMap/SCAN, tracker and typed safety gate. The
analytic test room is NOT bag data and NOT proof about the real robot's blind
regions. No synthetic collision validation or direct task-success publisher.
"""
import argparse
from collections import Counter,deque
import json
import hashlib
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import numpy as np
import yaml

# At most 120 s of the suite's typed streams plus bounded retirement drain.
# This is observer storage only, not a production queue or an evidence renewal.
# Preserve loss accounting even if this finite capacity is exceeded.
OBSERVATION_EVENT_CAPACITY = 60000


def box_returns(origins, directions, lower, upper):
    """Analytic physical surface returns; miss/parallel rays remain no return."""
    parallel=np.abs(directions)<1e-12
    outside=np.any(parallel&((origins<lower)|(origins>upper)),axis=1)
    with np.errstate(divide='ignore',invalid='ignore'):
        a=(lower-origins)/directions;b=(upper-origins)/directions
    near=np.max(np.where(parallel,-np.inf,np.minimum(a,b)),axis=1)
    far=np.min(np.where(parallel,np.inf,np.maximum(a,b)),axis=1)
    hit=(~outside)&(far>=np.maximum(near,.08))
    return np.where(hit,np.where(near>.08,near,far),np.inf)


def child(directory,duration,case):
    from d1max_pct_scan.isolated_zenoh import validate_environment
    validate_environment()
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
    from d1max_planning_interfaces.msg import (NavigationState,LocalNavigationState,MotionDemand,MotionValidation,
        ExecutionPermit,TrajectoryValidation,StopReport,SDKExecutionState,ExecutionHandoffGrant,
        PreparedMotionDemand,ExecutionCommitAck,SDKStationaryEvidence,TrajectoryAdmission,ReferenceReceipt,TrackerGeometryReceipt,TaggedBspline,TrackingProgress,RouteProgress)
    from d1max_navigation_bt_interfaces.action import Navigate
    from d1max_navigation_bt_interfaces.srv import ConfirmExecution
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Path as NavPath
    from sensor_msgs.msg import PointCloud2,PointField
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
    from d1max_pct_scan.ray_projection import FIELDS,RAY_DTYPE
    session=json.loads((directory/'session.json').read_text())
    scan_parameters=yaml.safe_load((directory/'scan.yaml').read_text())['/**']['ros__parameters']
    body_offset=scan_parameters['grid_map.double_cylinder_offset']
    body_radius=scan_parameters['grid_map.double_cylinder_radius']
    wall_x=.9 if case=='blocked_safe_distance' else .65
    audit=json.loads((Path(session['planning_manifest']).parent/'native_route_audit_lowest_slice.json').read_text())
    trial=next(q for q in audit['queries'] if q['start_index']=='gateway_start' and q['success'])
    start=np.asarray(trial['start_xyz']); goal=np.asarray(trial['goal_xyz'])
    box_case=case in ('static_box','dynamic_box')
    unobserved_case=case=='unobserved_start'
    long_case=case in ('long_straight','long_map_correction','global_gap','local_gap') or box_case or unobserved_case
    box_lower=np.array([-.18,-1.8,-.05]);box_upper=np.array([.18,-1.4,.65])
    if long_case:
        from d1max_pct_scan.source_identity import SourceIdentityBridge
        support=SourceIdentityBridge.from_artifacts(session['planning_manifest'])
        # Audited -Y corridor on the unchanged original-coordinate map.
        # The earlier +X endpoint required a 6.78 m detour, not a straight run.
        goal=(np.array([start[0]+3.2,start[1],0.]) if unobserved_case
              else np.array([start[0],start[1]-3.15,0.]))
        goal[2]=support.query(goal[:2].reshape(1,2))[0][0]
        trial=dict(start_xyz=start.tolist(),goal_xyz=goal.tolist(),
            selection=('legacy_detour_and_missing_beam_fault_injection' if unobserved_case
                       else 'audited_3.15m_minus_y_original_support_corridor'),
            note='Actual route eligibility is checked by the live native PCT service, not inherited from the short gateway audit.')
    delta=goal[:2]-start[:2]
    gradient=(goal[2]-start[2])*delta/float(delta@delta)
    initial_yaw=-math.pi/2 if long_case and not unobserved_case else 0.
    az,el=np.meshgrid(np.linspace(-math.pi,math.pi,720,endpoint=False),np.linspace(-1.4,1.4,96))
    dirs=np.c_[np.cos(el.ravel())*np.cos(az.ravel()),np.cos(el.ravel())*np.sin(az.ravel()),np.sin(el.ravel())]
    def stamp(out,ns): out.sec,out.nanosec=divmod(int(ns),10**9)
    class Plant(Node):
        def __init__(self):
            super().__init__('isolated_single_floor_plant')
            self.xyz=np.array([0.,0.,session['body_height']]); self.yaw=initial_yaw
            self.history=deque(maxlen=100)
            self.applied=None; self.applied_at=0.; self.last_tick=time.monotonic()
            self.counts=Counter(); self.last={}; self.route_hashes=set(); self.max_speed=0.; self.distance=0.
            self.events=deque(maxlen=OBSERVATION_EVENT_CAPACITY);self.event_keys={};self.dropped_events=0
            self.plant_samples=deque(maxlen=36000);self.tick_count=0
            self.state_sources={'local':deque(maxlen=18000),'global':deque(maxlen=18000)}
            self.route_display=[];self.late_route_display=None
            self.route_vertices=[]
            self.route_display_sub=self.create_subscription(NavPath,
                '/d1max/live_planning/committed_route_visual',self.on_route_display,
                QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.state_pub=self.create_publisher(NavigationState,'/d1max/localization/navigation/state',10)
            self.local_state_pub=self.create_publisher(LocalNavigationState,'/d1max/localization/navigation/local_state',10)
            self.ray_pub=self.create_publisher(PointCloud2,'/d1max/localization/perception/rays_raw',qos_profile_sensor_data)
            self.tf_pub=self.create_publisher(TFMessage,'/tf_static',QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.create_subscription(MotionDemand,'/d1max/live_planning/execution/applied_motion',self.on_motion,10)
            self.create_subscription(TaggedBspline,'/d1max/live_planning/scan_tagged_bspline',self.on_curve,10)
            for label,topic,cls in [('permit','execution/permit',ExecutionPermit),('validation','execution/validation',TrajectoryValidation),
                ('demand','execution/demand',MotionDemand),('safe_demand','execution/safe_demand',MotionDemand),
                ('motion_validation','execution/motion_validation',MotionValidation),
                ('handoff_grant','execution/handoff_grant',ExecutionHandoffGrant),
                ('prepared_demand','execution/prepared_demand',PreparedMotionDemand),
                ('safe_prepared_demand','execution/safe_prepared_demand',PreparedMotionDemand),
                ('commit_ack','execution/commit_ack',ExecutionCommitAck),
                ('stationary_evidence','execution/stationary_evidence',SDKStationaryEvidence),
                ('trajectory_admission','execution/admission',TrajectoryAdmission),
                ('tracker_geometry_receipt','execution/tracker_geometry_receipt',TrackerGeometryReceipt),
                ('tracking_progress','tracking_progress',TrackingProgress),
                ('route_progress','execution/route_progress',RouteProgress),
                ('reference_receipt','execution/reference_receipt',ReferenceReceipt),
                ('blocked_entry','execution/blocked_entry',MotionValidation),
                ('stop','execution/stop_report',StopReport),('sdk','execution/sdk_state',SDKExecutionState)]:
                self.create_subscription(cls,'/d1max/live_planning/'+topic,lambda m,k=label:self.observe(k,m),10)
            for label,topic in [('bt','bt/status'),('global','global_status'),('reference','scan_bridge_status'),
                ('native','native_local_attempt_debug'),('rays','rays_status'),('projector','ray_projector_status'),
                ('safety','execution/safety_status'),('tracker','tracker_status'),
                ('pipeline','execution/pipeline_timing')]:
                qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL) if label=='bt' else 10
                self.create_subscription(String,'/d1max/live_planning/'+topic,lambda m,k=label:self.on_status(k,m),qos)
            self.action=ActionClient(self,Navigate,'/d1max/live_planning/bt/navigate')
            self.confirm=self.create_client(ConfirmExecution,'/d1max/live_planning/bt/confirm_execution')
            self.goal_future=self.result_future=self.confirm_future=None
            self.geometry_ready=False; self.confirm_attempts=0
            self.cancel_future=None;self.disturbance_at=None;self.drop_end=None
            self.map_shift=np.zeros(3)
            self.goal_handle=None; self.start_wall=time.monotonic(); self.outcome=None
            self.retiring=False
            # Record the observer lifecycle independently of delayed component
            # startup. This is not localization, permission or motion evidence.
            self.event('observer_started',event_capacity=self.events.maxlen)
            self.create_timer(.02,self.tick); self.create_timer(.1,self.scan); self.create_timer(.5,self.task)
            self.static()
        def static(self):
            transforms=[]
            for parent,child in [('d1max_loc_base_link','d1max_loc_tracking'),('d1max_loc_tracking','d1max_loc_lidar')]:
                t=TransformStamped(); t.header.frame_id=parent; t.child_frame_id=child
                t.transform.rotation.w=1.; transforms.append(t)
            self.tf_pub.publish(TFMessage(transforms=transforms))
        @staticmethod
        def path_identity(message):
            xyz=[[p.pose.position.x,p.pose.position.y,p.pose.position.z] for p in message.poses]
            return dict(points=len(xyz),geometry_sha256=hashlib.sha256(json.dumps(xyz).encode()).hexdigest(),
                frame=message.header.frame_id,source_ns=message.header.stamp.sec*10**9+message.header.stamp.nanosec)
        def on_route_display(self,message):
            self.route_display.append(self.path_identity(message));self.route_display=self.route_display[-8:]
            self.route_vertices=[[p.pose.position.x,p.pose.position.y,p.pose.position.z]
                for p in message.poses]
        def subscribe_late_display(self):
            self.late_display_sub=self.create_subscription(NavPath,
                '/d1max/live_planning/committed_route_visual',
                lambda message:setattr(self,'late_route_display',self.path_identity(message)),
                QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        def on_curve(self,message):
            # Preserve the actual original entry evidence for post-run diagnosis;
            # this observer neither prepares nor validates a trajectory.
            self.counts['tagged_bspline']+=1
            ns=lambda s:s.sec*10**9+s.nanosec
            xyz=lambda v:[v.x,v.y,v.z]
            pose=message.join_pose
            self.event('tagged_bspline',trajectory_id=message.trajectory.traj_id,
                session_id=message.session_id,generation=message.generation,
                frame_id=message.frame_id,task_id=message.task_id,route_hash=message.route_hash,
                anchor_id=message.anchor_id,anchor_revision=message.anchor_revision,
                start_time_ns=ns(message.trajectory.start_time),join_source_stamp_ns=ns(message.join_source_stamp),
                valid_start_time=message.valid_start_time,valid_start_arc_length=message.valid_start_arc_length,
                join_position=xyz(pose.position),
                join_orientation=[pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z],
                join_velocity=xyz(message.join_twist.linear),
                join_acceleration=xyz(message.join_acceleration.linear),
                join_acceleration_valid=message.join_acceleration_valid,
                spline_order=message.trajectory.order,knots=list(message.trajectory.knots),
                control_points=[xyz(p) for p in message.trajectory.pos_pts])
        def observe(self,key,m):
            self.counts[key]+=1
            self.last[key]=dict(reason=getattr(m,'reason',''),
                valid=getattr(m,'valid',False),allowed=getattr(m,'allowed',False),
                measured_stop_confirmed=getattr(m,'measured_stop_confirmed',False),
                execution_id=getattr(m,'execution_id',''))
            values={name:getattr(m,name) for name in ('schema_version','reason','phase','valid','allowed','revoked','geometry_committed',
                'trajectory_id','validation_sequence','control_epoch','grant_ready','control_owned','stop_submitted',
                'measured_stop_confirmed','stop_request_id','sequence','demand_sequence','map_snapshot_revision',
                'trajectory_validation_sequence','motion_validation_sequence','permit_sequence','hold','safety_checked',
                'handoff_id','grant_sequence','expected_commit_sequence','previous_commit_sequence','commit_sequence',
                'applied','write_submitted','write_acknowledged','writer_commit_sequence','applied_trajectory_id',
                'candidate_trajectory_id','incumbent_trajectory_id','entry_admission_sequence',
                'whole_curve','remaining_curve','checked_from_time','checked_to_time','curve_duration',
                'valid_start_time','reverse_margin_m','collision_policy',
                'support_reference_id','support_hash','frame_id','transition_mode',
                'accepted','expected_trajectory_id','usable','nonzero_blocked','zero_write_sequence',
                'mc_raw_stamp_ns','mc_clock_epoch','time_basis','stationary_samples',
                'stationary_duration_sec','measured_linear_mps','measured_angular_radps',
                'mc_capture_delay_bound_sec','physical_acceptance_verified','installed',
                'installation_sequence','admission_sequence') if hasattr(m,name)}
            values.update({name:getattr(m,name) for name in
                ('execution_id','sdk_session','sdk_arm_generation','transport_mode') if hasattr(m,name)})
            if hasattr(m,'candidate'):
                for name,p in (('candidate',m.candidate),('incumbent',m.incumbent)):
                    values[name]=dict(trajectory_id=p.trajectory_id,permit_sequence=p.sequence,
                        validation_sequence=p.validation_sequence,geometry_committed=p.geometry_committed,
                        frame_id=p.frame_id,
                        allowed=p.allowed,phase=p.phase,revoked=p.revoked,sdk_session=p.sdk_session,
                        sdk_arm_generation=p.sdk_arm_generation,control_epoch=p.control_epoch,
                        execution_id=p.execution_id,transport_mode=p.transport_mode,
                        source_stamp_ns=p.source_stamp.sec*10**9+p.source_stamp.nanosec,
                        valid_until_ns=p.valid_until.sec*10**9+p.valid_until.nanosec,
                        version={k:getattr(p.version,k) for k in p.version.get_fields_and_field_types()})
                if m.transition_mode==1:
                    e=m.stationary_evidence
                    values['stationary_evidence']={k:getattr(e,k) for k in
                        ('schema_version','sequence','writer_commit_sequence','applied_trajectory_id',
                         'zero_write_sequence','mc_raw_stamp_ns','mc_clock_epoch','time_basis',
                         'stationary_samples','stationary_duration_sec','measured_linear_mps',
                         'measured_angular_radps','mc_capture_delay_bound_sec','usable','nonzero_blocked',
                         'physical_acceptance_verified','execution_id','control_epoch','sdk_session',
                         'sdk_arm_generation','transport_mode')}
                    values['stationary_evidence']['version']={k:getattr(e.version,k) for k in e.version.get_fields_and_field_types()}
                    for k in ('zero_ack_at','source_stamp','received_stamp','capture_lower_bound','capture_upper_bound','valid_until'):
                        s=getattr(e,k);values['stationary_evidence'][k+'_ns']=s.sec*10**9+s.nanosec
            if hasattr(m,'candidate_version'):
                values['candidate_version']={k:getattr(m.candidate_version,k) for k in m.candidate_version.get_fields_and_field_types()}
                values['incumbent_version']={k:getattr(m.incumbent_version,k) for k in m.incumbent_version.get_fields_and_field_types()}
            if hasattr(m,'demand'):
                d=m.demand
                values.update(trajectory_id=d.trajectory_id,demand_sequence=d.sequence,
                    vx=d.velocity.linear.x,wz=d.velocity.angular.z,
                    permit_sequence=d.permit_sequence,motion_validation_sequence=d.motion_validation_sequence,
                    demand_source_stamp_ns=d.source_stamp.sec*10**9+d.source_stamp.nanosec,
                    body_source_stamp_ns=d.body_source_stamp.sec*10**9+d.body_source_stamp.nanosec,
                    valid_until_ns=d.valid_until.sec*10**9+d.valid_until.nanosec)
                values['version']={k:getattr(d.version,k) for k in d.version.get_fields_and_field_types()}
                values['entry_admission_sequence']=m.entry_admission.sequence
                values['entry_validation_sequence']=m.entry_admission.validation_sequence
                values.update({name:getattr(d,name) for name in
                    ('execution_id','sdk_session','sdk_arm_generation','control_epoch','transport_mode')})
            if hasattr(m,'velocity'):
                values.update(vx=m.velocity.linear.x,wz=m.velocity.angular.z)
            if hasattr(m,'applied_velocity'):
                values.update(vx=m.applied_velocity.linear.x,wz=m.applied_velocity.angular.z)
            if key=='tracking_progress':
                values.update({name:getattr(m,name) for name in
                    ('session_id','task_id','route_hash','anchor_revision','generation',
                     'route_id','segment_id','map_version_id','localization_epoch',
                     'localization_seed_id','anchor_id','context_sequence','curve_time',
                     'arc_length','s_committed','acceleration_valid','holding')})
                values.update(frame_id=m.header.frame_id,
                    source_stamp_ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec,
                    position=[m.pose.position.x,m.pose.position.y,m.pose.position.z],
                    quaternion=[m.pose.orientation.x,m.pose.orientation.y,m.pose.orientation.z,m.pose.orientation.w],
                    linear_velocity=[m.twist.linear.x,m.twist.linear.y,m.twist.linear.z],
                    angular_velocity=[m.twist.angular.x,m.twist.angular.y,m.twist.angular.z])
            if key=='route_progress':
                xyz=lambda p:[p.x,p.y,p.z]
                quat=lambda q:[q.x,q.y,q.z,q.w]
                values.update(edge_index=m.edge_index,measured_arc_m=m.measured_arc_m,
                    confirmed_arc_m=m.confirmed_arc_m,cross_track_m=m.cross_track_m,
                    body_reference_height_m=m.body_reference_height_m,
                    source_map_body_xyz=xyz(m.source_map_body_xyz),
                    odom_body_frame=m.odom_body_pose.header.frame_id,
                    odom_body_source_stamp_ns=m.odom_body_pose.header.stamp.sec*10**9+m.odom_body_pose.header.stamp.nanosec,
                    odom_body_xyz=xyz(m.odom_body_pose.pose.position),
                    odom_body_quaternion=quat(m.odom_body_pose.pose.orientation),
                    map_from_odom_xyz=xyz(m.map_from_odom.position),
                    map_from_odom_quaternion=quat(m.map_from_odom.orientation))
            if hasattr(m,'version'):
                values['version']={k:getattr(m.version,k) for k in m.version.get_fields_and_field_types()}
                values.update({name:getattr(m.version,name) for name in ('task_id','route_hash','reference_generation',
                    'anchor_id','anchor_revision','context_sequence','localization_epoch','localization_seed_id')})
            for name in ('source_stamp','valid_until','check_begin','check_end','body_source_stamp','demand_source_stamp',
                         'demand_body_source_stamp','front_ray_source_stamp','rear_ray_source_stamp','applied_at',
                         'transition_deadline','retain_incumbent_until','entry_source_stamp','demand_valid_until',
                         'checked_at','zero_ack_at','received_stamp','capture_lower_bound','capture_upper_bound','installed_at','anchor_source_stamp'):
                if hasattr(m,name):
                    s=getattr(m,name);values[name+'_ns']=s.sec*10**9+s.nanosec
            self.event(key,**values)
            if key=='permit' and m.version.route_hash:self.route_hashes.add(m.version.route_hash)
            if key=='permit':self.geometry_ready=m.geometry_committed and not m.revoked
        def on_status(self,key,m):
            try:self.last[key]=json.loads(m.data)
            except ValueError:self.last[key]={'raw':m.data[:600]}
            self.counts[key]+=1
            value=self.last[key]
            if key=='pipeline':
                self.event(key,**value)
                return
            self.event(key,**{name:value[name] for name in ('phase','reason','active','trajectory_id',
                'execution_authorized','physical_stop_confirmed','worker_reason',
                'prepare_reason','permit_reject_reason','writer_commit_sequence','execution_id',
                'control_epoch','prepared_handoff_id','handoff_reason','route_hash',
                'execution_blocked_age_s','execution_progress_reason',
                'handoff_ready_window_s',
                'localization_epoch','localization_seed_id','join_diagnostic',
                'entry_reobserved','original_join_source_stamp_ns') if name in value})
        def event(self,key,**data):
            if self.event_keys.get(key)==data:return
            self.event_keys[key]=data
            if len(self.events)==self.events.maxlen:self.dropped_events+=1
            self.events.append(dict(elapsed_s=time.monotonic()-self.start_wall,key=key,**data))
        def on_motion(self,m):
            if m.transport_mode!='isolated_mock' or m.version.session_id!=session['id']:
                raise RuntimeError('foreign_plant_motion')
            self.applied,self.applied_at=m,time.monotonic(); self.counts['applied']+=1
            self.event('motion',reason=m.reason,hold=m.hold,x=m.velocity.linear.x,yaw=m.velocity.angular.z,
                trajectory_id=m.trajectory_id,permit_sequence=m.permit_sequence,
                execution_id=m.execution_id,control_epoch=m.control_epoch,sdk_session=m.sdk_session,
                sdk_arm_generation=m.sdk_arm_generation,observer_receipt_ns=self.get_clock().now().nanoseconds,
                transport_mode=m.transport_mode,
                version={k:getattr(m.version,k) for k in m.version.get_fields_and_field_types()},
                demand_sequence=m.sequence,validation_sequence=m.validation_sequence,
                motion_validation_sequence=m.motion_validation_sequence,
                source_stamp_ns=m.source_stamp.sec*10**9+m.source_stamp.nanosec,
                valid_until_ns=m.valid_until.sec*10**9+m.valid_until.nanosec)
        def tick(self):
            wall=time.monotonic(); dt=min(.05,wall-self.last_tick); self.last_tick=wall
            vx=wz=0.
            if self.applied is not None and wall-self.applied_at<.25:
                vx=self.applied.velocity.linear.x; wz=self.applied.velocity.angular.z
            self.max_speed=max(self.max_speed,abs(vx)); self.distance+=abs(vx)*dt
            self.yaw+=wz*dt
            self.xyz[:2]+=vx*dt*np.array([math.cos(self.yaw),math.sin(self.yaw)])
            self.xyz[2]=session['body_height']+float(gradient@self.xyz[:2])
            ns=self.get_clock().now().nanoseconds
            self.history.append((ns,*self.xyz,self.yaw))
            m=NavigationState(schema_version=2,session_id=session['id'],map_version_id=session['version_id'],
                localization_epoch=1,localization_seed_id='isolated-initial',usable=True,reason='control_driven_plant_not_real_localization')
            for s in (m.source_stamp,m.posterior_stamp,m.imu_stamp):stamp(s,ns)
            for o,frame,position in [(m.local_odometry,'d1max_loc_odom',self.xyz),
                (m.global_odometry,'d1max_loc_map',self.xyz+start+self.map_shift)]:
                stamp(o.header.stamp,ns);o.header.frame_id=frame;o.child_frame_id='d1max_loc_base_link'
                o.pose.pose.position.x,o.pose.pose.position.y,o.pose.pose.position.z=map(float,position)
                o.pose.pose.orientation.z=math.sin(self.yaw/2);o.pose.pose.orientation.w=math.cos(self.yaw/2)
                o.twist.twist.linear.x=vx
                o.twist.twist.linear.z=vx*float(gradient@np.array([math.cos(self.yaw),math.sin(self.yaw)]))
                o.twist.twist.angular.z=wz
            # Withhold only the global-pair correction transport. Local plant
            # measurements remain original and current; this is not a sensor
            # dropout or a fabricated global heartbeat.
            missing_local=(case=='local_gap' and self.drop_end is not None and wall<self.drop_end)
            if not missing_local and not (case=='global_gap' and self.drop_end is not None and wall<self.drop_end):
                self.state_pub.publish(m)
                self.counts['plant_global_state']+=1
                self.state_sources['global'].append(dict(elapsed_s=wall-self.start_wall,source_ns=ns))
            if not missing_local:
                self.local_state_pub.publish(LocalNavigationState(schema_version=1,session_id=m.session_id,
                    map_version_id=m.map_version_id,localization_epoch=m.localization_epoch,
                    localization_seed_id=m.localization_seed_id,local_odometry=m.local_odometry,
                    source_stamp=m.source_stamp,posterior_stamp=m.posterior_stamp,imu_stamp=m.imu_stamp,
                    usable=m.usable,reason=m.reason))
                self.counts['plant_local_state']+=1
                o=m.local_odometry
                self.state_sources['local'].append(dict(elapsed_s=wall-self.start_wall,source_ns=ns,
                    posterior_ns=ns,imu_ns=ns,usable=m.usable,
                    session_id=m.session_id,map_version_id=m.map_version_id,
                    localization_epoch=m.localization_epoch,localization_seed_id=m.localization_seed_id,
                    frame_id=o.header.frame_id,
                    position=[o.pose.pose.position.x,o.pose.pose.position.y,o.pose.pose.position.z],
                    quaternion=[o.pose.pose.orientation.x,o.pose.pose.orientation.y,o.pose.pose.orientation.z,o.pose.pose.orientation.w],
                    linear_velocity=[o.twist.twist.linear.x,o.twist.twist.linear.y,o.twist.twist.linear.z],
                    angular_velocity=[o.twist.twist.angular.x,o.twist.twist.angular.y,o.twist.twist.angular.z]))
            self.tick_count+=1
            if self.tick_count%5==0:
                self.plant_samples.append(dict(elapsed_s=wall-self.start_wall,
                    x=float(self.xyz[0]),y=float(self.xyz[1]),z=float(self.xyz[2]),
                    yaw=self.yaw,vx=vx,wz=wz))
        def scan(self):
            self.static()
            if len(self.history)<5:return
            ns=self.get_clock().now().nanoseconds-60_000_000
            offsets=(np.arange(len(dirs))%100)*100000
            times=(ns+offsets).astype(float)
            history=np.asarray(self.history,dtype=float)
            if times.min()<history[0,0] or times.max()>history[-1,0]:return
            positions=np.c_[[np.interp(times,history[:,0],history[:,i]) for i in (1,2,3)]].T
            yaws=np.interp(times,history[:,0],history[:,4]); c,s=np.cos(yaws),np.sin(yaws)
            worlddirs=np.c_[c*dirs[:,0]-s*dirs[:,1],s*dirs[:,0]+c*dirs[:,1],dirs[:,2]]
            for sensor,ox in ((0,.4),(1,-.4)):
                if case=='rear_dropout' and sensor==1 and self.drop_end is not None and time.monotonic()<self.drop_end:
                    continue
                origin_body=np.array([ox,0.,-.05]);origin=positions+np.c_[c*ox,s*ox,np.full(len(c),-.05)]
                normal=np.array([-gradient[0],-gradient[1],1.]); floor=-(origin@normal)/(worlddirs@normal)
                ceiling=(2.-origin[:,2])/worlddirs[:,2]
                candidates=[floor,ceiling]
                for axis in (0,1):
                    for edge in ((-6.,6.) if long_case else (-3.,3.)):
                        candidates.append((edge-origin[:,axis])/worlddirs[:,axis])
                if case in ('blocked','blocked_safe_distance') and self.disturbance_at is not None:
                    candidates.append((wall_x-origin[:,0])/worlddirs[:,0])
                if case=='static_box' or (case=='dynamic_box' and self.disturbance_at is not None):
                    candidates.append(box_returns(origin,worlddirs,box_lower,box_upper))
                distances=np.stack(candidates);distances[distances<=.08]=np.inf
                depth=distances.min(axis=0)
                # These are actual analytic-room returns, not an 8 m scanner.
                # Discarding a distant endpoint would also erase every nearer
                # free-space observation along its beam. GridMap owns clipping
                # to its rolling bounds and preserves the truncated ray's
                # no-hit semantics; never invent a hit at the clip distance.
                good=np.isfinite(depth)
                if unobserved_case:
                    # Preserve the known bad input as a negative test: this
                    # case must time out safely, never be declared navigable.
                    good&=depth<=8.
                endpoints=origin_body+dirs[good]*depth[good,None]
                points=np.zeros(len(endpoints),dtype=RAY_DTYPE)
                for name,values in zip(('x','y','z'),endpoints.T):points[name]=values
                for name,value in zip(('origin_x','origin_y','origin_z'),origin_body):points[name]=value
                points['sensor_id']=sensor;points['ring']=np.arange(len(points))%96
                points['source_index']=np.arange(len(points));points['intensity']=1.
                points['offset_time']=offsets[good]
                points['timestamp']=points['source_timestamp']=(ns+offsets[good])*1e-9
                points['raw_timestamp']=(ns+offsets[good])/1000.
                cloud=PointCloud2(height=1,width=len(points),fields=[PointField(name=a,offset=b,datatype=c,count=d) for a,b,c,d in FIELDS],
                    is_bigendian=False,point_step=64,row_step=len(points)*64,data=points.tobytes(),is_dense=True)
                cloud.header.frame_id='d1max_loc_lidar';stamp(cloud.header.stamp,ns);self.ray_pub.publish(cloud)
        def task(self):
            if self.result_future and self.result_future.done():
                r=self.result_future.result().result
                self.outcome=dict(success=r.success,reason=r.reason,retirement_confirmed=r.retirement_confirmed,
                    physical_stop_confirmed=r.physical_stop_confirmed)
            if self.retiring or self.outcome is not None:return
            if time.monotonic()-self.start_wall<5:return
            if self.goal_future is None and self.action.server_is_ready():
                g=Navigate.Goal(schema_version=2,goal_kind='3d',has_goal_yaw=case=='goal_yaw',goal_yaw_tolerance_rad=.1745329252)
                g.goal.header.frame_id='d1max_loc_map';g.goal.header.stamp=self.get_clock().now().to_msg()
                g.goal.pose.orientation.w=1.
                if case=='goal_yaw':
                    g.goal.pose.orientation.z=math.sin(.35/2);g.goal.pose.orientation.w=math.cos(.35/2)
                g.goal.pose.position.x,g.goal.pose.position.y,g.goal.pose.position.z=map(float,goal)
                self.goal_future=self.action.send_goal_async(g)
            if self.goal_future and self.goal_future.done() and self.result_future is None:
                self.goal_handle=self.goal_future.result()
                if self.goal_handle.accepted:self.result_future=self.goal_handle.get_result_async()
                else:self.outcome={'success':False,'reason':'goal_rejected'}
            status=self.last.get('bt',{})
            if self.confirm_future is not None and self.confirm_future.done():
                result=self.confirm_future.result()
                self.event('confirmation',accepted=result.accepted,reason=result.reason)
                if not result.accepted and result.reason in ('waiting_matching_native_and_tracker_preparation','waiting_single_sdk_writer'):
                    self.confirm_future=None
            if self.confirm_future is None and self.geometry_ready and all(status.get(k) for k in ('task_id','route_id','route_hash')) and self.confirm.service_is_ready():
                request=ConfirmExecution.Request(schema_version=2,session_id=session['id'],
                    task_id=status['task_id'],route_id=status['route_id'],route_hash=status['route_hash'],request_id='isolated-test-confirmation')
                self.confirm_future=self.confirm.call_async(request)
                self.confirm_attempts+=1
            if self.distance>.02 and self.disturbance_at is None and case not in ('normal','goal_yaw','long_straight','static_box'):
                self.disturbance_at=time.monotonic()
                self.event('fixture_disturbance',case=case,x=float(self.xyz[0]),yaw=self.yaw,
                    vx=self.applied.velocity.linear.x if self.applied else 0.,
                    wall_x=wall_x if case in ('blocked','blocked_safe_distance') else None,
                    envelope_clearance_m=wall_x-float(self.xyz[0])-abs(body_offset*math.cos(self.yaw))-body_radius)
                if case=='cancel' and self.goal_handle:
                    self.cancel_future=self.goal_handle.cancel_goal_async()
                elif case=='rear_dropout':self.drop_end=time.monotonic()+1.
                elif case in ('global_gap','local_gap'):self.drop_end=time.monotonic()+1.
                elif case in ('map_correction','long_map_correction'):self.map_shift[0]=.12
        def report(self):
            confirmation=None
            if self.confirm_future and self.confirm_future.done():
                r=self.confirm_future.result();confirmation=dict(accepted=r.accepted,reason=r.reason,execution_authorized=r.execution_authorized)
            elapsed=time.monotonic()-self.start_wall
            validations=[e for e in self.events if e['key']=='validation' and 'check_begin_ns' in e]
            motion_checks=[e for e in self.events if e['key']=='motion_validation' and 'check_begin_ns' in e]
            timing={}
            for name,values in {
                'native_validation_ms':[(e['check_end_ns']-e['check_begin_ns'])/1e6 for e in validations],
                'oldest_ray_at_check_ms':[(e['check_end_ns']-min(e['front_ray_source_stamp_ns'],e['rear_ray_source_stamp_ns']))/1e6 for e in validations],
                'actual_command_sweep_ms':[(e['check_end_ns']-e['check_begin_ns'])/1e6 for e in motion_checks],
                'command_age_at_check_end_ms':[(e['check_end_ns']-e['demand_source_stamp_ns'])/1e6 for e in motion_checks],
            }.items():
                finite=[x for x in values if math.isfinite(x) and 0<=x<100000]
                timing[name]={'samples':len(finite),'p50':float(np.percentile(finite,50)),
                    'p95':float(np.percentile(finite,95)),'p99':float(np.percentile(finite,99)),
                    'max':max(finite)} if finite else {'samples':0}
            clearance=min((wall_x-s['x']-abs(body_offset*math.cos(s['yaw']))-body_radius
                           for s in self.plant_samples),default=wall_x-body_offset-body_radius)
            box_clearance=float('inf')
            if box_case:
                for sample in self.plant_samples:
                    for sign in (-1.,1.):
                        center=np.array([sample['x'],sample['y']])+sign*body_offset*np.array([
                            math.cos(sample['yaw']),math.sin(sample['yaw'])])
                        delta=np.maximum(np.maximum(box_lower[:2]-center,center-box_upper[:2]),0.)
                        box_clearance=min(box_clearance,float(np.linalg.norm(delta))-body_radius)
            return dict(kind='isolated_control_driven_analytic_room_real_navigation_components',
                physical_acceptance=False,case=case,session_id=session['id'],source_trial=trial,counts=dict(self.counts),
                obstacle_fixture=(dict(wall_x=wall_x,minimum_safety_envelope_clearance_m=clearance,
                    body_cylinder_offset=body_offset,body_cylinder_radius=body_radius)
                    if case in ('blocked','blocked_safe_distance') else
                    dict(lower=box_lower.tolist(),upper=box_upper.tolist(),
                        minimum_safety_envelope_clearance_m=box_clearance,
                        meaning='bounded_box_static_or_step_appearance_not_a_pedestrian_model') if box_case else None),
                elapsed_s=elapsed,whole_run_message_rates_hz={k:v/elapsed for k,v in self.counts.items()},timing=timing,
                max_speed_mps=self.max_speed,distance_m=self.distance,final_local_xyz=self.xyz.tolist(),
                sensor_fixture=dict(horizontal_samples=720,vertical_samples=96,
                    room_half_extent_m=6. if long_case else 3.,
                    return_policy='actual_analytic_room_endpoints_native_grid_clips_rays',
                    fault_injection=('discard_all_beams_with_endpoint_beyond_8m' if unobserved_case else None),
                    ground_xy_gradient=gradient.tolist(),initial_yaw_rad=initial_yaw,
                    physical_calibration=False),
                final_map_shift=self.map_shift.tolist(),final_yaw_rad=self.yaw,
                handoff_contract=session['handoff_contract'],
                route_hashes=sorted(self.route_hashes),last=self.last,confirmation=confirmation,
                confirmation_attempts=self.confirm_attempts,events=list(self.events),
                observation_window=dict(event_capacity=self.events.maxlen,dropped_events=self.dropped_events),
                plant_samples=list(self.plant_samples),route_display=self.route_display,
                state_sources={k:list(v) for k,v in self.state_sources.items()},
                committed_route_xyz=self.route_vertices,
                late_route_display=self.late_route_display,outcome=self.outcome)
    rclpy.init();node=Plant();deadline=time.monotonic()+duration
    try:
        while time.monotonic()<deadline and node.outcome is None:rclpy.spin_once(node,timeout_sec=.01)
        evaluation_deadline_reached=node.outcome is None
        if evaluation_deadline_reached:
            # Keep the CONTROL-DRIVEN plant and its original-source telemetry
            # alive while the task retires. Destroying it immediately after a
            # zero ACK invents a missing-MC failure instead of testing stopping.
            # This drain cannot turn a timed-out navigation into a passing run.
            node.retiring=True
            node.event('fixture_retirement_drain',reason='evaluation_deadline_reached',budget_s=6.)
            if node.goal_handle and node.last.get('bt',{}).get('phase') not in ('stopping','terminal','failed'):
                node.cancel_future=node.goal_handle.cancel_goal_async()
            drain_deadline=time.monotonic()+6.
            while time.monotonic()<drain_deadline and node.outcome is None:
                rclpy.spin_once(node,timeout_sec=.01)
        if node.outcome is not None:
            node.subscribe_late_display();late_deadline=time.monotonic()+1.
            while time.monotonic()<late_deadline and node.late_route_display is None:
                rclpy.spin_once(node,timeout_sec=.01)
        report=node.report()
        report['evaluation_deadline_reached']=evaluation_deadline_reached
        (directory/'graph_report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
        print(json.dumps(dict(outcome=report['outcome'],counts=report['counts'],distance_m=report['distance_m'])),flush=True)
        result=report['outcome']
        stop=report['last'].get('stop',{})
        software_stopped=(stop.get('measured_stop_confirmed') is True and
            stop.get('execution_id')==report['last'].get('bt',{}).get('execution_id') and
            bool(stop.get('execution_id')))
        passed=(result and result['retirement_confirmed'] and software_stopped and not result['physical_stop_confirmed'] and
            ((case=='cancel' and not result['success'] and node.cancel_future is not None)
             or (case in ('blocked','blocked_safe_distance') and not result['success'] and 'block' in result['reason']
                 and report['obstacle_fixture']['minimum_safety_envelope_clearance_m']>=0.)
             or (case not in ('cancel','blocked','blocked_safe_distance') and result['success']
                 and report['distance_m']>(2. if long_case else .1))))
        display=report['route_display']
        passed=passed and bool(display) and display[-1]['points']>1 and report['late_route_display']==display[-1]
        if box_case:passed=passed and report['obstacle_fixture']['minimum_safety_envelope_clearance_m']>=0.
        if unobserved_case:
            passed=bool(result and not result['success'] and result['retirement_confirmed']
                and result['reason'].startswith(('follow_map_timeout:', 'follow_reference_timeout:',
                    'follow_trajectory_timeout:', 'follow_preparation_episode_timeout:'))
                and not result['physical_stop_confirmed']
                and not (report['confirmation'] or {}).get('accepted',False)
                and not report['counts'].get('applied',0)
                and not report['last'].get('bt',{}).get('execution_authorized',False)
                and report['distance_m']==0.)
        return 0 if passed and not evaluation_deadline_reached else 2
    finally:node.destroy_node();rclpy.shutdown()


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--duration',type=float,default=45.)
    p.add_argument('--case',choices=('normal','cancel','rear_dropout','map_correction','goal_yaw','blocked','blocked_safe_distance','long_straight','long_map_correction','global_gap','local_gap','static_box','dynamic_box','unobserved_start'),default='normal')
    p.add_argument('--map-directory',type=Path)
    p.add_argument('--child',action='store_true');a=p.parse_args();directory=a.output.resolve()
    if a.child:return child(directory,a.duration,a.case)
    from d1max_pct_scan.isolated_zenoh import private_router,stop_owned
    from d1max_pct_scan.single_floor_session import prepare,verify
    import psutil
    prepare(directory,**({'map_directory':a.map_directory} if a.map_directory else {}));verify(directory,seal_runtime=True)
    with private_router(directory/'zenoh') as env:
        with (directory/'supervisor.log').open('w') as stream:
            graph=subprocess.Popen([sys.executable,'-m','d1max_pct_scan.single_floor_session','run','--session',str(directory)],
                env=env,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
            plant=None;resource_samples=[];previous_cpu={};previous_wall=time.monotonic()
            try:
                plant=subprocess.Popen([sys.executable,__file__,'--child','--output',str(directory),
                    '--duration',str(a.duration),'--case',a.case],env=env)
                while plant.poll() is None:
                    now=time.monotonic();processes=[]
                    for pid in (graph.pid,plant.pid):
                        try:
                            parent=psutil.Process(pid);processes.extend([parent,*parent.children(recursive=True)])
                        except psutil.Error:pass
                    sampled={}
                    for process in processes:
                        try:
                            cpu=process.cpu_times();total=cpu.user+cpu.system
                            old=previous_cpu.get(process.pid,total)
                            sampled[process.pid]=dict(pid=process.pid,name=process.name(),
                                rss_bytes=process.memory_info().rss,cpu_seconds=total,
                                cpu_cores=max(0.,total-old)/max(1e-6,now-previous_wall))
                        except psutil.Error:continue
                    previous_cpu={pid:row['cpu_seconds'] for pid,row in sampled.items()};previous_wall=now
                    resource_samples.append(dict(monotonic_s=now,processes=list(sampled.values()),
                        rss_bytes_sum=sum(row['rss_bytes'] for row in sampled.values()),
                        cpu_cores_sum=sum(row['cpu_cores'] for row in sampled.values())))
                    resource_samples=resource_samples[-7200:]
                    time.sleep(1.)
                return plant.returncode
            finally:
                if plant is not None and plant.poll() is None:
                    plant.terminate()
                    try:plant.wait(timeout=3)
                    except subprocess.TimeoutExpired:plant.kill();plant.wait()
                if graph.poll() is None:
                    os.killpg(graph.pid,signal.SIGTERM)
                    try:graph.wait(timeout=20)
                    except subprocess.TimeoutExpired:stop_owned(graph)
                (directory/'resource_report.json').write_text(json.dumps(dict(
                    scope='own_navigation_graph_plus_control_driven_sensor_fixture',
                    note='1 Hz sampled RSS sum double-counts shared pages; CPU excludes first sample and is not a NUC or one-hour acceptance.',
                    physical_acceptance=False,samples=resource_samples,
                    peak_sampled_rss_bytes_sum=max((s['rss_bytes_sum'] for s in resource_samples),default=0),
                    peak_sampled_cpu_cores_sum=max((s['cpu_cores_sum'] for s in resource_samples[1:]),default=0)),indent=2)+'\n')


if __name__=='__main__':sys.exit(main())
