"""Isolated whole-chain smoke test. Synthetic sensors only; no robot connection.

Run with D1 ROS environment + nav overlay sourced. Owns and cleans its subprocess
groups, uses a loopback-only Zenoh router and domain 214. Leaves report artifacts.
"""
import json
import os
from pathlib import Path
import signal
import re
import subprocess
import tempfile
import time
import uuid
from array import array as byte_array
import numpy as np
import yaml

def run():
    root=Path(tempfile.mkdtemp(prefix='d1max-localization-smoke-'))
    print('ARTIFACTS='+str(root),flush=True)
    package=Path(__file__).resolve().parents[1]
    router={'mode':'router','connect':{'endpoints':[]},'listen':{'endpoints':['tcp/127.0.0.1:17447'],'exit_on_failure':True},'scouting':{'multicast':{'enabled':False},'gossip':{'enabled':False}}}
    session={'mode':'client','connect':{'endpoints':['tcp/127.0.0.1:17447'],'exit_on_failure':True},'scouting':{'multicast':{'enabled':False},'gossip':{'enabled':False}}}
    (root/'router.json5').write_text(json.dumps(router));(root/'client.json5').write_text(json.dumps(session))
    os.environ.update(RMW_IMPLEMENTATION='rmw_zenoh_cpp',ROS_DOMAIN_ID='214',ZENOH_ROUTER_CONFIG_URI=str(root/'router.json5'),ZENOH_SESSION_CONFIG_URI=str(root/'client.json5'),ROS_LOG_DIR=str(root/'roslogs'),RUST_LOG='warn')
    os.environ.pop('ROS_LOCALHOST_ONLY',None)
    children=[];streams=[]
    def start(command,name):
        stream=(root/(name+'.log')).open('w');streams.append(stream)
        child=subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True);children.append(child);return child
    report={}
    sensor_epoch_offset=float(os.environ.get('D1MAX_QA_SENSOR_OFFSET_SEC','0'))
    report['synthetic_sensor_epoch_offset_sec']=sensor_epoch_offset
    try:
        router_child=start(['ros2','run','rmw_zenoh_cpp','rmw_zenohd'],'router');time.sleep(.5)
        assert router_child.poll() is None,'QA router failed; inspect port conflict'
        tests=subprocess.run(['/usr/bin/python3','-m','pytest',str(package/'test'),'--ignore='+str(package/'test/smoke_localization.py'),'-q'],capture_output=True,text=True,timeout=45)
        (root/'pytest.log').write_text(tests.stdout+tests.stderr);assert tests.returncode==0,tests.stdout+tests.stderr
        print(tests.stdout,flush=True)
        # Asymmetric room with walls, floor, beam and column. Truth is a nonzero pose.
        x=np.arange(-6,6,.12);y=np.arange(-4,4,.12);z=np.arange(-1,2.5,.12)
        def surface(a,b,fixed,axis):
            aa,bb=np.meshgrid(a,b);out=np.empty((aa.size,3));out[:,axis]=fixed;other=[i for i in range(3) if i!=axis];out[:,other[0]]=aa.ravel();out[:,other[1]]=bb.ravel();return out
        points=np.concatenate([surface(x,y,-1,2),surface(x,z,-4,1),surface(x,z,4,1),surface(y,z,-6,0),surface(y,z,6,0),surface(np.arange(-1,1,.1),z,2,0),surface(np.arange(-2,1,.1),np.arange(.5,1.5,.1),1,1)]).astype(np.float32)
        source=os.environ.get('D1MAX_QA_MAP_PCD')
        if source:
            # Optional read-only binary XYZI map fixture; scans remain SYNTHETIC.
            with Path(source).open('rb') as stream:
                header=[]
                for _ in range(30):
                    line=stream.readline().decode('ascii').strip();header.append(line)
                    if line.startswith('DATA'):break
                assert 'DATA binary' in header and 'FIELDS x y z intensity' in header and 'SIZE 4 4 4 4' in header,header
                points=np.frombuffer(stream.read(),dtype='<f4').reshape(-1,4)[:,:3].copy()
            report['map_fixture']=source
        n=len(points);cloud=np.column_stack([points,np.ones(n,dtype=np.float32)*60]).astype('<f4')
        with (root/'map.pcd').open('wb') as stream:
            stream.write(('VERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH '+str(n)+'\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS '+str(n)+'\nDATA binary\n').encode());stream.write(cloud.tobytes())
        identifier=uuid.uuid4().hex
        (root/'session.json').write_text(json.dumps({'id':identifier,'version_id':'synthetic-room','map_pcd':str(root/'map.pcd')}))
        config=yaml.safe_load((package/'config/localization.yaml').read_text())
        backend=os.environ.get('D1MAX_QA_BACKEND','legacy_ekf')
        config['localization_pipeline']['ros__parameters']['backend']=backend
        moving=os.environ.get('D1MAX_QA_MOTION')=='1'
        assert not moving or backend=='lio_pcd','Motion fixture targets LIO, not MC-integrating legacy'
        report['backend']=backend;report['motion_fixture']=moving
        (root/'localization.yaml').write_text(yaml.safe_dump(config))
        launch=start(['ros2','launch','d1max_localization','localization.launch.py','config:='+str(root/'localization.yaml'),'session_dir:='+str(root),'map_pcd:='+str(root/'map.pcd')],'localization')
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import PointCloud2,PointField,Imu
        from std_msgs.msg import String
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Odometry
        from tf2_msgs.msg import TFMessage
        from scipy.spatial.transform import Rotation
        rclpy.init();node=rclpy.create_node('isolated_localization_fixture')
        front=node.create_publisher(PointCloud2,'/front_lidar',qos_profile_sensor_data);rear=node.create_publisher(PointCloud2,'/rear_lidar',qos_profile_sensor_data)
        imu_pub=node.create_publisher(Imu,'/front_lidar/imu',qos_profile_sensor_data)
        state_pub=node.create_publisher(String,'/d1max_sdk_bridge/robot_state',10);speed_pub=node.create_publisher(String,'/d1max_sdk_bridge/velocity',10)
        seen_tf=[];seen_pose=[];seen_preview=[];seen_local=[];seen_global=[];seen_navigation=[];dynamic_tf=[]
        def on_tf(message):
            seen_tf.extend(t for t in message.transforms if t.header.frame_id=='d1max_loc_map')
            dynamic_tf.extend(message.transforms)
        subscriptions=[node.create_subscription(TFMessage,'/tf',on_tf,100),node.create_subscription(PoseStamped,'/d1max/localization/pose',lambda m:seen_pose.append(m),20)]
        subscriptions.append(node.create_subscription(PointCloud2,'/d1max/localization/scan_initial_preview',lambda m:seen_preview.append((m.width,m.header.frame_id)),qos_profile_sensor_data))
        high_rate=backend=='lio_pcd' and config.get('navigation_estimation',{}).get('ros__parameters',{}).get('enabled',False)
        if high_rate:
            subscriptions.extend([
                node.create_subscription(Odometry,'/d1max/localization/odometry/local',lambda m:seen_local.append(m),100),
                node.create_subscription(Odometry,'/d1max/localization/odometry/global',lambda m:seen_global.append(m),100),
                node.create_subscription(String,'/d1max/localization/navigation/status',lambda m:seen_navigation.append(json.loads(m.data)),20)])
        truth=np.array([1.,-1.,.4])
        truth_rotation=Rotation.from_euler('xyz',[7.,-5.,20.],degrees=True)
        tracking=(points-truth)@truth_rotation.as_matrix()
        report['truth_rpy_deg']=[7.,-5.,20.]
        gravity=truth_rotation.inv().apply([0.,0.,1.])
        # Inverse of adapter's (x,y,z)->(x,-z,y), input acceleration in g.
        raw_gravity=[gravity[0],gravity[2],-gravity[1]]
        if source:
            tracking=tracking[(np.linalg.norm(tracking,axis=1)<12)&(np.linalg.norm(tracking,axis=1)>.6)]
            tracking=tracking[::max(1,len(tracking)//18000)]
            assert len(tracking)>1000,'Not enough local geometry for this fixture truth pose'
        report['synthetic_scan_points']=len(tracking);report['map_points']=n
        p=config['dual_lidar_adapter']['ros__parameters'];rf=Rotation.from_quat(p['front_rotation']).as_matrix();rr=Rotation.from_quat(p['rear_to_front_rotation']).as_matrix();tr=np.array(p['rear_to_front_translation'])
        raw_front=tracking[::2]@rf;raw_rear=(tracking[1::2]@rf-tr)@rr
        motion_start=None
        frequency=5./3.;period=2*np.pi/frequency
        tilt=Rotation.from_euler('xyz',[7.,-5.,0.],degrees=True).as_matrix()
        def motion_at(times):
            times=np.atleast_1d(times)
            elapsed=times-motion_start if motion_start is not None else np.zeros_like(times)
            active=(elapsed>=0)&(elapsed<=period)&(motion_start is not None)
            phase=frequency*np.clip(elapsed,0,period)
            bend=np.where(active,1-np.cos(phase),0.)
            rate=np.where(active,frequency*np.sin(phase),0.)
            acc=np.where(active,frequency**2*np.cos(phase),0.)
            positions=truth+np.column_stack([1.2*bend,.3*bend,0*bend])
            accelerations=np.column_stack([1.2*acc,.3*acc,0*acc])
            yaw=np.deg2rad(20)+.6*bend
            rotations=Rotation.from_euler('z',yaw).as_matrix()@tilt
            angular=np.outer(.6*rate,tilt.T@np.array([0.,0.,1.]))
            return positions,rotations,accelerations,angular
        def rolling_raw(world,now,rear_sensor=False):
            times=now-.1+np.linspace(0,.099,len(world))
            pos,rot,_,_=motion_at(times)
            local=np.einsum('ni,nij->nj',world-pos,rot)
            sensor=local@rf
            return (sensor-tr)@rr if rear_sensor else sensor
        def message(raw,frame,now):
            m=PointCloud2();m.header.frame_id=frame;m.header.stamp=rclpy.time.Time(seconds=now-.1).to_msg();m.height=1;m.width=len(raw);m.is_dense=True;m.point_step=26;m.row_step=m.width*26
            m.fields=[PointField(name=name,offset=offset,datatype=datatype,count=1) for name,offset,datatype in [('x',0,7),('y',4,7),('z',8,7),('intensity',12,7),('ring',16,4),('timestamp',18,8)]]
            array=np.empty(len(raw),dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('intensity','<f4'),('ring','<u2'),('timestamp','<f8')])
            for i,key in enumerate(('x','y','z')):array[key]=raw[:,i]
            array['intensity']=60;array['ring']=np.arange(len(raw))%96;array['timestamp']=now-.1+np.linspace(0,.099,len(raw));m.data=byte_array('B',array.tobytes());return m
        last={'imu':0.,'sdk':0.,'cloud':0.,'state':0.}
        def pump(duration,publish=True,imu_enabled=True,rear_enabled=True):
            end=time.monotonic()+duration
            while time.monotonic()<end:
                assert launch.poll() is None,'Localization exited: '+(root/'localization.log').read_text()[-5000:]
                now=time.time()
                if publish:
                    if now-last['state']>=1:
                        m=String();m.data=json.dumps({'received_at_unix':now,'forward_speed':0.,'lateral_speed':0.,'yaw_speed':0.,'head_direction':1});state_pub.publish(m);last['state']=now
                    if now-last['sdk']>=.02:
                        m=String();m.data=json.dumps({'source':'sdk_mc','frame':'sdk_body','received_at_unix':now,'stamp_unix':now,
                            'source_timestamp_ns':str(int(time.monotonic()*1e9)),'session':'synthetic-mc','generation':0,
                            'clock_mode':'source_delta_host_anchor','v_body':[.035 if moving else 0.,0.,0.],'omega_body':[0.,0.,0.],
                            'forward_speed':.035 if moving else 0.,'lateral_speed':0.,'yaw_speed':0.});speed_pub.publish(m);last['sdk']=now
                    if imu_enabled and now-last['imu']>=.005:
                        m=Imu();m.header.frame_id='rslidar_head_imu';m.header.stamp=rclpy.time.Time(seconds=now-sensor_epoch_offset).to_msg()
                        m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z=raw_gravity
                        if moving:
                            _,rot,acc,omega=motion_at(now)
                            specific=rot[0].T@(acc[0]+np.array([0.,0.,9.81]))/9.81
                            m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z=map(float,(specific[0],specific[2],-specific[1]))
                            m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z=map(float,(omega[0,0],omega[0,2],-omega[0,1]))
                        imu_pub.publish(m);last['imu']=now
                    if now-last['cloud']>=.1:
                        f=rolling_raw(points[::2],now) if moving else raw_front
                        r=rolling_raw(points[1::2],now,True) if moving else raw_rear
                        front.publish(message(f,'rslidar_head',now-sensor_epoch_offset))
                        if rear_enabled:rear.publish(message(r,'rslidar_tail',now-sensor_epoch_offset))
                        last['cloud']=now
                rclpy.spin_once(node,timeout_sec=.001)
        pump(7)
        health=json.loads((root/'status.json').read_text());report['before_seed']=health
        assert health['state']=='waiting_initial_pose',health
        if sensor_epoch_offset:
            assert abs(float(health['input_clock']['offset_seconds'])-sensor_epoch_offset)<.1,health
        assert not seen_tf and not seen_pose,'Global TF/pose must not be exposed before a verified seed'
        assert not seen_preview,'No guessed preview before an explicit seed'
        # Give a BODY seed just like the Web; derive it from the fixture tracking
        # seed using the inverse configured extrinsic, not a hardcoded offset.
        from d1max_localization.initial_pose import body_to_tracking_transform
        from d1max_localization.math_utils import Pose3,compose,inverse
        supervisor=config['localization_supervisor']['ros__parameters']
        extrinsic=body_to_tracking_transform(supervisor['tracking_offset_body'],supervisor['sdk_to_tracking_yaw'])
        body_seed=compose(Pose3((1.2,-1.1,.45),(0.,0.,np.sin(.02),np.cos(.02))),inverse(extrinsic))
        body_yaw=Rotation.from_quat(body_seed.orientation).as_euler('xyz')[2]
        command={'id':uuid.uuid4().hex,'session_id':identifier,'created_at':time.time(),'reference':'body','x':body_seed.position[0],'y':body_seed.position[1],'z':body_seed.position[2],'yaw':body_yaw}
        (root/'initial_pose.json').write_text(json.dumps(command))
        for _ in range(25):
            pump(1);health=json.loads((root/'status.json').read_text())
            if health['localized'] and seen_pose:break
        report['after_seed']=health
        assert health['localized'] and seen_pose and seen_tf,(health,(root/'localization.log').read_text()[-5000:])
        pose=seen_pose[-1].pose.position;error=float(np.linalg.norm(np.array([pose.x,pose.y,pose.z])-truth));report['position_error_m']=error
        assert error<.20,error
        q=seen_pose[-1].pose.orientation
        attitude_error=float((truth_rotation.inv()*Rotation.from_quat([q.x,q.y,q.z,q.w])).magnitude()*180/np.pi)
        report['attitude_error_deg']=attitude_error
        assert attitude_error<2.,attitude_error
        assert any(width>0 and frame=='d1max_loc_map' for width,frame in seen_preview),seen_preview
        assert any(width==0 for width,_ in seen_preview),'Initial preview not cleared on lock'
        assert int(health['matcher']['attempts'])>=3,health
        if moving:
            before=len(seen_pose);pump(3)
            static_positions=np.array([[m.pose.position.x,m.pose.position.y,m.pose.position.z] for m in seen_pose[before:]])
            assert len(static_positions)>10
            report['static_span_with_mc_bias_m']=float(np.linalg.norm(np.ptp(static_positions,axis=0)))
            assert report['static_span_with_mc_bias_m']<.08,report
            motion_start=time.time()+.2;before=len(seen_pose);pump(period+1.)
            motion_poses=seen_pose[before:]
            assert len(motion_poses)>int(period*6),'Too few verified moving poses'
            times=np.array([m.header.stamp.sec+m.header.stamp.nanosec*1e-9 for m in motion_poses])
            truth_p,truth_r,_,_=motion_at(times)
            observed=np.array([[m.pose.position.x,m.pose.position.y,m.pose.position.z] for m in motion_poses])
            errors=np.linalg.norm(observed-truth_p,axis=1)
            angles=[]
            for m,r in zip(motion_poses,truth_r):
                q=m.pose.orientation;angles.append((Rotation.from_matrix(r).inv()*Rotation.from_quat([q.x,q.y,q.z,q.w])).magnitude())
            report['synthetic_peak_speed_mps']=float(frequency*np.hypot(1.2,.3))
            report['synthetic_peak_yaw_rate_radps']=.6*frequency
            report['motion_max_error_m']=float(max(errors));report['motion_rms_error_m']=float(np.sqrt(np.mean(errors**2)))
            report['motion_max_angle_deg']=float(np.rad2deg(max(angles)));report['motion_output_max_gap_sec']=float(max(np.diff(times)))
            health=json.loads((root/'status.json').read_text());report['after_motion']=health
            assert health['localized'],health
            assert max(errors)<.25 and max(angles)<np.deg2rad(3),report
            assert max(np.diff(times))<.4,report
            if high_rate:
                report['motion_output_observed_hz']=float((len(times)-1)/(times[-1]-times[0]))
                assert 40.<report['motion_output_observed_hz']<60.,report
                assert max(np.diff(times))<.10,report
                assert health['navigation']['valid'] and health['output_observed_hz']>40.,health
                assert not health['navigation_ready'],'Unverified calibration must not enable navigation'
                assert {m.child_frame_id for m in seen_global}=={'d1max_loc_base_link'}
                local_stamps={(m.header.stamp.sec,m.header.stamp.nanosec) for m in seen_local}
                assert all((m.header.stamp.sec,m.header.stamp.nanosec) in local_stamps for m in seen_global), 'TF/odometry timestamps not aligned'
                tf_publishers=node.get_publishers_info_by_topic('/tf')
                # Humble rclpy has no subscription MessageInfo callback. Check
                # actual edges and duplicate timestamps, plus disabled writers.
                assert 'navigation_output' in {p.node_name for p in tf_publishers}
                assert config['ekf_navigation']['ros__parameters']['publish_tf'] is False
                assert config['/d1max/localization/lio/laserMapping']['ros__parameters']['publish.tf_enabled'] is False
                edges={(t.header.frame_id,t.child_frame_id) for t in dynamic_tf}
                assert edges=={('d1max_loc_map','d1max_loc_odom'),('d1max_loc_odom','d1max_loc_base_link')},edges
                tf_keys={(t.header.frame_id,t.child_frame_id,t.header.stamp.sec,t.header.stamp.nanosec) for t in dynamic_tf}
                assert len(tf_keys)==len(dynamic_tf),'Duplicate public TF samples'
                report['single_dynamic_tf_authority']=True
                report['body_reference_and_common_timestamps']=True
            # Pause only the matcher child started by this isolated fixture.
            match=re.search(r'\[fused_icp_matcher-\d+\]: process started with pid \[(\d+)\]',(root/'localization.log').read_text())
            assert match,'Cannot resolve owned matcher child'
            matcher_pid=int(match.group(1))
            os.kill(matcher_pid,signal.SIGSTOP)
            try:
                pump(1.8);lost=json.loads((root/'status.json').read_text());report['matcher_paused']=lost
                assert not lost['localized'] and lost['state']=='relocalizing',lost
                previous=len(seen_pose);pump(.4);assert len(seen_pose)==previous
            finally:os.kill(matcher_pid,signal.SIGCONT)
            for _ in range(6):
                pump(.5);recovered=json.loads((root/'status.json').read_text())
                if recovered['localized']:break
            report['matcher_resumed']=recovered
            assert recovered['localized'],recovered
            assert recovered['active_seed_ns']==recovered['confirmed_seed_ns'] and recovered['verified_confirmations']>=3,recovered
            before=len(seen_pose);pump(1.2,rear_enabled=False)
            front_health=json.loads((root/'status.json').read_text());report['rear_missing']=front_health
            assert front_health['localized'] and len(seen_pose)-before>=6,front_health
            assert int(front_health['input_clock']['front_only_attempts'])>=4,front_health
            pump(.5)
            previous_epoch=front_health['local_epoch']
            pump(.12,imu_enabled=False);pump(1.)
            gap_health=json.loads((root/'status.json').read_text());report['hard_imu_gap']=gap_health
            assert not gap_health['localized'] and gap_health['frontend']['reset_count']>=1,gap_health
            previous=len(seen_pose)
            for _ in range(12):
                pump(.5);gap_recovered=json.loads((root/'status.json').read_text())
                if gap_recovered['initial_pose_ready']:break
            report['imu_reinitialized']=gap_recovered
            assert gap_recovered['initial_pose_ready'] and gap_recovered['local_epoch']>previous_epoch,gap_recovered
            assert not gap_recovered['localized'] and gap_recovered['active_seed_ns'] is None,gap_recovered
            assert len(seen_pose)==previous,'Old map seed survived local epoch reset'
            command.update(id=uuid.uuid4().hex,created_at=time.time())
            (root/'initial_pose.json').write_text(json.dumps(command))
            for _ in range(12):
                pump(.5);reseeded=json.loads((root/'status.json').read_text())
                if reseeded['localized']:break
            report['imu_reseeded']=reseeded
            assert reseeded['localized'] and reseeded['verified_confirmations']>=3,reseeded
        report['preview_is_separate_map_cloud']=True
        topics=[name for name,_ in node.get_topic_names_and_types()]
        assert not any('cmd_vel' in name or 'navigate_to_pose' in name for name in topics),topics
        pump(2,publish=False);health=json.loads((root/'status.json').read_text());report['stale']=health
        assert not health['localized'],health
        previous=(len(seen_pose),len(seen_tf));pump(1,publish=False);assert previous==(len(seen_pose),len(seen_tf)),'Verified output continues on stale sensors'
        if high_rate:
            assert seen_navigation and not seen_navigation[-1]['valid'] and not seen_navigation[-1]['navigation_ready']
            assert seen_navigation[-1]['global_observed_hz']==0.,seen_navigation[-1]
        node.destroy_node();rclpy.shutdown();report['passed']=True
    finally:
        for child in reversed(children):
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
                try:child.wait(timeout=12)
                except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait(timeout=3)
        for stream in streams:stream.close()
        (root/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
        print(json.dumps({'artifact_dir':str(root),'passed':report.get('passed',False),'position_error_m':report.get('position_error_m')},ensure_ascii=False),flush=True)

if __name__=='__main__':run()
