#!/usr/bin/env python3
"""Real recorded LIO observations -> native SCAN, isolated differential diagnostic.

Not a PCT/end-to-end navigation success test. The reference is the subsequently
recorded body trajectory (an offline oracle), not an invented straight corridor.
The bag cannot react to a planned detour. No controller/SDK/cmd_vel is started.
One constant clock offset retains actual scan/odom intervals and measured twist.
"""
from array import array
from collections import Counter
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time

import numpy as np
import yaml
from capture_frontend import cloud_arrays, json_value, save_json, sha256

WS=Path('/home/dndx/d1max_nav_ws')
PORT=17470
FRAME='d1max_loc_odom'
PREFIX='/TEST_ONLY_real_bag/'
PROFILES=('strict6','strict2','strict2_v06','occupied2','occupied2_v06')


def records(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line]


def profile_overrides(profile):
    if profile not in PROFILES:raise ValueError('Unknown bounded experiment profile')
    horizon=6. if profile=='strict6' else 2.
    values={'grid_map.frame_id':FRAME,'fsm.planning_horizon':horizon,
            'manager.planning_horizon':horizon,'grid_map.require_observed_free':profile.startswith('strict')}
    if profile.endswith('_v06'):
        values.update({'manager.max_vel':.60,'optimization.max_vel':.60})
    return values


def prepare_data(capture):
    sys.path[:0]=[str(WS/'src/d1max_pct_scan'),str(WS/'src/d1max_localization')]
    from d1max_localization.estimation.contracts import MotionState, body_state
    from d1max_localization.math_utils import Pose3
    from d1max_localization.initial_pose import body_to_tracking_transform
    capture=capture.resolve()
    result=json.loads((capture/'result.json').read_text())
    if not result.get('passed') or result.get('owned_processes_surviving'):
        raise ValueError('Requires completed genuine frontend capture')
    config=yaml.safe_load((capture/'localization.yaml').read_text())
    front=config['lio_localizer']['ros__parameters']
    # Exactly the production reference-point AND axis conversion. Its rotation
    # is -sdk_to_tracking_yaw, not +yaw, and includes the angular lever arm.
    transform=body_to_tracking_transform(front['tracking_offset_body'],front['sdk_to_tracking_yaw'])
    samples=records(capture/'sample.jsonl')
    odometry=records(capture/'odometry.jsonl')
    states={int(s['stamp_ns']):s for s in records(capture/'local_sample.jsonl')
            if s.get('valid') and s.get('stamp_ns')}
    body=[]
    for odom in odometry:
        state=states.get(odom['stamp_ns'])
        if state is None or state['frame']!=FRAME: continue
        if not state.get('inertial',{}).get('world_velocity'):
            raise ValueError('Native inertial velocity missing; refusing synthetic zero velocity')
        motion=MotionState(odom['stamp_ns']*1e-9,
            Pose3(tuple(odom['position']),tuple(odom['orientation'])),
            tuple(state['inertial']['world_velocity']),tuple(odom['angular_child_frame']),
            odom['pose_covariance'],odom['twist_covariance'],odom['stamp_ns']*1e-9,odom['stamp_ns']*1e-9)
        pose,linear,angular,pcov,tcov=body_state(motion,transform)
        body.append(dict(stamp_ns=odom['stamp_ns'],position=pose.position,
                         orientation=pose.orientation,linear=linear,angular=angular,
                         pose_covariance=pcov,twist_covariance=tcov))
    if len(body)<10 or len(samples)<5: raise ValueError('Insufficient genuine capture')
    epochs={s['local_sample']['epoch'] for s in samples}
    if len(epochs)!=1: raise ValueError('Split epoch resets before comparing planner inputs')
    for values in (samples,body):
        if any(b['stamp_ns']<=a['stamp_ns'] for a,b in zip(values,values[1:])):
            raise ValueError('Input stamps must advance strictly; no sorting or retiming faulty source')
    from d1max_pct_scan.live_scan_contract import transform_xyz
    for item in samples:
        path=(capture/item['cloud_file']).resolve()
        if not path.is_relative_to(capture/'clouds') or sha256(path)!=item['cloud_sha256']:
            raise ValueError('Cloud source path/hash mismatch')
        if int(item['local_sample']['stamp_ns'])!=item['stamp_ns'] or item['odometry']['stamp_ns']!=item['stamp_ns']:
            raise ValueError('Cloud/odometry posterior not exact-time paired')
        with np.load(path,allow_pickle=False) as data:
            item['world_cloud']=transform_xyz(data['xyz'],item['odometry']['position'],item['odometry']['orientation'])
    return samples,body


def phase_time_summary(events, duration):
    """Duration-weighted diagnostic phases, not invented voxel unknown ratios."""
    totals=Counter(); previous=0.;phase='no_debug_yet'
    for event in events:
        at=min(duration,max(previous,float(event['elapsed_s'])))
        totals[phase]+=at-previous;previous=at;phase=event['phase']
    totals[phase]+=max(0.,duration-previous)
    return {'phase_seconds':dict(totals),
            'waiting_observed_space_fraction':totals['waiting_observed_space']/duration if duration else None,
            'unknown_voxel_fraction':None,
            'unknown_fraction_scope':'time-weighted waiting_observed_space diagnostic, NOT voxel-space coverage'}


def spline_measurement(curve):
    from scipy.interpolate import BSpline
    p=np.asarray(curve['points'],dtype=float);knots=np.asarray(curve['knots'],dtype=float);order=curve['order']
    if order!=3 or len(knots)!=len(p)+order+1 or len(p)<4:
        raise ValueError('Malformed native cubic B-spline')
    begin,end=knots[order],knots[-order-1]
    if not np.isfinite(p).all() or not np.isfinite(knots).all() or end<=begin:
        raise ValueError('Invalid native curve data')
    spline=BSpline(knots,p,order);sample=np.linspace(begin,end,min(50000,max(2,int((end-begin)/.01)+1)))
    xyz=spline(sample);v=spline.derivative(1)(sample);a=spline.derivative(2)(sample)
    return xyz,{'duration_s':float(end-begin),'start_position':xyz[0].tolist(),
                'end_position':xyz[-1].tolist(),'start_velocity_world':v[0].tolist(),
                'max_speed_mps':float(np.linalg.norm(v,axis=1).max()),
                'max_acceleration_mps2':float(np.linalg.norm(a,axis=1).max())}


def plot_comparison(out, samples, body, report):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    figure,axes=plt.subplots(1,len(report['profiles']),figsize=(6*len(report['profiles']),8),squeeze=False)
    reference=np.array([b['position'] for b in body]);cloud=np.concatenate([s['world_cloud'][::8] for s in samples])
    # Faint genuine point context, not a fabricated complete obstacle map.
    low,high=np.quantile(reference[:,2],[.01,.99]);view=cloud[(cloud[:,2]>low-.7)&(cloud[:,2]<high+.8)]
    for ax,(name,values) in zip(axes[0],report['profiles'].items()):
        ax.set_facecolor('#161c25');ax.scatter(view[:,0],view[:,1],s=.12,color='#5c7788',alpha=.2,rasterized=True)
        ax.plot(reference[:,0],reference[:,1],color='#489bff',lw=2,label='recorded future reference (oracle)')
        data=json.loads((out/(name+'_outputs.json')).read_text())
        for i,curve in enumerate(data['curves']):
            xyz,_=spline_measurement(curve)
            ax.plot(xyz[:,0],xyz[:,1],color='#45e691',lw=1.1,alpha=.8,label='accepted native spline' if i==0 else None)
        targets=np.array([x['target'] for x in data['targets'] if x['valid']])
        if len(targets):ax.scatter(targets[:,0],targets[:,1],s=9,color='#ffab40',label='native local targets')
        ax.scatter(reference[0,0],reference[0,1],s=50,color='white',edgecolor='black',label='capture start')
        ax.set_title(f"{name}: {values['tagged_count']} accepted\nunknown-wait {values['waiting_observed_space_fraction']:.0%}")
        ax.set_aspect('equal');ax.set_xlabel('local odom X [m]');ax.set_ylabel('local odom Y [m]')
        ax.legend(loc='upper right',fontsize=7)
    figure.suptitle('Real bag native SCAN diagnostic — recorded motion, no controller, no PCT claim')
    figure.tight_layout();figure.savefig(out/'comparison.png',dpi=160);plt.close(figure)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture',type=Path)
    parser.add_argument('--seconds',type=float,default=35.)
    parser.add_argument('--profiles',nargs='+',default=['strict6','strict2','occupied2'])
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if not 10<=args.seconds<=60 or any(p not in PROFILES for p in args.profiles):
        raise ValueError('Bounded known profiles only')
    args.output=args.output.resolve();args.capture=args.capture.resolve()
    if args.output.exists() or args.output.parent!=WS/'log/scan_bag_replay':
        raise ValueError('Use a new exact log/scan_bag_replay child')
    if os.environ.get('RMW_IMPLEMENTATION')!='rmw_zenoh_cpp': raise ValueError('Zenoh required')
    with socket.socket() as probe: probe.bind(('127.0.0.1',PORT))
    samples,body=prepare_data(args.capture)
    args.output.mkdir(parents=True)
    env=dict(os.environ)
    for key in ('ZENOH_CONFIG_OVERRIDE','ZENOH_SESSION_CONFIG','ZENOH_ROUTER_CONFIG'):
        if env.get(key):raise ValueError('Unexpected transport override '+key)
    env.pop('ROS_LOCALHOST_ONLY',None);os.environ.pop('ROS_LOCALHOST_ONLY',None)
    for kind in ('client','router'):
        config=dict(mode=kind,scouting={'multicast':{'enabled':False},'gossip':{'enabled':False}},
            connect={'endpoints':[f'tcp/127.0.0.1:{PORT}'] if kind=='client' else []},
            listen={'endpoints':[f'tcp/127.0.0.1:{PORT}'] if kind=='router' else []})
        (args.output/(kind+'.json5')).write_text(json.dumps(config))
    env.update(RMW_IMPLEMENTATION='rmw_zenoh_cpp',ROS_DOMAIN_ID='227',
        ZENOH_SESSION_CONFIG_URI=str(args.output/'client.json5'),
        ZENOH_ROUTER_CONFIG_URI=str(args.output/'router.json5'),ROS_LOG_DIR=str(args.output/'ros_logs'),
        OMP_NUM_THREADS='2',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1')
    os.environ.update(env)
    from d1max_pct_scan.live_session import DEFAULT_CONFIG,scan_parameters
    from d1max_pct_scan.live_scan_contract import transform_xyz
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from ament_index_python.packages import get_package_prefix
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2,PointField
    from std_msgs.msg import Bool
    from d1max_planning_interfaces.msg import ReferencePath,TaggedBspline,LocalPlanDebug
    cfg=yaml.safe_load(DEFAULT_CONFIG.read_text())
    scanexe=Path(get_package_prefix('scan_planner'))/'lib/scan_planner/scan_planner_node'
    children=[];streams=[];node=None
    def interrupt(*_): raise KeyboardInterrupt('offline replay interrupted')
    handlers={s:signal.signal(s,interrupt) for s in (signal.SIGINT,signal.SIGTERM)}
    report=dict(kind='REAL_BAG_NATIVE_DIFFERENTIAL_NO_MOTION',capture=str(args.capture),
        reference='future recorded LIO body trajectory; NOT PCT output or ground truth',
        observation='genuine dual deskewed LIO cloud; combined-origin limitation unchanged',
        acceleration='native FSM default zero (unmeasured), measured initial velocity retained',
        frame=FRAME,private_domain=227,private_port=PORT,robot_commands_sent=False,profiles={},
        input_hashes={str(args.capture/n):sha256(args.capture/n) for n in ('manifest.json','localization.yaml','sample.jsonl','odometry.jsonl','local_sample.jsonl')},
        harness_sha256=sha256(Path(__file__)),native_binary_sha256=sha256(scanexe),
        capture_samples=len(samples),capture_odometry=len(body),seconds_per_profile=args.seconds,
        native_initial_acceleration_policy='FSM zero assumed acceleration; no measured-acceleration inlet; not an IMU measurement',
        test_completed=False)
    def start(name,cmd):
        stream=(args.output/(name+'.log')).open('x');streams.append(stream)
        child=subprocess.Popen(cmd,env=env,cwd=args.output,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        children.append(child);return child
    def stop(child):
        if child.poll() is None:
            os.killpg(child.pid,signal.SIGINT)
            try: child.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid,signal.SIGKILL);child.wait(timeout=2)
    def stamp(target,ns): target.sec,target.nanosec=divmod(int(ns),1000000000)
    def pose(msg,record):
        msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z=map(float,record['position'])
        q=record['orientation'];msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w=map(float,q)

    class Probe(Node):
        def __init__(self,name):
            super().__init__('real_bag_scan_probe_'+name)
            self.name=name;self.prefix=PREFIX+name+'/';self.sid='REAL_BAG_'+name
            self.bodypub=self.create_publisher(Odometry,self.prefix+'body',qos_profile_sensor_data)
            self.sensorpub=self.create_publisher(Odometry,self.prefix+'sensor',qos_profile_sensor_data)
            self.cloudpub=self.create_publisher(PointCloud2,self.prefix+'cloud',qos_profile_sensor_data)
            self.freezepub=self.create_publisher(Bool,self.prefix+'frozen',1)
            self.refpub=self.create_publisher(ReferencePath,self.prefix+'reference',1)
            self.phases=Counter();self.outputs=[];self.targets=[]
            self.begin=time.monotonic();self.last_body=None;self.occupancy=[]
            self.create_subscription(LocalPlanDebug,self.prefix+'debug',self.debug,50)
            self.create_subscription(TaggedBspline,self.prefix+'tagged',self.tagged,50)
            self.create_subscription(PointCloud2,self.prefix+'scan/grid_map/occupancy',self.occupied,qos_profile_sensor_data)
        def debug(self,msg):
            self.phases[msg.phase]+=1
            self.targets.append(dict(phase=msg.phase,valid=msg.valid,progress=msg.progress_arc_m,
                target_arc=msg.target_arc_m,target=[msg.local_target.x,msg.local_target.y,msg.local_target.z],
                elapsed_s=time.monotonic()-self.begin))
        def tagged(self,msg):
            curve=dict(id=msg.trajectory.traj_id,knots=list(msg.trajectory.knots),
                order=msg.trajectory.order,points=[[p.x,p.y,p.z] for p in msg.trajectory.pos_pts],
                elapsed_s=time.monotonic()-self.begin,
                native_start_stamp_ns=int(msg.trajectory.start_time.sec)*10**9+int(msg.trajectory.start_time.nanosec),
                last_input_body=self.last_body)
            _,curve['measurement']=spline_measurement(curve)
            self.outputs.append(curve)
        def occupied(self,msg):
            self.occupancy.append({'elapsed_s':time.monotonic()-self.begin,'points':msg.width*msg.height})
        def body(self,record,offset):
            self.last_body=record
            msg=Odometry();msg.header.frame_id=FRAME;stamp(msg.header.stamp,record['stamp_ns']+offset)
            msg.child_frame_id='TEST_ONLY_body';pose(msg,record)
            msg.twist.twist.linear.x,msg.twist.twist.linear.y,msg.twist.twist.linear.z=map(float,record['linear'])
            msg.twist.twist.angular.x,msg.twist.twist.angular.y,msg.twist.twist.angular.z=map(float,record['angular'])
            msg.pose.covariance=list(record['pose_covariance']);msg.twist.covariance=list(record['twist_covariance'])
            self.bodypub.publish(msg)
        def cloud(self,sample,offset):
            odom=sample['odometry'];msg=Odometry();msg.header.frame_id=FRAME
            stamp(msg.header.stamp,sample['stamp_ns']+offset);msg.child_frame_id=sample['frame'];pose(msg,odom)
            points=sample['world_cloud']
            cloud=PointCloud2();cloud.header=msg.header;cloud.height=1;cloud.width=len(points)
            cloud.point_step=12;cloud.row_step=len(points)*12;cloud.is_dense=True
            cloud.fields=[PointField(name=n,offset=4*i,datatype=7,count=1) for i,n in enumerate('xyz')]
            cloud.data=array('B',points.tobytes());self.sensorpub.publish(msg);self.cloudpub.publish(cloud)
            return (self.get_clock().now().nanoseconds-(sample['stamp_ns']+offset))*1e-9
        def reference(self,elapsed,first_ns):
            route=[b for b in body if b['stamp_ns']>=first_ns+elapsed*1e9]
            points=[]
            for b in route:
                p=np.array(b['position'],dtype=float)
                if not points or np.linalg.norm(p-points[-1])>.12: points.append(p)
            if len(points)<2: return False
            msg=ReferencePath();msg.session_id=self.sid;msg.generation=1
            msg.path.header.frame_id=FRAME;msg.path.header.stamp=self.get_clock().now().to_msg()
            for p in points:
                item=PoseStamped();item.header=msg.path.header;item.pose.orientation.w=1.
                item.pose.position.x,item.pose.position.y,item.pose.position.z=map(float,p-np.array([0,0,cfg['body_height']]))
                msg.path.poses.append(item)
            self.refpub.publish(msg);return True

    try:
        router=start('router',[str(Path(get_package_prefix('rmw_zenoh_cpp'))/'lib/rmw_zenoh_cpp/rmw_zenohd')])
        time.sleep(.5)
        if router.poll() is not None:raise RuntimeError('Private router did not start')
        rclpy.init(args=[])
        for profile in args.profiles:
            node=Probe(profile);params=scan_parameters(cfg,node.sid)
            params.update(profile_overrides(profile));horizon=params['fsm.planning_horizon']
            path=args.output/(profile+'.yaml');path.write_text(yaml.safe_dump({'/**':{'ros__parameters':params}}))
            cmd=[str(scanexe),'--ros-args','-r','__ns:='+node.prefix+'scan','--params-file',str(path)]
            for source,target in {'body_pose':'body','sensor_pose':'sensor','cloud':'cloud',
                'typed_initial_path':'reference','planning/go2_execution_frozen':'frozen',
                'planning/tagged_bspline':'tagged','planning/local_plan_debug':'debug'}.items():
                cmd+=['-r',source+':='+node.prefix+target]
            child=start(profile,cmd);ready_until=time.monotonic()+8
            while not all(p.get_subscription_count() for p in (node.bodypub,node.sensorpub,node.cloudpub,node.refpub)):
                rclpy.spin_once(node,timeout_sec=.02)
                if child.poll() is not None or router.poll() is not None:raise RuntimeError('Native startup failed')
                if time.monotonic()>ready_until: raise TimeoutError('native subscriptions')
            first_ns=max(samples[0]['stamp_ns'],body[0]['stamp_ns'])
            offset=node.get_clock().now().nanoseconds-first_ns
            start_mono=time.monotonic();node.begin=start_mono;bi=ci=0;issued=False;max_lag=0.
            while (elapsed:=time.monotonic()-start_mono)<args.seconds:
                if child.poll() is not None or router.poll() is not None: raise RuntimeError('owned process exited')
                current=first_ns+int(elapsed*1e9)
                while bi<len(body) and body[bi]['stamp_ns']<=current:
                    if body[bi]['stamp_ns']>=first_ns: node.body(body[bi],offset)
                    bi+=1
                while ci<len(samples) and samples[ci]['stamp_ns']<=current:
                    if samples[ci]['stamp_ns']>=first_ns:
                        max_lag=max(max_lag,node.cloud(samples[ci],offset))
                    ci+=1
                node.freezepub.publish(Bool(data=True))
                if elapsed>=2 and not issued: issued=node.reference(elapsed,first_ns)
                rclpy.spin_once(node,timeout_sec=.01)
            forbidden=[t for t in ('/cmd_vel','/d1max/navigation/cmd_vel','/d1max/pct_scan/cmd_vel_safe')
                       if node.get_publishers_info_by_topic(t)]
            speed=np.array([np.linalg.norm(b['linear']) for b in body[:bi]])
            report['profiles'][profile]=dict(phases=dict(node.phases),tagged_count=len(node.outputs),
                reference_issued=issued,body_records=bi,cloud_records=ci,max_delivery_lag_s=max_lag,
                original_first_stamp_ns=first_ns,constant_clock_offset_ns=offset,
                horizon_m=horizon,require_observed_free=params['grid_map.require_observed_free'],
                body_speed_max_mps=float(speed.max()),body_speed_p95_mps=float(np.quantile(speed,.95)),
                configured_max_velocity_mps=params['manager.max_vel'],
                initial_velocity_over_configured_limit_fraction=float((speed>params['manager.max_vel']).mean()),
                accepted_max_velocity_mps=max([x['measurement']['max_speed_mps'] for x in node.outputs],default=None),
                accepted_max_acceleration_mps2=max([x['measurement']['max_acceleration_mps2'] for x in node.outputs],default=None),
                occupied_voxel_count_max=max([x['points'] for x in node.occupancy],default=0),
                **phase_time_summary(node.targets,args.seconds),
                forbidden_publishers=forbidden,execution_authorized=False)
            save_json(args.output/(profile+'_outputs.json'),dict(curves=node.outputs,targets=node.targets,occupancy=node.occupancy))
            print(profile+': '+json.dumps(report['profiles'][profile]),flush=True)
            stop(child);node.destroy_node();node=None
        report['test_completed']=True
        plot_comparison(args.output,samples,body,report)
    except BaseException as error:
        report['error']=f'{type(error).__name__}: {error}'
        raise
    finally:
        for child in reversed(children): stop(child)
        if node: node.destroy_node()
        if rclpy.ok(): rclpy.try_shutdown()
        for stream in streams: stream.close()
        report['owned_processes_surviving']=[p.pid for p in children if p.poll() is None]
        with socket.socket() as probe:report['private_port_released']=probe.connect_ex(('127.0.0.1',PORT))!=0
        report['exit_codes']=[p.returncode for p in children]
        save_json(args.output/'report.json',report)
        for sig,handler in handlers.items():signal.signal(sig,handler)
    return 0


if __name__=='__main__': raise SystemExit(main())
