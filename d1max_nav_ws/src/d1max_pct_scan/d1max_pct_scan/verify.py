"""Bounded, reproducible offline integration acceptance, never robot movement."""
import argparse
import json
import math
import os
from pathlib import Path
import time
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Empty
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path as RosPath
from std_srvs.srv import SetBool
from d1max_planning_interfaces.msg import TaggedBspline
from .simulation import check_isolation


class Acceptance(Node):
    def __init__(self,session):
        super().__init__('pct_scan_offline_acceptance');self.session=session;self.data={};self.history=[];self.events=[];self.path=None
        self.received={};self.bsplines=0;self.validated=0;self.max_speed=0.;self.max_yaw=0.;self.commands=0
        self.goal_pub=self.create_publisher(PoseStamped,'/d1max/pct_scan/goal',1)
        self.cancel_pub=self.create_publisher(Empty,'/d1max/pct_scan/cancel',1)
        for name,topic in [('global','global_status'),('tracker','tracker_status'),('sim','simulation_status'),('gate','gate_status'),('guard','trajectory_guard_status')]:
            self.create_subscription(String,'/d1max/pct_scan/'+topic,lambda m,k=name:self.status(k,m),10)
        self.create_subscription(RosPath,'/d1max/pct_scan/global_path',self.path_cb,10)
        self.create_subscription(TaggedBspline,'/d1max/pct_scan/s_'+session['session_id']+'/scan/planning/tagged_bspline',self.spline,10)
        self.create_subscription(TaggedBspline,'/d1max/pct_scan/validated_bspline',self.valid_spline,10)
        self.create_subscription(Twist,'/d1max/pct_scan/cmd_vel_safe',self.command,10)
        self.sensors=self.create_client(SetBool,'/d1max/pct_scan/simulation/sensors')
        self.obstacle=self.create_client(SetBool,'/d1max/pct_scan/simulation/obstacle')

    def status(self,key,msg):
        obj=json.loads(msg.data);self.data[key]=obj;self.received[key]=time.monotonic()
        if key=='guard':self.events.append(obj)
        if key=='sim':
            if obj.get('session_id')!=self.session['session_id'] or obj.get('mode')!='PCD_SOFTWARE_SIMULATION' or obj.get('real_motion_enabled'):
                raise RuntimeError('Wrong session or not isolated software simulation')
            self.history.append(obj)

    def path_cb(self,msg):
        if msg.poses:self.path=msg

    def spline(self,_):self.bsplines+=1
    def valid_spline(self,_):self.validated+=1

    def command(self,msg):
        self.max_speed=max(self.max_speed,math.hypot(msg.linear.x,msg.linear.y));self.max_yaw=max(self.max_yaw,abs(msg.angular.z));self.commands+=1
        if self.max_speed>.30001 or self.max_yaw>.50001:raise RuntimeError('Offline speed envelope exceeded')

    def wait(self,predicate,timeout,label):
        end=time.monotonic()+timeout;progress=0.
        while time.monotonic()<end:
            rclpy.spin_once(self,timeout_sec=.05)
            if predicate():return
            if label=='reach route goal' and self.data.get('global',{}).get('state') in ('failed','blocked','canceled','waiting_odometry'):
                raise RuntimeError('Route stopped before arrival: '+json.dumps(self.data.get('guard',{}),ensure_ascii=False))
            if time.monotonic()>progress:
                progress=time.monotonic()+10
                print(json.dumps({'waiting':label,'global':self.data.get('global',{}).get('state'),
                    'tracker':self.data.get('tracker',{}).get('reason'),'pose':self.data.get('sim',{}).get('pose'),
                    'splines':self.bsplines,'validated':self.validated,'guard':self.data.get('guard',{}).get('reason')},ensure_ascii=False),flush=True)
        raise RuntimeError('Timeout: '+label+'; status='+json.dumps(self.data,ensure_ascii=False))

    def spin_for(self,duration):
        end=time.monotonic()+duration
        while time.monotonic()<end:rclpy.spin_once(self,timeout_sec=.05)

    def goal(self,xy):
        msg=PoseStamped();msg.header.frame_id=self.session['pct_parameters']['planning_frame'];msg.header.stamp=self.get_clock().now().to_msg()
        msg.pose.position.x=float(xy[0]);msg.pose.position.y=float(xy[1]);msg.pose.orientation.w=1.
        self.goal_pub.publish(msg)

    def toggle(self,client,value):
        if not client.wait_for_service(timeout_sec=2.):raise RuntimeError('Simulation service unavailable')
        future=client.call_async(SetBool.Request(data=value))
        self.wait(future.done,3.,'simulation service')
        if not future.result().success:raise RuntimeError(future.result().message)

    def stopped(self):
        sim=self.data.get('sim',{});vel=sim.get('velocity',[1.,1.])
        return len(vel)==2 and max(abs(x) for x in vel)<1e-5

    def no_drift(self,duration):
        initial=self.data['sim']['pose'][:2];self.spin_for(duration)
        drift=math.dist(initial,self.data['sim']['pose'][:2])
        if drift>.01 or not self.stopped():raise RuntimeError('Motion resumed or stop drift '+str(drift))
        return drift


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--session',type=Path,default=Path('/home/dndx/d1max_nav_ws/log/pct_scan/last_session.json'))
    ap.add_argument('--case',choices=['smoke','full','observe'],default='full')
    ap.add_argument('--goal',type=float,nargs=2,help='Offline goal override, X Y in the configured map frame')
    ap.add_argument('--arrival-timeout',type=float,default=100.)
    args=ap.parse_args()
    session=json.loads(args.session.read_text());cfg=yaml.safe_load(Path(session['config']).read_text())
    if args.goal:cfg['test_goal_xy']=args.goal
    if session.get('mode')!='sim' or session.get('real_motion_enabled'):raise RuntimeError('Offline session required')
    os.environ.update(RMW_IMPLEMENTATION='rmw_zenoh_cpp',ROS_DOMAIN_ID='24',ZENOH_SESSION_CONFIG_URI=str(Path(session['directory'])/'client.json5'))
    for key in ('ZENOH_CONFIG_OVERRIDE','ZENOH_SESSION_CONFIG'):os.environ.pop(key,None)
    check_isolation();rclpy.init();node=Acceptance(session)
    report={'session_id':session['session_id'],'kind':'PCD_SOFTWARE_SIMULATION','case':args.case,'goal_xy':cfg['test_goal_xy'],'passed':False,'tests':{},
            'limitations':['Ideal odometry and static cropped PCD observations; not real localization or occlusion simulation','Does not prove robot safety or whole-map connectivity']}
    start=time.monotonic()
    try:
        node.wait(lambda:all(k in node.data for k in ('global','sim','tracker','gate')) and node.data['global'].get('ready'),35.,'ready')
        report['tests']['isolated_ready']=True
        if args.case=='observe':node.spin_for(5.)
        else:
            original_count=node.validated
            node.goal(cfg['test_goal_xy'])
            node.wait(lambda:node.path is not None and node.validated>original_count and node.data['tracker'].get('accepted_trajectories',0)>0,30.,'actual PCT path and SCAN spline accepted')
            report['tests']['native_pct_and_scan']=True
            node.wait(lambda:node.data.get('global',{}).get('state')=='arrived' and node.stopped(),args.arrival_timeout,'reach route goal')
            error=math.dist(node.data['sim']['pose'][:2],cfg['test_goal_xy'])
            if error>.26:raise RuntimeError('Arrival error '+str(error))
            if node.data['sim']['collision_stops']:raise RuntimeError('Simulator collision clamp hid a planner/controller failure')
            report['tests']['arrive']={'error_m':error,'distance_m':node.data['sim']['distance_m'],'simulator_collision_clamps':0}
            report['tests']['arrival_holds_m']=node.no_drift(1.)
            if args.case=='full':
                origin=cfg['initial_xy'];base_distance=node.data['sim']['distance_m'];node.goal(origin)
                node.wait(lambda:node.data['sim']['distance_m']>base_distance+.1,30.,'move before cancel')
                node.cancel_pub.publish(Empty());node.wait(lambda:node.data['global'].get('state')=='canceled' and node.stopped(),2.,'cancel stop')
                report['tests']['cancel_holds_m']=node.no_drift(1.5)
                base_distance=node.data['sim']['distance_m'];node.goal(origin)
                node.wait(lambda:node.data['sim']['distance_m']>base_distance+.1,30.,'move before sensor drop')
                node.toggle(node.sensors,False);drop=time.monotonic();node.wait(node.stopped,1.5,'sensor drop stop')
                report['tests']['sensor_drop']={'stop_latency_s':time.monotonic()-drop,'hold_drift_m':node.no_drift(1.)}
                node.toggle(node.sensors,True);report['tests']['no_auto_resume_m']=node.no_drift(1.)
                base_distance=node.data['sim']['distance_m'];node.goal(origin)
                node.wait(lambda:node.data['sim']['distance_m']>base_distance+.1,30.,'move before obstacle')
                node.toggle(node.obstacle,True);created=time.monotonic();node.wait(node.stopped,1.5,'obstacle stop')
                report['tests']['obstacle_stop']={'latency_s':time.monotonic()-created,'hold_drift_m':node.no_drift(.6)}
                if node.data['sim']['collision_stops']:raise RuntimeError('Obstacle test relied on simulator collision clamp')
                node.cancel_pub.publish(Empty());node.toggle(node.obstacle,False)
        report['passed']=True
    except (Exception,KeyboardInterrupt) as exc:
        report['error']=str(exc);print('FAILED '+str(exc),flush=True)
    finally:
        if rclpy.ok():node.cancel_pub.publish(Empty());node.spin_for(.3)
        for client,value in [(node.sensors,True),(node.obstacle,False)]:
            try:node.toggle(client,value)
            except Exception:pass
        path_points=[[p.pose.position.x,p.pose.position.y,p.pose.position.z] for p in node.path.poses] if node.path else []
        report.update(elapsed_s=time.monotonic()-start,max_safe_speed_mps=node.max_speed,max_safe_yaw_rps=node.max_yaw,global_path=path_points,
                      raw_splines=node.bsplines,validated_splines=node.validated,last_status=node.data,samples=node.history,guard_events=node.events)
        if len(path_points)>1 and node.history:
            a=np.asarray(path_points)[:-1,:2];delta=np.diff(np.asarray(path_points)[:,:2],axis=0)
            errors=[]
            for sample in node.history:
                p=np.asarray(sample['pose'][:2]);t=np.clip(np.sum((p-a)*delta,axis=1)/np.maximum(np.sum(delta*delta,axis=1),1e-12),0,1)
                errors.append(float(np.min(np.linalg.norm(p-(a+t[:,None]*delta),axis=1))))
            report['distance_to_global_polyline_m']={'max':max(errors),'mean':float(np.mean(errors))}
        target=Path(session['directory'])/('acceptance_'+args.case+'.json');target.write_text(json.dumps(report,ensure_ascii=False,indent=2))
        print(json.dumps({'passed':report['passed'],'report':str(target),'tests':report['tests']},ensure_ascii=False,indent=2),flush=True)
        node.destroy_node();rclpy.try_shutdown()
    if not report['passed']:raise SystemExit(1)


if __name__=='__main__':main()
