#!/usr/bin/env python3
"""Real warm ComputeRoute Action benchmark on a private loopback Zenoh graph.

Only NavigationState is simulated (stationary at each recorded query start).
No FollowRoute, navigator, SDK, local planner, motion or production service is
started. Loading/warmup is measured separately from request-to-result latency.
"""
import argparse
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[2]
# Use the exact installed overlay selected by the caller, just like the full
# graph. Prepending the source tree would silently invalidate release evidence.
PREFIX='/d1max/live_planning/'


def summarize(values):
    import numpy as np
    return {key:float(np.percentile(values,q)) for key,q in [('p50',50),('p95',95),('p99',99),('max',100)]} if values else {}


def child(directory,repetitions,timeout):
    from d1max_pct_scan.isolated_zenoh import validate_environment
    validate_environment()
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from d1max_planning_interfaces.msg import NavigationState
    from d1max_navigation_bt_interfaces.action import ComputeRoute
    from std_msgs.msg import String
    session=json.loads((directory/'session.json').read_text())
    source=Path(session['planning_manifest']).parent/'checked_smooth_repeated_strict_ceiling.json'
    audited=json.loads(source.read_text())
    queries=[]
    for q in audited['queries']:
        if q.get('repeat',0)==0 and q['success']:
            queries.append(q)
    if len(queries)!=6:raise ValueError('exact_six_validated_map_queries_required')
    rclpy.init()
    class Bench(Node):
        def __init__(self):
            super().__init__('isolated_compute_route_benchmark')
            self.xyz=list(queries[0]['start_xyz']);self.status={};self.status_received=0.
            self.pub=self.create_publisher(NavigationState,'/d1max/localization/navigation/state',10)
            self.create_subscription(String,PREFIX+'global_status',self.on_status,10)
            self.action=ActionClient(self,ComputeRoute,PREFIX+'bt/compute_route')
            self.create_timer(.02,self.publish_body)
        def on_status(self,message):
            self.status=json.loads(message.data);self.status_received=time.monotonic()
        def publish_body(self):
            stamp=self.get_clock().now().to_msg()
            message=NavigationState(schema_version=2,session_id=session['id'],map_version_id=session['version_id'],
                localization_epoch=1,localization_seed_id='static-benchmark',usable=True,
                reason='SIMULATED_STATIC_INPUT_NOT_ROBOT_LOCALIZATION')
            message.source_stamp=stamp;message.posterior_stamp=stamp;message.imu_stamp=stamp
            for odom,frame in ((message.local_odometry,'d1max_loc_odom'),(message.global_odometry,'d1max_loc_map')):
                odom.header.stamp=stamp;odom.header.frame_id=frame;odom.child_frame_id='d1max_loc_base_link'
                odom.pose.pose.position.x=float(self.xyz[0]);odom.pose.pose.position.y=float(self.xyz[1])
                odom.pose.pose.position.z=float(self.xyz[2]+session['body_height'])
                odom.pose.pose.orientation.w=1.
            self.pub.publish(message)
    node=Bench();records=[];began=time.monotonic()
    report=dict(schema=1,benchmark='real_compute_route_action',actual_native=True,actual_ros_action=True,
        input='simulated_static_navigation_state',follow_used=False,sdk_used=False,motion_used=False,
        physical_acceptance=False,query_source=str(source),session_id=session['id'],
        map_version_id=session['version_id'],queries=records,input_hashes=session['input_hashes'],
        query_source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        benchmark_script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        timer_periods_s=dict(adapter=.05,worker=.05,worker_status=.5),
        percentile_warning='18 samples only; empirical tail quantiles are not statistically robust P99 guarantees')
    def wait(predicate,seconds):
        end=time.monotonic()+seconds
        while not predicate() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
        return predicate()
    try:
        if not wait(lambda:node.action.server_is_ready() and node.status.get('native_warmup',{}).get('phase')=='ready',60):
            raise TimeoutError('worker_or_action_startup_not_ready:'+json.dumps(node.status))
        report['startup_wait_s']=time.monotonic()-began
        report['native_warmup']=deepcopy(node.status.get('native_warmup'))
        for repeat in range(repetitions):
            for index,q in enumerate(queries):
                node.xyz=list(q['start_xyz'])
                wait(lambda:False,.15)  # establish fresh static start before timing
                goal=ComputeRoute.Goal(schema_version=2,session_id=session['id'],
                    task_id=f'benchmark.{repeat}.{index}',goal_kind='3d',has_goal_yaw=False,goal_yaw_tolerance_rad=.15)
                goal.goal.header.frame_id='d1max_loc_map';goal.goal.header.stamp=node.get_clock().now().to_msg()
                p=goal.goal.pose.position;p.x,p.y,p.z=map(float,q['goal_xyz']);goal.goal.pose.orientation.w=1.
                t0=time.perf_counter();future=node.action.send_goal_async(goal)
                record=dict(repeat=repeat,case=f"{q['start_index']}->{q['goal_index']}",
                    start_xyz=q['start_xyz'],goal_xyz=q['goal_xyz'],start_layer=q['start_layer'],goal_layer=q['goal_layer'])
                records.append(record)
                if not wait(future.done,timeout):raise TimeoutError('goal_ack_timeout')
                handle=future.result();record['action_ack_s']=time.perf_counter()-t0
                if not handle.accepted:raise RuntimeError('compute_goal_rejected')
                result=handle.get_result_async()
                if not wait(result.done,timeout):
                    cancellation=handle.cancel_goal_async();wait(cancellation.done,3.)
                    raise TimeoutError('compute_result_timeout')
                outcome=result.result().result
                record.update(action_end_to_end_s=time.perf_counter()-t0,success=outcome.success,
                    reason=outcome.reason,route_hash=outcome.snapshot.route_hash,points=len(outcome.route.poses))
                record['actual_source_layers']=sorted(set(outcome.snapshot.source_layer_ids))
                record['worker_metrics']=deepcopy(node.status.get('worker_metrics',{}))
                record['worker_generation']=node.status.get('generation')
                print(json.dumps(record),flush=True)
                if not outcome.success:raise RuntimeError('compute_failed:'+outcome.reason)
        report['success']=True
    except Exception as error:
        report['success']=False;report['error']=str(error);report['last_status']=node.status
    finally:
        report['duration_s']=time.monotonic()-began
        report['summary_s']={key:summarize([r[key] for r in records if r.get('success') and key in r])
            for key in ('action_end_to_end_s','action_ack_s')}
        for field in ('native_plan_elapsed_sec','worker_elapsed_sec','parent_validation_elapsed_sec',
                      'message_build_sec','total_goal_elapsed_sec','initialization_sec'):
            report['summary_s'][field]=summarize([r['worker_metrics'][field] for r in records if r.get('success')
                and isinstance(r.get('worker_metrics',{}).get(field),(int,float))])
        report['per_case_s']={case:summarize([r['action_end_to_end_s'] for r in records if r.get('success') and r['case']==case])
            for case in dict.fromkeys(r['case'] for r in records)}
        (directory/'action_benchmark.json').write_text(json.dumps(report,indent=2)+'\n')
        node.destroy_node();rclpy.shutdown()
    return 0 if report['success'] else 2


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--map-directory',type=Path)
    parser.add_argument('--repetitions',type=int,default=3)
    parser.add_argument('--timeout',type=float,default=15.)
    parser.add_argument('--child',action='store_true')
    args=parser.parse_args();directory=args.output.resolve()
    if not 1<=args.repetitions<=10 or not 5<=args.timeout<=60:parser.error('bounded repetitions/timeout required')
    if args.child:return child(directory,args.repetitions,args.timeout)
    from d1max_pct_scan.single_floor_session import prepare
    from d1max_pct_scan.bt_configuration import worker_arguments
    from d1max_pct_scan.isolated_zenoh import private_router,stop_owned
    prepare(directory,**({'map_directory':args.map_directory} if args.map_directory else {}))
    commands={name:[sys.executable,'-m','d1max_pct_scan.'+module,'--ros-args','--params-file',str(directory/params)]
        for name,module,params in [('global','live_global_planner','global.yaml'),('adapters','bt_adapters','bt_adapter.yaml')]}
    commands['global']+=worker_arguments()
    children=[];streams=[]
    try:
        with private_router(directory/'zenoh') as env:
            env['PYTHONUNBUFFERED']='1'
            for name,command in commands.items():
                stream=(directory/(name+'.log')).open('w');streams.append(stream)
                children.append(subprocess.Popen(command,cwd=ROOT,env=env,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True))
            try:
                result=subprocess.run([sys.executable,__file__,'--child','--output',str(directory),
                    '--repetitions',str(args.repetitions),'--timeout',str(args.timeout)],cwd=ROOT,env=env,
                    timeout=80+args.repetitions*6*(args.timeout+1))
            finally:
                for process in reversed(children):stop_owned(process)
        for stream in streams:stream.close()
        report_path=directory/'action_benchmark.json';report=json.loads(report_path.read_text())
        # Native A* itself logs one search duration; it is not the complete
        # native plan (checked smoothing), much less full Action latency.
        text=(directory/'global.log').read_text(errors='replace')
        times=[float(v)/1000 for v in re.findall(r'path found, time elapsed: ([0-9.]+) ms',text)]
        report['native_search_log_samples_s']=times
        report['native_search_log_summary_s']=summarize(times)
        report['native_search_log_count_matches_requests']=len(times)==len(report['queries'])
        report['owned_processes_stopped']=all(p.poll() is not None for p in children)
        report['commands']=commands
        report_path.write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps({'report':str(report_path),'success':report['success'],
            'summary_s':report['summary_s'],'native_search_s':report['native_search_log_summary_s']}),flush=True)
        return result.returncode
    finally:
        for process in reversed(children):stop_owned(process)
        for stream in streams:
            if not stream.closed:stream.close()


if __name__=='__main__':sys.exit(main())
