"""One managed offline PCT/SCAN session; all children share a private router."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import yaml
from ament_index_python.packages import get_package_share_directory, get_package_prefix


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--session',type=Path,required=True);args=ap.parse_args()
    s=json.loads(args.session.read_text());directory=args.session.parent
    if s.get('mode')!='sim':raise ValueError('This entry is OFFLINE ONLY; no SDK connection or live mode')
    for k in ('ZENOH_CONFIG_OVERRIDE','ZENOH_SESSION_CONFIG'):os.environ.pop(k,None)
    os.environ.update(RMW_IMPLEMENTATION='rmw_zenoh_cpp',ROS_DOMAIN_ID='24',ZENOH_SESSION_CONFIG_URI=str(directory/'client.json5'),ZENOH_ROUTER_CONFIG_URI=str(directory/'router.json5'),OMP_NUM_THREADS='4',OPENBLAS_NUM_THREADS='2')
    cfg=yaml.safe_load(Path(s['config']).read_text());processes=[];logs=[];stop=False
    def shutdown(*_):
        nonlocal stop
        stop=True
    signal.signal(signal.SIGINT,shutdown);signal.signal(signal.SIGTERM,shutdown)
    def spawn(cmd):
        print('START '+json.dumps(cmd),flush=True)
        name=cmd[3] if cmd[:2]==['ros2','run'] else Path(cmd[0]).name
        stream=(directory/(name+'.log')).open('a');logs.append(stream)
        p=subprocess.Popen(cmd,stdout=stream,stderr=subprocess.STDOUT);processes.append(p);return p
    def ros(package,exe,name,params=None,remaps=()):
        command=[str(Path(get_package_prefix(package))/'lib'/package/exe),'--ros-args','-r','__node:='+name]
        if params is not None:
            path=directory/(name+'.yaml');path.write_text(yaml.safe_dump({'/**':{'ros__parameters':params}}));command+=['--params-file',str(path)]
        for a,b in remaps:command+=['-r',a+':='+b]
        return spawn(command)
    try:
        router=Path(get_package_prefix('rmw_zenoh_cpp'))/'lib/rmw_zenoh_cpp/rmw_zenohd';spawn([str(router)]);time.sleep(.8)
        if processes[0].poll() is not None:raise RuntimeError('Private router failed; refusing to reuse an unknown router')
        ros('d1max_pct_scan','pct_scan_simulator','pct_scan_simulator',{'config':s['config'],'session_id':s['session_id']})
        # True native PCT route server and true SCAN optimizer, never Web A*.
        ros('d1max_pct_planner','pct_route_server','pct_route_server',s['pct_parameters'])
        scan_config=yaml.safe_load(Path(s['scan_config']).read_text())
        scan_params=next(iter(scan_config.values()))['ros__parameters'];scan_params.update(s['scan_parameters'])
        scan_ns='/d1max/pct_scan/s_'+s['session_id']+'/scan'
        scan_file=directory/'scan.yaml';scan_file.write_text(yaml.safe_dump({'/**':{'ros__parameters':scan_params}}))
        spawn([str(Path(get_package_prefix('scan_planner'))/'lib/scan_planner/scan_planner_node'),'--ros-args','-r','__ns:='+scan_ns,'-r','__node:=scan_planner_node','--params-file',str(scan_file),
            '-r','body_pose:=/d1max/localization/odometry/global','-r','sensor_pose:=/d1max/pct_scan/sensor_pose','-r','cloud:=/d1max/pct_scan/cloud_map',
            '-r','typed_initial_path:=/d1max/pct_scan/reference_path','-r','planning/go2_execution_frozen:=/d1max/pct_scan/execution_frozen'])
        ros('d1max_trajectory_tracker',s['tracker_executable'],'pct_scan_tracker',s['tracker_parameters'])
        ros('d1max_pct_scan','pct_scan_trajectory_guard','pct_scan_trajectory_guard',{
            'session_id':s['session_id'],'planning_grid_path':cfg['planning_grid'],'frame_id':cfg['map_frame'],
            'body_height':cfg['body_height'],'input_topic':scan_ns+'/planning/tagged_bspline','audit_directory':str(directory/'rejected_splines')})
        collision=yaml.safe_load((Path(get_package_share_directory('d1max_navigation'))/'config/collision_monitor.yaml').read_text())['collision_monitor']['ros__parameters']
        collision.update(cmd_vel_in_topic='/d1max/pct_scan/cmd_vel_raw',cmd_vel_out_topic='/d1max/pct_scan/cmd_vel_checked',base_frame_id=cfg['base_frame'],odom_frame_id=cfg['odom_frame'])
        collision['scan']['topic']='/d1max/pct_scan/scan';collision['StopFootprint']['polygon_pub_topic']='/d1max/pct_scan/stop_polygon'
        ros('nav2_collision_monitor','collision_monitor','pct_scan_collision',collision)
        ros('nav2_lifecycle_manager','lifecycle_manager','pct_scan_collision_lifecycle',{'autostart':True,'node_names':['pct_scan_collision'],'bond_timeout':4.})
        ros('d1max_navigation','navigation_command_gate','pct_scan_gate',{
            'mode':'simulation','motion_enabled':False,'navigation_session_id':s['session_id'],
            'map_frame':cfg['map_frame'],'odom_frame':cfg['odom_frame'],'base_frame':cfg['base_frame'],
            'max_forward':cfg['max_speed'],'max_lateral':0.,'max_yaw':cfg['max_yaw_rate'],'max_planar_speed':1.5,
            'input_topic':'/d1max/pct_scan/cmd_vel_checked','output_topic':'/d1max/pct_scan/cmd_vel_safe',
            'scan_topic':'/d1max/pct_scan/scan','status_topic':'/d1max/pct_scan/gate_status'})
        if not s['headless']:
            spawn([str(Path(get_package_prefix('rviz2'))/'lib/rviz2/rviz2'),'-d',s['rviz_config'],'--ros-args','-r','__node:=pct_scan_rviz','-r','/goal_pose:=/d1max/pct_scan/goal'])
        while not stop:
            ended=[p.pid for p in processes if p.poll() is not None]
            if ended:raise RuntimeError('Managed child exited: '+str(ended))
            time.sleep(.2)
    finally:
        for p in reversed(processes[1:]):
            if p.poll() is None:p.send_signal(signal.SIGINT)
        deadline=time.monotonic()+7
        for p in reversed(processes[1:]):
            try:p.wait(timeout=max(.1,deadline-time.monotonic()))
            except subprocess.TimeoutExpired:p.terminate()
        if processes and processes[0].poll() is None:
            processes[0].send_signal(signal.SIGINT)
            try:processes[0].wait(timeout=2.)
            except subprocess.TimeoutExpired:processes[0].terminate()
        for stream in logs:stream.close()


if __name__=='__main__':main()
