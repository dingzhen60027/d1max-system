"""Owned, loopback-only PCT + SCAN offline sessions. Never opens a robot SDK."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import socket
import subprocess
import uuid
from datetime import datetime
import yaml
from d1max_pct_planner import paths

WS=paths.nav_root()
SOURCE=WS/'src/d1max_pct_scan'
ROOT=WS/'log/pct_scan'
UNIT='d1max-pct-scan.service'
OWNER='D1MAX-PCT-SCAN-OFFLINE:'


def service():
    result=subprocess.run(['systemctl','--user','show',UNIT,'--property=ActiveState,Description'],capture_output=True,text=True,check=False)
    return dict(line.split('=',1) for line in result.stdout.splitlines() if '=' in line)


def stop():
    state=service()
    if state.get('ActiveState') not in ('active','activating','deactivating'):return
    if not state.get('Description','').startswith(OWNER):raise RuntimeError('Unknown service owner; refusing to stop it')
    subprocess.run(['systemctl','--user','stop',UNIT],check=True,timeout=20)
    print('Stopped offline PCT + SCAN and its managed child processes; robot SDK untouched.')


def rviz_config(sid,cfg):
    def topic(name):return {'Value':name,'Depth':1,'Reliability Policy':'Reliable','Durability Policy':'Volatile'}
    scan_ns='/d1max/pct_scan/s_'+sid+'/scan'
    x,y=cfg['initial_xy']
    return {'Panels':[{'Class':'rviz_common/Displays','Name':'Displays'},{'Class':'rviz_common/Tool Properties','Name':'Tool Properties'}],
      'Visualization Manager':{'Class':'','Global Options':{'Fixed Frame':cfg['map_frame'],'Background Color':'20; 24; 30','Frame Rate':30},
      'Displays':[
        {'Class':'rviz_default_plugins/Grid','Name':'Grid','Enabled':True,'Cell Size':1.,'Plane Cell Count':80},
        {'Class':'rviz_default_plugins/PointCloud2','Name':'Recorded SC-PGO map (offline)','Enabled':True,'Topic':{**topic('/d1max/pct_scan/map_cloud'),'Durability Policy':'Transient Local'},'Style':'Points','Size (Pixels)':2,'Color Transformer':'AxisColor','Axis':'Z','Decay Time':0},
        {'Class':'rviz_default_plugins/Path','Name':'PCT global route','Enabled':True,'Topic':topic('/d1max/pct_scan/global_path'),'Color':'70; 220; 255','Line Style':'Billboards','Line Width':.05},
        {'Class':'rviz_default_plugins/Path','Name':'Actual simulated travel','Enabled':True,'Topic':{**topic('/d1max/pct_scan/simulated_trail'),'Durability Policy':'Transient Local'},'Color':'255; 120; 40','Line Style':'Billboards','Line Width':.035},
        {'Class':'rviz_default_plugins/Marker','Name':'SCAN optimized local spline','Enabled':True,'Topic':topic(scan_ns+'/optimal_list')},
        {'Class':'rviz_default_plugins/Marker','Name':'Simulated body (NOT ROBOT)','Enabled':True,'Topic':{**topic('/d1max/pct_scan/simulation_marker'),'Durability Policy':'Transient Local'}},
        {'Class':'rviz_default_plugins/Polygon','Name':'Safety stop zone','Enabled':True,'Topic':topic('/d1max/pct_scan/stop_polygon'),'Color':'255; 80; 80'}],
      'Tools':[{'Class':'rviz_default_plugins/Interact'},{'Class':'rviz_default_plugins/MoveCamera'},{'Class':'rviz_default_plugins/Select'},{'Class':'rviz_default_plugins/SetGoal','Topic':'/d1max/pct_scan/goal'}],
      'Views':{'Current':{'Class':'rviz_default_plugins/Orbit','Distance':16.,'Pitch':1.25,'Yaw':0.,'Focal Point':{'X':float(x),'Y':float(y),'Z':0.},'Target Frame':cfg['map_frame']}}},
      'Window Geometry':{'Width':1500,'Height':1000,'X':80,'Y':30}}


def start(args):
    state=service()
    if state.get('ActiveState') in ('active','activating','deactivating'):raise RuntimeError('Offline session already exists; stop it explicitly before starting another')
    # Never reuse someone else's router, even if it happens to be local.
    with socket.socket() as sock:
        sock.bind(('127.0.0.1',7464))
    cfg=yaml.safe_load(args.config.read_text())
    for key in ('map_pcd','ground_pcd','planning_grid'):
        if not Path(cfg[key]).is_file():raise ValueError('Missing '+key+': '+cfg[key])
    if not 0 < cfg['max_speed'] <= .3 or not 0 < cfg['max_yaw_rate'] <= .5:
        raise ValueError('Offline acceptance profile is limited to 0.30 m/s and 0.50 rad/s')
    sid=uuid.uuid4().hex[:12];directory=ROOT/(datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+sid);directory.mkdir(parents=True)
    config_path=directory/'offline.yaml';config_path.write_text(yaml.safe_dump(cfg))
    scout={'multicast':{'enabled':False},'gossip':{'enabled':False}}
    common={'scouting':scout,'timestamping':{'enabled':True,'drop_future_timestamp':False}}
    client={**common,'mode':'client','connect':{'endpoints':['tcp/127.0.0.1:7464'],'exit_on_failure':True}}
    router={**common,'mode':'router','listen':{'endpoints':['tcp/127.0.0.1:7464'],'exit_on_failure':True},'connect':{'endpoints':[]}}
    for name,data in [('client',client),('router',router)]:
        (directory/(name+'.json5')).write_text(json.dumps(data,indent=2))
    pct=paths.expand_tree(yaml.safe_load((WS/'src/d1max_pct_planner/config/pct_scan_single_floor.yaml').read_text()))['pct_route_server']['ros__parameters']
    pct.update(session_id=sid,planning_grid=cfg['planning_grid'],planning_frame=cfg['map_frame'],body_frame=cfg['base_frame'],body_height_m=cfg['body_height'])
    scan={'use_sim_time':False,'fsm.navi_mode':3,'fsm.require_tagged_reference':True,'fsm.navigation_session_id':sid,
          'fsm.strict_input_frames':True,'fsm.odom_twist_in_body_frame':True,'fsm.odom_timeout':.5,
          'fsm.reference_path_z_offset':cfg['body_height'],'fsm.reference_start_tolerance':1.,'fsm.max_replan_interval':1.,
          'fsm.reference_path_guidance':True,
          'optimization.vel_tolerance':.03,'optimization.acc_tolerance':.05,
          'grid_map.frame_id':cfg['map_frame'],'grid_map.body_height':cfg['body_height'],'grid_map.sliding_map_frame_id':'d1max_pct_scan_'+sid+'_local',
          'grid_map.strict_input_frames':True,'grid_map.maximum_cloud_pose_dt':.25,'grid_map.cloud_is_world':True,'grid_map.need_extrinsic':False,
          'manager.max_vel':cfg['max_speed'],'optimization.max_vel':cfg['max_speed'],'manager.feasibility_tolerance':.05}
    tracker={'session_id':sid,'planning_frame':cfg['map_frame'],'base_frame':cfg['base_frame'],
             'odom_topic':'/d1max/localization/odometry/global','trajectory_topic':'/d1max/pct_scan/validated_bspline',
             'max_speed':cfg['max_speed'],'max_yaw_rate':cfg['max_yaw_rate']}
    rviz_path=directory/'offline.rviz';rviz_path.write_text(yaml.safe_dump(rviz_config(sid,cfg),sort_keys=False))
    session={'mode':'sim','session_id':sid,'robot_connected':False,'real_motion_enabled':False,'directory':str(directory),'config':str(config_path),
             'headless':args.headless,'rviz_config':str(rviz_path),'pct_parameters':pct,'scan_parameters':scan,'tracker_parameters':tracker,
             'scan_config':str(WS/'src/d1max_scan_planner/config/d1max_scan_planner.yaml'),'tracker_executable':'trajectory_tracker'}
    snapshot=directory/'session.json';snapshot.write_text(json.dumps(session,indent=2))
    environment={key:os.environ[key] for key in ('PATH','LD_LIBRARY_PATH','PYTHONPATH','AMENT_PREFIX_PATH','CMAKE_PREFIX_PATH','COLCON_PREFIX_PATH','DISPLAY','XAUTHORITY','XDG_RUNTIME_DIR','LANG') if key in os.environ}
    environment.update(RMW_IMPLEMENTATION='rmw_zenoh_cpp',ROS_DOMAIN_ID='24',QT_QPA_PLATFORM='xcb',ROS_LOG_DIR=str(directory/'ros_logs'))
    cmd=['systemd-run','--user','--collect','--unit='+UNIT,'--property=Description='+OWNER+sid,'--property=Type=exec',
         '--property=KillMode=mixed','--property=KillSignal=SIGINT','--property=TimeoutStopSec=15','--property=SendSIGKILL=yes','--working-directory='+str(WS)]
    cmd+=['--setenv='+k+'='+v for k,v in environment.items()]
    cmd+=[str(WS/'install/d1max_pct_scan/lib/d1max_pct_scan/pct_scan_run'),'--session',str(snapshot)]
    subprocess.run(cmd,check=True,timeout=15)
    (ROOT/'last_session.json').write_text(json.dumps(session,indent=2))
    print(json.dumps({'started':'OFFLINE PCT + SCAN','session':str(snapshot),'test_goal_xy':cfg['test_goal_xy'],'robot_connected':False},indent=2))


def main():
    parser=argparse.ArgumentParser(description='Offline PCD + PCT + SCAN, isolated Zenoh, no SDK')
    parser.add_argument('action',choices=['start','stop','status','cancel'],nargs='?',default='start')
    parser.add_argument('--headless',action='store_true')
    parser.add_argument('--config',type=Path,default=SOURCE/'config/offline.yaml')
    args=parser.parse_args();ROOT.mkdir(parents=True,exist_ok=True)
    with (ROOT/'session.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        if args.action=='start':start(args)
        elif args.action=='stop':stop()
        elif args.action=='status':print(json.dumps({'service':service(),'last_session':json.loads((ROOT/'last_session.json').read_text()) if (ROOT/'last_session.json').exists() else None},indent=2))
        else:
            state=service()
            if state.get('ActiveState')!='active' or not state.get('Description','').startswith(OWNER):raise RuntimeError('No owned offline session')
            s=json.loads((ROOT/'last_session.json').read_text())
            if state.get('Description')!=OWNER+s['session_id']:raise RuntimeError('Session snapshot does not match the running service')
            os.environ.update(RMW_IMPLEMENTATION='rmw_zenoh_cpp',ROS_DOMAIN_ID='24',ZENOH_SESSION_CONFIG_URI=str(Path(s['directory'])/'client.json5'))
            for key in ('ZENOH_CONFIG_OVERRIDE','ZENOH_SESSION_CONFIG'):os.environ.pop(key,None)
            subprocess.run(['ros2','topic','pub','--once','--wait-matching-subscriptions','1','/d1max/pct_scan/cancel','std_msgs/msg/Empty','{}'],check=True,timeout=10)


if __name__=='__main__':main()
