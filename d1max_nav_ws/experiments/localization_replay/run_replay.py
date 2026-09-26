"""One-shot, supervised real-bag localization assessment; no robot actuation.

Uses original production LIO/PCD/filter configs. A dedicated replay-only seed
adapter omits only the absent SDK head flag, never fabricates SDK speed/state.
One fixed shared sensor epoch shift preserves sample intervals at 1x; no /clock
or recorded TF is replayed into the estimator. Maps/bags stay read-only.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import yaml
from ament_index_python.packages import get_package_prefix

ROOT = Path(__file__).resolve().parent
WS = Path('/home/dndx/d1max_nav_ws')
APP = Path('/home/dndx/智元四足机器人D1 Max二次开发文档资料包v0.1.0/d1max_ros2')


def save(path, value):
    temp = path.with_suffix('.tmp')
    temp.write_text(json.dumps(value, ensure_ascii=False, indent=2))
    temp.replace(path)


def executable(package, name):
    return str(Path(get_package_prefix(package))/'lib'/package/name)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=1000.)
    args = parser.parse_args()
    out = args.output.resolve()
    if out.parent != WS/'log/localization_replay' or out.exists():
        raise RuntimeError('Require a new exact directory under log/localization_replay')
    if not 20 <= args.duration <= 1200:
        raise ValueError('Bounded replay duration required')
    metadata = yaml.safe_load((args.bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    required = ['/front_lidar', '/rear_lidar', '/front_lidar/imu']
    topic_types = {v['topic_metadata']['name']:v['topic_metadata']['type'] for v in metadata['topics_with_message_count']}
    if any(t not in topic_types for t in required):
        raise RuntimeError('Required real sensor topic absent')
    from d1max_navigation.session import selected_map, map_view
    version, folder, grid, manifest = selected_map(APP)
    if (version != 'grid-0af429985e454a9e99c8aaef'
            or args.bag.resolve() != WS/'bags/slam_raw_20260917_171716_fe8f38'):
        raise RuntimeError('This audited seed/lineage is valid only for bag917 and the selected matching map; re-audit other inputs')
    with socket.socket() as probe:
        if probe.connect_ex(('127.0.0.1', 17449)) == 0:
            raise RuntimeError('Replay router port already owned; no reuse')
    if os.environ.get('RMW_IMPLEMENTATION') != 'rmw_zenoh_cpp':
        raise RuntimeError('Middleware change forbidden')
    if os.environ.get('ZENOH_CONFIG_OVERRIDE') or os.environ.get('ZENOH_SESSION_CONFIG'):
        raise RuntimeError('Unexpected transport override')
    out.mkdir(parents=True)
    for name, mode in [('router', 'router'), ('client', 'client')]:
        config = {'mode':mode, 'connect':{'endpoints':['tcp/127.0.0.1:17449'] if mode=='client' else []},
            'listen':{'endpoints':['tcp/127.0.0.1:17449'] if mode=='router' else []},
            'scouting':{'multicast':{'enabled':False}, 'gossip':{'enabled':False}}}
        save(out/(name+'.json5'), config)
    os.environ.update(ROS_DOMAIN_ID='24', ZENOH_SESSION_CONFIG_URI=str(out/'client.json5'),
        ZENOH_ROUTER_CONFIG_URI=str(out/'router.json5'), ROS_LOG_DIR=str(out/'ros_logs'), QT_QPA_PLATFORM='xcb')
    os.environ.pop('ROS_LOCALHOST_ONLY', None)
    from replay_localizer import validate_isolation
    validate_isolation()
    config = yaml.safe_load((WS/'src/d1max_localization/config/localization.yaml').read_text())
    # Parameter snapshot only; do not modify estimator gates or calibration.
    (out/'localization.yaml').write_text(yaml.safe_dump(config, sort_keys=False))
    session = {'id':out.name, 'version_id':version, 'mode':'RECORDED_BAG_LOCALIZATION_TEST',
        'bag':str(args.bag.resolve()), 'map_pcd':str(folder/'localization.pcd'), 'map_yaml':str(folder/'map.yaml'),
        'playback_rate':1.0, 'topics':required, 'sdk_recorded':False, 'ground_truth_available':False,
        'same_data_as_mapping':True, 'sensor_epoch':'existing estimate_shared_epoch; one constant offset',
        'initial_pose_reference':'tracking', 'robot_motion_enabled':False,
        'started_at':time.time(), 'duration_limit_sec':args.duration}
    save(out/'session.json', session)
    rviz = yaml.safe_load((WS/'src/d1max_navigation/rviz/localization_test.rviz').read_text())
    rviz['Visualization Manager']['Views']['Current'].update(map_view(grid, manifest, (1500,1200)))
    rviz['Visualization Manager']['Tools'] = [t for t in rviz['Visualization Manager']['Tools'] if 'Pose' not in t['Class']]
    for d in rviz['Visualization Manager']['Displays']:
        if d['Class'].endswith('/Marker'):
            d['Name']='RECORDED BAG · NO MOTION'
    (out/'replay.rviz').write_text(yaml.safe_dump(rviz, sort_keys=False, allow_unicode=True))
    children = {}; handles=[]; stopped=False
    def stop_signal(*_):
        nonlocal stopped
        stopped=True
    signal.signal(signal.SIGINT, stop_signal); signal.signal(signal.SIGTERM, stop_signal)
    def start(name, argv):
        stream=(out/(name+'.log')).open('w');handles.append(stream)
        child=subprocess.Popen(argv, cwd=out, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        children[name]=child
        save(out/'processes.json',{k:{'pid':v.pid,'argv':v.args} for k,v in children.items()})
        return child
    def ros(name, package, executable, params=None, remap=None, extras=None):
        argv=[str(Path(get_package_prefix(package))/'lib'/package/executable),'--ros-args']
        if params: argv+=['--params-file',str(params)]
        for old,new in (remap or []): argv+=['-r',old+':='+new]
        for k,v in (extras or {}).items():
            argv+=['-p',k+':='+str(v).lower() if isinstance(v,bool) else k+':='+str(v)]
        return start(name,argv)
    def pause(seconds):
        end=time.monotonic()+seconds
        while not stopped and time.monotonic()<end:time.sleep(.1)
    summary={'completed':False, 'reason':'starting'}
    try:
        start('router',[executable('rmw_zenoh_cpp','rmw_zenohd')]);pause(1.)
        if children['router'].poll() is not None:raise RuntimeError('Replay router failed')
        params=out/'localization.yaml'
        ros('adapter','d1max_localization','dual_lidar_adapter',params)
        ros('lio','faster_lio','run_mapping_online',params,[('__ns','/d1max/localization/lio'),('__node','laserMapping'),
            ('Odometry','odometry'),('cloud_registered_body','deskewed')])
        ros('matcher','d1max_localization','fused_icp_matcher',params,[('__node','lio_global_matcher')],{'map_pcd':folder/'localization.pcd'})
        from d1max_localization.estimation.configuration import navigation_parameters
        prediction,output,ekf=navigation_parameters(config)
        for node_name,values in [('lio_predictor',prediction),('navigation_output',output),('ekf_navigation',ekf)]:
            p=out/(node_name+'.yaml');p.write_text(yaml.safe_dump({node_name:{'ros__parameters':values}}))
            if node_name=='ekf_navigation':
                start(node_name,[executable('robot_localization','ekf_node'),'--ros-args','-r','__node:=ekf_navigation',
                    '--params-file',str(params),'--params-file',str(p),'-r','odometry/filtered:=/d1max/localization/estimator/odometry_raw',
                    '-r','set_pose:=/d1max/localization/estimator/set_pose'])
            else:ros(node_name,'d1max_localization',node_name,p)
        start('localizer',['python3',str(ROOT/'replay_localizer.py'),'--ros-args','--params-file',str(params),
            '-p','session_dir:='+str(out),'-p','navigation_output_enabled:=true'])
        ros('map','nav2_map_server','map_server',extras={'yaml_filename':folder/'map.yaml', 'frame_id':'d1max_loc_map',
            'topic_name':'/d1max/navigation/map'})
        lifecycle=out/'map_lifecycle.yaml'
        lifecycle.write_text(yaml.safe_dump({'map_lifecycle':{'ros__parameters':{'autostart':True,'node_names':['map_server'],'bond_timeout':4.0}}}))
        ros('map_lifecycle','nav2_lifecycle_manager','lifecycle_manager',lifecycle,[('__node','map_lifecycle')])
        ros('view_status','d1max_navigation','navigation_test_status',extras={'offline':False,'expected_version_id':version,
            'expected_session_id':out.name,'anchor_x':float(map_view(grid,manifest)['X']),
            'anchor_y':float(map_view(grid,manifest)['Y']+25),'text_height':.8})
        start('rviz',['/opt/ros/humble/lib/rviz2/rviz2','-d',str(out/'replay.rviz')])
        start('observer',['python3',str(ROOT/'observe_replay.py'),'--output',str(out/'metrics')])
        pause(6.)
        for name,child in children.items():
            if child.poll() is not None:raise RuntimeError(name+' exited before playback')
        player=start('player',['ros2','bag','play',str(args.bag.resolve()),'--rate','1.0',
            '--disable-keyboard-controls','--read-ahead-queue-size','1000','--topics',*required])
        begin=time.monotonic();seeded=False;last_print=-5.; firstlock=None
        while not stopped and time.monotonic()-begin<args.duration:
            elapsed=time.monotonic()-begin
            if player.poll() is not None:
                summary.update(completed=player.returncode==0,reason='bag_finished',player_exit=player.returncode);break
            for name,child in children.items():
                if name!='player' and child.poll() is not None:raise RuntimeError(name+' exited during replay')
            path=out/'status.json';state=json.loads(path.read_text()) if path.exists() else {}
            if not seeded and state.get('initial_pose_ready'):
                # Source-map first keyframe near origin; known initial pose only,
                # never feed the later optimized trajectory as localization input.
                command={'id':'initial_tracking_seed','session_id':out.name,'created_at':time.time(),
                    'reference':'tracking','x':-.00039093,'y':-.000194879,'z':-.0000692083,'yaw':0.0}
                save(out/'initial_pose.json',command);seeded=True
                print('Submitted one initial tracking-reference seed; awaiting actual registration.',flush=True)
            if state.get('localized') and firstlock is None:firstlock=elapsed
            if elapsed-last_print>=5.:
                report={'elapsed_sec':round(elapsed,1),'state':state.get('state'), 'localized':state.get('localized'),
                    'epoch':state.get('local_epoch'),'accepted':state.get('fusion',{}).get('accepted'),
                    'global_hz':state.get('output_observed_hz'), 'matcher':state.get('matcher'),
                    'local_fault':state.get('local_fault'),'last_error':state.get('last_error')}
                save(out/'progress.json',report);print(json.dumps(report,ensure_ascii=False),flush=True);last_print=elapsed
            # A failed acquisition is a test result, not permission to relax gates
            # or repeatedly inject the recorded trajectory until it looks good.
            if firstlock is None and elapsed>90.:
                summary.update(reason='no_verified_localization_within_90_seconds');break
            time.sleep(.1)
        else:summary.update(reason='operator_stop' if stopped else 'duration_limit',completed=not stopped)
        summary.update(first_localization_sec=firstlock,seed_submitted=seeded,elapsed_sec=time.monotonic()-begin)
        pause(1.)
    except Exception as error:
        summary.update(reason=str(error),completed=False)
        raise
    finally:
        # Stop playback first, then only processes owned by this test.
        order=['player']+[n for n in reversed(children) if n not in ('player','router')]+['router']
        for name in order:
            child=children.get(name)
            if child is None or child.poll() is not None:continue
            os.killpg(child.pid,signal.SIGINT)
            try:child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid,signal.SIGKILL);child.wait(timeout=3)
        for stream in handles:stream.close()
        summary['exit_codes']={k:v.returncode for k,v in children.items()}
        summary['robot_commands_sent']=False
        save(out/'result.json',summary)
        print(json.dumps(summary,ensure_ascii=False),flush=True)


if __name__=='__main__':main()
