"""Real BT + real LIO/GICP/EKF with recorded raw sensors; no SDK or motion.

This supervisor controls ONLY playback and process lifetime. BT task states,
health, localization confirmations and global routes are produced by the real
nodes. A Navigate preview goal is sent before startup to exercise the actual
InitialLocalization node, not a replacement Python task machine.
"""
import argparse
from collections import deque
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import sys
import time

import yaml


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')


def binary(package, name):
    from ament_index_python.packages import get_package_prefix
    return str(Path(get_package_prefix(package)) / 'lib' / package / name)


def replay(env, directory, session, commands, bag, duration, with_rviz):
    os.environ.update(env)
    from d1max_pct_scan.isolated_zenoh import stop_owned, validate_environment
    validate_environment()
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.serialization import deserialize_message
    from rclpy.qos import qos_profile_sensor_data
    from rosidl_runtime_py.utilities import get_message
    from rosgraph_msgs.msg import Clock
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2, Imu
    from std_msgs.msg import String
    from visualization_msgs.msg import Marker
    from d1max_navigation_bt_interfaces.action import Navigate
    from d1max_planning_interfaces.msg import NavigationState as NavigationStateMessage, LocalNavigationState

    children = []; logs = []; stopping = False
    def stop(*_):
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGTERM, stop); signal.signal(signal.SIGINT, stop)
    def start(name, command):
        stream = (directory/(name+'.log')).open('w'); logs.append(stream)
        p = subprocess.Popen(command, env=env, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        children.append((name, p)); return p

    # Read one raw recording; never replay its old localization, map TF,
    # robot control, goals or health. All sensor bytes/times remain unchanged.
    metadata = yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    source = bag/metadata['relative_file_paths'][0]
    db = sqlite3.connect('file:'+str(source)+'?mode=ro', uri=True)
    selected = ('/front_lidar', '/rear_lidar', '/front_lidar/imu')
    topics = {row[0]:(row[1], get_message(row[2])) for row in db.execute('SELECT id,name,type FROM topics') if row[1] in selected}
    if {v[0] for v in topics.values()} != set(selected):
        raise ValueError('raw_dual_lidar_and_front_imu_required')
    first = db.execute('SELECT MIN(timestamp) FROM messages WHERE topic_id IN (%s)' % ','.join('?'*len(topics)), tuple(topics)).fetchone()[0]
    imu_id = next(k for k,v in topics.items() if v[0] == '/front_lidar/imu')
    samples = db.execute('SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp LIMIT 200', (imu_id,)).fetchall()
    # One fixed receipt->sensor-epoch offset for /clock only. Raw headers and
    # per-point absolute timestamps are not changed or freshly restamped.
    offset = max(deserialize_message(data, Imu).header.stamp.sec*10**9+
        deserialize_message(data, Imu).header.stamp.nanosec-stamp for stamp,data in samples) + 2_000_000
    cursor = db.execute('SELECT timestamp,topic_id,data FROM messages WHERE topic_id IN (%s) AND timestamp<=? ORDER BY timestamp,id' % ','.join('?'*len(topics)), (*topics,first+round(duration*1e9)))
    next_row = cursor.fetchone()
    rclpy.init()
    # Install AFTER rclpy.init(), which otherwise replaces SIGTERM and can
    # invalidate the context before the supervisor reaches child cleanup.
    signal.signal(signal.SIGTERM, stop); signal.signal(signal.SIGINT, stop)
    node = rclpy.create_node('d1max_bt_recorded_sensor_input')
    pubs = {k:node.create_publisher(v[1],v[0],qos_profile_sensor_data) for k,v in topics.items()}
    clock = node.create_publisher(Clock, '/clock', 10)
    replay_status = node.create_publisher(String, '/d1max/replay/status', 1)
    marker = node.create_publisher(Marker, '/d1max/replay/bt_stage', 1)
    client = ActionClient(node, Navigate, '/d1max/live_planning/bt/navigate')
    last = {}; pose_samples = deque(maxlen=100000); atomic_samples = deque(maxlen=100000); events = []
    atomic_receipts = deque(maxlen=100000); imu_ages = deque(maxlen=100000)
    posterior_ages = deque(maxlen=100000); realtime = {}
    local_samples=deque(maxlen=100000);local_receipts=deque(maxlen=100000);local_unavailable=deque(maxlen=10000)
    local_timing_rows=deque(maxlen=100000);atomic_timing_rows=deque(maxlen=100000)
    timing_counts={'local':0,'global':0}
    lio_notices = deque(maxlen=10000); lio_valid_samples = 0
    bt_input_timing = {}; planning_observations = {}
    status_stream = (directory/'events.jsonl').open('w')
    def receive(key, m):
        try: v = json.loads(m.data)
        except (ValueError, TypeError): return
        previous = last.get(key, {})
        last[key] = v
        fields = ('state','reason') if key == 'startup' else ('active_node','root_status','reason') if key == 'bt' else ('valid','reason','fault')
        if any(v.get(f) != previous.get(f) for f in fields):
            record = dict(wall=time.time(), virtual_ns=virtual_ns, topic=key, value=v)
            status_stream.write(json.dumps(record, ensure_ascii=False)+'\n'); status_stream.flush()
            events.append({k:record[k] for k in ('wall','virtual_ns','topic')} | {f:v.get(f) for f in fields})
            print(key, {f:v.get(f) for f in fields}, flush=True)
    subscriptions = [node.create_subscription(String, topic, lambda m,k=key:receive(k,m), 10)
        for key,topic in [('startup','/d1max/localization/global_relocalization/status'),
            ('bt','/d1max/live_planning/bt/status'),('navigation','/d1max/localization/navigation/status')]]
    def pose(m):
        p=m.pose.pose.position
        pose_samples.append((m.header.stamp.sec*10**9+m.header.stamp.nanosec,p.x,p.y,p.z))
    subscriptions.append(node.create_subscription(Odometry,'/d1max/localization/odometry/global',pose,qos_profile_sensor_data))
    def atomic_state(m):
        stamp=m.source_stamp.sec*10**9+m.source_stamp.nanosec
        receipt=time.monotonic_ns()
        atomic_samples.append(stamp);atomic_receipts.append(receipt)
        timing_counts['global']+=1
        atomic_timing_rows.append(dict(source_ns=stamp,receipt_monotonic_ns=receipt,
            posterior_ns=m.posterior_stamp.sec*10**9+m.posterior_stamp.nanosec,
            imu_ns=m.imu_stamp.sec*10**9+m.imu_stamp.nanosec))
        imu_ages.append((stamp-m.imu_stamp.sec*10**9-m.imu_stamp.nanosec)*1e-9)
        posterior_ages.append((stamp-m.posterior_stamp.sec*10**9-m.posterior_stamp.nanosec)*1e-9)
    subscriptions.append(node.create_subscription(NavigationStateMessage,
        '/d1max/localization/navigation/state',atomic_state,10))
    def local_state(m):
        source=m.source_stamp.sec*10**9+m.source_stamp.nanosec
        receipt=time.monotonic_ns()
        timing_counts['local']+=1
        p,q=m.local_odometry.pose.pose.position,m.local_odometry.pose.pose.orientation
        local_timing_rows.append(dict(source_ns=source,receipt_monotonic_ns=receipt,
            posterior_ns=m.posterior_stamp.sec*10**9+m.posterior_stamp.nanosec,
            imu_ns=m.imu_stamp.sec*10**9+m.imu_stamp.nanosec,
            session_id=m.session_id,localization_epoch=m.localization_epoch,
            localization_seed_id=m.localization_seed_id,usable=m.usable,reason=m.reason,
            pose_xyz=[p.x,p.y,p.z] if m.usable else None,
            pose_xyzw=[q.x,q.y,q.z,q.w] if m.usable else None))
        if m.usable:
            local_samples.append(source);local_receipts.append(receipt)
        else:local_unavailable.append(dict(source_ns=source,reason=m.reason,epoch=m.localization_epoch))
    subscriptions.append(node.create_subscription(LocalNavigationState,
        '/d1max/localization/navigation/local_state',local_state,10))
    def timing(m):
        try: realtime.update(json.loads(m.data))
        except (ValueError,TypeError): pass
    subscriptions.append(node.create_subscription(String,
        '/d1max/localization/navigation/realtime',timing,10))
    def native_posterior(m):
        nonlocal lio_valid_samples
        try: value=json.loads(m.data)
        except (ValueError,TypeError): return
        if value.get('valid') is True:
            lio_valid_samples+=1
        else:
            lio_notices.append(dict(virtual_ns=virtual_ns,**value))
    subscriptions.append(node.create_subscription(String,
        '/d1max/localization/lio/local_sample',native_posterior,20))
    def input_timing(m):
        try: bt_input_timing.update(json.loads(m.data))
        except (ValueError,TypeError): pass
    subscriptions.append(node.create_subscription(String,
        '/d1max/replay/bt_input_timing',input_timing,10))
    if session.get('replay_planning_graph'):
        def planning_status(key, message):
            try: value = json.loads(message.data)
            except (ValueError, TypeError): return
            planning_observations[key] = value
            # Keep the original production readiness facts, not only the last
            # post-playback drain state. No authority or sensor times change.
            # This on-disk diagnostic stream is independent of control queues.
            if key == 'map':
                record = dict(wall=time.time(),virtual_ns=virtual_ns,
                    topic='planning_map',value=value)
                status_stream.write(json.dumps(record,ensure_ascii=False)+'\n')
        for key, topic in [('reference','scan_bridge_status'), ('projector','ray_projector_status'),
                ('map','rays_status'), ('native','native_local_attempt_debug'), ('tracker','tracker_status')]:
            subscriptions.append(node.create_subscription(String, '/d1max/live_planning/'+topic,
                lambda m,k=key:planning_status(k,m),10))
    virtual_ns = first+offset
    report = dict(session_id=session['id'], bag=str(bag), map_pcd=session['map_pcd'],
        input_topics=selected, source_headers_unchanged=True, clock_receipt_offset_ns=offset,
        old_localization_replayed=False, sdk_connected=False, motion_enabled=False,
        physical_acceptance=False, pipeline='BehaviorTree.CPP + Faster-LIO + native GICP + robot_localization',
        virtual_clock_pause_for_startup=True, sdk_head_state_present=False,
        seed_admission='isolated_tracking_reference_only_not_live_sdk_acceptance',
        timing_backend=session['replay_timing_backend'])
    if session.get('replay_planning_graph'):
        report['pipeline'] += ' + PCT + odom reference + native SCAN/GridMap + unrouted tracker'
    started = time.monotonic(); play_elapsed = 0.; played = 0; paused = False; held = False
    pause_start = None; startup_ready_at = None; goal_future = None; goal_handle = None
    deferred_views={k:v for k,v in commands.items() if k in ('view','map_layers','execution_view')}
    views_started=False
    def start_views():
        nonlocal views_started
        if views_started: return
        for name,cmd in deferred_views.items(): start(name,cmd)
        if with_rviz:
            layouts = ('global_planning','local_planning') if session.get('replay_planning_graph') else ('localization',)
            for layout in layouts:
                start('rviz_'+layout,['rviz2','-d',str(directory/(layout+'.rviz')),
                    '--ros-args','-p','use_sim_time:=true','-r','__node:=d1max_bt_replay_'+layout])
        views_started=True
    try:
        for name,cmd in commands.items():
            if name not in deferred_views: start(name,cmd)
        # Dense-map rendering is lower priority than the bounded startup
        # search. Do not compete with FPFH/GICP during initialization.
        if not session.get('replay_planning_graph'): start_views()
        # Prewarm index and native map before playing. Wall poll survives the
        # frozen virtual clock. No sensor is invented to make a node ready.
        deadline=time.monotonic()+150.
        while not stopping and last.get('startup',{}).get('state') not in ('waiting_stationary','waiting_lio_and_head_forward','waiting_scan'):
            now_clock=Clock(); now_clock.clock.sec,now_clock.clock.nanosec=divmod(virtual_ns,10**9); clock.publish(now_clock)
            rclpy.spin_once(node,timeout_sec=.01)
            if time.monotonic()>deadline: raise RuntimeError('startup_index_or_lifecycle_timeout')
            for name,p in children:
                if not name.startswith('rviz_') and p.poll() is not None: raise RuntimeError('component_exited:'+name)
        # Submit an explicit PREVIEW task so the real BT waits on initial pose.
        # Target is a recorded/map-supported one-floor test goal, not motion.
        deadline=time.monotonic()+10.
        while not stopping and not client.server_is_ready():
            rclpy.spin_once(node,timeout_sec=.02)
            if time.monotonic()>deadline: raise RuntimeError('navigate_action_not_ready')
        goal=Navigate.Goal(schema_version=2,goal_kind='3d',has_goal_yaw=False,goal_yaw_tolerance_rad=.15)
        goal.goal.header.frame_id='d1max_loc_map';goal.goal.header.stamp.sec,goal.goal.header.stamp.nanosec=divmod(virtual_ns,10**9)
        goal.goal.pose.position.x,goal.goal.pose.position.y,goal.goal.pose.position.z=session['replay_preview_goal']
        goal.goal.pose.orientation.w=1.
        goal_future=client.send_goal_async(goal)
        previous_mono=time.monotonic(); loop_start=previous_mono
        while not stopping and time.monotonic()-loop_start<duration+90.:
            now=time.monotonic(); dt=now-previous_mono; previous_mono=now
            if goal_handle is None and goal_future.done():
                goal_handle=goal_future.result()
                if not goal_handle.accepted: raise RuntimeError('bt_preview_goal_rejected')
                report['bt_goal_accepted']=True
            startup=last.get('startup',{})
            if startup.get('state')=='searching' and not held:
                paused=held=True;pause_start=now;print('Playback clock paused for genuine global search',flush=True)
            if paused and startup.get('state') in ('confirming','ready','failed'):
                paused=False;report['startup_pause_wall_seconds']=now-pause_start
                print('Playback resumed; native matcher must verify seed',flush=True)
            if not paused: play_elapsed+=dt
            virtual_ns=first+offset+round(min(play_elapsed,duration)*1e9)
            replay_status.publish(String(data=json.dumps(dict(session_id=session['id'],paused=paused,virtual_ns=virtual_ns))))
            stamp=Clock();stamp.clock.sec,stamp.clock.nanosec=divmod(virtual_ns,10**9);clock.publish(stamp)
            while next_row is not None and next_row[0]+offset<=virtual_ns:
                receipt,tid,data=next_row;pubs[tid].publish(deserialize_message(data,topics[tid][1]));played+=1
                next_row=cursor.fetchone()
            rclpy.spin_once(node,timeout_sec=.002)
            if startup.get('state')=='ready' and startup_ready_at is None:
                startup_ready_at=play_elapsed
                start_views()
            if played and int(now*2)!=int((now-dt)*2):
                bt=last.get('bt',{});msg=Marker();msg.header.frame_id='d1max_loc_map';msg.header.stamp=stamp.clock
                msg.ns='bt_replay_stage';msg.id=0;msg.type=Marker.TEXT_VIEW_FACING;msg.action=Marker.ADD
                msg.pose.position.x=0.;msg.pose.position.y=0.;msg.pose.position.z=3.;msg.pose.orientation.w=1.
                msg.scale.z=.4;msg.color.r=msg.color.g=msg.color.b=msg.color.a=1.
                msg.text='BT: '+bt.get('active_node','waiting_task')+'\n'+startup.get('state','waiting')
                marker.publish(msg)
                if session.get('replay_planning_graph'):
                    write_json(directory/'current_status.json',dict(
                        session_id=session['id'],played_seconds=play_elapsed,paused=paused,
                        startup=startup,bt=bt,navigation=last.get('navigation',{}),
                        planning=planning_observations,sdk_connected=False,motion_enabled=False))
            for name,p in children:
                if not name.startswith('rviz_') and p.poll() is not None: raise RuntimeError('component_exited:'+name)
            if startup.get('state')=='failed': raise RuntimeError('automatic_initialization_failed:'+startup.get('reason',''))
            if play_elapsed>=duration: break
        report.update(played_messages=played,played_seconds=play_elapsed,wall_seconds=time.monotonic()-started,
            startup_ready_at_bag_seconds=startup_ready_at,final_startup=last.get('startup',{}),
            final_bt=last.get('bt',{}),final_navigation=last.get('navigation',{}),events=events)
        times=[v[0]*1e-9 for v in pose_samples];gaps=[b-a for a,b in zip(times,times[1:])]
        report['continuous_pose']=dict(samples=len(times),source_hz=(len(times)-1)/(times[-1]-times[0]) if len(times)>1 else 0.,
            max_gap_s=max(gaps) if gaps else None,first=list(pose_samples[0]) if pose_samples else None,
            last=list(pose_samples[-1]) if pose_samples else None)
        records=[json.loads(line) for line in (directory/'events.jsonl').read_text().splitlines()]
        report['bt_initial_localization_verified']=any(e.get('node')=='InitialLocalization' and e.get('current')=='SUCCESS'
            for r in records if r['topic']=='bt' for e in r['value'].get('transitions',[]))
        report['initialization_passed']=bool(startup_ready_at is not None and len(times)>100
            and report['bt_initial_localization_verified'])
        state_gaps=[(b-a)*1e-9 for a,b in zip(atomic_samples,tuple(atomic_samples)[1:])]
        report['atomic_navigation_state']=dict(samples=len(atomic_samples),
            max_gap_s=max(state_gaps) if state_gaps else None)
        from d1max_localization.realtime_navigation_output import quantiles_ms
        receipt_gaps=[(b-a)*1e-9 for a,b in zip(atomic_receipts,tuple(atomic_receipts)[1:])]
        report['realtime']=dict(target_update_period_ms=20.,
            state_interval_ms=quantiles_ms(state_gaps),
            receipt_interval_ms=quantiles_ms(receipt_gaps),
            imu_source_age_ms=quantiles_ms(imu_ages),
            posterior_source_age_ms=quantiles_ms(posterior_ages),
            intervals_over_20ms=sum(g>.020 for g in state_gaps),
            intervals_over_40ms=sum(g>.040 for g in state_gaps),
            node=realtime,hard_realtime_verified=False,
            source_clock_physically_calibrated=False)
        local_gaps=[(b-a)*1e-9 for a,b in zip(local_samples,tuple(local_samples)[1:])]
        local_receipt_gaps=[(b-a)*1e-9 for a,b in zip(local_receipts,tuple(local_receipts)[1:])]
        report['continuous_local_state']=dict(samples=len(local_samples),
            state_interval_ms=quantiles_ms(local_gaps),receipt_interval_ms=quantiles_ms(local_receipt_gaps),
            unavailable=list(local_unavailable),max_gap_s=max(local_gaps) if local_gaps else None,
            independent_global_correction=True,hard_realtime_verified=False)
        report['native_lio']=dict(valid_samples=lio_valid_samples,invalid_notices=list(lio_notices))
        report['bt_input_timing']=bt_input_timing
        from tools.validation.recorded_input_timing import audit
        report['recorded_raw_input_timing']=audit(bag,duration,offset)
        after_ready_ns=first+offset+round(startup_ready_at*1e9) if startup_ready_at is not None else None
        # Preserve literal source and observer-receipt times. Aggregate maxima
        # cannot otherwise be assigned to initialization versus navigation.
        # This is diagnostic only: observer QoS cannot certify losslessness,
        # and unusable rows never fabricate a held pose or renew any lease.
        timing_file=directory/'original_state_timing.json'
        write_json(timing_file,dict(schema=1,after_ready_source_ns=after_ready_ns,
            source_headers_unchanged=True,observer_qos_depth=10,
            observer_losslessness_verified=False,queue_capacity=100000,
            overflow_counts={key:max(0,timing_counts[key]-len(rows)) for key,rows in
                (('local',local_timing_rows),('global',atomic_timing_rows))},
            local=list(local_timing_rows),global_state=list(atomic_timing_rows)))
        report['original_state_timing']=dict(path=str(timing_file),
            sha256=hashlib.sha256(timing_file.read_bytes()).hexdigest(),
            observer_losslessness_verified=False)
        after_ready_local=[s for s in local_samples if after_ready_ns is not None and s>=after_ready_ns]
        after_ready_gaps=[(b-a)*1e-9 for a,b in zip(after_ready_local,after_ready_local[1:])]
        report['continuous_local_state']['after_ready']=dict(
            boundary_source_ns=after_ready_ns,samples=len(after_ready_local),
            state_interval_ms=quantiles_ms(after_ready_gaps),
            scope='usable_original_sources_after_observed_initialization_not_hard_realtime_proof')
        report['continuous_unavailable_transitions']=sum(r['topic']=='navigation'
            and r['value'].get('valid') is False and after_ready_ns is not None
            and after_ready_ns <= r['virtual_ns'] < first+offset+round((duration-.5)*1e9)
            for r in records)
        report['continuous_stability_passed']=bool(report['initialization_passed']
            and len(atomic_samples)>100 and state_gaps and max(state_gaps)<=.08
            and report['continuous_unavailable_transitions']==0)
        report['navigation_execution_tested']=False
        report['local_planner_started']=bool(session.get('replay_planning_graph'))
        report['planning_observations']=planning_observations
        report['passed']=report['initialization_passed'] and report['continuous_stability_passed']
        if with_rviz:
            print('Playback finished. RViz holds the final view; close it or interrupt to clean up.',flush=True)
            write_json(directory/'report.json',report)
            while not stopping and any(name.startswith('rviz_') and p.poll() is None for name,p in children):
                rclpy.spin_once(node,timeout_sec=.05)
    except Exception as error:
        report.update(passed=False,error=type(error).__name__+':'+str(error),final_startup=last.get('startup',{}),final_bt=last.get('bt',{}))
        raise
    finally:
        try:
            if goal_handle is not None and goal_handle.accepted and rclpy.ok():
                goal_handle.cancel_goal_async()
                end=time.monotonic()+.5
                while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
        except Exception as error:
            report['cleanup_cancel_error']=type(error).__name__+':'+str(error)
        finally:
            # Child lifetime must not depend on cancel ACK or ROS teardown.
            # Kill only process groups created by this isolated supervisor.
            cleanup_errors=[]
            for name,p in reversed(children):
                try: stop_owned(p)
                except Exception as error: cleanup_errors.append(name+':'+str(error))
            report['cleanup_errors']=cleanup_errors
            try:
                write_json(directory/'report.json',report)
            finally:
                for stream in logs:stream.close()
                status_stream.close();db.close()
                node.destroy_node();rclpy.try_shutdown()


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--bag',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--seconds',type=float,default=90.)
    parser.add_argument('--rviz',action='store_true')
    parser.add_argument('--timing-backend',choices=('integrated','split'),default='integrated')
    parser.add_argument('--planning-graph',action='store_true',
        help='Also start native GridMap/SCAN, odom reference, unrouted tracker and both RViz layouts; never SDK/motion.')
    args=parser.parse_args()
    if not 10<=args.seconds<=400: raise ValueError('bounded_replay_seconds_required')
    root=Path(__file__).resolve().parents[2];directory=args.output.resolve()
    from d1max_pct_scan.single_floor_session import prepare, runtime_commands
    from d1max_pct_scan.isolated_zenoh import private_router
    session=prepare(directory,transport_mode='live',purpose='planning_only',expected_sdk_session='not-connected-recorded-input')
    session.update(input_source='isolated_recorded_rosbag',motion_control_enabled=False,
        replay_planning_graph=args.planning_graph,
        replay_timing_backend=args.timing_backend,
        replay_preview_goal=[.04922180175781232,3.295704650878907,-.5180169343948364])
    write_json(directory/'session.json',session)
    config=yaml.safe_load((directory/'localization.yaml').read_text())
    if args.timing_backend=='split':
        # Exact former timing profile for a same-input before/after run.
        config['navigation_estimation']['ros__parameters'].update(
            {'prediction.max_horizon':.25,'limits.max_prediction_horizon':.25})
    config['dual_lidar_adapter']['ros__parameters']['input_clock_mode']='recorded_sim_time'
    for value in config.values():
        if isinstance(value,dict) and 'ros__parameters' in value:value['ros__parameters']['use_sim_time']=True
    # navigation_parameters validates a known schema; use_sim_time is a ROS
    # boundary parameter, not a new predictor/estimator model parameter.
    config['navigation_estimation']['ros__parameters'].pop('use_sim_time',None)
    (directory/'localization.yaml').write_text(yaml.safe_dump(config))
    original=runtime_commands(directory,session)
    selected={'global','adapters','navigator','lifecycle'}
    if args.planning_graph:
        selected.update(('route_display_cache','reference','perception','scan','tracker'))
    commands={k:v for k,v in original.items() if k in selected}
    if args.planning_graph:
        for name,module in [('view','live_view'),('map_layers','live_map_layers'),('execution_view','execution_view')]:
            commands[name]=[sys.executable,'-m','d1max_pct_scan.'+module,'--session',str(directory),'--ros-args']
    commands['adapters']=[sys.executable,str(root/'tools/validation/replay_bt_input_observer.py'),
        '--ros-args','--params-file',str(directory/'bt_adapter.yaml')]
    for name in commands:
        if '--ros-args' not in commands[name]: commands[name].append('--ros-args')
        commands[name]+=['-p','use_sim_time:=true']
    from d1max_localization.estimation.configuration import navigation_parameters
    prediction,output,ekf=navigation_parameters(config)
    if args.timing_backend=='integrated':
        from d1max_localization.realtime_navigation_output import predictor_parameters
        output.update(predictor_parameters(prediction))
    for name,values in [('prediction',prediction),('output',output),('ekf_extra',ekf)]:
        values.update(use_sim_time=True)
        if name=='output': values['session_dir']=str(directory)
        (directory/(name+'.yaml')).write_text(yaml.safe_dump({'/**':{'ros__parameters':values}}))
    commands.update(adapter=[binary('d1max_localization','dual_lidar_adapter'),'--ros-args','--params-file',str(directory/'localization.yaml')],
        lio=[binary('faster_lio','run_mapping_online'),'--ros-args','--params-file',str(directory/'localization.yaml'),'-p','use_sim_time:=true',
             '-r','__ns:=/d1max/localization/lio','-r','__node:=laserMapping','-r','Odometry:=odometry','-r','cloud_registered_body:=deskewed'],
        matcher=[binary('d1max_localization','fused_icp_matcher'),'--ros-args','-r','__node:=lio_global_matcher','--params-file',str(directory/'localization.yaml'),'-p','map_pcd:='+session['map_pcd']],
        global_localizer=[sys.executable,str(root/'tools/validation/replay_global_localizer.py'),'--ros-args','--params-file',str(directory/'localization.yaml'),
            '-p','session_dir:='+str(directory),'-p','map_pcd:='+session['map_pcd'],'-p','navigation_output_enabled:=true'],
        predictor=[binary('d1max_localization','causal_lio_predictor'),'--ros-args','--params-file',str(directory/'prediction.yaml')],
        ekf=[binary('robot_localization','ekf_node'),'--ros-args','-r','__node:=ekf_navigation','--params-file',str(directory/'localization.yaml'),
            '--params-file',str(directory/'ekf_extra.yaml'),'-r','odometry/filtered:=/d1max/localization/estimator/odometry_raw','-r','set_pose:=/d1max/localization/estimator/set_pose'],
        output=[sys.executable,str(root/'tools/validation/replay_navigation_output.py'),
            '--ros-args','--params-file',str(directory/'output.yaml')])
    if args.timing_backend=='integrated': commands.pop('predictor')
    rviz=yaml.safe_load((root/'src/d1max_navigation/rviz/localization_test.rviz').read_text())
    displays=rviz['Visualization Manager']['Displays']
    displays[:]=[d for d in displays if d['Class'] in ('rviz_default_plugins/PointCloud2','rviz_default_plugins/Path')]
    for d in displays:
        if 'map_cloud' in d['Topic']['Value']: d.update(Enabled=True,Value=True,Alpha=.45)
    displays.extend([{'Class':'rviz_default_plugins/Axes','Name':'机器人位置姿态 · 坐标轴','Enabled':True,'Reference Frame':'d1max_loc_base_link','Length':.6,'Radius':.02},
        {'Class':'rviz_default_plugins/Marker','Name':'真实 BT 阶段','Enabled':True,'Topic':{'Value':'/d1max/replay/bt_stage','Reliability Policy':'Reliable','Durability Policy':'Volatile'}}])
    rviz['Visualization Manager']['Views']['Current'].update(Distance=22.,Pitch=.8,Yaw=.4,**{'Focal Point':{'X':0.,'Y':5.,'Z':0.}})
    (directory/'localization.rviz').write_text(yaml.safe_dump(rviz,allow_unicode=True))
    if args.planning_graph:
        for layout in ('global','local'):
            filename=directory/(layout+'_planning.rviz')
            config=yaml.safe_load(filename.read_text())
            config['Visualization Manager']['Displays'].append(displays[-1])
            if layout=='global':
                config['Visualization Manager']['Views']['Current'].update(Distance=24.,Pitch=.85,Yaw=.4,
                    **{'Focal Point':{'X':0.,'Y':5.,'Z':0.}})
            filename.write_text(yaml.safe_dump(config,allow_unicode=True))
    inventory=dict(commands=commands,xml_sha256=hashlib.sha256((directory/'navigation.xml').read_bytes()).hexdigest(),
        sdk_connected=False,motion_nodes=[],production_release_replaced=False,
        planning_graph=args.planning_graph,
        tracker_output='preview/demand_unrouted' if args.planning_graph else None)
    write_json(directory/'inventory.json',inventory)
    with private_router(directory/'zenoh') as env:
        env['D1MAX_REPLAY_INTEGRATED_OUTPUT']='1' if args.timing_backend=='integrated' else '0'
        replay(env,directory,session,commands,args.bag.resolve(),args.seconds,args.rviz)


if __name__=='__main__': main()
