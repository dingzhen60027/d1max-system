"""Full Nav2 managed lifecycle, with explicit isolated simulation/live profiles."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import uuid

import yaml
from ament_index_python.packages import get_package_share_directory
from .initial_pose import LocalWebClient
from .session import selected_map, map_view, port_open, api
from .motion_limits import configured_nav2, gate_parameters, validated_motion_limits

UNIT = 'd1max-navigation.service'
OWNER = 'D1MAX_NAVIGATION_V1:'
PREVIEW_UNIT = 'd1max-nav2-localization-test.service'
PREVIEW_OWNER = 'D1MAX_NAV2_LOCALIZATION_TEST_V1:'


def acquire_session_lock(lock, timeout=6.0):
    """Serialize CLI / Web requests without turning brief contention into failure."""
    deadline = time.monotonic()+timeout
    while True:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            return
        except BlockingIOError:
            remaining = deadline-time.monotonic()
            if remaining <= 0:
                raise RuntimeError('Navigation session busy for 6 seconds; no operation was executed') from None
            time.sleep(min(.05, remaining))


def validate_frame_contract(settings):
    # All three are shared by the existing localizer and the Nav2 plugin YAML.
    # Reject a partial rename instead of silently splitting the TF tree.
    for key, expected in (('map_frame', 'd1max_loc_map'),
                          ('odom_frame', 'd1max_loc_odom'),
                          ('base_frame', 'd1max_loc_base_link')):
        if settings.get(key) != expected:
            raise ValueError('Frame contract mismatch: ' + key + ' must be ' + expected)


def info(unit=UNIT):
    result = subprocess.run(['systemctl', '--user', 'show', unit,
        '--property=LoadState,ActiveState,SubState,MainPID,Description,ControlGroup'],
        capture_output=True, text=True, timeout=5)
    state = dict(line.split('=', 1) for line in result.stdout.splitlines() if '=' in line)
    absent = state.get('LoadState') == 'not-found' and state.get('ActiveState') == 'inactive'
    if not state or (result.returncode and not absent):
        raise RuntimeError(result.stderr or 'Cannot confirm service ownership')
    return state


def process_group_populated(state):
    group = state.get('ControlGroup', '')
    if group:
        path = (Path('/sys/fs/cgroup') / group.lstrip('/') / 'cgroup.events').resolve()
        if not path.is_relative_to('/sys/fs/cgroup'):
            raise RuntimeError('Invalid process group')
        if path.is_file() and 'populated 1' in path.read_text():
            return True
    return False


def alive(state):
    return (state.get('ActiveState') in {'active', 'activating', 'deactivating'}
            or process_group_populated(state))


def stop_owned(unit=UNIT, owner=OWNER):
    current = info(unit)
    if alive(current):
        if not current.get('Description', '').startswith(owner):
            raise RuntimeError('Refusing to stop unowned unit: ' + unit)
        subprocess.run(['systemctl', '--user', 'stop', unit], check=True, timeout=20)
        if alive(info(unit)):
            raise RuntimeError('Process group not empty: ' + unit)


def live_context(settings, version, enable_motion):
    if not port_open(7448):
        raise RuntimeError('No live Zenoh uplink. Connect the robot in Web first; simulation is a separate mode.')
    current = api(settings['web_url'], 'overview')
    monitor = current.get('connection', {})
    health = monitor.get('health', {})
    if not (monitor.get('active') and health.get('sdk_fresh') and health.get('lidar_fresh')
            and health.get('replay') is False):
        raise RuntimeError('Fresh real SDK and LiDAR data are required; no replay fallback')
    if current.get('phase') in {'idle', 'stopped', 'failed'}:
        current = api(settings['web_url'], 'start', {'version_id': version})
    if current.get('phase') not in {'running', 'starting'} or current.get('version_id') != version:
        raise RuntimeError('Localization/map conflict: stop or correct it in Web first')
    # A requested motion capability is not an arm and never overrides calibration.
    # The gate and same-session SDK adapter independently require explicit arming
    # and fresh validated state. Default false can never be armed at runtime.
    return current['id']


def configure_rviz(share, directory, settings, grid, manifest):
    rviz = yaml.safe_load((share / 'rviz/navigation.rviz').read_text())
    manager = rviz['Visualization Manager']
    manager['Global Options']['Fixed Frame'] = settings['map_frame']
    manager['Views']['Current'].update(map_view(grid, manifest, (1500, 1200)))
    for display in manager['Displays']:
        topic = display.get('Topic', {}).get('Value', '')
        if settings['mode'] == 'sim' and topic in {
                '/d1max/localization/lio/deskewed', '/d1max/localization/scan_initial_preview',
                '/d1max/localization/map_cloud', '/d1max/navigation/test_status'}:
            display['Enabled'] = display['Value'] = False
        if topic == '/d1max/navigation/simulation_status':
            display['Enabled'] = display['Value'] = settings['mode'] == 'sim'
    path = directory / ('FULL_NAV2_' + settings['mode'].upper() + '.rviz')
    path.write_text(yaml.safe_dump(rviz, sort_keys=False, allow_unicode=True))
    return str(path)


def start(args, app, nav, share, root):
    if alive(info()):
        raise RuntimeError('Navigation already running; use status/restart/stop. No duplicate started.')
    if not os.environ.get('DISPLAY') and not args.headless:
        raise RuntimeError('No desktop DISPLAY; use --headless for automated simulation only')
    web_owned = bool(getattr(args, 'web_owned', False))
    if web_owned and info('d1max-web-managed.service').get('ActiveState') != 'active':
        raise RuntimeError('Web-owned navigation requires the active managed Web service')
    settings = yaml.safe_load((share / 'config/navigation_runtime.yaml').read_text())
    validate_frame_contract(settings)
    # Fail before Web/localization or process changes, not after the robot is
    # connected. All command-producing layers get this same per-session profile.
    settings['motion_limits'] = validated_motion_limits(settings.get('motion_limits'))
    settings['command_gate_limits'] = gate_parameters(settings['motion_limits'])
    nav2_config = configured_nav2(yaml.safe_load((share / 'config/nav2.yaml').read_text()),
                                 settings['motion_limits'])
    LocalWebClient(settings['web_url'])
    version, folder, grid, manifest = selected_map(app)
    settings.update(mode=args.mode, version_id=version, map_name=manifest['name'],
        map_yaml=str(folder / 'map.yaml'), localization_pcd=str(folder / 'localization.pcd'),
        localization_session_id='', enable_motion=args.enable_motion,
        headless=args.headless, created_at=time.time(), navigation_session_id=uuid.uuid4().hex,
        web_owned=web_owned)
    if args.mode == 'sim' and args.enable_motion:
        raise ValueError('--enable-motion is hardware capability only, not valid in simulation')
    if args.mode == 'live':
        settings['localization_session_id'] = live_context(settings, version, args.enable_motion)
    if args.mode == 'sim' and port_open(7460):
        previous = info(PREVIEW_UNIT)
        if not (alive(previous) and previous.get('Description', '').startswith(PREVIEW_OWNER)):
            raise RuntimeError('Offline port 7460 occupied by unknown process; not adopted')
    # Previous map-only window is an owned diagnostic, not another navigation stack.
    stop_owned(PREVIEW_UNIT, PREVIEW_OWNER)
    if args.mode == 'sim':
        if port_open(7460):
            raise RuntimeError('Offline port 7460 occupied by unknown process; not adopted')
        session_cfg = share / 'config/zenoh-offline-session.json5'
        router_cfg = share / 'config/zenoh-offline-router.json5'
    else:
        session_cfg = app / 'foxglove_d1max/config/zenoh-live.json5'
        router_cfg = app / 'foxglove_d1max/config/zenoh-router-live.json5'
    center = map_view(grid, manifest)
    settings.update(status_anchor_x=center['X'], status_anchor_y=center['Y']+manifest['height']*grid['resolution']*.39)
    directory = root / (time.strftime('%Y%m%d_%H%M%S')+'_'+args.mode+'_'+str(os.getpid()))
    directory.mkdir()
    settings['rviz_config'] = configure_rviz(share, directory, settings, grid, manifest)
    nav2_snapshot = directory / 'nav2.yaml'
    nav2_snapshot.write_text(yaml.safe_dump(nav2_config, sort_keys=False, allow_unicode=True))
    settings['nav2_config'] = str(nav2_snapshot)
    collision_snapshot = directory / 'collision_monitor.yaml'
    collision_snapshot.write_text((share / 'config/collision_monitor.yaml').read_text())
    settings['collision_monitor_config'] = str(collision_snapshot)
    settings['bt_to_pose'] = str(share / 'behavior_trees/navigate_to_pose.xml')
    settings['bt_through_poses'] = str(share / 'behavior_trees/navigate_through_poses.xml')
    snapshot = directory / 'session.json'
    snapshot.write_text(json.dumps(settings, ensure_ascii=False, indent=2))
    (root / 'last_session.json').write_text(json.dumps(settings, ensure_ascii=False, indent=2))
    env = {k: os.environ[k] for k in ('PATH','LD_LIBRARY_PATH','PYTHONPATH','AMENT_PREFIX_PATH',
        'CMAKE_PREFIX_PATH','COLCON_PREFIX_PATH','DISPLAY','XAUTHORITY','XDG_RUNTIME_DIR','LANG') if k in os.environ}
    env.update(RMW_IMPLEMENTATION='rmw_zenoh_cpp', ROS_DOMAIN_ID='24',
        ZENOH_SESSION_CONFIG_URI=str(session_cfg), ZENOH_ROUTER_CONFIG_URI=str(router_cfg),
        QT_QPA_PLATFORM='xcb', ROS_LOG_DIR=str(directory / 'ros_logs'))
    command = ['systemd-run','--user','--collect','--unit='+UNIT,
        '--property=Description='+OWNER+args.mode+':'+version+':'+settings['navigation_session_id'],
        # Signal launch once; launch forwards SIGINT to its children. Mixed
        # still kills the entire cgroup after the timeout, without duplicate
        # concurrent SIGINT from both systemd and ROS launch during cleanup.
        '--property=Type=exec','--property=KillMode=mixed','--property=KillSignal=SIGINT',
        '--property=TimeoutStopSec=15','--property=SendSIGKILL=yes','--working-directory='+str(nav)]
    dependencies = ['d1max-web-managed.service'] if web_owned else []
    if args.mode == 'live':
        dependencies += ['d1max-localization-managed.service', 'd1max-monitor-managed.service']
    if dependencies:
        for prop in ('BindsTo','After'):
            command += ['--property='+prop+'='+' '.join(dependencies)]
    if web_owned:
        command += ['--property=PartOf=d1max-web-managed.service']
    command += ['--setenv='+k+'='+v for k,v in env.items()]
    command += ['ros2','launch','d1max_navigation','navigation.launch.py','session:='+str(snapshot)]
    subprocess.run(command,check=True,timeout=15)
    print(json.dumps({'started':args.mode,'map':manifest['name'],'session':str(snapshot),
        'navigation_session_id':settings['navigation_session_id'],
        'navigation_action':'/d1max/navigation/navigate_to_pose',
        'physical_motion_armed':False,'simulation':args.mode=='sim'},ensure_ascii=False,indent=2))


def main():
    parser=argparse.ArgumentParser(description='Full Nav2: isolated simulation or explicitly gated live navigation')
    parser.add_argument('action',nargs='?',default='start',choices=['start','stop','restart','status','command'])
    parser.add_argument('--mode',choices=['sim','live'],default='sim')
    parser.add_argument('--enable-motion',action='store_true',help='Live capability only; still requires explicit arm and verified calibration')
    parser.add_argument('--headless',action='store_true')
    parser.add_argument('--web-owned', action='store_true', help='Bind this navigation session to the managed Web service lifetime')
    parser.add_argument('--operation', choices=['status','arm','disarm','goal','cancel'])
    parser.add_argument('--session-id', default='')
    parser.add_argument('--version-id', default='')
    parser.add_argument('--x', type=float)
    parser.add_argument('--y', type=float)
    parser.add_argument('--yaw', type=float)
    args=parser.parse_args()
    if args.action == 'command' and not args.operation:
        parser.error('command requires --operation status|arm|disarm|goal|cancel')
    nav=Path(os.environ['D1MAX_NAV_ROOT']); app=Path(os.environ['D1MAX_APP_ROOT'])
    root=nav/'log/navigation'; root.mkdir(parents=True,exist_ok=True)
    try:
        with (root/'.lock').open('a') as lock:
            acquire_session_lock(lock)
            if args.action=='command':
                from .navigation_commands import execute
                execute(args, app, nav, root)
                return
            if args.action=='status':
                previous=root/'last_session.json'
                current = info()
                print(json.dumps({'unit':current,'process_group_populated':process_group_populated(current),
                    'last_session':json.loads(previous.read_text()) if previous.exists() else None},ensure_ascii=False,indent=2)); return
            if args.action in {'stop','restart'}:
                stop_owned()
                print('Owned Nav2 process group stopped; shared SDK/Web/localization unchanged.')
            if args.action in {'start','restart'}:
                start(args,app,nav,Path(get_package_share_directory('d1max_navigation')),root)
    except (OSError,ValueError,RuntimeError,subprocess.SubprocessError) as error:
        print('Navigation: '+str(error),file=sys.stderr); sys.exit(1)
