"""Owned live planning; preview default, explicit motion stack opt-in.

This supervisor never creates an SDK connection or enables the robot.
"""
import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid
from urllib.request import build_opener, ProxyHandler, Request

import yaml
from .live_runtime import child_failure_reason, file_sha256, stop_owned_children
from .live_visualization import configure as configure_rviz
from .robot_profile import load_robot_profile, scan_robot_parameters
from .live_view_reload import (VERSION as UI_RELOAD_VERSION, REQUEST_TTL,
    REQUEST_FILE, RESULT_FILE, owned_directory, preview_reload_session,
    read_owned_json, write_owned_json, write_owned_text, verify_runtime_owner, validate_request, verified_result,
    replace_ui_children, RvizClosedDuringReload, token as reload_token)

WS = Path('/home/dndx/d1max_nav_ws')
APP = Path('/home/dndx/智元四足机器人D1 Max二次开发文档资料包v0.1.0/d1max_ros2')
ROOT = WS / 'log/live_planning'
START_LOCK = WS / 'log/.localization-start.lock'
UNIT = 'd1max-live-planning-view.service'
OWNER = 'D1MAX-LIVE-PLANNING-VIEW:'
ACTIVE = {'active', 'activating', 'deactivating', 'reloading'}
DEFAULT_CONFIG = WS / 'src/d1max_pct_scan/config/live_visualization.yaml'


def load_config(path):
    cfg = yaml.safe_load(Path(path).read_text())
    if (not isinstance(cfg, dict) or cfg.get('mode') != 'LIVE_VISUALIZATION_NO_MOTION'
            or cfg.get('motion_control_enabled') is not False):
        raise ValueError('Live visualization must explicitly forbid motion')
    if cfg.get('frame_id') != 'd1max_loc_map':
        raise ValueError('Do not relabel conditioned PCD as a localization map')
    cfg.setdefault('perception_backend', 'deskewed_cloud')
    if cfg['perception_backend'] not in ('deskewed_cloud', 'per_sensor_rays'):
        raise ValueError('Unknown local perception backend')
    for name in ('map_pcd', 'planning_manifest', 'tomogram_npz', 'crossfloor_route_config',
                 'localization_config', 'scan_config', 'robot_profile'):
        if not Path(cfg[name]).is_absolute() or not Path(cfg[name]).is_file():
            raise ValueError('Missing absolute input: ' + name)
    for name in ('current_floor', 'goal_floor'):
        if cfg[name] not in ('floor1', 'floor2'):
            raise ValueError('Only explicit floor1/floor2 are supported at endpoints')
    for name, low, high in (
        ('initial_body_z', -10, 10), ('body_height', .1, 1.),
        ('body_height_min_m', .1, .8), ('body_height_max_m', .2, 1.2),
        ('max_start_move_m', .01, .3), ('result_timeout_s', 5, 60),
        ('freshness_s', .1, .5), ('scan_preview_speed_mps', .01, .3),
        ('scan_preview_acc_mps2', .01, .35),
        ('scan_local_horizon_m', .5, 4.),
        ('ground_support_height_tolerance_m', .01, .3),
        ('ground_support_max_step_m', .01, .25),
    ):
        value = cfg.get(name)
        if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
            raise ValueError('Invalid visualization limit: ' + name)
    if cfg['body_height_min_m'] >= cfg['body_height_max_m']:
        raise ValueError('Invalid body height interval')
    robot = load_robot_profile(cfg['robot_profile'])
    engineering = robot['engineering']
    if (abs(cfg['body_height']-engineering['body_reference_height_m']) > 1e-9
            or cfg['scan_preview_speed_mps'] > engineering['preview_speed_mps']
            or cfg['scan_preview_acc_mps2'] > engineering['preview_acceleration_mps2']):
        raise ValueError('Live planning settings disagree with the D1 Max robot profile')
    cfg['robot_profile_snapshot'] = robot
    manifest = json.loads(Path(cfg['planning_manifest']).read_text())
    if (Path(manifest['source_path']).resolve() != Path(cfg['map_pcd']).resolve()
            or file_sha256(cfg['map_pcd']) != manifest['source_sha256']):
        raise ValueError('Localization must use the exact original coordinates/source of this PCT derivative')
    return cfg


def scan_parameters(cfg, session_id):
    params = next(iter(yaml.safe_load(Path(cfg['scan_config']).read_text()).values()))['ros__parameters']
    profile = cfg.get('robot_profile_snapshot') or load_robot_profile(cfg['robot_profile'])
    if profile['engineering']['ground_exclusion_height_m'] <= params['grid_map.resolution']:
        raise ValueError('Ground exclusion must exceed the actual voxel height')
    params.update(scan_robot_parameters(profile))
    params.update({
        'use_sim_time': False, 'fsm.navi_mode': 3,
        'fsm.require_tagged_reference': True, 'fsm.navigation_session_id': session_id,
        'fsm.strict_input_frames': True, 'fsm.odom_twist_in_body_frame': True,
        'fsm.odom_timeout': .5, 'fsm.max_replan_interval': 1.,
        'fsm.reference_goal_xy_tolerance': .20, 'fsm.reference_goal_z_tolerance': .15,
        'fsm.reference_path_guidance': True, 'fsm.reference_path_z_offset': cfg['body_height'],
        'fsm.reference_start_tolerance': 1., 'grid_map.frame_id': cfg['frame_id'],
        'grid_map.sliding_map_frame_id': 'd1max_live_scan_' + session_id[:8],
        'grid_map.strict_input_frames': True, 'grid_map.maximum_cloud_pose_dt': .25,
        'grid_map.require_observed_free': True,
        'grid_map.require_localization_context': True,
        'grid_map.localization_session_id': session_id,
        'grid_map.visualization_rate_hz': 3.0,
        'grid_map.cloud_is_world': True, 'grid_map.need_extrinsic': False,
        'grid_map.use_projected_rays': cfg.get('perception_backend') == 'per_sensor_rays',
        'manager.max_vel': cfg['scan_preview_speed_mps'],
        'optimization.max_vel': cfg['scan_preview_speed_mps'],
        'manager.max_acc': cfg['scan_preview_acc_mps2'],
        'optimization.max_acc': cfg['scan_preview_acc_mps2'],
        'fsm.planning_horizon': cfg['scan_local_horizon_m'],
        'manager.planning_horizon': cfg['scan_local_horizon_m'],
    })
    return params


def prepare_ground_support(cfg, directory):
    """Build once per session; no PCD/tomogram reads in the spline callback."""
    from .live_map_geometry import build_layers
    from .pct_ground_support import write_support_index
    original, _costs, _blocked, metadata = build_layers(cfg)
    manifest = json.loads(Path(cfg['planning_manifest']).read_text())
    metadata['source_pcd_sha256'] = manifest['source_sha256']
    path = Path(directory) / 'pct_ground_support.npz'
    digest = write_support_index(path, original, metadata)
    return dict(ground_support_index=str(path), ground_support_sha256=digest,
                ground_support_source_pcd_sha256=metadata['source_pcd_sha256'],
                ground_support_tomogram_sha256=metadata['source_tomogram_sha256'])


def bridge_parameters(cfg, session_id):
    result = {name: cfg[name] for name in (
        'ground_support_index', 'ground_support_sha256',
        'ground_support_source_pcd_sha256', 'ground_support_tomogram_sha256',
        'ground_support_height_tolerance_m', 'ground_support_max_step_m')}
    return dict(result, session_id=session_id, localization_session_id=session_id,
                map_frame=cfg['frame_id'], body_height=cfg['body_height'],
                perception_backend=cfg.get('perception_backend', 'deskewed_cloud'))


def prepare(path=DEFAULT_CONFIG):
    cfg = load_config(path)
    identifier = uuid.uuid4().hex
    directory = ROOT / (time.strftime('%Y%m%d_%H%M%S') + '_' + identifier[:8])
    directory.mkdir(mode=0o700, parents=True)
    session = {**cfg, 'id': identifier, 'version_id': 'sc_pgo_20260923_original_coordinates',
        'created_at': time.time(), 'floor': cfg['current_floor'], 'config_source': str(Path(path).resolve()),
        'ui_reload_supported': UI_RELOAD_VERSION, 'preview_freeze_owner': 'supervisor_child'}
    session.update(prepare_ground_support(session, directory))
    write_owned_json(directory / 'session.json', session)
    config = yaml.safe_load(Path(cfg['localization_config']).read_text())
    if cfg['perception_backend'] == 'per_sensor_rays':
        config['dual_lidar_adapter']['ros__parameters']['perception_rays.enabled'] = True
    (directory / 'localization.yaml').write_text(yaml.safe_dump(config))
    (directory / 'scan.yaml').write_text(yaml.safe_dump({'/**': {'ros__parameters': scan_parameters(cfg, identifier)}}))
    global_params = {k: cfg[k] for k in ('planning_manifest', 'tomogram_npz',
        'crossfloor_route_config', 'current_floor', 'goal_floor', 'body_height_min_m',
        'body_height_max_m', 'max_start_move_m', 'result_timeout_s', 'freshness_s')}
    global_params.update(session_id=identifier, output_directory=str(directory))
    (directory / 'global.yaml').write_text(yaml.safe_dump({'/**': {'ros__parameters': global_params}}))
    (directory / 'bridge.yaml').write_text(yaml.safe_dump(
        {'/**': {'ros__parameters': bridge_parameters(session, identifier)}}))
    write_view_configs(directory, identifier)
    return directory, session


def prepare_motion(path=DEFAULT_CONFIG):
    """Offline preparation only. Existing calibration/profile flags stay intact."""
    from .motion_stack import execution_config, parameters
    if load_config(path).get('perception_backend') == 'per_sensor_rays':
        raise ValueError('Per-sensor ray integration is preview-only until physical validation')
    directory, session = prepare(path)
    session.update(mode='LIVE_NAVIGATION', motion_control_enabled=True,
                   ui_reload_supported=0, preview_freeze_owner='motion_coordinator')
    session['motion'] = execution_config(session)
    write_owned_json(directory / 'session.json', session)
    bridge = bridge_parameters(session, session['id'])
    bridge.update(execution_mode='execution',
                  execution_tracker_node='/d1max/live_planning/motion_coordinator')
    (directory / 'bridge.yaml').write_text(yaml.safe_dump({'/**': {'ros__parameters': bridge}}))
    (directory / 'motion.yaml').write_text(yaml.safe_dump(parameters(session, WS)))
    write_view_configs(directory, session['id'], motion=True)
    return directory, session


def unit(name=UNIT):
    value = subprocess.run(['systemctl', '--user', 'show', name,
        '--property=ActiveState,Description,MainPID,ControlGroup,InvocationID'], capture_output=True,
        text=True, timeout=5)
    state = dict(line.split('=', 1) for line in value.stdout.splitlines() if '=' in line)
    if state.get('ActiveState') not in ACTIVE | {'inactive', 'failed'}:
        raise RuntimeError('Cannot verify service state: ' + name)
    return state


def unit_busy(state):
    if state.get('ActiveState') in ACTIVE:
        return True
    group = state.get('ControlGroup', '')
    if not group:
        return False
    root = Path('/sys/fs/cgroup').resolve()
    path = (root / group.lstrip('/')).resolve()
    if not path.is_relative_to(root):
        raise RuntimeError('Invalid owned service cgroup')
    try:
        return 'populated 1' in (path / 'cgroup.events').read_text()
    except FileNotFoundError:
        return False


def stop():
    state = unit()
    if not unit_busy(state):
        return
    if not state.get('Description', '').startswith(OWNER):
        raise RuntimeError('Unowned unit; will not stop it')
    subprocess.run(['systemctl', '--user', 'stop', UNIT], check=True, timeout=20)
    if unit_busy(unit()):
        raise RuntimeError('Owned session cleanup is not complete; do not restart')


def view_config(session_id='', *, motion=False, layout='global'):
    cfg = yaml.safe_load((WS / 'src/d1max_navigation/rviz/localization_test.rviz').read_text())
    result = configure_rviz(cfg, session_id=session_id, layout=layout)
    if motion:
        result['Panels'][0]['Motion Capable'] = True
        result['Panels'].insert(0, {'Class': 'd1max_pct_rviz_tools/MotionControlPanel',
                                   'Name': '运动控制', 'Session ID': session_id})
    return result


def write_view_configs(directory, session_id, *, motion=False):
    """Generate two presentation-only presets; no estimator or planner restart.

    live.rviz remains the stable startup filename for existing supervisors.
    Both standalone presets bind to the same session/topics and ROS frames.
    """
    rendered = {layout: yaml.safe_dump(view_config(session_id, motion=motion, layout=layout),
                                     allow_unicode=True, sort_keys=False)
                for layout in ('global', 'local')}
    for layout, contents in rendered.items():
        write_owned_text(Path(directory)/(layout+'_planning.rviz'), contents)
    write_owned_text(Path(directory)/'live.rviz', rendered['global'])


def export_views(directory=None):
    """Refresh only owned preview layout files, without touching ROS/services."""
    if directory is None:
        directory = read_owned_json(ROOT / 'last_session.json')['directory']
    directory = owned_directory(directory, ROOT)
    session = read_owned_json(directory / 'session.json')
    preview_reload_session(session)
    write_view_configs(directory, session['id'])
    return {'session_id': session['id'], 'started_processes': False,
            'layouts': {layout: str(directory/(layout+'_planning.rviz'))
                        for layout in ('global', 'local')}}


def reload_view(directory=None):
    """Request a versioned owned preview UI reload; never start/stop a service."""
    if directory is None:
        directory = read_owned_json(ROOT / 'last_session.json')['directory']
    directory = owned_directory(directory, ROOT)
    session = read_owned_json(directory / 'session.json')
    state = unit()
    runtime = read_owned_json(directory / 'runtime_status.json')
    capability = verify_runtime_owner(session, runtime, state, owner_prefix=OWNER, now=time.time())
    request_id, issued = uuid.uuid4().hex, time.time()
    request = dict(schema=UI_RELOAD_VERSION, action='reload-view', session_id=session['id'],
        owner_nonce=capability['owner_nonce'], request_id=request_id,
        issued_at=issued, expires_at=issued+REQUEST_TTL)
    write_owned_json(directory / REQUEST_FILE, request)
    deadline = time.monotonic() + 12.
    while time.monotonic() < deadline:
        try:
            result = read_owned_json(directory / RESULT_FILE)
        except FileNotFoundError:
            result = {}
        result = verified_result(result, request, owner_pid=capability['owner_pid'],
                                 invocation_id=capability['invocation_id'], now=time.time())
        if result is not None:
            current = unit()
            if (current.get('ActiveState') != 'active' or current.get('Description') != OWNER+session['id']
                    or current.get('MainPID') != state.get('MainPID')
                    or current.get('InvocationID') != state.get('InvocationID')):
                raise RuntimeError('Preview supervisor changed while UI reload was in progress')
            if result.get('status') == 'reloaded':
                return result
            if result.get('status') == 'failed':
                raise RuntimeError('UI reload failed: ' + str(result.get('error', 'unknown failure')))
        time.sleep(.1)
    raise RuntimeError('UI reload acknowledgement timed out; inspect ui_reload_result.json, do not assume success')


def start(config_path=DEFAULT_CONFIG, *, motion=False):
    # The Web localization entry holds this same lease through systemd startup.
    # Serializing the check+start prevents two concurrent TF authorities.
    START_LOCK.parent.mkdir(parents=True, exist_ok=True)
    with START_LOCK.open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise RuntimeError('Another localization entry is starting; retry after it finishes') from error
        return _start_locked(config_path, motion=True) if motion else _start_locked(config_path)


def check_localization_imports():
    # Use this launcher's deployment environment, not a source-tree test path.
    # Importing entry modules does not call main()/rclpy.init or create nodes.
    from importlib import import_module
    for module in ('lio_localizer', 'lio_predictor', 'navigation_output'):
        try:
            import_module('d1max_localization.'+module)
        except ModuleNotFoundError as error:
            raise RuntimeError('定位程序安装不完整：缺少模块 '+str(error.name)+
                               '；请重新构建 d1max_localization') from None


def _start_locked(config_path, *, motion=False):
    if unit_busy(unit()):
        raise RuntimeError('Live view already running; no duplicate started')
    for name in ('d1max-pct-preview.service', 'd1max-pct-scan.service',
                 'd1max-nav2-localization-test.service'):
        if unit_busy(unit(name)):
            raise RuntimeError('Stop the other planning session first: ' + name)
    if unit_busy(unit('d1max-localization-managed.service')):
        raise RuntimeError('Web localization already running; do not start a duplicate')
    if subprocess.run(['pgrep', '-f', '/lib/d1max_localization/lio_localizer'],
                      stdout=subprocess.DEVNULL).returncode == 0:
        raise RuntimeError('Existing localization process; refusing a second TF authority')
    check_localization_imports()
    # Read the local connection manager directly. Do not make a recursive HTTP
    # request to the Web server which may be waiting for this launcher.
    credential = APP / 'foxglove_d1max/config/manager.local.json'
    if credential.stat().st_mode & 0o077:
        raise RuntimeError('Connection manager credential must remain private')
    token = json.loads(credential.read_text())['token']
    request = Request('http://127.0.0.1:8771/v1/status',
                      headers={'Authorization': 'Bearer ' + token})
    with build_opener(ProxyHandler({})).open(request, timeout=5) as reply:
        connection = json.load(reply)['monitor']
    health = connection.get('health', {})
    if not (connection.get('active') and health.get('lidar_fresh') and health.get('sdk_fresh')
            and health.get('replay') is False):
        raise RuntimeError('Fresh real sensors required; this launcher never connects/takes over SDK')
    if not os.environ.get('DISPLAY'):
        raise RuntimeError('Missing desktop or the verified 09-23 source map')
    directory, session = prepare_motion(config_path) if motion else prepare(config_path)
    identifier = session['id']
    environment = {k: os.environ[k] for k in ('PATH', 'LD_LIBRARY_PATH', 'PYTHONPATH',
        'AMENT_PREFIX_PATH', 'CMAKE_PREFIX_PATH', 'COLCON_PREFIX_PATH', 'DISPLAY',
        'XAUTHORITY', 'XDG_RUNTIME_DIR', 'LANG', 'DBUS_SESSION_BUS_ADDRESS',
        'D1MAX_PCT_COMPUTE_CONFIG') if k in os.environ}
    environment.update(RMW_IMPLEMENTATION='rmw_zenoh_cpp', ROS_DOMAIN_ID='24',
        ZENOH_SESSION_CONFIG_URI=str(APP / 'foxglove_d1max/config/zenoh-live.json5'),
        QT_QPA_PLATFORM='xcb', OMP_NUM_THREADS='4', OPENBLAS_NUM_THREADS='2',
        ROS_LOG_DIR=str(directory / 'ros_logs'))
    command = ['systemd-run', '--user', '--collect', '--unit=' + UNIT,
        '--property=Description=' + OWNER + identifier, '--property=Type=exec',
        '--property=KillMode=control-group', '--property=KillSignal=SIGINT',
        '--property=TimeoutStopSec=12', '--property=SendSIGKILL=yes',
        '--property=BindsTo=d1max-monitor-managed.service d1max-web-managed.service',
        '--property=PartOf=d1max-web-managed.service',
        '--property=After=d1max-monitor-managed.service d1max-web-managed.service',
        '--working-directory=' + str(WS)]
    command += ['--setenv=' + k + '=' + v for k, v in environment.items()]
    command += [sys.executable, '-m', 'd1max_pct_scan.live_session', 'run', '--session', str(directory)]
    subprocess.run(command, check=True, timeout=10)
    write_owned_json(ROOT / 'last_session.json', {'directory': str(directory), **session})
    print(json.dumps({'directory': str(directory), **session}, ensure_ascii=False))


def run(directory):
    # Internal systemd entry, never an alternate public launcher bypassing the
    # startup lease, sensor checks and process ownership checks.
    state = unit()
    if (not os.environ.get('INVOCATION_ID')
            or state.get('MainPID') != str(os.getpid())):
        raise RuntimeError('run is an internal owned-systemd entry; use start')
    s = json.loads((directory / 'session.json').read_text())
    if state.get('Description') != OWNER + s['id']:
        raise RuntimeError('Session does not belong to this systemd unit')
    if unit_busy(unit('d1max-localization-managed.service')):
        raise RuntimeError('Conflicting Web localization; refusing duplicate TF authority')
    motion = s.get('mode') == 'LIVE_NAVIGATION' and s.get('motion_control_enabled') is True
    if not motion and (s['mode'] != 'LIVE_VISUALIZATION_NO_MOTION'
                       or s['motion_control_enabled'] is not False):
        raise ValueError('Invalid live session capability')
    if motion:
        from .motion_execution import MotionConfig
        MotionConfig(session_id=s['id'], map_version_id=s['version_id'], **s['motion'])
    # Environment overrides may not reconnect this process to a replay/private graph.
    os.environ.pop('ZENOH_CONFIG_OVERRIDE', None)
    os.environ.pop('ZENOH_SESSION_CONFIG', None)
    if (os.environ.get('RMW_IMPLEMENTATION') != 'rmw_zenoh_cpp'
            or os.environ.get('ZENOH_SESSION_CONFIG_URI') != str(APP / 'foxglove_d1max/config/zenoh-live.json5')):
        raise ValueError('Live visualization requires the configured live Zenoh graph')
    ui_supported = not motion and s.get('ui_reload_supported') == UI_RELOAD_VERSION
    if ui_supported:
        preview_reload_session(s)
        owned_directory(directory, ROOT)
        if not reload_token(os.environ.get('INVOCATION_ID')):
            raise ValueError('UI reload requires a verified systemd invocation identity')
    ui_state, ui_error, ui_nonce, last_request_id = 'running', '', uuid.uuid4().hex, None
    ignored_ui_pids = set()
    children, streams = [], {}
    terminal_error = None
    exit_reason = 'stop_requested'
    started_at = time.time()
    def record_runtime(phase, **extra):
        try:
            write_owned_json(directory / 'runtime_status.json', {
                'session_id': s['id'], 'received_at_unix': time.time(),
                'started_at': started_at, 'phase': phase,
                'mode': s['mode'], 'motion_enabled': False, 'motion_capable': motion,
                'ui_reload_supported': UI_RELOAD_VERSION if ui_supported else 0,
                'ui_reload': dict(owner_pid=os.getpid(), invocation_id=os.environ.get('INVOCATION_ID'),
                    owner_nonce=ui_nonce, state=ui_state, error=ui_error) if ui_supported else None,
                'error': terminal_error, 'reason': exit_reason,
                'children': [{'name': name, 'pid': p.pid, 'returncode': p.poll()}
                             for name, p in children], **extra})
        except OSError:
            pass  # Disk errors must never prevent owned-process cleanup.
    stopping = False
    def handler(*_):
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGINT, handler)
    signal.signal(signal.SIGTERM, handler)
    def launch_child(name, args):
        previous = streams.pop(name, None)
        if previous is not None:
            previous.close()
        stream = (directory / (name + '.log')).open('a')
        streams[name] = stream
        return subprocess.Popen(args, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
    ui_commands = {
        'view': [sys.executable, '-m', 'd1max_pct_scan.live_view', '--session', str(directory)],
        'rviz': ['rviz2', '-d', str(directory / 'live.rviz'), '--ros-args', '-r', '__node:=d1max_live_view_rviz'],
    }
    def spawn(name, args):
        children.append((name, launch_child(name, args)))
    def configure_ui():
        write_view_configs(directory, s['id'])
    def reply_ui_reload(result):
        nonlocal ui_error
        try:
            write_owned_json(directory / RESULT_FILE, result)
        except OSError as error:
            # A UI mailbox disk error must not kill localization or its goal.
            ui_error = 'UI reload acknowledgement unavailable: ' + str(error)[:300]
    def poll_ui_reload():
        nonlocal ui_state, ui_error, ui_nonce, last_request_id, stopping, exit_reason
        if not ui_supported:
            return
        try:
            request = read_owned_json(directory / REQUEST_FILE, maximum_bytes=4096)
        except FileNotFoundError:
            return
        except (OSError, ValueError):
            return  # An invalid/unowned mailbox is never an execution request.
        request_id = request.get('request_id')
        if not reload_token(request_id) or request_id == last_request_id:
            return
        last_request_id = request_id
        result = dict(schema=UI_RELOAD_VERSION, session_id=s['id'], request_id=request_id,
                      status='failed', motion_enabled=False, owner_nonce=request.get('owner_nonce'),
                      owner_pid=os.getpid(), invocation_id=os.environ.get('INVOCATION_ID'))
        try:
            validate_request(request, session_id=s['id'], owner_nonce=ui_nonce, now=time.time())
        except ValueError as error:
            result.update(error=str(error), completed_at=time.time())
            reply_ui_reload(result)
            return
        # Consume this launch-scoped challenge before any effects. Even a copied
        # request with a different request_id cannot replay a completed reload.
        ui_nonce, ui_state, ui_error = uuid.uuid4().hex, 'reloading', ''
        record_runtime('running')
        try:
            details = replace_ui_children(children, stop_children=stop_owned_children,
                spawn=lambda name: launch_child(name, ui_commands[name]), configure=configure_ui,
                stopping=lambda: stopping, terminated_ui_pids=ignored_ui_pids)
            ignored_ui_pids.clear()
            ui_state = 'running'
            result.update(status='reloaded', **details,
                ui_state_reset=['unsubmitted_goal_editor', 'rviz_camera'],
                goal_resubmitted=False, initial_pose_resubmitted=False)
        except RvizClosedDuringReload as error:
            ui_state, ui_error, exit_reason, stopping = 'failed', str(error), 'rviz_closed', True
            result['error'] = ui_error
        except Exception as error:
            ui_state, ui_error = 'failed', str(error)[:500]
            result['error'] = ui_error
        ignored_ui_pids.intersection_update(child.pid for _, child in children)
        result['completed_at'] = time.time()
        reply_ui_reload(result)
        record_runtime('running')
    try:
        record_runtime('starting')
        spawn('localization', ['ros2', 'launch', 'd1max_localization', 'localization.launch.py',
            'config:=' + str(directory / 'localization.yaml'), 'session_dir:=' + str(directory),
            'map_pcd:=' + s['map_pcd']])
        if ui_supported:
            spawn('preview_freeze', [sys.executable, '-m', 'd1max_pct_scan.preview_freeze',
                                     '--session', str(directory)])
        spawn('view', ui_commands['view'])
        spawn('map_layers', [sys.executable, '-m', 'd1max_pct_scan.live_map_layers', '--session', str(directory)])
        spawn('global', [sys.executable, '-m', 'd1max_pct_scan.live_global_planner',
            '--ros-args', '--params-file', str(directory / 'global.yaml')])
        spawn('bridge', [sys.executable, '-m', 'd1max_pct_scan.live_scan_bridge',
            '--ros-args', '--params-file', str(directory / 'bridge.yaml')])
        if s.get('perception_backend') == 'per_sensor_rays':
            if motion:
                raise ValueError('Per-sensor ray integration is not motion-authorized')
            spawn('perception', [sys.executable, '-m', 'd1max_pct_scan.perception_ray_projector',
                                  '--session', str(directory)])
        if motion:
            spawn('motion_stack', ['ros2', 'launch', 'd1max_pct_scan', 'motion.launch.py',
                                   'session:=' + str(directory)])
        from ament_index_python.packages import get_package_prefix
        scan = Path(get_package_prefix('scan_planner')) / 'lib/scan_planner/scan_planner_node'
        remaps = {
            'body_pose': '/d1max/live_planning/body_pose',
            'sensor_pose': '/d1max/live_planning/sensor_pose',
            'cloud': '/d1max/live_planning/cloud_map',
            'projected_rays': '/d1max/live_planning/rays_map',
            'grid_map/projected_rays_status': '/d1max/live_planning/rays_status',
            'typed_initial_path': '/d1max/live_planning/scan_reference',
            'planning/go2_execution_frozen': '/d1max/live_planning/execution_frozen',
            'grid_map/localization_context': '/d1max/live_planning/scan_map_context',
            'grid_map/localization_context_ack': '/d1max/live_planning/scan_map_context_ack',
            'planning/tagged_bspline': '/d1max/live_planning/scan_tagged_bspline',
            'planning/local_plan_debug': '/d1max/live_planning/native_local_debug',
            'planning/local_attempt_debug': '/d1max/live_planning/native_local_attempt_debug',
        }
        scan_command = [str(scan), '--ros-args', '-r', '__ns:=/d1max/live_planning/scan',
            '-r', '__node:=scan_planner_node', '--params-file', str(directory / 'scan.yaml')]
        for source, target in remaps.items():
            scan_command += ['-r', source + ':=' + target]
        spawn('scan', scan_command)
        spawn('rviz', ui_commands['rviz'])
        record_runtime('running')
        last_runtime = time.monotonic()
        while not stopping:
            # A failed explicit UI replacement must not destroy localization or
            # the user's goal. A normal RViz close still stops the whole preview.
            exited = next(((name, p.returncode) for name, p in children
                if p.pid not in ignored_ui_pids and p.poll() is not None), None)
            if exited:
                name, code = exited
                if name == 'rviz' and code == 0:
                    exit_reason = 'rviz_closed'
                    break
                raise RuntimeError(child_failure_reason(directory, name, code))
            poll_ui_reload()
            if time.monotonic()-last_runtime >= 1.:
                record_runtime('running')
                last_runtime = time.monotonic()
            time.sleep(.2)
    except BaseException as error:
        terminal_error = str(error)[:500] or type(error).__name__
        exit_reason = 'child_or_startup_failure'
        raise
    finally:
        record_runtime('stopping')
        cleanup = stop_owned_children(children)
        record_runtime('failed' if terminal_error else 'stopped', cleanup=cleanup)
        for stream in streams.values():
            stream.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('action', choices=['prepare', 'prepare-motion', 'start', 'start-motion',
                                         'stop', 'status', 'run', 'reload-view', 'export-views'])
    parser.add_argument('--session', type=Path)
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    args = parser.parse_args()
    ROOT.mkdir(parents=True, exist_ok=True)
    if args.action == 'run':
        run(args.session)
        return
    with (ROOT / '.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if args.action in ('start', 'start-motion'):
            start(args.config, motion=args.action == 'start-motion')
        elif args.action in ('prepare', 'prepare-motion'):
            directory, session = prepare_motion(args.config) if args.action == 'prepare-motion' else prepare(args.config)
            print(json.dumps({'directory': str(directory), 'prepared_only': True,
                'connected': False, 'started_processes': False, 'session_id': session['id']}, ensure_ascii=False))
        elif args.action == 'stop':
            stop()
        elif args.action == 'reload-view':
            print(json.dumps(reload_view(args.session), ensure_ascii=False))
        elif args.action == 'export-views':
            print(json.dumps(export_views(args.session), ensure_ascii=False))
        else:
            print(json.dumps(unit(), ensure_ascii=False))


if __name__ == '__main__':
    main()
