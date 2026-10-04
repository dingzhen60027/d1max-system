"""Owned planning-only RViz workbench; private loopback Zenoh, no controls."""
import argparse
from datetime import datetime
import fcntl
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
import uuid

import yaml

from .paths import expand_tree, nav_root

WS = nav_root()
SOURCE = WS / 'src/d1max_pct_planner'
ROOT = WS / 'log/pct_preview'
UNIT = 'd1max-pct-preview.service'
OWNER = 'D1MAX-PCT-PREVIEW:'
PORT = 7465
ACTIVE_STATES = ('active', 'activating', 'deactivating', 'reloading')
PARAMETERS = (
    'map_backend', 'tomogram_path', 'minimum_headroom_m', 'unknown_ceiling_policy',
    'selection_mode', 'placement_anchor_z', 'max_follow_height_change_m',
    'planning_grid', 'map_pcd', 'vendor_root', 'planning_frame',
    'initial_start_xyz', 'max_selection_height_error_m', 'selection_max_age_s',
    'planning_timeout_s', 'map_display_max_points', 'handle_scale_m',
    'cost_margin_m', 'minimum_clearance_m', 'optimization_guard_cells',
    'max_heading_rate', 'max_ground_step_m',
    'astar_cost_weight', 'optimizer_cost_margin',
    'path_refinement', 'refinement_corner_cut_m',
    'placement_anchor_xy', 'place_endpoints_on_start',
    'restore_route_file', 'restore_tomogram_path',
    'crossfloor_route_config',
)


def service():
    result = subprocess.run(
        ['systemctl', '--user', 'show', UNIT, '--property=ActiveState,Description,SubState'],
        capture_output=True, text=True, check=False, timeout=5,
    )
    return dict(line.split('=', 1) for line in result.stdout.splitlines() if '=' in line)


def stop():
    state = service()
    if state.get('ActiveState') not in ACTIVE_STATES:
        return
    if not state.get('Description', '').startswith(OWNER):
        raise RuntimeError('Unknown preview service owner; refusing to stop it')
    subprocess.run(['systemctl', '--user', 'stop', UNIT], check=True, timeout=20)
    print('Stopped planning-only preview and its managed children; navigation and SDK untouched.')


def validate_config(cfg):
    if not isinstance(cfg, dict):
        raise ValueError('Preview config must be a mapping')
    unknown = set(cfg) - set(PARAMETERS)
    if unknown:
        raise ValueError('Unknown preview parameters: ' + ', '.join(sorted(unknown)))
    from d1max_pct_planner.planner_core import validate_native_parameters
    validate_native_parameters(cfg.get('astar_cost_weight', .2), cfg.get('optimizer_cost_margin', 15.0))
    backend = cfg.get('map_backend', 'legacy_grid')
    if cfg.get('crossfloor_route_config'):
        crossfloor = Path(cfg['crossfloor_route_config'])
        if backend != 'official_tomogram' or not crossfloor.is_absolute() or not crossfloor.is_file():
            raise ValueError('Cross-floor preview requires an absolute coordinator config and official tomogram')
    refinement = cfg.get('path_refinement', 'none')
    if refinement not in ('none', 'visibility_c2'):
        raise ValueError('path_refinement must be none or visibility_c2')
    if refinement != 'none' and backend != 'official_tomogram':
        raise ValueError('Corridor refinement requires the official layered map validator')
    cut = cfg.get('refinement_corner_cut_m', 1.5)
    if isinstance(cut, bool) or not isinstance(cut, (int, float)) or not math.isfinite(cut) or not 0 < cut <= 5:
        raise ValueError('refinement_corner_cut_m must be in (0, 5] metres')
    if backend not in ('legacy_grid', 'official_tomogram'):
        raise ValueError('Unknown map_backend; refusing silent map fallback')
    required_maps = ['map_pcd', 'tomogram_path' if backend == 'official_tomogram' else 'planning_grid']
    if 'planning_grid' in cfg and backend != 'legacy_grid':
        raise ValueError('Official PCT must not also configure a legacy planning_grid')
    restore = [cfg.get(key, '') for key in ('restore_route_file', 'restore_tomogram_path')]
    if any(restore):
        if backend != 'official_tomogram' or not all(
                isinstance(path, str) and Path(path).is_absolute() and Path(path).is_file()
                for path in restore):
            raise ValueError('Route restoration requires both existing absolute official-PCT files')
    for key in required_maps:
        if not isinstance(cfg.get(key), str) or not Path(cfg[key]).is_file():
            raise ValueError('Missing ' + key + ': ' + str(cfg.get(key)))
    if not isinstance(cfg.get('vendor_root'), str) or not Path(cfg['vendor_root']).is_dir():
        raise ValueError('Missing vendor_root')
    frame = cfg.get('planning_frame')
    if not isinstance(frame, str) or not frame or frame.startswith('/') or any(c.isspace() for c in frame):
        raise ValueError('Invalid planning_frame')
    xyz = cfg.get('initial_start_xyz', [])
    if 'place_endpoints_on_start' in cfg and not isinstance(cfg['place_endpoints_on_start'], bool):
        raise ValueError('place_endpoints_on_start must be boolean')
    if not isinstance(xyz, list) or len(xyz) not in (0, 3):
        raise ValueError('initial_start_xyz must be empty or three finite numbers')
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in xyz):
        raise ValueError('initial_start_xyz must be empty or three finite numbers')
    for key in ('max_selection_height_error_m', 'selection_max_age_s', 'planning_timeout_s', 'handle_scale_m'):
        value = cfg.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(key + ' must be finite and positive')
    count = cfg.get('map_display_max_points')
    if isinstance(count, bool) or not isinstance(count, int) or count <= 0:
        raise ValueError('map_display_max_points must be a positive integer')
    if cfg['max_selection_height_error_m'] > 0.15:
        raise ValueError('Selection height tolerance cannot exceed 0.15 m')
    for key in ('cost_margin_m', 'max_heading_rate', 'max_ground_step_m'):
        if key in cfg and (isinstance(cfg[key], bool) or not isinstance(cfg[key], (int, float))
                           or not math.isfinite(cfg[key]) or cfg[key] <= 0):
            raise ValueError(key + ' must be finite and positive')
    if 'minimum_clearance_m' in cfg:
        value = cfg['minimum_clearance_m']
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
            raise ValueError('minimum_clearance_m must be finite and nonnegative')
    if 'optimization_guard_cells' in cfg:
        value = cfg['optimization_guard_cells']
        if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= 5:
            raise ValueError('optimization_guard_cells must be an integer in [0, 5]')
    if 'placement_anchor_xy' in cfg:
        xy = cfg['placement_anchor_xy']
        if not isinstance(xy, list) or len(xy) != 2 or any(
                isinstance(v, bool) or not isinstance(v, (float, int)) or not math.isfinite(v) for v in xy):
            raise ValueError('placement_anchor_xy must contain two finite numbers')
    if backend == 'official_tomogram':
        if not cfg['tomogram_path'].endswith('.npz'):
            raise ValueError('Preview accepts safe NPZ tomography, not executable pickle')
        if cfg.get('selection_mode', 'ground') not in ('ground', 'free'):
            raise ValueError('selection_mode must be ground or free')
        if cfg.get('unknown_ceiling_policy', 'allow_unobserved') not in ('allow_unobserved', 'reject'):
            raise ValueError('Unknown ceiling policy must be explicit')
        for key in ('minimum_headroom_m', 'max_follow_height_change_m'):
            value = cfg.get(key)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError(key + ' must be positive')
        if isinstance(cfg.get('placement_anchor_z'), bool) or not isinstance(cfg.get('placement_anchor_z'), (int, float)) or not math.isfinite(cfg['placement_anchor_z']):
            raise ValueError('placement_anchor_z must be finite')
    return cfg


def isolation_configs():
    common = {
        'scouting': {'multicast': {'enabled': False}, 'gossip': {'enabled': False}},
        'timestamping': {'enabled': True, 'drop_future_timestamp': False},
    }
    endpoint = 'tcp/127.0.0.1:' + str(PORT)
    return {
        'client': {**common, 'mode': 'client', 'connect': {'endpoints': [endpoint], 'exit_on_failure': True}},
        'router': {**common, 'mode': 'router', 'listen': {'endpoints': [endpoint], 'exit_on_failure': True}, 'connect': {'endpoints': []}},
    }


def iter_displays(displays):
    """Visit displays inside RViz groups as well as top-level displays."""
    for display in displays:
        yield display
        yield from iter_displays(display.get('Displays', []))


def rviz_config(cfg):
    config = yaml.safe_load((SOURCE / 'rviz/preview.rviz').read_text())
    manager = config['Visualization Manager']
    manager['Global Options']['Fixed Frame'] = cfg['planning_frame']
    manager['Views']['Current']['Target Frame'] = cfg['planning_frame']
    if cfg['planning_frame'] == 'd1max_flat_floor_planning':
        for display in iter_displays(manager['Displays']):
            if display.get('Topic', {}).get('Value') == '/d1max/pct_preview/map':
                display['Name'] = '平地规划点云'
    if cfg.get('crossfloor_route_config'):
        manager['Views']['Current'].update(Distance=110.0, Pitch=.625, Yaw=.8,
                                            **{'Focal Point': {'X': -10., 'Y': 29., 'Z': 1.2}})
        for display in iter_displays(manager['Displays']):
            topic = display.get('Topic', {}).get('Value')
            if topic == '/d1max/pct_preview/map':
                display['Name'] = '跨层规划点云'
            if display.get('Color Transformer') == 'AxisColor':
                # The processed two-floor scene uses a common height scale for
                # cloud and surface; independent auto ranges give false cues.
                display['Autocompute Value Bounds'] = {
                    'Value': False, 'Min Value': -1.0, 'Max Value': 7.0}
            if topic in ('/d1max/pct_preview/tomogram', '/d1max/pct_preview/blocked_surfaces') and display.get('Style') == 'Flat Squares':
                display['Size (m)'] = .085
        manager['Views']['Saved'] = [
            {**manager['Views']['Current'], 'Name': '跨层总览'},
            {**manager['Views']['Current'], 'Name': '俯视', 'Pitch': 1.5707},
            {**manager['Views']['Current'], 'Name': '楼梯', 'Distance': 20.0,
             'Pitch': .35, 'Yaw': 2.4,
             'Focal Point': {'X': -30.5, 'Y': 51.5, 'Z': 1.2}},
        ]
    for view in manager['Views'].get('Saved', []):
        view['Target Frame'] = cfg['planning_frame']
    return config


def start(args):
    if service().get('ActiveState') in ACTIVE_STATES:
        raise RuntimeError('Preview already exists; stop it explicitly before starting another')
    cfg = validate_config(expand_tree(yaml.safe_load(args.config.read_text())))
    # Never adopt an unrelated router, including the offline navigation router.
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', PORT))
    sid = uuid.uuid4().hex[:12]
    directory = ROOT / (datetime.now().strftime('%Y%m%d_%H%M%S') + '_' + sid)
    directory.mkdir(parents=True)
    config_path = directory / 'preview.yaml'
    config_path.write_text(yaml.safe_dump(cfg, sort_keys=False))
    for name, data in isolation_configs().items():
        (directory / (name + '.json5')).write_text(json.dumps(data, indent=2))
    rviz_path = directory / 'preview.rviz'
    rviz_path.write_text(yaml.safe_dump(rviz_config(cfg), sort_keys=False))
    params = {key: cfg[key] for key in PARAMETERS if key in cfg}
    if 'placement_anchor_xy' in params:
        params['placement_anchor_xy'] = [float(v) for v in params['placement_anchor_xy']]
    for key in ('max_selection_height_error_m', 'selection_max_age_s', 'planning_timeout_s',
                'handle_scale_m', 'cost_margin_m', 'minimum_clearance_m',
                'max_heading_rate', 'max_ground_step_m', 'minimum_headroom_m',
                'placement_anchor_z', 'max_follow_height_change_m',
                'astar_cost_weight', 'optimizer_cost_margin', 'refinement_corner_cut_m'):
        if key in params:
            params[key] = float(params[key])
    # Empty ROS YAML arrays have no element type; rely on the node's dynamic default.
    if not params.get('initial_start_xyz'):
        params.pop('initial_start_xyz', None)
    else:
        params['initial_start_xyz'] = [float(value) for value in params['initial_start_xyz']]
    params.update(output_directory=str(directory), use_sim_time=False)
    params_path = directory / 'server.yaml'
    params_path.write_text(yaml.safe_dump({'/**': {'ros__parameters': params}}, sort_keys=False))
    snapshot = {
        'mode': 'planning_preview', 'session_id': sid, 'robot_connected': False,
        'real_motion_enabled': False, 'directory': str(directory), 'config': str(config_path),
        'server_parameters': str(params_path), 'headless': bool(args.headless),
        'rviz_config': str(rviz_path), 'router_port': PORT,
    }
    snapshot_path = directory / 'session.json'
    snapshot_path.write_text(json.dumps(snapshot, indent=2))
    permitted = (
        'PATH', 'LD_LIBRARY_PATH', 'PYTHONPATH', 'AMENT_PREFIX_PATH', 'CMAKE_PREFIX_PATH',
        'COLCON_PREFIX_PATH', 'DISPLAY', 'XAUTHORITY', 'XDG_RUNTIME_DIR', 'LANG',
        'D1MAX_PCT_COMPUTE_CONFIG',
    )
    environment = {key: os.environ[key] for key in permitted if key in os.environ}
    environment.update(RMW_IMPLEMENTATION='rmw_zenoh_cpp', ROS_DOMAIN_ID='24',
                       QT_QPA_PLATFORM='xcb', ROS_LOG_DIR=str(directory / 'ros_logs'))
    command = [
        'systemd-run', '--user', '--collect', '--unit=' + UNIT,
        '--property=Description=' + OWNER + sid, '--property=Type=exec',
        '--property=KillMode=mixed', '--property=KillSignal=SIGINT',
        '--property=TimeoutStopSec=15', '--property=SendSIGKILL=yes',
        '--working-directory=' + str(WS),
    ]
    command += ['--setenv=' + key + '=' + value for key, value in environment.items()]
    command += [sys.executable, str(SOURCE / 'd1max_pct_planner/preview_session.py'),
                'run', '--session', str(snapshot_path)]
    subprocess.run(command, check=True, timeout=15)
    (ROOT / 'last_session.json').write_text(json.dumps(snapshot, indent=2))
    print(json.dumps({'started': 'PCT PLANNING ONLY', 'session': str(snapshot_path),
                      'robot_connected': False, 'motion_enabled': False}, indent=2))


def runtime_environment(directory):
    # systemd's user manager may itself carry live-session environment variables.
    for key in tuple(os.environ):
        if key.startswith('ZENOH_'):
            os.environ.pop(key, None)
    os.environ.update(
        RMW_IMPLEMENTATION='rmw_zenoh_cpp', ROS_DOMAIN_ID='24',
        ZENOH_SESSION_CONFIG_URI=str(directory / 'client.json5'),
        ZENOH_ROUTER_CONFIG_URI=str(directory / 'router.json5'),
        OMP_NUM_THREADS='4', OPENBLAS_NUM_THREADS='2',
    )


def runtime_commands(snapshot, package_prefix):
    """Only these three executables can be started; no shell or node selection input."""
    def executable(package, name):
        return str(Path(package_prefix(package)) / 'lib' / package / name)
    commands = [
        [executable('rmw_zenoh_cpp', 'rmw_zenohd')],
        [executable('d1max_pct_planner', 'pct_preview_server'), '--ros-args',
         '-r', '__node:=pct_preview_server', '--params-file', snapshot['server_parameters']],
    ]
    if not snapshot['headless']:
        commands.append([executable('rviz2', 'rviz2'), '-d', snapshot['rviz_config'],
                         '--ros-args', '-r', '__node:=pct_preview_rviz'])
    return commands


def shutdown_children(processes):
    # Stop clients first so their shutdown can still communicate with our router.
    groups = (list(reversed(processes[1:])), processes[:1])
    for children in groups:
        for process in children:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
        deadline = time.monotonic() + (6.0 if len(children) > 1 else 2.0)
        for process in children:
            try:
                process.wait(timeout=max(0.1, deadline - time.monotonic()))
            except subprocess.TimeoutExpired:
                process.terminate()
                try:
                    process.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=1.0)


def run(snapshot_path):
    from ament_index_python.packages import get_package_prefix

    snapshot = json.loads(snapshot_path.read_text())
    if snapshot.get('mode') != 'planning_preview' or snapshot.get('robot_connected') is not False or snapshot.get('real_motion_enabled') is not False:
        raise ValueError('Only disconnected planning preview sessions are supported')
    directory = snapshot_path.parent.resolve()
    if directory != Path(snapshot['directory']).resolve() or snapshot.get('router_port') != PORT:
        raise ValueError('Invalid preview session identity')
    # Verify the private configurations before opening any socket.
    for name, expected in isolation_configs().items():
        if json.loads((directory / (name + '.json5')).read_text()) != expected:
            raise ValueError('Preview requires unchanged loopback-only Zenoh configs')
    runtime_environment(directory)
    commands = runtime_commands(snapshot, get_package_prefix)
    from d1max_pct_planner.native_runtime import prepare_native_environment
    config = yaml.safe_load(Path(snapshot['config']).read_text())
    # Dynamic linker lookup paths must be set BEFORE Python/native imports.
    # Only the PCT server/its spawned worker get bundled GTSAM; ROS, RViz,
    # SDK and the user's login environment remain untouched.
    native_environment = prepare_native_environment(config['vendor_root'], os.environ)
    processes, streams = [], []
    stopping = False

    def request_stop(*_):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        for index, command in enumerate(commands):
            if stopping:
                break
            print('START ' + json.dumps(command), flush=True)
            stream = (directory / (Path(command[0]).name + '.log')).open('a')
            streams.append(stream)
            processes.append(subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                                              env=native_environment if index == 1 else None))
            if index == 0:
                deadline = time.monotonic() + 5.0
                while not stopping:
                    if processes[0].poll() is not None:
                        raise RuntimeError('Private router exited; refusing any other router')
                    try:
                        with socket.create_connection(('127.0.0.1', PORT), timeout=0.1):
                            break
                    except OSError:
                        if time.monotonic() >= deadline:
                            raise RuntimeError('Private router did not open its loopback port')
                        time.sleep(0.05)
        while not stopping:
            ended = [p.pid for p in processes if p.poll() is not None]
            if ended:
                raise RuntimeError('Managed preview child exited: ' + str(ended))
            time.sleep(0.2)
    finally:
        shutdown_children(processes)
        for stream in streams:
            stream.close()


def main():
    parser = argparse.ArgumentParser(description='3D point selection and PCT global path only; no robot control')
    parser.add_argument('action', choices=['start', 'stop', 'status', 'run'], nargs='?', default='start')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--config', type=Path, default=SOURCE / 'config/preview.yaml')
    parser.add_argument('--session', type=Path)
    args = parser.parse_args()
    if args.action == 'run':
        if args.session is None:
            parser.error('run requires --session')
        run(args.session)
        return
    ROOT.mkdir(parents=True, exist_ok=True)
    with (ROOT / 'session.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        if args.action == 'start':
            start(args)
        elif args.action == 'stop':
            stop()
        else:
            previous = ROOT / 'last_session.json'
            print(json.dumps({'service': service(), 'last_session': json.loads(previous.read_text()) if previous.exists() else None}, indent=2))


if __name__ == '__main__':
    main()
