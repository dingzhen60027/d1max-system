"""Managed map/RViz session. Offline is isolated; live reuses Web-owned localization."""
import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import re
import socket
import subprocess
import sys
import time
from urllib.error import HTTPError
from urllib.request import ProxyHandler, Request, build_opener

import yaml
from ament_index_python.packages import get_package_share_directory
from .initial_pose import LocalWebClient, _NoRedirect

UNIT = 'd1max-nav2-localization-test.service'
OWNER = 'D1MAX_NAV2_LOCALIZATION_TEST_V1:'


def unit_state():
    result = subprocess.run(['systemctl', '--user', 'show', UNIT,
        '--property=LoadState,ActiveState,SubState,MainPID,Description,ControlGroup'],
        capture_output=True, text=True, timeout=5)
    state = dict(line.split('=', 1) for line in result.stdout.splitlines() if '=' in line)
    absent = (state.get('LoadState') == 'not-found' and state.get('ActiveState') == 'inactive'
              and state.get('MainPID', '0') == '0' and not state.get('ControlGroup'))
    if (result.returncode != 0 and not absent) or not state:
        raise RuntimeError(result.stderr.strip() or 'Cannot query user systemd')
    return state


def populated(state):
    group = state.get('ControlGroup', '')
    if not group:
        return False
    path = (Path('/sys/fs/cgroup') / group.lstrip('/') / 'cgroup.events').resolve()
    if not path.is_relative_to('/sys/fs/cgroup'):
        raise RuntimeError('Invalid cgroup path')
    return path.is_file() and 'populated 1' in path.read_text()


def active(state):
    return state.get('ActiveState') in {'active', 'activating', 'deactivating'} or populated(state)


def stop():
    state = unit_state()
    if active(state):
        if not state.get('Description', '').startswith(OWNER):
            raise RuntimeError('Refusing to stop an unowned service')
        subprocess.run(['systemctl', '--user', 'stop', UNIT], check=True, timeout=20)
        if active(unit_state()):
            raise RuntimeError('Test process group has not stopped; no restart allowed')
    print('Nav2 map/RViz test stopped. Existing Web/localization/SDK services are unchanged.')


def api(base, suffix, body=None):
    request = Request(base.rstrip('/') + '/api/localization/' + suffix,
        data=None if body is None else json.dumps(body).encode(),
        headers={'Content-Type': 'application/json'})
    try:
        with build_opener(ProxyHandler({}), _NoRedirect()).open(request, timeout=6) as response:
            return json.load(response)
    except HTTPError as error:
        raise RuntimeError(error.read().decode(errors='replace')) from error


def selected_map(app):
    root = app / 'map_manager/data/navigation2d'
    state = json.loads((root / 'state.json').read_text())
    version = state.get('selected_id', '')
    if not re.fullmatch(r'grid-[0-9a-f]{24}', version) or version in state.get('archived', []):
        raise ValueError('Select one complete, non-archived 2D map in Web first')
    folder = root / 'versions' / version
    for name in ('map.yaml', 'localization.pcd', 'manifest.json'):
        if not (folder / name).is_file():
            raise ValueError('Incomplete map version: ' + str(folder / name))
    grid = yaml.safe_load((folder / 'map.yaml').read_text())
    image = (folder / grid['image']).resolve()
    if not image.is_file() or not image.is_relative_to(folder.resolve()):
        raise ValueError('Map image must exist inside its version directory')
    manifest = json.loads((folder / 'manifest.json').read_text())
    return version, folder, grid, manifest


def map_view(grid, manifest, viewport=(1120, 700)):
    resolution = float(grid['resolution'])
    width, height = int(manifest['width']), int(manifest['height'])
    x, y, yaw = map(float, grid['origin'])
    if not all(math.isfinite(v) for v in (resolution, x, y, yaw)) or resolution <= 0 or min(width, height) <= 0:
        raise ValueError('Invalid map dimensions/origin')
    wx, wy = width * resolution, height * resolution
    c, s = math.cos(yaw), math.sin(yaw)
    return {'X': x + c * wx / 2 - s * wy / 2,
            'Y': y + s * wx / 2 + c * wy / 2,
            'Scale': 0.85 * min(viewport[0] / (abs(c)*wx+abs(s)*wy), viewport[1] / (abs(s)*wx+abs(c)*wy))}


def prepare_rviz(template, settings, grid, manifest):
    value = yaml.safe_load(template.read_text())
    manager = value['Visualization Manager']
    manager['Global Options']['Fixed Frame'] = settings['map_frame']
    geometry = value['Window Geometry']
    manager['Views']['Current'].update(map_view(grid, manifest,
        (max(400, geometry['Width'] - 900), max(300, geometry['Height'] - 220))))
    displays = manager['Displays']
    displays[0]['Topic']['Value'] = settings['map_topic']
    displays[0]['Name'] = '2D 地图 · OFFLINE' if settings['mode'] == 'offline' else '2D 地图'
    if settings['mode'] == 'offline':
        for display in displays[1:]:
            if display['Class'] != 'rviz_default_plugins/Marker':
                display['Enabled'] = display['Value'] = False
        # No stale or misleading seed while offline. Live mode restores the arrow tool.
        manager['Tools'] = [tool for tool in manager['Tools'] if tool['Class'] != 'rviz_default_plugins/SetInitialPose']
    else:
        for tool in manager['Tools']:
            if tool['Class'] == 'rviz_default_plugins/SetInitialPose':
                tool['Topic']['Value'] = settings['initial_pose_topic']
    return value


def port_open(port):
    try:
        with socket.create_connection(('127.0.0.1', port), timeout=0.3):
            return True
    except OSError:
        return False


def start(args, app, nav, share, root):
    current = unit_state()
    if active(current):
        if not current.get('Description', '').startswith(OWNER):
            raise RuntimeError('Unowned service conflict')
        raise RuntimeError('A test is already open; use restart --mode offline|live, or stop. No duplicate was started.')
    if not os.environ.get('DISPLAY'):
        raise RuntimeError('RViz requires a desktop DISPLAY')
    settings = yaml.safe_load(Path(args.config or share / 'config/localization_test.yaml').read_text())
    LocalWebClient(settings['web_url'])  # Validate loopback URL; no network operation.
    if settings['map_frame'] != 'd1max_loc_map':
        raise ValueError('Current localization contract uses d1max_loc_map; do not invent an identity TF')
    if not math.isfinite(float(settings['initial_pose_z'])) or abs(float(settings['initial_pose_z'])) > 100:
        raise ValueError('Invalid body Z seed')
    version, folder, grid, manifest = selected_map(app)
    settings.update(mode=args.mode, version_id=version, map_yaml=str(folder / 'map.yaml'),
        localization_pcd=str(folder / 'localization.pcd'), map_name=manifest['name'],
        localization_session_id='', motion_control_enabled=False, created_at=time.time())
    center = map_view(grid, manifest)
    # Status belongs to the map view, never a fabricated robot/body frame.
    settings['status_anchor_x'] = center['X']
    settings['status_anchor_y'] = center['Y'] + manifest['height'] * grid['resolution'] * 0.39
    if args.mode == 'offline':
        if port_open(7460):
            raise RuntimeError('Offline port 7460 is occupied; refusing to reuse an unknown router')
        session_config = share / 'config/zenoh-offline-session.json5'
        router_config = share / 'config/zenoh-offline-router.json5'
    else:
        if not port_open(7448):
            raise RuntimeError('Live Zenoh is not running. Connect the robot in Web first; no automatic SDK takeover.')
        current = api(settings['web_url'], 'overview')
        connection = current.get('connection', {})
        health = connection.get('health', {})
        if not (connection.get('active') and health.get('sdk_fresh') and health.get('lidar_fresh') and health.get('replay') is False):
            raise RuntimeError('Live SDK/LiDAR are not fresh. Refusing replay/stale data as real localization.')
        if current.get('phase') in {'stopped', 'idle', 'failed'}:
            current = api(settings['web_url'], 'start', {'version_id': version})
        if current.get('phase') not in {'running', 'starting'} or current.get('version_id') != version:
            raise RuntimeError('Localization session/map conflict. Check Web; no existing session was stopped.')
        settings['localization_session_id'] = current['id']
        session_config = app / 'foxglove_d1max/config/zenoh-live.json5'
        router_config = app / 'foxglove_d1max/config/zenoh-router-live.json5'
    directory = root / (time.strftime('%Y%m%d_%H%M%S') + '_' + args.mode + '_' + str(os.getpid()))
    directory.mkdir()
    settings['rviz_config'] = str(directory / ('nav2_' + args.mode + '.rviz'))
    Path(settings['rviz_config']).write_text(yaml.safe_dump(prepare_rviz(
        share / 'rviz/localization_test.rviz', settings, grid, manifest), allow_unicode=True, sort_keys=False))
    config_path = directory / 'session.json'
    config_path.write_text(json.dumps(settings, ensure_ascii=False, indent=2))
    (root / 'last_session.json').write_text(json.dumps(settings, ensure_ascii=False, indent=2))
    env = {k: os.environ[k] for k in ('PATH', 'LD_LIBRARY_PATH', 'PYTHONPATH', 'AMENT_PREFIX_PATH',
        'CMAKE_PREFIX_PATH', 'COLCON_PREFIX_PATH', 'DISPLAY', 'XAUTHORITY', 'XDG_RUNTIME_DIR', 'LANG') if k in os.environ}
    env.update(RMW_IMPLEMENTATION='rmw_zenoh_cpp', ROS_DOMAIN_ID='24',
        ZENOH_SESSION_CONFIG_URI=str(session_config), ZENOH_ROUTER_CONFIG_URI=str(router_config),
        QT_QPA_PLATFORM='xcb', ROS_LOG_DIR=str(directory / 'ros_logs'))
    command = ['systemd-run', '--user', '--collect', '--unit=' + UNIT,
        '--property=Description=' + OWNER + args.mode + ':' + version,
        '--property=Type=exec', '--property=KillMode=control-group',
        '--property=KillSignal=SIGINT', '--property=TimeoutStopSec=12',
        '--property=SendSIGKILL=yes', '--working-directory=' + str(nav)]
    if args.mode == 'live':
        for prop in ('BindsTo', 'After'):
            command += ['--property=' + prop + '=d1max-localization-managed.service d1max-monitor-managed.service']
    command += ['--setenv=' + k + '=' + v for k, v in env.items()]
    command += ['ros2', 'launch', 'd1max_navigation', 'localization_test.launch.py', 'session:=' + str(config_path)]
    subprocess.run(command, check=True, timeout=15)
    print(json.dumps({'started': args.mode, 'map': manifest['name'], 'version': version,
        'frame': settings['map_frame'], 'middleware': 'rmw_zenoh_cpp',
        'robot_connected_by_this_test': False, 'motion_control_enabled': False,
        'session': str(config_path)}, ensure_ascii=False, indent=2))


def main():
    parser = argparse.ArgumentParser(description='Nav2 map + RViz localization test; no motion commands')
    parser.add_argument('action', choices=['start', 'stop', 'restart', 'status'], nargs='?', default='start')
    parser.add_argument('--mode', choices=['offline', 'live'], default='offline')
    parser.add_argument('--config', help='Stage-1 YAML configuration, not a SLAM calibration file')
    args = parser.parse_args()
    nav = Path(os.environ['D1MAX_NAV_ROOT'])
    app = Path(os.environ['D1MAX_APP_ROOT'])
    root = nav / 'log/nav2_localization_test'
    root.mkdir(parents=True, exist_ok=True)
    try:
        with (root / '.lock').open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            if args.action == 'status':
                state = unit_state()
                previous = root / 'last_session.json'
                print(json.dumps({'unit': state, 'last_session': json.loads(previous.read_text()) if previous.exists() else None}, ensure_ascii=False, indent=2))
            else:
                if args.action in {'stop', 'restart'}:
                    stop()
                if args.action in {'start', 'restart'}:
                    start(args, app, nav, Path(get_package_share_directory('d1max_navigation')), root)
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
        print('Nav2 test: ' + str(error), file=sys.stderr)
        sys.exit(1)
