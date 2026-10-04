"""Owned, private-Zenoh global BT stage. Never starts SDK/control/localization.

This is a planning test entry, not a renamed single-floor execution release.
Only actual BT + Action adapters + source-route/native PCT and a stationary
start fixture run. Closing this workbench retires its own test processes.
"""
import argparse
from datetime import datetime
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time

import yaml
from d1max_pct_planner.paths import nav_root, expand_tree
from d1max_pct_planner.preview_session import rviz_config
from .bt_configuration import prepare_tree, worker_arguments

UNIT = 'd1max-crossfloor-global-test.service'
PORT = 7469


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def ros_executable(package, name):
    from ament_index_python.packages import get_package_prefix
    path = Path(get_package_prefix(package))/'lib'/package/name
    if not path.is_file():
        raise ValueError(f'Missing executable {path}; run tools/crossfloor_global_entry.sh build')
    return str(path)


def prepare(directory):
    ws = nav_root()
    artifact = ws/'maps/processed/sc_pgo_20260923_crossfloor_complete_001'
    manifest = artifact/'manifest.json'
    source = json.loads(manifest.read_text())['source_path']
    session = dict(id=directory.name, frame_id='d1max_loc_map', map_pcd=source,
        planning_manifest=str(manifest), tomogram_npz=str(artifact/'pct/tomogram.npz'),
        behavior_tree_xml=str(ws/'src/d1max_navigation_bt/trees/test_global_route_stage.xml'),
        navigation_contract={'frames': {'body_frame': 'd1max_loc_base_link',
                                       'tracking_frame': 'd1max_loc_tracking'}},
        body_height=.55, result_timeout_s=10., freshness_s=.5)
    prepare_tree(directory, session, ws)
    (directory/'global').mkdir()
    map_id = 'crossfloor-stage:'+sha(manifest)[:24]
    def params(filename, values):
        path = directory/filename
        path.write_text(yaml.safe_dump({'/**': {'ros__parameters': values}}, allow_unicode=True))
        return str(path)
    for filename, extra in (
        ('bt.yaml', dict(execution_transport_mode='isolated_mock', execution_purpose='execution')),
        ('bt_adapter.yaml', dict(pipeline_contract='atomic_navigation_v3',
            execution_mode='preview', map_version_id=map_id, transport_mode='isolated_mock'))):
        path = directory/filename
        values = yaml.safe_load(path.read_text())['/**']['ros__parameters']
        params(filename, {**values, **extra})
    global_file = params('global.yaml', dict(session_id=session['id'],
        pipeline_contract='atomic_navigation_v3', map_version_id=map_id, transport_mode='isolated_mock',
        planning_manifest=str(manifest), tomogram_npz=session['tomogram_npz'],
        crossfloor_route_config=str(artifact/'route.yaml'), current_floor='floor1', goal_floor='floor2',
        output_directory=str(directory/'global'), result_timeout_s=10., warmup_timeout_s=60.,
        freshness_s=.5))
    cfg = expand_tree(yaml.safe_load((ws/'src/d1max_pct_planner/config/preview_crossfloor_complete.yaml').read_text()))
    for name in ('restore_route_file', 'restore_tomogram_path'):
        cfg[name] = ''
    cfg.update(external_planner=True, selection_mode='ground', place_endpoints_on_start=False,
        output_directory=str(directory/'routes'), session_id=session['id'], map_version_id=map_id,
        planning_manifest=str(manifest), fixture_body_height=.55)
    editor_file = params('editor.yaml', cfg)
    display_cfg = rviz_config(cfg)
    displays = display_cfg['Visualization Manager']['Displays']
    selection = next(display for display in displays if display['Name']=='选点')
    selection['Displays'].append(dict(Class='rviz_default_plugins/MarkerArray',
        Name='落地证据 · 楼层与高度差', Enabled=True,
        Topic={'Value': '/d1max/pct_preview/markers', 'Depth': 1,
               'Reliability Policy': 'Reliable', 'Durability Policy': 'Transient Local'}))
    display_cfg['Visualization Manager']['Global Options']['Background Color'] = '22; 27; 34'
    display_cfg['Visualization Manager']['Views']['Current']['Distance'] = 100.
    (directory/'global.rviz').write_text(yaml.safe_dump(display_cfg, allow_unicode=True))
    python = sys.executable
    commands = [
        [ros_executable('rmw_zenoh_cpp', 'rmw_zenohd')],
        [python, '-m', 'd1max_pct_scan.live_global_planner', '--ros-args', '--params-file', global_file, *worker_arguments()],
        [python, '-m', 'd1max_pct_scan.bt_adapters', '--ros-args', '--params-file', str(directory/'bt_adapter.yaml')],
        [ros_executable('d1max_navigation_bt', 'navigator_node'), '--ros-args', '--params-file', str(directory/'bt.yaml')],
        [ros_executable('nav2_lifecycle_manager', 'lifecycle_manager'), '--ros-args', '--params-file', str(directory/'bt_lifecycle.yaml')],
        [python, '-m', 'd1max_pct_scan.global_stage_editor', '--ros-args', '--params-file', editor_file],
    ]
    endpoint = f'tcp/127.0.0.1:{PORT}'
    common = {'scouting': {'multicast': {'enabled': False}, 'gossip': {'enabled': False}},
              'timestamping': {'enabled': True, 'drop_future_timestamp': False}}
    for kind, values in (('client', {'mode': 'client', 'connect': {'endpoints': [endpoint]}}),
                         ('router', {'mode': 'router', 'listen': {'endpoints': [endpoint]},
                                     'connect': {'endpoints': []}})):
        (directory/f'zenoh_{kind}.json5').write_text(json.dumps({**common, **values}, indent=2))
    inventory = dict(scope='crossfloor_global_planning_stage_only',
        robot_connected=False, motion_enabled=False, arrival_tested=False,
        simulated_input='stationary_start_fixture', domain=os.environ['ROS_DOMAIN_ID'],
        router=endpoint, source_revision=subprocess.check_output(['git','rev-parse','HEAD'], cwd=ws, text=True).strip(),
        dirty_status=subprocess.check_output(['git','status','--short'], cwd=ws, text=True),
        session=session, commands=commands,
        artifacts={str(path): sha(path) for path in (manifest, Path(source),
            Path(session['tomogram_npz']), artifact/'route.yaml', Path(session['behavior_tree_xml']))},
        modules={}, executables={})
    for module in ('d1max_pct_scan.global_stage_editor', 'd1max_pct_scan.live_global_planner',
                   'd1max_pct_scan.bt_adapters', 'd1max_pct_scan.native_global_worker',
                   'd1max_pct_scan.source_route', 'd1max_pct_scan.source_route_ros',
                   'd1max_pct_scan.pointcloud_helpers.ground_path_bridge',
                   'd1max_pct_planner.tomogram_selection'):
        path = importlib.util.find_spec(module).origin
        inventory['modules'][module] = {'path': path, 'sha256': sha(path)}
    for command in commands:
        path = Path(command[0]).resolve()
        inventory['executables'][str(path)] = sha(path)
    from d1max_pct_planner.native_runtime import vendorverify
    inventory['native_pct'] = vendorverify(cfg['vendor_root'])
    inventory['effective_parameters'] = {str(path): sha(path)
        for path in sorted(directory.glob('*.yaml'))}
    inventory['isolated_environment'] = {name: os.environ.get(name, '') for name in
        ('RMW_IMPLEMENTATION', 'ROS_DOMAIN_ID', 'D1MAX_NAV_ISOLATED',
         'D1MAX_OFFLINE_ZENOH_TEST', 'D1MAX_NAV_TRANSPORT', 'OMP_NUM_THREADS',
         'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS', 'D1MAX_RELEASE',
         'AMENT_PREFIX_PATH', 'PYTHONPATH', 'LD_LIBRARY_PATH')}
    (directory/'inventory.json').write_text(json.dumps(inventory, indent=2, ensure_ascii=False)+'\n')
    return commands


def run(with_rviz, regression):
    if (os.environ.get('D1MAX_NAV_ISOLATED')!='1' or os.environ.get('ROS_DOMAIN_ID')!='219'
            or os.environ.get('RMW_IMPLEMENTATION')!='rmw_zenoh_cpp'):
        raise ValueError('private_crossfloor_stage_environment_required')
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', PORT))  # Never adopt an existing router.
    root = Path(os.environ['D1MAX_GLOBAL_STAGE_ROOT'])
    directory = root/'sessions'/('global_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    directory.mkdir(parents=True)
    commands = prepare(directory)
    env = dict(os.environ, ZENOH_SESSION_CONFIG_URI=str(directory/'zenoh_client.json5'),
               ZENOH_ROUTER_CONFIG_URI=str(directory/'zenoh_router.json5'))
    stopped = False
    def stop(_signum, _frame):
        nonlocal stopped
        stopped = True
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    children, streams = [], []
    try:
        for index, command in enumerate(commands):
            stream = (directory/f'node_{index}.log').open('w')
            streams.append(stream)
            children.append(subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                env=env, start_new_session=True))
            if index == 0:
                time.sleep(.6)
                if children[0].poll() is not None:
                    raise RuntimeError('Private router failed; see node_0.log')
        print(f'Private cross-floor global stage: {directory}', flush=True)
        (root/'active.json').write_text(json.dumps(dict(directory=str(directory), pid=os.getpid())))
        if regression:
            result = subprocess.run([sys.executable, '-m', 'd1max_pct_scan.global_stage_regression',
                '--output', str(directory/'regression.json')], env=env, timeout=150)
            if result.returncode:
                raise RuntimeError('Global-stage regression failed; inspect regression.json and node logs')
            return
        display = None
        if with_rviz:
            stream = (directory/'rviz.log').open('w')
            streams.append(stream)
            display = subprocess.Popen([ros_executable('rviz2','rviz2'), '-d', str(directory/'global.rviz')],
                stdout=stream, stderr=subprocess.STDOUT, env=env, start_new_session=True)
            children.append(display)
        while not stopped:
            if display is not None and display.poll() is not None:
                break
            failed = [i for i, child in enumerate(children[:len(commands)]) if child.poll() is not None]
            if failed:
                raise RuntimeError(f'Stage process exited: {failed}; see {directory}')
            time.sleep(.2)
    finally:
        # Owner-only cleanup: terminate editor/manager/BT/adapters/native worker,
        # then router. No broad pkill, no SDK or production service changes.
        for child in reversed(children[1:]):
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
        deadline = time.monotonic()+6.
        for child in reversed(children[1:]):
            try:
                child.wait(timeout=max(.1, deadline-time.monotonic()))
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait(timeout=2.)
        if children:
            router = children[0]
            if router.poll() is None:
                os.killpg(router.pid, signal.SIGINT)
            try:
                router.wait(timeout=2.)
            except subprocess.TimeoutExpired:
                os.killpg(router.pid, signal.SIGKILL)
                router.wait(timeout=2.)
        for stream in streams:
            stream.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=('start','stop','run'))
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--regression', action='store_true')
    args = parser.parse_args()
    if args.command == 'stop':
        state = subprocess.check_output(['systemctl','--user','show',UNIT,
            '--property=ActiveState,Description'],text=True)
        if 'ActiveState=inactive' in state:
            return
        if 'Description=D1MAX-CROSSFLOOR-GLOBAL-STAGE' not in state:
            raise RuntimeError('Unknown service owner; refusing to stop it')
        subprocess.run(['systemctl','--user','stop',UNIT], check=True)
    elif args.command == 'start' and not args.regression:
        result = subprocess.run(['systemctl','--user','is-active',UNIT], capture_output=True, text=True)
        if result.stdout.strip() in ('active','activating'):
            print('Cross-floor planning stage already running; not duplicated.')
            return
        entry = str(nav_root()/'tools/crossfloor_global_entry.sh')
        command = ['systemd-run','--user', '--unit='+UNIT, '--collect',
                   '--description=D1MAX-CROSSFLOOR-GLOBAL-STAGE', '--property=TimeoutStopSec=15',
                   entry, 'run']
        if args.headless:
            command.append('--headless')
        subprocess.run(command, check=True)
    else:
        run(not args.headless, args.regression)


if __name__ == '__main__':
    main()
