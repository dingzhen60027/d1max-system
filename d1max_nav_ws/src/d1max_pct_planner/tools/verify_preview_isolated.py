#!/usr/bin/env python3
"""Run backend acceptance on a fresh private Zenoh router, never the live UI.

The existing RViz session, its endpoints and its systemd service are not changed.
This does not perform a rendered-screen or mouse interaction acceptance test.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time
import traceback

import yaml
from ament_index_python.packages import get_package_prefix

from d1max_pct_planner.preview_session import PARAMETERS, SOURCE, isolation_configs, validate_config


def process_identity(pid):
    try:
        fields = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
        return {'pid': pid, 'start_time_ticks': fields[19], 'state': fields[0]}
    except (OSError, IndexError):
        return None


def collect_owned(processes, descendants):
    pending = [process.pid for process in processes if process.poll() is None]
    visited = set()
    while pending:
        pid = pending.pop()
        if pid in visited:
            continue
        visited.add(pid)
        try:
            children = [int(value) for value in Path(f'/proc/{pid}/task/{pid}/children').read_text().split()]
        except OSError:
            continue
        for child in children:
            identity = process_identity(child)
            if identity:
                descendants[(child, identity['start_time_ticks'])] = identity
                pending.append(child)


def same_running(identity):
    actual = process_identity(identity['pid'])
    return (actual is not None and actual['state'] != 'Z'
            and actual['start_time_ticks'] == identity['start_time_ticks'])


def cleanup(processes, descendants):
    """Stop only owned Popen children and verified exact descendant PIDs."""
    collect_owned(processes, descendants)
    events = []
    # Verifier and server first; the private router remains alive during ROS shutdown.
    for process in reversed(processes):
        signals = []
        for sig, timeout in ((signal.SIGINT, 5.0), (signal.SIGTERM, 1.0), (signal.SIGKILL, 1.0)):
            if process.poll() is not None:
                break
            process.send_signal(sig)
            signals.append(sig.name)
            try:
                process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                pass
        events.append({'pid': process.pid, 'command': process.args,
                       'signals': signals, 'returncode': process.poll()})
    for identity in descendants.values():
        signals = []
        for sig, timeout in ((signal.SIGTERM, 1.0), (signal.SIGKILL, 1.0)):
            if not same_running(identity):
                break
            try:
                os.kill(identity['pid'], sig)
                signals.append(sig.name)
            except ProcessLookupError:
                break
            deadline = time.monotonic() + timeout
            while same_running(identity) and time.monotonic() < deadline:
                time.sleep(0.05)
        events.append({**identity, 'descendant': True, 'signals': signals,
                       'still_running': same_running(identity)})
    return events


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=SOURCE / 'config/preview.yaml')
    parser.add_argument('--output-dir', type=Path, required=True,
                        help='New directory only; existing output is never overwritten')
    parser.add_argument('--timeout', type=float, default=150.0)
    options = parser.parse_args()
    if not 45.0 <= options.timeout <= 300.0:
        parser.error('--timeout must be in [45, 300] seconds')
    output = options.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=False)
    report = {'passed': False, 'mode': 'ISOLATED_BACKEND_ACCEPTANCE',
              'scope': 'Private ROS backend protocol acceptance, no RViz or manual mouse validation',
              'live_session_modified': False, 'robot_connected': False, 'motion_enabled': False,
              'output_directory': str(output), 'middleware': 'rmw_zenoh_cpp'}
    processes, streams, descendants = [], [], {}
    port = None

    def interrupted(signum, _frame):
        raise InterruptedError(f'Runner interrupted by signal {signum}')

    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)
    try:
        cfg = validate_config(yaml.safe_load(options.config.read_text()))
        report['source_config'] = str(options.config.resolve())
        report['map_backend'] = cfg.get('map_backend', 'legacy_grid')
        (output / 'preview.yaml').write_text(yaml.safe_dump(cfg, sort_keys=False))
        params = {key: cfg[key] for key in PARAMETERS if key in cfg}
        for key in ('placement_anchor_xy', 'initial_start_xyz'):
            if key in params:
                params[key] = [float(value) for value in params[key]]
        if not params.get('initial_start_xyz'):
            params.pop('initial_start_xyz', None)
        for key in ('max_selection_height_error_m', 'selection_max_age_s', 'planning_timeout_s',
                    'handle_scale_m', 'cost_margin_m', 'minimum_clearance_m',
                    'max_heading_rate', 'max_ground_step_m', 'minimum_headroom_m',
                    'placement_anchor_z', 'max_follow_height_change_m'):
            if key in params:
                params[key] = float(params[key])
        params.update(output_directory=str(output / 'paths'), use_sim_time=False)
        params_path = output / 'server.yaml'
        params_path.write_text(yaml.safe_dump({'/**': {'ros__parameters': params}}, sort_keys=False))

        # Bind an ephemeral loopback port exclusively before selecting it. The
        # reservation is released immediately before the owned router starts.
        reservation = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        reservation.bind(('127.0.0.1', 0))
        port = reservation.getsockname()[1]
        report['router_port'] = port
        endpoint = f'tcp/127.0.0.1:{port}'
        configs = isolation_configs()
        configs['client']['connect']['endpoints'] = [endpoint]
        configs['router']['listen']['endpoints'] = [endpoint]
        if configs['router']['connect']['endpoints']:
            raise RuntimeError('Isolated router must not connect to another router')
        for name, config in configs.items():
            for mechanism in ('multicast', 'gossip'):
                if config['scouting'][mechanism]['enabled'] is not False:
                    raise RuntimeError('Isolated acceptance requires disabled discovery')
            (output / (name + '.json5')).write_text(json.dumps(config, indent=2) + '\n')
        with tempfile.TemporaryDirectory(prefix='.private_runtime_', dir=output) as runtime_path:
            runtime = Path(runtime_path)
            for name, config in configs.items():
                (runtime / (name + '.json5')).write_text(json.dumps(config, indent=2))
            environment = {key: value for key, value in os.environ.items() if not key.startswith('ZENOH_')}
            environment.update(RMW_IMPLEMENTATION='rmw_zenoh_cpp', ROS_DOMAIN_ID='24',
                               ZENOH_SESSION_CONFIG_URI=str(runtime / 'client.json5'),
                               ZENOH_ROUTER_CONFIG_URI=str(runtime / 'router.json5'),
                               ROS_LOG_DIR=str(output / 'ros_logs'),
                               OMP_NUM_THREADS='4', OPENBLAS_NUM_THREADS='2')
            report['zenoh_environment_keys'] = sorted(key for key in environment if key.startswith('ZENOH_'))

            def executable(package, name):
                return str(Path(get_package_prefix(package)) / 'lib' / package / name)

            def spawn(name, command, env_override=None):
                stream = (output / (name + '.log')).open('w')
                streams.append(stream)
                process = subprocess.Popen(command, env=environment if env_override is None else env_override, stdout=stream,
                                           stderr=subprocess.STDOUT, start_new_session=True)
                processes.append(process)
                print(json.dumps({'started': name, 'pid': process.pid, 'private_port': port}), flush=True)
                return process

            try:
                reservation.close()
                router = spawn('router', [executable('rmw_zenoh_cpp', 'rmw_zenohd')])
                deadline = time.monotonic() + 6.0
                while True:
                    if router.poll() is not None:
                        raise RuntimeError('Owned router exited; refusing another router')
                    try:
                        with socket.create_connection(('127.0.0.1', port), timeout=0.1):
                            break
                    except OSError:
                        if time.monotonic() >= deadline:
                            raise RuntimeError('Owned isolated router startup timed out')
                        time.sleep(0.05)
                time.sleep(0.3)
                if router.poll() is not None:
                    raise RuntimeError('Owned router failed after opening its port')
                # Resolve native PCT/GTSAM ABI dependencies before this Python
                # server process starts. Router and verifier retain the base
                # ROS environment and every private-Zenoh isolation setting.
                from d1max_pct_planner.native_runtime import prepare_native_environment
                server_environment = prepare_native_environment(cfg['vendor_root'], environment)
                for key in ('RMW_IMPLEMENTATION', 'ROS_DOMAIN_ID',
                            'ZENOH_SESSION_CONFIG_URI', 'ZENOH_ROUTER_CONFIG_URI'):
                    if server_environment.get(key) != environment[key]:
                        raise RuntimeError('Native environment changed private ROS/Zenoh isolation')
                server = spawn('server', [executable('d1max_pct_planner', 'pct_preview_server'),
                    '--ros-args', '-r', '__node:=pct_preview_server', '--params-file', str(params_path)],
                    env_override=server_environment)
                if cfg.get('map_backend') == 'official_tomogram':
                    verifier_command = [sys.executable, '-m', 'd1max_pct_planner.official_preview_verify',
                        '--tomogram', cfg['tomogram_path'],
                        '--max-ground-step', str(cfg.get('max_ground_step_m', .15)),
                        '--output', str(output / 'acceptance.json')]
                else:
                    verifier_command = [sys.executable, '-m', 'd1max_pct_planner.preview_verify',
                        '--grid', cfg['planning_grid'], '--output', str(output / 'acceptance.json')]
                verifier = spawn('verifier', verifier_command)
                deadline = time.monotonic() + options.timeout
                while verifier.poll() is None:
                    collect_owned(processes, descendants)
                    if router.poll() is not None or server.poll() is not None:
                        raise RuntimeError('An owned backend process exited during verification')
                    if time.monotonic() >= deadline:
                        raise TimeoutError('Bounded isolated acceptance timed out')
                    time.sleep(0.2)
                result = json.loads((output / 'acceptance.json').read_text())
                report['verifier_returncode'] = verifier.returncode
                report['acceptance_passed'] = result.get('passed') is True
                report['passed'] = verifier.returncode == 0 and result.get('passed') is True
                report['checks'] = len(result.get('checks', []))
                if not report['passed']:
                    report['acceptance_error'] = result.get('error')
            finally:
                # Cleanup precedes removal of temporary configuration files.
                report['cleanup'] = cleanup(processes, descendants)
                processes.clear()
    except Exception as exc:
        report['passed'] = False
        report['error'] = f'{type(exc).__name__}: {exc}'
        report['traceback'] = traceback.format_exc()
    finally:
        if processes:
            report['cleanup'] = cleanup(processes, descendants)
        for stream in streams:
            stream.close()
        if port is not None:
            try:
                with socket.create_connection(('127.0.0.1', port), timeout=0.15):
                    report['private_port_closed'] = False
            except OSError:
                report['private_port_closed'] = True
        cleanup_failed = any(item.get('still_running') or (
            not item.get('descendant') and item.get('returncode') is None)
            for item in report.get('cleanup', []))
        report['passed'] = bool(report['passed'] and not cleanup_failed and report.get('private_port_closed'))
        (output / 'runner_report.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
        (output / 'README.md').write_text(
            '# Isolated preview backend validation\n\n'
            'This run used a fresh loopback-only Zenoh router with discovery disabled. '
            'It did not publish to or stop the live RViz session.\n\n'
            '`acceptance.json` records backend XYZ/attitude editing and native PCT protocol checks. '
            '`runner_report.json` records the private port, exact owned-process cleanup and overall result.\n\n'
            'The live RViz graph/QoS check is separate. This isolated run is headless; '
            'it is not a rendered-screen, manual mouse, robot motion or physical navigation test.\n')
    print(json.dumps({'passed': report['passed'], 'report': str(output / 'runner_report.json')},
                     allow_nan=False), flush=True)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
