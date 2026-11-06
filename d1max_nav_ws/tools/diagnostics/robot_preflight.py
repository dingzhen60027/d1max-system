#!/usr/bin/env python3
"""Read-only readiness check before and after connecting the robot.

Never starts, stops or restarts a service, never opens an SDK or ROS session
and never sends a command. It reads files, systemd state, routes and makes
plain TCP connects to the configured robot endpoints. Exit 0 only when every
check passes; blockers are printed with who acts on them.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import socket
import subprocess
import sys

NAV_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(NAV_ROOT / 'src/d1max_pct_planner'))
from d1max_pct_planner import paths  # noqa: E402

RELEASE_MONITOR = 'sdk/install/d1max_sdk_bridge/lib/d1max_sdk_bridge/sdk_monitor_bridge'


class Report:
    def __init__(self):
        self.rows = []
        self.motion_ready = False
        self.physical_record_valid = False
        self.runtime_profile_valid = False
        self.purpose = None

    def add(self, name, ok, detail, action=''):
        self.rows.append({'check': name, 'ok': bool(ok), 'detail': detail, 'action': '' if ok else action})

    @property
    def ok(self):
        return all(row['ok'] for row in self.rows)


def run(*command):
    return subprocess.run(command, capture_output=True, text=True, timeout=4)


def unit_state(unit):
    # One property per call: systemctl does not keep the requested order.
    values = [run('systemctl', '--user', 'show', unit, f'--property={name}', '--value').stdout.strip()
              for name in ('ActiveState', 'MainPID')]
    return [values[0] or 'unknown', values[1]]


def process_exe(pid):
    try:
        return str(Path(f'/proc/{pid}/exe').readlink())
    except (OSError, ValueError):
        return ''


def children(pid):
    try:
        return [int(value) for value in Path(f'/proc/{pid}/task/{pid}/children').read_text().split()]
    except OSError:
        return []


def descendants(pid):
    pending, found = [pid], []
    while pending:
        current = pending.pop()
        for child in children(current):
            found.append(child)
            pending.append(child)
    return found


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def load_helper(name, path):
    """Import only the entry's stdlib helper, not a ROS/SDK session."""
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise ValueError('entry_helper_missing:' + str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def selected_release(nav_root):
    override = os.environ.get('D1MAX_RELEASE', '').strip()
    if override:
        return Path(override).resolve(strict=True)
    helper = load_helper('d1max_preflight_selector',
                         nav_root / 'tools/release/single_floor_entry_preflight.py')
    return helper.select(nav_root)


def check_release(report, release):
    try:
        root, descriptor = paths.release_descriptor(release)
        manifest = json.loads((root / descriptor['sealed_manifest']).read_text())
        changed = [path for path, expected in manifest['files'].items() if sha256(path) != expected]
        report.add('release_sealed', not changed, f'{root.name}: {len(manifest["files"])} files, changed={changed[:3]}',
                   '重新封存前不要运行；核对谁改了封存文件')
        report.add('release_not_activated', manifest.get('activation') is False and manifest.get('physical_acceptance') is False,
                   f'activation={manifest.get("activation")} physical_acceptance={manifest.get("physical_acceptance")}')
        return root
    except (OSError, ValueError, KeyError) as error:
        report.add('release_sealed', False, str(error), '设置 D1MAX_RELEASE 为已封存 release')
        return None


def check_monitor(report, release, app_root):
    state, pid = unit_state('d1max-monitor-managed.service')
    sdk = [process_exe(child) for child in descendants(int(pid))] if pid.isdecimal() and int(pid) > 0 else []
    sdk = [exe for exe in sdk if exe.endswith('/sdk_monitor_bridge')]
    wanted = str(release / RELEASE_MONITOR) if release else ''
    report.add('monitor_running', state == 'active' and len(sdk) == 1, f'{state}, sdk={sdk}',
               '经 manager 启动受管 monitor（需要你批准）')
    report.add('monitor_is_release_v3', bool(wanted) and sdk == [wanted], f'expected {wanted or "<release>"}',
               f'写 {app_root}/foxglove_d1max/config/monitor-release 后经 manager 重启 monitor（需要你批准）')


def check_web(report, app_root):
    state, pid = unit_state('d1max-web-managed.service')
    report.add('web_running', state == 'active', f'{state} pid={pid}', '经 manager 启动 Web')
    # Check the actual backend, not the unrelated historical Nav2 shell script.
    scripts = [app_root / 'map_manager/backend' / name
               for name in ('single_floor.py', 'mainline_release.py', 'mainline_views.py')]
    try:
        started = Path(f'/proc/{int(pid)}').stat().st_mtime if pid.isdecimal() and int(pid) > 0 else None
        current = started is not None and all(started >= script.stat().st_mtime for script in scripts)
    except OSError:
        current = False
    report.add('web_loaded_current_backend', current, 'backend=single_floor/mainline_release/mainline_views',
               '通过现有 manager 重启 Web，加载当前后端；本检查不重启服务')


def check_launch_configuration(report, release, nav_root, app_root):
    """Static deployment bindings; never bind a session or grant execution."""
    activation_path = Path(os.environ.get('D1MAX_MAP_MANAGER_DATA',
        str(app_root / 'map_manager/data'))) / 'single_floor/activation.json'
    activation = None
    try:
        info = activation_path.stat()
        if activation_path.is_symlink() or info.st_uid != os.getuid() or info.st_mode & 0o077:
            raise ValueError('private_owned_activation_required')
        activation = json.loads(activation_path.read_text())
        launcher = load_helper('d1max_preflight_web_launch_policy',
            app_root / 'map_manager/backend/mainline_release.py')
        activation = launcher.activation_launch_config(activation)
        if activation.get('schema_version') != 3 or activation.get('profile') != 'single_floor_live':
            raise ValueError('activation_schema')
        report.purpose = activation['purpose']
        if report.purpose not in ('planning_only', 'execution'):
            raise ValueError('explicit_purpose_required')
        if Path(activation['release_root']).resolve(strict=True) != release:
            raise ValueError('web_and_navigation_release_differ')
        descriptor = json.loads((release / 'release.json').read_text())
        seal = release / descriptor['sealed_manifest']
        if (Path(activation['release_manifest']) != seal or
                sha256(seal) != activation['release_manifest_sha256']):
            raise ValueError('web_activation_manifest_changed')
        entry = Path(activation['entrypoint'])
        if (entry not in (nav_root/'tools/navigation_entry.sh', nav_root/'tools/single_floor_entry.sh')
                or entry.is_symlink() or not os.access(entry, os.X_OK)
                or sha256(entry) != activation['entrypoint_sha256']):
            raise ValueError('web_entry_binding_changed')
        if (entry.name == 'navigation_entry.sh' and sha256(nav_root/'tools/single_floor_entry.sh')
                != activation.get('compatibility_entrypoint_sha256')):
            raise ValueError('web_compatibility_entry_binding_changed')
        policy = activation.get('sdk_session_policy', 'fixed')
        if (policy not in ('fixed', 'bind_current_on_start') or
                policy == 'bind_current_on_start' and report.purpose != 'planning_only' and not activation.get('runtime_motion_profile') or
                policy == 'fixed' and not activation.get('expected_sdk_session')):
            raise ValueError('sdk_session_binding_invalid')
        report.add('web_activation_matches_mainline', True,
                   f'{activation_path}, purpose={report.purpose}, SDK policy={policy}')
    except (OSError, ValueError, KeyError, TypeError, AttributeError) as error:
        report.add('web_activation_matches_mainline', False, str(error),
                   '核对实际 Web DATA_ROOT 下的 activation 与唯一 selector；不要用旧入口兜底')
        activation = None
    try:
        helper = load_helper('d1max_preflight_sdk_startup',
            app_root / 'foxglove_d1max/scripts/sdk_motion_startup.py')
        selected = helper.resolve_monitor_release()
        report.add('sdk_launcher_matches_mainline', selected == release,
                   f'configured={selected}, navigation={release}',
                   'SDK 默认用 @mainline；检查显式 D1MAX_MONITOR_RELEASE 覆盖')
    except (OSError, ValueError, KeyError, RuntimeError) as error:
        report.add('sdk_launcher_matches_mainline', False, str(error), '核对 SDK 唯一启动配置')
    return activation


def check_network(report, app_root):
    config = json.loads((app_root / 'foxglove_d1max/config/manager.json').read_text())
    interface = config.get('robot_interface', '')
    try:
        carrier = (Path('/sys/class/net') / interface / 'carrier').read_text().strip() == '1'
    except OSError:
        carrier = False
    report.add('robot_link', carrier, f'{interface} carrier={carrier}', f'插上机器人网线（{interface}）并启用 D1max 连接')
    for host, port in config.get('robot_endpoints', []):
        route = run('ip', '-j', 'route', 'get', host)
        try:
            device = json.loads(route.stdout)[0].get('dev')
        except (ValueError, IndexError):
            device = None
        report.add(f'route_{host}', device == interface, f'dev={device}', f'{host} 必须经过 {interface}，关闭代理 TUN 抢路由')
        try:
            with socket.create_connection((host, int(port)), timeout=1.5):
                reachable = True
        except OSError:
            reachable = False
        report.add(f'tcp_{host}:{port}', reachable and device == interface, f'reachable={reachable}', '机器人上电并确认端点')


def check_acceptance(report, release, activation, *, files_only=False):
    # Runtime validity is not physical acceptance. Reuse the sole writer's
    # parser for both normal configuration and legacy measured records.
    record = Path(activation['motion_profile_record']) if activation and activation.get('motion_profile_record') else None
    if record is None and files_only and report.purpose == 'planning_only':
        report.add('configured_mode', True, 'planning_only; motion is NOT accepted or enabled')
        return
    try:
        if record is None or release is None:
            raise ValueError('no_bound_motion_profile')
        if sha256(record) != activation['motion_profile_sha256']:
            raise ValueError('motion_profile_binding_changed')
        value = json.loads(record.read_text())
        checker = release/'sdk/install/d1max_sdk_bridge/lib/d1max_sdk_bridge/execution_acceptance_check'
        result = subprocess.run([str(checker), str(record), value['robot_id'], value['sdk_version'],
            value['calibration_sha256'], value['robot_profile_sha256']], capture_output=True,
            text=True, timeout=10, check=False)
        accepted = json.loads(result.stdout)
        if result.returncode or accepted.get('valid') is not True:
            raise ValueError('writer_rejected_motion_profile:' + result.stdout.strip()[:512])
        descriptor = json.loads((release/'release.json').read_text())
        profile = release/descriptor['template_source_root']/'src/d1max_scan_planner/config/d1max_robot.yaml'
        if sha256(profile) != value['robot_profile_sha256']:
            raise ValueError('accepted_profile_differs_from_actual_navigation_profile')
    except (OSError, ValueError, KeyError, TypeError, subprocess.TimeoutExpired) as error:
        report.add('motion_profile', False, str(error),
                   '核对 Web、导航和 SDK 的同一运行配置，不要求标定报告')
        return
    report.runtime_profile_valid = True
    report.physical_record_valid = accepted.get('physical_acceptance_verified') is True
    report.add('motion_profile', True, f'writer-validated runtime parameters={record}; physical_acceptance={report.physical_record_valid}')


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--offline', action='store_true', help='skip robot network checks')
    parser.add_argument('--files-only', action='store_true',
                        help='configuration and sealed files only; no service, network, ROS or SDK session checks')
    parser.add_argument('--json', action='store_true')
    args = parser.parse_args()
    report = Report()
    nav_root, app_root = paths.nav_root(), paths.app_root()
    try:
        release = check_release(report, selected_release(nav_root))
    except (OSError, ValueError, KeyError) as error:
        report.add('release_sealed', False, str(error), '核对 deploy/single_floor_release.json 的默认选择')
        release = None
    activation = check_launch_configuration(report, release, nav_root, app_root)
    if not args.files_only:
        check_web(report, app_root)
        check_monitor(report, release, app_root)
        if not args.offline:
            check_network(report, app_root)
    check_acceptance(report, release, activation, files_only=args.files_only)
    if args.json:
        print(json.dumps({'ok': report.ok, 'scope': 'files_only' if args.files_only else 'readiness',
                          'purpose': report.purpose, 'motion_ready': report.motion_ready,
                          'physical_record_valid': report.physical_record_valid,
                          'runtime_profile_valid': report.runtime_profile_valid,
                          'sdk_connected_by_check': False, 'checks': report.rows}, ensure_ascii=False, indent=1))
    else:
        for row in report.rows:
            print(('OK   ' if row['ok'] else 'FAIL ') + f"{row['check']}: {row['detail']}" +
                  (f"\n       → {row['action']}" if row['action'] else ''))
        print(f'purpose={report.purpose}; motion_ready={report.motion_ready}; no session started')
    return 0 if report.ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
