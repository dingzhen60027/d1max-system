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
import json
from pathlib import Path
import socket
import subprocess
import sys

NAV_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(NAV_ROOT / 'src/d1max_pct_planner'))
from d1max_pct_planner import paths  # noqa: E402

# Physical acceptance flags checked by the monitor's acceptance validator.
ACCEPTANCE_FLAGS = ('speed_mapping_verified', 'stop_timing_verified',
                    'body_envelope_verified', 'raw_ray_safety_verified')
RELEASE_MONITOR = 'sdk/install/d1max_sdk_bridge/lib/d1max_sdk_bridge/sdk_monitor_bridge'


class Report:
    def __init__(self):
        self.rows = []

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


def check_web(report, nav_root):
    state, pid = unit_state('d1max-web-managed.service')
    report.add('web_running', state == 'active', f'{state} pid={pid}', '经 manager 启动 Web')
    # A Web process started before the current entry scripts still resolves
    # the previous layout; it must be restarted to load them.
    script = nav_root / 'scripts/navigation/start_navigation.sh'
    try:
        started = Path(f'/proc/{int(pid)}').stat().st_mtime if pid.isdecimal() and int(pid) > 0 else None
        current = started is not None and started >= script.stat().st_mtime
    except OSError:
        current = False
    report.add('web_loaded_current_layout', current, f'script={script.relative_to(nav_root)}',
               '经 manager 重启 Web 以加载新的脚本路径（需要你批准）')


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


def check_acceptance(report, release):
    record = release / 'physical_acceptance/record_draft.json' if release else None
    try:
        value = json.loads(record.read_text())
    except (OSError, ValueError, AttributeError):
        report.add('physical_acceptance', False, 'record missing', '按物理验收流程采集')
        return
    pending = [flag for flag in ACCEPTANCE_FLAGS if value.get(flag) is not True]
    missing = [row['kind'] for row in value.get('records', []) if not row.get('path')]
    report.add('physical_acceptance', not pending and not missing and bool(value.get('robot_id')),
               f'pending={pending} missing_evidence={missing} robot_id={value.get("robot_id")}',
               '按物理验收流程实测；在此之前 v3 只能监视，不能运动')


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--offline', action='store_true', help='skip robot network checks')
    parser.add_argument('--json', action='store_true')
    args = parser.parse_args()
    report = Report()
    nav_root, app_root = paths.nav_root(), paths.app_root()
    try:
        release = check_release(report, paths.release_root())
    except ValueError as error:
        report.add('release_sealed', False, str(error), '设置 D1MAX_RELEASE')
        release = None
    check_web(report, nav_root)
    check_monitor(report, release, app_root)
    if not args.offline:
        check_network(report, app_root)
    check_acceptance(report, release)
    if args.json:
        print(json.dumps({'ok': report.ok, 'checks': report.rows}, ensure_ascii=False, indent=1))
    else:
        for row in report.rows:
            print(('OK   ' if row['ok'] else 'FAIL ') + f"{row['check']}: {row['detail']}" +
                  (f"\n       → {row['action']}" if row['action'] else ''))
    return 0 if report.ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
