"""MOLA task lifecycle, isolated from live SLAM and localization services."""
from contextlib import contextmanager
from datetime import datetime
import fcntl
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import threading
import uuid
from .configuration import parse_config, config_yaml, validate_bag
from .toolchain import availability

UNIT = 'd1max-mola-mapping.service'
MARKER = 'D1MAX_MOLA_V1:'
ACTIVE = {'starting', 'running', 'stopping', 'conflict', 'detached'}


def now():
    return datetime.now().astimezone().isoformat(timespec='seconds')


def atomic_json(path, value):
    path = Path(path)
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2))
    os.replace(temporary, path)


class Systemd:
    def __init__(self, unit=UNIT):
        self.unit = unit

    def show(self):
        result = subprocess.run(['systemctl', '--user', 'show', self.unit,
            '--property=ActiveState,SubState,MainPID,Description,ControlGroup'],
            capture_output=True, text=True, timeout=3)
        if result.returncode and not result.stdout:
            raise RuntimeError('systemd 用户服务不可用')
        return dict(line.split('=', 1) for line in result.stdout.splitlines() if '=' in line)

    def populated(self, unit):
        group = unit.get('ControlGroup', '')
        if not group:
            return False
        path = (Path('/sys/fs/cgroup') / group.lstrip('/')).resolve()
        if not path.is_relative_to('/sys/fs/cgroup'):
            raise RuntimeError('cgroup 路径非法')
        try:
            return 'populated 1' in (path / 'cgroup.events').read_text()
        except FileNotFoundError:
            return False

    def start(self, session, app_root):
        subprocess.run(['systemd-run', '--user', '--collect', '--unit=' + self.unit,
            '--property=Description=' + MARKER + session.name,
            '--property=KillMode=control-group', '--property=KillSignal=SIGINT',
            '--property=TimeoutStopSec=12', '--property=SendSIGKILL=yes',
            '--property=PartOf=d1max-web-managed.service',
            '--property=WorkingDirectory=' + str(app_root),
            '--property=StandardOutput=append:' + str(session / 'runtime.log'),
            '--property=StandardError=append:' + str(session / 'runtime.log'),
            '--setenv=PYTHONPATH=' + str(app_root),
            '--setenv=D1MAX_MOLA_PREFIX=' + availability(app_root)['prefix'],
            sys.executable, '-m', 'backend.mapping.worker', str(session)],
            capture_output=True, text=True, check=True, timeout=8)

    def stop(self):
        subprocess.run(['systemctl', '--user', 'stop', self.unit], capture_output=True,
                       text=True, check=True, timeout=18)


class MolaRuntime:
    def __init__(self, root, app_root, *, system=None):
        self.root, self.app_root = Path(root).resolve(), Path(app_root).resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        self.system = system or Systemd()
        self.lock = threading.RLock()
        self.state_path = self.root / 'state.json'
        self.state = {'status': 'idle', 'algorithm': 'mola_lio_lc', 'id': None}
        self._load()

    def _load(self):
        if self.state_path.is_file():
            value = json.loads(self.state_path.read_text())
            if value.get('id') and not re.fullmatch('[a-f0-9]{32}', value['id']):
                raise ValueError('MOLA 会话标识损坏')
            self.state.update(value)

    @contextmanager
    def transaction(self):
        with self.lock, (self.root / 'lifecycle.lock').open('a') as stream:
            fcntl.flock(stream, fcntl.LOCK_EX)
            self._load()
            yield

    def owned(self, unit):
        return bool(self.state.get('id')) and unit.get('Description') == MARKER + self.state['id']

    def alive(self, unit):
        return unit.get('ActiveState') in {'active', 'activating', 'deactivating'} or self.system.populated(unit)

    def availability(self):
        return availability(self.app_root)

    def snapshot(self):
        with self.lock:
            result = dict(self.state)
            result['logs'] = []
            # No bus calls for a never-started plugin; catalog is read-only.
            if not result.get('id'):
                return result
            session = self.root / result['id']
            try:
                manifest = json.loads((session / 'manifest.json').read_text())
                result.update({k: manifest[k] for k in ('stage', 'progress', 'error', 'artifacts', 'completed_at') if k in manifest})
            except (OSError, ValueError):
                manifest = {}
            try:
                stage = manifest.get('stage', '')
                log = session / f'{stage}.log' if re.fullmatch('[a-z_]+', stage) else session / 'runtime.log'
                if not log.is_file():
                    log = session / 'runtime.log'
                with log.open('rb') as stream:
                    stream.seek(0, 2)
                    stream.seek(max(0, stream.tell() - 12000))
                    result['logs'] = stream.read().decode(errors='replace').splitlines()[-80:]
            except OSError:
                pass
            try:
                unit = self.system.show()
                if self.alive(unit):
                    result['pid'] = int(unit.get('MainPID') or 0)
                    if not self.owned(unit):
                        result.update(status='conflict', error='MOLA 服务被未知任务占用，不会自动清理')
                    else:
                        result['status'] = 'stopping' if unit.get('ActiveState') == 'deactivating' else 'running'
                else:
                    result['pid'] = None
                    result['status'] = manifest.get('status', 'interrupted')
                    if result['status'] not in {'complete', 'failed', 'cancelled'}:
                        result.update(status='failed', error='离线任务已退出，结果未完成；请查看日志')
            except (OSError, RuntimeError, subprocess.SubprocessError) as exc:
                result.update(status='conflict', error=str(exc))
            result['run_directory'] = str(session)
            return result

    def start(self, options):
        if set(options) - {'config_yaml', 'bag_path'}:
            raise ValueError('未知 MOLA 启动选项')
        default = (self.app_root / 'config/mapping/mola_lio_lc.yaml').read_text()
        config = parse_config(options.get('config_yaml') or default, options.get('bag_path'))
        config.input.bag_path = str(validate_bag(config))
        installed = self.availability()
        if not installed['available']:
            raise RuntimeError(installed['reason'])
        with self.transaction():
            unit = self.system.show()
            if self.alive(unit):
                raise RuntimeError('MOLA 任务仍有进程，请先停止；不会重复启动')
            session = self.root / uuid.uuid4().hex
            session.mkdir()
            (session / 'task.yaml').write_text(config_yaml(config))
            self.state = {'status': 'starting', 'algorithm': 'mola_lio_lc',
                          'id': session.name, 'started_at': now(), 'pid': None}
            atomic_json(session / 'manifest.json', {**self.state, 'stage': 'queued', 'progress': 0, 'artifacts': []})
            atomic_json(self.state_path, self.state)
            try:
                self.system.start(session, self.app_root)
            except (OSError, subprocess.SubprocessError):
                self.state.update(status='conflict', error='启动回执异常，请先核对任务状态；禁止自动重试')
                atomic_json(self.state_path, self.state)
                raise RuntimeError(self.state['error']) from None
            return self.snapshot()

    def stop(self):
        with self.transaction():
            unit = self.system.show()
            if self.alive(unit):
                if not self.owned(unit):
                    raise RuntimeError('拒绝停止不属于本模块的进程')
                session = self.root / self.state['id']
                (session / 'cancel.request').touch()
                self.system.stop()
                if self.alive(self.system.show()):
                    raise RuntimeError('MOLA 进程尚未全部退出，禁止切换方案')
            return self.snapshot()

    def recover(self):
        # Recover ownership via persisted UUID + systemd description, never a bare PID.
        return self.snapshot()

    def close(self):
        if self.state.get('id'):
            unit = self.system.show()
            if self.alive(unit) and self.owned(unit):
                self.stop()
