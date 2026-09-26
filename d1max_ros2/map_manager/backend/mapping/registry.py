"""One registry for Web catalog and routing; existing SLAM launchers stay unchanged."""
from dataclasses import dataclass
from pathlib import Path
import threading
from ..runtime import RuntimeManager
from .mola_runtime import MolaRuntime


@dataclass(frozen=True)
class MappingPlugin:
    id: str
    name: str
    frontend: str
    backend: str | None
    input_mode: str = 'live'
    manual_save: bool = True


PLUGINS = (
    MappingPlugin('faster_lio', 'Faster-LIO', 'Faster-LIO', None),
    MappingPlugin('fastlio2', 'FAST-LIO2', 'FAST-LIO2', None),
    MappingPlugin('faster_lio_pgo', 'Faster-LIO + SC-PGO', 'Faster-LIO', 'SC-PGO'),
    MappingPlugin('mola_lio_lc', 'MOLA-LIO + 离线回环', 'MOLA-LIO · GICP + IMU',
                  'FrameToFrame · GICP + GNC', 'rosbag', False),
)
BUSY = {'running', 'stopping', 'starting', 'detached', 'conflict'}


class MappingRuntime:
    def __init__(self, nav_root: Path, state_path: Path, app_root: Path, *, legacy=None, mola=None):
        self.legacy = legacy or RuntimeManager(nav_root, state_path)
        self.mola = mola or MolaRuntime(state_path.parent / 'mola', app_root)
        self.lock = threading.RLock()
        self.adapters = {p.id: self.mola if p.input_mode == 'rosbag' else self.legacy for p in PLUGINS}

    def catalog(self):
        availability = self.mola.availability()
        return [{**p.__dict__, 'available': availability['available'] if p.input_mode == 'rosbag' else True,
                 'unavailable_reason': availability.get('reason') if p.input_mode == 'rosbag' else None}
                for p in PLUGINS]

    def snapshot(self):
        old, new = self.legacy.snapshot(), self.mola.snapshot()
        if old['status'] in BUSY:
            return old
        if new['status'] in BUSY or (new.get('started_at') or '') > (old.get('started_at') or ''):
            return new
        return old

    def start(self, algorithm, options=None):
        with self.lock:
            if algorithm not in self.adapters:
                raise ValueError('未注册的建图方案')
            if any(a.snapshot()['status'] in BUSY for a in (self.legacy, self.mola)):
                raise RuntimeError('请先停止当前建图，并确认进程已退出；不支持运行中切换')
            adapter = self.adapters[algorithm]
            if adapter is self.legacy:
                if options:
                    raise ValueError('当前实时方案不接受离线配置')
                return adapter.start(algorithm)
            return adapter.start(options or {})

    def stop(self, timeout=75.):
        with self.lock:
            if self.mola.snapshot()['status'] in BUSY:
                return self.mola.stop()
            if self.legacy.snapshot()['status'] == 'detached':
                raise RuntimeError('原建图进程归属待核对，不能清空状态后直接重启')
            return self.legacy.stop(timeout=timeout)

    def save(self):
        if self.snapshot().get('algorithm') == 'mola_lio_lc':
            raise RuntimeError('MOLA 离线任务完成后自动保存；取消任务不等于完成建图')
        return self.legacy.save()

    def recover_stale_state(self):
        self.legacy.recover_stale_state()
        self.mola.recover()

    def close(self):
        self.mola.close()
        self.legacy.close()
