"""Local Web boundary for the owned Nav2 session; never talks to the robot SDK.

The ROS command client repeats session, freshness, and motion admission checks.
No browser-supplied executable, ROS name, filesystem path, or speed is accepted.
"""
from __future__ import annotations

import json
from pathlib import Path
import subprocess
import threading
import time
from typing import Literal

from fastapi import APIRouter, Depends, HTTPException, Request
from pydantic import BaseModel, ConfigDict, Field

from .grid_maps import atomic_json

OWNER = 'D1MAX_NAVIGATION_V1:'


class StartNavigation(BaseModel):
    model_config = ConfigDict(extra='forbid')
    version_id: str = Field(pattern=r'^grid-[a-f0-9]{24}$')
    mode: Literal['sim', 'live'] = 'sim'
    enable_motion: bool = False
    show_rviz: bool = True


class NavigationCommand(BaseModel):
    model_config = ConfigDict(extra='forbid', allow_inf_nan=False)
    operation: Literal['arm', 'disarm']
    session_id: str = Field(min_length=1, max_length=160, pattern=r'^[a-zA-Z0-9_-]+$')
    version_id: str = Field(pattern=r'^grid-[a-f0-9]{24}$')


class NavigationRuntime:
    def __init__(self, root, nav_root, localization, *, runner=subprocess.run):
        self.root = Path(root).resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        self.script = Path(nav_root).resolve() / 'start_navigation.sh'
        self.localization = localization
        self.runner = runner
        self.lock = threading.RLock()
        self.owner_path = self.root / 'web_session.json'
        self.health_cache = ({}, 0., None)

    def call(self, *arguments, json_result=True):
        if not self.script.is_file():
            raise HTTPException(409, 'Nav2 启动程序尚未安装')
        try:
            result = self.runner(['/usr/bin/bash', str(self.script), *arguments],
                                 text=True, capture_output=True, timeout=35)
        except subprocess.TimeoutExpired:
            raise HTTPException(503, 'Nav2 操作回执超时；请核对状态，没有自动重试') from None
        except OSError:
            raise HTTPException(503, 'Nav2 启动程序不可用') from None
        if result.returncode:
            detail = (result.stderr or result.stdout or 'Nav2 操作失败').strip()[-1500:]
            raise HTTPException(409, detail)
        if not json_result:
            return {}
        try:
            value = json.loads(result.stdout)
            if not isinstance(value, dict):
                raise ValueError('object required')
            return value
        except ValueError:
            raise HTTPException(503, 'Nav2 回执格式异常；请核对会话状态') from None

    def snapshot(self):
        with self.lock:
            return self._snapshot()

    def _snapshot(self):
        try:
            raw = self.call('status')
            unit = raw.get('unit') or {}
            session = raw.get('last_session') or {}
            active = unit.get('ActiveState') in {'active', 'activating', 'deactivating'}
            active = active or raw.get('process_group_populated') is True
            owned = str(unit.get('Description', '')).startswith(OWNER)
            phase = ('conflict' if not owned else
                     'stopping' if unit.get('ActiveState') == 'deactivating' else
                     'starting' if unit.get('ActiveState') == 'activating' else 'running') if active else 'stopped'
            # Reuse only a very short read-only probe. Every motion operation
            # still performs authoritative ROS-side checks with a new request.
            session_id = session.get('navigation_session_id') or session.get('session_id')
            health = {}
            if active and owned:
                cached, stamp, cached_id = self.health_cache
                if cached_id == session_id and 0 <= time.monotonic()-stamp < 1.5:
                    health = cached
                else:
                    try:
                        health = self.call('command', '--operation', 'status')
                    except HTTPException as error:
                        health = {'error': str(error.detail)}
                    self.health_cache = (health, time.monotonic(), session_id)
            return {'phase': phase, 'busy': active, 'installed': True,
                    'session_id': session_id,
                    'version_id': session.get('version_id'), 'map_name': session.get('map_name'),
                    'mode': session.get('mode'), 'enable_motion': session.get('enable_motion') is True,
                    'show_rviz': session.get('headless') is False,
                    'health': health, 'error': raw.get('error'),
                    'max_speed_mps': min(1.5, float(session.get('max_speed_mps', 1.5))),
                    'unit': unit}
        except (HTTPException, TypeError, ValueError) as error:
            return {'phase': 'unavailable', 'busy': True, 'installed': self.script.is_file(),
                    'health': {}, 'error': str(getattr(error, 'detail', error)), 'max_speed_mps': 1.5}

    @property
    def pinned_id(self):
        current = self.snapshot()
        if current['phase'] in {'unavailable', 'conflict'}:
            raise HTTPException(409, '无法确认导航会话，禁止切换或清理地图；请先核对 Nav2 状态')
        return current.get('version_id') if current['busy'] else None

    def start(self, request, version):
        with self.lock:
            current = self.snapshot()
            if current['busy']:
                raise HTTPException(409, 'Nav2 已运行或状态待核对，禁止重复启动')
            if not version['selected'] or version['archived'] or not version['complete']:
                raise HTTPException(409, '请选择完整且未归档的单层地图')
            if request.mode == 'sim' and request.enable_motion:
                raise HTTPException(422, '离线仿真禁止启用 SDK 运动能力')
            if request.mode == 'live':
                # Never let the CLI recursively start localization while this
                # request holds the Web map/lifecycle lock.
                local = self.localization.snapshot()
                if local.get('phase') != 'running' or local.get('version_id') != request.version_id:
                    raise HTTPException(409, '先在定位页启动同一地图的实机定位，再启动 Nav2')
                connection = local.get('connection') or {}
                health = connection.get('health') or {}
                if not (connection.get('active') and health.get('sdk_fresh') and health.get('lidar_fresh')
                        and health.get('replay') is False):
                    raise HTTPException(409, '实机导航需要新鲜的 SDK 和雷达数据；禁止用回放代替')
            args = ['start', '--mode', request.mode, '--web-owned']
            if not request.show_rviz:
                args.append('--headless')
            if request.enable_motion:
                args.append('--enable-motion')
            result = self.call(*args)
            self.health_cache = ({}, 0., None)
            atomic_json(self.owner_path, {'session_id': result.get('navigation_session_id') or result.get('session_id'),
                                         'session': result.get('session'), 'version_id': request.version_id})
            return self.snapshot()

    def stop(self):
        with self.lock:
            self.call('stop', json_result=False)
            self.health_cache = ({}, 0., None)
            current = self.snapshot()
            if current['busy']:
                raise HTTPException(409, 'Nav2 进程组仍存在；禁止重复启动')
            return current

    def command(self, request):
        with self.lock:
            current = self.snapshot()
            if (current['phase'] != 'running' or current.get('session_id') != request.session_id
                    or current.get('version_id') != request.version_id):
                raise HTTPException(409, '导航会话或地图已变化，请刷新后重新操作')
            if request.operation == 'arm' and (current.get('mode') != 'live' or not current['enable_motion']):
                raise HTTPException(409, '此会话没有实机运动能力；仿真不需要解锁')
            # The ROS client is authoritative for fresh gate/SDK state. In
            # particular, a status screen is never an authorization token.
            args = ['command', '--operation', request.operation, '--session-id', request.session_id,
                    '--version-id', request.version_id]
            result = self.call(*args)
            self.health_cache = ({}, 0., None)
            return result

    def close(self):
        # Do not stop a terminal-created session merely because Web closes.
        try:
            owner = json.loads(self.owner_path.read_text())
            current = self.snapshot()
            if owner.get('session_id') and current.get('session_id') == owner['session_id'] and current['busy']:
                self.stop()
        except (OSError, ValueError, HTTPException):
            pass


def create_navigation_router(runtime, grid, shared_lock, other_busy):
    def check_write(request: Request):
        if request.method in {'GET', 'HEAD', 'OPTIONS'}:
            return
        if request.headers.get('content-type', '').split(';')[0] != 'application/json':
            raise HTTPException(415, '导航操作必须使用 JSON 请求')
        origin = request.headers.get('origin')
        if origin and origin not in {'http://127.0.0.1:8766', 'http://localhost:8766',
                                     'http://127.0.0.1:5173', 'http://localhost:5173'}:
            raise HTTPException(403, '不允许来自其他网站的导航操作')

    router = APIRouter(prefix='/api/navigation', dependencies=[Depends(check_write)])

    @router.get('/overview')
    def overview():
        return runtime.snapshot()

    @router.post('/start', status_code=202)
    def start(request: StartNavigation):
        with shared_lock, grid.lock:
            if other_busy() or grid.job['running']:
                raise HTTPException(409, '请先结束建图、数据采集或地图处理任务')
            return runtime.start(request, grid.item(request.version_id))

    @router.post('/stop')
    def stop():
        return runtime.stop()

    @router.post('/command')
    def command(request: NavigationCommand):
        return runtime.command(request)

    return router
