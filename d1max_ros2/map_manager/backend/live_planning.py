"""Web lifecycle for the owned, RViz-driven NO-MOTION live planning view.

Web starts/stops the fixed launcher and reports small session-bound snapshots.
Initial pose and planning goals are deliberately not Web API operations.
"""
from __future__ import annotations

from contextlib import contextmanager
import json
import math
import os
from pathlib import Path
import re
import subprocess
import threading
import time
import yaml

from fastapi import APIRouter, Depends, HTTPException, Request
from pydantic import BaseModel, ConfigDict

from .localization import Systemd


UNIT = 'd1max-live-planning-view.service'
OFFLINE_UNITS = ('d1max-pct-preview.service', 'd1max-pct-scan.service')
OWNER = 'D1MAX-LIVE-PLANNING-VIEW:'
ACTIVE = {'active', 'activating', 'deactivating', 'reloading'}
MODE = 'LIVE_VISUALIZATION_NO_MOTION'
COMPONENT_ROLES = ('localization', 'global_planner', 'local_planner')
COMPONENT_ID = re.compile(r'^[a-z][a-z0-9_-]{0,63}$')
SESSION_NAME = re.compile(r'^20\d{6}_\d{6}_[a-f0-9]{8}$')
ID = re.compile(r'^[a-f0-9]{32}$')
ALLOWED_ORIGINS = {'http://127.0.0.1:8766', 'http://localhost:8766',
                   'http://127.0.0.1:5173', 'http://localhost:5173'}
DESKTOP_KEYS = {'DISPLAY', 'XAUTHORITY', 'WAYLAND_DISPLAY', 'XDG_RUNTIME_DIR',
                'DBUS_SESSION_BUS_ADDRESS', 'LANG'}


class EmptyRequest(BaseModel):
    model_config = ConfigDict(extra='forbid')


def _small_json(path: Path, *, maximum_bytes=65536):
    try:
        if path.is_symlink() or not path.is_file() or path.stat().st_size > maximum_bytes:
            return None
        value = json.loads(path.read_text())
    except (OSError, ValueError, UnicodeError):
        return None
    return value if isinstance(value, dict) else None


def _small_yaml(path: Path, *, maximum_bytes=65536):
    try:
        if path.is_symlink() or not path.is_file() or path.stat().st_size > maximum_bytes:
            return None
        value = yaml.safe_load(path.read_text(encoding='utf-8'))
    except (OSError, ValueError, UnicodeError, yaml.YAMLError):
        return None
    return value if isinstance(value, dict) else None


def _display_text(value, *, maximum):
    if (not isinstance(value, str) or len(value) > maximum
            or any(ord(char) < 32 or ord(char) == 127 for char in value)):
        return None
    return value.strip() or None


def _display_metadata(config):
    """Only fixed YAML or an owned session snapshot may name algorithms."""
    config = config if isinstance(config, dict) else {}
    display = config.get('display')
    display = display if isinstance(display, dict) else {}
    configured = display.get('components')
    configured = configured if isinstance(configured, dict) else {}
    components = {}
    for role in COMPONENT_ROLES:
        item = configured.get(role)
        item = item if isinstance(item, dict) else {}
        identifier = item.get('id')
        name = _display_text(item.get('name'), maximum=80)
        if not isinstance(identifier, str) or not COMPONENT_ID.fullmatch(identifier) or not name:
            components[role] = {'id': None, 'name': '未配置'}
        else:
            components[role] = {'id': identifier, 'name': name}
    floor = config.get('current_floor')
    return {
        'map_name': _display_text(display.get('map_name'), maximum=100) or '未配置地图',
        'components': components,
        'current_floor': floor if floor in ('floor1', 'floor2') else None,
    }


def _fresh(value, *, session_id: str, stamp_field: str, now: float, maximum_age=2.):
    if not isinstance(value, dict) or value.get('session_id') != session_id:
        return {}
    stamp = value.get(stamp_field)
    if (type(stamp) not in (int, float) or not 0 < stamp
            or not -.1 <= now-stamp < maximum_age):
        return {}
    return value


def _stages(phase, health, global_status, scan_status, *, now):
    """Backend-owned stage admission; no stale spline can leave a green tile."""
    if phase != 'running':
        return {role: {'label': '未启动', 'tone': 'waiting', 'expires_at_unix': None}
                for role in COMPONENT_ROLES}
    navigation = health.get('navigation') if isinstance(health.get('navigation'), dict) else {}
    epoch, seed = health.get('local_epoch'), health.get('active_seed_ns')
    nav_stamp = navigation.get('received_at_unix')
    confirmations = health.get('verified_confirmations')
    continuous = health.get('continuous_pose_valid', health.get('localized'))
    health_stamp = health.get('wall_time')
    # Preview is not hardware/navigation qualification. Preserve source context
    # and freshness, but never require or manufacture navigation_ready here.
    locked = (continuous is True and not health.get('local_fault')
              and type(health_stamp) in (int, float) and math.isfinite(health_stamp)
              and -.1 <= now-health_stamp <= .6
              and type(confirmations) is int and confirmations >= 3
              and type(epoch) is int and epoch > 0 and isinstance(seed, str) and bool(seed)
              and seed == health.get('confirmed_seed_ns')
              and (health.get('continuous_pose_valid') is True or navigation.get('valid') is True)
              and not navigation.get('fault')
              and type(navigation.get('epoch')) is int and navigation.get('epoch') == epoch
              and navigation.get('seed_id') == seed
              and type(nav_stamp) in (int, float) and math.isfinite(nav_stamp)
              and -.1 <= now-nav_stamp <= .6)
    def same_context(value):
        return (value.get('session_id') == health.get('session_id')
                and type(value.get('localization_epoch')) is int
                and value.get('localization_epoch') == epoch
                and value.get('localization_seed_id') == seed)
    global_ready = (locked and global_status.get('active_reference') is True
                    and same_context(global_status)
                    and global_status.get('state') in {'shadow_path_published', 'active'})
    spline_stamp = scan_status.get('last_spline_stamp')
    spline_id = scan_status.get('last_spline_id')
    spline_fresh = (type(spline_stamp) in (int, float) and math.isfinite(spline_stamp)
                    and -.1 <= now-spline_stamp <= 2.)
    route = global_status.get('last_route')
    path_stamp = route.get('path_stamp') if isinstance(route, dict) else None
    reference_stamp = scan_status.get('owner_reference_stamp', scan_status.get('reference_stamp'))
    reference_paired = (all(type(value) in (int, float) and math.isfinite(value)
                            for value in (path_stamp, reference_stamp))
                        and abs(path_stamp-reference_stamp) < 1e-6)
    local_ready = (global_ready and scan_status.get('ready') is True
                   and same_context(scan_status) and reference_paired
                   and scan_status.get('active_reference') is True
                   and scan_status.get('local_debug_valid') is True
                   and scan_status.get('spline_visual_valid') is True
                   and type(scan_status.get('local_debug_plan_id')) is int
                   and scan_status.get('local_debug_plan_id') == spline_id
                   and type(spline_id) is int
                   and spline_id >= 0 and spline_fresh)
    loc_state = health.get('state') if isinstance(health.get('state'), str) else None
    frontend = health.get('frontend')
    front = frontend if isinstance(frontend, dict) else {}
    prediction = navigation.get('prediction')
    prediction = prediction if isinstance(prediction, dict) else {}
    gap, warning = front.get('scan_imu_gap_sec'), front.get('imu_gap_warning_sec', .015)
    finite = lambda value: type(value) in (int, float) and math.isfinite(value)
    scan_degraded = (type(front.get('epoch')) is int and front.get('epoch') == epoch
                    and (front.get('degraded') is True
                         or finite(gap) and finite(warning) and 0 <= warning < gap <= 1.))
    prediction_degraded = prediction.get('degraded') is True
    coasting = prediction.get('prediction_mode') == 'coasting'
    degraded = locked and (scan_degraded or prediction_degraded)
    degradation = []
    if degraded and scan_degraded:
        degradation.append(f'当前扫描 IMU 缺口 {gap*1000:.1f} ms'
            if finite(gap) and 0 <= gap <= 1. else '当前扫描 IMU 短时缺帧')
    if degraded and coasting:
        degradation.append('短时运动预测，等待新 IMU 数据')
    elif degraded and prediction_degraded:
        imu_gap = prediction.get('imu_gap')
        predicted_gap = imu_gap.get('max_sec') if isinstance(imu_gap, dict) else None
        if not (finite(predicted_gap) and 0 <= predicted_gap <= 1.):
            predicted_gap = prediction.get('largest_imu_gap_sec')
        degradation.append(f'运动预测 IMU 缺口 {predicted_gap*1000:.1f} ms'
            if finite(predicted_gap) and 0 <= predicted_gap <= 1. else '运动预测短时数据降级')
    pending_confirmation = (health.get('localized') is False and not health.get('local_fault')
        and loc_state in ('acquiring', 'relocalizing')
        and type(epoch) is int and epoch > 0 and isinstance(seed, str)
        and 1 <= len(seed) <= 20 and seed.isdecimal() and int(seed) > 0
        and health.get('confirmed_seed_ns') is None
        and type(confirmations) is int and 0 <= confirmations < 3
        and health.get('frontend_ready') is True and isinstance(frontend, dict)
        and type(frontend.get('epoch')) is int and frontend.get('epoch') == epoch
        and frontend.get('ready') is True and not frontend.get('fault'))
    global_state = (global_status.get('state')
                    if isinstance(global_status.get('state'), str) else None)
    loc_warning = loc_state in {'lost', 'fault', 'navigation_fault', 'degraded', 'output_waiting'}
    global_warning = global_state in {'goal_rejected', 'planning_failed',
                                      'planning_timeout', 'result_rejected'}
    loc_labels = {'waiting_sensors': '待数据', 'calibrating': '初始化中',
                  'recovering_local': '待恢复', 'waiting_initial_pose': '待初值',
                  'acquiring': '定位中', 'filter_initializing': '等待融合输出',
                  'output_waiting': '已定位 · 输出暂缓' if health.get('map_localized') is True else '等待定位输出',
                  'relocalizing': '重定位中', 'lost': '已失锁',
                  'fault': '已暂停', 'navigation_fault': '已暂停',
                  'degraded': '待核对'}
    loc_expiry = min(health_stamp + .6, nav_stamp + .6) if locked else None
    global_expiry = min(loc_expiry, global_status['received_at_unix'] + 2.) if global_ready else None
    local_expiry = min(global_expiry, scan_status['received_at_unix'] + 2.,
                       spline_stamp + 2.) if local_ready else None
    goal_paused = (global_status.get('paused_for_recovery') is True
                   and global_status.get('goal_retained') is True)
    planning_phase = global_status.get('planning_phase')
    expired = planning_phase == 'expired' or global_state in {'expired', 'planning_timeout'}
    computing = (global_status.get('planning') is True
                 or global_status.get('pending_worker_start') is True)
    global_label = ('请求超时' if expired else '需调整' if global_warning
                    else '规划中' if computing
                    else ('路线保留 · 等待定位' if global_status.get('visual_path_available')
                          else '已算完 · 等待定位输出') if planning_phase == 'awaiting_navigation'
                    else '已生成' if global_ready else '等待定位恢复' if goal_paused
                    else '待定位' if not locked else '待目标')
    local_label = ('已生成' if local_ready else '待全局路径' if not global_ready
                   else '规划中' if scan_status.get('ready') is True else '待实时数据')
    local_failures = {'failed': '局部规划失败',
        'failed_reference_search': '未找到绕行通道',
        'failed_rebound_search': '绕障搜索未成功',
        'failed_optimization': '轨迹优化未收敛',
        'failed_final_collision': '轨迹碰撞校验未通过',
        'failed_dynamics': '轨迹动力学未通过',
        'failed_reference_geometry': '参考路径无效',
        'failed_reference_search_budget': '绕行搜索达到上限',
        'failed_reference_target_occupied': '局部目标被占用',
        'failed_reference_start_occupied': '起点包络与占据格重叠',
        'failed_reference_lattice_occupied': '搜索格端点受阻',
        'failed_reference_outside_map': '局部端点超出地图',
        'failed_reference_search_collision': '绕行连接段碰撞',
        'waiting_environment': '受阻 · 等待环境更新',
        'waiting_sensor_map': '等待局部地图',
        'waiting_observed_space': '绕行空间尚未观测',
        'failed_ground_support': '轨迹离开可通行地面',
        'waiting_goal_reached': '已接近局部目标'}
    local_phase = scan_status.get('local_debug_phase')
    local_blocked = (not local_ready and global_ready and same_context(scan_status)
                     and scan_status.get('active_reference') is True
                     and local_phase in local_failures)
    if local_blocked:
        local_label = local_failures[local_phase]
    return {
        'localization': {'label': '已定位' if locked else '匹配确认中' if pending_confirmation
                         else loc_labels.get(loc_state, '待定位'),
                         'tone': 'ready' if locked else 'warning' if loc_warning else 'waiting',
                         'degraded': degraded,
                         'detail': '',
                         'quality_detail': '；'.join(degradation) + '；定位预览仍有效' if degraded else '',
                         'expires_at_unix': loc_expiry},
        'global_planner': {'label': global_label,
                           'tone': 'warning' if expired or goal_paused else 'ready' if global_ready else 'warning' if global_warning and locked else 'waiting',
                           'expires_at_unix': global_expiry},
        'local_planner': {'label': local_label,
                          'tone': 'ready' if local_ready else 'warning' if local_blocked else 'waiting',
                          'detail': str(scan_status.get('last_error') or local_phase) if local_blocked else '',
                          'expires_at_unix': local_expiry},
    }


class LivePlanningRuntime:
    def __init__(self, nav_root, localization_runtime, *, system=None, runner=None):
        self.nav_root = Path(nav_root).resolve()
        self.root = self.nav_root / 'log/live_planning'
        self.script = self.nav_root / 'scripts/planning/start_live_planning_view.sh'
        self.config = self.nav_root / 'src/d1max_pct_scan/config/live_visualization.yaml'
        self.localization_runtime = localization_runtime
        self.system = system or Systemd()
        self.runner = runner or subprocess.run
        self.lock = threading.RLock()
        self.lifecycle_lock = threading.Lock()
        self.operation = None

    @contextmanager
    def _lifecycle(self, phase):
        # Keep start/stop mutually exclusive, but never hold the status mutex
        # across map preparation or a subprocess. Overview must remain usable
        # while the launcher can legitimately take several seconds.
        if not self.lifecycle_lock.acquire(blocking=False):
            raise HTTPException(409, '服务正在启动或停止，请等待当前操作完成')
        try:
            with self.lock:
                self.operation = phase
            yield
        finally:
            with self.lock:
                self.operation = None
            self.lifecycle_lock.release()

    def _owned_session(self, description):
        """Validate unit owner, last-session pointer and fixed child directory."""
        if not isinstance(description, str) or not description.startswith(OWNER):
            return None
        identifier = description[len(OWNER):]
        if not ID.fullmatch(identifier):
            return None
        pointer = _small_json(self.root / 'last_session.json')
        if pointer is None or pointer.get('id') != identifier:
            return None
        directory_value = pointer.get('directory')
        if not isinstance(directory_value, str):
            return None
        directory = Path(directory_value)
        if (not directory.is_absolute() or directory.is_symlink()
                or not SESSION_NAME.fullmatch(directory.name)
                or not directory.name.endswith(identifier[:8])):
            return None
        try:
            resolved = directory.resolve(strict=True)
        except OSError:
            return None
        if resolved.parent != self.root.resolve() or not resolved.is_dir():
            return None
        session = _small_json(resolved / 'session.json')
        if (session is None or session.get('id') != identifier
                or session.get('mode') != MODE
                or session.get('motion_control_enabled') is not False
                or session.get('frame_id') != 'd1max_loc_map'):
            return None
        return identifier, resolved, session

    def _show(self):
        try:
            unit = self.system.show(UNIT)
            busy = unit.get('ActiveState') in ACTIVE or self.system.populated(unit)
        except (OSError, RuntimeError, subprocess.SubprocessError):
            return None, True, '无法确认独立实机可视化服务状态；拒绝重复启动'
        return unit, bool(busy), None

    def _connection(self):
        try:
            return self.localization_runtime.manager().get('monitor', {})
        except HTTPException as error:
            return {'phase': 'unavailable', 'active': False, 'error': str(error.detail)}
        except (OSError, RuntimeError, ValueError, TypeError):
            return {'phase': 'unavailable', 'active': False, 'error': '连接管理器不可用'}

    def snapshot(self, *, connection=True, include_operation=True):
        # Query the manager first so the health/status files are checked as
        # late as possible before returning a snapshot to the browser.
        connection_state = self._connection() if connection else None
        with self.lock:
            unit, busy, error = self._show()
            phase = 'stopped'
            identifier = None
            metadata = _display_metadata(None)
            metadata_source = 'unconfigured'
            health = global_status = scan_status = {}
            view_status = {}
            terminal_status = {}
            if unit is None:
                phase = 'conflict'
            elif busy:
                owned = self._owned_session(unit.get('Description'))
                if owned is None:
                    phase, error = 'conflict', '运行中的会话所有权或状态目录无法核实；不会接管'
                else:
                    identifier, directory, session = owned
                    metadata = _display_metadata(session)
                    metadata_source = 'session_snapshot'
                    state = unit.get('ActiveState')
                    phase = ('running' if state == 'active' else
                             'stopping' if state == 'deactivating' else 'starting')
                    if phase == 'running':
                        now = time.time()
                        health = _fresh(_small_json(directory / 'status.json'),
                                        session_id=identifier, stamp_field='wall_time', now=now)
                        global_status = _fresh(_small_json(directory / 'global_status.json'),
                                               session_id=identifier,
                                               stamp_field='received_at_unix', now=now)
                        view_status = _fresh(_small_json(directory / 'view_status.json'),
                                             session_id=identifier,
                                             stamp_field='received_at_unix', now=now)
                        if (view_status.get('mode') == MODE
                                and view_status.get('motion_enabled') is False):
                            scan_status = _fresh(view_status.get('scan_status'),
                                                 session_id=identifier,
                                                 stamp_field='received_at_unix', now=now)
                        else:
                            view_status = {}
            elif unit is not None:
                metadata = _display_metadata(_small_yaml(self.config))
                metadata_source = 'fixed_config'
                # A collected systemd unit may no longer retain its exit code.
                # Keep bounded terminal evidence, never resurrect live health.
                pointer = _small_json(self.root / 'last_session.json') or {}
                last_id = pointer.get('id', '')
                owned = self._owned_session(OWNER + last_id) if isinstance(last_id, str) else None
                if owned is not None:
                    last_id, directory, _ = owned
                    candidate = _fresh(_small_json(directory / 'runtime_status.json'),
                        session_id=last_id, stamp_field='received_at_unix', now=time.time(),
                        maximum_age=86400.)
                    if (candidate.get('phase') in {'failed', 'stopped'}
                            and candidate.get('mode') == MODE
                            and candidate.get('motion_enabled') is False):
                        terminal_status = candidate
                        if candidate['phase'] == 'failed':
                            phase = 'failed'
                            error = '上次会话已清理：' + str(candidate.get('error') or candidate.get('reason'))[:500]
            now = time.time()
            stages = _stages(phase, health, global_status, scan_status, now=now)
            result = {
                'phase': phase, 'busy': busy, 'installed': self.script.is_file()
                    and os.access(self.script, os.X_OK) and self.config.is_file(),
                'mode': MODE, **metadata, 'metadata_source': metadata_source,
                'snapshot_at_unix': now,
                'motion_enabled': False, 'session_id': identifier, 'error': error,
                'health': health, 'global_status': global_status,
                'scan_status': scan_status, 'view_status': view_status,
                'last_exit': terminal_status,
                'stages': stages,
            }
            if include_operation and self.operation:
                result.update(phase=self.operation, busy=True, error=None,
                    stages=_stages(self.operation, {}, {}, {}, now=now))
        if connection:
            result['connection'] = connection_state
        return result

    def _desktop_environment(self):
        try:
            completed = self.runner(['systemctl', '--user', 'show-environment'],
                                    capture_output=True, text=True, check=True, timeout=5)
        except (OSError, subprocess.SubprocessError):
            raise HTTPException(503, '无法读取桌面会话环境，未启动实机可视化') from None
        desktop = {}
        for line in completed.stdout.splitlines():
            key, separator, value = line.partition('=')
            if separator and key in DESKTOP_KEYS and len(value) <= 4096 and '\x00' not in value:
                desktop[key] = value
        if not desktop.get('DISPLAY'):
            raise HTTPException(409, '未检测到桌面 DISPLAY，请在图形登录会话启动')
        environment = {key: os.environ[key] for key in ('PATH', 'HOME', 'USER', 'LANG')
                       if key in os.environ}
        environment.update(desktop)
        return environment

    def start(self):
        with self._lifecycle('starting'):
            current = self.snapshot(connection=False, include_operation=False)
            if current['busy']:
                raise HTTPException(409, current['error'] or '实机可视化会话已经运行或切换中')
            if not current['installed']:
                raise HTTPException(503, '独立实机可视化程序尚未安装')
            try:
                web = self.system.show('d1max-web-managed.service')
            except (OSError, RuntimeError, subprocess.SubprocessError):
                raise HTTPException(409, '无法核对受托管 Web 进程，不启动实机会话') from None
            try:
                managed_pid = int(web.get('MainPID') or 0)
            except (ValueError, TypeError, AttributeError):
                managed_pid = 0
            if managed_pid != os.getpid() or web.get('ActiveState') != 'active':
                raise HTTPException(409, '请使用受托管 Web 启动，以保证关闭时清理进程')
            for offline_unit in OFFLINE_UNITS:
                try:
                    existing = self.system.show(offline_unit)
                    occupied = (existing.get('ActiveState') in ACTIVE
                                or self.system.populated(existing))
                except (OSError, RuntimeError, subprocess.SubprocessError):
                    raise HTTPException(409, '无法核对离线规划服务状态，拒绝并行启动') from None
                if occupied:
                    raise HTTPException(409, '请先停止离线规划会话')
            environment = self._desktop_environment()
            try:
                self.runner([str(self.script), 'start'], capture_output=True, text=True,
                            check=True, timeout=45, env=environment)
            except subprocess.CalledProcessError as exc:
                # Parse only a fixed, safe preflight diagnostic. Launcher
                # stderr can contain credentials; never return its raw text.
                tail = exc.stderr[-8192:] if isinstance(exc.stderr, str) else ''
                missing = re.search(r'定位程序安装不完整：缺少模块 '
                    r'(d1max_localization(?:\.[A-Za-z_][A-Za-z0-9_]*){1,8})(?=[；\s]|$)', tail)
                if missing:
                    raise HTTPException(503, '定位程序安装不完整：缺少模块 '+missing.group(1)
                        +'；请重新构建 d1max_localization') from None
                raise HTTPException(503, '启动失败，请核对实机可视化日志') from None
            except (OSError, subprocess.SubprocessError):
                # Launcher diagnostics may contain the local manager credential.
                raise HTTPException(503, '启动失败，请核对实机可视化日志') from None
            return self.snapshot(include_operation=False)

    def stop(self):
        with self._lifecycle('stopping'):
            current = self.snapshot(connection=False, include_operation=False)
            if not current['busy']:
                return self.snapshot(include_operation=False)
            if current['phase'] == 'conflict' or not current['session_id']:
                raise HTTPException(409, '会话所有权未核实，拒绝停止其他进程')
            try:
                self.runner([str(self.script), 'stop'], capture_output=True, text=True,
                            check=True, timeout=30)
            except (OSError, subprocess.SubprocessError):
                raise HTTPException(503, '停止未确认；请核对服务状态') from None
            after = self.snapshot(connection=False, include_operation=False)
            if after['busy']:
                raise HTTPException(409, '实机可视化进程组尚未清空，禁止重复启动')
            return self.snapshot(include_operation=False)

    def close(self):
        try:
            self.stop()
        except (HTTPException, OSError, RuntimeError, subprocess.SubprocessError):
            pass  # The systemd unit is also BindsTo/PartOf the managed Web unit.


def create_live_planning_router(runtime, shared_lock, grid_lock, other_busy, *, retired=False):
    @contextmanager
    def try_lock(lock):
        if not lock.acquire(blocking=False):
            raise HTTPException(409, '另一个操作正在处理，请稍后重试')
        try:
            yield
        finally:
            lock.release()

    def check_write(request: Request):
        if request.method in {'GET', 'HEAD', 'OPTIONS'}:
            return
        if request.headers.get('content-type', '').split(';')[0] != 'application/json':
            raise HTTPException(415, '实机可视化操作必须使用 JSON 请求')
        origin = request.headers.get('origin')
        if origin and origin not in ALLOWED_ORIGINS:
            raise HTTPException(403, '不允许来自其他网站的实机可视化操作')

    router = APIRouter(prefix='/api/live-planning', dependencies=[Depends(check_write)])

    @router.get('/overview')
    def overview():
        return runtime.snapshot()

    @router.post('/start', status_code=202)
    def start(_request: EmptyRequest):
        if retired:
            raise HTTPException(410, '旧预览入口已停用，请使用“定位与规划”的主线入口')
        with try_lock(shared_lock), try_lock(grid_lock):
            if other_busy():
                raise HTTPException(409, '请先结束建图、录包、点云处理、旧导航或旧定位会话')
            return runtime.start()

    @router.post('/stop')
    def stop(_request: EmptyRequest):
        return runtime.stop()

    return router
