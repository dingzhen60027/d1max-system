"""Offline Web lifecycle tests: no systemd operations or robot connection."""
import json
from copy import deepcopy
import os
from pathlib import Path
import subprocess
import tempfile
import threading
import time
import unittest
from fastapi import FastAPI, HTTPException
from fastapi.testclient import TestClient

from backend.live_planning import (
    LivePlanningRuntime, MODE, OWNER, UNIT, create_live_planning_router, _stages,
)


IDENTIFIER = 'a' * 32


class FakeSystem:
    def __init__(self):
        self.live = {'ActiveState': 'inactive', 'Description': '', 'MainPID': '0'}
        self.web = {'ActiveState': 'active', 'MainPID': str(os.getpid())}
        self.offline = {'ActiveState': 'inactive'}
        self.fail = False
        self.residual = False

    def show(self, unit):
        if self.fail:
            raise RuntimeError('systemd unavailable')
        if unit == 'd1max-web-managed.service':
            return self.web
        if unit in ('d1max-pct-preview.service', 'd1max-pct-scan.service'):
            return self.offline
        return self.live

    def populated(self, unit):
        return self.residual


class FakeLocalization:
    def manager(self):
        return {'monitor': {'active': True, 'health': {'sdk_fresh': False}}}


class LivePlanningTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix='d1max-live-web-test-')
        self.addCleanup(temporary.cleanup)
        self.nav = Path(temporary.name)
        self.system = FakeSystem()
        self.calls = []
        self.runtime = LivePlanningRuntime(self.nav, FakeLocalization(),
                                           system=self.system, runner=self.runner)
        self.runtime.script.write_text('#!/bin/sh\nexit 0\n')
        self.runtime.script.chmod(0o700)
        self.runtime.config.parent.mkdir(parents=True)
        self.runtime.config.write_text('''mode: LIVE_VISUALIZATION_NO_MOTION
current_floor: floor1
display:
  map_name: 固定配置地图
  components:
    localization: {id: lio_pcd, name: LIO 定位}
    global_planner: {id: pct_native, name: PCT 全局}
    local_planner: {id: scan_planner, name: SCAN 局部}
''')

    def write_session(self, *, display=None, floor='floor1'):
        directory = self.runtime.root / ('20260924_120000_' + IDENTIFIER[:8])
        directory.mkdir(parents=True, exist_ok=True)
        session = {'id': IDENTIFIER, 'mode': MODE, 'motion_control_enabled': False,
                   'frame_id': 'd1max_loc_map', 'current_floor': floor}
        if display is not None:
            session['display'] = display
        (directory / 'session.json').write_text(json.dumps(session))
        self.runtime.root.mkdir(parents=True, exist_ok=True)
        (self.runtime.root / 'last_session.json').write_text(json.dumps({
            'id': IDENTIFIER, 'directory': str(directory)}))
        self.system.live = {'ActiveState': 'active', 'Description': OWNER + IDENTIFIER,
                            'MainPID': '234'}
        return directory

    def runner(self, arguments, **kwargs):
        self.calls.append((arguments, kwargs))
        if arguments == ['systemctl', '--user', 'show-environment']:
            return subprocess.CompletedProcess(arguments, 0,
                stdout='DISPLAY=:0\nXAUTHORITY=/tmp/d1max-Xauthority\nSECRET_CREDENTIAL=private\n')
        if arguments == [str(self.runtime.script), 'start']:
            self.write_session()
        elif arguments == [str(self.runtime.script), 'stop']:
            self.system.live = {'ActiveState': 'inactive', 'Description': '', 'MainPID': '0'}
        return subprocess.CompletedProcess(arguments, 0, stdout='')

    def app(self, busy=lambda: False):
        app = FastAPI()
        app.include_router(create_live_planning_router(self.runtime,
                           threading.RLock(), threading.RLock(), busy))
        return app

    def test_owned_running_session_and_fresh_status_only(self):
        directory = self.write_session()
        now = time.time()
        (directory / 'status.json').write_text(json.dumps({
            'session_id': IDENTIFIER, 'wall_time': now, 'state': 'localized'}))
        (directory / 'global_status.json').write_text(json.dumps({
            'session_id': IDENTIFIER, 'received_at_unix': now, 'state': 'path_ready'}))
        scan = {'session_id': IDENTIFIER, 'received_at_unix': now, 'ready': True}
        (directory / 'view_status.json').write_text(json.dumps({
            'session_id': IDENTIFIER, 'received_at_unix': now, 'mode': MODE,
            'motion_enabled': False, 'scan_status': scan}))
        status = self.runtime.snapshot()
        self.assertEqual(status['phase'], 'running')
        self.assertEqual(status['health']['state'], 'localized')
        self.assertEqual(status['global_status']['state'], 'path_ready')
        self.assertTrue(status['scan_status']['ready'])
        self.assertFalse(status['motion_enabled'])
        self.assertTrue(status['connection']['active'])
        (directory / 'status.json').write_text(json.dumps({
            'session_id': IDENTIFIER, 'wall_time': now - 10, 'state': 'localized'}))
        (directory / 'view_status.json').write_text(json.dumps({
            'session_id': IDENTIFIER, 'received_at_unix': now, 'mode': MODE,
            'motion_enabled': False, 'scan_status': dict(scan, session_id='other')}))
        stale = self.runtime.snapshot(connection=False)
        self.assertEqual(stale['health'], {})
        self.assertEqual(stale['scan_status'], {})

    def test_config_metadata_is_generic_and_session_snapshot_takes_priority(self):
        stopped = self.runtime.snapshot(connection=False)
        self.assertEqual(stopped['metadata_source'], 'fixed_config')
        self.assertEqual(stopped['map_name'], '固定配置地图')
        self.assertEqual(stopped['components']['local_planner'],
                         {'id': 'scan_planner', 'name': 'SCAN 局部'})
        self.assertEqual(stopped['current_floor'], 'floor1')
        self.assertGreater(stopped['snapshot_at_unix'], 0)
        self.write_session(display={'map_name': '会话快照地图', 'components': {
            'localization': {'id': 'alternative_lio', 'name': '另一定位实现'},
            'global_planner': {'id': 'new_global', 'name': '另一全局规划'},
            'local_planner': {'id': 'new_local', 'name': '另一局部规划'}}}, floor='floor2')
        running = self.runtime.snapshot(connection=False)
        self.assertEqual(running['metadata_source'], 'session_snapshot')
        self.assertEqual(running['map_name'], '会话快照地图')
        self.assertEqual(running['current_floor'], 'floor2')
        self.assertEqual(running['components']['global_planner']['id'], 'new_global')
        self.system.live['ActiveState'] = 'inactive'
        self.assertEqual(self.runtime.snapshot(connection=False)['map_name'], '固定配置地图')

    def test_missing_or_invalid_metadata_never_invents_an_algorithm(self):
        self.write_session()  # An older owned session has no display snapshot.
        running = self.runtime.snapshot(connection=False)
        self.assertEqual(running['map_name'], '未配置地图')
        self.assertEqual(running['components']['global_planner'],
                         {'id': None, 'name': '未配置'})
        self.system.live['ActiveState'] = 'inactive'
        self.runtime.config.write_text('x' * 65537)
        stopped = self.runtime.snapshot(connection=False)
        self.assertEqual(stopped['map_name'], '未配置地图')
        self.assertIsNone(stopped['current_floor'])
        self.assertTrue(all(value['name'] == '未配置'
                            for value in stopped['components'].values()))

    def test_stage_green_requires_fresh_lock_global_reference_and_spline(self):
        directory = self.write_session()
        now = time.time()
        health = {'session_id': IDENTIFIER, 'wall_time': now,
                  'state': 'tracking', 'localized': True, 'navigation_ready': False,
                  'local_epoch': 2, 'active_seed_ns': 'seed', 'confirmed_seed_ns': 'seed',
                  'verified_confirmations': 3,
                  'navigation': {'valid': True, 'navigation_ready': False,
                                 'epoch': 2, 'seed_id': 'seed', 'received_at_unix': now}}
        global_status = {'session_id': IDENTIFIER, 'received_at_unix': now,
                         'localization_epoch': 2, 'localization_seed_id': 'seed',
                         'navigation_ready': False, 'last_route': {'path_stamp': now},
                         'state': 'shadow_path_published', 'active_reference': True}
        scan = {'session_id': IDENTIFIER, 'received_at_unix': now,
                'localization_epoch': 2, 'localization_seed_id': 'seed',
                'local_debug_valid': True, 'spline_visual_valid': True,
                'local_debug_plan_id': 7, 'reference_stamp': now,
                'ready': True, 'active_reference': True,
                'last_spline_id': 7, 'last_spline_stamp': now}
        def publish():
            (directory / 'status.json').write_text(json.dumps(health))
            (directory / 'global_status.json').write_text(json.dumps(global_status))
            (directory / 'view_status.json').write_text(json.dumps({
                'session_id': IDENTIFIER, 'received_at_unix': now, 'mode': MODE,
                'motion_enabled': False, 'scan_status': scan}))
            return self.runtime.snapshot(connection=False)['stages']
        stages = publish()
        self.assertEqual([stages[key]['tone'] for key in
                          ('localization', 'global_planner', 'local_planner')],
                         ['ready', 'ready', 'ready'])
        self.assertEqual([stages[key]['label'] for key in
                          ('localization', 'global_planner', 'local_planner')],
                         ['已定位', '已生成', '已生成'])
        self.assertTrue(all(now < stage['expires_at_unix'] <= now + 2.
                            for stage in stages.values()))
        for phase, label in (('waiting_observed_space', '绕行空间尚未观测'),
                             ('failed_reference_start_occupied', '起点包络与占据格重叠'),
                             ('failed_reference_lattice_occupied', '搜索格端点受阻'),
                             ('failed_reference_outside_map', '局部端点超出地图'),
                             ('failed_ground_support', '轨迹离开可通行地面')):
            saved = scan.copy()
            scan.update(spline_visual_valid=False, local_debug_phase=phase)
            self.assertEqual(publish()['local_planner']['label'], label)
            self.assertNotEqual(publish()['local_planner']['tone'], 'ready')
            scan.clear(); scan.update(saved)
        health['frontend'] = {'epoch': 2, 'scan_imu_gap_sec': .07, 'imu_gap_warning_sec': .015}
        self.assertEqual(publish()['localization']['label'], '已定位')
        self.assertEqual(publish()['localization']['tone'], 'ready')
        self.assertEqual(publish()['localization']['detail'], '')
        self.assertTrue(publish()['localization']['degraded'])
        self.assertEqual(publish()['local_planner']['tone'], 'ready')
        health['frontend'].update(scan_imu_gap_sec=.01, max_observed_gap=.5, degraded=False)
        self.assertEqual(publish()['localization']['label'], '已定位')
        health['navigation']['prediction'] = {'degraded': True, 'largest_imu_gap_sec': .06}
        self.assertIn('60.0 ms', publish()['localization']['quality_detail'])
        health['navigation']['prediction']['imu_gap'] = {'count': 1, 'max_sec': .07,
            'max_rotation_rad': .01, 'integrated_sec': .07, 'rejected_reason': ''}
        self.assertIn('70.0 ms', publish()['localization']['quality_detail'])
        self.assertNotIn('60.0 ms', publish()['localization']['quality_detail'])
        for malformed in (None, [], {'max_sec': True}, {'max_sec': float('nan')}, {'max_sec': 8.}):
            health['navigation']['prediction']['imu_gap'] = malformed
            self.assertIn('60.0 ms', publish()['localization']['quality_detail'])
        health['navigation']['prediction']['imu_gap'] = {'max_sec': .07, 'count': 1}
        health['navigation']['prediction']['degraded'] = False
        self.assertFalse(publish()['localization']['degraded'])
        for changed in ({'active_reference': False}, {'ready': False},
                        {'last_spline_id': -1}, {'last_spline_stamp': now - 3.},
                        {'localization_epoch': 1}, {'localization_seed_id': 'old'},
                        {'local_debug_valid': False}, {'spline_visual_valid': False},
                        {'local_debug_plan_id': 6}, {'reference_stamp': now - 1.}):
            saved = scan.copy()
            scan.update(changed)
            self.assertNotEqual(publish()['local_planner']['tone'], 'ready')
            scan.clear(); scan.update(saved)
        for changed in ({'localization_epoch': 1}, {'localization_seed_id': 'old'}):
            saved = global_status.copy()
            global_status.update(changed)
            self.assertNotEqual(publish()['global_planner']['tone'], 'ready')
            self.assertNotEqual(publish()['local_planner']['tone'], 'ready')
            global_status.clear(); global_status.update(saved)
        for changed in ({'verified_confirmations': 2}, {'active_seed_ns': 'new'},
                        {'local_epoch': 3}, {'local_fault': 'imu_gap'}):
            saved = health.copy()
            health.update(changed)
            self.assertTrue(all(stage['tone'] != 'ready' for stage in publish().values()))
            health.clear(); health.update(saved)
        for changed in ({'epoch': 1}, {'seed_id': 'old'}, {'received_at_unix': now - 1.},
                        {'fault': 'reset'}):
            saved = health['navigation'].copy()
            health['navigation'].update(changed)
            self.assertTrue(all(stage['tone'] != 'ready' for stage in publish().values()))
            health['navigation'].clear(); health['navigation'].update(saved)
        global_status['active_reference'] = False
        self.assertNotEqual(publish()['global_planner']['tone'], 'ready')
        global_status['active_reference'] = True
        health['navigation']['valid'] = False
        self.assertEqual([stage['tone'] for stage in publish().values()],
                         ['waiting', 'waiting', 'waiting'])
        health['navigation']['valid'] = True
        health['wall_time'] = now - 10.
        self.assertNotEqual(publish()['localization']['tone'], 'ready')
        health['wall_time'] = now
        self.system.live['ActiveState'] = 'inactive'
        self.assertEqual([stage['label'] for stage in publish().values()],
                         ['未启动', '未启动', '未启动'])

    def test_unknown_status_shapes_do_not_break_the_display_adapter(self):
        directory = self.write_session()
        now = time.time()
        (directory / 'status.json').write_text(json.dumps({
            'session_id': IDENTIFIER, 'wall_time': now, 'state': [], 'navigation': []}))
        (directory / 'global_status.json').write_text(json.dumps({
            'session_id': IDENTIFIER, 'received_at_unix': now, 'state': {}}))
        stages = self.runtime.snapshot(connection=False)['stages']
        self.assertTrue(all(stage['tone'] != 'ready' for stage in stages.values()))

    def test_valid_continuous_pose_has_one_label_without_hiding_real_expiry_or_faults(self):
        health = {'session_id': IDENTIFIER, 'wall_time': 99.9, 'state': 'tracking_degraded',
                  'localized': False, 'map_localized': False, 'continuous_pose_valid': True,
                  'local_epoch': 2, 'active_seed_ns': 'seed', 'confirmed_seed_ns': 'seed',
                  'verified_confirmations': 3, 'navigation_ready': False,
                  'navigation': {'valid': False, 'epoch': 2, 'seed_id': 'seed',
                                 'received_at_unix': 99.9, 'navigation_ready': False,
                                 'calibration': {'extrinsics_verified': False,
                                                 'time_alignment_verified': False}}}
        original = deepcopy(health)
        for prediction in ({}, {'prediction_mode': 'coasting', 'degraded': True},
                           {'degraded': True, 'largest_imu_gap_sec': .07}):
            current = deepcopy(health)
            current['navigation']['prediction'] = prediction
            before = deepcopy(current)
            stages = _stages('running', current, {}, {}, now=100.)
            loc = stages['localization']
            self.assertEqual((loc['label'], loc['tone'], loc['detail']), ('已定位', 'ready', ''))
            self.assertLessEqual(loc['expires_at_unix'], 100.5)
            self.assertEqual(current, before)  # No status or acceptance mutation.
            self.assertNotEqual(stages['global_planner']['tone'], 'ready')
            self.assertNotEqual(stages['local_planner']['tone'], 'ready')
        self.assertEqual(health, original)
        for change in ({'continuous_pose_valid': False, 'localized': True},
                       {'wall_time': 99.39}, {'local_epoch': 3},
                       {'active_seed_ns': 'new'}, {'verified_confirmations': 2},
                       {'local_fault': 'imu_gap_hard_limit'}):
            current = deepcopy(health); current.update(change)
            self.assertNotEqual(_stages('running', current, {}, {}, now=100.)['localization']['tone'], 'ready')
        for change in ({'fault': 'filter_jump'}, {'seed_id': 'old'}, {'epoch': 1},
                       {'received_at_unix': 99.39}):
            current = deepcopy(health); current['navigation'].update(change)
            self.assertNotEqual(_stages('running', current, {}, {}, now=100.)['localization']['tone'], 'ready')
        waiting = deepcopy(health)
        waiting.update(continuous_pose_valid=False, state='output_waiting', map_localized=True)
        paused = _stages('running', waiting, {}, {}, now=100.)['localization']
        self.assertEqual(paused['label'], '已定位 · 输出暂缓')
        self.assertNotEqual(paused['tone'], 'ready')

    def test_seed_confirmation_and_epoch_reset_are_distinct_display_states(self):
        health = {'session_id': IDENTIFIER, 'wall_time': 100., 'localized': False,
                  'initial_pose_ready': True, 'state': 'acquiring', 'local_epoch': 2,
                  'active_seed_ns': '100000000000', 'confirmed_seed_ns': None,
                  'verified_confirmations': 0, 'frontend_ready': True,
                  'frontend': {'epoch': 2, 'ready': True}}
        def label(value):
            return _stages('running', value, {}, {}, now=100.)['localization']['label']
        self.assertEqual(label(health), '匹配确认中')
        for change in ({'local_epoch': 3}, {'verified_confirmations': True},
                       {'active_seed_ns': 'bad'}, {'state': 'waiting_sensors'},
                       {'frontend_ready': False}, {'confirmed_seed_ns': 'old'}):
            self.assertNotEqual(label(dict(health, **change)), '匹配确认中')
        self.assertEqual(label(dict(health, state='waiting_initial_pose', local_epoch=3,
                                    active_seed_ns=None)), '待初值')
        self.assertEqual(label(dict(health, state='filter_initializing')), '等待融合输出')
        paused = dict(health, state='output_waiting', map_localized=True,
                      navigation={'valid': False, 'state': 'waiting_local'})
        self.assertEqual(label(paused), '已定位 · 输出暂缓')
        stages = _stages('running', paused, {}, {}, now=100.)
        self.assertTrue(all(stage['tone'] != 'ready' for stage in stages.values()))

    def test_old_or_unowned_unit_never_reports_health_or_stops(self):
        directory = self.write_session()
        (directory / 'status.json').write_text(json.dumps({
            'session_id': IDENTIFIER, 'wall_time': time.time(), 'state': 'localized'}))
        self.system.live['Description'] = OWNER + 'b' * 32
        result = self.runtime.snapshot(connection=False)
        self.assertEqual(result['phase'], 'conflict')
        self.assertEqual(result['health'], {})
        with self.assertRaises(HTTPException):
            self.runtime.stop()
        self.assertEqual(self.calls, [])

    def test_terminal_failure_is_visible_but_never_resurrects_health(self):
        directory = self.write_session()
        now = time.time()
        (directory / 'runtime_status.json').write_text(json.dumps({
            'session_id': IDENTIFIER, 'received_at_unix': now, 'phase': 'failed',
            'mode': MODE, 'motion_enabled': False, 'error': 'scan exited with code 2',
            'cleanup': {'all_direct_children_exited': True}}))
        (directory / 'status.json').write_text(json.dumps({
            'session_id': IDENTIFIER, 'wall_time': now, 'localized': True}))
        self.system.live = {'ActiveState': 'inactive', 'Description': '', 'MainPID': '0'}
        result = self.runtime.snapshot(connection=False)
        self.assertEqual(result['phase'], 'failed')
        self.assertIn('scan exited', result['error'])
        self.assertFalse(result['busy'])
        self.assertEqual(result['health'], {})
        self.assertIsNone(result['session_id'])
        # Failure requires a new explicit start, it never auto-restarts.
        self.assertEqual(self.calls, [])
        self.assertEqual(self.runtime.start()['phase'], 'running')

    def test_start_stop_fixed_command_and_environment_whitelist(self):
        started = self.runtime.start()
        self.assertEqual(started['session_id'], IDENTIFIER)
        self.assertEqual(started['phase'], 'running')
        self.assertEqual(self.calls[1][0], [str(self.runtime.script), 'start'])
        env = self.calls[1][1]['env']
        self.assertEqual(env['DISPLAY'], ':0')
        self.assertEqual(env['XAUTHORITY'], '/tmp/d1max-Xauthority')
        self.assertNotIn('SECRET_CREDENTIAL', env)
        with self.assertRaises(HTTPException) as caught:
            self.runtime.start()
        self.assertEqual(caught.exception.status_code, 409)
        stopped = self.runtime.stop()
        self.assertEqual(stopped['phase'], 'stopped')
        self.assertEqual(self.calls[-1][0], [str(self.runtime.script), 'stop'])
        self.assertIsNone(stopped['session_id'])

    def test_start_rejects_nonmanaged_web_and_unknown_state(self):
        self.system.web['MainPID'] = '99999'
        with self.assertRaises(HTTPException) as caught:
            self.runtime.start()
        self.assertEqual(caught.exception.status_code, 409)
        self.assertEqual(self.calls, [])
        self.system.web['MainPID'] = 'not-a-pid'
        with self.assertRaises(HTTPException) as caught:
            self.runtime.start()
        self.assertEqual(caught.exception.status_code, 409)
        self.system.fail = True
        self.assertEqual(self.runtime.snapshot(connection=False)['phase'], 'conflict')
        with self.assertRaises(HTTPException):
            self.runtime.start()
        self.assertEqual(self.calls, [])

    def test_offline_pct_session_blocks_live_start(self):
        self.system.offline['ActiveState'] = 'activating'
        with self.assertRaises(HTTPException) as caught:
            self.runtime.start()
        self.assertEqual(caught.exception.status_code, 409)
        self.assertEqual(self.calls, [])

    def test_launcher_failure_never_exposes_credential(self):
        def failing_runner(arguments, **kwargs):
            if arguments[0] == 'systemctl':
                return self.runner(arguments, **kwargs)
            raise subprocess.CalledProcessError(1, arguments,
                                                stderr='credential=secret-token')
        self.runtime.runner = failing_runner
        with self.assertRaises(HTTPException) as caught:
            self.runtime.start()
        self.assertNotIn('secret-token', str(caught.exception.detail))

    def test_overview_remains_responsive_during_slow_start_and_rejects_duplicate_actions(self):
        entered, release = threading.Event(), threading.Event()
        completed, errors = [], []
        def delayed_runner(arguments, **kwargs):
            if arguments == [str(self.runtime.script), 'start']:
                entered.set()
                if not release.wait(2):
                    raise RuntimeError('test timed out')
            return self.runner(arguments, **kwargs)
        self.runtime.runner = delayed_runner
        def launch():
            try:
                completed.append(self.runtime.start())
            except Exception as exc:
                errors.append(exc)
        thread = threading.Thread(target=launch)
        thread.start()
        try:
            self.assertTrue(entered.wait(1))
            before = time.monotonic()
            status = self.runtime.snapshot()
            self.assertLess(time.monotonic()-before, .25)
            self.assertEqual(status['phase'], 'starting')
            self.assertTrue(status['busy'])
            self.assertIsNone(status['error'])
            self.assertTrue(all(s['tone'] != 'ready' for s in status['stages'].values()))
            for action in (self.runtime.start, self.runtime.stop):
                before = time.monotonic()
                with self.assertRaises(HTTPException) as caught:
                    action()
                self.assertEqual(caught.exception.status_code, 409)
                self.assertLess(time.monotonic()-before, .25)
        finally:
            release.set()
            thread.join(2)
        self.assertFalse(thread.is_alive())
        self.assertEqual(errors, [])
        self.assertEqual(completed[0]['phase'], 'running')
        self.assertIsNone(self.runtime.operation)

    def test_overview_remains_responsive_during_slow_stop(self):
        self.write_session()
        entered, release = threading.Event(), threading.Event()
        def delayed_runner(arguments, **kwargs):
            if arguments == [str(self.runtime.script), 'stop']:
                entered.set()
                if not release.wait(2):
                    raise RuntimeError('test timed out')
            return self.runner(arguments, **kwargs)
        self.runtime.runner = delayed_runner
        thread = threading.Thread(target=self.runtime.stop)
        thread.start()
        try:
            self.assertTrue(entered.wait(1))
            before = time.monotonic()
            status = self.runtime.snapshot()
            self.assertLess(time.monotonic()-before, .25)
            self.assertEqual(status['phase'], 'stopping')
            self.assertTrue(status['busy'])
        finally:
            release.set()
            thread.join(2)
        self.assertFalse(thread.is_alive())
        self.assertEqual(self.runtime.snapshot()['phase'], 'stopped')

    def test_preflight_missing_module_has_bounded_safe_error_and_releases_operation(self):
        def failing_runner(arguments, **kwargs):
            if arguments[0] == 'systemctl':
                return self.runner(arguments, **kwargs)
            raise subprocess.CalledProcessError(1, arguments, stderr='token=secret-token\n'
                'RuntimeError: 定位程序安装不完整：缺少模块 d1max_localization.estimation.pose_status；'
                '请重新构建 d1max_localization\n')
        self.runtime.runner = failing_runner
        with self.assertRaises(HTTPException) as caught:
            self.runtime.start()
        self.assertEqual(caught.exception.status_code, 503)
        self.assertIn('d1max_localization.estimation.pose_status', caught.exception.detail)
        self.assertNotIn('secret-token', caught.exception.detail)
        self.assertIsNone(self.runtime.operation)
        self.assertFalse(self.runtime.snapshot()['busy'])

    def test_router_shared_job_lock_returns_busy_without_waiting(self):
        lock = threading.RLock()
        app = FastAPI()
        app.include_router(create_live_planning_router(self.runtime, lock, threading.RLock(), lambda: False))
        with TestClient(app) as client, lock:
            before = time.monotonic()
            result = client.post('/api/live-planning/start', json={})
            self.assertEqual(result.status_code, 409)
            self.assertLess(time.monotonic()-before, .25)
            self.assertEqual(client.get('/api/live-planning/overview').status_code, 200)
        self.assertEqual(self.calls, [])

    def test_router_origin_json_and_busy_gate(self):
        with TestClient(self.app(busy=lambda: True)) as client:
            self.assertEqual(client.get('/api/live-planning/overview').status_code, 200)
            self.assertEqual(client.post('/api/live-planning/start', data='{}', headers={
                'Content-Type': 'text/plain'}).status_code, 415)
            self.assertEqual(client.post('/api/live-planning/start', json={}, headers={
                'Origin': 'https://evil.invalid'}).status_code, 403)
            self.assertEqual(client.post('/api/live-planning/start', json={}).status_code, 409)
        self.assertEqual(self.calls, [])
        with TestClient(self.app()) as client:
            self.assertEqual(client.post('/api/live-planning/start', json={'goal': [1, 2]}).status_code, 422)
            self.assertEqual(client.post('/api/live-planning/start', json={}).status_code, 202)
            self.assertEqual(client.post('/api/live-planning/stop', json={}).status_code, 200)


if __name__ == '__main__':
    unittest.main()
