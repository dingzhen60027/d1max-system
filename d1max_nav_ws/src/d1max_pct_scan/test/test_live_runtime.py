import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from unittest.mock import patch
import pytest

from d1max_pct_scan.live_runtime import atomic_json, child_failure_reason, file_sha256, stop_owned_children


def test_launch_zero_exit_preserves_allowlisted_missing_module_cause(tmp_path):
    (tmp_path/'localization.log').write_text(
        "secret credential elsewhere\n[lio_localizer-4] ModuleNotFoundError: No module named "
        "'d1max_localization.estimation.pose_status'\n[INFO] shutdown exit code 0\n")
    reason = child_failure_reason(tmp_path, 'localization', 0)
    assert 'd1max_localization.estimation.pose_status' in reason
    assert 'secret' not in reason and 'credential' not in reason


def test_failure_log_is_bounded_and_does_not_expose_arbitrary_exceptions(tmp_path):
    (tmp_path/'global.log').write_text('private_token=SECRET\n'*10000)
    reason = child_failure_reason(tmp_path, 'global', -2)
    assert reason == 'global exited with code -2; stopping owned session'
    assert child_failure_reason(tmp_path, 'missing', 1).startswith('missing exited')
    assert 'SECRET' not in child_failure_reason(tmp_path, '../global', 1)


def test_import_preflight_reports_missing_module_without_starting_nodes():
    from d1max_pct_scan.live_session import check_localization_imports
    import importlib
    with patch.object(importlib, 'import_module', side_effect=ModuleNotFoundError(
            'missing', name='d1max_localization.estimation.pose_status')):
        with pytest.raises(RuntimeError, match='定位程序安装不完整.*pose_status'):
            check_localization_imports()
    with patch.object(importlib, 'import_module') as import_module:
        check_localization_imports()
        assert [c.args[0] for c in import_module.call_args_list] == [
            'd1max_localization.lio_localizer', 'd1max_localization.lio_predictor',
            'd1max_localization.navigation_output']


def test_stream_hash_matches_without_whole_file_read(tmp_path):
    path = tmp_path / 'map.pcd'
    data = bytes(range(256)) * 20000
    path.write_bytes(data)
    with patch.object(Path, 'read_bytes', side_effect=AssertionError('whole-file copy')):
        assert file_sha256(path, chunk_bytes=65536) == hashlib.sha256(data).hexdigest()


def test_atomic_snapshot_replaces_without_partial_read(tmp_path):
    path = tmp_path / 'status.json'
    atomic_json(path, {'phase': 'starting'})
    atomic_json(path, {'phase': 'stopped', 'value': '一楼'})
    assert json.loads(path.read_text()) == {'phase': 'stopped', 'value': '一楼'}
    assert not (tmp_path / 'status.json.tmp').exists()


def test_cleanup_reaps_owned_process_and_escalates(tmp_path):
    ready = tmp_path / 'ready'
    child = subprocess.Popen([sys.executable, '-c',
        "import signal,time,pathlib,sys; signal.signal(signal.SIGINT,signal.SIG_IGN); "
        "signal.signal(signal.SIGTERM,signal.SIG_IGN); pathlib.Path(sys.argv[1]).touch(); time.sleep(30)",
        str(ready)], start_new_session=True)
    try:
        deadline = time.monotonic()+3
        while not ready.exists() and time.monotonic() < deadline:
            time.sleep(.01)
        assert ready.exists()
        report = stop_owned_children([('fixture', child)], interrupt_s=.1, terminate_s=.1, kill_s=.5)
        assert report['all_direct_children_exited']
        assert report['all_owned_groups_exited']
        assert report['signals'] == ['SIGINT', 'SIGTERM', 'SIGKILL']
        assert report['elapsed_s'] < 1.
    finally:
        if child.poll() is None:
            os.killpg(child.pid, 9)
        child.wait(timeout=2)


def test_partial_launch_failure_cleans_its_children_and_records_cause(tmp_path, monkeypatch):
    from d1max_pct_scan import live_session
    session = {'id': 'test-owned', 'mode': 'LIVE_VISUALIZATION_NO_MOTION',
               'motion_control_enabled': False, 'map_pcd': '/test-only.pcd'}
    atomic_json(tmp_path / 'session.json', session)
    monkeypatch.setenv('INVOCATION_ID', 'fixture')
    monkeypatch.setenv('RMW_IMPLEMENTATION', 'rmw_zenoh_cpp')
    monkeypatch.setenv('ZENOH_SESSION_CONFIG_URI', str(live_session.APP / 'foxglove_d1max/config/zenoh-live.json5'))
    monkeypatch.setattr(live_session, 'unit', lambda name=live_session.UNIT:
        {'MainPID': str(os.getpid()), 'Description': live_session.OWNER + session['id'],
         'ActiveState': 'active'} if name == live_session.UNIT else {'ActiveState': 'inactive'})
    original_popen = subprocess.Popen
    spawned = []
    def fixture_popen(_args, **options):
        if len(spawned) == 2:
            raise OSError('deliberate third-child failure')
        child = original_popen([sys.executable, '-c', 'import time; time.sleep(30)'], **options)
        spawned.append(child)
        return child
    monkeypatch.setattr(live_session.subprocess, 'Popen', fixture_popen)
    # This fixture tests post-preflight process cleanup only. Bundle/overlay
    # rejection before any Popen is covered by test_navigation_contract.py.
    monkeypatch.setattr(live_session, 'verify_bundle', lambda *_: None)
    monkeypatch.setattr(live_session, 'validate_tree_wiring', lambda *_: None)
    monkeypatch.setattr(live_session, 'verify_deployment', lambda *_: None)
    with pytest.raises(OSError, match='deliberate'):
        live_session.run(tmp_path)
    report = json.loads((tmp_path / 'runtime_status.json').read_text())
    assert report['phase'] == 'failed'
    assert report['cleanup']['all_owned_groups_exited']
    assert report['cleanup']['all_direct_children_exited']
    assert all(child.poll() is not None for child in spawned)
