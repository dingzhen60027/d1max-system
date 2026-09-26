"""Mocked lifecycle/contract tests; never start ROS, SDK, services or processes."""
import os
from unittest.mock import patch

import pytest

from d1max_pct_scan import live_session
from d1max_pct_scan.live_view_reload import (
    REQUEST_FILE, RESULT_FILE, UI_NAMES, RvizClosedDuringReload,
    owned_directory, preview_reload_session, read_owned_json, write_owned_json,
    validate_request, verify_runtime_owner, verified_result, replace_ui_children)

SID, INVOCATION, NONCE = 'a'*32, 'b'*32, 'c'*32


def session():
    return dict(id=SID, mode='LIVE_VISUALIZATION_NO_MOTION', motion_control_enabled=False,
                ui_reload_supported=1, preview_freeze_owner='supervisor_child', map_pcd='/test.pcd')


def request():
    return dict(schema=1, action='reload-view', session_id=SID, owner_nonce=NONCE,
                request_id='d'*32, issued_at=100., expires_at=105.)


def owner_fixture():
    runtime = dict(session_id=SID, phase='running', motion_capable=False,
        received_at_unix=100., ui_reload_supported=1,
        ui_reload=dict(owner_pid=321, invocation_id=INVOCATION, owner_nonce=NONCE))
    owner = dict(MainPID='321', InvocationID=INVOCATION,
                 Description=live_session.OWNER+SID, ActiveState='active')
    return runtime, owner


@pytest.mark.parametrize('change', [
    {'mode': 'LIVE_NAVIGATION'}, {'motion_control_enabled': True},
    {'motion_control_enabled': 0}, {'id': '../other'}, {'ui_reload_supported': 0},
    {'ui_reload_supported': True}, {'preview_freeze_owner': 'view'},
])
def test_only_explicit_versioned_preview_can_reload_or_own_heartbeat(change):
    with pytest.raises(ValueError):
        preview_reload_session(dict(session(), **change))


@pytest.mark.parametrize('change', [
    {'action': 'start'}, {'session_id': 'f'*32}, {'owner_nonce': 'f'*32},
    {'schema': True}, {'command': ['sh', '-c', 'unsafe']}, {'session': '/elsewhere'},
    {'expires_at': 106.}, {'expires_at': 99.}, {'issued_at': 101.},
    {'issued_at': True}, {'expires_at': float('nan')}, {'request_id': '../path'},
])
def test_request_has_fixed_action_identity_and_bounded_deadline(change):
    with pytest.raises(ValueError):
        validate_request(dict(request(), **change), session_id=SID, owner_nonce=NONCE, now=100.1)


def test_expired_request_and_consumed_owner_nonce_cannot_replay():
    assert validate_request(request(), session_id=SID, owner_nonce=NONCE, now=100.1) == 'd'*32
    for arguments in (dict(now=105.01, owner_nonce=NONCE), dict(now=100.2, owner_nonce='f'*32)):
        with pytest.raises(ValueError):
            validate_request(request(), session_id=SID, **arguments)


@pytest.mark.parametrize('scope,change', [
    ('runtime', {'ui_reload_supported': 0}), ('runtime', {'received_at_unix': 90.}),
    ('runtime', {'session_id': 'f'*32}), ('runtime', {'motion_capable': True}),
    ('runtime', {'phase': 'stopping'}), ('unit', {'MainPID': '999'}),
    ('unit', {'InvocationID': 'f'*32}), ('unit', {'Description': 'unowned'}),
    ('unit', {'ActiveState': 'inactive'}),
])
def test_runtime_capability_requires_fresh_exact_systemd_owner(scope, change):
    runtime, owner = owner_fixture()
    (runtime if scope == 'runtime' else owner).update(change)
    with pytest.raises(ValueError):
        verify_runtime_owner(session(), runtime, owner, owner_prefix=live_session.OWNER, now=100.1)


@pytest.mark.parametrize('change', [
    {'schema': True}, {'owner_nonce': 'f'*32}, {'owner_pid': 322},
    {'invocation_id': 'f'*32}, {'completed_at': float('nan')},
    {'completed_at': 99.}, {'completed_at': 101.}, {'status': 'pending'},
    {'children': [{'name': 'scan', 'pid': 400}, {'name': 'view', 'pid': 401}]},
])
def test_matching_ack_is_bound_to_nonce_owner_source_time_and_actual_ui_pids(change):
    result = dict(schema=1, session_id=SID, request_id='d'*32, owner_nonce=NONCE,
        owner_pid=321, invocation_id=INVOCATION, status='reloaded', completed_at=100.1,
        processes_running=True, children=[{'name': 'view', 'pid': 400}, {'name': 'rviz', 'pid': 401}])
    assert verified_result(result, request(), owner_pid=321, invocation_id=INVOCATION, now=100.2)
    with pytest.raises(ValueError):
        verified_result(dict(result, **change), request(), owner_pid=321,
                        invocation_id=INVOCATION, now=100.2)


def test_mailbox_rejects_symlink_oversize_and_external_directory(tmp_path):
    directory = tmp_path/'session'; directory.mkdir(mode=0o700)
    assert owned_directory(directory, tmp_path) == directory
    write_owned_json(directory/'input.json', {'valid': True})
    assert read_owned_json(directory/'input.json') == {'valid': True}
    (directory/'link.json').symlink_to(directory/'input.json')
    with pytest.raises(OSError):
        read_owned_json(directory/'link.json')
    with pytest.raises(ValueError):
        read_owned_json(directory/'input.json', maximum_bytes=2)
    with pytest.raises(ValueError):
        owned_directory(tmp_path, tmp_path)
    (tmp_path/'link').symlink_to(directory, target_is_directory=True)
    with pytest.raises(ValueError):
        owned_directory(tmp_path/'link', tmp_path)


class Child:
    def __init__(self, pid, code=None):
        self.pid, self.returncode = pid, code

    def poll(self):
        return self.returncode


def lifecycle():
    names = ('localization', 'preview_freeze', 'view', 'map_layers', 'global', 'bridge', 'scan', 'rviz')
    children = [(name, Child(100+i)) for i, name in enumerate(names)]
    stops, starts = [], []
    def stop(selected, **_):
        stops.append([name for name, _ in selected])
        for _, child in selected:
            child.returncode = 0
        return dict(all_direct_children_exited=True, all_owned_groups_exited=True)
    def spawn(name):
        assert all(child.poll() is not None for current, child in children if current == name)
        starts.append(name)
        return Child(200+len(starts))
    return children, stops, starts, dict(stop_children=stop, spawn=spawn,
        configure=lambda: None, stopping=lambda: False, settle_s=0.)


def test_only_ui_replaced_with_exact_backend_identity_and_no_duplicates():
    children, stops, starts, arguments = lifecycle()
    original = dict(children)
    result = replace_ui_children(children, **arguments)
    assert stops == [['view', 'rviz']] and starts == ['view', 'rviz']
    for name, child in children:
        if name not in UI_NAMES:
            assert child is original[name] and child.poll() is None
    assert set(dict(children)) == set(original)
    assert result['render_verified'] is False and result['processes_running'] is True


def test_old_ui_group_must_be_gone_before_replacement():
    children, _, starts, arguments = lifecycle()
    original = list(children)
    arguments['stop_children'] = lambda *a, **k: dict(
        all_direct_children_exited=True, all_owned_groups_exited=False)
    with pytest.raises(RuntimeError, match='remain alive'):
        replace_ui_children(children, **arguments)
    assert children == original and starts == []


def test_partial_ui_failure_cleans_ui_only_and_allows_retry():
    children, stops, _, arguments = lifecycle()
    backend = [(n, c) for n, c in children if n not in UI_NAMES]
    good_spawn = arguments['spawn']
    def fail_second(name):
        if name == 'rviz':
            raise OSError('fixture failure')
        return good_spawn(name)
    arguments['spawn'] = fail_second
    with pytest.raises(OSError):
        replace_ui_children(children, **arguments)
    assert children == backend and stops == [['view', 'rviz'], ['view']]
    arguments['spawn'] = good_spawn
    replace_ui_children(children, **arguments)
    assert [(n, c) for n, c in children if n not in UI_NAMES] == backend


@pytest.mark.parametrize('scenario', ['no_heartbeat', 'backend_dead', 'stopping', 'duplicate'])
def test_bad_preconditions_do_not_launch_anything(scenario):
    children, stops, starts, arguments = lifecycle()
    if scenario == 'no_heartbeat':
        children[:] = [(n, c) for n, c in children if n != 'preview_freeze']
    elif scenario == 'backend_dead':
        dict(children)['localization'].returncode = 1
    elif scenario == 'stopping':
        arguments['stopping'] = lambda: True
    else:
        children.append(('view', Child(999)))
    with pytest.raises(RuntimeError):
        replace_ui_children(children, **arguments)
    assert stops == starts == []


def test_user_close_of_replacement_rviz_is_terminal():
    children, stops, _, arguments = lifecycle()
    arguments['spawn'] = lambda name: Child(222 if name == 'view' else 223, 0 if name == 'rviz' else None)
    with pytest.raises(RvizClosedDuringReload):
        replace_ui_children(children, **arguments)
    assert stops == [['view', 'rviz'], ['view', 'rviz']]


def test_cli_refuses_old_supervisor_without_writing_request(tmp_path, monkeypatch):
    directory = tmp_path/'session'; directory.mkdir(mode=0o700)
    write_owned_json(directory/'session.json', session())
    write_owned_json(directory/'runtime_status.json', dict(session_id=SID, phase='running'))
    monkeypatch.setattr(live_session, 'ROOT', tmp_path)
    monkeypatch.setattr(live_session, 'unit', lambda: owner_fixture()[1])
    with patch.object(live_session.subprocess, 'Popen') as popen:
        with pytest.raises(ValueError, match='Running supervisor does not support'):
            live_session.reload_view(directory)
        popen.assert_not_called()
    assert not (directory/REQUEST_FILE).exists()


@pytest.mark.parametrize('configure_failure', [False, True])
def test_mock_supervisor_preserves_backends_and_normal_rviz_close(tmp_path, monkeypatch, configure_failure):
    directory = tmp_path/'session'; directory.mkdir(mode=0o700)
    write_owned_json(directory/'session.json', session())
    monkeypatch.setattr(live_session, 'ROOT', tmp_path)
    monkeypatch.setenv('INVOCATION_ID', INVOCATION)
    monkeypatch.setenv('RMW_IMPLEMENTATION', 'rmw_zenoh_cpp')
    monkeypatch.setenv('ZENOH_SESSION_CONFIG_URI', str(live_session.APP/'foxglove_d1max/config/zenoh-live.json5'))
    monkeypatch.setattr(live_session, 'unit', lambda name=live_session.UNIT:
        dict(MainPID=str(os.getpid()), InvocationID=INVOCATION, Description=live_session.OWNER+SID,
             ActiveState='active') if name == live_session.UNIT else {'ActiveState': 'inactive'})
    handlers, launches, stops, snapshots = {}, [], [], []
    monkeypatch.setattr(live_session.signal, 'signal', lambda signum, fn: handlers.update({signum: fn}))
    def fake_popen(args, **options):
        assert options['start_new_session'] is True and not options.get('shell')
        child = Child(1000+len(launches))
        launches.append((list(args), child))
        return child
    def fake_stop(selected, **_):
        stops.append([(name, child.pid) for name, child in selected])
        for _, child in selected:
            child.returncode = 0
        return dict(all_direct_children_exited=True, all_owned_groups_exited=True)
    def configure(sid, *, motion=False, layout='global'):
        assert motion is False and layout in ('global', 'local')
        if configure_failure:
            raise OSError('fixture invalid config')
        return {'test_only_session': sid, 'layout': layout}
    monkeypatch.setattr(live_session.subprocess, 'Popen', fake_popen)
    monkeypatch.setattr(live_session, 'stop_owned_children', fake_stop)
    monkeypatch.setattr(live_session, 'view_config', configure)
    monkeypatch.setattr(live_session, 'replace_ui_children', lambda children, **kwargs:
        replace_ui_children(children, settle_s=0., **kwargs))
    initial_request = None
    phase = 0
    def advance(_seconds):
        nonlocal phase, initial_request
        runtime = read_owned_json(directory/'runtime_status.json')
        if phase == 0:
            snapshots.append(runtime)
            issued = live_session.time.time()
            initial_request = dict(request(), owner_nonce=runtime['ui_reload']['owner_nonce'],
                                   issued_at=issued, expires_at=issued+5.)
            write_owned_json(directory/REQUEST_FILE, initial_request)
            phase = 1
        elif phase == 1:
            result = read_owned_json(directory/RESULT_FILE)
            snapshots.append(runtime)
            if configure_failure:
                assert result['status'] == 'failed' and not stops
                launches[7][1].returncode = 0  # User closes the untouched old RViz.
            else:
                assert result['status'] == 'reloaded' and not result['goal_resubmitted']
                assert not result['initial_pose_resubmitted']
                write_owned_json(directory/REQUEST_FILE, dict(initial_request, request_id='e'*32))
            phase = 2
        else:
            assert not configure_failure, 'Closing untouched RViz must stop, not loop headless'
            result = read_owned_json(directory/RESULT_FILE)
            assert result['status'] == 'failed' and 'nonce' in result['error']
            handlers[live_session.signal.SIGTERM]()
    monkeypatch.setattr(live_session.time, 'sleep', advance)
    live_session.run(directory)
    assert phase == 2 and len(launches) == (8 if configure_failure else 10)
    before, after = [{c['name']: c['pid'] for c in value['children']} for value in snapshots]
    assert {n: p for n, p in before.items() if n not in UI_NAMES} == {
        n: p for n, p in after.items() if n not in UI_NAMES}
    if configure_failure:
        assert read_owned_json(directory/'runtime_status.json')['reason'] == 'rviz_closed'
    else:
        assert before['view'] != after['view'] and before['rviz'] != after['rviz']
        assert [name for name, _ in stops[0]] == ['view', 'rviz']
    assert sum('d1max_pct_scan.preview_freeze' in args for args, _ in launches) == 1
    assert not any(any('sdk' in part.lower() for part in args) for args, _ in launches)
    assert not (directory/'initial_pose.json').exists()
