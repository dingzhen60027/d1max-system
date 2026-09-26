"""No services/processes are started: lifecycle calls are replaced by fakes."""
import ast
import json
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from d1max_navigation import navigation_session as session


SHARE = Path(__file__).resolve().parents[1]
INACTIVE = {'LoadState': 'not-found', 'ActiveState': 'inactive', 'MainPID': '0'}


@pytest.fixture
def fake_session(monkeypatch, tmp_path):
    calls = []
    preview_stops = []
    root = tmp_path / 'sessions'
    root.mkdir()
    folder = tmp_path / 'grid-test'
    folder.mkdir()
    grid = {'resolution': 0.05, 'origin': [-10.0, -5.0, 0.0]}
    manifest = {'name': 'Test floor', 'width': 500, 'height': 300}
    monkeypatch.setattr(session, 'info', lambda unit=session.UNIT: INACTIVE.copy())
    monkeypatch.setattr(session, 'selected_map', lambda app: ('grid-test', folder, grid, manifest))
    monkeypatch.setattr(session, 'port_open', lambda port: False)
    monkeypatch.setattr(session, 'stop_owned', lambda *args: preview_stops.append(args))
    monkeypatch.setattr(session, 'api', lambda *args, **kwargs: pytest.fail('Simulation must not call live Web APIs'))
    monkeypatch.setattr(session, 'live_context', lambda *args: pytest.fail('Simulation must not connect or start localization'))
    monkeypatch.setattr(session.subprocess, 'run', lambda command, **kwargs: calls.append(command))
    monkeypatch.setenv('DISPLAY', ':99')
    monkeypatch.setenv('RMW_IMPLEMENTATION', 'deliberately-not-zenoh')
    args = SimpleNamespace(mode='sim', enable_motion=False, headless=True)
    return SimpleNamespace(args=args, app=tmp_path / 'app', nav=tmp_path / 'nav',
                           root=root, calls=calls, preview_stops=preview_stops)


def test_full_sim_start_isolated_zenoh_no_web_or_sdk_calls(fake_session):
    f = fake_session
    session.start(f.args, f.app, f.nav, SHARE, f.root)
    assert len(f.calls) == 1
    command = f.calls[0]
    assert command[:3] == ['systemd-run', '--user', '--collect']
    assert '--setenv=RMW_IMPLEMENTATION=rmw_zenoh_cpp' in command
    assert '--setenv=ROS_DOMAIN_ID=24' in command
    assert any('ZENOH_SESSION_CONFIG_URI=' in arg and 'zenoh-offline-session.json5' in arg for arg in command)
    assert not any('zenoh-live' in arg or 'd1max-sdk' in arg or 'BindsTo=' in arg for arg in command)
    assert '--property=KillMode=mixed' in command
    assert '--property=KillSignal=SIGINT' in command
    assert command[-5:-1] == ['ros2', 'launch', 'd1max_navigation', 'navigation.launch.py']
    snapshot = Path(command[-1].removeprefix('session:='))
    cfg = json.loads(snapshot.read_text())
    assert cfg['mode'] == 'sim' and cfg['enable_motion'] is False
    assert cfg['version_id'] == 'grid-test'
    assert cfg['localization_session_id'] == ''
    assert Path(cfg['map_yaml']).parent == Path(cfg['localization_pcd']).parent
    assert cfg['bt_to_pose'].endswith('/behavior_trees/navigate_to_pose.xml')
    assert cfg['bt_through_poses'].endswith('/behavior_trees/navigate_through_poses.xml')
    assert Path(cfg['nav2_config']).parent == snapshot.parent
    assert cfg['command_gate_limits'] == {'max_forward': 0.3, 'max_lateral': 0.0, 'max_yaw': 0.5}
    assert cfg['motion_limits']['max_planar'] == 1.5
    assert len(cfg['navigation_session_id']) == 32
    assert any(arg.startswith('--property=Description=') and arg.endswith(':'+cfg['navigation_session_id']) for arg in command)
    assert Path(cfg['collision_monitor_config']).parent == snapshot.parent
    nav2 = yaml.safe_load(Path(cfg['nav2_config']).read_text())
    assert nav2['controller_server']['ros__parameters']['FollowPath']['max_vel_x'] == 0.3
    assert f.preview_stops == [(session.PREVIEW_UNIT, session.PREVIEW_OWNER)]


@pytest.mark.parametrize('state', ['active', 'activating', 'deactivating'])
def test_duplicate_full_start_never_touches_web_or_spawns(monkeypatch, tmp_path, state):
    monkeypatch.setattr(session, 'info', lambda: {'ActiveState': state, 'Description': session.OWNER + 'sim'})
    monkeypatch.setattr(session, 'api', lambda *args: pytest.fail('duplicate must not call Web'))
    monkeypatch.setattr(session.subprocess, 'run', lambda *args, **kwargs: pytest.fail('duplicate must not launch'))
    with pytest.raises(RuntimeError, match='already running'):
        session.start(None, tmp_path, tmp_path, SHARE, tmp_path)


def test_full_stop_refuses_unknown_owner(monkeypatch):
    monkeypatch.setattr(session, 'info', lambda unit: {'ActiveState': 'active', 'Description': 'unrelated service'})
    monkeypatch.setattr(session.subprocess, 'run', lambda *args, **kwargs: pytest.fail('must not stop unrelated processes'))
    with pytest.raises(RuntimeError, match='unowned'):
        session.stop_owned()


def test_full_stop_checks_owned_cgroup_empty(monkeypatch):
    states = iter([{'ActiveState': 'active', 'Description': session.OWNER + 'sim'}, INACTIVE])
    commands = []
    monkeypatch.setattr(session, 'info', lambda unit: next(states))
    monkeypatch.setattr(session.subprocess, 'run', lambda command, **kwargs: commands.append(command))
    session.stop_owned()
    assert commands == [['systemctl', '--user', 'stop', session.UNIT]]


def test_full_stop_cannot_claim_success_while_still_active(monkeypatch):
    monkeypatch.setattr(session, 'info', lambda unit: {'ActiveState': 'active', 'Description': session.OWNER + 'sim'})
    monkeypatch.setattr(session.subprocess, 'run', lambda command, **kwargs: None)
    with pytest.raises(RuntimeError, match='not empty'):
        session.stop_owned()


def test_sim_refuses_hardware_capability(fake_session):
    f = fake_session
    f.args.enable_motion = True
    with pytest.raises(ValueError, match='not valid in simulation'):
        session.start(f.args, f.app, f.nav, SHARE, f.root)
    assert not f.calls


def test_web_owned_start_requires_managed_web_service(fake_session):
    f = fake_session
    f.args.web_owned = True
    with pytest.raises(RuntimeError, match='active managed Web'):
        session.start(f.args, f.app, f.nav, SHARE, f.root)
    assert not f.calls
    assert not f.preview_stops


def test_web_owned_session_is_bound_to_web_lifetime(fake_session, monkeypatch):
    f = fake_session
    f.args.web_owned = True
    monkeypatch.setattr(session, 'info', lambda unit=session.UNIT:
        {'ActiveState': 'active'} if unit == 'd1max-web-managed.service' else INACTIVE.copy())
    session.start(f.args, f.app, f.nav, SHARE, f.root)
    assert '--property=BindsTo=d1max-web-managed.service' in f.calls[0]
    assert '--property=PartOf=d1max-web-managed.service' in f.calls[0]
    assert '--property=After=d1max-web-managed.service' in f.calls[0]
    cfg = json.loads(Path(f.calls[0][-1].removeprefix('session:=')).read_text())
    assert cfg['web_owned'] is True


def test_sim_refuses_unknown_existing_router(fake_session, monkeypatch):
    f = fake_session
    monkeypatch.setattr(session, 'port_open', lambda port: port == 7460)
    with pytest.raises(RuntimeError, match='unknown process'):
        session.start(f.args, f.app, f.nav, SHARE, f.root)
    assert not f.calls
    assert not f.preview_stops


def test_partial_frame_rename_rejected():
    settings = {'map_frame': 'd1max_loc_map', 'odom_frame': 'd1max_loc_odom',
                'base_frame': 'd1max_loc_base_link'}
    session.validate_frame_contract(settings)
    settings['map_frame'] = 'map'
    with pytest.raises(ValueError, match='Frame contract'):
        session.validate_frame_contract(settings)


def test_display_required_unless_explicit_headless(fake_session, monkeypatch):
    f = fake_session
    f.args.headless = False
    monkeypatch.delenv('DISPLAY', raising=False)
    with pytest.raises(RuntimeError, match='DISPLAY'):
        session.start(f.args, f.app, f.nav, SHARE, f.root)
    assert not f.calls


@pytest.mark.parametrize('mode', ['sim', 'live'])
def test_full_rviz_labels_simulation_and_live_exclusively(tmp_path, mode):
    path = session.configure_rviz(SHARE, tmp_path, {'mode': mode, 'map_frame': 'd1max_loc_map'},
        {'resolution': 0.05, 'origin': [-10., -5., 0.]}, {'width': 500, 'height': 300})
    rviz = yaml.safe_load(Path(path).read_text())
    displays = {d.get('Topic', {}).get('Value'): d for d in rviz['Visualization Manager']['Displays']}
    assert displays['/d1max/navigation/simulation_status']['Enabled'] is (mode == 'sim')
    assert displays['/d1max/navigation/test_status']['Enabled'] is (mode == 'live')
    assert displays['/d1max/localization/scan_initial_preview']['Enabled'] is (mode == 'live')
    assert displays['/d1max/navigation/global_costmap/costmap']['Enabled'] is True
    assert displays['/d1max/navigation/local_costmap/costmap']['Enabled'] is True
    assert any(t['Class'] == 'nav2_rviz_plugins/GoalTool' for t in rviz['Visualization Manager']['Tools'])


def test_full_info_fail_closed_on_bus_failure(monkeypatch):
    monkeypatch.setattr(session.subprocess, 'run', lambda *args, **kwargs: SimpleNamespace(
        returncode=1, stdout='', stderr='Failed to connect to bus'))
    with pytest.raises(RuntimeError, match='bus'):
        session.info()


def test_full_info_first_use_absent_is_not_bus_failure(monkeypatch):
    monkeypatch.setattr(session.subprocess, 'run', lambda *args, **kwargs: SimpleNamespace(
        returncode=4, stdout='LoadState=not-found\nActiveState=inactive\nMainPID=0\n', stderr=''))
    assert session.info()['LoadState'] == 'not-found'


def test_lock_retries_brief_web_status_contention(monkeypatch):
    attempts = []
    clock = [0.0]
    def flock(*args):
        attempts.append(args)
        if len(attempts) < 3:
            raise BlockingIOError('held by status request')
    monkeypatch.setattr(session.fcntl, 'flock', flock)
    monkeypatch.setattr(session.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(session.time, 'sleep', lambda seconds: clock.__setitem__(0, clock[0]+seconds))
    session.acquire_session_lock(object())
    assert len(attempts) == 3
    assert clock[0] == pytest.approx(.1)


def test_lock_contention_has_bounded_timeout_and_explicit_busy_reason(monkeypatch):
    clock = [0.0]
    def flock(*args):
        raise BlockingIOError('held')
    monkeypatch.setattr(session.fcntl, 'flock', flock)
    monkeypatch.setattr(session.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(session.time, 'sleep', lambda seconds: clock.__setitem__(0, clock[0]+seconds))
    with pytest.raises(RuntimeError, match='no operation was executed'):
        session.acquire_session_lock(object())
    assert clock[0] == pytest.approx(6.0)


def test_live_requires_fresh_real_data_no_replay(monkeypatch):
    monkeypatch.setattr(session, 'port_open', lambda port: port == 7448)
    monkeypatch.setattr(session, 'api', lambda *args: {
        'phase': 'running', 'id': 'old-session', 'version_id': 'grid-test',
        'connection': {'active': True, 'health': {'sdk_fresh': True, 'lidar_fresh': True, 'replay': True}},
    })
    with pytest.raises(RuntimeError, match='no replay'):
        session.live_context({'web_url': 'http://127.0.0.1:8766'}, 'grid-test', False)


def test_live_map_mismatch_cannot_reuse_other_localization(monkeypatch):
    monkeypatch.setattr(session, 'port_open', lambda port: port == 7448)
    monkeypatch.setattr(session, 'api', lambda *args: {
        'phase': 'running', 'id': 'other-session', 'version_id': 'other-map',
        'connection': {'active': True, 'health': {'sdk_fresh': True, 'lidar_fresh': True, 'replay': False}},
    })
    with pytest.raises(RuntimeError, match='conflict'):
        session.live_context({'web_url': 'http://127.0.0.1:8766'}, 'grid-test', False)


def test_full_launcher_namespace_tf_and_simulation_branch_contract():
    source = (SHARE / 'launch/navigation.launch.py').read_text()
    tree = ast.parse(source)
    assert "NS='d1max/navigation'" in source
    assert 'namespace=NS' in source and 'root_key=NS' in source
    for forbidden in ('static_transform_publisher', 'nav2_amcl', 'robot_state_publisher',
                      'TransformBroadcaster', "('/tf', 'tf')", '("/tf", "tf")'):
        assert forbidden not in source
    sim_branch = next(n for n in ast.walk(tree) if isinstance(n, ast.If)
        and isinstance(n.test, ast.Name) and n.test.id == 'sim'
        and 'navigation_simulator' in ast.unparse(n))
    assert 'navigation_simulator' in '\n'.join(ast.unparse(n) for n in sim_branch.body)
    live_source = '\n'.join(ast.unparse(n) for n in sim_branch.orelse)
    assert 'navigation_simulator' not in live_source
    assert 'navigation_cloud_to_scan' in live_source
    assert 'rviz_initial_pose_bridge' in live_source
    # Recovery and controller commands share a private channel, not /cmd_vel.
    assert source.count("('cmd_vel','cmd_vel_raw')") == 3
    assert 'cmd_vel_smoothed' in source
    assert 'OnProcessExit' in source and 'Shutdown' in source
