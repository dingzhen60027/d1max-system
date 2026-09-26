"""Pure session/config tests: every subprocess and socket is replaced by a stub.

These tests never start systemd, a router, ROS, or a robot SDK.
"""
import json
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from d1max_pct_scan import session


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def harness(tmp_path, monkeypatch):
    workspace = tmp_path / 'ws'
    root = workspace / 'log/pct_scan'
    root.mkdir(parents=True)
    config = yaml.safe_load((PACKAGE_ROOT / 'config/offline.yaml').read_text())
    for name in ('map_pcd', 'ground_pcd', 'planning_grid'):
        fake = tmp_path / name
        fake.touch()
        config[name] = str(fake)
    planner_config = workspace / 'src/d1max_pct_planner/config/pct_scan_single_floor.yaml'
    planner_config.parent.mkdir(parents=True)
    planner_config.write_text(yaml.safe_dump({'pct_route_server': {'ros__parameters': {
        'planning_frame': 'unreplaced_frame', 'body_frame': 'unreplaced_body',
        'body_height_m': -99.0, 'goal_topic': '/d1max/pct_scan/goal',
    }}}))
    source_config = tmp_path / 'offline.yaml'
    calls, bindings = [], []

    class FakeSocket:
        def __enter__(self):
            return self

        def __exit__(self, *_args):
            return False

        def bind(self, address):
            bindings.append(address)

    def fake_run(command, **kwargs):
        calls.append((list(command), kwargs))
        return SimpleNamespace(returncode=0, stdout='', stderr='')

    monkeypatch.setattr(session, 'WS', workspace)
    monkeypatch.setattr(session, 'ROOT', root)
    monkeypatch.setattr(session, 'service', lambda: {'ActiveState': 'inactive'})
    monkeypatch.setattr(session.socket, 'socket', lambda *_args, **_kwargs: FakeSocket())
    monkeypatch.setattr(session.subprocess, 'run', fake_run)
    monkeypatch.setattr(session.uuid, 'uuid4', lambda: SimpleNamespace(hex='123456789abcdef0123456789abcdef0'))
    # session.main mutates os.environ on cancel; isolate those writes too.
    monkeypatch.setattr(session.os, 'environ', {
        'PATH': '/test/bin', 'DISPLAY': ':42',
        'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp', 'ROS_DOMAIN_ID': '77',
        'ZENOH_CONFIG_OVERRIDE': 'connect/endpoints=["tcp/robot:7447"]',
        'ZENOH_SESSION_CONFIG': '/test/robot_config.json5',
        'D1MAX_NAVIGATION_CONTROL_ENABLED': 'true',
    })

    def start(headless=True):
        source_config.write_text(yaml.safe_dump(config))
        session.start(SimpleNamespace(config=source_config, headless=headless))
        return json.loads((root / 'last_session.json').read_text())

    return SimpleNamespace(start=start, cfg=config, calls=calls, bindings=bindings,
                           root=root, source_config=source_config, monkeypatch=monkeypatch)


def test_default_profile_is_offline_and_cannot_enable_sdk(harness):
    snapshot = harness.start()
    assert snapshot['mode'] == 'sim'
    assert snapshot['robot_connected'] is False
    assert snapshot['real_motion_enabled'] is False
    assert snapshot['headless'] is True
    assert len(harness.calls) == 1
    command, options = harness.calls[0]
    assert command[0] == 'systemd-run'
    assert command[-3] == str(session.WS / 'install/d1max_pct_scan/lib/d1max_pct_scan/pct_scan_run')
    assert command[-2] == '--session'
    assert not any('sdk' in item.lower() for item in command)
    assert not any('D1MAX_NAVIGATION_CONTROL_ENABLED' in item for item in command)
    assert '--setenv=RMW_IMPLEMENTATION=rmw_zenoh_cpp' in command
    assert '--setenv=ROS_DOMAIN_ID=24' in command
    assert not any('ZENOH_CONFIG_OVERRIDE=' in item or 'ZENOH_SESSION_CONFIG=' in item
                   for item in command)
    assert options['check'] is True and options['timeout'] == 15


def test_private_router_7464_has_no_discovery_or_upstream(harness):
    snapshot = harness.start()
    assert harness.bindings == [('127.0.0.1', 7464)]
    directory = Path(snapshot['directory'])
    client = json.loads((directory / 'client.json5').read_text())
    router = json.loads((directory / 'router.json5').read_text())
    assert client['mode'] == 'client'
    assert client['connect']['endpoints'] == ['tcp/127.0.0.1:7464']
    assert router['mode'] == 'router'
    assert router['listen']['endpoints'] == ['tcp/127.0.0.1:7464']
    assert router['connect']['endpoints'] == []
    for config in (client, router):
        assert config['scouting']['multicast']['enabled'] is False
        assert config['scouting']['gossip']['enabled'] is False
    assert client['connect']['exit_on_failure'] is True
    assert router['listen']['exit_on_failure'] is True


def test_systemd_owns_and_finally_kills_child_group(harness):
    snapshot = harness.start()
    command, _ = harness.calls[0]
    assert '--property=Description=' + session.OWNER + snapshot['session_id'] in command
    assert '--property=KillMode=mixed' in command
    assert '--property=KillSignal=SIGINT' in command
    assert '--property=SendSIGKILL=yes' in command
    assert '--property=TimeoutStopSec=15' in command
    assert '--collect' in command


@pytest.mark.parametrize('field,value', [
    ('max_speed', 0.0), ('max_speed', -0.1), ('max_speed', 0.301),
    ('max_speed', float('nan')), ('max_speed', float('inf')),
    ('max_yaw_rate', 0.0), ('max_yaw_rate', -0.1),
    ('max_yaw_rate', 0.501), ('max_yaw_rate', float('nan')),
])
def test_unsafe_speed_is_rejected_before_process_start(harness, field, value):
    harness.cfg[field] = value
    with pytest.raises(ValueError, match='limited to'):
        harness.start()
    assert harness.calls == []
    assert not (harness.root / 'last_session.json').exists()


def test_explicit_frames_height_and_limits_flow_to_each_adapter(harness):
    harness.cfg.update(map_frame='test_map', base_frame='test_body', odom_frame='test_odom',
                       body_height=0.61, max_speed=0.23, max_yaw_rate=0.42)
    snapshot = harness.start()
    pct, scan, tracker = [snapshot[key] for key in
                          ('pct_parameters', 'scan_parameters', 'tracker_parameters')]
    assert pct['planning_frame'] == scan['grid_map.frame_id'] == tracker['planning_frame'] == 'test_map'
    assert pct['body_frame'] == tracker['base_frame'] == 'test_body'
    assert pct['body_height_m'] == scan['grid_map.body_height'] == scan['fsm.reference_path_z_offset'] == 0.61
    assert scan['manager.max_vel'] == scan['optimization.max_vel'] == tracker['max_speed'] == 0.23
    assert tracker['max_yaw_rate'] == 0.42
    assert pct['session_id'] == scan['fsm.navigation_session_id'] == tracker['session_id'] == snapshot['session_id']
    assert scan['fsm.require_tagged_reference'] is True
    assert scan['fsm.navi_mode'] == 3
    assert scan['grid_map.cloud_is_world'] is True
    assert scan['grid_map.need_extrinsic'] is False
    assert tracker['trajectory_topic'] == '/d1max/pct_scan/validated_bspline'
    saved = yaml.safe_load(Path(snapshot['config']).read_text())
    assert saved['odom_frame'] == 'test_odom'


def test_rviz_goal_is_pose_topic_not_nav2_action(harness):
    snapshot = harness.start(headless=False)
    assert snapshot['headless'] is False
    rviz = yaml.safe_load(Path(snapshot['rviz_config']).read_text())['Visualization Manager']
    tools = [tool for tool in rviz['Tools'] if tool['Class'] == 'rviz_default_plugins/SetGoal']
    assert tools == [{'Class': 'rviz_default_plugins/SetGoal', 'Topic': '/d1max/pct_scan/goal'}]
    assert rviz['Global Options']['Fixed Frame'] == harness.cfg['map_frame']
    marker = next(display for display in rviz['Displays'] if display['Name'] == 'SCAN optimized local spline')
    assert snapshot['session_id'] in marker['Topic']['Value']


@pytest.mark.parametrize('active_state', ['active', 'activating', 'deactivating'])
def test_existing_session_is_not_replaced_or_stacked(harness, active_state):
    harness.monkeypatch.setattr(session, 'service', lambda: {'ActiveState': active_state})
    with pytest.raises(RuntimeError, match='already exists'):
        harness.start()
    assert harness.bindings == []
    assert harness.calls == []


def test_existing_unknown_router_is_never_reused(harness):
    class OccupiedSocket:
        def __enter__(self):
            return self
        def __exit__(self, *_args):
            return False
        def bind(self, _address):
            raise OSError('Address already in use')
    harness.monkeypatch.setattr(session.socket, 'socket', lambda: OccupiedSocket())
    with pytest.raises(OSError, match='already in use'):
        harness.start()
    assert harness.calls == []


def test_cancel_only_targets_exact_current_session(harness):
    snapshot = harness.start()
    harness.calls.clear()
    harness.monkeypatch.setattr(session, 'service', lambda: {
        'ActiveState': 'active', 'Description': session.OWNER + snapshot['session_id']})
    harness.monkeypatch.setattr('sys.argv', ['pct-scan', 'cancel'])
    session.main()
    assert len(harness.calls) == 1
    command, kwargs = harness.calls[0]
    assert command == ['ros2', 'topic', 'pub', '--once', '--wait-matching-subscriptions', '1',
                       '/d1max/pct_scan/cancel', 'std_msgs/msg/Empty', '{}']
    assert kwargs['timeout'] == 10
    assert session.os.environ['ZENOH_SESSION_CONFIG_URI'] == str(Path(snapshot['directory']) / 'client.json5')
    assert session.os.environ['RMW_IMPLEMENTATION'] == 'rmw_zenoh_cpp'
    assert session.os.environ['ROS_DOMAIN_ID'] == '24'
    assert 'ZENOH_CONFIG_OVERRIDE' not in session.os.environ
    assert 'ZENOH_SESSION_CONFIG' not in session.os.environ


@pytest.mark.parametrize('description,active_state', [
    (session.OWNER + 'stale_another_session', 'active'),
    ('FOREIGN_SESSION', 'active'),
    (session.OWNER + '123456789abc', 'inactive'),
])
def test_cancel_rejects_stale_snapshot_foreign_or_stopped_session(harness, description, active_state):
    harness.start()
    harness.calls.clear()
    harness.monkeypatch.setattr(session, 'service', lambda: {
        'ActiveState': active_state, 'Description': description})
    harness.monkeypatch.setattr('sys.argv', ['pct-scan', 'cancel'])
    with pytest.raises(RuntimeError):
        session.main()
    assert harness.calls == []


def test_stop_does_not_touch_unknown_service(harness):
    harness.monkeypatch.setattr(session, 'service', lambda: {
        'ActiveState': 'active', 'Description': 'not-owned-by-pct-scan'})
    with pytest.raises(RuntimeError, match='Unknown service owner'):
        session.stop()
    assert harness.calls == []


def test_stop_only_uses_exact_owned_unit(harness):
    harness.monkeypatch.setattr(session, 'service', lambda: {
        'ActiveState': 'active', 'Description': session.OWNER + 'test_session'})
    session.stop()
    assert harness.calls == [(['systemctl', '--user', 'stop', session.UNIT], {'check': True, 'timeout': 20})]


def test_failed_systemd_start_does_not_publish_success_snapshot(harness):
    def fail(command, **kwargs):
        raise session.subprocess.CalledProcessError(1, command)
    harness.monkeypatch.setattr(session.subprocess, 'run', fail)
    with pytest.raises(session.subprocess.CalledProcessError):
        harness.start()
    assert not (harness.root / 'last_session.json').exists()
