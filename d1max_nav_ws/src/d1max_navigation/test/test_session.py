from pathlib import Path
import json
import math
import pytest
import yaml

from d1max_navigation.session import map_view, prepare_rviz, selected_map, unit_state, start, stop

SHARE = Path(__file__).resolve().parents[1]


def test_first_launch_not_found_is_not_a_bus_failure(monkeypatch):
    from types import SimpleNamespace
    monkeypatch.setattr('d1max_navigation.session.subprocess.run', lambda *a, **kw: SimpleNamespace(
        returncode=4, stdout='LoadState=not-found\nActiveState=inactive\nMainPID=0\nControlGroup=\n', stderr=''))
    assert unit_state()['LoadState'] == 'not-found'


def test_bus_failure_stays_fail_closed(monkeypatch):
    from types import SimpleNamespace
    monkeypatch.setattr('d1max_navigation.session.subprocess.run', lambda *a, **kw: SimpleNamespace(
        returncode=1, stdout='', stderr='Failed to connect to bus'))
    with pytest.raises(RuntimeError, match='bus'):
        unit_state()


def test_stop_refuses_unknown_owner(monkeypatch):
    monkeypatch.setattr('d1max_navigation.session.unit_state', lambda: {
        'ActiveState': 'active', 'Description': 'Someone else', 'MainPID': '77'})
    monkeypatch.setattr('d1max_navigation.session.subprocess.run', lambda *a, **kw: pytest.fail('Must not stop unknown service'))
    with pytest.raises(RuntimeError, match='unowned'):
        stop()


def test_duplicate_start_is_not_a_second_process(monkeypatch, tmp_path):
    monkeypatch.setattr('d1max_navigation.session.unit_state', lambda: {
        'ActiveState': 'active', 'Description': 'D1MAX_NAV2_LOCALIZATION_TEST_V1:offline'})
    monkeypatch.setattr('d1max_navigation.session.api', lambda *a, **kw: pytest.fail('Must not call Web'))
    monkeypatch.setattr('d1max_navigation.session.subprocess.run', lambda *a, **kw: pytest.fail('Must not launch duplicate'))
    with pytest.raises(RuntimeError, match='already open'):
        start(None, tmp_path, tmp_path, tmp_path, tmp_path)


def test_map_center_matches_selected_floor():
    view = map_view({'resolution': .05, 'origin': [-85.95, -14, 0]}, {'width': 2130, 'height': 1246})
    assert view['X'] == pytest.approx(-32.7)
    assert view['Y'] == pytest.approx(17.15)
    assert 8 < view['Scale'] < 11


def test_rotated_map_center():
    view = map_view({'resolution': 1., 'origin': [0, 0, math.pi/2]}, {'width': 10, 'height': 4})
    assert view['X'] == pytest.approx(-2.)
    assert view['Y'] == pytest.approx(5.)


def test_offline_does_not_show_stale_robot_or_allow_seed():
    settings = {'map_frame': 'd1max_loc_map', 'map_topic': '/d1max/navigation/map', 'mode': 'offline'}
    value = prepare_rviz(SHARE / 'rviz/localization_test.rviz', settings,
        {'resolution': .05, 'origin': [0, 0, 0]}, {'width': 100, 'height': 50, 'name': 'test'})
    manager = value['Visualization Manager']
    assert all(not d['Enabled'] for d in manager['Displays'][1:] if d['Class'] != 'rviz_default_plugins/Marker')
    assert next(d for d in manager['Displays'] if d['Class'] == 'rviz_default_plugins/Marker')['Enabled']
    assert 'OFFLINE' in manager['Displays'][0]['Name']
    assert not any(t['Class'].endswith('SetInitialPose') for t in manager['Tools'])


def test_live_arrow_is_body_odometry_and_no_motion_tool():
    value = yaml.safe_load((SHARE / 'rviz/localization_test.rviz').read_text())
    manager = value['Visualization Manager']
    odom = next(d for d in manager['Displays'] if d['Class'].endswith('/Odometry'))
    assert odom['Topic']['Value'] == '/d1max/localization/odometry/global'
    assert odom['Keep'] == 1
    assert odom['Shape']['Value'] == 'Arrow'
    assert odom['Shape']['Shaft Radius'] < 0.05
    assert all('Goal' not in t['Class'] for t in manager['Tools'])
    assert not any('Navigation' in p['Class'] for p in value['Panels'])


def test_map_version_rejects_archived(tmp_path):
    root = tmp_path / 'map_manager/data/navigation2d'
    root.mkdir(parents=True)
    version = 'grid-' + 'a' * 24
    (root / 'state.json').write_text(json.dumps({'selected_id': version, 'archived': [version]}))
    with pytest.raises(ValueError, match='non-archived'):
        selected_map(tmp_path)


def test_launch_has_no_second_localizer_or_controller():
    text = (SHARE / 'launch/localization_test.launch.py').read_text()
    assert "package='nav2_map_server'" in text
    for forbidden in ("package='nav2_amcl'", "executable='controller_server'", "executable='bt_navigator'", "static_transform_publisher"):
        assert forbidden not in text
    # Bond identifiers must be local node names, resolved within namespace.
    assert "'node_names': ['map_server']" in text
