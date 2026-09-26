"""No ROS/systemd/network is started by these planning-preview lifecycle tests."""
import json
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from d1max_pct_planner import preview_session as session

PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_flat_floor_preview_does_not_label_derived_cloud_as_raw_source():
    cfg = yaml.safe_load((PACKAGE_ROOT / 'config/preview_flat_floor.yaml').read_text())
    result = session.rviz_config(cfg)
    displays = list(session.iter_displays(result['Visualization Manager']['Displays']))
    cloud = next(d for d in displays if d.get('Topic', {}).get('Value') == '/d1max/pct_preview/map')
    assert cloud['Name'] == '平地规划点云'
    original = yaml.safe_load((PACKAGE_ROOT / 'rviz/preview.rviz').read_text())
    assert len(displays) == len(list(session.iter_displays(original['Visualization Manager']['Displays'])))
    assert result['Panels'] == original['Panels']


@pytest.fixture
def harness(tmp_path, monkeypatch):
    root = tmp_path / 'log'
    root.mkdir()
    config = yaml.safe_load((PACKAGE_ROOT / 'config/preview.yaml').read_text())
    for key in ('map_pcd', 'tomogram_path'):
        path = tmp_path / (key + ('.npz' if key == 'tomogram_path' else ''))
        path.touch()
        config[key] = str(path)
    vendor = tmp_path / 'vendor'
    vendor.mkdir()
    config['vendor_root'] = str(vendor)
    config_file = tmp_path / 'preview.yaml'
    calls, bindings = [], []

    class FakeSocket:
        def __enter__(self):
            return self
        def __exit__(self, *_):
            return False
        def bind(self, address):
            bindings.append(address)

    def fake_run(command, **kwargs):
        calls.append((list(command), kwargs))
        return SimpleNamespace(returncode=0, stdout='', stderr='')

    monkeypatch.setattr(session, 'ROOT', root)
    monkeypatch.setattr(session, 'service', lambda: {'ActiveState': 'inactive'})
    monkeypatch.setattr(session.socket, 'socket', lambda: FakeSocket())
    monkeypatch.setattr(session.subprocess, 'run', fake_run)
    monkeypatch.setattr(session.uuid, 'uuid4', lambda: SimpleNamespace(hex='123456789abcdef0123456789abcdef0'))
    monkeypatch.setattr(session.os, 'environ', {
        'PATH': '/test/bin', 'DISPLAY': ':7',
        'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp', 'ROS_DOMAIN_ID': '80',
        'ZENOH_CONFIG_OVERRIDE': 'connect/endpoints=["tcp/robot:7447"]',
        'ZENOH_SESSION_CONFIG_URI': '/robot.json5',
        'D1MAX_NAVIGATION_CONTROL_ENABLED': 'true',
    })

    def start(headless=True):
        config_file.write_text(yaml.safe_dump(config))
        session.start(SimpleNamespace(config=config_file, headless=headless))
        return json.loads((root / 'last_session.json').read_text())

    return SimpleNamespace(start=start, cfg=config, calls=calls, bindings=bindings,
                           root=root, monkeypatch=monkeypatch)


def test_planning_only_start_has_private_transport_and_owned_process_group(harness):
    snapshot = harness.start()
    assert snapshot['mode'] == 'planning_preview'
    assert snapshot['robot_connected'] is False
    assert snapshot['real_motion_enabled'] is False
    assert snapshot['router_port'] == 7465
    command, options = harness.calls[0]
    assert '--unit=d1max-pct-preview.service' in command
    assert '--property=Description=' + session.OWNER + snapshot['session_id'] in command
    assert '--property=KillMode=mixed' in command
    assert '--property=KillSignal=SIGINT' in command
    assert '--property=TimeoutStopSec=15' in command
    assert '--property=SendSIGKILL=yes' in command
    assert '--setenv=RMW_IMPLEMENTATION=rmw_zenoh_cpp' in command
    assert '--setenv=ROS_DOMAIN_ID=24' in command
    assert command[-3] == 'run'
    assert command[-2] == '--session'
    assert options['check'] is True
    assert not any('D1MAX_NAVIGATION_CONTROL_ENABLED' in part or 'ZENOH_CONFIG_OVERRIDE=' in part for part in command)


def test_private_7465_router_has_no_upstream_or_discovery(harness):
    snapshot = harness.start()
    assert harness.bindings == [('127.0.0.1', 7465)]
    directory = Path(snapshot['directory'])
    client = json.loads((directory / 'client.json5').read_text())
    router = json.loads((directory / 'router.json5').read_text())
    assert client['connect']['endpoints'] == ['tcp/127.0.0.1:7465']
    assert router['listen']['endpoints'] == ['tcp/127.0.0.1:7465']
    assert router['connect']['endpoints'] == []
    for cfg in (router, client):
        assert cfg['scouting']['multicast']['enabled'] is False
        assert cfg['scouting']['gossip']['enabled'] is False


def test_corridor_refinement_is_explicit_and_reaches_worker_parameters(harness):
    harness.cfg['map_backend'] = 'official_tomogram'
    harness.cfg.pop('planning_grid', None)
    harness.cfg['path_refinement'] = 'visibility_c2'
    harness.cfg['refinement_corner_cut_m'] = 1
    snapshot = harness.start()
    params = yaml.safe_load(Path(snapshot['server_parameters']).read_text())['/**']['ros__parameters']
    assert params['path_refinement'] == 'visibility_c2'
    assert params['refinement_corner_cut_m'] == 1.0
    assert isinstance(params['refinement_corner_cut_m'], float)


@pytest.mark.parametrize('mode,cut', [('unvalidated', 1.5), ('visibility_c2', 0),
                                     ('visibility_c2', True), ('visibility_c2', float('nan')),
                                     ('visibility_c2', 6)])
def test_invalid_refinement_configuration_fails_before_start(harness, mode, cut):
    harness.cfg.update(path_refinement=mode, refinement_corner_cut_m=cut)
    with pytest.raises(ValueError, match='refinement|Refinement'):
        harness.start()
    assert not harness.calls


def test_empty_start_is_omitted_from_ros_params_and_selection_is_strict(harness):
    snapshot = harness.start()
    params = yaml.safe_load(Path(snapshot['server_parameters']).read_text())['/**']['ros__parameters']
    assert 'initial_start_xyz' not in params
    assert params['max_selection_height_error_m'] == 0.08
    assert params['planning_frame'] == 'd1max_loc_map'
    assert params['use_sim_time'] is False
    assert params['place_endpoints_on_start'] is True
    assert params['output_directory'] == snapshot['directory']
    assert yaml.safe_load(Path(snapshot['config']).read_text())['initial_start_xyz'] == []


def test_optional_start_xyz_has_explicit_float_array(harness):
    harness.cfg['initial_start_xyz'] = [1, 2, 3]
    snapshot = harness.start()
    params = yaml.safe_load(Path(snapshot['server_parameters']).read_text())['/**']['ros__parameters']
    assert params['initial_start_xyz'] == [1.0, 2.0, 3.0]
    assert all(type(value) is float for value in params['initial_start_xyz'])


def test_route_restore_requires_an_explicit_file_pair(harness):
    harness.cfg['restore_route_file'] = harness.cfg['map_pcd']
    with pytest.raises(ValueError, match='both existing absolute'):
        harness.start()
    harness.cfg['restore_tomogram_path'] = harness.cfg['tomogram_path']
    snapshot = harness.start()
    params = yaml.safe_load(Path(snapshot['server_parameters']).read_text())['/**']['ros__parameters']
    assert params['restore_route_file'] == harness.cfg['map_pcd']
    assert params['restore_tomogram_path'] == harness.cfg['tomogram_path']


def test_crossfloor_config_reaches_server_and_keeps_planning_only_session(harness):
    route = harness.root / 'route.yaml'
    route.write_text('schema_version: 1\n')
    harness.cfg['crossfloor_route_config'] = str(route)
    snapshot = harness.start()
    params = yaml.safe_load(Path(snapshot['server_parameters']).read_text())['/**']['ros__parameters']
    assert params['crossfloor_route_config'] == str(route)
    assert snapshot['real_motion_enabled'] is False
    rviz = yaml.safe_load(Path(snapshot['rviz_config']).read_text())
    displays = list(session.iter_displays(rviz['Visualization Manager']['Displays']))
    cloud = next(d for d in displays if d.get('Topic', {}).get('Value') == '/d1max/pct_preview/map')
    assert cloud['Name'] == '跨层规划点云'


@pytest.mark.parametrize('path,backend', [('relative.yaml', 'official_tomogram'),
                                       ('/missing/route.yaml', 'official_tomogram'),
                                       ('existing', 'legacy_grid')])
def test_invalid_crossfloor_adapter_config_never_starts_service(harness, path, backend):
    if path == 'existing':
        target = harness.root / 'route.yaml'
        target.write_text('schema_version: 1\n')
        path = str(target)
    harness.cfg.update(crossfloor_route_config=path, map_backend=backend)
    with pytest.raises(ValueError, match='Cross-floor'):
        harness.start()
    assert not harness.calls


def test_rviz_uses_3d_tools_and_xyz_handles_no_2d_goal(harness):
    snapshot = harness.start(headless=False)
    rviz = yaml.safe_load(Path(snapshot['rviz_config']).read_text())
    manager = rviz['Visualization Manager']
    tools = {tool['Class'] for tool in manager['Tools']}
    assert 'd1max_pct_rviz_tools/Start3D' in tools
    assert 'd1max_pct_rviz_tools/Goal3D' in tools
    assert 'rviz_default_plugins/SetGoal' not in tools
    assert 'd1max_pct_rviz_tools/PlanningPanel' in {panel['Class'] for panel in rviz['Panels']}
    handles = next(display for display in session.iter_displays(manager['Displays']) if display['Class'] == 'rviz_default_plugins/InteractiveMarkers')
    assert handles['Interactive Markers Namespace'] == '/pct_preview_points'
    assert 'Update Topic' not in handles  # ROS 1 key silently fails in Humble.
    assert manager['Views']['Current']['Pitch'] == 0.625
    assert manager['Views']['Current']['Yaw'] == 0.8
    assert snapshot['headless'] is False


def test_preview_ports_legbot_global_display_style_without_fake_live_layers():
    cfg = yaml.safe_load((PACKAGE_ROOT / 'config/preview_crossfloor_complete.yaml').read_text())
    manager = session.rviz_config(cfg)['Visualization Manager']
    groups = {item['Name']: item for item in manager['Displays'] if item['Class'] == 'rviz_common/Group'}
    assert groups['全局规划']['Enabled'] is True
    assert groups['通行代价']['Enabled'] is False
    cloud, voxels, path = groups['全局规划']['Displays']
    assert cloud['Style'] == 'Points' and cloud['Alpha'] == .22
    assert voxels['Style'] == 'Boxes' and voxels['Size (m)'] == .08 and voxels['Alpha'] == .65
    assert cloud['Color Transformer'] == voxels['Color Transformer'] == 'AxisColor'
    assert cloud['Autocompute Value Bounds'] == voxels['Autocompute Value Bounds']
    assert cloud['Autocompute Value Bounds']['Value'] is False
    assert path['Color'] == '255; 60; 40' and path['Line Width'] == .12
    assert path['Offset']['Z'] == .18  # Lift display only, not navigation data.
    assert all(d['Color Transformer'] == 'RGB8' for d in groups['通行代价']['Displays'])
    for display in session.iter_displays(manager['Displays']):
        assert display['Class'] not in ('rviz_default_plugins/RobotModel', 'rviz_default_plugins/Odometry')
        if 'Topic' in display:
            assert display['Topic']['Value'].startswith('/d1max/pct_preview/')
            assert display['Topic']['Durability Policy'] == 'Transient Local'
    assert manager['Global Options']['Background Color'] == '38; 38; 38'
    views = manager['Views']
    assert {v['Name'] for v in views['Saved']} == {'跨层总览', '俯视', '楼梯'}
    assert all(v['Target Frame'] == cfg['planning_frame'] for v in [views['Current'], *views['Saved']])


def test_grouped_display_customization_does_not_mutate_template():
    template = (PACKAGE_ROOT / 'rviz/preview.rviz').read_text()
    crossfloor = yaml.safe_load((PACKAGE_ROOT / 'config/preview_crossfloor_complete.yaml').read_text())
    session.rviz_config(crossfloor)
    single = yaml.safe_load((PACKAGE_ROOT / 'config/preview.yaml').read_text())
    config = session.rviz_config(single)
    assert config['Visualization Manager']['Views']['Saved'] == []
    assert (PACKAGE_ROOT / 'rviz/preview.rviz').read_text() == template


@pytest.mark.parametrize('state', session.ACTIVE_STATES)
def test_existing_service_never_stacked_or_replaced(harness, state):
    harness.monkeypatch.setattr(session, 'service', lambda: {'ActiveState': state})
    with pytest.raises(RuntimeError, match='already exists'):
        harness.start()
    assert harness.calls == []
    assert harness.bindings == []


def test_unknown_router_is_not_reused(harness):
    class OccupiedSocket:
        def __enter__(self):
            return self
        def __exit__(self, *_):
            return False
        def bind(self, _):
            raise OSError('Address already in use')
    harness.monkeypatch.setattr(session.socket, 'socket', lambda: OccupiedSocket())
    with pytest.raises(OSError, match='already in use'):
        harness.start()
    assert harness.calls == []


@pytest.mark.parametrize('key,value', [
    ('map_pcd', '/not/a/file'), ('planning_grid', '/not/a/grid'),
    ('vendor_root', '/not/a/vendor'), ('planning_frame', '/map'),
    ('initial_start_xyz', [1, 2]), ('initial_start_xyz', [1, 2, float('nan')]),
    ('initial_start_xyz', [True, 2, 3]), ('planning_timeout_s', 0),
    ('selection_max_age_s', float('inf')), ('handle_scale_m', -1),
    ('max_selection_height_error_m', float('nan')), ('map_display_max_points', True),
    ('max_selection_height_error_m', .3), ('cost_margin_m', 0),
    ('minimum_clearance_m', -1), ('optimization_guard_cells', 1.5),
    ('max_ground_step_m', float('nan')), ('max_heading_rate', False),
    ('astar_cost_weight',0), ('astar_cost_weight',10.01), ('astar_cost_weight',True),
    ('astar_cost_weight',float('inf')), ('optimizer_cost_margin',-1),
    ('optimizer_cost_margin',20), ('optimizer_cost_margin',float('nan')),
])
def test_invalid_config_rejected_before_start(harness, key, value):
    harness.cfg[key] = value
    with pytest.raises(ValueError):
        harness.start()
    assert harness.calls == []


def test_control_parameters_cannot_be_smuggled_into_config(harness):
    harness.cfg['motion_enabled'] = True
    with pytest.raises(ValueError, match='Unknown preview parameters'):
        harness.start()
    assert harness.calls == []


def test_native_cost_configuration_reaches_ros_with_numeric_type(harness):
    harness.cfg.update(astar_cost_weight=1,optimizer_cost_margin=10)
    snapshot=harness.start()
    params=yaml.safe_load(Path(snapshot['server_parameters']).read_text())['/**']['ros__parameters']
    assert params['astar_cost_weight']==1.0 and type(params['astar_cost_weight']) is float
    assert params['optimizer_cost_margin']==10.0 and type(params['optimizer_cost_margin']) is float


def test_stop_unknown_unit_refused(harness):
    harness.monkeypatch.setattr(session, 'service', lambda: {'ActiveState': 'active', 'Description': 'other'})
    with pytest.raises(RuntimeError, match='Unknown preview service owner'):
        session.stop()
    assert harness.calls == []


def test_stop_only_targets_preview_unit(harness):
    harness.monkeypatch.setattr(session, 'service', lambda: {'ActiveState': 'active', 'Description': session.OWNER + 'sid'})
    session.stop()
    assert harness.calls == [(['systemctl', '--user', 'stop', session.UNIT], {'check': True, 'timeout': 20})]


def test_runtime_clears_inherited_live_zenoh_configs(harness):
    session.os.environ['ZENOH_ROUTER_CONFIG_URI'] = '/live/router.json'
    session.os.environ['ZENOH_SESSION_CONFIG'] = '/live/client.json'
    session.runtime_environment(Path('/preview'))
    env = session.os.environ
    assert env['RMW_IMPLEMENTATION'] == 'rmw_zenoh_cpp'
    assert env['ROS_DOMAIN_ID'] == '24'
    assert env['ZENOH_SESSION_CONFIG_URI'] == '/preview/client.json5'
    assert env['ZENOH_ROUTER_CONFIG_URI'] == '/preview/router.json5'
    assert 'ZENOH_CONFIG_OVERRIDE' not in env
    assert 'ZENOH_SESSION_CONFIG' not in env


@pytest.mark.parametrize('headless,count', [(True, 2), (False, 3)])
def test_runtime_executable_allowlist_has_no_controller_simulation_or_sdk(headless, count):
    commands = session.runtime_commands({'headless': headless, 'server_parameters': '/params.yaml', 'rviz_config': '/preview.rviz'}, lambda _: '/ros')
    assert len(commands) == count
    assert [Path(command[0]).name for command in commands] == ['rmw_zenohd', 'pct_preview_server'] + ([] if headless else ['rviz2'])
    assert all('pct_scan' not in ' '.join(command) for command in commands)
