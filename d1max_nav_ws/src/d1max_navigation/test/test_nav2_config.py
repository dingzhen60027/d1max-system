"""Contract tests for the actual Nav2 stack, not the map-only debug launch."""

import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def config():
    return yaml.safe_load((ROOT / 'config/nav2.yaml').read_text())


@pytest.fixture
def rviz():
    return yaml.safe_load((ROOT / 'rviz/navigation.rviz').read_text())


def params(config, name):
    block = config[name]
    if name in ('global_costmap', 'local_costmap'):
        block = block[name]
    return block['ros__parameters']


def test_real_nav2_servers_managed_in_order(config):
    nodes = params(config, 'lifecycle_manager_navigation')['node_names']
    assert set(nodes) == {
        'planner_server', 'controller_server', 'smoother_server',
        'behavior_server', 'bt_navigator', 'velocity_smoother', 'collision_monitor', 'waypoint_follower',
    }
    assert nodes.index('planner_server') < nodes.index('bt_navigator')
    assert nodes.index('controller_server') < nodes.index('bt_navigator')
    assert params(config, 'lifecycle_manager_localization')['node_names'] == ['map_server']
    assert 'amcl' not in config


def test_collision_monitor_is_independent_bounded_and_before_gate():
    c = yaml.safe_load((ROOT / 'config/collision_monitor.yaml').read_text())['collision_monitor']['ros__parameters']
    assert c['base_frame_id'] == 'd1max_loc_base_link'
    assert c['odom_frame_id'] == 'd1max_loc_odom'
    assert c['cmd_vel_in_topic'] == '/d1max/navigation/cmd_vel_smoothed'
    assert c['cmd_vel_out_topic'] == '/d1max/navigation/cmd_vel_collision_checked'
    assert c['source_timeout'] <= 0.5
    assert c['scan']['topic'] == '/d1max/navigation/scan'
    p = c['StopFootprint']
    assert p['action_type'] == 'stop' and p['enabled'] is True
    assert max(p['points'][0::2]) == pytest.approx(.49+.03+.1)
    assert max(p['points'][1::2]) == pytest.approx(.29+.03+.1)


def test_existing_localization_frames_not_replaced(config):
    assert params(config, 'map_server')['frame_id'] == 'd1max_loc_map'
    assert params(config, 'global_costmap')['global_frame'] == 'd1max_loc_map'
    assert params(config, 'local_costmap')['global_frame'] == 'd1max_loc_odom'
    for name in ('global_costmap', 'local_costmap', 'bt_navigator', 'behavior_server', 'smoother_server'):
        assert params(config, name)['robot_base_frame'] == 'd1max_loc_base_link'
    for name in ('controller_server', 'bt_navigator', 'velocity_smoother'):
        assert params(config, name)['odom_topic'] == '/d1max/localization/odometry/local'


@pytest.mark.parametrize('name', ['global_costmap', 'local_costmap'])
def test_costmap_has_static_live_obstacles_and_inflation(config, name):
    p = params(config, name)
    assert p['plugins'] == ['static_layer', 'obstacle_layer', 'inflation_layer']
    assert p['static_layer']['map_topic'] == '/d1max/navigation/map'
    assert p['static_layer']['map_subscribe_transient_local'] is True
    assert p['obstacle_layer']['scan']['topic'] == '/d1max/navigation/scan'
    assert p['obstacle_layer']['scan']['data_type'] == 'LaserScan'
    assert p['obstacle_layer']['scan']['clearing'] is True
    assert p['obstacle_layer']['scan']['marking'] is True
    assert p['obstacle_layer']['scan']['expected_update_rate'] <= 0.5
    assert p['inflation_layer']['inflation_radius'] > 0.5
    assert p['track_unknown_space'] is True


def test_common_explicit_footprint_and_local_rolling_window(config):
    g, l = (params(config, name) for name in ('global_costmap', 'local_costmap'))
    assert g['footprint'] == l['footprint']
    polygon = yaml.safe_load(g['footprint'])
    assert len(polygon) == 4
    assert max(p[0] for p in polygon) - min(p[0] for p in polygon) == pytest.approx(0.98)
    assert max(p[1] for p in polygon) - min(p[1] for p in polygon) == pytest.approx(0.58)
    assert all(math.isfinite(axis) for point in polygon for axis in point)
    assert l['rolling_window'] is True
    assert l['width'] == 8 and l['height'] == 8


def test_conservative_controller_and_smoother_consistent(config):
    c = params(config, 'controller_server')
    f = c['FollowPath']
    s = params(config, 'velocity_smoother')
    assert c['controller_frequency'] >= 20.0
    assert f['plugin'] == 'dwb_core::DWBLocalPlanner'
    assert 'ObstacleFootprint' in f['critics']
    assert f['min_vel_x'] == 0.0
    assert f['max_vel_y'] == 0.0
    assert s['max_velocity'] == [f['max_vel_x'], f['max_vel_y'], f['max_vel_theta']]
    assert f['max_vel_x'] <= 0.30 and f['max_vel_theta'] <= 0.50
    assert s['velocity_timeout'] <= 0.5
    assert s['feedback'] == 'CLOSED_LOOP'
    # Humble uses the singular parameter; newer distributions use a list.
    assert c['progress_checker_plugin'] == 'progress_checker'


def test_unknown_space_not_plannable_and_waypoints_stop_on_failure(config):
    assert params(config, 'planner_server')['GridBased']['allow_unknown'] is False
    assert params(config, 'waypoint_follower')['stop_on_failure'] is True


@pytest.mark.parametrize('filename,planner_tag', [
    ('navigate_to_pose.xml', 'ComputePathToPose'),
    ('navigate_through_poses.xml', 'ComputePathThroughPoses'),
])
def test_goal_trees_have_replanning_control_bounded_passive_recovery(config, filename, planner_tag):
    root = ET.parse(ROOT / 'behavior_trees' / filename).getroot()
    tags = {e.tag for e in root.iter()}
    assert {planner_tag, 'FollowPath', 'Wait', 'ClearEntireCostmap', 'RateController'} <= tags
    assert not tags.intersection({'Spin', 'BackUp', 'DriveOnHeading', 'AssistedTeleop'})
    for node in root.iter('RecoveryNode'):
        assert 0 < int(node.attrib['number_of_retries']) <= 2
    assert params(config, 'behavior_server')['behavior_plugins'] == ['wait']
    for node in root.iter('FollowPath'):
        assert node.attrib['controller_id'] in params(config, 'controller_server')['controller_plugins']
        assert node.attrib['goal_checker_id'] in params(config, 'controller_server')['goal_checker_plugins']


def test_rviz_displays_all_navigation_layers_and_goal_panel(rviz):
    vm = rviz['Visualization Manager']
    topics = {d.get('Topic', {}).get('Value') for d in vm['Displays']}
    assert {
        '/d1max/navigation/map',
        '/d1max/navigation/global_costmap/costmap',
        '/d1max/navigation/local_costmap/costmap',
        '/d1max/navigation/plan',
        '/d1max/navigation/local_plan',
        '/d1max/navigation/local_costmap/published_footprint',
        '/d1max/navigation/scan',
        '/d1max/localization/odometry/global',
        '/d1max/navigation/simulation_status',
    } <= topics
    assert vm['Global Options']['Fixed Frame'] == 'd1max_loc_map'
    assert any(t['Class'] == 'nav2_rviz_plugins/GoalTool' for t in vm['Tools'])
    assert any(p['Class'] == 'nav2_rviz_plugins/Navigation 2' for p in rviz['Panels'])
    for display in vm['Displays']:
        if display.get('Topic', {}).get('Value', '').endswith('/costmap'):
            assert display['Enabled'] is True
            assert display['Color Scheme'] == 'costmap'
            assert display['Topic']['Durability Policy'] == 'Transient Local'
            assert display['Update Topic']['Value'] == display['Topic']['Value'] + '_updates'


def test_configs_never_contain_hardware_command_topic(config, rviz):
    text = yaml.safe_dump([config, rviz])
    assert '/cmd_vel' not in text
    assert '/d1max_sdk_bridge/cmd_vel' not in text
    assert 'take_over' not in text
