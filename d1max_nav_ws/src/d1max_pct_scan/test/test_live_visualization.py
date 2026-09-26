"""Presentation contracts only: no estimator, graph or robot processes."""
from copy import deepcopy
from pathlib import Path

import yaml
import pytest

from d1max_pct_scan.live_visualization import PREFIX, LOCAL_MAP_DISPLAY_TTL, configure


def configuration():
    path = Path(__file__).resolve().parents[2] / 'd1max_navigation/rviz/localization_test.rviz'
    return yaml.safe_load(path.read_text())


def displays(result):
    return {d.get('Topic', {}).get('Value'): d
            for group in result['Visualization Manager']['Displays'] for d in group['Displays']}


def groups(result):
    return {g['Name']: g for g in result['Visualization Manager']['Displays']}


def visible_topics(result):
    return {d.get('Topic', {}).get('Value') for g in groups(result).values()
            for d in g['Displays'] if g['Enabled'] and d['Enabled']}


def test_crossfloor_default_is_3d_and_follow_has_local_scale():
    views = configure(configuration())['Visualization Manager']['Views']
    current = views['Current']
    assert current['Class'].endswith('/Orbit')
    assert 0.4 < current['Pitch'] < 1.2
    assert current['Name'] == '跨层 3D' and current['Distance'] >= 90.
    assert current['Target Frame'] == '<Fixed Frame>'
    local = next(v for v in views['Saved'] if v['Name'] == '局部跟随')
    assert local['Class'].endswith('/Orbit') and local['Distance'] == 10.0
    assert local['Target Frame'] == 'd1max_loc_base_link'
    assert any(v['Class'].endswith('/TopDownOrtho') for v in views['Saved'])
    assert any(v['Name']=='跨层 3D' and v['Distance']>=90. for v in views['Saved'])
    assert [v['Name'] for v in views['Saved'][:3]] == ['全局总览', '局部跟随', '跨层 3D']
    follow_views = configure(configuration(), layout='local')['Visualization Manager']['Views']
    assert follow_views['Current'] == follow_views['Saved'][1]
    assert follow_views['Saved'] == views['Saved']
    assert current == views['Saved'][2]


def test_raw_hit_voxels_do_not_masquerade_as_classified_obstacles():
    cfg = configure(configuration(), layout='local')
    local = next(g for g in cfg['Visualization Manager']['Displays'] if g['Name'] == '局部规划')
    assert local['Enabled']
    assert any(d.get('Topic',{}).get('Value')=='/d1max/localization/lio/deskewed'
               and d['Enabled'] for d in local['Displays'])
    obstacle = next(d for d in local['Displays'] if d.get('Topic', {}).get('Value') == PREFIX+'scan/grid_map/occupancy')
    assert obstacle['Enabled'] and obstacle['Value']
    assert obstacle['Name'] == '滑动占据地图（含地面）'
    assert '含地面' in obstacle['Name'] and obstacle['Alpha'] <= .25
    assert obstacle['Color'] == '170; 182; 196'
    assert obstacle['Topic']['Reliability Policy'] == 'Best Effort'
    assert obstacle['Decay Time'] == LOCAL_MAP_DISPLAY_TTL
    inflated = displays(cfg)[PREFIX+'scan/grid_map/occupancy_inflate']
    assert not inflated['Enabled']  # diagnostic envelope must not obscure the real obstacle
    assert inflated['Decay Time'] == LOCAL_MAP_DISPLAY_TTL
    # Native snapshots are 3 Hz: no deterministic 83 ms hole each frame, with
    # one lost frame tolerated visually only. The sensor/command lease is unchanged.
    assert 2/3. < LOCAL_MAP_DISPLAY_TTL <= 1.
    boundary = displays(cfg)[PREFIX+'scan/grid_map/sliding_map_bbox']
    assert boundary['Class'] == 'rviz_default_plugins/Marker'
    assert boundary['Name'] == '滑动窗口边界' and boundary['Enabled']
    assert boundary['Topic']['Durability Policy'] == 'Volatile'
    assert boundary['Topic']['Reliability Policy'] == 'Reliable'


def test_structural_context_is_faint_but_height_information_retained():
    layers = displays(configure(configuration()))
    source = layers['/d1max/localization/map_cloud']
    assert source['Color Transformer'] == 'AxisColor' and source['Axis'] == 'Z'
    assert source['Style'] == 'Points' and source['Size (Pixels)'] == 1
    assert source['Alpha'] < layers[PREFIX+'traversable_surface']['Alpha']
    assert source['Alpha'] < layers[PREFIX+'scan/grid_map/occupancy']['Alpha']
    assert layers[PREFIX+'blocked_surface']['Alpha'] < .3
    path = layers[PREFIX+'global_path_visual']
    assert path['Color'] == '65; 145; 255' and path['Alpha'] == 1.0
    assert path['Offset']['Z'] == .15  # only RViz offset, never geometry mutation


def test_handles_preserved_names_algorithm_neutral_and_base_not_mutated():
    base = configuration(); original = deepcopy(base)
    output = configure(base, session_id='visual-audit')
    assert base == original
    manager = output['Visualization Manager']
    all_layers = [d for g in manager['Displays'] for d in g['Displays']]
    handles = next(d for d in all_layers if d['Class'].endswith('/InteractiveMarkers'))
    assert handles['Enabled'] and handles['Interactive Markers Namespace'] == '/d1max_live_goal'
    assert not any('PCT' in d['Name'] or 'SCAN' in d['Name'] for d in all_layers)
    assert not any('RobotModel' in d['Class'] for d in all_layers)
    body = displays(output)[PREFIX+'body_marker']
    assert body['Name'] == '机器狗坐标系' and body['Enabled']
    assert body['Topic']['Depth'] >= 3
    tools = {t['Class']: t for t in manager['Tools']}
    assert tools['d1max_pct_rviz_tools/LiveGoal3D']['Session ID'] == 'visual-audit'
    assert tools['rviz_default_plugins/SetInitialPose']['Topic']['Value'] != tools['rviz_default_plugins/SetGoal']['Topic']['Value']


def test_profiles_separate_heavy_map_layers_and_keep_same_tree_and_coordinate_frame():
    base = configuration()
    global_cfg = configure(base, session_id='layer-audit', layout='global')
    local_cfg = configure(base, session_id='layer-audit', layout='local')
    global_groups, local_groups = groups(global_cfg), groups(local_cfg)
    assert list(global_groups) == list(local_groups) == ['定位', '地图', '全局规划', '局部规划', '调试']
    for name in global_groups:
        assert [d['Name'] for d in global_groups[name]['Displays']] == [
            d['Name'] for d in local_groups[name]['Displays']]
    assert global_groups['地图']['Enabled'] and not local_groups['地图']['Enabled']
    assert not global_groups['局部规划']['Enabled'] and local_groups['局部规划']['Enabled']
    assert displays(global_cfg).keys() == displays(local_cfg).keys()
    global_topics, local_topics = visible_topics(global_cfg), visible_topics(local_cfg)
    for topic in ('/d1max/localization/map_cloud', PREFIX+'traversable_surface', PREFIX+'blocked_surface'):
        assert topic in global_topics and topic not in local_topics
    for topic in ('/d1max/localization/lio/deskewed', PREFIX+'scan/grid_map/occupancy',
                  PREFIX+'scan/grid_map/sliding_map_bbox', PREFIX+'local_debug',
                  PREFIX+'local_attempt_debug', PREFIX+'scan_optimal'):
        assert topic in local_topics and topic not in global_topics
    for topic in (PREFIX+'body_marker', PREFIX+'global_path_visual'):
        assert topic in global_topics and topic in local_topics
    assert '/d1max/localization/trajectory' not in local_topics
    assert '/d1max/localization/scan_initial_preview' not in local_topics
    for layout, cfg in (('global', global_cfg), ('local', local_cfg)):
        assert cfg['Visualization Manager']['Global Options']['Fixed Frame'] == base[
            'Visualization Manager']['Global Options']['Fixed Frame']
        assert cfg['Panels'][0]['Layout'] == layout
        assert cfg['Panels'][0]['Session ID'] == 'layer-audit'
    route = displays(local_cfg)[PREFIX+'global_path_visual']
    assert route['Line Width'] == .035 and route['Alpha'] == .35
    assert displays(global_cfg)[PREFIX+'body_marker'] == displays(local_cfg)[PREFIX+'body_marker']


def test_local_profile_hides_goal_editing_but_preserves_tools_and_goal_topic():
    global_cfg = configure(configuration(), layout='global')
    local_cfg = configure(configuration(), layout='local')
    assert global_cfg['Visualization Manager']['Tools'] == local_cfg['Visualization Manager']['Tools']
    handle = next(d for d in groups(local_cfg)['全局规划']['Displays']
                  if d['Class'].endswith('/InteractiveMarkers'))
    assert not handle['Enabled'] and not handle['Value']
    assert handle['Interactive Markers Namespace'] == '/d1max_live_goal'


@pytest.mark.parametrize('layout', ['', 'all', 'LOCAL', None])
def test_invalid_layout_does_not_silently_show_mixed_view(layout):
    with pytest.raises(ValueError, match='unsupported RViz layout'):
        configure(configuration(), layout=layout)
