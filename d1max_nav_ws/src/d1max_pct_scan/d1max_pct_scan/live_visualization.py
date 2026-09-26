"""Algorithm-neutral RViz presentation. No planning or frame conversion here."""
from copy import deepcopy

PREFIX = '/d1max/live_planning/'
# Native occupancy snapshots publish at 3 Hz. A .25 s decay expired before the
# next healthy .333 s update, visibly blinking even without sensor loss. Keep a
# bounded .75 s display window (one dropped frame), not an immortal stale cloud.
LOCAL_MAP_DISPLAY_TTL = .75


def topic(value, *, transient=False, sensor=False):
    return {'Value': value, 'Depth': 1, 'History Policy': 'Keep Last',
            'Reliability Policy': 'Best Effort' if sensor else 'Reliable',
            'Durability Policy': 'Transient Local' if transient else 'Volatile'}


def display(kind, name, *, enabled=True, **values):
    return {'Class': 'rviz_default_plugins/' + kind, 'Name': name,
            'Enabled': enabled, 'Value': enabled, **values}


def group(name, children, *, enabled=True):
    return {'Class': 'rviz_common/Group', 'Name': name, 'Enabled': enabled,
            'Displays': children}


def configure(base, *, session_id='', layout='global'):
    """Build one presentation profile over the same planning topics.

    The local view uses SCAN's published sliding occupancy, not a new map or
    a cropped global PCD. Its camera follows the body while the fixed frame
    remains the existing map frame. These choices never alter collision data.
    Display/group names are shared with NavigationDiagnosticsPanel's layout
    switch; both profiles retain the complete tree for reversible switching.
    """
    if layout not in ('global', 'local'):
        raise ValueError(f'unsupported RViz layout: {layout!r}')
    local_layout = layout == 'local'
    cfg = deepcopy(base)
    vm = cfg['Visualization Manager']
    old = {d.get('Topic', {}).get('Value'): d for d in vm['Displays']}
    map_cloud = old['/d1max/localization/map_cloud']
    # Like the upstream navigation view: faint structural context behind the
    # task's route/active obstacle layers, not a second opaque live cloud.
    map_cloud.update(Name='建筑点云', Enabled=True, Value=True, Alpha=.10,
                     **{'Color Transformer': 'AxisColor', 'Axis': 'Z',
                        'Autocompute Value Bounds': True, 'Style': 'Points',
                        'Size (Pixels)': 1, 'Use Fixed Frame': True})
    cloud = old['/d1max/localization/lio/deskewed']
    cloud.update(Name='实时点云', Enabled=True, Value=True, Color='55; 215; 235', Alpha=.42,
                 **{'Color Transformer': 'FlatColor', 'Style': 'Points', 'Size (Pixels)': 1,
                    'Decay Time': .25, 'Use Fixed Frame': True})
    pose = old['/d1max/localization/odometry/global']
    # Keep the actual odometry display available for covariance inspection, but
    # use a finite-lifetime measured coordinate frame: Keep=1 has
    # no TTL and otherwise leaves a seemingly live robot after data loss.
    pose.update(Name='原始里程计 / 协方差', Enabled=False, Value=False)
    trajectory = old['/d1max/localization/trajectory']
    trajectory.update(Name='定位轨迹', Enabled=not local_layout, Value=not local_layout,
        **{'Line Width': .018, 'Alpha': .50,
        'Offset': {'X': 0., 'Y': 0., 'Z': 0.}})
    initial = old['/d1max/localization/scan_initial_preview']
    initial.update(Name='初值预览 · 未确认', Enabled=not local_layout, Value=not local_layout)
    vm['Displays'] = [
        group('定位', [
            display('Marker', '机器狗坐标系', Topic={**topic(PREFIX+'body_marker'), 'Depth': 10}),
            trajectory, initial]),
        group('地图', [map_cloud, display('PointCloud2', '可通行地面',
            Topic=topic(PREFIX+'traversable_surface', transient=True),
            **{'Position Transformer': 'XYZ', 'Color Transformer': 'RGB8', 'Style': 'Boxes',
               'Size (m)': .095, 'Alpha': .28, 'Decay Time': 0, 'Use Fixed Frame': True}),
            display('PointCloud2', '地图禁行区域', Topic=topic(PREFIX+'blocked_surface', transient=True),
            **{'Position Transformer': 'XYZ', 'Color Transformer': 'RGB8', 'Style': 'Boxes',
               'Size (m)': .095, 'Alpha': .12, 'Decay Time': 0, 'Use Fixed Frame': True})],
            enabled=not local_layout),
        group('全局规划', [display('Path', '全局路径',
            Topic=topic(PREFIX+'global_path_visual', transient=True),
            **{'Buffer Length': 1, 'Color': '65; 145; 255', 'Line Style': 'Billboards',
               'Line Width': .035 if local_layout else .065,
               'Alpha': .35 if local_layout else 1., 'Pose Style': 'None',
               # RViz-only offset clears the surface boxes. The published
               # path and all planning/ground-height checks remain unchanged.
               'Offset': {'X': 0., 'Y': 0., 'Z': .15}}),
            display('InteractiveMarkers', '3D 目标手柄', enabled=not local_layout, **{
                'Interactive Markers Namespace': '/d1max_live_goal', 'Show Axes': False,
                'Show Descriptions': True, 'Show Visual Aids': False})]),
        group('局部规划', [
            cloud,
            # Native rolling occupancy includes support ground, not just
            # classified obstacles. Keep it neutral and translucent so it
            # cannot be mistaken for a red, impassable surface classification.
            display('PointCloud2', '滑动占据地图（含地面）',
                Topic=topic(PREFIX+'scan/grid_map/occupancy', sensor=True),
                **{'Position Transformer': 'XYZ', 'Color Transformer': 'FlatColor',
                   'Color': '170; 182; 196', 'Style': 'Boxes', 'Size (m)': .08,
                   'Alpha': .20, 'Decay Time': LOCAL_MAP_DISPLAY_TTL, 'Use Fixed Frame': True}),
            # This is the native rolling buffer's real map-frame boundary,
            # not a view-dependent crop or an assertion that it is all free.
            display('Marker', '滑动窗口边界',
                    Topic=topic(PREFIX+'scan/grid_map/sliding_map_bbox')),
            display('MarkerArray', '跟踪路段与局部目标',
                    Topic=topic(PREFIX+'local_debug', transient=True)),
            display('MarkerArray', '搜索尝试与受阻位置',
                    Topic=topic(PREFIX+'local_attempt_debug', transient=True)),
            display('Marker', '局部轨迹',
                    Topic=topic(PREFIX+'scan_optimal', transient=True))],
            enabled=local_layout),
        group('调试', [pose,
            display('PointCloud2', '碰撞膨胀体素', enabled=False,
                Topic=topic(PREFIX+'scan/grid_map/occupancy_inflate', sensor=True),
                **{'Position Transformer': 'XYZ', 'Color Transformer': 'FlatColor',
                   'Color': '255; 150; 50', 'Style': 'Boxes', 'Size (m)': .08,
                   'Alpha': .18, 'Decay Time': LOCAL_MAP_DISPLAY_TTL})]),
    ]
    cfg['Panels'] = [
        {'Class': 'd1max_pct_rviz_tools/NavigationDiagnosticsPanel', 'Name': '导航监视',
         'Session ID': session_id, 'Layout': layout},
        {'Class': 'rviz_common/Displays', 'Name': '图层',
         'Property Tree Widget': {'Expanded': ['/定位1', '/局部规划1'] if local_layout
                                  else ['/定位1', '/地图1', '/全局规划1'],
                                  'Splitter Ratio': .52}},
        {'Class': 'rviz_common/Views', 'Name': '视角'},
    ]
    for tool in vm['Tools']:
        if tool['Class'] == 'rviz_default_plugins/SetInitialPose':
            tool['Topic'] = topic(PREFIX+'initialpose')
    vm['Tools'] += [
        {'Class': 'd1max_pct_rviz_tools/LiveGoal3D', 'Session ID': session_id},
        {'Class': 'rviz_default_plugins/SetGoal', 'Topic': topic(PREFIX+'goal')},
        {'Class': 'rviz_default_plugins/PublishPoint', 'Single click': True,
         'Topic': topic(PREFIX+'place_goal3d')},
    ]
    overview = {'Class': 'rviz_default_plugins/TopDownOrtho', 'Name': '全局总览',
                'Target Frame': '<Fixed Frame>', 'X': -6., 'Y': 30., 'Scale': 13.,
                'Angle': 0., 'Near Clip Distance': .01}
    follow = {'Class': 'rviz_default_plugins/Orbit', 'Name': '局部跟随',
              'Target Frame': 'd1max_loc_base_link', 'Distance': 10., 'Yaw': .8, 'Pitch': .85,
              'Focal Point': {'X': 1.5, 'Y': 0., 'Z': 0.}, 'Near Clip Distance': .02,
              'Field of View': .7853981634, 'Invert Z Axis': False}
    spatial = {'Class': 'rviz_default_plugins/Orbit', 'Name': '跨层 3D',
               'Target Frame': '<Fixed Frame>', 'Distance': 98., 'Yaw': .8, 'Pitch': .72,
               'Focal Point': {'X': -7., 'Y': 28., 'Z': 1.5}, 'Near Clip Distance': .02,
               'Field of View': .7853981634, 'Invert Z Axis': False}
    # Retain the former intermediate camera as an optional saved view. The
    # whole-building profile must show both floors; the local profile instead
    # follows the actual robot at a scale useful for obstacle inspection.
    navigation = dict(spatial, Name='导航视角', Distance=24.,
        **{'Focal Point': {'X': 0., 'Y': 1.5, 'Z': .25}})
    vm['Views'] = {'Current': deepcopy(follow if local_layout else spatial),
                  # Panel buttons address the first three saved views.
                  'Saved': [deepcopy(overview), follow, spatial, deepcopy(navigation)]}
    vm['Global Options'].update({'Background Color': '24; 29; 36', 'Frame Rate': 30})
    cfg['Window Geometry'].update(Width=1500, Height=950, **{'Hide Right Dock': False})
    return cfg
