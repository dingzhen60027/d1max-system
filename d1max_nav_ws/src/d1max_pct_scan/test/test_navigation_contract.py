"""No ROS nodes, SDK, services or command publication in these regressions."""
from copy import deepcopy
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import pytest
import yaml
from d1max_pct_planner.paths import expand_tree

from d1max_pct_scan import live_session
from d1max_pct_scan import navigation_contract as contract


@pytest.fixture
def localization():
    cfg = expand_tree(yaml.safe_load(live_session.DEFAULT_CONFIG.read_text()))
    return yaml.safe_load(Path(cfg['localization_config']).read_text())


def test_localization_roles_share_body_and_have_one_dynamic_tf_authority(localization):
    value = contract.localization_contract(localization, 'd1max_loc_map')
    assert value['body_pose_reference'] == 'body_center'
    assert value['twist_reference'] == 'body_center_body_axes'
    assert value['global_planning']['frame'] == value['local_planning']['frame'] == 'd1max_loc_map'
    assert value['required_control']['frame'] == 'd1max_loc_odom'
    assert value['required_control']['implemented'] is False
    assert value['ground_to_body_height_applications'] == 1
    assert value['tf_authority'] == 'navigation_output'


@pytest.mark.parametrize('node,key,value', [
    ('localization_pipeline', 'backend', 'legacy_ekf'),
    ('navigation_estimation', 'enabled', False),
    ('navigation_estimation', 'enabled', 1),
    ('navigation_estimation', 'body_frame', 'tracking_is_not_body'),
    ('lio_localizer', 'map_frame', 'conditioned_map'),
    ('lio_localizer', 'odom_frame', 'different_odom'),
    ('lio_localizer', 'tracking_frame', 'different_tracking'),
    ('lio_global_matcher', 'world_frame', 'different_map'),
    ('lio_global_matcher', 'input_deskewed', False),
    ('lio_global_matcher', 'deskew_enabled', True),
    ('lio_global_matcher', 'translation_deskew_enabled', True),
    ('lio_global_matcher', 'imu_topic', '/wrong_imu'),
    ('dual_lidar_adapter', 'target_frame', 'different_imu_axes'),
    ('/d1max/localization/lio/laserMapping', 'publish.tf_enabled', True),
    ('/d1max/localization/lio/laserMapping', 'localization.enabled', False),
    ('/d1max/localization/lio/laserMapping', 'publish.scan_publish_en', False),
    ('/d1max/localization/lio/laserMapping', 'publish.body_frame', 'd1max_loc_base_link'),
    ('/d1max/localization/lio/laserMapping', 'common.imu_topic', '/other_imu'),
    ('/d1max/localization/lio/laserMapping', 'publish.scan_bodyframe_pub_en', False),
    ('ekf_navigation', 'publish_tf', True),
])
def test_mismatched_contract_rejected_before_launch(localization, node, key, value):
    localization[node]['ros__parameters'][key] = value
    with pytest.raises(ValueError):
        contract.localization_contract(localization, 'd1max_loc_map')


def test_unused_legacy_ekf_tf_does_not_create_false_conflict(localization):
    assert localization['ekf_local']['ros__parameters']['publish_tf'] is True
    contract.localization_contract(localization, 'd1max_loc_map')


@pytest.mark.parametrize('mode', ['front', 'rear'])
def test_single_source_projector_cannot_satisfy_dual_native_map_contract(localization, mode):
    cfg = {'perception_backend': 'per_sensor_rays'}
    contract.validate_perception_branch(cfg, localization)
    localization['dual_lidar_adapter']['ros__parameters']['lidar_mode'] = mode
    with pytest.raises(ValueError, match='dual lidar'):
        contract.validate_perception_branch(cfg, localization)


def test_matching_renamed_imu_topics_still_cannot_feed_fixed_predictor(localization):
    localization['dual_lidar_adapter']['ros__parameters']['output_imu_topic'] = '/not_predictor_input'
    localization['/d1max/localization/lio/laserMapping']['ros__parameters']['common.imu_topic'] = '/not_predictor_input'
    with pytest.raises(ValueError, match='input contract'):
        contract.localization_contract(localization, 'd1max_loc_map')


def test_map_identity_depends_on_contents_not_date_or_path(tmp_path):
    cfg = {}
    for key in contract.ARTIFACT_KEYS:
        path = tmp_path/key
        path.write_text(key)
        cfg[key] = str(path)
    first, _ = contract.artifact_identity(cfg)
    assert first.startswith('navigation-map-')
    moved = tmp_path/'renamed'
    Path(cfg['map_pcd']).rename(moved)
    cfg['map_pcd'] = str(moved)
    assert contract.artifact_identity(cfg)[0] == first
    moved.write_text('new coordinates, same filenames and frame label')
    assert contract.artifact_identity(cfg)[0] != first


def test_session_bundle_checks_effective_configs_but_not_camera(tmp_path):
    with patch.object(live_session, 'ROOT', tmp_path), \
         patch.object(live_session.subprocess, 'Popen') as process, \
         patch.object(live_session, 'build_opener') as network:
        directory, session = live_session.prepare()
        process.assert_not_called()
        network.assert_not_called()
    contract.verify_bundle(directory, session)
    assert session == json.loads((directory/'session.json').read_text())
    (directory/'live.rviz').write_text('presentation only')
    contract.verify_bundle(directory, session)
    altered = deepcopy(session)
    altered['body_height'] += .1
    with pytest.raises(ValueError, match='bundle'):
        contract.verify_bundle(directory, altered)
    (directory/'scan.yaml').write_text('different_collision_geometry: true')
    with pytest.raises(ValueError, match='bundle'):
        contract.verify_bundle(directory, session)


def test_saved_session_requires_verified_bundle_before_any_process(tmp_path, monkeypatch):
    session = dict(id='s', mode='LIVE_VISUALIZATION_NO_MOTION', motion_control_enabled=False)
    (tmp_path/'session.json').write_text(json.dumps(session))
    import os
    monkeypatch.setenv('INVOCATION_ID', 'test')
    monkeypatch.setattr(live_session, 'unit', lambda name=live_session.UNIT:
        dict(MainPID=str(os.getpid()), Description=live_session.OWNER+'s', ActiveState='active')
        if name == live_session.UNIT else dict(ActiveState='inactive'))
    with patch.object(live_session.subprocess, 'Popen') as spawn:
        with pytest.raises(ValueError, match='bundle'):
            live_session.run(tmp_path)
        spawn.assert_not_called()


@pytest.mark.parametrize('node,key,value', [
    ('bridge', 'localization_session_id', 'previous-session'),
    ('bridge', 'body_frame', 'd1max_loc_tracking'),
    ('bridge', 'body_height', 1.1),
    ('scan', 'fsm.reference_path_z_offset', 1.1),
    ('scan', 'grid_map.use_projected_rays', False),
    ('scan', 'grid_map.require_localization_context', False),
    ('scan', 'grid_map.double_cylinder_radius', .01),
    ('global', 'planning_manifest', '/other-map/manifest.json'),
])
def test_generated_handoff_cannot_silently_drift(tmp_path, node, key, value):
    cfg = live_session.load_config(live_session.DEFAULT_CONFIG)
    cfg.update(id='handoff-test', version_id='map-fixture', **{name: 'test' for name in (
        'ground_support_index', 'ground_support_sha256',
        'ground_support_source_pcd_sha256', 'ground_support_tomogram_sha256')})
    params = dict(bridge=live_session.bridge_parameters(cfg, cfg['id']),
                  scan=live_session.scan_parameters(cfg, cfg['id']),
                  global_={name: cfg[name] for name in ('planning_manifest',
                      'tomogram_npz', 'crossfloor_route_config')})
    params['global'] = dict(params.pop('global_'), session_id=cfg['id'])
    for name, entry in params.items():
        (tmp_path/(name+'.yaml')).write_text(yaml.safe_dump({'/**': {'ros__parameters': entry}}))
    live_session.prepare_tree(tmp_path, cfg, live_session.WS)
    contract.validate_effective_wiring(tmp_path, cfg)
    params[node][key] = value
    (tmp_path/(node+'.yaml')).write_text(yaml.safe_dump({'/**': {'ros__parameters': params[node]}}))
    with pytest.raises(ValueError):
        contract.validate_effective_wiring(tmp_path, cfg)


def fake_install(tmp_path):
    prefixes = {}
    for package, files in {
        'd1max_localization': ('lib/d1max_localization/dual_lidar_adapter',
                              'lib/d1max_localization/fused_icp_matcher',
                              'lib/d1max_localization/lio_localizer',
                              'lib/d1max_localization/lio_predictor',
                              'lib/d1max_localization/navigation_output',
                              'share/d1max_localization/launch/localization.launch.py'),
        'faster_lio': ('lib/faster_lio/run_mapping_online',),
        'scan_planner': ('lib/scan_planner/scan_planner_node',),
        'd1max_navigation_bt': ('lib/d1max_navigation_bt/navigator_node',
            'share/d1max_navigation_bt/trees/navigate_committed_route.xml'),
        'd1max_pct_rviz_tools': ('lib/libd1max_pct_rviz_tools.so',
            'share/d1max_pct_rviz_tools/plugins.xml'),
    }.items():
        prefixes[package] = str(tmp_path/'install'/package)
        for filename in files:
            path = Path(prefixes[package])/filename
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text('fixture only, not executable')
    specs = {}
    for package in ('d1max_pct_scan', 'd1max_pct_planner', 'd1max_localization'):
        root = tmp_path/'src'/package/package
        root.mkdir(parents=True)
        (root/'__init__.py').write_text('')
        (root/'code.py').write_text('VERSION = 1')
        specs[package] = SimpleNamespace(origin=str(root/'__init__.py'))
    root = tmp_path/'tools/pointcloud_preprocessing'
    root.mkdir(parents=True)
    (root/'__init__.py').write_text('')
    specs['tools.pointcloud_preprocessing'] = SimpleNamespace(origin=str(root/'__init__.py'))
    for name in ('ground_path_bridge', 'flat_floor', 'pcd_io'):
        (root/(name+'.py')).write_text('VERSION = 1')
        specs['tools.pointcloud_preprocessing.'+name] = SimpleNamespace(origin=str(root/(name+'.py')))
    for package in ('d1max_planning_interfaces', 'scan_planner_msgs', 'd1max_navigation_bt_interfaces'):
        prefix = tmp_path/'install'/package
        prefixes[package] = str(prefix)
        (prefix/'lib').mkdir(parents=True)
        for kind in ('generator_c', 'generator_py', 'typesupport_c', 'typesupport_cpp',
                     'typesupport_introspection_c', 'typesupport_introspection_cpp'):
            (prefix/'lib'/('lib'+package+'__rosidl_'+kind+'.so')).write_text('binary fixture 1')
        root = prefix/'local/lib/python3.10/dist-packages'/package
        root.mkdir(parents=True)
        (root/'__init__.py').write_text('')
        (root/'msg').mkdir()
        (root/'msg/_fixture.py').write_text('SCHEMA = 1')
        (root/(package+'_s__rosidl_typesupport_c.cpython-310-test.so')).write_text('extension fixture 1')
        (root/('lib'+package+'__rosidl_generator_py.so')).write_text('generator fixture 1')
        specs[package] = SimpleNamespace(origin=str(root/'__init__.py'))
    return prefixes, specs


def test_behavior_tree_runtime_and_generated_actions_are_pinned_not_assumed_deployed(tmp_path):
    prefixes, specs = fake_install(tmp_path)
    kwargs = dict(prefix_lookup=prefixes.__getitem__, spec_lookup=specs.get, include_behavior_tree=True)
    before = contract.deployment_evidence(tmp_path, **kwargs)
    binary = 'd1max_navigation_bt/lib/d1max_navigation_bt/navigator_node'
    xml = 'd1max_navigation_bt/share/d1max_navigation_bt/trees/navigate_committed_route.xml'
    assert binary in before['files'] and xml in before['files']
    assert any(k.startswith('d1max_navigation_bt_interfaces/python/') for k in before['files'])
    assert 'd1max_pct_rviz_tools/lib/libd1max_pct_rviz_tools.so' in before['files']
    Path(before['files'][binary]['path']).write_text('changed binary')
    after = contract.deployment_evidence(tmp_path, **kwargs)
    assert before != after
    Path(after['files'][binary]['path']).unlink()
    with pytest.raises(ValueError, match='executable'):
        contract.deployment_evidence(tmp_path, **kwargs)


def test_actual_resolved_sources_and_binary_are_separate_evidence(tmp_path):
    prefixes, specs = fake_install(tmp_path)
    before = contract.deployment_evidence(tmp_path, prefix_lookup=prefixes.__getitem__,
                                          spec_lookup=specs.get)
    source = tmp_path/'src/d1max_pct_scan/d1max_pct_scan/code.py'
    source.write_text('VERSION = 2')
    after = contract.deployment_evidence(tmp_path, prefix_lookup=prefixes.__getitem__,
                                         spec_lookup=specs.get)
    binary = 'scan_planner/lib/scan_planner/scan_planner_node'
    assert before['files'][binary] == after['files'][binary]
    assert before['files']['d1max_pct_scan/code.py'] != after['files']['d1max_pct_scan/code.py']
    with patch.object(contract, 'deployment_evidence', return_value=after):
        with pytest.raises(ValueError, match='deployment changed'):
            contract.verify_deployment(before, tmp_path)


def test_foreign_ament_overlay_and_python_copy_rejected(tmp_path):
    prefixes, specs = fake_install(tmp_path)
    wrong = dict(prefixes, scan_planner='/opt/other-install')
    with pytest.raises(ValueError, match='foreign AMENT'):
        contract.deployment_evidence(tmp_path, prefix_lookup=wrong.__getitem__, spec_lookup=specs.get)
    specs['d1max_pct_scan'] = SimpleNamespace(origin='/tmp/different_repository/__init__.py')
    with pytest.raises(ValueError, match='outside actual workspace'):
        contract.deployment_evidence(tmp_path, prefix_lookup=prefixes.__getitem__, spec_lookup=specs.get)


@pytest.mark.parametrize('relative', [
    'tools/pointcloud_preprocessing/ground_path_bridge.py',
    'tools/pointcloud_preprocessing/flat_floor.py',
    'tools/pointcloud_preprocessing/pcd_io.py',
    'install/d1max_localization/lib/d1max_localization/lio_localizer',
    'install/d1max_localization/lib/d1max_localization/lio_predictor',
    'install/d1max_localization/lib/d1max_localization/navigation_output',
    'install/d1max_planning_interfaces/lib/libd1max_planning_interfaces__rosidl_typesupport_cpp.so',
    'install/scan_planner_msgs/lib/libscan_planner_msgs__rosidl_typesupport_introspection_cpp.so',
    'install/d1max_planning_interfaces/local/lib/python3.10/dist-packages/d1max_planning_interfaces/msg/_fixture.py',
    'install/scan_planner_msgs/local/lib/python3.10/dist-packages/scan_planner_msgs/scan_planner_msgs_s__rosidl_typesupport_c.cpython-310-test.so',
])
def test_runtime_bridge_wrappers_and_interface_edits_change_deployment(tmp_path, relative):
    prefixes, specs = fake_install(tmp_path)
    before = contract.deployment_evidence(tmp_path, prefix_lookup=prefixes.__getitem__, spec_lookup=specs.get)
    (tmp_path/relative).write_text('changed runtime dependency')
    after = contract.deployment_evidence(tmp_path, prefix_lookup=prefixes.__getitem__, spec_lookup=specs.get)
    assert before != after
    assert any(record['path'] == str(tmp_path/relative) for record in after['files'].values())
    with patch.object(contract, 'deployment_evidence', return_value=after):
        with pytest.raises(ValueError, match='deployment changed'):
            contract.verify_deployment(before, tmp_path)


@pytest.mark.parametrize('package', ['d1max_planning_interfaces', 'scan_planner_msgs'])
def test_interface_foreign_prefix_or_python_origin_rejected(tmp_path, package):
    prefixes, specs = fake_install(tmp_path)
    wrong = dict(prefixes, **{package: '/opt/foreign_interfaces'})
    with pytest.raises(ValueError, match='foreign AMENT'):
        contract.deployment_evidence(tmp_path, prefix_lookup=wrong.__getitem__, spec_lookup=specs.get)
    specs[package] = SimpleNamespace(origin=str(tmp_path/'src/foreign_interface/__init__.py'))
    with pytest.raises(ValueError, match='foreign generated Python'):
        contract.deployment_evidence(tmp_path, prefix_lookup=prefixes.__getitem__, spec_lookup=specs.get)


def test_resolved_bridge_dependency_cannot_come_from_another_package_copy(tmp_path):
    prefixes, specs = fake_install(tmp_path)
    specs['tools.pointcloud_preprocessing.flat_floor'] = SimpleNamespace(
        origin=str(tmp_path/'other_tools/flat_floor.py'))
    with pytest.raises(ValueError, match='ground bridge dependency'):
        contract.deployment_evidence(tmp_path, prefix_lookup=prefixes.__getitem__, spec_lookup=specs.get)


@pytest.mark.parametrize('relative', [
    'install/d1max_planning_interfaces/lib/libd1max_planning_interfaces__rosidl_typesupport_cpp.so',
    'install/scan_planner_msgs/local/lib/python3.10/dist-packages/scan_planner_msgs/scan_planner_msgs_s__rosidl_typesupport_c.cpython-310-test.so',
])
def test_missing_required_interface_binary_is_not_silently_omitted(tmp_path, relative):
    prefixes, specs = fake_install(tmp_path)
    (tmp_path/relative).unlink()
    with pytest.raises(ValueError, match='missing generated'):
        contract.deployment_evidence(tmp_path, prefix_lookup=prefixes.__getitem__, spec_lookup=specs.get)


def test_unrelated_libraries_and_tools_tests_are_not_deployment_inputs(tmp_path):
    prefixes, specs = fake_install(tmp_path)
    before = contract.deployment_evidence(tmp_path, prefix_lookup=prefixes.__getitem__, spec_lookup=specs.get)
    (tmp_path/'install/scan_planner_msgs/lib/libunrelated_ros.so').write_text('not a loaded interface')
    test = tmp_path/'tools/pointcloud_preprocessing/tests/test_example.py'
    test.parent.mkdir(); test.write_text('test-only change')
    after = contract.deployment_evidence(tmp_path, prefix_lookup=prefixes.__getitem__, spec_lookup=specs.get)
    assert before == after


def test_interface_symlink_install_can_resolve_into_this_workspaces_build(tmp_path):
    prefixes, specs = fake_install(tmp_path)
    package = 'd1max_planning_interfaces'
    installed = Path(specs[package].origin).parent
    built = tmp_path/'build'/package/'rosidl_generator_py'/package
    built.parent.mkdir(parents=True)
    installed.rename(built)
    installed.symlink_to(built, target_is_directory=True)
    value = contract.deployment_evidence(tmp_path, prefix_lookup=prefixes.__getitem__, spec_lookup=specs.get)
    assert any(record['path'].startswith(str(built)) for record in value['files'].values())
