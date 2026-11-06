import json
import os
from pathlib import Path
import pytest
import yaml
from d1max_pct_planner import paths
from d1max_pct_scan.single_floor_session import prepare,view_commands,runtime_lock,runtime_commands


def recorded_map():
    explicit=os.environ.get('D1MAX_TEST_MAP_DIRECTORY')
    if explicit:return Path(explicit).resolve()
    try:return paths.default_map_directory()
    except (ValueError,OSError):return None


@pytest.mark.skipif(recorded_map() is None or not recorded_map().is_dir(),reason='requires the release map package')
def test_real_artifact_preparation_is_odom_typed_no_preview_exclusion(tmp_path,monkeypatch):
    monkeypatch.setattr(paths,'template_root',lambda:Path(__file__).resolve().parents[3])
    def forbidden(*a,**kw): raise AssertionError('prepare attempted to launch a process')
    monkeypatch.setattr('subprocess.Popen',forbidden)
    session=prepare(tmp_path/'session',session_id='unit-preparation',map_directory=recorded_map())
    assert session['physical_acceptance'] is False
    assert 'preview_ray_exclusion' not in session
    entity=session['entity_self_filter']
    assert entity['mode']=='filter'
    assert entity['body_frame']=='d1max_loc_base_link'
    assert entity['pose_source']=='fixed_standing_cad'
    assert entity['joint_source']=='not_available_not_required'
    assert 'joint_state_topic' not in entity
    assert entity['measurement_padding_m']==.005
    from d1max_pct_scan.robot_self_geometry import RobotSelfModel
    from d1max_pct_scan.single_floor_session import sha
    model=RobotSelfModel.load(entity['model_manifest'])
    for asset in model.asset_paths:
        assert session['input_hashes'][str(asset.resolve())]==sha(asset)
    assert session['local_planning_frame']=='d1max_loc_odom'
    def params(name): return yaml.safe_load((tmp_path/'session'/name).read_text())['/**']['ros__parameters']
    assert params('scan.yaml')['fsm.reference_path_z_offset']==0.
    from d1max_pct_scan.reference_geometry_contract import NATIVE_START_DISTANCE_M
    assert params('scan.yaml')['fsm.reference_start_tolerance']==NATIVE_START_DISTANCE_M
    assert params('scan.yaml')['grid_map.box_length']==pytest.approx(1.03)
    assert params('scan.yaml')['grid_map.box_width']==pytest.approx(.40)
    assert params('scan.yaml')['grid_map.stair_subvoxel_collision'] is True
    assert 'planar_clearance_m' not in params('scan.yaml')
    assert params('scan.yaml')['fsm.max_replan_interval']==.05
    assert params('scan.yaml')['fsm.min_replan_interval']==0.
    assert params('scan.yaml')['optimization.lambda_planar_curvature']==.2
    assert params('scan.yaml')['optimization.preferred_turn_radius']==1.
    assert params('scan.yaml')['optimization.soft_clearance_distance']==.15
    assert params('scan.yaml')['optimization.soft_clearance_budget_ms']==30.
    assert params('scan.yaml')['fsm.failed_replan_cooldown']==.2
    assert params('scan.yaml')['grid_map.require_observed_free'] is False
    assert session['scan_collision_policy']=='official'
    assert session['execution_collision_policy']=='official'
    assert session['scan_planning_collision_policy']=='official'
    assert params('scan.yaml')['fsm.trajectory_geometry_policy']=='official_inflated_geometry'
    assert params('scan.yaml')['grid_map.integration_rate_hz']==10.
    assert session['local_planning_window']['integration_rate_hz']==10.
    assert session['local_planning_window']['visualization_rate_hz']==3.
    gravity=yaml.safe_load((tmp_path/'session/localization.yaml').read_text())['lio_localizer']['ros__parameters'][
            'global_relocalization.gravity.map_contract_json']
    record=json.loads(gravity)
    assert session['registration_map_gravity']['map_sha256']==record['map_sha256']
    assert session['registration_map_gravity']['physical_calibration_verified'] is False
    for item in record['evidence'].values():
        assert session['input_hashes'][str(Path(item['path']).resolve())]==item['sha256']
    assert session['local_state_contract']=='continuous_odom_v2'
    assert session['handoff_contract']=='local_executor_v1'
    assert session['execution_architecture']=='task_follower_v1'
    assert params('scan.yaml')['fsm.writer_handoff_enabled'] is False
    assert params('scan.yaml')['fsm.local_follower_enabled'] is True
    assert params('scan.yaml')['fsm.navigation_map_version_id']==params('reference.yaml')['map_version_id']
    assert params('scan.yaml')['fsm.planning_only'] is False
    # Presence in the shared template does not enable the geometry-only branch
    # in execution; the native purpose flag, not these distances, selects it.
    assert params('scan.yaml')['manager.preview_candidate_connector_m']==.15
    assert params('scan.yaml')['manager.preview_candidate_projection_window_m']==.5
    assert 'manager.max_yaw_rate' not in params('scan.yaml')
    assert 'manager.tangent_rate_diagnostic_limit' not in params('scan.yaml')
    assert session['local_pipeline_contract']=='local_executor_v1'
    assert params('tracker.yaml')['writer_handoff_enabled'] is False
    # Formal tracking reports versioned measured progress/holding. The old
    # unversioned Go2 preview Bool must not be another execution state owner.
    assert 'frozen_topic' not in params('tracker.yaml')
    node_source=(paths.template_root()/'src/d1max_trajectory_tracker/src/trajectory_tracker.cpp').read_text()
    assert 'frozen_pub_' not in node_source
    session_source=(paths.template_root()/'src/d1max_pct_scan/d1max_pct_scan/single_floor_session.py').read_text()
    assert "'planning/go2_execution_frozen':" not in session_source
    assert not (tmp_path/'session/safety.yaml').exists()
    assert params('reference.yaml')['geometry_owner']=='local_executor'
    assert params('bt.yaml')['execution_architecture']=='task_follower_v1'
    assert params('tracker.yaml')['local_state_enabled'] is True
    assert params('reference.yaml')['local_state_enabled'] is True
    assert params('reference.yaml')['map_evidence_max_age_s']==params('scan.yaml')['grid_map.cloud_pose_max_age']==session['perception_timeout_s']==.5
    assert params('bt_adapter.yaml')['local_state_enabled'] is True
    assert params('mock_sdk.yaml')['execution_mc_delay_bound_s']==.02
    assert session['execution_mode_policy']=='fixed_stair'
    assert session['execution_stair_runtime_parameter_source']=='general_runtime_reuse'
    assert params('reference.yaml')['execution_mode_policy']=='fixed_stair'
    assert params('scan.yaml')['fsm.execution_mode_policy']=='fixed_stair'
    assert params('mock_sdk.yaml')['execution_mode_policy']=='fixed_stair'
    assert params('mock_sdk.yaml')['execution_stair_runtime_parameter_source']=='general_runtime_reuse'
    assert params('mock_sdk.yaml')['local_state_enabled'] is True
    assert params('mock_sdk.yaml')['local_navigation_state_topic']=='/d1max/localization/navigation/local_state'
    assert params('tracker.yaml')['planning_frame']=='d1max_loc_odom'
    assert params('tracker.yaml')['max_speed']==.3
    assert params('tracker.yaml')['controller_frequency_hz']==session['controller_frequency_hz']==20.
    assert params('tracker.yaml')['diagnostics_frequency_hz']==5.
    assert params('tracker.yaml')['max_jerk']==.70
    assert params('scan.yaml')['fsm.continuation_max_jerk']==params('tracker.yaml')['max_jerk']
    assert params('tracker.yaml')['max_yaw_jerk']==4.
    assert params('tracker.yaml')['heading_threshold']==.8
    assert params('tracker.yaml')['velocity_damping_gain']==0.
    assert params('tracker.yaml')['yaw_damping_gain']==.4
    assert params('tracker.yaml')['kp_position']==.8
    assert params('tracker.yaml')['kp_yaw']==.9
    assert params('tracker.yaml')['max_acceleration']==.35
    assert params('tracker.yaml')['max_yaw_acceleration']==.8
    assert params('tracker.yaml')['lookahead']==1.2
    assert params('tracker.yaml')['min_lookahead_distance_m']==.40
    assert params('tracker.yaml')['max_lookahead_distance_m']==.80
    assert session['motion_model']=='holonomic'
    assert session['kinematic_profile']=='holonomic_xyyaw'
    assert params('tracker.yaml')['motion_model']=='holonomic'
    assert params('tracker.yaml')['align_then_forward'] is session['align_then_forward'] is True
    assert params('scan.yaml')['manager.align_then_forward'] is True
    assert params('tracker.yaml')['steering_error_scale_rad']==.02
    assert params('tracker.yaml')['holonomic_yaw_policy']==params('scan.yaml')['manager.body_heading_policy']=='path_tangent'
    assert params('scan.yaml')['fsm.kinematic_profile']=='holonomic_xyyaw'
    assert params('tracker.yaml')['execution_braking_model_sha256']==session['execution_braking_model_sha256']
    assert params('mock_sdk.yaml')['execution_task_writer_enabled'] is True
    assert params('mock_sdk.yaml')['execution_v3_enabled'] is False
    assert params('bt_adapter.yaml')['execution_mode']=='single_floor'
    assert params('reference.yaml')['planning_manifest']==session['planning_manifest']
    assert params('tracker.yaml')['support_topic']=='/d1max/live_planning/execution/support'
    assert params('bt.yaml')['preview_only'] is False
    assert params('bt.yaml')['execution_transport_mode']=='isolated_mock'
    assert session['initial_pose_transaction']=='bt_initial_pose_transaction_v1'
    assert params('bt.yaml')['dependency_health_required'] is True
    assert params('component_health.yaml')['startup_timeout_s']==session['component_startup_timeout_s']
    assert session['component_startup_timeout_s']>=params('bt.yaml')['worker_startup_timeout_s']
    from d1max_pct_scan.navigation_runtime_policy import validate_session_policy
    validate_session_policy(tmp_path/'session',session)
    assert session['runtime_policy']['recovery_timeout_s']==30.
    assert params('bt.yaml')['follow_reference_wait_s']==params('bt_adapter.yaml')['follow_reference_wait_s']==30.
    assert params('bt.yaml')['dependency_heartbeat_timeout_s']==1.5
    assert params('component_health.yaml')['callback_timeout_s']==1.5
    assert params('global.yaml')['warmup_timeout_s']==params('bt.yaml')['worker_startup_timeout_s']==60.
    assert params('global.yaml')['readiness_timeout_s']==params('global.yaml')['commit_wait_timeout_s']==30.
    assert params('bt.yaml')['compute_readiness_timeout_s']==params('bt_adapter.yaml')['readiness_timeout_s']==30.
    assert params('reference.yaml')['async_support_preparation'] is True
    assert session['execution_timing']['required_reaction_bound_s']==pytest.approx(.77)
    assert params('tracker.yaml')['command_pipeline_bound_s']==session['execution_timing']['command_pipeline_bound_s']==.2
    assert params('tracker.yaml')['writer_period_s']==session['execution_timing']['writer_period_s']==.05
    from d1max_pct_scan.single_floor_session import validate_tracking_parameters
    validate_tracking_parameters(tmp_path/'session',session)
    tracker_file=tmp_path/'session/tracker.yaml'
    tracker_text=tracker_file.read_text()
    changed=yaml.safe_load(tracker_text)
    changed['/**']['ros__parameters']['controller_frequency_hz']=50.
    tracker_file.write_text(yaml.safe_dump(changed))
    with pytest.raises(ValueError,match='inconsistent_tracking_controller_parameters'):
        validate_tracking_parameters(tmp_path/'session',session)
    tracker_file.write_text(tracker_text)
    changed=yaml.safe_load(tracker_text)
    changed['/**']['ros__parameters']['writer_period_s']=.10
    tracker_file.write_text(yaml.safe_dump(changed))
    with pytest.raises(ValueError,match='inconsistent_tracking_controller_parameters'):
        validate_tracking_parameters(tmp_path/'session',session)
    tracker_file.write_text(tracker_text)
    # All controllers and native footprint checks consume the same heading
    # policy; changing only one side must not produce a runnable new package.
    scan_file=tmp_path/'session/scan.yaml';scan_text=scan_file.read_text()
    changed=yaml.safe_load(scan_text)
    changed['/**']['ros__parameters']['manager.align_then_forward']=False
    scan_file.write_text(yaml.safe_dump(changed))
    with pytest.raises(ValueError,match='inconsistent_native_tracking_heading_contract'):
        validate_tracking_parameters(tmp_path/'session',session)
    scan_file.write_text(scan_text)
    changed=yaml.safe_load(scan_text)
    changed['/**']['ros__parameters']['fsm.continuation_max_jerk']=1.4
    scan_file.write_text(yaml.safe_dump(changed))
    with pytest.raises(ValueError,match='inconsistent_continuation_scheduling_profile'):
        validate_tracking_parameters(tmp_path/'session',session)
    scan_file.write_text(scan_text)
    for parameter in ('velocity_damping_gain','yaw_damping_gain','max_jerk','max_yaw_jerk',
                      'steering_enter_angle_rad','steering_exit_angle_rad','steering_error_scale_rad'):
        changed=yaml.safe_load(tracker_text)
        changed['/**']['ros__parameters'][parameter]+= .031
        tracker_file.write_text(yaml.safe_dump(changed))
        with pytest.raises(ValueError,match='inconsistent_tracking_controller_parameters'):
            validate_tracking_parameters(tmp_path/'session',session)
    tracker_file.write_text(tracker_text)
    assert params('bt.yaml')['stationary_linear_threshold_mps']==params('tracker.yaml')['stationary_linear_threshold_mps']==.03
    assert params('bt.yaml')['stationary_velocity_metric']==params('tracker.yaml')['stationary_velocity_metric']==session['stationary_evidence'].get('velocity_metric','spatial_norm_3d')
    assert params('bt.yaml')['stationary_minimum_samples']==params('tracker.yaml')['stationary_minimum_samples']==3
    assert session['perception_projector']['max_input_points']==100000
    from d1max_pct_scan.single_floor_session import sha
    for relative in ('src/d1max_navigation_bt/trees/navigate_with_global_relocalization.xml',
                     'src/d1max_pct_scan/config/navigation_runtime.yaml',
                     'src/d1max_pct_scan/config/motion_tracking.yaml',
                     'src/d1max_localization/config/global_relocalization.yaml',
                     'src/d1max_navigation/rviz/localization_test.rviz'):
        template=paths.template_root()/relative
        assert session['input_hashes'][str(template.resolve())]==sha(template)
    import xml.etree.ElementTree as ET
    source=ET.parse(session['behavior_tree_xml']).getroot()
    effective=ET.parse(tmp_path/'session/navigation.xml').getroot()
    assert [n.tag for n in source.iter()]==[n.tag for n in effective.iter()]
    assert session['input_hashes'][str(Path(session['behavior_tree_xml']).resolve())]==sha(session['behavior_tree_xml'])
    assert 'WaitForInitialLocalization' in (tmp_path/'session/navigation.xml').read_text()
    localization=yaml.safe_load((tmp_path/'session/localization.yaml').read_text())['lio_localizer']['ros__parameters']
    assert localization['global_relocalization.enabled'] is True
    assert localization['global_relocalization.registration.exhaustive_tiles'] is True
    assert localization['global_relocalization.registration.search_timeout_s']==25.
    for layout in ('global','local'):
        cfg=yaml.safe_load((tmp_path/'session'/(layout+'_planning.rviz')).read_text())
        terrain=[display for group in cfg['Visualization Manager']['Displays']
                for display in group.get('Displays',[]) if display.get('Name')=='可通行地面']
        assert len(terrain)==1
        assert terrain[0]['Size (m)']==session['terrain_resolution_m']
        assert terrain[0]['Style']=='Flat Squares'
        assert cfg['Panels'][0]['Motion Capable'] is True
        assert cfg['Visualization Manager']['Global Options']['Fixed Frame']==(
            'd1max_loc_odom' if layout=='local' else 'd1max_loc_map')
        if layout=='local':
            assert not any(tool['Class'] in ('rviz_default_plugins/SetInitialPose','rviz_default_plugins/SetGoal',
                'rviz_default_plugins/PublishPoint','d1max_pct_rviz_tools/LiveGoal3D')
                for tool in cfg['Visualization Manager']['Tools'])
        assert not any('MotionControlPanel' in panel['Class'] for panel in cfg['Panels'])
        routes=[display for group in cfg['Visualization Manager']['Displays']
                for display in group.get('Displays',[]) if display.get('Name')=='全局路径']
        assert len(routes)==1
        assert routes[0]['Topic']['Value']=='/d1max/live_planning/committed_route_visual'
        assert routes[0]['Topic']['Durability Policy']=='Transient Local'
    with pytest.raises(ValueError,match='overwrite'):prepare(tmp_path/'session')


def test_live_cannot_be_prepared_from_mock_flags(tmp_path):
    with pytest.raises(ValueError,match='live_requires'):
        prepare(tmp_path/'live',transport_mode='live')


def live_localization_config():
    root=paths.nav_root()/'src/d1max_localization/config'
    config=yaml.safe_load((root/'localization.yaml').read_text())
    config['navigation_estimation']['ros__parameters'].update(
        yaml.safe_load((root/'realtime_navigation.yaml').read_text())['navigation_estimation']['ros__parameters'])
    return config


def test_live_localization_preflight_uses_the_actual_launch_parameter_resolver():
    from d1max_pct_scan.single_floor_session import validate_live_localization_parameters
    config=live_localization_config()
    validate_live_localization_parameters(config)
    config['ekf_navigation']['ros__parameters']['publish_tf']=True
    with pytest.raises(ValueError,match='EKF must be private'):
        validate_live_localization_parameters(config)


@pytest.mark.parametrize('enabled',[False,None,0,1,'true'])
def test_live_localization_preflight_rejects_missing_or_non_bool_enabled(enabled):
    from d1max_pct_scan.single_floor_session import validate_live_localization_parameters
    config=live_localization_config()
    config['navigation_estimation']['ros__parameters']['enabled']=enabled
    with pytest.raises(ValueError,match='requires_enabled_continuous_navigation_output'):
        validate_live_localization_parameters(config)


def test_live_localization_preflight_rejects_disabled_legacy_and_incompatible_prediction():
    from d1max_pct_scan.single_floor_session import validate_live_localization_parameters
    config=live_localization_config()
    config['localization_pipeline']['ros__parameters']['backend']='legacy_ekf'
    with pytest.raises(ValueError,match='requires_atomic_lio_navigation_output'):
        validate_live_localization_parameters(config)
    config['localization_pipeline']['ros__parameters']['backend']='lio_pcd'
    config['navigation_estimation']['ros__parameters']['prediction.max_horizon']=.4
    with pytest.raises(ValueError,match='admission must cover'):
        validate_live_localization_parameters(config)


def test_live_localization_source_only_copy_cannot_masquerade_as_selected_install(tmp_path,monkeypatch):
    from types import SimpleNamespace
    from d1max_pct_scan import single_floor_session as entry
    prefix=tmp_path/'installed/d1max_localization';prefix.mkdir(parents=True)
    source=tmp_path/'checkout/d1max_localization';source.mkdir(parents=True)
    def origin(module):
        filename=module.rsplit('.',1)[-1]+'.py'
        installed=prefix/filename;installed.write_text('# byte-identical\n')
        checkout=source/filename;checkout.write_bytes(installed.read_bytes())
        return SimpleNamespace(origin=str(checkout))
    monkeypatch.setattr(entry.importlib.util,'find_spec',origin)
    with pytest.raises(ValueError,match='python_outside_selected_install'):
        entry.validate_live_localization_parameters(live_localization_config(),package_prefix=prefix)


def test_live_localization_preflight_accepts_consistent_selected_install_without_ros(tmp_path,monkeypatch):
    from types import SimpleNamespace
    from d1max_pct_scan import single_floor_session as entry
    prefix=tmp_path/'installed/d1max_localization';prefix.mkdir(parents=True)
    modules=[]
    def origin(module):
        modules.append(module)
        target=prefix/(module.rsplit('.',1)[-1]+'.py');target.write_text('# installed boundary\n')
        return SimpleNamespace(origin=str(target))
    monkeypatch.setattr(entry.importlib.util,'find_spec',origin)
    entry.validate_live_localization_parameters(live_localization_config(),package_prefix=prefix)
    assert modules==['d1max_localization.'+name for name in
        ('global_lio_localizer','realtime_navigation_output','navigation_output','navigation_wire')]


@pytest.mark.skipif(recorded_map() is None or not recorded_map().is_dir(),reason='requires the release map package')
def test_live_planning_only_retains_real_geometry_without_motion_pipeline(tmp_path,monkeypatch):
    monkeypatch.setattr(paths,'template_root',lambda:Path(__file__).resolve().parents[3])
    from d1max_pct_scan.single_floor_session import validate_planning_only_parameters
    def forbidden(*a,**kw):raise AssertionError('planning preparation launched a process or requested acceptance')
    monkeypatch.setattr('subprocess.Popen',forbidden)
    monkeypatch.setattr('subprocess.run',forbidden)
    monkeypatch.setattr('d1max_pct_scan.single_floor_session.check_acceptance',forbidden)
    directory=tmp_path/'planning'
    session=prepare(directory,session_id='planning-test',transport_mode='live',map_directory=recorded_map(),
                    expected_sdk_session='already-running-read-only',purpose='planning_only')
    assert session['purpose']==session['execution_purpose']=='planning_only'
    assert session['motion_control_enabled'] is False
    assert session['mode']=='SINGLE_FLOOR_PLANNING_ONLY'
    assert not session['physical_acceptance_record']
    assert not session['execution_braking_model_record']
    assert not session['execution_braking_model_sha256']
    assert session['body_height_calibration_status']=='engineering_nominal_not_physically_verified'
    for name in ('braking_model.json','safety.yaml','mock_sdk.yaml'):
        assert not (directory/name).exists()
    def params(name):return yaml.safe_load((directory/name).read_text())['/**']['ros__parameters']
    assert params('bt.yaml')['execution_purpose']=='planning_only'
    assert params('bt.yaml')['preview_only'] is True
    assert params('tracker.yaml')['demand_topic']=='/d1max/live_planning/preview/demand'
    assert params('scan.yaml')['fsm.execution_protocol'] is True
    assert params('scan.yaml')['grid_map.box_length']==pytest.approx(1.03)
    assert params('scan.yaml')['grid_map.box_width']==pytest.approx(.40)
    assert 'planar_clearance_m' not in params('scan.yaml')
    assert session['scan_collision_policy']=='official'
    assert session['scan_planning_collision_policy']=='official'
    assert session['execution_collision_policy']=='official'
    assert params('scan.yaml')['grid_map.require_observed_free'] is False
    assert params('scan.yaml')['grid_map.planning_collision_policy']=='official'
    assert params('scan.yaml')['fsm.trajectory_geometry_policy']=='official_inflated_geometry'
    assert params('scan.yaml')['fsm.planning_only'] is False
    assert params('scan.yaml')['manager.preview_candidate_connector_m']==.15
    assert params('scan.yaml')['manager.preview_candidate_projection_window_m']==.5
    assert 'manager.max_yaw_rate' not in params('scan.yaml')
    assert 'manager.tangent_rate_diagnostic_limit' not in params('scan.yaml')
    assert session['local_pipeline_contract']=='local_executor_v1'
    assert session['local_planning_window']==dict(size_m=[12.,12.,3.],resolution_m=.05,reference_horizon_m=4.,
        integration_rate_hz=10.,visualization_rate_hz=3.)
    assert params('reference.yaml')['reference_horizon_m']==params('scan.yaml')['fsm.planning_horizon']==params('scan.yaml')['manager.planning_horizon']==4.
    assert [params('scan.yaml')['grid_map.sliding_map_size_'+axis] for axis in ('x','y','z')]==[12.,12.,3.]
    assert [params('scan.yaml')['grid_map.local_update_range_'+axis] for axis in ('x','y','z')]==[6.,6.,1.5]
    assert params('scan.yaml')['grid_map.use_projected_rays'] is True
    assert params('scan.yaml')['grid_map.preview_only'] is False
    assert params('scan.yaml')['grid_map.cloud_pose_max_age']==.5
    validate_planning_only_parameters(directory,session)
    monkeypatch.setattr('ament_index_python.packages.get_package_prefix',lambda name:'/isolated/'+name)
    commands=runtime_commands(directory,session)
    assert {'localization','global','reference','scan','navigator'}<=set(commands)
    assert 'tracker' in commands
    assert 'local_follower' in commands['tracker'][0]
    assert not {'mock_sdk','sdk','safety'}&set(commands)
    assert not any(Path(cmd[0]).name=='trajectory_tracker' for cmd in commands.values())
    assert not any('sdk_execution' in str(cmd) or 'execution_safety_node' in str(cmd) for cmd in commands.values())
    for layout in ('global','local'):
        cfg=yaml.safe_load((directory/(layout+'_planning.rviz')).read_text())
        assert cfg['Panels'][0]['Motion Capable'] is False
    for name,key,value in (('bt.yaml','preview_only',False),
                           ('bt.yaml','execution_purpose','execution'),
                           ('scan.yaml','fsm.writer_handoff_enabled',True),
                           ('component_health.yaml','motion_pipeline',True),
                           ('tracker.yaml','demand_topic','/d1max/live_planning/execution/demand')):
        original=(directory/name).read_text()
        content=yaml.safe_load(original);content['/**']['ros__parameters'][key]=value
        (directory/name).write_text(yaml.safe_dump(content))
        with pytest.raises(ValueError,match='capability_mismatch'):
            validate_planning_only_parameters(directory,session)
        (directory/name).write_text(original)
    for value in (True,None,0,1,'false'):
        original=(directory/'scan.yaml').read_text()
        content=yaml.safe_load(original);content['/**']['ros__parameters']['grid_map.require_observed_free']=value
        (directory/'scan.yaml').write_text(yaml.safe_dump(content))
        with pytest.raises(ValueError,match='inconsistent_planning_geometry_or_motion_evidence_policy'):
            validate_planning_only_parameters(directory,session)
        (directory/'scan.yaml').write_text(original)
    for name in ('safety.yaml','mock_sdk.yaml'):
        (directory/name).write_text('{}')
        with pytest.raises(ValueError,match='forbids_motion_pipeline'):
            validate_planning_only_parameters(directory,session)
        (directory/name).unlink()


@pytest.mark.skipif(recorded_map() is None or not recorded_map().is_dir(),reason='requires the release map package')
def test_live_execution_uses_default_runtime_parameters_without_calibration_report(tmp_path,monkeypatch):
    from d1max_pct_scan import single_floor_session as entry
    root=Path(__file__).resolve().parents[3]
    monkeypatch.setattr(paths,'template_root',lambda:root)
    def forbidden(*args,**kwargs):raise AssertionError('prepare must not start robot/ROS processes')
    monkeypatch.setattr('subprocess.Popen',forbidden)
    checked=[]
    def parameters(record):
        checked.append(Path(record))
        content=json.loads(Path(record).read_text())
        assert content['record_kind']=='runtime_motion_profile'
        assert content['physical_acceptance_verified'] is False
        assert 'records' not in content and 'evidence_id' not in content
        return dict(valid=True,physically_verified=False,kinematic_profile=content['kinematic_profile'],
                    max_speed_mps=content['measurements']['max_speed_mps'],max_yaw_radps=content['measurements']['max_yaw_radps'])
    monkeypatch.setattr(entry,'check_acceptance',parameters)
    record=root/'src/d1max_pct_scan/config/runtime_motion_profile.json'
    robot_profile=root/'src/d1max_scan_planner/config/d1max_robot.yaml'
    self_filter=root/'src/d1max_pct_scan/config/entity_self_filter.yaml'
    original_hashes={path:entry.sha(path) for path in (record,robot_profile,self_filter)}
    directory=tmp_path/'runtime'
    session=prepare(directory,session_id='runtime-default',transport_mode='live',
        expected_sdk_session='actual-existing-sdk',map_directory=recorded_map())
    assert checked==[record]
    assert session['physical_acceptance_record']==''
    assert session['physical_acceptance'] is False
    assert session['runtime_motion_profile']==str(record)
    assert session['motion_control_enabled'] is True
    assert session['motion_parameter_basis']=='vendor_nominal_engineering_and_operator_observation'
    assert session['body_height_calibration_status']=='engineering_nominal_not_physically_verified'
    assert session['body_height_calibration_id'].startswith('runtime_profile_sha256:')
    assert session['execution_braking_model_sha256']==entry.sha(record)
    def params(name):return yaml.safe_load((directory/name).read_text())['/**']['ros__parameters']
    assert params('bt.yaml')['execution_record_sha256']==entry.sha(record)
    assert params('scan.yaml')['fsm.execution_braking_model_sha256']==entry.sha(record)
    assert params('tracker.yaml')['execution_braking_model_sha256']==entry.sha(record)
    # Session-only planning clearance changes the shared native XY kernel,
    # not the robot/self model or SDK writer's already-bound profile identity.
    scan=params('scan.yaml')
    assert scan['grid_map.box_length']==pytest.approx(1.03)
    assert scan['grid_map.box_width']==pytest.approx(.40)
    assert session['scan_planar_clearance_m']==.05
    assert 'planar_clearance_m' not in scan
    assert session['scan_stair_planar_clearance_m']==.025
    assert 'stair_planar_clearance_m' not in scan
    assert scan['grid_map.box_nominal_length']==.93
    assert scan['grid_map.box_nominal_width']==.30
    assert scan['grid_map.stair_planar_clearance']==.025
    for side in ('below','above'):
        assert scan['grid_map.body_'+side]==scan['grid_map.planning_body_'+side]==.25
    assert scan['grid_map.body_height']==.50
    assert scan['optimization.dist0']==.28
    assert scan['optimization.soft_clearance_distance']==.15
    assert scan['optimization.soft_clearance_budget_ms']==30.
    assert {path:entry.sha(path) for path in original_hashes}==original_hashes
    assert json.loads(record.read_text())['robot_profile_sha256']==original_hashes[robot_profile]
    assert session['body_height_calibration_id']=='runtime_profile_sha256:'+original_hashes[record]
    assert session['input_hashes'][str(robot_profile.resolve())]==original_hashes[robot_profile]
    timing=json.loads(record.read_text())['execution_timing']
    assert session['perception_timeout_s']==timing['sensor_source_age_bound_s']==.6
    assert params('scan.yaml')['grid_map.cloud_pose_max_age']==session['perception_timeout_s']
    assert params('reference.yaml')['map_evidence_max_age_s']==session['perception_timeout_s']
    # Construct the actual receiving component with generated parameters too:
    # matching YAML values alone cannot catch a stale constructor ceiling.
    from d1max_pct_scan.continuous_reference_node import ReferenceCallbacks
    reference=ReferenceCallbacks(params('reference.yaml'),lambda *_:None,lambda:0,lambda:0.)
    assert reference.map_evidence_age_ns==600_000_000
    assert reference.close()
    assert params('scan.yaml')['grid_map.preview_only'] is False
    for name in ('bt.yaml','tracker.yaml'):
        assert params(name)['command_pipeline_bound_s']==timing['command_pipeline_bound_s']==.1
        assert params(name)['stationary_velocity_metric']=='planar_xy_with_vertical_guard_v1'
    assert params('tracker.yaml')['writer_period_s']==timing['writer_period_s']==.05
    assert params('tracker.yaml')['command_response_enabled'] is True
    assert params('tracker.yaml')['forward_response_reference']==.3
    assert params('tracker.yaml')['yaw_response_reference']==.5
    assert params('tracker.yaml')['max_input_rate']==1.6
    assert params('tracker.yaml')['max_input_acceleration']==12.
    # The controller and the sole SDK writer describe the exact same static
    # response; changing a generated coefficient alone is not a new profile.
    from d1max_pct_scan.single_floor_session import validate_tracking_parameters
    validate_tracking_parameters(directory,session)
    tracker_path=directory/'tracker.yaml';tracker_original=tracker_path.read_text()
    changed=yaml.safe_load(tracker_original)
    changed['/**']['ros__parameters']['yaw_response_effective_fraction']=.19
    tracker_path.write_text(yaml.safe_dump(changed))
    with pytest.raises(ValueError,match='inconsistent_tracking_controller_parameters'):
        validate_tracking_parameters(directory,session)
    tracker_path.write_text(tracker_original)
    assert not (directory/'safety.yaml').exists()
    assert params('bt.yaml')['preview_only'] is False
    assert params('scan.yaml')['fsm.planning_only'] is False
    assert params('scan.yaml')['grid_map.require_observed_free'] is False
    assert params('scan.yaml')['grid_map.planning_collision_policy']=='official'


@pytest.mark.parametrize('purpose',['planning_only','execution'])
def test_scan_collision_policy_is_shared_across_purposes_without_launching(tmp_path,purpose):
    from d1max_pct_scan.single_floor_session import validate_collision_policy_parameters
    motion=purpose=='execution'
    session=dict(purpose=purpose,execution_purpose=purpose,transport_mode='live',
        motion_control_enabled=motion,mode='SINGLE_FLOOR_PLANNING_ONLY' if not motion else 'SINGLE_FLOOR_EXECUTION',
        physical_acceptance_record='',execution_braking_model_record='',execution_braking_model_sha256='',
        scan_collision_policy='official',scan_planning_collision_policy='official',
        trajectory_geometry_policy='official_inflated_geometry',execution_collision_policy='official')
    scan={'grid_map.require_observed_free':False,'grid_map.planning_collision_policy':'official',
        'fsm.trajectory_geometry_policy':'official_inflated_geometry','fsm.planning_only':False}
    tracker={'trajectory_geometry_policy':'official_inflated_geometry'}
    def save(name,value):(tmp_path/name).write_text(yaml.safe_dump({'/**':{'ros__parameters':value}}))
    save('scan.yaml',scan)
    if motion:save('tracker.yaml',tracker)
    validate_collision_policy_parameters(tmp_path,session)
    for value in (True,None,0,1,'observed_free'):
        save('scan.yaml',dict(scan,**{'grid_map.require_observed_free':value}))
        with pytest.raises(ValueError,match='inconsistent_planning_geometry_or_motion_evidence_policy'):
            validate_collision_policy_parameters(tmp_path,session)
    save('scan.yaml',scan)
    for field,value in (('scan_collision_policy','observed_free'),
                        ('scan_planning_collision_policy','observed_free'),
                        ('trajectory_geometry_policy','observed_free'),
                        ('execution_collision_policy','observed_free')):
        with pytest.raises(ValueError,match='inconsistent_planning_geometry_or_motion_evidence_policy'):
            validate_collision_policy_parameters(tmp_path,dict(session,**{field:value}))
    if motion:
        save('tracker.yaml',{'trajectory_geometry_policy':'observed_free'})
        with pytest.raises(ValueError,match='inconsistent_planning_geometry_or_motion_evidence_policy'):
            validate_collision_policy_parameters(tmp_path,session)


@pytest.mark.parametrize('kwargs',[
    dict(purpose='planning_only',transport_mode='isolated_mock'),
    dict(purpose='unknown',transport_mode='live'),
    dict(purpose='planning_only',transport_mode='live',acceptance_record='/not-a-preview-record'),
])
def test_planning_only_rejects_mixed_or_motion_capabilities(tmp_path,kwargs):
    with pytest.raises(ValueError,match='planning_only'):
        prepare(tmp_path/'invalid',**kwargs)


def test_session_purpose_cannot_be_upgraded_by_one_alias():
    from d1max_pct_scan.single_floor_session import planning_only_profile
    with pytest.raises(ValueError,match='purpose_mismatch'):
        planning_only_profile(dict(purpose='planning_only',execution_purpose='execution'))


def test_ui_is_separate_from_task_and_control_graph(tmp_path):
    commands=view_commands(tmp_path,'local')
    assert set(commands)=={'view','map_layers','execution_view','rviz'}
    assert 'global_planning.rviz' in str(commands['rviz'])
    assert commands['rviz']==view_commands(tmp_path,'global')['rviz']
    assert not any('motion_coordinator' in str(v) or 'live_scan_bridge' in str(v) for v in commands.values())


def test_global_route_display_cache_is_core_owned_not_destroyed_by_closing_ui(tmp_path,monkeypatch):
    monkeypatch.setattr('ament_index_python.packages.get_package_prefix',lambda name:'/isolated/'+name)
    runtime=runtime_commands(tmp_path,{'transport_mode':'isolated_mock'})
    assert 'route_display_cache' in runtime
    assert 'd1max_pct_scan.committed_route_view' in runtime['route_display_cache']
    for layout in ('global','local'):
        assert 'route_display_cache' not in view_commands(tmp_path,layout)
        assert not any('committed_route_view' in str(command) for command in view_commands(tmp_path,layout).values())


def test_optional_display_fault_never_takes_ownership_of_navigation_stop():
    from d1max_pct_scan.single_floor_session import component_is_critical
    assert not component_is_critical('route_display_cache')
    assert all(component_is_critical(name) for name in
        ('navigator','adapters','perception','reference','tracker','safety','mock_sdk','scan','lifecycle'))


def test_legacy_rviz_configuration_still_uses_worker_live_path():
    from d1max_pct_scan.live_visualization import configure
    base=yaml.safe_load((paths.nav_root()/'src/d1max_navigation/rviz/localization_test.rviz').read_text())
    config=configure(base,session_id='legacy')
    routes=[display for group in config['Visualization Manager']['Displays']
            for display in group.get('Displays',[]) if display.get('Name')=='全局路径']
    assert routes[0]['Topic']['Value']=='/d1max/live_planning/global_path_visual'


def test_duplicate_supervisor_is_rejected_before_any_cleanup(tmp_path,monkeypatch):
    monkeypatch.setattr('d1max_pct_scan.single_floor_session._core_lock_root',lambda:tmp_path)
    with runtime_lock(tmp_path):
        with pytest.raises(ValueError,match='already_running'):
            with runtime_lock(tmp_path):
                pytest.fail('duplicate supervisor acquired the session')
    with runtime_lock(tmp_path):
        pass


def test_two_session_directories_cannot_own_the_same_robot_domain(tmp_path,monkeypatch):
    from d1max_pct_scan import single_floor_session as entry
    monkeypatch.setattr(entry,'_core_lock_root',lambda:tmp_path)
    monkeypatch.setenv('ROS_DOMAIN_ID','142')
    first=tmp_path/'first';second=tmp_path/'second';first.mkdir();second.mkdir()
    with runtime_lock(first) as fd:
        assert fd>=0
        with pytest.raises(ValueError,match='navigation_core_already_running'):
            with runtime_lock(second):pytest.fail('second directory bypassed core ownership')
        monkeypatch.setenv('ROS_DOMAIN_ID','143')
        with runtime_lock(second):pass
    monkeypatch.setenv('ROS_DOMAIN_ID','142')
    with runtime_lock(second):pass


def test_two_layouts_share_one_window_lock_and_display_lease(tmp_path,monkeypatch):
    from d1max_pct_scan.single_floor_session import view_layout_lock,_try_display_lease
    monkeypatch.setattr('d1max_pct_scan.single_floor_session._core_lock_root',lambda:tmp_path)
    with view_layout_lock(tmp_path,'global'):
        for layout in ('global','local'):
            with pytest.raises(ValueError,match='navigation_view_already_running'):
                with view_layout_lock(tmp_path,layout):pytest.fail('duplicate navigation window')
        first=_try_display_lease(tmp_path)
        assert first is not None and first[1]>=0
        assert _try_display_lease(tmp_path) is None
        first[0].__exit__(None,None,None)
        replacement=_try_display_lease(tmp_path)
        assert replacement is not None
        replacement[0].__exit__(None,None,None)
    for layout in ('global','local'):
        command=view_commands(tmp_path,layout,include_publishers=False)
        assert set(command)=={'rviz'}
        assert command['rviz'][-1]=='__node:=d1max_navigation_rviz'


def test_view_lock_is_not_bypassed_by_a_different_session_folder(tmp_path,monkeypatch):
    from d1max_pct_scan.single_floor_session import view_layout_lock
    monkeypatch.setattr('d1max_pct_scan.single_floor_session._core_lock_root',lambda:tmp_path)
    first=tmp_path/'first';second=tmp_path/'second';first.mkdir();second.mkdir()
    with view_layout_lock(first,'local'):
        with pytest.raises(ValueError,match='navigation_view_already_running'):
            with view_layout_lock(second,'global'):pytest.fail('second session bypassed viewer lock')
    with view_layout_lock(second,'global'):
        pass


def test_existing_legacy_layout_lock_prevents_another_rviz(tmp_path,monkeypatch):
    from d1max_pct_scan.single_floor_session import view_layout_lock,_owned_file_lock
    monkeypatch.setattr('d1max_pct_scan.single_floor_session._core_lock_root',lambda:tmp_path)
    with _owned_file_lock(tmp_path/'view-local.lock','fixture'):
        with pytest.raises(ValueError,match='legacy_navigation_view_already_running'):
            with view_layout_lock(tmp_path,'global'):pytest.fail('legacy RViz was overlapped')


def test_core_lease_survives_supervisor_exit_until_inherited_child_retires(tmp_path,monkeypatch):
    """Use a pipe-only child, never ROS/SDK, to exercise kernel lease lifetime."""
    import subprocess
    import sys
    from d1max_pct_scan import single_floor_session as entry
    monkeypatch.setattr(entry,'_core_lock_root',lambda:tmp_path)
    monkeypatch.setenv('ROS_DOMAIN_ID','142')
    first=tmp_path/'first';second=tmp_path/'second';first.mkdir();second.mkdir()
    child=None
    try:
        with runtime_lock(first) as fd:
            child=subprocess.Popen([sys.executable,'-c',
                'import sys; print("ready",flush=True); sys.stdin.read()'],pass_fds=(fd,),
                stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
            assert child.stdout.readline().strip()=='ready'
        with pytest.raises(ValueError,match='navigation_core_already_running'):
            with runtime_lock(second):pytest.fail('live child lost the supervisor lease')
        child.communicate('',timeout=3.)
        with runtime_lock(second):pass
    finally:
        if child is not None and child.poll() is None:
            child.terminate();child.wait(timeout=3.)


def test_legacy_empty_private_directory_lock_is_hardened_without_replacing_inode(tmp_path):
    from d1max_pct_scan.single_floor_session import _owned_file_lock
    lock=tmp_path/'old.lock';lock.touch();lock.chmod(0o664)
    before=lock.stat().st_ino
    with _owned_file_lock(lock,'busy'):
        assert lock.stat().st_ino==before
        assert lock.stat().st_mode&0o777==0o600
        with pytest.raises(ValueError,match='busy'):
            with _owned_file_lock(lock,'busy'):pytest.fail('second lease admitted')
    assert lock.stat().st_ino==before


def test_lock_hardening_never_mutates_data_or_shared_directory(tmp_path):
    from d1max_pct_scan.single_floor_session import _owned_file_lock
    lock=tmp_path/'data.lock';lock.write_text('not an empty lease');lock.chmod(0o664)
    with pytest.raises(ValueError,match='navigation_lock_not_private'):
        with _owned_file_lock(lock,'busy'):pytest.fail('unexpected file hardened')
    assert lock.read_text()=='not an empty lease' and lock.stat().st_mode&0o777==0o664
    shared=tmp_path/'shared';shared.mkdir(mode=0o755)
    lock=shared/'old.lock';lock.touch();lock.chmod(0o664)
    with pytest.raises(ValueError,match='navigation_lock_not_private'):
        with _owned_file_lock(lock,'busy'):pytest.fail('shared directory lock admitted')
    assert lock.stat().st_mode&0o777==0o664


def test_pregraph_failure_reports_no_children_and_original_reason(tmp_path,monkeypatch):
    from d1max_pct_scan import single_floor_session as entry
    (tmp_path/'session.json').write_text(json.dumps({'id':'startup-test'}))
    def fail_before_lock(_):raise ValueError('navigation_core_already_running')
    monkeypatch.setattr(entry,'runtime_lock',fail_before_lock)
    monkeypatch.setattr(entry,'_run_locked',lambda *a,**k:pytest.fail('children started'))
    with pytest.raises(ValueError,match='navigation_core_already_running'):entry.run(tmp_path)
    report=json.loads((tmp_path/'core_startup.json').read_text())
    assert report['schema']==1 and report['session_id']=='startup-test'
    assert report['phase']=='failed' and report['children_started'] is False
    assert report['error']=='navigation_core_already_running'
    assert (tmp_path/'core_startup.json').stat().st_mode&0o777==0o600


def test_spawn_attempt_failure_is_not_reported_as_pregraph_failure(tmp_path,monkeypatch):
    from contextlib import contextmanager
    from d1max_pct_scan import single_floor_session as entry
    (tmp_path/'session.json').write_text(json.dumps({'id':'startup-test'}))
    @contextmanager
    def mock_lock(_):yield 7
    def fail_after_attempt(directory,**_):
        entry._startup_report(directory,'startup-test','spawning',True)
        raise OSError('child_start_failed')
    monkeypatch.setattr(entry,'runtime_lock',mock_lock)
    monkeypatch.setattr(entry,'_run_locked',fail_after_attempt)
    with pytest.raises(OSError,match='child_start_failed'):entry.run(tmp_path)
    report=json.loads((tmp_path/'core_startup.json').read_text())
    assert report['phase']=='failed' and report['children_started'] is True
