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
    def forbidden(*a,**kw): raise AssertionError('prepare attempted to launch a process')
    monkeypatch.setattr('subprocess.Popen',forbidden)
    session=prepare(tmp_path/'session',session_id='unit-preparation',map_directory=recorded_map())
    assert session['physical_acceptance'] is False
    assert 'preview_ray_exclusion' not in session
    assert session['local_planning_frame']=='d1max_loc_odom'
    def params(name): return yaml.safe_load((tmp_path/'session'/name).read_text())['/**']['ros__parameters']
    assert params('scan.yaml')['fsm.reference_path_z_offset']==0.
    assert params('scan.yaml')['grid_map.require_observed_free'] is True
    assert params('scan.yaml')['grid_map.integration_rate_hz']==5.
    assert session['local_state_contract']=='continuous_odom_v1'
    assert session['handoff_contract']=='writer_applied_cas_v3'
    assert params('scan.yaml')['fsm.writer_handoff_enabled'] is True
    assert params('tracker.yaml')['writer_handoff_enabled'] is True
    assert params('safety.yaml')['writer_handoff_enabled'] is True
    assert params('tracker.yaml')['local_state_enabled'] is True
    assert params('reference.yaml')['local_state_enabled'] is True
    assert params('reference.yaml')['map_evidence_max_age_s']==params('scan.yaml')['grid_map.cloud_pose_max_age']==session['perception_timeout_s']==.5
    assert params('bt_adapter.yaml')['local_state_enabled'] is True
    assert params('mock_sdk.yaml')['execution_mc_delay_bound_s']==.02
    assert params('mock_sdk.yaml')['local_state_enabled'] is True
    assert params('mock_sdk.yaml')['local_navigation_state_topic']=='/d1max/localization/navigation/local_state'
    assert params('tracker.yaml')['planning_frame']=='d1max_loc_odom'
    assert params('tracker.yaml')['max_speed']==.3
    assert params('tracker.yaml')['execution_braking_model_sha256']==session['execution_braking_model_sha256']
    assert params('safety.yaml')['max_speed']==.3
    assert params('safety.yaml')['max_yaw']==.5
    assert params('bt_adapter.yaml')['execution_mode']=='single_floor'
    assert params('reference.yaml')['planning_manifest']==session['planning_manifest']
    assert params('tracker.yaml')['support_topic']=='/d1max/live_planning/execution/support'
    assert params('bt.yaml')['preview_only'] is False
    assert params('bt.yaml')['execution_transport_mode']=='isolated_mock'
    assert session['initial_pose_transaction']=='bt_initial_pose_transaction_v1'
    assert params('bt.yaml')['dependency_health_required'] is True
    assert params('component_health.yaml')['startup_timeout_s']==session['component_startup_timeout_s']
    assert session['component_startup_timeout_s']>=params('bt.yaml')['worker_startup_timeout_s']
    assert params('reference.yaml')['async_support_preparation'] is True
    assert session['execution_timing']['required_reaction_bound_s']==pytest.approx(.67)
    assert params('bt.yaml')['stationary_linear_threshold_mps']==params('tracker.yaml')['stationary_linear_threshold_mps']==.03
    assert params('bt.yaml')['stationary_minimum_samples']==params('tracker.yaml')['stationary_minimum_samples']==3
    assert session['perception_projector']['max_input_points']==100000
    from d1max_pct_scan.single_floor_session import sha
    for relative in ('src/d1max_navigation_bt/trees/navigate_with_global_relocalization.xml',
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
    assert params('scan.yaml')['fsm.execution_protocol'] is True
    assert params('scan.yaml')['grid_map.require_observed_free'] is True
    assert params('tracker.yaml')['demand_topic']=='/d1max/live_planning/preview/demand_unrouted'
    validate_planning_only_parameters(directory,session)
    monkeypatch.setattr('ament_index_python.packages.get_package_prefix',lambda name:'/isolated/'+name)
    commands=runtime_commands(directory,session)
    assert {'localization','global','reference','scan','tracker','navigator'}<=set(commands)
    assert not {'mock_sdk','sdk','safety'}&set(commands)
    assert not any('sdk_execution' in str(cmd) or 'execution_safety_node' in str(cmd) for cmd in commands.values())
    for layout in ('global','local'):
        cfg=yaml.safe_load((directory/(layout+'_planning.rviz')).read_text())
        assert cfg['Panels'][0]['Motion Capable'] is False
    for name,key,value in (('bt.yaml','preview_only',False),
                           ('bt.yaml','execution_purpose','execution'),
                           ('tracker.yaml','demand_topic','/d1max/live_planning/execution/demand')):
        original=(directory/name).read_text()
        content=yaml.safe_load(original);content['/**']['ros__parameters'][key]=value
        (directory/name).write_text(yaml.safe_dump(content))
        with pytest.raises(ValueError,match='capability_mismatch'):
            validate_planning_only_parameters(directory,session)
        (directory/name).write_text(original)
    (directory/'safety.yaml').write_text('{}')
    with pytest.raises(ValueError,match='forbids_motion_pipeline'):
        validate_planning_only_parameters(directory,session)


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
