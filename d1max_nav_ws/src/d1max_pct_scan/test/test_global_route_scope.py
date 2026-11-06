"""Map geometry does not replace the existing graph; no ROS/SDK startup."""
from copy import deepcopy
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml

from d1max_pct_planner import paths
from d1max_pct_scan import single_floor_session as session_entry
from d1max_pct_scan.navigation_contract import mainline_coordinate_contract

ROOT=Path(__file__).resolve().parents[3]
MAP=ROOT/'maps/processed/sc_pgo_20260923_crossfloor_complete_001'


def global_session():
    return dict(purpose='planning_only',execution_purpose='planning_only',
        planning_scope='global_route',mode='NAVIGATION_GLOBAL_ROUTE_PLANNING_ONLY',
        physical_acceptance=False,transport_mode='live',motion_control_enabled=False,
        map_pcd='/passive/original.pcd',localization_config='/passive/localization.yaml',
        physical_acceptance_record='',execution_braking_model_record='',
        execution_braking_model_sha256='',map_capabilities=dict(
            geometry_kind='conditioned_multifloor_ground',execution_geometry_eligible=False))


def test_global_only_runtime_uses_same_bt_worker_and_never_opens_local_or_sdk_executables(tmp_path,monkeypatch):
    looked_up=[]
    def prefix(name):
        looked_up.append(name)
        return '/isolated-install/'+name
    monkeypatch.setattr('ament_index_python.packages.get_package_prefix',prefix)
    commands=session_entry.runtime_commands(tmp_path,global_session())
    assert set(commands)=={'localization','global','adapters','navigator','component_health','lifecycle','route_display_cache'}
    assert 'd1max_pct_scan.live_global_planner' in commands['global']
    assert 'd1max_pct_scan.bt_adapters' in commands['adapters']
    assert 'd1max_navigation_bt' in commands['navigator'][0]
    assert set(looked_up)=={'d1max_navigation_bt','nav2_lifecycle_manager'}
    assert not any('sdk' in str(cmd) or 'crossfloor_global_session' in str(cmd) for cmd in commands.values())


@pytest.mark.parametrize('field,value',[
    ('purpose','execution'),('transport_mode','isolated_mock'),('motion_control_enabled',True),
    ('physical_acceptance',True),('mode','SINGLE_FLOOR_PLANNING_ONLY'),
])
def test_global_scope_cannot_promote_planning_result_to_motion_session(tmp_path,field,value):
    value_session=global_session()
    value_session[field]=value
    with pytest.raises(ValueError):session_entry.runtime_commands(tmp_path,value_session)


@pytest.mark.parametrize('default,explicit,expected',[
    ('planning_only',None,'planning_only'),('planning_only','execution','execution'),
    ('execution',None,'execution'),
])
def test_cli_uses_sealed_release_purpose_but_preserves_explicit_user_choice(
        tmp_path,monkeypatch,default,explicit,expected):
    import sys
    calls=[]
    monkeypatch.setattr(paths,'release_root',lambda:tmp_path)
    monkeypatch.setattr(paths,'release_descriptor',lambda release:(release,dict(default_session_purpose=default)))
    monkeypatch.setattr(session_entry,'prepare',lambda target,**options:calls.append(options) or dict(id='passive'))
    arguments=['navigation_session','prepare','--session',str(tmp_path/'session'),
               '--transport-mode','live','--expected-sdk-session','integrity-preflight-not-sdk']
    if explicit:arguments.extend(['--purpose',explicit])
    monkeypatch.setattr(sys,'argv',arguments)
    session_entry.main()
    assert calls[0]['purpose']==expected


@pytest.fixture(scope='module')
def prepared_global(tmp_path_factory):
    if not (MAP/'manifest.json').is_file():pytest.skip('requires the recorded 10-02 planning artifact')
    patch=pytest.MonkeyPatch()
    patch.setattr(paths,'template_root',lambda:ROOT)
    def forbidden(*args,**kwargs):raise AssertionError('preparation attempted process or SDK startup')
    patch.setattr('subprocess.Popen',forbidden)
    patch.setattr('subprocess.run',forbidden)
    patch.setattr(session_entry,'check_acceptance',forbidden)
    target=tmp_path_factory.mktemp('global-route-session')/'session'
    try:
        session=session_entry.prepare(target,map_directory=MAP,transport_mode='live',purpose='planning_only',
            planning_scope='global_route',session_id='global-route-passive',
            expected_sdk_session='integrity-preflight-not-sdk')
    finally:patch.undo()
    return target,session


def parameters(target):
    return {name:yaml.safe_load((target/(name+'.yaml')).read_text())['/**']['ros__parameters']
            for name in ('bt','bt_adapter','global','reference','scan','tracker','component_health')}


@pytest.fixture(scope='module',params=[('planning_only','live'),('execution','isolated_mock')])
def prepared_full(tmp_path_factory,request):
    if not (MAP/'manifest.json').is_file():pytest.skip('requires the recorded 10-02 planning artifact')
    purpose,transport=request.param
    patch=pytest.MonkeyPatch()
    patch.setattr(paths,'template_root',lambda:ROOT)
    def forbidden(*args,**kwargs):raise AssertionError('passive preparation attempted child/SDK startup')
    patch.setattr('subprocess.Popen',forbidden)
    patch.setattr('subprocess.run',forbidden)
    patch.setattr(session_entry,'check_acceptance',forbidden)
    target=tmp_path_factory.mktemp('full-conditioned-'+purpose)/'session'
    try:
        session=session_entry.prepare(target,map_directory=MAP,transport_mode=transport,purpose=purpose,
            session_id='full-conditioned-'+purpose,expected_sdk_session='integrity-preflight-not-sdk')
    finally:patch.undo()
    return target,session


def test_conditioned_map_default_retains_full_existing_bt_and_local_graph(prepared_full,monkeypatch):
    target,session=prepared_full
    assert session['planning_scope']=='navigation'
    assert session['map_capabilities']['execution_geometry_eligible'] is False
    assert session['physical_acceptance'] is False
    assert session['behavior_tree_xml']==str(ROOT/'src/d1max_navigation_bt/trees/navigate_with_global_relocalization.xml')
    tags={node.tag for node in ET.parse(target/'navigation.xml').getroot().iter()}
    assert {'TaskContextValid','WaitForInitialLocalization','ComputeGlobalRoute',
            'FollowCommittedRoute','VerifyMeasuredArrival'}<=tags
    values=parameters(target)
    assert values['bt']['planning_scope']=='navigation'
    assert values['bt_adapter']['local_state_enabled'] is True
    assert values['component_health']['local_executor_enabled'] is True
    monkeypatch.setattr('ament_index_python.packages.get_package_prefix',lambda name:'/isolated-install/'+name)
    commands=session_entry.runtime_commands(target,session)
    assert {'reference','perception','scan','tracker','global','adapters','navigator',
            'component_health','lifecycle','route_display_cache'}<=set(commands)
    assert not any('crossfloor_global_session' in str(cmd) for cmd in commands.values())
    if session['purpose']=='planning_only':
        assert values['component_health']['motion_pipeline'] is False
        assert 'localization' in commands and 'mock_sdk' not in commands
        assert not (target/'mock_sdk.yaml').exists()
        session_entry.validate_planning_only_parameters(target,session)
    else:
        # An isolated configuration enumerates its mock without starting it.
        assert 'mock_sdk' in commands and session['motion_control_enabled'] is False
        session_entry.validate_collision_policy_parameters(target,session)
        session_entry.validate_tracking_parameters(target,session)
    assert all(session['input_hashes'][p]==digest for p,digest in session['map_runtime_dependencies'].items())


def test_full_conditioned_coordinate_audit_keeps_source_body_geometry_and_one_height_conversion(prepared_full):
    target,session=prepared_full
    values=parameters(target)
    report=mainline_coordinate_contract(yaml.safe_load((target/'localization.yaml').read_text()),
        session,values,json.loads(Path(session['planning_manifest']).read_text()))
    assert report['planning_map_geometry']==dict(frame='d1max_multifloor_planning',
        ground_only_bridge=True,control_tf=False,motion_authorized=False,planning_scope='navigation')
    assert report['roles']['velocity_command']=='d1max_loc_base_link'
    assert report['ground_to_body']['applications']==1
    assert report['ground_to_body']['owner']=='continuous_reference'
    wrong=deepcopy(session);wrong['map_capabilities']['execution_geometry_eligible']=True
    with pytest.raises(ValueError,match='conditioned_map_geometry'):
        mainline_coordinate_contract(yaml.safe_load((target/'localization.yaml').read_text()),
            wrong,values,json.loads(Path(session['planning_manifest']).read_text()))


def test_actual_conditioned_map_preparation_pins_runtime_dependencies_without_motion(prepared_global):
    target,session=prepared_global
    assert session['pipeline_contract']=='single_floor_v3'
    assert session['execution_architecture']=='task_follower_v1'
    assert session['initial_pose_transaction']=='bt_initial_pose_transaction_v1'
    assert session['planning_scope']=='global_route' and session['map_capabilities']['floor_ids']==['floor1','floor2']
    assert session['map_capabilities']['stairs_planning_enabled'] is True
    assert session['map_capabilities']['execution_geometry_eligible'] is False
    assert session['physical_acceptance'] is False and session['motion_control_enabled'] is False
    assert session['map_runtime_dependencies']
    assert all(session['input_hashes'][p]==digest for p,digest in session['map_runtime_dependencies'].items())
    assert any(p.endswith('optimized_poses.txt') for p in session['map_runtime_dependencies'])
    assert any(p.endswith('floor_evidence.npz') for p in session['map_runtime_dependencies'])
    assert session['tomogram_npz']==str(MAP/'pct/tomogram.npz')
    assert session['behavior_tree_xml']==str(ROOT/'src/d1max_navigation_bt/trees/preview_global_route.xml')
    tags={node.tag for node in ET.parse(target/'navigation.xml').getroot().iter()}
    assert {'WaitForInitialLocalization','TaskContextValid','ComputeGlobalRoute'}<=tags
    assert not tags&{'FollowCommittedRoute','VerifyMeasuredArrival'}
    values=parameters(target)
    assert values['component_health']['planning_scope']=='global_route'
    assert values['bt']['planning_scope']=='global_route'
    assert values['component_health']['local_executor_enabled'] is False
    assert values['component_health']['motion_pipeline'] is False
    # Dormant controller parameters remain the current implementation, never old stage gains.
    assert values['tracker']['velocity_damping_gain']==0.
    assert values['tracker']['yaw_damping_gain']==0.
    assert values['tracker']['align_then_forward'] is False
    assert values['tracker']['heading_threshold']==.8
    assert values['tracker']['max_speed']==.30
    assert not (target/'mock_sdk.yaml').exists() and not (target/'braking_model.json').exists()
    session_entry.validate_planning_only_parameters(target,session)


def test_preflight_rejects_bt_scope_that_would_create_a_nonexistent_local_owner(prepared_global):
    target,session=prepared_global
    path=target/'bt.yaml'
    original=path.read_text()
    content=yaml.safe_load(original)
    content['/**']['ros__parameters']['planning_scope']='navigation'
    try:
        path.write_text(yaml.safe_dump(content))
        with pytest.raises(ValueError,match='planning_scope_mismatch'):
            session_entry.validate_planning_only_parameters(target,session)
    finally:path.write_text(original)


def test_conditioned_coordinate_audit_never_relabels_body_control_or_ground_bridge_as_tf(prepared_global):
    target,session=prepared_global
    values=parameters(target)
    localization=yaml.safe_load((target/'localization.yaml').read_text())
    manifest=json.loads(Path(session['planning_manifest']).read_text())
    report=mainline_coordinate_contract(localization,session,values,manifest)
    assert report['roles']==dict(global_route='d1max_loc_map',local_map='d1max_loc_odom',
        local_curve='d1max_loc_odom',velocity_command='d1max_loc_base_link')
    assert report['planning_map_geometry']==dict(frame='d1max_multifloor_planning',
        ground_only_bridge=True,control_tf=False,motion_authorized=False,planning_scope='global_route')
    assert report['ground_to_body']['applications']==0
    assert report['ground_to_body']['owner']=='none_global_route_only'
    wrong=deepcopy(session);wrong['map_capabilities']['execution_geometry_eligible']=True
    with pytest.raises(ValueError,match='conditioned_map_geometry'):
        mainline_coordinate_contract(localization,wrong,values,manifest)
    values['bt']['planning_frame']='d1max_multifloor_planning'
    with pytest.raises(ValueError,match='coordinate wiring mismatch'):
        mainline_coordinate_contract(localization,session,values,manifest)
