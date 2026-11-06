"""Version-pinned navigation graph; prepare never launches or connects SDK.

The live graph attaches to ONE separately authorized SDK Monitor session. The
isolated graph substitutes only robot transport and sensors, never planners.
"""
import argparse
from contextlib import contextmanager,ExitStack
import fcntl
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid
import yaml
from d1max_pct_planner import paths
from .reference_geometry_contract import NATIVE_START_DISTANCE_M

PREFIX='/d1max/live_planning/'


def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''): h.update(block)
    return h.hexdigest()


def python_runtime_members(root):
    """Freeze Python and native extension code used by the same package."""
    return {p.resolve(strict=True) for p in Path(root).rglob('*')
            if p.is_file() and p.suffix in ('.py', '.so')}


def yaml_parameters(path, params):
    Path(path).write_text(yaml.safe_dump({'/**':{'ros__parameters':params}},sort_keys=True))


def scan_map_cadence(parameters):
    """The SCAN template owns fusion/display cadence; never override it here."""
    result={}
    for name,limit in (('integration_rate_hz',20.),('visualization_rate_hz',5.)):
        key='grid_map.'+name
        value=float(parameters[key])
        if not math.isfinite(value) or not 0. < value <= limit:
            raise ValueError('invalid_scan_'+name)
        result[key]=value
    return result


def prepare(output, *, map_directory=None, transport_mode='isolated_mock',
            session_id=None, expected_sdk_session='', acceptance_record='',purpose='execution',
            planning_scope='navigation'):
    from .navigation_map_package import validate_package,package_version
    from .live_view_reload import write_owned_json
    from .bt_configuration import prepare_tree,route_provenance
    from .robot_profile import load_robot_profile,scan_robot_parameters
    from .navigation_runtime_policy import load as load_runtime_policy
    from .tracking_profile import load as load_tracking_profile
    from dataclasses import asdict
    if transport_mode not in ('live','isolated_mock'):
        raise ValueError('explicit_execution_transport_required')
    planning_only=purpose=='planning_only'
    if purpose not in ('execution','planning_only') or planning_only and transport_mode!='live':
        raise ValueError('planning_only_requires_live_transport')
    if planning_scope not in ('navigation','global_route'):
        raise ValueError('invalid_navigation_planning_scope')
    global_scope=planning_scope=='global_route'
    if global_scope and (not planning_only or transport_mode!='live'):
        raise ValueError('global_route_scope_requires_planning_only')
    if planning_only and acceptance_record:
        raise ValueError('planning_only_cannot_carry_motion_acceptance')
    if transport_mode=='live' and not expected_sdk_session:
        raise ValueError('live_requires_existing_sdk_session')
    directory=Path(output).resolve()
    if directory.exists() and any(directory.iterdir()):
        raise ValueError('refusing_to_overwrite_existing_session')
    templates=paths.template_root()
    # A motion profile is runtime configuration, not an offline acceptance
    # report. The same sealed file is loaded by the sole SDK writer. Starting
    # this graph still grants no task and sends no command.
    if transport_mode=='live' and not planning_only and not acceptance_record:
        acceptance_record=str(templates/'src/d1max_pct_scan/config/runtime_motion_profile.json')
    policy_path=templates/'src/d1max_pct_scan/config/navigation_runtime.yaml'
    policy=load_runtime_policy(policy_path)
    tracking_path=templates/'src/d1max_pct_scan/config/motion_tracking.yaml'
    tracking=load_tracking_profile(tracking_path)
    map_directory=Path(map_directory or paths.default_map_directory()).resolve(strict=True)
    manifest=json.loads((map_directory/'manifest.json').read_text())
    identity_map=manifest.get('geometry_operation')=='source_identity'
    # The original-record floor branch retains its existing execution scope.
    floor=manifest.get('floor_id')
    if identity_map and (manifest.get('xyz_modified') is not False
            or not isinstance(floor,str) or not floor or manifest.get('frame_id')!='d1max_loc_map'):
        raise ValueError('source_identity_original_coordinates_required')
    validated_map=validate_package(map_directory,purpose=purpose)
    bridge=validated_map['bridge']
    capabilities=validated_map['capabilities']
    if not identity_map:floor=capabilities['floor_ids'][0]  # startup/UI hint; actual support resolves the start
    route=yaml.safe_load((map_directory/'route.yaml').read_text())
    # This graph executes floor segments only: stairs need segment transitions.
    if identity_map and (route.get('stairs_enabled') is not False or route.get('floor_id')!=floor):
        raise ValueError('route_must_match_map_floor_without_stairs')
    if sha(manifest['source_path'])!=manifest['source_sha256']:
        raise ValueError('localization_source_map_changed')
    identifier=session_id or uuid.uuid4().hex
    if not identifier or len(identifier)>128 or not all(c.isalnum() or c in '_-' for c in identifier):
        raise ValueError('invalid_session_id')
    sdk_session=expected_sdk_session or 'isolated-'+identifier
    acceptance=None
    runtime_profile=False
    motion_parameter_basis='physical_record_or_isolated_fixture'
    if transport_mode=='live' and not planning_only:
        acceptance=check_acceptance(acceptance_record)
        record=json.loads(Path(acceptance_record).read_text())
        runtime_profile=record.get('record_kind')=='runtime_motion_profile'
        if runtime_profile:motion_parameter_basis=record['parameter_basis']
    if acceptance and acceptance.get('kinematic_profile','forward_yaw')!=tracking['kinematic_profile']:
        raise ValueError('accepted_motion_kinematics_does_not_match_tracking_profile')
    max_speed=min(tracking['max_speed_mps'],float(acceptance['max_speed_mps'])) if acceptance else tracking['max_speed_mps']
    max_yaw=min(tracking['max_yaw_radps'],float(acceptance['max_yaw_radps'])) if acceptance else tracking['max_yaw_radps']
    if not 0 < max_speed <= .30 or not 0 < max_yaw <= .50:
        raise ValueError('invalid_accepted_motion_limits')
    source_config=templates/'src/d1max_localization/config/localization.yaml'
    profile_path=templates/'src/d1max_scan_planner/config/d1max_robot.yaml'
    profile=load_robot_profile(profile_path)
    entity_policy_path=templates/'src/d1max_pct_scan/config/entity_self_filter.yaml'
    entity_model_path=templates/'src/d1max_pct_scan/config/d1max_entity_model.json'
    entity_policy=yaml.safe_load(entity_policy_path.read_text())
    entity_policy['model_manifest']=str(entity_model_path)
    body_height=profile['engineering']['body_reference_height_m']
    if transport_mode=='live' and not planning_only:
        record=json.loads(Path(acceptance_record).read_text())
        if (record.get('robot_profile_sha256')!=sha(profile_path)
                or abs(record['measurements']['body_height_m']-body_height)>1e-6):
            raise ValueError('accepted_body_geometry_does_not_match_navigation_profile')
    # An isolated fixture is not a physical calibration. Live startup separately
    # verifies the record at the SDK writer, not a copied boolean in this file.
    calibration=('engineering_profile_sha256:'+sha(profile_path) if planning_only else
        'isolated_fixture:'+identifier if transport_mode=='isolated_mock' else
        ('runtime_profile_sha256:' if runtime_profile else 'physical_record_sha256:')+sha(acceptance_record))
    version=package_version(validated_map)
    frames=dict(map_frame='d1max_loc_map',odom_frame='d1max_loc_odom',
        body_frame='d1max_loc_base_link',tracking_frame='d1max_loc_tracking')
    session=dict(id=identifier,version_id=version,pipeline_contract='single_floor_v3',
        view_contract='single_window_layout_v1',
        initial_pose_transaction='bt_initial_pose_transaction_v1',
        component_health_contract='functional_liveness_v1',
        local_state_contract='continuous_odom_v2',
        handoff_contract='local_executor_v1',execution_architecture='task_follower_v1',
        interface_version=3,task_orchestrator='behaviortree_cpp_v3',
        mode=('NAVIGATION_GLOBAL_ROUTE_PLANNING_ONLY' if global_scope else
              'SINGLE_FLOOR_PLANNING_ONLY' if planning_only else 'SINGLE_FLOOR_EXECUTION'),
        planning_scope=planning_scope,
        map_capabilities=capabilities,
        map_runtime_dependencies=validated_map['dependency_files'],
        map_rebuild_input_references=validated_map.get('rebuild_input_references',[]),
        purpose=purpose,execution_purpose=purpose,transport_mode=transport_mode,
        execution_mode_policy=policy.execution_mode_policy,
        execution_stair_runtime_parameter_source=policy.execution_stair_runtime_parameter_source,
        motion_control_enabled=transport_mode=='live' and not planning_only,physical_acceptance=False,
        physical_acceptance_record=str(Path(acceptance_record).resolve()) if acceptance_record and not runtime_profile else '',
        runtime_motion_profile=str(Path(acceptance_record).resolve()) if runtime_profile else '',
        motion_parameter_basis=motion_parameter_basis,
        expected_sdk_session=sdk_session,map_pcd=manifest['source_path'],
        max_speed_mps=max_speed,max_yaw_radps=max_yaw,
        kinematic_profile=tracking['kinematic_profile'],motion_model=tracking['motion_model'],
        body_heading_policy=tracking['heading_policy'],tracking_profile_source=str(tracking_path),
        align_then_forward=tracking['align_then_forward'],
        controller_frequency_hz=tracking['controller_frequency_hz'],
        planning_manifest=str(map_directory/'manifest.json'),tomogram_npz=str(validated_map['tomogram_path']),
        crossfloor_route_config=str(map_directory/'route.yaml'),
        terrain_resolution_m=validated_map['tomogram'].resolution,
        localization_config=str(directory/'localization.yaml'),robot_profile=str(profile_path),
        frame_id=frames['map_frame'],local_planning_frame=frames['odom_frame'],
        current_floor=floor,goal_floor=floor,floor=floor,body_height=body_height,
        body_height_calibration_id=calibration,body_height_min_m=.25,body_height_max_m=.85,
        body_height_calibration_status='engineering_nominal_not_physically_verified' if planning_only or runtime_profile else 'execution_record_or_isolated_fixture',
        max_start_move_m=.30,result_timeout_s=policy.global_route_timeout_s,
        warmup_timeout_s=policy.worker_startup_timeout_s,
        readiness_timeout_s=policy.recovery_timeout_s,
        commit_wait_timeout_s=policy.recovery_timeout_s,freshness_s=.4,
        runtime_policy=asdict(policy),runtime_policy_source=str(policy_path),
        perception_backend='per_sensor_rays',
        scan_collision_policy='official',perception_timeout_s=.5,
        scan_planning_collision_policy='official',trajectory_geometry_policy='official_inflated_geometry',
        execution_collision_policy='official',
        navigation_contract=dict(frames=frames),ui_lifetime_policy='independent',
        ground_support_height_tolerance_m=.20,ground_support_max_step_m=.17)
    session['entity_self_filter']=entity_policy
    directory.mkdir(parents=True,exist_ok=True,mode=0o700)
    from d1max_pct_planner.runtime_freeze import capture_runtime_inputs
    runtime_inputs=directory/'runtime_inputs.json'
    write_owned_json(runtime_inputs,capture_runtime_inputs())
    session['runtime_inputs_contract']=str(runtime_inputs)
    model_path=None if planning_only else Path(acceptance_record).resolve() if acceptance else directory/'braking_model.json'
    if not acceptance and not planning_only:
        model=dict(schema_version=3,transport_mode='isolated_mock',fixture_only=True,
            kinematic_profile=tracking['kinematic_profile'],
            model='reaction_braking_reachable_v1',session_id=identifier,
            note='Analytic control-driven fixture; not a physical robot calibration.',
            execution_timing=dict(sensor_source_age_bound_s=.5,command_pipeline_bound_s=.20,
                writer_period_s=.05,source_time_uncertainty_s=.02),
            stationary_evidence=dict(profile='general_low_speed',linear_threshold_mps=.03,angular_threshold_radps=.05,
                stationary_duration_s=1.,reentry_duration_s=.6,minimum_new_samples=3,
                mc_expected_hz=50.,mc_min_hz=40.,mc_max_hz=60.,
                measured_static_linear_bound_mps=.01,measured_static_angular_bound_radps=.02),
            measurements=dict(model='reaction_braking_reachable_v1',max_speed_mps=max_speed,max_yaw_radps=max_yaw,reaction_bound_s=.77,
                stopping_distance_m=.08,stopping_yaw_rad=.20,stop_latency_bound_s=.50,
                tracking_error_bound_m=.01,heading_error_bound_rad=.03))
        if tracking['motion_model']=='holonomic':
            model['measurements']['stopping_lateral_distance_m']=.08
            # Unit-gain simulated transport only; never a D1 Max measurement.
            model['measurements']['lateral_metres_per_fraction']=1.
        write_owned_json(model_path,model)
    session['execution_braking_model_record']=str(model_path) if model_path else ''
    session['execution_braking_model_sha256']=sha(model_path) if model_path else ''
    if model_path:
        from .braking_model import load_model
        load_model(model_path,session['execution_braking_model_sha256'],transport_mode)
        from .execution_timing import validate_timing,validate_stationary
        bound_record=json.loads(model_path.read_text())
        # One original-source budget belongs to the hashed runtime record.
        # Map fusion and reference health must not retain a shorter hard-coded
        # 500 ms limit after the writer/Native record has been changed.
        timing=validate_timing(bound_record)
        session['perception_timeout_s']=timing['sensor_source_age_bound_s']
        session['execution_timing']=validate_timing(bound_record,
            sensor_timeout_s=session['perception_timeout_s'])
        session['stationary_evidence']=validate_stationary(bound_record)
    localization=yaml.safe_load(source_config.read_text())
    # Resolve the new stage into the session snapshot, not live-editable launch
    # defaults. The pinned legacy config/launch remain byte-identical.
    from d1max_localization.global_registration import RegistrationConfig
    from d1max_localization.startup_relocalization import StartupConfig
    global_template=templates/'src/d1max_localization/config/global_relocalization.yaml'
    global_params={'global_relocalization.registration.'+k:v for k,v in asdict(RegistrationConfig()).items()}
    global_params.update({'global_relocalization.startup.'+k:v for k,v in asdict(StartupConfig()).items()})
    global_params.update(yaml.safe_load(global_template.read_text())['lio_localizer']['ros__parameters'])
    global_params.update(localization['lio_localizer']['ros__parameters'])
    localization['lio_localizer']['ros__parameters']=global_params
    session['global_relocalization_template']=str(global_template)
    # Registration gravity is a map-bound runtime dependency, not an assumed
    # vertical Z or a physical-calibration flag. Seal its actual lineage too.
    gravity_inputs=[]
    gravity_record=global_params.get('global_relocalization.gravity.map_contract_json','')
    if global_params.get('global_relocalization.enabled') and gravity_record:
        from d1max_localization.registration_gravity import load_map_gravity_contract
        session['registration_map_gravity']=load_map_gravity_contract(
            gravity_record,manifest['source_sha256'],frames['map_frame'])
        gravity_inputs=[Path(item['path']) for item in json.loads(gravity_record)['evidence'].values()]
    realtime_template=templates/'src/d1max_localization/config/realtime_navigation.yaml'
    realtime_params=yaml.safe_load(realtime_template.read_text())['navigation_estimation']['ros__parameters']
    localization['navigation_estimation']['ros__parameters'].update(realtime_params)
    session['realtime_navigation_template']=str(realtime_template)
    session['realtime_navigation_template_sha256']=sha(realtime_template)
    localization['dual_lidar_adapter']['ros__parameters']['perception_rays.enabled']=True
    from .perception_contract import MAX_ACQUISITION_POINTS
    session['perception_projector']=dict(max_input_points=MAX_ACQUISITION_POINTS)
    localization['dual_lidar_adapter']['ros__parameters']['perception_rays.max_input_points']=MAX_ACQUISITION_POINTS
    (directory/'localization.yaml').write_text(yaml.safe_dump(localization))
    tree_source=templates/'src/d1max_navigation_bt/trees'/(
        'preview_global_route.xml' if global_scope else 'navigate_with_global_relocalization.xml')
    session['behavior_tree_xml']=str(tree_source)
    prepare_tree(directory,session,templates)
    bt=yaml.safe_load((directory/'bt.yaml').read_text())['/**']['ros__parameters']
    bt.update(planning_frame='d1max_loc_map',preview_only=planning_only,execution_purpose=purpose,
        planning_scope=session['planning_scope'],
        execution_architecture='task_follower_v1',
        dependency_health_required=True,
        execution_transport_mode=transport_mode,expected_sdk_session=sdk_session,
        arrival_evidence_timeout_s=4.)
    bt.update(policy.bt_parameters())
    if model_path:
        stationary=session['stationary_evidence']
        bt.update(command_pipeline_bound_s=session['execution_timing']['command_pipeline_bound_s'],
            stationary_velocity_metric=stationary.get('velocity_metric','spatial_norm_3d'),
            stationary_linear_threshold_mps=stationary['linear_threshold_mps'],
            stationary_angular_threshold_radps=stationary['angular_threshold_radps'],
            stationary_stop_duration_s=stationary['stationary_duration_s'],
            stationary_reentry_duration_s=stationary['reentry_duration_s'],
            stationary_minimum_samples=stationary['minimum_new_samples'])
    if acceptance:
        bt.update(execution_acceptance_record=str(Path(acceptance_record).resolve()),
                  execution_record_sha256=sha(acceptance_record))
    # Cold native map/worker initialization has its own bounded preparation
    # budget; the functional monitor must not kill it at an unrelated 30 s.
    startup=policy.worker_startup_timeout_s
    if not math.isfinite(startup) or startup>60.:
        raise ValueError('invalid_component_startup_budget')
    session['component_startup_timeout_s']=startup
    yaml_parameters(directory/'bt.yaml',bt)
    adapter=yaml.safe_load((directory/'bt_adapter.yaml').read_text())['/**']['ros__parameters']
    adapter.update(execution_mode='single_floor',pipeline_contract='single_floor_v3',
        execution_architecture='task_follower_v1',
        map_version_id=version,transport_mode=transport_mode,local_state_enabled=not global_scope,
        arrival_evidence_timeout_s=bt['arrival_evidence_timeout_s'])
    yaml_parameters(directory/'bt_adapter.yaml',adapter)
    global_params={k:session[k] for k in ('planning_manifest','tomogram_npz','crossfloor_route_config',
        'current_floor','goal_floor','body_height_min_m','body_height_max_m','max_start_move_m',
        'result_timeout_s','warmup_timeout_s','readiness_timeout_s','commit_wait_timeout_s',
        'freshness_s','pipeline_contract','transport_mode')}
    global_params.update(session_id=identifier,map_version_id=version,output_directory=str(directory))
    yaml_parameters(directory/'global.yaml',global_params)
    # One configuration owns map extent and planning lookahead. Do not make the
    # displayed map larger while the reference/solver remain stuck at 2 metres.
    scan_source=templates/'src/d1max_scan_planner/config/d1max_scan_planner.yaml'
    scan=next(iter(yaml.safe_load(scan_source.read_text()).values()))['ros__parameters']
    # Session policy, not a native ROS parameter or a physical/SDK profile
    # change. One padded XY kernel serves candidate and current-curve checks.
    planar_clearance=scan.pop('planar_clearance_m',0.)
    stair_clearance=scan.pop('stair_planar_clearance_m',None)
    robot_parameters=scan_robot_parameters(profile,strict_layer_available=False,
        planar_clearance_m=planar_clearance,stair_planar_clearance_m=stair_clearance)
    session['scan_planar_clearance_m']=float(planar_clearance)
    if stair_clearance is not None:
        session['scan_stair_planar_clearance_m']=float(stair_clearance)
    map_cadence=scan_map_cadence(scan)
    horizon=float(scan['fsm.planning_horizon'])
    if not math.isfinite(horizon) or horizon!=float(scan['manager.planning_horizon']):
        raise ValueError('local_reference_and_solver_horizon_mismatch')
    session['local_planning_window']=dict(
        size_m=[scan['grid_map.sliding_map_size_'+axis] for axis in ('x','y','z')],
        resolution_m=scan['grid_map.resolution'],reference_horizon_m=horizon,
        integration_rate_hz=map_cadence['grid_map.integration_rate_hz'],
        visualization_rate_hz=map_cadence['grid_map.visualization_rate_hz'])
    session['local_pipeline_contract']='local_executor_v1'
    ref=dict(session_id=identifier,map_version_id=version,body_height_m=body_height,
        execution_purpose=purpose,
        execution_mode_policy=policy.execution_mode_policy,
        local_state_enabled=True,async_support_preparation=True,geometry_owner='local_executor',
        body_height_calibration_id=calibration,freshness_s=.4,transport_mode=transport_mode,
        map_evidence_max_age_s=session['perception_timeout_s'],
        reference_horizon_m=horizon,floor_id=floor,
        planning_manifest=session['planning_manifest'],
        **route_provenance(session))
    yaml_parameters(directory/'reference.yaml',ref)
    scan.update(robot_parameters)
    scan.update({'use_sim_time':False,'fsm.navi_mode':3,'fsm.require_tagged_reference':True,
        'fsm.execution_mode_policy':policy.execution_mode_policy,
        'fsm.component_callback_timeout_s':policy.component_callback_timeout_s,
        'fsm.kinematic_profile':tracking['kinematic_profile'],
        'manager.body_heading_policy':tracking['heading_policy'],
        'manager.align_then_forward':tracking['align_then_forward'],
        'fsm.planning_only':False,
        'fsm.require_reference_schema_v2':True,'fsm.navigation_session_id':identifier,
        'fsm.navigation_map_version_id':version,
        'fsm.execution_protocol':True,'fsm.execution_transport_mode':transport_mode,
        'fsm.writer_handoff_enabled':False,'fsm.local_follower_enabled':True,
        'fsm.control_body_frame':frames['body_frame'],
        'fsm.trajectory_geometry_policy':session['trajectory_geometry_policy'],
        'fsm.execution_braking_model_record':session['execution_braking_model_record'],
        'fsm.execution_braking_model_sha256':session['execution_braking_model_sha256'],
        'fsm.strict_input_frames':True,'fsm.odom_twist_in_body_frame':True,
        'fsm.odom_timeout':.4,
        'fsm.reference_goal_xy_tolerance':.20,'fsm.reference_goal_z_tolerance':.15,
        'fsm.reference_path_guidance':True,'fsm.reference_path_z_offset':0.,
        # ReferenceWindow begins at its bounded route projection, not at the
        # actual body. Match the sender's XY/Z correspondence before Native
        # performs its unchanged collision-checked connector search.
        'fsm.reference_start_tolerance':NATIVE_START_DISTANCE_M,
        'manager.max_vel':max_speed,'optimization.max_vel':max_speed,
        'manager.max_acc':tracking['max_acceleration_mps2'],'optimization.max_acc':tracking['max_acceleration_mps2'],
        # Scheduling reserve uses the same command-shaping parameter as the
        # follower; this is not a second braking/collision authorization.
        'fsm.continuation_max_jerk':tracking['max_jerk_mps3'],
        'grid_map.frame_id':frames['odom_frame'],'grid_map.sliding_map_frame_id':'scan_local_'+identifier[:8],
        'grid_map.strict_input_frames':True,'grid_map.require_observed_free':False,
        'grid_map.planning_collision_policy':session['scan_planning_collision_policy'],
        'grid_map.preview_only':False,'grid_map.require_localization_context':True,
        'grid_map.localization_session_id':identifier,'grid_map.maximum_cloud_pose_dt':.25,
        'grid_map.cloud_pose_max_age':session['perception_timeout_s'],'grid_map.cloud_is_world':True,'grid_map.need_extrinsic':False,
        'grid_map.use_projected_rays':True,
        **map_cadence})
    yaml_parameters(directory/'scan.yaml',scan)
    tracker=dict(session_id=identifier,map_version_id=version,transport_mode=transport_mode,
        local_state_enabled=True,writer_handoff_enabled=False,
        require_body_heading_seed=True,
        trajectory_geometry_policy=session['trajectory_geometry_policy'],
        local_navigation_state_topic='/d1max/localization/navigation/local_state',
        map_frame=frames['map_frame'],planning_frame=frames['odom_frame'],base_frame=frames['body_frame'],
        motion_model=tracking['motion_model'],holonomic_yaw_policy=tracking['heading_policy'],
        align_then_forward=tracking['align_then_forward'],
        controller_frequency_hz=tracking['controller_frequency_hz'],
        diagnostics_frequency_hz=tracking['diagnostics_frequency_hz'],
        lookahead=tracking['preview_time_s'],
        min_lookahead_distance_m=tracking['min_lookahead_distance_m'],
        max_lookahead_distance_m=tracking['max_lookahead_distance_m'],
        kp_position=tracking['position_gain'],kp_yaw=tracking['yaw_gain'],
        heading_threshold=tracking['heading_error_threshold_rad'],
        steering_enter_angle_rad=tracking['steering_enter_angle_rad'],
        steering_exit_angle_rad=tracking['steering_exit_angle_rad'],
        steering_error_scale_rad=tracking['steering_error_scale_rad'],
        velocity_damping_gain=tracking['velocity_damping_gain'],
        yaw_damping_gain=tracking['yaw_damping_gain'],
        max_speed=max_speed,max_yaw_rate=max_yaw,max_acceleration=tracking['max_acceleration_mps2'],
        max_yaw_acceleration=tracking['max_yaw_acceleration_radps2'],
        max_jerk=tracking['max_jerk_mps3'],max_yaw_jerk=tracking['max_yaw_jerk_radps3'],
        command_pipeline_bound_s=.1 if planning_only else session['execution_timing']['command_pipeline_bound_s'],
        writer_period_s=.05 if planning_only else session['execution_timing']['writer_period_s'],
        odom_timeout=.4,task_timeout=.75,trajectory_timeout=2.,goal_tolerance=.20,goal_height_tolerance=.15,
        execution_braking_model_sha256=session['execution_braking_model_sha256'],
        trajectory_topic=PREFIX+'scan_tagged_bspline',progress_topic=PREFIX+'tracking_progress',
        demand_topic=PREFIX+('preview/demand' if planning_only else 'execution/demand'),
        command_topic=PREFIX+('preview/command_debug' if planning_only else 'execution/command_debug'),
        support_topic=PREFIX+'execution/support',
        status_topic=PREFIX+'tracker_status')
    if model_path:
        stationary=session['stationary_evidence']
        tracker.update(stationary_linear_threshold_mps=stationary['linear_threshold_mps'],
            stationary_velocity_metric=stationary.get('velocity_metric','spatial_norm_3d'),
            stationary_angular_threshold_radps=stationary['angular_threshold_radps'],
            stationary_reentry_duration_s=stationary['reentry_duration_s'],
            stationary_minimum_samples=stationary['minimum_new_samples'])
    from .tracking_profile import command_response_parameters
    tracker.update(command_response_parameters(tracking,
        json.loads(model_path.read_text()) if model_path else None))
    yaml_parameters(directory/'tracker.yaml',tracker)
    yaml_parameters(directory/'component_health.yaml',dict(session_id=identifier,
        output_directory=str(directory),motion_pipeline=not planning_only,
        local_executor_enabled=not global_scope,planning_scope=session['planning_scope'],**policy.health_parameters()))
    model_params={key:session[key] for key in ('execution_braking_model_record','execution_braking_model_sha256')}
    if not planning_only:
        yaml_parameters(directory/'mock_sdk.yaml',dict(session_id=identifier,map_version_id=version,
            execution_mode_policy=policy.execution_mode_policy,
            execution_stair_runtime_parameter_source=policy.execution_stair_runtime_parameter_source,
            execution_task_writer_enabled=True,execution_v3_enabled=False,navigation_control_enabled=False,
            sdk_session=sdk_session,execution_mc_delay_bound_s=.02,local_state_enabled=True,
            local_navigation_state_topic='/d1max/localization/navigation/local_state',**model_params))
    from .live_visualization import configure
    rviz_source=templates/'src/d1max_navigation/rviz/localization_test.rviz'
    base=yaml.safe_load(rviz_source.read_text())
    for layout in ('global','local'):
        config=configure(base,session_id=identifier,layout=layout,session_directory=directory,
                         terrain_resolution_m=session['terrain_resolution_m'],
                         local_window_size_m=session['local_planning_window']['size_m'])
        config['Panels'][0]['Motion Capable']=not planning_only
        # Only the typed task confirmation UI, never legacy manual SDK controls.
        config['Visualization Manager']['Global Options']['Fixed Frame']=(
            frames['odom_frame'] if layout=='local' else frames['map_frame'])
        if layout=='local':
            # This window observes continuous odom. Initial poses and targets
            # belong to the global map window, not an accidentally odom-framed
            # click converted by relabelling its coordinates.
            config['Visualization Manager']['Tools']=[tool for tool in config['Visualization Manager']['Tools']
                if tool['Class'] not in ('rviz_default_plugins/SetInitialPose','rviz_default_plugins/SetGoal',
                    'rviz_default_plugins/PublishPoint','d1max_pct_rviz_tools/LiveGoal3D')]
        for group in config['Visualization Manager']['Displays']:
            for display in group.get('Displays',[]):
                if display.get('Name')=='局部轨迹':display['Name']='已提交局部轨迹'
                if display.get('Name')=='全局路径':
                    # Only the v3 profile uses committed-route display history.
                    # Task retirement still clears the worker's live output.
                    display['Topic']['Value']=PREFIX+'committed_route_visual'
        (directory/(layout+'_planning.rviz')).write_text(yaml.safe_dump(config,allow_unicode=True))
    inputs=[Path(session[k]) for k in ('map_pcd','planning_manifest','tomogram_npz','crossfloor_route_config','robot_profile')]
    inputs.extend(Path(p) for p in validated_map['dependency_files'])
    # Seal the templates actually used to create future sessions, not only the
    # generated copy: otherwise editing a source XML after release sealing
    # silently changes task policy in an apparently identical release.
    inputs.extend([source_config,global_template,realtime_template,scan_source,tree_source,rviz_source,policy_path,tracking_path])
    inputs.extend(gravity_inputs)
    inputs.extend([entity_policy_path,entity_model_path])
    # CAD assets are immutable inputs, not unchecked paths outside the release.
    from .robot_self_geometry import RobotSelfModel
    entity_model=RobotSelfModel.load(entity_model_path)
    inputs.extend(entity_model.asset_paths)
    inputs.append(runtime_inputs)
    if model_path:inputs.append(model_path)
    if acceptance_record: inputs.append(Path(acceptance_record).resolve())
    session['input_hashes']={str(p.resolve()):sha(p) for p in inputs}
    session['parameter_hashes']={p.name:sha(p) for p in directory.iterdir() if p.suffix in ('.yaml','.xml','.rviz')}
    write_owned_json(directory/'session.json',session)
    return session


def planning_only_profile(session):
    purpose=session.get('purpose',session.get('execution_purpose','execution'))
    if purpose not in ('execution','planning_only') or session.get('execution_purpose',purpose)!=purpose:
        raise ValueError('session_purpose_mismatch')
    if purpose!='planning_only':return False
    scope=session.get('planning_scope','navigation')
    if scope not in ('navigation','global_route'):
        raise ValueError('invalid_navigation_planning_scope')
    expected_mode=('NAVIGATION_GLOBAL_ROUTE_PLANNING_ONLY' if scope=='global_route'
                   else 'SINGLE_FLOOR_PLANNING_ONLY')
    if (session.get('transport_mode')!='live' or session.get('motion_control_enabled') is not False
            or session.get('mode')!=expected_mode
            or session.get('physical_acceptance_record') or session.get('execution_braking_model_record')
            or session.get('execution_braking_model_sha256')):
        raise ValueError('invalid_planning_only_capability')
    if scope=='global_route' and (session.get('physical_acceptance') is not False
            or session.get('map_capabilities',{}).get('geometry_kind')!='conditioned_multifloor_ground'
            or session.get('map_capabilities',{}).get('execution_geometry_eligible') is not False):
        raise ValueError('conditioned_global_route_cannot_authorize_execution')
    return True


def validate_collision_policy_parameters(directory,session):
    """Use one SCAN collision policy in preview, admission and tracking.

    Execution purpose still controls SDK capability, not another occupancy rule.
    Unknown evidence is preserved in the map, but is not relabelled as collision
    by a downstream observed-free gate.
    """
    motion=not planning_only_profile(session)
    def params(name):return yaml.safe_load((Path(directory)/name).read_text())['/**']['ros__parameters']
    scan=params('scan.yaml')
    tracker=params('tracker.yaml') if motion else None
    profile=session.get('kinematic_profile','forward_yaw')
    model='holonomic' if profile=='holonomic_xyyaw' else 'forward_yaw'
    heading=session.get('body_heading_policy','path_tangent')
    if (profile not in ('forward_yaw','holonomic_xyyaw')
            or scan.get('fsm.kinematic_profile','forward_yaw')!=profile
            or scan.get('manager.body_heading_policy','path_tangent')!=heading
            or motion and (tracker.get('motion_model','forward_yaw')!=model
                or tracker.get('holonomic_yaw_policy','path_tangent')!=heading)):
        raise ValueError('inconsistent_planner_tracker_motion_profile')
    if (scan.get('manager.align_then_forward',False)!=session.get('align_then_forward',False)
            or motion and tracker.get('align_then_forward',False)!=session.get('align_then_forward',False)):
        raise ValueError('inconsistent_tracker_following_strategy')
    if (session.get('scan_collision_policy')!='official'
            or session.get('scan_planning_collision_policy')!='official'
            or session.get('trajectory_geometry_policy')!='official_inflated_geometry'
            or session.get('execution_collision_policy')!='official'
            or scan.get('grid_map.planning_collision_policy')!='official'
            or scan.get('fsm.trajectory_geometry_policy')!='official_inflated_geometry'
            or motion and tracker.get('trajectory_geometry_policy')!='official_inflated_geometry'
            or scan.get('fsm.planning_only') is not False
            or scan.get('grid_map.require_observed_free') is not False):
        raise ValueError('inconsistent_planning_geometry_or_motion_evidence_policy')


def validate_tracking_parameters(directory,session):
    """Check the generated controller and BT against their sealed shared inputs."""
    if planning_only_profile(session):return
    from .tracking_profile import load as load_tracking_profile
    tracking=load_tracking_profile(session['tracking_profile_source'])
    def params(name):return yaml.safe_load((Path(directory)/name).read_text())['/**']['ros__parameters']
    expected={'controller_frequency_hz':tracking['controller_frequency_hz'],
        'diagnostics_frequency_hz':tracking['diagnostics_frequency_hz'],
        'lookahead':tracking['preview_time_s'],
        'min_lookahead_distance_m':tracking['min_lookahead_distance_m'],
        'max_lookahead_distance_m':tracking['max_lookahead_distance_m'],
        'kp_position':tracking['position_gain'],'kp_yaw':tracking['yaw_gain'],
        'heading_threshold':tracking['heading_error_threshold_rad'],
        'steering_enter_angle_rad':tracking['steering_enter_angle_rad'],
        'steering_exit_angle_rad':tracking['steering_exit_angle_rad'],
        'steering_error_scale_rad':tracking['steering_error_scale_rad'],
        'motion_model':tracking['motion_model'],
        'holonomic_yaw_policy':tracking['heading_policy'],
        'align_then_forward':tracking['align_then_forward'],
        'require_body_heading_seed':True,
        'velocity_damping_gain':tracking['velocity_damping_gain'],
        'yaw_damping_gain':tracking['yaw_damping_gain'],
        'max_acceleration':tracking['max_acceleration_mps2'],
        'max_yaw_acceleration':tracking['max_yaw_acceleration_radps2'],
        'max_jerk':tracking['max_jerk_mps3'],
        'max_yaw_jerk':tracking['max_yaw_jerk_radps3'],
        'command_pipeline_bound_s':session['execution_timing']['command_pipeline_bound_s'],
        'writer_period_s':session['execution_timing']['writer_period_s']}
    from .tracking_profile import command_response_parameters
    model_path=session.get('execution_braking_model_record')
    expected.update(command_response_parameters(tracking,
        json.loads(Path(model_path).read_text()) if model_path else None))
    tracker=params('tracker.yaml')
    if any(tracker.get(key)!=value for key,value in expected.items()):
        raise ValueError('inconsistent_tracking_controller_parameters')
    scan=params('scan.yaml')
    if (scan.get('manager.align_then_forward')!=tracking['align_then_forward']
            or scan.get('manager.body_heading_policy')!=tracking['heading_policy']
            or scan.get('fsm.kinematic_profile')!=tracking['kinematic_profile']):
        raise ValueError('inconsistent_native_tracking_heading_contract')
    if scan.get('fsm.continuation_max_jerk')!=tracking['max_jerk_mps3']:
        raise ValueError('inconsistent_continuation_scheduling_profile')
    if params('bt.yaml').get('command_pipeline_bound_s')!=expected['command_pipeline_bound_s']:
        raise ValueError('inconsistent_execution_command_pipeline_budget')


def validate_planning_only_parameters(directory,session):
    if not planning_only_profile(session):return
    validate_collision_policy_parameters(directory,session)
    def params(name):return yaml.safe_load((Path(directory)/name).read_text())['/**']['ros__parameters']
    bt,scan,health=params('bt.yaml'),params('scan.yaml'),params('component_health.yaml')
    tracker=params('tracker.yaml')
    global_scope=session.get('planning_scope')=='global_route'
    if bt.get('planning_scope','navigation')!=session.get('planning_scope','navigation'):
        raise ValueError('navigation_bt_planning_scope_mismatch')
    if (bt.get('execution_purpose')!='planning_only' or bt.get('preview_only') is not True
            or bt.get('execution_transport_mode')!='live' or bt.get('execution_acceptance_record')
            or bt.get('execution_record_sha256')
            or session.get('local_pipeline_contract')!='local_executor_v1'
            or health.get('motion_pipeline') is not False
            or tracker.get('demand_topic')!=PREFIX+'preview/demand'
            or scan.get('fsm.execution_protocol') is not True
            or scan.get('fsm.writer_handoff_enabled') is not False
            or scan.get('fsm.local_follower_enabled') is not True
            or scan.get('fsm.execution_transport_mode')!='live'
            or scan.get('fsm.execution_braking_model_record') or scan.get('fsm.execution_braking_model_sha256')):
        raise ValueError('planning_only_parameter_capability_mismatch')
    if global_scope:
        import xml.etree.ElementTree as ET
        tags={node.tag for node in ET.parse(Path(directory)/'navigation.xml').getroot().iter()}
        if (health.get('planning_scope')!='global_route' or health.get('local_executor_enabled') is not False
                or not {'TaskContextValid','WaitForInitialLocalization','ComputeGlobalRoute'}<=tags
                or tags&{'FollowCommittedRoute','VerifyMeasuredArrival','ConfirmExecution'}):
            raise ValueError('global_route_scope_must_not_require_local_execution_or_arrival')
    if any((Path(directory)/name).exists() for name in ('safety.yaml','mock_sdk.yaml')):
        raise ValueError('planning_only_forbids_motion_pipeline_configuration')


def runtime_commands(directory, session):
    from ament_index_python.packages import get_package_prefix
    from .bt_configuration import worker_arguments
    def binary(package,executable):
        return str(Path(get_package_prefix(package))/'lib'/package/executable)
    def py(module,params):
        return [sys.executable,'-m','d1max_pct_scan.'+module,'--ros-args','--params-file',str(directory/params)]
    def cpp(package,exe,params):
        return [binary(package,exe),'--ros-args','--params-file',str(directory/params)]
    global_scope=session.get('planning_scope')=='global_route'
    if global_scope and not planning_only_profile(session):
        raise ValueError('global_route_scope_requires_planning_only')
    commands={
        'route_display_cache':[sys.executable,'-m','d1max_pct_scan.committed_route_view','--session',str(directory)],
        'global':py('live_global_planner','global.yaml')+worker_arguments(),
        'adapters':py('bt_adapters','bt_adapter.yaml'),
        'component_health':py('component_health_monitor','component_health.yaml'),
        'navigator':cpp('d1max_navigation_bt','navigator_node','bt.yaml'),
        'lifecycle':cpp('nav2_lifecycle_manager','lifecycle_manager','bt_lifecycle.yaml')}
    if not global_scope:
        commands.update(
            reference=py('continuous_reference_node','reference.yaml'),
            perception=[sys.executable,'-m','d1max_pct_scan.perception_ray_projector','--session',str(directory)],
            scan=cpp('scan_planner','scan_planner_node','scan.yaml'),
            tracker=cpp('d1max_trajectory_tracker','local_follower','tracker.yaml'))
    remaps={'body_pose':PREFIX+'body_pose','sensor_pose':PREFIX+'sensor_pose',
        'cloud':PREFIX+'unused_lio_cloud','projected_rays':PREFIX+'rays_odom',
        'grid_map/projected_rays_status':PREFIX+'rays_status',
        'typed_initial_path':PREFIX+'scan_reference',
        'grid_map/localization_context':PREFIX+'scan_map_context',
        'grid_map/localization_context_ack':PREFIX+'scan_map_context_ack',
        'planning/tagged_bspline':PREFIX+'scan_tagged_bspline',
        'planning/committed_bspline':PREFIX+'committed_bspline',
        'planning/tracking_progress':PREFIX+'tracking_progress',
        'planning/local_plan_debug':PREFIX+'native_local_debug',
        'planning/local_attempt_debug':PREFIX+'native_local_attempt_debug'}
    if not global_scope:
        commands['scan']+=['-r','__ns:=/d1max/live_planning/scan','-r','__node:=scan_planner_node']
        for source,target in remaps.items(): commands['scan']+=['-r',source+':='+target]
    commands['lifecycle']+=['-r','__node:=d1max_navigation_lifecycle']
    if session['transport_mode']=='isolated_mock':
        commands={'mock_sdk':cpp('d1max_sdk_bridge','sdk_execution_mock','mock_sdk.yaml'),**commands}
    else:
        commands={'localization':['ros2','launch','d1max_localization','global_localization.launch.py',
            'config:='+str(directory/'localization.yaml'),'session_dir:='+str(directory),
            'map_pcd:='+session['map_pcd'],
            'relocalization_config:='+session['localization_config']],**commands}
    return commands


def view_commands(directory,layout,*,include_publishers=True):
    if layout not in ('global','local'):raise ValueError('unknown_view_layout')
    publishers={name:[sys.executable,'-m','d1max_pct_scan.'+module,'--session',str(directory)]
        for name,module in (('view','live_view'),('map_layers','live_map_layers'),('execution_view','execution_view'))}
    # One tree retains the map-framed initial/goal tools. Local mode is a
    # display/camera selection, not a second process or a different task.
    return {**(publishers if include_publishers else {}),
        'rviz':['rviz2','-d',str(directory/'global_planning.rviz'),'--ros-args','-r',
                '__node:=d1max_navigation_rviz']}


@contextmanager
def _owned_file_lock(path,reason):
    """Do not unlink a lock inode while another process might still own it."""
    import stat
    path=Path(path)
    fd=os.open(path,os.O_CREAT|os.O_RDWR|os.O_NOFOLLOW|os.O_CLOEXEC,0o600)
    try:
        info=os.fstat(fd)
        if not stat.S_ISREG(info.st_mode) or info.st_uid!=os.getuid() or info.st_nlink!=1:
            raise ValueError('navigation_lock_not_private:'+str(path))
        repair=bool(info.st_mode&0o077)
        if repair:
            # Older shell diagnostics created this empty lease with umask
            # permissions. Harden only a self-owned, empty lock in a private
            # owned directory; preserve its inode and exclusive lease.
            parent=path.parent.lstat()
            if (info.st_size or not stat.S_ISDIR(parent.st_mode)
                    or parent.st_uid!=os.getuid() or parent.st_mode&0o077):
                raise ValueError('navigation_lock_not_private:'+str(path))
        try:fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError as exc:raise ValueError(reason) from exc
        if repair:os.fchmod(fd,0o600)
        yield fd
    finally:os.close(fd)


@contextmanager
def view_layout_lock(directory,layout):
    if layout not in ('global','local'):raise ValueError('unknown_view_layout')
    domain=int(os.environ.get('ROS_DOMAIN_ID','24'))
    if domain<0:raise ValueError('invalid_navigation_domain')
    # The window lease is namespace-wide, including CLI callers with different
    # session folders. Keep both old lock names while running, so an existing
    # legacy viewer of this session cannot coexist with a new one.
    from contextlib import ExitStack
    with ExitStack() as locks:
        fd=locks.enter_context(_owned_file_lock(
            _core_lock_root()/('view-domain-'+str(domain)+'.lock'),'navigation_view_already_running'))
        for old_layout in ('global','local'):
            locks.enter_context(_owned_file_lock(Path(directory)/('view-'+old_layout+'.lock'),
                'legacy_navigation_view_already_running'))
        yield fd


def _try_display_lease(directory):
    """One publisher set serves the single viewer's global/local layouts."""
    lease=_owned_file_lock(Path(directory)/'display-publishers.lock','display_publishers_already_running')
    try:fd=lease.__enter__()
    except ValueError as error:
        if str(error)=='display_publishers_already_running':return None
        raise
    return lease,fd


def view(directory,layout):
    """Independent UI process group. Closing it never drains/cancels the owner."""
    from .isolated_zenoh import validate_environment
    from .live_runtime import stop_owned_children
    directory=Path(directory).resolve(strict=True);s=verify_view(directory)
    if os.environ.get('RMW_IMPLEMENTATION')!='rmw_zenoh_cpp':raise ValueError('unchanged_zenoh_required')
    if s['transport_mode']=='isolated_mock':validate_environment()
    from .view_layout_requests import viewer_environment
    with view_layout_lock(directory,layout) as window_fd:
        view_env=viewer_environment(directory,s,layout)
        children=[];logs=[];stopping=False;display_lease=None
        def stop(*_):
            nonlocal stopping
            stopping=True
        previous={sig:signal.signal(sig,stop) for sig in (signal.SIGINT,signal.SIGTERM)}
        try:
            for name,cmd in view_commands(directory,layout,include_publishers=False).items():
                log=(directory/(name+'-single.log')).open('a');logs.append(log)
                children.append((name,subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True,
                    env=view_env,pass_fds=(window_fd,))))
            while not stopping and all(p.poll() is None for _,p in children):
                if display_lease is None:
                    display_lease=_try_display_lease(directory)
                    if display_lease is not None:
                        for name,cmd in view_commands(directory,layout).items():
                            if name=='rviz':continue
                            log=(directory/(name+'.log')).open('a');logs.append(log)
                            children.append((name,subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True,
                                pass_fds=(display_lease[1],window_fd))))
                time.sleep(.1)
        finally:
            # Release only after RViz and the publisher children are reaped;
            # a direct second CLI cannot overlap a lingering window.
            try:
                stop_owned_children(children,interrupt_s=6.,terminate_s=1.,kill_s=1.)
            finally:
                release_supervisor_resources(logs,previous,
                    display_lease[0] if display_lease is not None else None)


def release_supervisor_resources(logs,previous,lease=None):
    """All local resources unwind even if one close/release operation fails."""
    with ExitStack() as resources:
        for sig,handler in previous.items():resources.callback(signal.signal,sig,handler)
        if lease is not None:resources.callback(lease.__exit__,None,None,None)
        for stream in logs:resources.callback(stream.close)


def check_acceptance(record):
    """Writer's runtime-profile/legacy-record validator, not a motion permit."""
    from ament_index_python.packages import get_package_prefix
    record=Path(record).resolve(strict=True)
    content=json.loads(record.read_text())
    checker=Path(get_package_prefix('d1max_sdk_bridge'))/'lib/d1max_sdk_bridge/execution_acceptance_check'
    command=[str(checker),str(record),content['robot_id'],content['sdk_version'],content['calibration_sha256'],
             content['robot_profile_sha256']]
    result=subprocess.run(command,text=True,capture_output=True,timeout=10)
    if result.returncode:
        raise ValueError('motion_profile_rejected:'+result.stdout.strip())
    value=json.loads(result.stdout)
    if value.get('valid') is not True: raise ValueError('motion_profile_invalid')
    if content.get('record_kind')=='runtime_motion_profile':
        from .execution_timing import validate_runtime_profile,validate_sdk_input_mapping
        validate_runtime_profile(content)
        mapping=validate_sdk_input_mapping(content)
        if mapping is not None and (value.get('sdk_input_mapping')!=mapping
                or value.get('parameter_basis')!=content['parameter_basis']):
            raise ValueError('sdk_writer_input_mapping_version_mismatch')
    return value


def validate_live_localization_parameters(config, *, package_prefix=None):
    """Fail before launch if the selected live graph cannot emit its contract.

    Python source imports are useful for unit tests, but an installed wrapper
    must not silently import an unrelated source/legacy package at runtime.
    ``package_prefix`` is the actual ament-selected localization installation,
    not the developer checkout and not merely the enclosing release root.
    """
    if config.get('localization_pipeline',{}).get('ros__parameters',{}).get('backend')!='lio_pcd':
        raise ValueError('single_floor_requires_atomic_lio_navigation_output')
    pipeline=config.get('navigation_estimation',{}).get('ros__parameters',{})
    if pipeline.get('enabled') is not True:
        raise ValueError('single_floor_requires_enabled_continuous_navigation_output')
    if package_prefix is not None:
        prefix=Path(package_prefix).resolve(strict=True)
        for module in ('global_lio_localizer','realtime_navigation_output','navigation_output','navigation_wire'):
            name='d1max_localization.'+module
            spec=importlib.util.find_spec(name)
            origin=Path(spec.origin).resolve(strict=True) if spec is not None and spec.origin else None
            if origin is None or prefix not in origin.parents:
                raise ValueError('localization_python_outside_selected_install:'+name)
    # This is the same resolver used by global_localization.launch.py. A true
    # flag alone is insufficient: EKF inputs, frames and prediction bounds must
    # form the installed producer's executable contract before any ROS starts.
    from d1max_localization.estimation.configuration import navigation_parameters
    if navigation_parameters(config) is None:
        raise ValueError('single_floor_requires_enabled_continuous_navigation_output')


def verify(directory, *, seal_runtime=False):
    from .interface_preflight import validate_execution_interfaces
    from .navigation_runtime_policy import validate_session_policy
    directory=Path(directory).resolve(strict=True)
    s=json.loads((directory/'session.json').read_text())
    if s.get('pipeline_contract')!='single_floor_v3' or s.get('interface_version')!=3:
        raise ValueError('not_a_v3_single_floor_session')
    if s.get('handoff_contract')!='local_executor_v1' or s.get('execution_architecture')!='task_follower_v1':
        raise ValueError('local_executor_task_writer_contract_required')
    if s.get('initial_pose_transaction')!='bt_initial_pose_transaction_v1':
        raise ValueError('initial_pose_transaction_contract_required')
    if s.get('component_health_contract')!='functional_liveness_v1':
        raise ValueError('component_health_contract_required')
    for path,digest in s['input_hashes'].items():
        if sha(path)!=digest: raise ValueError('changed_input:'+path)
    for name,digest in s['parameter_hashes'].items():
        if sha(directory/name)!=digest: raise ValueError('changed_parameter:'+name)
    from .navigation_map_package import validate_package,package_version
    map_package=validate_package(Path(s['planning_manifest']).parent,
                                 purpose=s.get('purpose','execution'))
    if (str(Path(s['tomogram_npz']).resolve())!=str(map_package['tomogram_path'].resolve())
            or str(Path(s['map_pcd']).resolve())!=str(Path(map_package['manifest']['source_path']).resolve())
            or s.get('map_capabilities',map_package['capabilities'])!=map_package['capabilities']):
        raise ValueError('navigation_session_map_capability_or_artifact_mismatch')
    if map_package['capabilities']['geometry_kind']=='conditioned_multifloor_ground':
        if (s.get('version_id')!=package_version(map_package)
                or s.get('map_runtime_dependencies')!=map_package['dependency_files']
                or s.get('map_rebuild_input_references')!=map_package['rebuild_input_references']
                or any(s['input_hashes'].get(p)!=digest
                       for p,digest in map_package['dependency_files'].items())):
            raise ValueError('conditioned_multifloor_session_dependencies_not_sealed')
    validate_session_policy(directory,s)
    validate_planning_only_parameters(directory,s)
    motion=not planning_only_profile(s)
    # One startup-only coordinate audit, using the exact files children will
    # read. It does not introduce a second live-state or collision gate.
    from .navigation_contract import mainline_coordinate_contract
    coordinate_names=('bt','bt_adapter','global','reference','scan','tracker')
    coordinates=mainline_coordinate_contract(
        yaml.safe_load((directory/'localization.yaml').read_text()),s,
        {name:yaml.safe_load((directory/(name+'.yaml')).read_text())['/**']['ros__parameters']
         for name in coordinate_names}, json.loads(Path(s['planning_manifest']).read_text()))
    # Search/trajectory geometry and actual motion evidence have different
    # domains. Freeze their agreement end-to-end; an old strict consumer must
    # not silently reject all official-model candidates at runtime.
    if motion:
        validate_collision_policy_parameters(directory,s)
        validate_tracking_parameters(directory,s)
    handoff_parameters=[('scan.yaml','fsm.writer_handoff_enabled')]
    handoff_parameters.append(('tracker.yaml','writer_handoff_enabled'))
    for name,key in handoff_parameters:
        value=yaml.safe_load((directory/name).read_text())['/**']['ros__parameters'].get(key)
        if value is not False:raise ValueError('legacy_writer_handoff_must_be_disabled:'+name)
    if (yaml.safe_load((directory/'scan.yaml').read_text())['/**']['ros__parameters'].get('fsm.local_follower_enabled') is not True
            or yaml.safe_load((directory/'reference.yaml').read_text())['/**']['ros__parameters'].get('geometry_owner')!='local_executor'
            or yaml.safe_load((directory/'bt.yaml').read_text())['/**']['ros__parameters'].get('execution_architecture')!='task_follower_v1'
            or (directory/'safety.yaml').exists()):
        raise ValueError('mixed_local_execution_owners')
    types=validate_execution_interfaces()
    release=paths.release_root()
    dependencies=paths.localization_dependencies_root()
    if dependencies is not None:
        from ament_index_python.packages import get_package_prefix
        for package in ('faster_lio','livox_ros_driver2'):
            if Path(get_package_prefix(package)).resolve(strict=True)!=dependencies/package:
                raise ValueError('unreleased_localization_dependency:'+package)
    commands=runtime_commands(directory,s)
    files={}
    from d1max_pct_planner.runtime_freeze import verify_runtime_inputs,contract_files,scientific_loader_dependencies
    input_contract=Path(s['runtime_inputs_contract']).resolve(strict=True)
    if input_contract!=directory/'runtime_inputs.json':
        raise ValueError('runtime_inputs_contract_outside_session')
    runtime_inputs=verify_runtime_inputs(json.loads(input_contract.read_text()))
    files.update(contract_files(runtime_inputs))
    scientific_loader=scientific_loader_dependencies(runtime_inputs['scientific'])
    files.update(scientific_loader)
    if dependencies is not None:
        for path in dependencies.rglob('*'):
            if path.is_file() and '__pycache__' not in path.parts:
                files[str(path.resolve())]=sha(path)
        provenance=dependencies.parent/'runtime_copy_provenance.json'
        files[str(provenance.resolve(strict=True))]=sha(provenance)
    for name,cmd in commands.items():
        if name=='localization':
            from ament_index_python.packages import get_package_prefix,get_package_share_directory
            for exe in ('dual_lidar_adapter','fused_icp_matcher','global_lio_localizer','realtime_navigation_output'):
                p=Path(get_package_prefix('d1max_localization'))/'lib/d1max_localization'/exe
                if release not in p.resolve().parents:
                    raise ValueError('unreleased_localization_binary:'+str(p))
                files[str(p.resolve())]=sha(p)
            launch=Path(get_package_share_directory('d1max_localization'))/'launch/global_localization.launch.py'
            files[str(launch.resolve())]=sha(launch)
            for package,exe in (('faster_lio','run_mapping_online'),('robot_localization','ekf_node')):
                p=Path(get_package_prefix(package))/'lib'/package/exe
                files[str(p.resolve())]=sha(p)
            config=yaml.safe_load((directory/'localization.yaml').read_text())
            validate_live_localization_parameters(config,package_prefix=get_package_prefix('d1max_localization'))
            continue
        if '-m' in cmd:
            module=cmd[cmd.index('-m')+1]
            origin=importlib.util.find_spec(module).origin
            if not origin: raise ValueError('unresolved_module:'+module)
            # Dependencies within this package matter as much as main.py.
            for p in python_runtime_members(Path(origin).parent): files[str(p)]=sha(p)
        else:
            p=Path(cmd[0]).resolve(strict=True)
            if name!='lifecycle' and release not in p.parents:
                raise ValueError('old_or_production_binary_in_v3_bundle:'+str(p))
            files[str(p)]=sha(p)
    for module in ('d1max_planning_interfaces','d1max_navigation_bt_interfaces'):
        root=Path(importlib.util.find_spec(module).origin).parent
        for p in root.rglob('*'):
            if p.suffix in ('.py','.so'): files[str(p.resolve())]=sha(p)
    for module in ('d1max_pct_planner','d1max_localization'):
        root=Path(importlib.util.find_spec(module).origin).parent
        for p in root.rglob('*.py'):files[str(p.resolve())]=sha(p)
    # Shared libraries can change independently of their launch executables.
    # ldd is read-only; only inspect the selected local release binaries.
    for path in tuple(files):
        with Path(path).open('rb') as stream: is_elf=stream.read(4)==b'\x7fELF'
        if not is_elf or path.endswith('.so'):continue
        result=subprocess.run(['ldd',path],text=True,capture_output=True,timeout=10)
        if result.returncode or 'not found' in result.stdout:
            raise ValueError('unresolved_runtime_library:'+path)
        for line in result.stdout.splitlines():
            target=line.split('=>',1)[-1].strip().split(' ',1)[0]
            if target.startswith('/'):
                dependency=Path(target).resolve(strict=True)
                files[str(dependency)]=sha(dependency)
    from d1max_pct_planner.native_runtime import vendorverify
    route=yaml.safe_load(Path(s['crossfloor_route_config']).read_text())
    native=vendorverify(route['vendor_root'])
    dependencies=[native['gtsam_library'],native['metis_library'],
                  *native['native_libraries'].values(),*native['build_evidence'].values()]
    dependencies.extend(Path(native['native_lib_dir']).glob('*.so'))
    for p in dependencies:files[str(Path(p).resolve())]=sha(p)
    manifest=dict(schema=3,session_sha256=sha(directory/'session.json'),types=types,runtime_inputs=runtime_inputs,
        scientific_loader_dependencies=scientific_loader,coordinate_contract=coordinates,
        files=files,commands=commands)
    seal=directory/'runtime_bundle.json'
    if seal_runtime:
        if seal.exists(): raise ValueError('runtime_already_sealed_prepare_a_new_session')
        seal.write_text(json.dumps(manifest,indent=2)+'\n')
    elif not seal.is_file() or json.loads(seal.read_text())!=manifest:
        raise ValueError('runtime_not_sealed_or_changed_prepare_a_new_session')
    return s,commands


def verify_view(directory):
    """Check display inputs, not rebuild the already sealed execution graph.

    The official entry checks the release before loading its setup hooks and
    installed code. The viewer additionally binds this session and its exact
    generated files. It neither discovers algorithm/ELF dependencies again nor
    changes readiness, execution permission or the core's verification.
    """
    directory=Path(directory).resolve(strict=True)
    session_path=directory/'session.json'
    session=json.loads(session_path.read_text())
    sealed=json.loads((directory/'runtime_bundle.json').read_text())
    if (sealed.get('schema')!=3 or sealed.get('session_sha256')!=sha(session_path)
            or session.get('pipeline_contract')!='single_floor_v3'
            or session.get('interface_version')!=3
            or session.get('view_contract')!='single_window_layout_v1'):
        raise ValueError('view_session_not_sealed_or_changed')
    for name,digest in session['parameter_hashes'].items():
        relative=Path(name)
        if relative.name!=name or not name or sha(directory/relative)!=digest:
            raise ValueError('changed_view_parameter:'+name)
    if 'global_planning.rviz' not in session['parameter_hashes']:
        raise ValueError('view_layout_not_sealed')
    for path,digest in session['input_hashes'].items():
        if not Path(path).is_absolute() or sha(path)!=digest:
            raise ValueError('changed_view_input:'+path)
    release=paths.release_root().resolve(strict=True)
    for module,component in (('d1max_pct_scan','application'),('d1max_pct_planner','application'),
                             ('d1max_planning_interfaces','interfaces'),
                             ('d1max_navigation_bt_interfaces','interfaces')):
        origin=Path(importlib.util.find_spec(module).origin).resolve(strict=True)
        if release/component/'install' not in origin.parents:
            raise ValueError('unreleased_view_module:'+module)
        actual=python_runtime_members(origin.parent)
        expected={Path(p) for p in sealed['files'] if Path(p).is_relative_to(origin.parent)
                  and Path(p).suffix in ('.py','.so')}
        if actual!=expected:raise ValueError('changed_view_module_inventory:'+module)
        for path in actual:
            resolved=path.resolve(strict=True)
            if sealed['files'].get(str(resolved))!=sha(resolved):
                raise ValueError('changed_view_module:'+str(resolved))
    from .interface_preflight import validate_execution_interfaces
    if validate_execution_interfaces()!=sealed['types']:
        raise ValueError('view_interface_contract_changed')
    return session


@contextmanager
def runtime_lock(directory):
    """One core for the fixed robot namespace/domain, not just one directory.

    SDK session IDs are not lock keys: reconnecting the same physical robot
    must never create a second owner of the existing namespace. The SDK also
    retains its independent process-wide writer lease.
    """
    root=_core_lock_root()
    domain=int(os.environ.get('ROS_DOMAIN_ID','24'))
    if domain<0:raise ValueError('invalid_navigation_domain')
    with _owned_file_lock(root/('core-domain-'+str(domain)+'.lock'),'navigation_core_already_running') as core_fd:
        with _owned_file_lock(Path(directory)/'.runtime.lock','single_floor_session_already_running'):
            yield core_fd


def _core_lock_root():
    import stat
    # Match the SDK's established /run/user inode family; XDG overrides cannot
    # bypass ownership by creating a different lock directory.
    root=Path('/run/user')/str(os.getuid())/'d1max-navigation'
    root.mkdir(mode=0o700,exist_ok=True)
    info=root.lstat()
    if not stat.S_ISDIR(info.st_mode) or info.st_uid!=os.getuid() or info.st_mode&0o077:
        raise ValueError('navigation_core_lock_directory_not_private')
    return root


def run(directory):
    directory=Path(directory).resolve(strict=True)
    session=json.loads((directory/'session.json').read_text())
    identifier=session['id']
    _startup_report(directory,identifier,'starting',False)
    try:
        with runtime_lock(directory) as core_fd:
            _run_locked(directory,core_fd=core_fd)
    except Exception as error:
        # Report pre-graph failure separately from failures after a child may
        # have started. Only the former can be retired without a task drain.
        attempted=True
        try:
            last=json.loads((directory/'core_startup.json').read_text())
            if last.get('session_id')==identifier:attempted=last.get('children_started') is not False
        except (OSError,ValueError,TypeError):pass
        _startup_report(directory,identifier,'failed',attempted,error=str(error)[:1024])
        raise


def _startup_report(directory,identifier,phase,children_started,*,error=''):
    from .live_view_reload import write_owned_json
    write_owned_json(Path(directory)/'core_startup.json',dict(schema=1,session_id=identifier,
        phase=phase,children_started=children_started,error=error,recorded_at_unix=time.time()))


def _run_locked(directory,core_fd=None):
    """Critical exits stop the graph; never starts a physical SDK connection."""
    from .isolated_zenoh import validate_environment
    from .live_runtime import stop_owned_children
    directory=Path(directory).resolve(strict=True)
    if os.environ.get('RMW_IMPLEMENTATION')!='rmw_zenoh_cpp':
        raise ValueError('unchanged_zenoh_required')
    s,commands=verify(directory)
    if s['transport_mode']=='isolated_mock': validate_environment()
    else:
        if not planning_only_profile(s):
            check_acceptance(s.get('runtime_motion_profile') or s['physical_acceptance_record'])
        if os.environ.get('D1MAX_NAV_TRANSPORT')=='isolated_mock':
            raise ValueError('live_session_cannot_use_mock_transport')
    stopping=False
    def signal_stop(_signum,_frame):
        nonlocal stopping
        stopping=True
    previous={sig:signal.signal(sig,signal_stop) for sig in (signal.SIGINT,signal.SIGTERM)}
    children=[]; logs=[]
    try:
        from .diagnostic_session import trace_environment
        trace_env=trace_environment(directory,s,commands)
        # Conservative before Popen, so even a crash immediately after spawn
        # cannot be mistaken for a failure with no running work.
        _startup_report(directory,s['id'],'spawning',True)
        from .runtime_cpu import component_command
        for name,cmd in commands.items():
            log=(directory/(name+'.log')).open('w'); logs.append(log)
            p=subprocess.Popen(component_command(name,cmd),stdout=log,stderr=subprocess.STDOUT,start_new_session=True,
                pass_fds=() if core_fd is None else (core_fd,),env=trace_env)
            children.append((name,p))
        # Creation of the last process must not conceal an earlier component
        # that already exited during the spawn loop. This ACK means that the
        # children were spawned and are alive, not functional/sensor readiness.
        exited=exited_critical_components(children)
        if exited:raise RuntimeError('critical_component_exited:'+repr(exited))
        from .shared_remote_launch import command as shared_remote_command
        remote_command=None if s.get('planning_scope')=='global_route' else shared_remote_command(s)
        if remote_command:
            try:
                remote_log=(directory/'manual_remote.log').open('w');logs.append(remote_log)
                remote_process=subprocess.Popen(remote_command,stdout=remote_log,stderr=subprocess.STDOUT,
                    start_new_session=True)
                children.append(('manual_remote',remote_process))
            except OSError:
                # Desktop availability is not a robot/core health fact.
                # The user can reopen the same panel through the existing Web button.
                pass
        _startup_report(directory,s['id'],'started',True)
        reported_display_exit=False
        from .component_health import SupervisorHealth
        from .navigation_runtime_policy import session_policy
        policy=session_policy(s)
        supervisor=SupervisorHealth(s['id'],started=time.monotonic(),
            startup_s=s.get('component_startup_timeout_s',30.),
            # Functional monitor loss first blocks execution at the BT. Only
            # persistent loss tears down the graph; real child exits above
            # remain immediate faults. No old report renews this interval.
            timeout_s=policy.recovery_timeout_s if policy is not None else 1.5)
        while not stopping:
            exited=exited_critical_components(children)
            if exited: raise RuntimeError('critical_component_exited:'+repr(exited))
            if s.get('component_health_contract')=='functional_liveness_v1':
                value=None
                try:
                    path=directory/'component_health.json'
                    if path.stat().st_size<=16384:value=json.loads(path.read_text())
                except (OSError,ValueError):pass
                fault=supervisor.check(value,monotonic=time.monotonic())
                if fault:raise RuntimeError(fault)
            for name,process in children:
                if name=='route_display_cache' and process.poll() is not None and not reported_display_exit:
                    reported_display_exit=True
                    (directory/'display_fault.json').write_text(json.dumps(dict(component=name,
                        returncode=process.returncode,recorded_at_unix=time.time(),
                        reason='historical_display_unavailable_execution_owner_unchanged'))+'\n')
            time.sleep(.1)
    finally:
        from .lifecycle_shutdown import drain_task_owner
        navigator_started=any(name=='navigator' for name,_ in children)
        navigator_alive=any(name=='navigator' and p.poll() is None for name,p in children)
        drain=dict(request_accepted=False,software_retired=False,physical_stop_confirmed=False,
            reason='task_owner_not_alive' if navigator_started else 'task_owner_never_started')
        if navigator_alive:
            # Navigator drains while perception, control and SDK still run.
            try:drain=drain_task_owner(s['id'])
            except Exception as error:
                drain['reason']='task_owner_drain_error:'+type(error).__name__
        try:
            # One set of shared phase deadlines, not 3+3 seconds PER child.
            # The helper also checks/retires recorded new-session groups whose
            # direct parent exited but left children behind. Together with the
            # six-second task drain, this stays inside systemd's 30-second cap.
            cleanup=stop_owned_children(children,interrupt_s=6.,terminate_s=1.,kill_s=1.)
        except Exception as error:
            cleanup=dict(all_direct_children_exited=False,all_owned_groups_exited=False,
                reason='owned_process_cleanup_error:'+type(error).__name__)
        try:
            from .live_view_reload import write_owned_json
            drain.update(session_id=s['id'],recorded_at_unix=time.time(),
                navigator_started=navigator_started,navigator_alive_at_drain=navigator_alive,
                process_cleanup=cleanup)
            # Publish only after actual group cleanup, atomically. A disk error
            # cannot skip process cleanup or turn a kill into physical stopping.
            write_owned_json(directory/'shutdown.json',drain)
        finally:
            release_supervisor_resources(logs,previous)


def component_is_critical(name):
    # Display-only history is hosted alongside the core for UI independence;
    # its failure is diagnosable but must not cancel an authorized task.
    return name not in ('route_display_cache','manual_remote')


def exited_critical_components(children):
    return [(name,p.returncode) for name,p in children
        if p.poll() is not None and component_is_critical(name)]


def default_session_purpose():
    """CLI release default only; the source prepare API remains execution."""
    try:release=paths.release_root()
    except ValueError:return 'execution'  # source-only development has no selected release
    _,descriptor=paths.release_descriptor(release)
    value=descriptor.get('default_session_purpose','execution')
    if value not in ('execution','planning_only'):
        raise ValueError('invalid_release_default_session_purpose')
    return value


def main():
    p=argparse.ArgumentParser()
    p.add_argument('action',choices=('prepare','seal','verify','run','view'))
    p.add_argument('--session',type=Path,required=True)
    p.add_argument('--map-directory',type=Path,help='map package; default: the release descriptor')
    p.add_argument('--transport-mode',choices=('isolated_mock','live'),default='isolated_mock')
    p.add_argument('--expected-sdk-session',default='')
    p.add_argument('--motion-profile','--acceptance-record',dest='acceptance_record',default='')
    p.add_argument('--purpose',choices=('execution','planning_only'),default=None)
    p.add_argument('--layout',choices=('global','local'),default='global')
    p.add_argument('--seal',action='store_true',help='seal a prepared session in the same process; starts no nodes')
    a=p.parse_args()
    if a.seal and a.action!='prepare':p.error('--seal is only valid with prepare')
    if a.action=='prepare':
        s=prepare(a.session,map_directory=a.map_directory,transport_mode=a.transport_mode,
            expected_sdk_session=a.expected_sdk_session,acceptance_record=a.acceptance_record,
            purpose=a.purpose if a.purpose is not None else default_session_purpose())
        if a.seal:verify(a.session,seal_runtime=True)
        print(json.dumps(dict(session_id=s['id'],prepared_only=True,sealed=a.seal,started_processes=False)))
    elif a.action in ('seal','verify'):
        s,c=verify(a.session,seal_runtime=a.action=='seal')
        print(json.dumps(dict(session_id=s['id'],verified_components=list(c),started_processes=False)))
    elif a.action=='view':view(a.session,a.layout)
    else: run(a.session)


if __name__=='__main__': main()
