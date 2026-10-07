#!/usr/bin/env python3
"""Prepare the existing navigation session for an isolated measured PhysX plant.

All simulation changes occur before existing runtime verification/sealing. The
actual BT, global/native local planners, tracker and safety contracts are kept.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import time
import yaml


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def validate_prior_body_domain(spec, resolution, exclusion=.10):
    """This flat fixture cannot certify low solids below its collision slice.

    A floor-contact voxel must not be cleared to accommodate wheels. Restrict
    the scene instead until a separate support/contact contract is implemented.
    One voxel above the exclusion bounds discrete rounding and measured pose.
    """
    if spec.get('robot', {}).get('kind') == 'official_spot_physx' and spec.get('flat_support_contact', {}).get('enabled'):
        return  # Explicit contact cells retain non-floor solids; no floor slice is omitted.
    minimum_top=float(spec['floor']['z'])+exclusion+resolution
    for box in spec['static_boxes']:
        if float(box['center'][2])+float(box['size'][2])/2. < minimum_top:
            raise ValueError('static_prior_unsupported_low_collider:'+box['name'])


def verify_candidate(release):
    release = Path(release).resolve(strict=True)
    descriptor = json.loads((release/'release.json').read_text())
    if descriptor.get('candidate_kind') != 'isolated_isaac_fixture' or descriptor.get('physical_acceptance') is not False:
        raise ValueError('explicit_isolated_isaac_candidate_required')
    manifest = json.loads((release/'isaac_candidate_integrity.json').read_text())
    if manifest.get('kind') != 'isaac_fixture_integrity_not_production_release_seal':
        raise ValueError('not_an_isaac_candidate_integrity_record')
    for filename, expected in manifest['files'].items():
        path = Path(filename).resolve(strict=True)
        if not path.is_relative_to(release) or sha(path) != expected:
            raise ValueError('changed_candidate_artifact:'+filename)
    return descriptor


def prepare_isaac_session(output, release, map_directory=None, session_id=None, seal=True):
    from d1max_pct_scan.navigation_session import prepare, verify
    release = Path(release).resolve(strict=True)
    descriptor = verify_candidate(release)
    if Path(os.environ.get('D1MAX_RELEASE', '')).resolve() != release:
        raise ValueError('source_candidate_overlay_and_set_D1MAX_RELEASE_before_prepare')
    directory = Path(output).resolve()
    config_path = release/descriptor['simulation_config']
    spec = json.loads(config_path.read_text())
    robot = spec['robot']
    session = prepare(directory, map_directory=map_directory, transport_mode='isolated_mock', session_id=session_id)
    from truth_map import StaticPrior
    prior_path = release/descriptor['static_collision_prior_manifest']
    prior = StaticPrior(prior_path).manifest
    validate_prior_body_domain(spec,prior['voxel_resolution'])
    if (prior['map_version'] != session['version_id'] or prior['frame_id'] != session['frame_id']
            or prior['spec_sha256'] != sha(config_path)
            or prior['map_from_odom'] != dict(transform_contract='fixed_identity_map_from_odom_v1',
                from_frame='d1max_loc_odom',to_frame='d1max_loc_map',
                translation=[0.,0.,0.],rotation_xyzw=[0.,0.,0.,1.])):
        raise ValueError('static_prior_source_or_coordinate_contract_mismatch')
    prior_params = dict(collision_evidence_mode='validated_static_prior',
        static_prior_manifest_sha256=sha(prior_path),
        static_prior_geometry_sha256=prior['collider_sha256'],
        static_prior_map_frame=prior['frame_id'],
        static_prior_transform_contract='fixed_identity_map_from_odom_v1')
    height = float(robot['body_reference_height'])
    body_length, body_width, body_vertical = map(float, robot['body_size'])
    quadruped = robot.get('kind') in ('official_go2_physx', 'official_spot_physx')
    if quadruped:
        # Command limits leave room for the measured oscillating gait. The
        # reachable model bounds actual velocity, separately from demands.
        session.update(max_speed_mps=.15, max_yaw_radps=.30)
        braking_path = Path(session['execution_braking_model_record'])
        braking = json.loads(braking_path.read_text())
        braking['note'] = ('Conservative Spot simulation fixture bounds; sampled official-policy tests, '
                           'not a real robot calibration or continuous contact proof.')
        braking['measurements'].update(max_speed_mps=.30, max_yaw_radps=.50,
            stop_latency_bound_s=3., stopping_distance_m=.50, stopping_yaw_rad=.40,
            tracking_error_bound_m=.10, heading_error_bound_rad=.10)
        if robot.get('kind') == 'official_spot_physx':
            # An articulated plant's instantaneous reachable velocity is
            # distinct from its permitted navigation command. This opt-in
            # record is accepted only by the isolated simulation transport.
            limits = robot['model_limits']
            reachable_speed = float(limits['max_linear_speed_mps'])
            reachable_yaw = float(limits['max_angular_speed_radps'])
            if not (0.15 <= reachable_speed <= .6 and .30 <= reachable_yaw <= .8):
                raise ValueError('invalid_spot_isolated_reachable_bounds')
            braking['measurements'].update(max_speed_mps=reachable_speed, max_yaw_radps=reachable_yaw)
            braking['isolated_platform_model'] = dict(schema=1, kind='official_spot_physx',
                command_max_speed_mps=session['max_speed_mps'],
                command_max_yaw_radps=session['max_yaw_radps'],
                reachable_max_speed_mps=reachable_speed, reachable_max_yaw_radps=reachable_yaw,
                source_scope='isolated_simulation_physx_measured_model')
            reference = robot.get('full_xyz_reference_model')
            if reference is not None:
                evidence_path = (release/'simulation'/reference['evidence_file']).resolve(strict=True)
                if not evidence_path.is_relative_to(release/'simulation') or sha(evidence_path) != reference['evidence_sha256']:
                    raise ValueError('spot_full_xyz_reference_evidence_changed')
                evidence = json.loads(evidence_path.read_text())
                reference_cap = float(reference['reference_max_speed_mps'])
                travel_cap = float(reference['measured_travel_max_speed_mps'])
                observed = float(evidence['observed_max_full_xyz_speed_mps'])
                if (evidence.get('kind') != 'isolated_spot_full_xyz_reference_admission_domain_v1'
                        or evidence.get('physical_acceptance') is not False
                        or not 0 < observed <= min(reference_cap, travel_cap) <= max(reference_cap, travel_cap) <= .50
                        or evidence['reference_max_speed_mps'] != reference_cap
                        or evidence['measured_travel_max_speed_mps'] != travel_cap):
                    raise ValueError('invalid_spot_full_xyz_reference_evidence')
                braking['isolated_full_xyz_reference_model'] = dict(schema=1, kind='official_spot_physx',
                    reference_max_speed_mps=reference_cap, measured_travel_max_speed_mps=travel_cap,
                    observed_max_full_xyz_speed_mps=observed, evidence_sha256=reference['evidence_sha256'],
                    source_scope='isolated_simulation_physx_measured_model')
        braking_path.write_text(json.dumps(braking, indent=2)+'\n')
        session['execution_braking_model_sha256'] = sha(braking_path)
        session['input_hashes'][str(braking_path)] = sha(braking_path)
    envelope = robot.get('navigation_envelope', {})
    if quadruped:
        body_length = max(body_length, float(envelope.get('length_m', 1.1)))
        width = max(body_width, float(envelope.get('width_m', .8)))
    else:
        width = max(body_width, float(robot['wheel_base'])+float(robot['wheel_width']))
    # Circumscribe the fixture chassis rectangle with two circles. Checking
    # width and length extents alone misses corners, so radius is sqrt(dx²+dy²).
    offset = body_length/4.
    radius = (offset**2+(width/2.)**2)**.5+(.06 if quadruped else .03)
    length = 2*(offset+radius)
    above_base = float(envelope.get('above_body_m', .55)) if quadruped else max(
        body_vertical/2., float(robot['wheel_axle_z'])+float(robot['wheel_radius']))+.05
    profile = dict(schema=1, model=robot.get('kind') if quadruped else 'Isaac Sim differential drive fixture', fixture_only=True,
        physical_acceptance=False, geometry_source=str(config_path), geometry_sha256=sha(config_path),
        body_reference_height_m=height, actual_body_size_m=robot['body_size'], actual_total_width_m=width,
        double_cylinder_radius_m=radius, double_cylinder_offset_m=offset,
        native_envelope_length_m=length, native_envelope_width_m=2*radius,
        obstacle_dilation_up_m=height-.10, obstacle_dilation_down_m=above_base,
        max_speed_mps=session['max_speed_mps'], max_yaw_radps=session['max_yaw_radps'], ground_exclusion_height_m=.10,
        note='Known simulation geometry only; never authorizes the D1 Max SDK or physical robot.')
    if not .25 <= height <= .85 or radius < width/2.:
        raise ValueError('wheel_fixture_geometry_outside_supported_floor_profile')
    profile_path = directory/('quadruped_fixture_profile.yaml' if quadruped else 'wheel_fixture_profile.yaml')
    profile_path.write_text(yaml.safe_dump(profile, sort_keys=False))
    session.update(simulation_backend='isaacsim_physx', simulation_clock='isaac_fixed_anchor_v1',
        perception_acquisition='isaac_physx_snapshot_v1',
        robot_profile=str(profile_path), body_height=height,
        body_height_calibration_id='isaac_geometry_sha256:'+sha(config_path),
        body_height_calibration_status='known_simulation_geometry_not_physical_calibration',
        isaac_bridge_contract=dict(schema=1, clock_anchor_ns=time.time_ns(),
            source_clock='isaac_physics_time', reset_policy='revoke_require_new_session',
            pause_policy='revoke_require_new_session', localization='groundtruth_fixture',
            command_source='/d1max/live_planning/execution/applied_motion',
            state_port=18741, command_port=18742, measured_velocity=True,
            scene_config=str(config_path), scene_sha256=sha(config_path)),
        simulation_initial_pose=robot['initial_pose'],
        simulation_goal=[robot['goal'][0],robot['goal'][1],spec['floor']['z']],
        physical_acceptance=False, motion_control_enabled=False)
    session['input_hashes'].update({str(profile_path):sha(profile_path), str(config_path):sha(config_path)})
    session['static_collision_prior_contract'] = dict(schema=1, **prior_params,
        manifest_path=str(prior_path),map_version=session['version_id'],
        data_sha256=prior['data_sha256'],voxel_resolution=prior['voxel_resolution'],
        max_body_tilt_rad=.35 if quadruped else .01,source_scope=prior['source_scope'],
        expected_body_world_z=float(spec['floor']['z'])+float(robot['body_reference_height']),
        max_body_height_error_m=.12 if quadruped else .005,
        runtime_geometry_attestation_required=True)
    if quadruped:
        from dynamic_collision import actor_registry, registry_digest, canonical
        session['perception_native_phase_contract'] = dict(schema=1,
            phase='physx_prephysics_capture_v1', physics_dt_ns=round(1e9/spec['physics']['frequency_hz']))
        registry = actor_registry(spec)
        if not spec.get('flat_support_contact', {}).get('enabled'):
            raise ValueError('quadruped_requires_sealed_flat_support_and_dynamic_collision_contract')
        dynamic_topic = '/d1max/localization/perception/dynamic_occupancy'
        dynamic_params = dict(dynamic_oracle_registry_sha256=registry_digest(registry),
            dynamic_oracle_actor_ids=[a['id'] for a in registry], dynamic_oracle_topic=dynamic_topic,
            projected_ray_exact_pair=True, dynamic_hit_provenance_enabled=True,
            static_prior_support_contact_enabled=True,
            static_prior_support_floor_z=float(spec['floor']['z']), static_prior_support_penetration_m=.02,
            static_prior_support_floor_endpoint_error_bound_m=float(spec['flat_support_contact']['floor_endpoint_error_bound_m']))
        if not registry:
            dynamic_params.pop('dynamic_oracle_actor_ids')
        session['dynamic_oracle_contract'] = dict(schema=1, registry_sha256=registry_digest(registry),
            actor_ids=[a['id'] for a in registry], topic=dynamic_topic,
            source_scope='isolated_same_step_PhysX_actor_truth_not_lidar',
            maximum_source_and_receipt_age_s=.20, maximum_reachable_horizon_s=.30)
        session['dynamic_oracle_contract']['reachable_horizon_ns'] = 6_000_000_000
        session['dynamic_oracle_contract']['maximum_reachable_horizon_s'] = 6.
        session['dynamic_oracle_contract']['message_source_lifetime_s'] = .30
        session['dynamic_oracle_contract']['native_hit_provenance'] = dict(schema=1,
            kind='physx_exact_hit_prim_ordinal_v1', point_step=72, registry_sha256=registry_digest(registry))
        session['static_collision_prior_contract'].update(body_envelope_attestation_required=True,
            body_envelope_registry_sha256=hashlib.sha256(canonical(spec['robot_collision_registry'])).hexdigest(),
            body_envelope=dict(radius=radius, offset=offset, above=above_base,
                support_floor_z=float(spec['floor']['z']), support_penetration_m=.02),
            full_leg_volume_required=True, floor_contact_scope='certified_flat_plane_only',
            floor_endpoint_error_bound_m=float(spec['flat_support_contact']['floor_endpoint_error_bound_m']))
    for artifact in (prior_path, prior_path.parent/prior['data_file'],
            release/'simulation/truth_map.py',release/descriptor.get('simulation_collision_stage',
                'simulation/assets/indoor_scene.usda')):
        session['input_hashes'][str(artifact)]=sha(artifact)
    session['isaac_bridge_contract'].update(lidar_origins_body=spec['lidar']['origins'],
        raw_cloud_topics=['/front_lidar','/rear_lidar'], raw_cloud_coordinate_system='actual_sensor_frame',
        raw_cloud_fields='xyz_actual_PhysX_ring_0_to_32_and_original_timestamp_no_unmeasured_intensity',
        native_imu=True, imu_frame='d1max_loc_base_link', imu_acceleration_units='m/s^2_specific_force',
        imu_primary_topic='/d1max/localization/imu', imu_acceleration_scale=1.,
        imu_source_identity='native_PhysX_IMUSensor_not_body_state_derivative')
    updates = {
        'scan.yaml': {'grid_map.double_cylinder_radius':radius, 'grid_map.double_cylinder_offset':offset,
            'grid_map.body_height':height, 'grid_map.obstacles_inflation_z_up':height-.10,
            'grid_map.obstacles_inflation_z_down':above_base,
            'grid_map.simulation_clock_contract':'isaac_fixed_anchor_v1',
            'grid_map.transport_mode':'isolated_mock',
            'use_sim_time':True},
        'reference.yaml': {'body_height_m':height,
            'body_height_calibration_id':session['body_height_calibration_id'],**prior_params},
        'bt_adapter.yaml': {'body_height':height},
    }
    updates['scan.yaml'].update({'grid_map.'+k:v for k,v in prior_params.items()})
    updates['scan.yaml'].update({'grid_map.static_prior_manifest_path':str(prior_path),
        'grid_map.static_prior_map_version_id':session['version_id']})
    if quadruped:
        updates['scan.yaml'].update({'grid_map.'+k:v for k,v in dynamic_params.items()})
        updates['scan.yaml'].update({'manager.max_vel':session['max_speed_mps'],
            'manager.fit_low_speed_entry_velocity':True,
            'optimization.max_vel':session['max_speed_mps']})
        updates['tracker.yaml'] = dict(max_speed=session['max_speed_mps'], max_yaw_rate=session['max_yaw_radps'],
            spatial_planar_braking_envelope=True, spatial_control_lookahead=True,
            execution_braking_model_record=session['execution_braking_model_record'])
        updates['safety.yaml'] = dict(max_speed=session['max_speed_mps'], max_yaw=session['max_yaw_radps'],
            maximum_isaac_actor_id=len(registry))
        # Carry the exact geometric certificate to the plant before sealing.
        session['isaac_bridge_contract']['body_envelope'] = session['static_collision_prior_contract']['body_envelope']
    for path in directory.glob('*.yaml'):
        if path.name in ('wheel_fixture_profile.yaml', 'quadruped_fixture_profile.yaml'):
            continue
        value = yaml.safe_load(path.read_text())
        if '/**' in value and 'ros__parameters' in value['/**']:
            value['/**']['ros__parameters'].update(use_sim_time=True)
            value['/**']['ros__parameters'].update(updates.get(path.name, {}))
            if quadruped:
                params = value['/**']['ros__parameters']
                for key in list(params):
                    if key.endswith('execution_braking_model_sha256'):
                        params[key] = session['execution_braking_model_sha256']
            path.write_text(yaml.safe_dump(value, sort_keys=True))
    # Ground-truth body/tracking/ray frames coincide; individual measured sensor
    # origins are retained per ray. Do not reuse D1 Max extrinsics for this wheel.
    localization_path = directory/'localization.yaml'
    localization = yaml.safe_load(localization_path.read_text())
    localization['lio_localizer']['ros__parameters'].update(tracking_offset_body=[0.,0.,0.], sdk_to_tracking_yaw=0.)
    front_origin,rear_origin=spec['lidar']['origins']
    localization['dual_lidar_adapter']['ros__parameters'].update(target_frame='d1max_loc_lidar',
        front_translation=front_origin, front_rotation=[0.,0.,0.,1.],
        rear_to_front_translation=[b-a for a,b in zip(front_origin,rear_origin)],
        rear_to_front_rotation=[0.,0.,0.,1.], imu_acceleration_scale=1.,
        input_clock_mode='recorded_sim_time')
    session['isaac_bridge_contract']['legacy_Airy_adapter_launched']=False
    session['isaac_bridge_contract']['future_LIO_adapter_requires']=\
        '33-ring PhysX profile, handling of unmeasured reflectivity and removal of Airy-specific Y-up IMU rotation; no Livox messages are fabricated'
    localization_path.write_text(yaml.safe_dump(localization, sort_keys=True))
    # Fixture-only recording views, written before the original session seal.
    # The source template's 98 m camera targets a different, much larger map.
    room=spec['room']
    center=dict(X=(room['x_min']+room['x_max'])/2.,Y=(room['y_min']+room['y_max'])/2.,Z=spec['floor']['z']+.4)
    overview=dict(Class='rviz_default_plugins/Orbit',Distance=max(26.,1.3*max(room['width'],room['depth'])),Pitch=1.,Yaw=.8,
        **{'Focal Point':center,'Field of View':1.2,'Target Frame':'<Fixed Frame>'})
    follow=dict(Class='rviz_default_plugins/Orbit',Distance=6.,Pitch=.85,Yaw=.8,
        **{'Focal Point':dict(X=0.,Y=0.,Z=0.),'Target Frame':'d1max_loc_base_link'})
    for layout in ('global','local'):
        path=directory/(layout+'_planning.rviz')
        view_config=yaml.safe_load(path.read_text())
        # Show the actual sensor-frame hit clouds in either presentation.
        # This group is display-only and uses the existing measured topics/TF.
        clouds=[]
        for name,topic,color in (('Front 3D LiDAR','/front_lidar','55; 215; 235'),
                                ('Rear 3D LiDAR','/rear_lidar','230; 120; 220')):
            clouds.append({'Class':'rviz_default_plugins/PointCloud2','Name':name,
                'Enabled':True,'Value':True,'Alpha':.65,'Color Transformer':'FlatColor',
                'Color':color,'Position Transformer':'XYZ','Decay Time':.15,
                'Style':'Points','Size (Pixels)':2,'Size (m)':.025,'Use Fixed Frame':True,
                'Topic':{'Value':topic,'Depth':2,'History Policy':'Keep Last',
                         'Durability Policy':'Volatile','Reliability Policy':'Best Effort'}})
        view_config['Visualization Manager']['Displays'].append({
            'Class':'rviz_common/Group','Name':'Measured 3D LiDAR',
            'Enabled':True,'Displays':clouds})
        if quadruped:
            view_config['Visualization Manager']['Displays'].append({
                'Class':'rviz_default_plugins/MarkerArray', 'Name':'Measured dynamic actors',
                'Enabled':True, 'Value':True, 'Topic':{'Value':'/d1max/isaacsim/dynamic_actors',
                    'Depth':1,'History Policy':'Keep Last','Reliability Policy':'Reliable','Durability Policy':'Volatile'}})
        views=view_config['Visualization Manager']['Views']
        views['Current'].update(overview if layout=='global' else follow)
        for saved in views.get('Saved',[]):
            if saved.get('Class')=='rviz_default_plugins/Orbit':
                saved.update(follow if saved.get('Target Frame')=='d1max_loc_base_link' else overview)
            elif saved.get('Class')=='rviz_default_plugins/TopDownOrtho':
                saved.update(X=center['X'],Y=center['Y'],Scale=50.)
        view_config['Window Geometry'].update(Width=960,Height=980)
        path.write_text(yaml.safe_dump(view_config,allow_unicode=True,sort_keys=False))
    session['parameter_hashes'] = {path.name:sha(path) for path in directory.iterdir()
        if path.suffix in ('.yaml', '.xml', '.rviz')}
    (directory/'session.json').write_text(json.dumps(session, indent=2)+'\n')
    if seal:
        verified, commands = verify(directory, seal_runtime=True)
        return verified, commands
    return session, {}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--release', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--map-directory', type=Path)
    parser.add_argument('--session-id')
    parser.add_argument('--no-seal', action='store_true', help='prepare files only; the graph refuses unsealed runtime')
    args = parser.parse_args()
    no_seal = args.no_seal
    del args.no_seal
    session, commands = prepare_isaac_session(**vars(args), seal=not no_seal)
    print(json.dumps(dict(session=str(args.output.resolve()), id=session['id'], commands=commands,
        physical_acceptance=False, runtime_sealed=not no_seal)))
