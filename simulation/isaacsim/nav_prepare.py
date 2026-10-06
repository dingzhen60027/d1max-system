#!/usr/bin/env python3
"""Prepare the existing navigation session for an isolated PhysX wheel fixture.

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
    width = max(body_width, float(robot['wheel_base'])+float(robot['wheel_width']))
    # Circumscribe the fixture chassis rectangle with two circles. Checking
    # width and length extents alone misses corners, so radius is sqrt(dx²+dy²).
    offset = body_length/4.
    radius = (offset**2+(width/2.)**2)**.5+.03
    length = 2*(offset+radius)
    above_base = max(body_vertical/2., float(robot['wheel_axle_z'])+float(robot['wheel_radius']))+.05
    profile = dict(schema=1, model='Isaac Sim differential drive fixture', fixture_only=True,
        physical_acceptance=False, geometry_source=str(config_path), geometry_sha256=sha(config_path),
        body_reference_height_m=height, actual_body_size_m=robot['body_size'], actual_total_width_m=width,
        double_cylinder_radius_m=radius, double_cylinder_offset_m=offset,
        native_envelope_length_m=length, native_envelope_width_m=2*radius,
        obstacle_dilation_up_m=height-.10, obstacle_dilation_down_m=above_base,
        max_speed_mps=.30, max_yaw_radps=.50, ground_exclusion_height_m=.10,
        note='Known simulation geometry only; never authorizes the D1 Max SDK or physical robot.')
    if not .25 <= height <= .85 or radius < width/2.:
        raise ValueError('wheel_fixture_geometry_outside_supported_floor_profile')
    profile_path = directory/'wheel_fixture_profile.yaml'
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
        max_body_tilt_rad=.01,source_scope=prior['source_scope'],
        expected_body_world_z=float(spec['floor']['z'])+float(robot['body_reference_height']),
        max_body_height_error_m=.005,
        runtime_geometry_attestation_required=True)
    for artifact in (prior_path, prior_path.parent/prior['data_file'],
            release/'simulation/truth_map.py',release/'simulation/assets/indoor_scene.usda'):
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
    for path in directory.glob('*.yaml'):
        if path.name == 'wheel_fixture_profile.yaml':
            continue
        value = yaml.safe_load(path.read_text())
        if '/**' in value and 'ros__parameters' in value['/**']:
            value['/**']['ros__parameters'].update(use_sim_time=True)
            value['/**']['ros__parameters'].update(updates.get(path.name, {}))
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
    overview=dict(Class='rviz_default_plugins/Orbit',Distance=26.,Pitch=1.,Yaw=.8,
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
