"""Session-wide navigation contract, checked before any owned ROS child starts.

Map planning and odom control are different roles, not interchangeable frame
labels. This module checks wiring/provenance; it never grants motion or changes
an estimator, collision rule, timestamp, or physical acceptance flag.
"""
import hashlib
import importlib.util
import json
from pathlib import Path

import yaml

from .live_runtime import file_sha256


FRAMES = dict(map_frame='d1max_loc_map', odom_frame='d1max_loc_odom',
              body_frame='d1max_loc_base_link', tracking_frame='d1max_loc_tracking',
              lidar_frame='d1max_loc_lidar')
BUNDLE_FILES = ('localization.yaml', 'global.yaml', 'bridge.yaml', 'scan.yaml',
                'pct_ground_support.npz')
ARTIFACT_KEYS = ('map_pcd', 'planning_manifest', 'tomogram_npz',
                 'crossfloor_route_config')


def identity(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                    allow_nan=False).encode()).hexdigest()


def localization_contract(config, frame_id):
    """Validate the effective launch branch, not unused legacy YAML sections."""
    from d1max_localization.estimation.configuration import navigation_parameters
    if config.get('localization_pipeline', {}).get('ros__parameters', {}).get('backend') != 'lio_pcd':
        raise ValueError('live planning requires the lio_pcd body-odometry contract')
    if config.get('navigation_estimation', {}).get('ros__parameters', {}).get('enabled') is not True:
        raise ValueError('live planning requires enabled continuous body navigation output')
    prediction, output, _ = navigation_parameters(config)
    front = config['lio_localizer']['ros__parameters']
    actual = {**{key: output[key] for key in ('map_frame', 'odom_frame', 'tracking_frame', 'body_frame')},
              'lidar_frame': front['lidar_frame']}
    # Downstream nodes currently have this explicit contract. Fail at startup
    # instead of silently accepting renamed frames then waiting forever.
    if actual != FRAMES or frame_id != actual['map_frame']:
        raise ValueError('localization/planning frame contract mismatch')
    lio = config['/d1max/localization/lio/laserMapping']['ros__parameters']
    matcher = config['lio_global_matcher']['ros__parameters']
    adapter = config['dual_lidar_adapter']['ros__parameters']
    if (lio.get('publish.tf_enabled') is not False
            or lio.get('publish.world_frame') != actual['odom_frame']
            or lio.get('publish.body_frame') != actual['tracking_frame']
            or matcher.get('world_frame') != actual['map_frame']
            or matcher.get('tracking_frame') != actual['tracking_frame']
            or matcher.get('lidar_frame') != actual['tracking_frame']
            or adapter.get('target_frame') != actual['lidar_frame']
            or prediction['imu_frame'] != actual['lidar_frame']):
        raise ValueError('LIO/matcher/adapter frame or single-TF-authority contract mismatch')
    if (lio.get('common.lid_topic') != adapter.get('output_livox_topic')
            or lio.get('common.imu_topic') != adapter.get('output_imu_topic')
            or adapter.get('output_livox_topic') != '/d1max/localization/livox_points'
            or adapter.get('output_imu_topic') != '/d1max/localization/imu'
            or lio.get('localization.enabled') is not True
            or lio.get('publish.scan_publish_en') is not True
            or lio.get('publish.scan_bodyframe_pub_en') is not True
            or matcher.get('scan_topic') != '/d1max/localization/lio/deskewed'
            or matcher.get('imu_topic') != adapter.get('output_imu_topic')
            or matcher.get('input_deskewed') is not True
            or matcher.get('deskew_enabled') is not False
            or matcher.get('translation_deskew_enabled') is not False):
        raise ValueError('localization producer/consumer input contract mismatch')
    return dict(schema=1, frames=actual, tf_authority='navigation_output',
        body_pose_reference='body_center', twist_reference='body_center_body_axes',
        output_rate_hz=output['output_rate_hz'],
        global_planning=dict(odometry='/d1max/localization/odometry/global',
                             frame=actual['map_frame']),
        local_planning=dict(odometry='/d1max/live_planning/body_pose',
                            frame=actual['map_frame'], role='collision_checked_map_curve'),
        required_control=dict(odometry='/d1max/localization/odometry/local',
            frame=actual['odom_frame'], conversion='same_time_versioned_map_to_odom_anchor',
            implemented=False),
        ground_to_body_height_applications=1)


def validate_perception_branch(session, localization):
    if (session.get('perception_backend') == 'per_sensor_rays'
            and localization['dual_lidar_adapter']['ros__parameters'].get('lidar_mode') != 'dual'):
        # Native map source-age admission requires BOTH sensor slots. A single
        # source projector is not a valid alternative until that contract changes.
        raise ValueError('per_sensor_rays native map requires dual lidar acquisition')


def artifact_identity(session):
    fingerprints = {key: file_sha256(session[key]) for key in ARTIFACT_KEYS}
    return 'navigation-map-' + identity(fingerprints), fingerprints


def validate_effective_wiring(directory, session):
    """Check producer/consumer edges in the files launch will actually read."""
    from .robot_profile import scan_robot_parameters
    parameters = {name: yaml.safe_load((Path(directory)/(name+'.yaml')).read_text())
                  ['/**']['ros__parameters'] for name in ('global', 'bridge', 'scan')}
    global_, bridge, scan = (parameters[name] for name in ('global', 'bridge', 'scan'))
    sid, frames = session['id'], session['navigation_contract']['frames']
    expected_edges = (
        (global_.get('session_id'), sid), (bridge.get('session_id'), sid),
        (bridge.get('localization_session_id'), sid),
        (bridge.get('map_version_id'), session['version_id']),
        (scan.get('fsm.navigation_session_id'), sid),
        (scan.get('grid_map.localization_session_id'), sid),
        (bridge.get('map_frame'), frames['map_frame']),
        (bridge.get('body_frame'), frames['body_frame']),
        (bridge.get('tracking_frame'), frames['tracking_frame']),
        (scan.get('grid_map.frame_id'), frames['map_frame']),
        (scan.get('fsm.reference_path_z_offset'), session['body_height']),
        (bridge.get('body_height'), session['body_height']),
        (bridge.get('perception_backend'), session['perception_backend']),
        (bridge.get('collision_policy'), session['scan_collision_policy']),
    )
    for actual, expected in expected_edges:
        if actual != expected or type(actual) is not type(expected):
            raise ValueError('effective navigation handoff contract mismatch')
    if (scan.get('fsm.require_tagged_reference') is not True
            or scan.get('grid_map.require_localization_context') is not True
            or scan.get('grid_map.use_projected_rays') is not
                (session['perception_backend'] == 'per_sensor_rays')
            or scan.get('grid_map.require_observed_free') is not
                (session['scan_collision_policy'] == 'observed_free')):
        raise ValueError('effective perception/reference ownership mismatch')
    for name in ('planning_manifest', 'tomogram_npz', 'crossfloor_route_config'):
        if global_.get(name) != session[name]:
            raise ValueError('global planner map artifact mismatch: '+name)
    for name, value in scan_robot_parameters(session['robot_profile_snapshot']).items():
        if scan.get(name) != value:
            raise ValueError('effective robot collision geometry mismatch: '+name)
    if session.get('task_orchestrator'):
        from .bt_configuration import validate_tree_wiring
        validate_tree_wiring(directory, session)


def bundle_files(session):
    from .bt_configuration import FILES
    return (BUNDLE_FILES + (FILES if session.get('task_orchestrator') else ())
            + (('motion.yaml',) if session.get('motion_control_enabled') is True else ()))


def seal_bundle(directory, session):
    """Record effective files and semantic identity, excluding camera/layouts."""
    directory = Path(directory)
    version, artifacts = artifact_identity(session)
    if session['version_id'] != version:
        raise ValueError('map artifacts changed during session preparation')
    localization = yaml.safe_load((directory/'localization.yaml').read_text())
    validate_perception_branch(session, localization)
    contract = localization_contract(localization, session['frame_id'])
    if contract != session['navigation_contract']:
        raise ValueError('effective localization differs from navigation contract')
    validate_effective_wiring(directory, session)
    files = bundle_files(session)
    return dict(schema=1, session_identity=identity(session), artifacts=artifacts,
                effective_files={name: file_sha256(directory/name) for name in files})


def verify_bundle(directory, session):
    """Old/unsealed or subsequently edited sessions must be prepared afresh."""
    expected = session.get('bundle_fingerprints')
    core = {key: value for key, value in session.items()
            if key not in ('bundle_fingerprints', 'deployment_evidence')}
    if not isinstance(expected, dict):
        raise ValueError('navigation session bundle changed or unsealed; prepare a new session')
    # Compare bytes before decoding potentially edited/partial YAML. Relative
    # filenames are a fixed allowlist, never paths supplied by a saved record.
    files = bundle_files(session)
    actual_files = {name: file_sha256(Path(directory)/name) for name in files}
    if (identity(core) != expected.get('session_identity')
            or actual_files != expected.get('effective_files')
            or seal_bundle(directory, core) != expected):
        raise ValueError('navigation session bundle changed or unsealed; prepare a new session')


def file_record(path, workspace):
    path, root = Path(path).resolve(), Path(workspace).resolve()
    if not path.is_file() or not path.is_relative_to(root):
        raise ValueError('navigation executable/module resolved outside actual workspace: '+str(path))
    return dict(path=str(path), sha256=file_sha256(path))


def deployment_evidence(workspace, *, prefix_lookup=None, spec_lookup=None,
                        include_behavior_tree=False):
    """Resolve the real AMENT/Python overlay. Never equate source edit with build.

    Records selected entrypoints and all Python source in the participating
    packages. Native ELF hashes are independent evidence, not a claim that a
    given source hash was compiled into that ELF. External ROS distro libraries
    are outside this workspace check and are not pinned by it.
    """
    if prefix_lookup is None:
        from ament_index_python.packages import get_package_prefix
        prefix_lookup = get_package_prefix
    spec_lookup = spec_lookup or importlib.util.find_spec
    workspace = Path(workspace).resolve()
    records, prefixes = {}, {}
    entries = {
        'd1max_localization': ('lib/d1max_localization/dual_lidar_adapter',
                              'lib/d1max_localization/fused_icp_matcher',
                              'lib/d1max_localization/lio_localizer',
                              'lib/d1max_localization/lio_predictor',
                              'lib/d1max_localization/navigation_output',
                              'share/d1max_localization/launch/localization.launch.py'),
        'faster_lio': ('lib/faster_lio/run_mapping_online',),
        'scan_planner': ('lib/scan_planner/scan_planner_node',),
        'd1max_planning_interfaces': (),
        'scan_planner_msgs': (),
    }
    if include_behavior_tree:
        entries.update(d1max_navigation_bt=(
            'lib/d1max_navigation_bt/navigator_node',
            'share/d1max_navigation_bt/trees/navigate_committed_route.xml'),
            d1max_navigation_bt_interfaces=(), d1max_pct_rviz_tools=(
                'lib/libd1max_pct_rviz_tools.so', 'share/d1max_pct_rviz_tools/plugins.xml'))
    for package, paths in entries.items():
        prefix = Path(prefix_lookup(package)).resolve()
        if not prefix.is_relative_to(workspace/'install'):
            raise ValueError('foreign AMENT overlay for '+package+': '+str(prefix))
        prefixes[package] = str(prefix)
        for name in paths:
            records[package+'/'+name] = file_record(prefix/name, workspace)
    for package in ('d1max_pct_scan', 'd1max_pct_planner', 'd1max_localization'):
        spec = spec_lookup(package)
        if spec is None or not spec.origin:
            raise ValueError('missing navigation Python package: '+package)
        root = Path(spec.origin).resolve().parent
        file_record(spec.origin, workspace)
        for path in sorted(root.rglob('*.py')):
            records[package+'/'+str(path.relative_to(root))] = file_record(path, workspace)
    # The non-rigid ground bridge is runtime code, not only a map-generation
    # utility. Pin the actual imported package and its same-package dependencies.
    package = 'tools.pointcloud_preprocessing'
    spec = spec_lookup(package)
    if spec is None or not spec.origin:
        raise ValueError('missing navigation Python package: '+package)
    file_record(spec.origin, workspace)
    root = Path(spec.origin).resolve().parent
    for module in ('ground_path_bridge', 'flat_floor', 'pcd_io'):
        module_spec = spec_lookup(package+'.'+module)
        if (module_spec is None or not module_spec.origin
                or Path(module_spec.origin).resolve().parent != root):
            raise ValueError('ground bridge dependency resolved outside its package: '+module)
        file_record(module_spec.origin, workspace)
    for path in sorted(root.rglob('*.py')):
        relative = path.relative_to(root)
        if not {'tests', '__pycache__'}.intersection(relative.parts):
            records[package+'/'+str(relative)] = file_record(path, workspace)
    # Pin only this workspace's generated interface libraries, not ROS distro
    # dependencies. Symlink-install Python sources may resolve into build/;
    # their import origin must still belong to the matching installed prefix.
    interface_packages = ('d1max_planning_interfaces', 'scan_planner_msgs') + (
        ('d1max_navigation_bt_interfaces',) if include_behavior_tree else ())
    for package in interface_packages:
        prefix = Path(prefixes[package])
        libraries = sorted((prefix/'lib').glob('lib'+package+'__rosidl_*.so*'))
        required = {'lib'+package+'__rosidl_'+kind+'.so' for kind in (
            'generator_c', 'generator_py', 'typesupport_c', 'typesupport_cpp',
            'typesupport_introspection_c', 'typesupport_introspection_cpp')}
        if not required.issubset({path.name for path in libraries}):
            raise ValueError('missing generated interface libraries: '+package)
        for path in libraries:
            records[package+'/lib/'+path.name] = file_record(path, workspace)
        spec = spec_lookup(package)
        if (spec is None or not spec.origin
                or not Path(spec.origin).absolute().is_relative_to(prefix)):
            raise ValueError('foreign generated Python interface: '+package)
        file_record(spec.origin, workspace)
        root = Path(spec.origin).resolve().parent
        extensions = sorted(root.glob(package+'_s__rosidl_typesupport_*.so'))
        if not any(path.name.startswith(package+'_s__rosidl_typesupport_c.')
                   for path in extensions):
            raise ValueError('missing generated Python typesupport extension: '+package)
        python_files = sorted(root.rglob('*.py'))
        for path in python_files + extensions:
            records[package+'/python/'+str(path.relative_to(root))] = file_record(path, workspace)
        generator = root/('lib'+package+'__rosidl_generator_py.so')
        records[package+'/python/'+generator.name] = file_record(generator, workspace)
    return dict(schema=1, workspace=str(workspace), prefixes=prefixes, files=records,
                native_build_source_equivalence='not_inferred')


def verify_deployment(expected, workspace):
    includes_bt = isinstance(expected, dict) and 'd1max_navigation_bt' in expected.get('prefixes', {})
    if not isinstance(expected, dict) or expected != deployment_evidence(
            workspace, include_behavior_tree=includes_bt):
        raise ValueError('navigation deployment changed or unverified; prepare a new session')


def main():
    """Offline inspection only; no node, transport, service or session startup."""
    import argparse
    from .live_session import load_config
    from .motion_stack import motion_architecture_blockers
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', type=Path, required=True)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    config = load_config(args.config)
    version, artifacts = artifact_identity(config)
    report = dict(schema=1, read_only_inspection=True, started_processes=False,
        config=str(args.config.resolve()), navigation_contract=config['navigation_contract'],
        map_version_id=version, artifacts=artifacts,
        deployment=deployment_evidence(args.workspace,
            include_behavior_tree=bool(config.get('task_orchestrator'))),
        execution_architecture_blockers=list(motion_architecture_blockers(config)),
        hardware_acceptance='pending', production_deployment_performed=False)
    encoded = json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+'\n'
    if args.output:
        args.output.write_text(encoded)
        print(args.output.resolve())
    else:
        print(encoded)


if __name__ == '__main__':
    main()
