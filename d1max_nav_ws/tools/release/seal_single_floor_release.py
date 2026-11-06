#!/usr/bin/env python3
"""Seal the actually imported isolated build. Does not launch ROS or the SDK.

This is a local integrity manifest, not physical acceptance or deployment.
Run after building the application without symlinks and sourcing all overlays.
An existing seal is never overwritten; archive it before resealing the selected
package after a complete build. Resealing is not physical motion acceptance.
The release names its manifest and default map in <release>/release.json,
which is itself sealed; the entry script reads it instead of a fixed revision.
"""
import argparse
import importlib.util
import json
from pathlib import Path
import subprocess
from datetime import datetime, timezone
from single_floor_entry_preflight import (elf_dependency_paths, verify_localization_dependencies,
    startup_prefixes, capture_startup_closure, source_backed_python_cache)


def elf_environment(path, native, native_environment):
    """Only PCT's child-owned ELFs use its GTSAM loader environment.

    Do not apply that override to ROS estimators, SDK, BT, tracker or RViz.
    Their actual launch environments can legitimately load different libraries.
    """
    path = Path(path).resolve()
    roots = (Path(native['native_lib_dir']).resolve(), Path(native['gtsam_lib_dir']).resolve())
    return native_environment if any(path.is_relative_to(root) for root in roots) else None


def localization_build_claim(prefix):
    if prefix is None:
        return 'external_selected_runtime'
    provenance = json.loads((Path(prefix).parent/'runtime_copy_provenance.json').read_text())
    # Initial dependencies were copied unchanged. A later native component
    # rebuild must not keep that historical claim for the updated bundle.
    return ('source_rebuild_with_runtime_copy' if provenance.get('rebuilt_packages')
            else 'same_byte_copy_not_recompiled')


def navigation_launch_bindings(app_root):
    """Bind navigation dispatch, not the independently deployed Web shell.

    The manifest must reject changed navigation/SDK launch code. General app
    routes and content-hashed frontend chunks are not navigation artifacts:
    editing a map page or adding a remote-control launcher must not invalidate
    an otherwise unchanged navigation package. Web route registration is
    checked by the Web application's own startup/route tests.
    """
    root = Path(app_root)
    return {relative: root / relative for relative in (
        'map_manager/backend/navigation.py',
        'map_manager/backend/single_floor.py',
        'map_manager/backend/mainline_release.py',
        'map_manager/backend/mainline_views.py',
        'map_manager/backend/navigation_session.py',
        'foxglove_d1max/scripts/start_sdk_monitor.sh',
        'foxglove_d1max/scripts/sdk_motion_startup.py',
        'foxglove_d1max/scripts/monitor_health.py',
        'foxglove_d1max/config/monitor-release',
        'foxglove_d1max/config/manager.json',
        'd1max_ros2_env.sh',
    )}


def snapshot_source_files(root):
    """Seal reproducible source inputs, not import-created Python bytecode.

    Reading a frozen source snapshot for an offline comparison may recreate
    __pycache__. Those derived files are not the deployed Python source and
    must not make the unchanged runtime fail its next integrity preflight.
    Keep all other files, including native objects and non-Python caches.
    """
    root = Path(root)
    members = {str(path): path for path in root.rglob('*') if path.is_file()}
    return (path for path in members.values() if not source_backed_python_cache(path, members))


def without_python_caches(files):
    """Filter the complete inventory, including legacy runtime input sources."""
    return {path: digest for path, digest in files.items()
            if not source_backed_python_cache(path, files)}


def main():
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    from d1max_pct_planner import paths
    from d1max_pct_scan.single_floor_session import prepare, verify, sha
    from d1max_pct_planner.native_runtime import prepare_native_environment, vendorverify
    from d1max_pct_planner.paths import expand_tree
    import yaml
    release,descriptor=paths.release_descriptor()
    parser=argparse.ArgumentParser(description=__doc__)
    parser.parse_args()
    output=paths.inside(release,descriptor['sealed_manifest'],what='sealed_manifest')
    preflight=paths.inside(release,descriptor['preflight_session'],what='preflight_session')
    if output.exists():raise ValueError('release_already_sealed_do_not_overwrite')
    imported=Path(importlib.util.find_spec('d1max_pct_scan').origin).resolve()
    if release/'application/install' not in imported.parents:
        raise ValueError('source_or_old_application_import_not_a_release')
    purpose=descriptor.get('default_session_purpose','execution')
    if purpose=='planning_only':
        # Pure artifact verification, never an SDK connection. The live graph
        # binds the real current SDK session only when explicitly started.
        prepare(preflight,purpose=purpose,transport_mode='live',
                expected_sdk_session='integrity-preflight-not-sdk')
    else:
        prepare(preflight,purpose=purpose)
    verify(preflight,seal_runtime=True)
    runtime=json.loads((preflight/'runtime_bundle.json').read_text())
    session=json.loads((preflight/'session.json').read_text())
    route=expand_tree(yaml.safe_load(Path(session['crossfloor_route_config']).read_text()))
    native=vendorverify(route['vendor_root'])
    native_environment=prepare_native_environment(route['vendor_root'])
    files={**runtime['files'],**session['input_hashes']}
    for name in session['parameter_hashes']:
        path=preflight/name; files[str(path.resolve())]=sha(path)
    def add(path):
        path=Path(path).resolve(strict=True);files[str(path)]=sha(path)
    localization_dependencies=paths.localization_dependencies_root()
    if localization_dependencies is not None:
        add(localization_dependencies.parent/'runtime_copy_provenance.json')
        for path in localization_dependencies.rglob('*'):
            if path.is_file() and '__pycache__' not in path.parts:add(path)
        verify_localization_dependencies(release,files,get_package_prefix)
    from single_floor_entry_preflight import tools_root
    frozen_tools=tools_root(release,paths.nav_root())
    add(frozen_tools/'single_floor_entry.sh')
    add(frozen_tools/'release/single_floor_entry_preflight.py')
    add(frozen_tools/'validation/bt_localization_replay_entry.sh')
    add(frozen_tools/'validation/run_bt_localization_replay.py')
    add(release/paths.RELEASE_DESCRIPTOR)
    startup_closure=capture_startup_closure(startup_prefixes(
        release,paths.nav_root(),get_package_prefix('rmw_zenoh_cpp')))
    for value in startup_closure['files'].values():
        add(value['resolved_path'])
    # Freeze the actual implementation, including newly introduced untracked
    # C++ headers. These copies are evidence/rebuild inputs, not imports from
    # a live, editable checkout. Copy them only after the final build.
    source_snapshot=release/'source_snapshot'
    if not source_snapshot.is_dir():
        raise ValueError('release_source_snapshot_missing')
    for path in snapshot_source_files(source_snapshot):
        add(path)
    # Live-only artifacts must be sealed even though no physical record exists
    # yet. No launch / SDK import is needed to inspect their bytes and linkage.
    for package,executables in {
        'd1max_sdk_bridge':('sdk_monitor_bridge','execution_acceptance_check'),
        'd1max_localization':('dual_lidar_adapter','fused_icp_matcher','global_lio_localizer','realtime_navigation_output'),
        'faster_lio':('run_mapping_online',),
        'robot_localization':('ekf_node',),
    }.items():
        prefix=Path(get_package_prefix(package))
        if package in ('d1max_sdk_bridge','d1max_localization') and release not in prefix.parents:
            raise ValueError('unreleased_component_binary:'+package)
        for executable in executables:add(prefix/'lib'/package/executable)
    localization=Path(get_package_share_directory('d1max_localization'))
    for path in (localization/'launch').rglob('*.py'):add(path)
    rviz_prefix=Path(get_package_prefix('d1max_pct_rviz_tools'))
    if release not in rviz_prefix.parents:raise ValueError('old_rviz_plugin_not_in_release')
    for root in (rviz_prefix/'lib',rviz_prefix/'share/d1max_pct_rviz_tools'):
        for path in root.rglob('*'):
            if path.is_file() and path.suffix in ('.so','.xml','.rviz','.yaml'):add(path)
    add(paths.app_root()/'foxglove_d1max/config/zenoh-live.json5')
    # The graph is not a complete local deployment if the external Web/SDK
    # dispatcher can silently select an older ELF or refuse the current name.
    # Bind actual launch code as external dependencies, not private activation
    # (which pins THIS manifest) or credentials. No circular selector/seal hash.
    launch_bindings={}
    for relative, path in navigation_launch_bindings(paths.app_root()).items():
        path=path.resolve(strict=True)
        add(path)
        launch_bindings[relative]={'path':str(path),'sha256':files[str(path)]}
    # RMW is loaded with dlopen, so ldd on planner executables misses it.
    rmw_prefix=Path(get_package_prefix('rmw_zenoh_cpp'))
    add(rmw_prefix/'lib/librmw_zenoh_cpp.so')
    for path in (rmw_prefix/'opt/zenoh_cpp_vendor/lib').glob('*.so*'):
        if path.is_file():add(path)
    for path in tuple(files):
        with Path(path).open('rb') as stream:elf=stream.read(4)==b'\x7fELF'
        if not elf:continue
        result=subprocess.run(['ldd',path],env=elf_environment(path,native,native_environment),
                              text=True,capture_output=True,timeout=10)
        if result.returncode or 'not found' in result.stdout:
            raise ValueError('unresolved_release_library:'+path)
        for target in elf_dependency_paths(result.stdout):add(target)
    files = without_python_caches(files)
    manifest=dict(schema=3,created_at=datetime.now(timezone.utc).isoformat(),
        activation=False,physical_acceptance=False,transport='rmw_zenoh_cpp',
        purpose='Integrity-sealed candidate, no deployment or motion authorization',
        source_snapshot=str(source_snapshot),
        files=dict(sorted(files.items())),types=runtime['types'],
        runtime_inputs=runtime['runtime_inputs'],
        scientific_loader_dependencies=runtime['scientific_loader_dependencies'],
        actual_tools_root=str(frozen_tools),
        launch_bindings=dict(scope='navigation_and_sdk_dispatchers_excluding_web_shell_and_static_assets',
                             files=launch_bindings),
        startup_closure=startup_closure,
        pct_dependency_resolution=dict(scope='PCT_ELFs_only_production_child_loader_environment',
            vendor_root=native['vendor_root'],gtsam_version=native['gtsam_version'],
            native_lib_dir=native['native_lib_dir'],gtsam_lib_dir=native['gtsam_lib_dir'],
            external_runtime_dependencies=[str(Path(native[k]).resolve()) for k in
                ('gtsam_library','metis_library') if not Path(native[k]).resolve().is_relative_to(release)]),
        localization_dependencies=dict(
            copied_runtime_prefix=str(localization_dependencies) if localization_dependencies else None,
            build_claim=localization_build_claim(localization_dependencies),
            external_runtime_dependencies=[p for p in files if not Path(p).is_relative_to(release)]),
        preflight_session=str(preflight/'session.json'))
    output.write_text(json.dumps(manifest,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps(dict(manifest=str(output),files=len(files),physical_acceptance=False)))


if __name__=='__main__':main()
