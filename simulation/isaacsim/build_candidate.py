#!/usr/bin/env python3
"""Assemble an explicitly isolated Isaac fixture from locally compiled artifacts.

This never selects/deploys a production release or creates physical acceptance.
The normal navigation session will independently seal/check its runtime closure.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil

from map_builder import build as build_map
from truth_map import build as build_prior

REPO = Path(__file__).resolve().parents[2]


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def copy_tree(source, target):
    """Dereference symlink-install files so actual imported bytes stay inside."""
    source = Path(source).resolve(strict=True)
    shutil.copytree(source, target, symlinks=False, dirs_exist_ok=True,
        ignore=shutil.ignore_patterns('__pycache__', '*.pyc', '.git', '.pytest_cache',
            'runs', 'robot_cache', '*.log'))


def verify_python_runtime_closure(nav_source, install):
    """Require each copied Python module to match its sealed source bytes.

    Colcon's Python installs can be copies even when native packages use
    symlink-install. Updating nav-source alone does not update those imports.
    Check the final merged install against the snapshot, without importing
    ROS or executing any installed module. Every discovered runtime location
    must agree, including nested modules and leftover installed Python files.
    """
    nav_source, install = Path(nav_source), Path(install)
    packages = [p for p in sorted((nav_source/'src').glob('d1max_*'))
        if (p/p.name/'__init__.py').is_file()]
    if not any(p.name == 'd1max_pct_scan' for p in packages):
        raise ValueError('python_runtime_source_package_missing:d1max_pct_scan')
    sites = sorted(p for layout in ('site-packages', 'dist-packages')
        for p in install.rglob(layout) if p.is_dir())
    checked = {}
    for package in packages:
        source = package/package.name
        # Source trees can contain uninstalled test directories. This repo's
        # Python packages use ordinary __init__.py packages, so only their
        # module closure is expected in the runtime. Installed extras below
        # are still rejected, including a leftover namespace directory.
        def source_module(path):
            parents = [path.parent, *path.parent.parents]
            return all((p/'__init__.py').is_file() for p in parents[:parents.index(source)+1])
        expected = {p.relative_to(source).as_posix(): digest(p)
            for p in sorted(source.rglob('*.py')) if '__pycache__' not in p.parts and source_module(p)}
        runtimes = [p/package.name for p in sites if (p/package.name).is_dir()]
        if not runtimes:
            raise ValueError('python_runtime_package_missing:'+package.name)
        for runtime in runtimes:
            actual = {p.relative_to(runtime).as_posix(): digest(p)
                for p in sorted(runtime.rglob('*.py')) if '__pycache__' not in p.parts}
            missing, extra = sorted(expected.keys()-actual.keys()), sorted(actual.keys()-expected.keys())
            if missing or extra:
                raise ValueError('python_runtime_module_closure_mismatch:'+package.name+
                    ':missing='+','.join(missing)+':unexpected='+','.join(extra))
            for name, source_hash in expected.items():
                if source_hash != actual[name]:
                    raise ValueError('python_runtime_source_mismatch:'+package.name+':'+name+
                        ':source_sha256='+source_hash+':installed_sha256='+actual[name])
        checked[package.name] = dict(module_count=len(expected),
            runtime_locations=[p.relative_to(install).as_posix() for p in runtimes])
    return dict(schema=1, contract='snapshot_installed_python_bytes_v1', packages=checked)


def assemble(output, nav_install, pct_vendor, scene_config, sdk_install=None, localization_install=None):
    output = Path(output).resolve()
    if output.exists():
        raise ValueError('candidate_output_must_not_exist')
    nav_install = Path(nav_install).resolve(strict=True)
    pct_vendor = Path(pct_vendor).resolve(strict=True)
    output.mkdir(parents=True)
    copy_tree(nav_install, output/'application/install')
    # A separately built SDK-free writer overlay may share the main install.
    # Merging its resource index/package hooks keeps one self-contained ROS
    # prefix and avoids accidentally sourcing a production SDK install.
    if sdk_install is not None and Path(sdk_install).resolve() != nav_install:
        copy_tree(sdk_install, output/'application/install')
    if localization_install is not None and Path(localization_install).resolve() != nav_install:
        copy_tree(localization_install, output/'application/install')
    snapshot = output/'source_snapshot/nav'
    snapshot.mkdir(parents=True)
    source = REPO/'d1max_nav_ws'
    for package in sorted((source/'src').glob('d1max_*')):
        if package.is_dir():
            copy_tree(package, snapshot/'src'/package.name)
    copy_tree(source/'src/scan_planner_vendor', snapshot/'src/scan_planner_vendor')
    # Reject stale copied installs before map generation, release metadata or
    # an integrity seal can make this directory a selectable candidate.
    python_runtime_closure = verify_python_runtime_closure(snapshot, output/'application/install')
    sdk_source=REPO/'d1max_ros2/sdk_bridge_ws/src/d1max_sdk_bridge'
    shutil.copytree(sdk_source, output/'source_snapshot/sdk/d1max_sdk_bridge',
        symlinks=False, ignore=shutil.ignore_patterns('vendor','__pycache__','*.pyc','.git','*.log'))
    copy_tree(source/'tools', snapshot/'tools')
    # Native build evidence contains target-machine paths, so it is explicitly
    # retained as an external selected runtime. Rewriting it would fabricate
    # compiler/ABI evidence; moving it requires a target-machine rebuild.
    simulation = output/'simulation'
    copy_tree(Path(__file__).resolve().parent, simulation)
    config = simulation/'assets/scene_config.json'
    shutil.copyfile(Path(scene_config).resolve(strict=True), config)
    map_result = build_map(config, output/'map/isaac_floor', pct_vendor)
    map_dir = output/'map/isaac_floor'
    identity = json.loads((map_dir/'manifest.json').read_text())
    map_version = hashlib.sha256(('source_identity\n'+identity['source_sha256']+'\n'+
        digest(map_dir/'manifest.json')+'\n'+digest(map_dir/'tomogram.npz')).encode()).hexdigest()
    prior_result = build_prior(config, simulation/'assets/indoor_scene.usda',
        map_dir/'collision_prior/static_prior.json', map_version=map_version)
    descriptor = dict(schema=1, candidate_kind='isolated_isaac_fixture',
        default_map_directory='map/isaac_floor', template_source_root='source_snapshot/nav',
        tools_source_root='source_snapshot/nav/tools',
        sealed_manifest='isaac_candidate_integrity.json', preflight_session='preflight_session',
        activation=False, physical_acceptance=False, production_release=False,
        external_pct_vendor=str(pct_vendor), simulation_config='simulation/assets/scene_config.json',
        static_collision_prior_manifest='map/isaac_floor/collision_prior/static_prior.json',
        purpose='Isolated mainline BT/PCT/SCAN/tracker/safety validation with a PhysX wheel fixture')
    (output/'release.json').write_text(json.dumps(descriptor, indent=2)+'\n')
    files = {str(path.resolve()): digest(path) for path in sorted(output.rglob('*'))
        if path.is_file() and '__pycache__' not in path.parts}
    manifest = dict(schema=1, created_at=datetime.now(timezone.utc).isoformat(),
        kind='isaac_fixture_integrity_not_production_release_seal', files=files,
        map=map_result, static_collision_prior=prior_result, activation=False, physical_acceptance=False,
        python_runtime_closure=python_runtime_closure,
        external_pct_vendor=str(pct_vendor),
        note='Session prepare/verify binds actual native/runtime loader dependencies before launch.')
    (output/'isaac_candidate_integrity.json').write_text(json.dumps(manifest, indent=2)+'\n')
    return dict(candidate=str(output), install=str(output/'application/install'),
        map=str(output/'map/isaac_floor'), physical_acceptance=False, production_release=False)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--nav-install', type=Path, required=True)
    parser.add_argument('--sdk-install', type=Path)
    parser.add_argument('--localization-install', type=Path)
    parser.add_argument('--pct-vendor', type=Path, required=True)
    parser.add_argument('--scene-config', type=Path, default=REPO/'simulation/isaacsim/assets/scene_config.json')
    parser.add_argument('--select-local', type=Path, help='Save an explicit local fixture selection; never changes the production selector')
    args = parser.parse_args()
    selector = args.select_local
    del args.select_local
    result = assemble(**vars(args))
    if selector is not None:
        selector = selector.resolve()
        selector.parent.mkdir(parents=True, exist_ok=True)
        temporary = selector.with_suffix('.tmp')
        temporary.write_text(json.dumps(dict(schema=1,
            kind='isolated_isaac_fixture_selection', candidate=result['candidate'],
            integrity_sha256=digest(Path(result['candidate'])/'isaac_candidate_integrity.json'),
            physical_acceptance=False, production_release=False), indent=2)+'\n')
        temporary.replace(selector)
    print(json.dumps(result))
