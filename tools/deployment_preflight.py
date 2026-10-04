#!/usr/bin/env python3
"""Read-only source/build inventory; never imports ROS or connects to a robot.

This is not the sealed runtime check or motion acceptance. Run using the Python
environment intended for the selected scope. Missing artifacts are reported,
not installed, downloaded, substituted or marked accepted.
"""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import platform
import shutil
import sys


def check_path(checks, name, path, required=True, directory=False):
    path = Path(path).expanduser().resolve()
    ok = path.is_dir() if directory else path.is_file()
    checks.append(dict(name=name, ok=ok, required=required, path=str(path)))


def inventory(repo, scope, *, environment=None):
    env = os.environ if environment is None else environment
    repo = Path(repo).resolve()
    nav = Path(env.get('D1MAX_NAV_ROOT', repo / 'd1max_nav_ws')).expanduser().resolve()
    app = Path(env.get('D1MAX_APP_ROOT', repo / 'd1max_ros2')).expanduser().resolve()
    checks = []
    for name, path in (
        ('source_mainline', nav / 'src/d1max_pct_scan/d1max_pct_scan/navigation_session.py'),
        ('public_entry', nav / 'tools/navigation_entry.sh'),
        ('build_source', repo / 'tools/build-source.sh'),
        ('web_source', app / 'map_manager/frontend/package-lock.json'),
        ('sdk_source', app / 'sdk_bridge_ws/src/d1max_sdk_bridge/CMakeLists.txt'),
        ('mapping_build_source', nav / 'scripts/build/build_slam.sh'),
        ('pct_native_source', Path(env.get('PCT_PLANNER_ROOT', nav / 'src/pct_planner_vendor'))
            / 'planner/lib/CMakeLists.txt'),
    ):
        check_path(checks, name, path)
    checks.append(dict(name='linux_x86_64_baseline', ok=(platform.system() == 'Linux'
        and platform.machine() == 'x86_64'), required=scope != 'source',
        detail=platform.system() + '/' + platform.machine()))
    if scope in ('nav', 'sdk', 'pct', 'all'):
        checks.append(dict(name='python310', ok=sys.version_info[:2] == (3, 10),
                           required=True, detail=platform.python_version()))
        for command in ('cmake', 'c++', 'rsync'):
            found = shutil.which(command)
            checks.append(dict(name=command, ok=found is not None, required=True, path=found))
    if scope in ('nav', 'sdk', 'all'):
        check_path(checks, 'ros_humble', '/opt/ros/humble/setup.bash')
        rmw = Path(env.get('D1MAX_RMW_PREFIX', app / 'local/opt/ros/humble')).expanduser()
        check_path(checks, 'zenoh_runtime', rmw / 'lib/librmw_zenoh_cpp.so')
        selected = env.get('RMW_IMPLEMENTATION')
        checks.append(dict(name='middleware_selection', ok=selected in (None, '', 'rmw_zenoh_cpp'),
                           required=True, detail=selected or 'not loaded'))
        found = shutil.which('colcon')
        checks.append(dict(name='colcon', ok=found is not None, required=True, path=found))
    if scope in ('nav', 'all'):
        # Check resource markers without importing rclpy or creating an endpoint.
        prefixes = [Path(p) for p in env.get('AMENT_PREFIX_PATH', '').split(os.pathsep) if p]
        prefixes.append(Path('/opt/ros/humble'))
        for package in ('fast_gicp', 'behaviortree_cpp_v3', 'nav2_util',
                        'nav2_collision_monitor', 'nav2_lifecycle_manager', 'robot_localization'):
            matches = [p for p in prefixes if (p / 'share/ament_index/resource_index/packages' / package).is_file()]
            checks.append(dict(name='ros_package:' + package, ok=bool(matches), required=True,
                               paths=[str(p) for p in matches]))
        livox = env.get('LIVOX_SDK2_PREFIX')
        libs = list(Path(livox).expanduser().glob('lib*/liblivox_lidar_sdk*')) if livox else []
        checks.append(dict(name='livox_sdk2', ok=bool(libs), required=True,
                           detail='Set LIVOX_SDK2_PREFIX to a built target-machine prefix',
                           paths=[str(p.resolve()) for p in libs]))
    if scope in ('sdk', 'all'):
        vendor = Path(env.get('D1MAX_SDK_VENDOR_ROOT',
            app / 'sdk_bridge_ws/src/d1max_sdk_bridge/vendor/robot_sdk')).expanduser()
        check_path(checks, 'sdk_headers', vendor / 'include', directory=True)
        check_path(checks, 'sdk_library', vendor / 'lib' / platform.machine() / 'librobot_sdk.so')
    modules = ()
    if scope in ('nav', 'pct', 'all'):
        modules += ('numpy', 'scipy', 'yaml', 'open3d')
    if scope in ('web', 'all'):
        modules += ('fastapi', 'uvicorn', 'yaml', 'numpy', 'open3d', 'PIL')
        for command in ('node', 'npm'):
            found = shutil.which(command)
            checks.append(dict(name=command, ok=found is not None, required=True, path=found))
    for name in dict.fromkeys(modules):
        try:
            spec = importlib.util.find_spec(name)
        except (ImportError, ValueError):
            spec = None
        checks.append(dict(name='python_module:' + name, ok=spec is not None, required=True,
                           path=spec.origin if spec else None))
    selector = nav / 'deploy/single_floor_release.json'
    release = None
    selector_error = None
    if env.get('D1MAX_RELEASE'):
        release = Path(env['D1MAX_RELEASE']).expanduser().resolve()
    elif selector.is_file():
        try:
            selected = json.loads(selector.read_text())
            relative = Path(selected['release_directory'])
            if relative.is_absolute() or '..' in relative.parts:
                raise ValueError('selector path escapes workspace')
            release = nav / relative
        except (KeyError, ValueError, TypeError):
            selector_error = 'invalid selector'
    else:
        selector_error = 'missing selector'
    return dict(schema=1, scope=scope, repository=str(repo), roots=dict(nav=str(nav), app=str(app)),
        python=sys.executable, checks=checks,
        passed=all(c['ok'] for c in checks if c['required']),
        runtime=dict(selected_release=str(release) if release else None,
            descriptor_present=bool(release and (release / 'release.json').is_file()),
            selector_error=selector_error, sealed_runtime_verified=False,
            physical_acceptance_verified=False,
            note='Source/build readiness is not runtime integrity, deployment or motion authorization'),
        external_data=dict(maps_root=env.get('D1MAX_MAPS_ROOT', str(nav / 'maps')),
            required=['original localization PCD', 'planning map + source_identity',
                      'PCT tomogram + route/conditioning manifests', 'real rosbag for replay'],
            included_in_git=False))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--scope', choices=('source', 'nav', 'sdk', 'pct', 'web', 'all'), default='source')
    parser.add_argument('--json', action='store_true')
    args = parser.parse_args()
    result = inventory(args.repo, args.scope)
    if args.json:
        print(json.dumps(result, ensure_ascii=False, indent=2))
    else:
        for c in result['checks']:
            print(('OK   ' if c['ok'] else 'MISS ') + c['name'] + ': '
                  + str(c.get('path', c.get('detail', c.get('paths', '')))))
        if result['passed']:
            print('SOURCE FILES READY' if args.scope == 'source' else 'CHECKED BUILD PREREQUISITES PRESENT')
        else:
            print('MISSING SOURCE/BUILD PREREQUISITES')
        print('Selected runtime:', result['runtime']['selected_release'])
        print('Descriptor present:', result['runtime']['descriptor_present'])
        print('No runtime seal, deployment or motion acceptance was verified.')
    return 0 if result['passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
