"""Safety checks for the source-build planner; no ROS/native build or SDK load."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


SCRIPT = Path(__file__).with_name('build-source.sh')


class SourceBuildTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix='d1max-build-plan-test-')
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.nav = self.root / 'nav'
        self.app = self.root / 'app'
        self.ros = self.root / 'ros'
        self.rmw = self.root / 'rmw'
        self.livox = self.root / 'livox'
        self.vendor = self.root / 'vendor'
        self.bin = self.root / 'bin'
        self.bin.mkdir()
        self.log = self.root / 'calls.jsonl'
        self.environment = dict(os.environ, PATH=str(self.bin) + ':' + os.environ['PATH'])
        for key in ('D1MAX_NAV_ROOT', 'D1MAX_APP_ROOT', 'D1MAX_RMW_PREFIX',
                    'LIVOX_SDK2_PREFIX', 'PCT_PLANNER_ROOT', 'D1MAX_SDK_VENDOR_ROOT'):
            self.environment.pop(key, None)
        self.environment['D1MAX_BUILD_TEST_LOG'] = str(self.log)
        for path in (
            self.nav / 'src/d1max_planning_interfaces/package.xml',
            self.nav / 'src/scan_planner_vendor/scan_planner_msgs/package.xml',
            self.nav / 'src/d1max_pct_scan/package.xml',
            self.nav / 'src/d1max_navigation_bt/package.xml',
            self.nav / 'src/pct_planner_vendor/planner/lib/CMakeLists.txt',
            self.nav / 'src/pct_planner_vendor/planner/lib/3rdparty/gtsam-4.1.1/CMakeLists.txt',
            self.nav / 'src/pct_planner_vendor/planner/lib/3rdparty/osqp/CMakeLists.txt',
            self.nav / 'src/pct_planner_vendor/planner/lib/3rdparty/pybind11/include/pybind11/pybind11.h',
            self.nav / 'src/pct_planner_vendor/planner/lib/3rdparty/osqp/lin_sys/direct/qdldl/qdldl_sources/include/qdldl.h',
            self.nav / 'src/pct_planner_vendor/planner/lib/3rdparty/osqp/lin_sys/direct/qdldl/qdldl_sources/src/qdldl.c',
            self.rmw / 'lib/librmw_zenoh_cpp.so',
            self.rmw / 'opt/zenoh_cpp_vendor/lib/.placeholder',
            self.rmw / 'share/rmw_zenoh_cpp/package.xml',
            self.livox / 'include/livox_lidar_api.h',
            self.livox / 'include/livox_lidar_def.h',
            self.livox / 'lib/liblivox_lidar_sdk_shared.so',
            self.app / 'sdk_bridge_ws/src/d1max_sdk_bridge/CMakeLists.txt',
            self.vendor / 'include/robot_sdk/sdk_client.hpp',
            self.app / 'map_manager/frontend/package.json',
            self.app / 'map_manager/frontend/package-lock.json',
        ):
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text('fixture\n')
        self.ros.mkdir()
        (self.ros / 'setup.bash').write_text('export ROS_DISTRO=humble\n')
        for library in (self.rmw / 'lib/librmw_zenoh_cpp.so',
                        self.livox / 'lib/liblivox_lidar_sdk_shared.so'):
            shutil.copy2('/bin/true', library)  # Architecture fixture, never loaded.
        self.make_program('node', '#!/bin/sh\nprintf "24.0.0\\n"\n')
        self.make_program('npm', '#!/bin/sh\nexit 0\n')

    def make_program(self, name, text):
        path = self.bin / name
        path.write_text(text)
        path.chmod(0o755)
        return path

    def invoke(self, *arguments):
        return subprocess.run(
            ['bash', str(SCRIPT), '--nav-root', str(self.nav), '--app-root', str(self.app),
             '--ros-prefix', str(self.ros), *map(str, arguments)],
            env=self.environment, text=True, capture_output=True, timeout=15,
        )

    def test_dry_run_plans_native_and_full_navigation_closure_without_writes(self):
        output = self.root / 'dry-output'
        result = self.invoke('--output', output, '--rmw-prefix', self.rmw,
                             '--livox-prefix', self.livox)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertFalse(output.exists())
        self.assertIn('--packages-up-to d1max_pct_scan', result.stdout)
        self.assertIn('nav-source/tools/pointcloud_preprocessing/', result.stdout)
        self.assertIn('--executor sequential', result.stdout)
        self.assertIn('gtsam-4.1.1', result.stdout)
        self.assertIn('--parallel 2', result.stdout)
        self.assertIn('not a sealed or activated navigation release', result.stdout)
        self.assertNotIn('d1max_ros2_env.sh', result.stdout)

    def test_missing_dependency_apply_stops_before_first_write(self):
        output = self.root / 'missing-output'
        result = self.invoke('--output', output, '--apply')
        self.assertEqual(result.returncode, 2)
        self.assertIn('explicit --rmw-prefix', result.stderr)
        self.assertIn('explicit --livox-prefix', result.stderr)
        self.assertFalse(output.exists())

    def test_existing_and_source_contained_outputs_are_rejected(self):
        for output in (self.nav, self.nav / 'new-build'):
            with self.subTest(output=output):
                result = self.invoke('--scope', 'web', '--output', output)
                self.assertEqual(result.returncode, 2)
                self.assertNotIn('npm --prefix', result.stdout)
        alias = self.root / 'nav-alias'
        alias.symlink_to(self.nav, target_is_directory=True)
        result = self.invoke('--scope', 'web', '--output', alias / 'new-build')
        self.assertEqual(result.returncode, 2)
        self.assertFalse((self.nav / 'new-build').exists())

    def test_web_plan_uses_lockfile_install_without_lifecycle_hooks(self):
        output = self.root / 'web-output'
        result = self.invoke('--scope', 'web', '--output', output)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('ci --ignore-scripts --no-audit --no-fund', result.stdout)
        self.assertIn('run build', result.stdout)
        self.assertNotIn('colcon', result.stdout)
        self.assertFalse(output.exists())

    def test_sdk_rejects_external_library_symlink_without_loading_it(self):
        library = self.vendor / 'lib' / os.uname().machine / 'librobot_sdk.so'
        library.parent.mkdir(parents=True)
        library.symlink_to('/bin/true')
        output = self.root / 'escaping-sdk-output'
        result = self.invoke('--scope', 'sdk', '--output', output,
                             '--rmw-prefix', self.rmw, '--sdk-vendor-root', self.vendor,
                             '--apply')
        self.assertEqual(result.returncode, 2)
        self.assertIn('escapes the supplied SDK tree', result.stderr)
        self.assertFalse(output.exists())

    @unittest.skipUnless(shutil.which('rsync') and shutil.which('readelf'), 'requires rsync/readelf')
    def test_sdk_apply_uses_copied_sources_and_clears_production_network_env(self):
        library = self.vendor / 'lib' / os.uname().machine / 'librobot_sdk.so'
        library.parent.mkdir(parents=True)
        shutil.copy2('/bin/true', library)  # Valid host ELF fixture, never loaded.
        self.make_program('colcon', '''#!/usr/bin/python3
import json, os, pathlib, sys
with open(os.environ['D1MAX_BUILD_TEST_LOG'], 'a') as stream:
    stream.write(json.dumps({'argv': sys.argv[1:], 'env': {
        k: os.environ.get(k) for k in ('ROS_DOMAIN_ID', 'ZENOH_SESSION_CONFIG_URI',
        'ROS_DISTRO', 'RMW_IMPLEMENTATION', 'CMAKE_BUILD_PARALLEL_LEVEL', 'CC', 'CXX')}}) + '\\n')
prefix = pathlib.Path(sys.argv[sys.argv.index('--install-base') + 1])
prefix.mkdir(parents=True)
(prefix / 'local_setup.bash').write_text('export D1MAX_FIXTURE_OVERLAY=1\\n')
''')
        python = self.make_program('python310', '#!/bin/sh\nprintf "3.10\\n"\n')
        c_compiler = self.make_program('fixture-cc', '#!/bin/sh\nexit 0\n')
        cxx_compiler = self.make_program('fixture-cxx', '#!/bin/sh\nexit 0\n')
        self.environment.update(ROS_DOMAIN_ID='24', ZENOH_SESSION_CONFIG_URI='/production/router.json5',
                                AMENT_PREFIX_PATH='/old/release', PYTHONPATH='/old/python')
        output = self.root / 'sdk-output'
        result = self.invoke('--scope', 'sdk', '--output', output, '--rmw-prefix', self.rmw,
                             '--sdk-vendor-root', self.vendor, '--python', python,
                             '--c-compiler', c_compiler, '--cxx-compiler', cxx_compiler, '--apply')
        self.assertEqual(result.returncode, 0, result.stderr)
        calls = [json.loads(line) for line in self.log.read_text().splitlines()]
        self.assertEqual(len(calls), 2)
        self.assertIn('d1max_planning_interfaces', calls[0]['argv'])
        self.assertIn('d1max_sdk_bridge', calls[1]['argv'])
        for call in calls:
            self.assertIsNone(call['env']['ROS_DOMAIN_ID'])
            self.assertIsNone(call['env']['ZENOH_SESSION_CONFIG_URI'])
            self.assertEqual(call['env']['RMW_IMPLEMENTATION'], 'rmw_zenoh_cpp')
            self.assertEqual(call['env']['ROS_DISTRO'], 'humble')
            self.assertEqual(call['env']['CMAKE_BUILD_PARALLEL_LEVEL'], '2')
            self.assertEqual(call['env']['CC'], str(c_compiler))
            self.assertEqual(call['env']['CXX'], str(cxx_compiler))
            base = call['argv'][call['argv'].index('--base-paths') + 1]
            self.assertTrue(Path(base).is_relative_to(output))
        copied = output / 'sdk-source/src/d1max_sdk_bridge/vendor/robot_sdk' / library.relative_to(self.vendor)
        self.assertEqual(copied.read_bytes(), library.read_bytes())
        self.assertFalse((self.nav / 'build').exists())
        self.assertFalse((self.app / 'sdk_bridge_ws/install').exists())
        self.assertTrue((output / 'nav-source/src/scan_planner_msgs/package.xml').is_file())


if __name__ == '__main__':
    unittest.main()
