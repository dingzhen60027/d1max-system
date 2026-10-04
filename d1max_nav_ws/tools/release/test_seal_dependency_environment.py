"""No native import or ROS graph; loader scopes must not leak between nodes."""
import importlib.util
from pathlib import Path
import sys

import pytest

ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT))
spec=importlib.util.spec_from_file_location('seal_single_floor_release',ROOT/'seal_single_floor_release.py')
seal=importlib.util.module_from_spec(spec);spec.loader.exec_module(seal)


@pytest.mark.parametrize('library',[
    '/release/pct_vendor/planner/lib/libgpmp_optimizer.so',
    '/release/pct_vendor/planner/lib/a_star.cpython-310-x86_64-linux-gnu.so',
    '/workspace/gtsam/install/lib/libgtsam.so.4.1.1',
    '/workspace/gtsam/install/lib/libmetis-gtsam.so',
])
def test_pct_native_and_its_actual_gtsam_dependencies_use_child_loader_environment(library):
    native=dict(native_lib_dir='/release/pct_vendor/planner/lib',gtsam_lib_dir='/workspace/gtsam/install/lib')
    environment=dict(LD_LIBRARY_PATH='/workspace/gtsam/install/lib:/release/pct_vendor/planner/lib:/ros/lib')
    assert seal.elf_environment(library,native,environment) is environment


@pytest.mark.parametrize('library',[
    '/release/native/install/scan_planner/lib/scan_planner/scan_planner_node',
    '/release/sdk/install/d1max_sdk_bridge/lib/d1max_sdk_bridge/sdk_monitor_bridge',
    '/release/application/install/d1max_localization/lib/d1max_localization/fused_icp_matcher',
    '/ros/lib/x86_64-linux-gnu/libgtsam.so.4.2.0',
    '/release/pct_vendor/planner/lib-foreign/libgpmp_optimizer.so',
])
def test_non_pct_launchers_keep_real_base_loader_environment(library):
    native=dict(native_lib_dir='/release/pct_vendor/planner/lib',gtsam_lib_dir='/workspace/gtsam/install/lib')
    assert seal.elf_environment(library,native,dict(LD_LIBRARY_PATH='/pct-only')) is None
