"""No native import or ROS graph; loader scopes must not leak between nodes."""
import importlib.util
import json
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


@pytest.mark.parametrize('rebuilt,expected',[
    ([], 'same_byte_copy_not_recompiled'),
    (['faster_lio'], 'source_rebuild_with_runtime_copy'),
])
def test_localization_build_claim_follows_actual_component_provenance(tmp_path,rebuilt,expected):
    prefix=tmp_path/'install'
    prefix.mkdir()
    (tmp_path/'runtime_copy_provenance.json').write_text(json.dumps(
        dict(rebuilt_packages=rebuilt,physical_acceptance=False)))
    assert seal.localization_build_claim(prefix)==expected


def test_external_localization_runtime_is_not_claimed_rebuilt():
    assert seal.localization_build_claim(None)=='external_selected_runtime'


def test_navigation_seal_does_not_depend_on_web_shell_or_built_ui(tmp_path):
    bindings = seal.navigation_launch_bindings(tmp_path)
    assert 'map_manager/backend/app.py' not in bindings
    assert not any('frontend' in name for name in bindings)
    # A normal UI rebuild removes old content-hashed assets. Neither that nor
    # unrelated Web routes can become a navigation startup dependency again.
    (tmp_path/'map_manager/frontend/dist').mkdir(parents=True)
    (tmp_path/'map_manager/frontend/dist/index.html').write_text('new UI')
    assert seal.navigation_launch_bindings(tmp_path) == bindings


def test_navigation_and_sdk_dispatchers_remain_integrity_bound(tmp_path):
    bindings = seal.navigation_launch_bindings(tmp_path)
    for relative in (
        'map_manager/backend/navigation.py',
        'map_manager/backend/single_floor.py',
        'map_manager/backend/mainline_release.py',
        'map_manager/backend/mainline_views.py',
        'map_manager/backend/navigation_session.py',
        'foxglove_d1max/scripts/start_sdk_monitor.sh',
        'foxglove_d1max/scripts/sdk_motion_startup.py',
        'd1max_ros2_env.sh',
    ):
        assert bindings[relative] == tmp_path/relative
