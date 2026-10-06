"""A candidate must contain the Python bytes described by its source snapshot.

These tests use only temporary files; no ROS graph or native build is involved.
"""
import shutil

import pytest

import build_candidate


def fixture_package(tmp_path, name='d1max_pct_scan', python='python3.10'):
    source = tmp_path/'source'
    module = source/'src'/name/name
    module.mkdir(parents=True)
    (module/'__init__.py').write_text('')
    (module/'bt_adapters.py').write_text('CANCEL_SOURCE_COMPARISON = "exact_nanoseconds"\n')
    (module/'nested').mkdir()
    (module/'nested'/'__init__.py').write_text('')
    (module/'nested'/'worker.py').write_text('RETIRED = True\n')
    install = tmp_path/'install'
    runtime = install/name/'lib'/python/'site-packages'/name
    shutil.copytree(module, runtime)
    return source, install, module, runtime


def test_full_module_closure_matches_without_importing_python(tmp_path):
    source, install, module, runtime = fixture_package(tmp_path)
    # Executing/importing these modules would fail. The guard compares bytes.
    (module/'never_import.py').write_text('raise RuntimeError("do not import")\n')
    shutil.copyfile(module/'never_import.py', runtime/'never_import.py')
    result = build_candidate.verify_python_runtime_closure(source, install)
    assert result['contract'] == 'snapshot_installed_python_bytes_v1'
    assert result['packages']['d1max_pct_scan']['module_count'] == 5
    assert result['packages']['d1max_pct_scan']['runtime_locations'] == [
        'd1max_pct_scan/lib/python3.10/site-packages/d1max_pct_scan']


@pytest.mark.parametrize('relative', ['bt_adapters.py', 'nested/worker.py'])
def test_new_snapshot_old_copied_install_is_rejected(tmp_path, relative):
    source, install, module, runtime = fixture_package(tmp_path)
    (runtime/relative).write_text('STALE_INSTALLED_BYTES = True\n')
    with pytest.raises(ValueError, match='python_runtime_source_mismatch:d1max_pct_scan:'+relative):
        build_candidate.verify_python_runtime_closure(source, install)


def test_missing_nested_module_is_rejected(tmp_path):
    source, install, module, runtime = fixture_package(tmp_path)
    (runtime/'nested/worker.py').unlink()
    with pytest.raises(ValueError, match='missing=nested/worker.py'):
        build_candidate.verify_python_runtime_closure(source, install)


def test_leftover_module_removed_from_source_is_rejected(tmp_path):
    source, install, module, runtime = fixture_package(tmp_path)
    (runtime/'retired_controller.py').write_text('STALE = True\n')
    with pytest.raises(ValueError, match='unexpected=retired_controller.py'):
        build_candidate.verify_python_runtime_closure(source, install)


def test_missing_installed_package_is_rejected(tmp_path):
    source, install, module, runtime = fixture_package(tmp_path)
    shutil.rmtree(runtime)
    with pytest.raises(ValueError, match='python_runtime_package_missing:d1max_pct_scan'):
        build_candidate.verify_python_runtime_closure(source, install)


def test_missing_required_source_package_cannot_skip_guard(tmp_path):
    source, install, module, runtime = fixture_package(tmp_path, 'd1max_navigation')
    with pytest.raises(ValueError, match='python_runtime_source_package_missing:d1max_pct_scan'):
        build_candidate.verify_python_runtime_closure(source, install)


def test_other_navigation_python_packages_are_also_checked(tmp_path):
    source, install, module, runtime = fixture_package(tmp_path)
    other = source/'src/d1max_navigation/d1max_navigation'
    other.mkdir(parents=True)
    (other/'__init__.py').write_text('')
    (other/'lifecycle.py').write_text('RETIREMENT_CONFIRMED = False\n')
    destination = runtime.parent/'d1max_navigation'
    shutil.copytree(other, destination)
    (destination/'lifecycle.py').write_text('RETIREMENT_CONFIRMED = True\n')
    with pytest.raises(ValueError, match='python_runtime_source_mismatch:d1max_navigation:lifecycle.py'):
        build_candidate.verify_python_runtime_closure(source, install)


def test_every_python_runtime_location_must_match(tmp_path):
    source, install, module, runtime = fixture_package(tmp_path)
    second = install/'d1max_pct_scan/lib/python3.12/site-packages/d1max_pct_scan'
    shutil.copytree(module, second)
    assert len(build_candidate.verify_python_runtime_closure(source, install)
        ['packages']['d1max_pct_scan']['runtime_locations']) == 2
    (second/'bt_adapters.py').write_text('OLD_SECOND_RUNTIME = True\n')
    with pytest.raises(ValueError, match='python_runtime_source_mismatch:d1max_pct_scan:bt_adapters.py'):
        build_candidate.verify_python_runtime_closure(source, install)


def test_ament_cmake_python_dist_packages_layout_is_checked(tmp_path):
    source, install, module, runtime = fixture_package(tmp_path)
    dist = install/'local/lib/python3.10/dist-packages/d1max_pct_scan'
    shutil.copytree(module, dist)
    shutil.rmtree(runtime)
    assert build_candidate.verify_python_runtime_closure(source, install)['packages']
    (dist/'nested/worker.py').write_text('STALE_DIST_PACKAGE = True\n')
    with pytest.raises(ValueError, match='python_runtime_source_mismatch:d1max_pct_scan:nested/worker.py'):
        build_candidate.verify_python_runtime_closure(source, install)


def test_actual_symlink_bytes_are_checked(tmp_path):
    source, install, module, runtime = fixture_package(tmp_path)
    target = runtime/'bt_adapters.py'
    target.unlink()
    target.symlink_to(module/'bt_adapters.py')
    assert build_candidate.verify_python_runtime_closure(source, install)['packages']
    stale = tmp_path/'old_adapter.py'
    stale.write_text('STALE = True\n')
    target.unlink()
    target.symlink_to(stale)
    with pytest.raises(ValueError, match='python_runtime_source_mismatch:d1max_pct_scan:bt_adapters.py'):
        build_candidate.verify_python_runtime_closure(source, install)


def test_bytecode_is_not_part_of_copied_runtime_closure(tmp_path):
    source, install, module, runtime = fixture_package(tmp_path)
    (runtime/'__pycache__').mkdir()
    (runtime/'__pycache__/cache.py').write_text('IGNORED = True\n')
    (runtime/'bt_adapters.pyc').write_bytes(b'stale bytecode')
    assert build_candidate.verify_python_runtime_closure(source, install)['packages']


def test_uninstalled_source_tests_are_outside_runtime_module_closure(tmp_path):
    source, install, module, runtime = fixture_package(tmp_path)
    (module/'nested/tests').mkdir()
    (module/'nested/tests/test_worker.py').write_text('assert False\n')
    assert build_candidate.verify_python_runtime_closure(source, install)['packages']


def test_assemble_rejects_v14_failure_before_map_or_candidate_seal(tmp_path, monkeypatch):
    source, install, module, runtime = fixture_package(tmp_path)
    (runtime/'bt_adapters.py').write_text('CANCEL_SOURCE_COMPARISON = "strict_float"\n')
    repo = tmp_path/'repo'
    repo.mkdir()
    shutil.copytree(source, repo/'d1max_nav_ws')
    (repo/'d1max_nav_ws/src/scan_planner_vendor').mkdir()
    vendor = tmp_path/'vendor'
    vendor.mkdir()
    monkeypatch.setattr(build_candidate, 'REPO', repo)

    def map_must_not_run(*args, **kwargs):
        pytest.fail('stale Python candidate reached map generation')

    monkeypatch.setattr(build_candidate, 'build_map', map_must_not_run)
    output = tmp_path/'candidate'
    with pytest.raises(ValueError, match='python_runtime_source_mismatch:d1max_pct_scan:bt_adapters.py'):
        build_candidate.assemble(output, install, vendor, tmp_path/'scene.json')
    assert not (output/'map').exists()
    assert not (output/'release.json').exists()
    assert not (output/'isaac_candidate_integrity.json').exists()
