from pathlib import Path

import pytest

from d1max_pct_planner import native_runtime as runtime


def write(path, content):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content)
    return path


@pytest.fixture
def vendor(tmp_path):
    root = tmp_path / 'vendor'
    lib = root / 'planner/lib'
    install = lib / '3rdparty/gtsam-4.1.1/install'
    write(install / 'include/gtsam/config.h', '#define GTSAM_VERSION_STRING "4.1.1"\n')
    write(install / 'lib/libgtsam.so.4.1.1', 'unique gtsam 4.1.1')
    (install / 'lib/libgtsam.so').symlink_to('libgtsam.so.4.1.1')
    write(install / 'lib/libmetis-gtsam.so', 'metis')
    target = lib / 'build/src/trajectory_optimization'
    write(target / 'CMakeFiles/gpmp_optimizer.dir/link.txt',
          'c++ ../../../3rdparty/gtsam-4.1.1/install/lib/libgtsam.so.4.1.1 -shared')
    write(target / 'CMakeFiles/gpmp_optimizer.dir/flags.make', f'CXX_INCLUDES = -I{install}/include')
    for name, directory in runtime._NATIVE_BUILD_DIRS.items():
        write(lib / name, 'binary:' + name)
        write(lib / 'build/src' / directory / name, 'binary:' + name)
    return root


def mapping(path, *, deleted=False, inode=None):
    return (f'123-456 r-xp 00000 00:00 {path.stat().st_ino if inode is None else inode} '
            f'{path}' + (' (deleted)' if deleted else '') + '\n')


def all_mappings(evidence):
    paths = [evidence['gtsam_library'], evidence['metis_library'],
             *evidence['native_libraries'].values()]
    return ''.join(mapping(Path(path)) for path in paths)


def test_prepared_environment_does_not_mutate_parent_or_change_zenoh(vendor):
    source = {'LD_LIBRARY_PATH': '/opt/ros/humble/lib::/system',
              'RMW_IMPLEMENTATION': 'rmw_zenoh_cpp', 'ROBOT_ADDRESS': 'untouched'}
    expected = source.copy()
    result = runtime.prepare_native_environment(vendor, source)
    evidence = runtime.vendorverify(vendor)
    assert source == expected
    assert result['RMW_IMPLEMENTATION'] == source['RMW_IMPLEMENTATION']
    assert result['ROBOT_ADDRESS'] == 'untouched'
    assert result['LD_LIBRARY_PATH'].split(':') == [evidence['gtsam_lib_dir'],
                                                   evidence['native_lib_dir'],
                                                   '/opt/ros/humble/lib', '/system']
    assert runtime.prepare_native_environment(vendor, result) == result


def test_compiled_version_and_link_evidence(vendor):
    evidence = runtime.vendorverify(vendor)
    assert evidence['gtsam_version'] == '4.1.1'
    assert evidence['gtsam_library'].endswith('libgtsam.so.4.1.1')


def test_missing_build_evidence_fails_closed(vendor):
    (vendor / 'planner/lib/build/src/trajectory_optimization/CMakeFiles/gpmp_optimizer.dir/link.txt').unlink()
    with pytest.raises(runtime.NativeRuntimeError, match='missing'):
        runtime.vendorverify(vendor)


def test_wrong_compile_link_library_fails_closed(vendor):
    target = vendor / 'planner/lib/build/src/trajectory_optimization'
    wrong = write(vendor / 'libgtsam.so.4.2.0', 'wrong')
    write(target / 'CMakeFiles/gpmp_optimizer.dir/link.txt', f'c++ {wrong}')
    with pytest.raises(runtime.NativeRuntimeError, match='compile/link'):
        runtime.vendorverify(vendor)


def test_changed_installed_binary_fails_closed(vendor):
    write(vendor / 'planner/lib/libgpmp_optimizer.so', 'stale installed copy')
    with pytest.raises(runtime.NativeRuntimeError, match='differs'):
        runtime.vendorverify(vendor)


def test_before_import_allows_absent_but_after_import_requires_actual_libraries(vendor):
    assert not runtime.probe_native_libraries(vendor, maps_text='')['runtime_verified']
    with pytest.raises(runtime.NativeRuntimeError) as raised:
        runtime.probe_native_libraries(vendor, require_loaded=True, maps_text='')
    assert raised.value.code == 'native_runtime_unverified'
    assert raised.value.details['relaunch_required']


def test_correct_process_mappings_pass(vendor):
    evidence = runtime.vendorverify(vendor)
    result = runtime.probe_native_libraries(vendor, require_loaded=True,
                                            maps_text=all_mappings(evidence))
    assert result['runtime_verified']
    assert not result['missing_libraries']


def test_ros_42_mapping_rejected_even_before_import(vendor, tmp_path):
    wrong = write(tmp_path / 'ros/libgtsam.so.4.2.0', 'wrong ABI')
    with pytest.raises(runtime.NativeRuntimeError) as raised:
        runtime.probe_native_libraries(vendor, maps_text=mapping(wrong))
    assert raised.value.code == 'native_abi_mismatch'
    assert raised.value.details['relaunch_required']
    assert raised.value.details['actual'] == str(wrong)


@pytest.mark.parametrize('change', ['deleted', 'inode'])
def test_replaced_or_deleted_mapped_library_rejected(vendor, change):
    path = Path(runtime.vendorverify(vendor)['gtsam_library'])
    text = mapping(path, deleted=change == 'deleted',
                   inode=path.stat().st_ino + 1 if change == 'inode' else None)
    with pytest.raises(runtime.NativeRuntimeError, match='relaunch'):
        runtime.probe_native_libraries(vendor, maps_text=text)


def test_scoped_experimental_lib_rejected(vendor, tmp_path):
    copy = write(tmp_path / 'experiment/libmap_manager.so', 'experimental build')
    with pytest.raises(runtime.NativeRuntimeError, match='relaunch'):
        runtime.probe_native_libraries(vendor, maps_text=mapping(copy))


def test_conflicting_preload_rejected(vendor):
    with pytest.raises(runtime.NativeRuntimeError, match='LD_PRELOAD'):
        runtime.prepare_native_environment(vendor, {'LD_PRELOAD': '/opt/ros/libgtsam.so.4'})
