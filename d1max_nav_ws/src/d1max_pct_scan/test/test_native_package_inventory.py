from pathlib import Path

from d1max_pct_scan.single_floor_session import python_runtime_members


def test_package_inventory_includes_native_extension_and_excludes_build_inputs(tmp_path):
    package = tmp_path / 'd1max_pct_scan'
    package.mkdir()
    extension = package / '_mesh_ray_native.cpython-310-x86_64-linux-gnu.so'
    python = package / 'robot_self_geometry.py'
    nested = package / 'helpers'
    nested.mkdir()
    helper = nested / 'geometry.py'
    for path in (extension, python, helper, package / 'kernel.cpp', package / 'model.stl'):
        path.write_bytes(b'fixture')
    (package / 'not_a_library.so').mkdir()
    assert python_runtime_members(package) == {extension, python, helper}


def test_native_extension_inventory_changes_are_visible_to_same_release_check(tmp_path):
    source = tmp_path / 'robot_self_geometry.py'
    source.write_text('')
    before = python_runtime_members(tmp_path)
    extension = tmp_path / '_mesh_ray_native.cpython-310-x86_64-linux-gnu.so'
    extension.write_bytes(b'fixture')
    assert python_runtime_members(tmp_path) - before == {extension}
    extension.unlink()
    assert python_runtime_members(tmp_path) == before
