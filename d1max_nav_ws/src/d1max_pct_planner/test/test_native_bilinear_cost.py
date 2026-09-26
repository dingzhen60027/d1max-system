"""Compile the real map-manager source in isolation; never replace native libs."""
from pathlib import Path
import shutil
import subprocess

import pytest


@pytest.fixture(scope='module')
def bilinear_probe(tmp_path_factory):
    compiler = shutil.which('c++')
    eigen = Path('/usr/include/eigen3')
    if compiler is None or not (eigen / 'Eigen/Core').is_file():
        pytest.skip('Standalone native interpolation regression needs C++ and Eigen3 headers')
    source_root = Path(__file__).resolve().parents[2]
    native = source_root / 'pct_planner_vendor/planner/lib/src'
    fixture = Path(__file__).with_name('native') / 'test_dense_elevation_bilinear.cc'
    executable = tmp_path_factory.mktemp('native_bilinear') / 'bilinear_probe'
    built = subprocess.run([
        compiler, '-std=c++14', '-O1', '-Wall', '-Wextra', '-pedantic',
        '-I' + str(eigen), '-I' + str(native), str(fixture),
        str(native / 'map_manager/dense_elevation_map.cc'), '-o', str(executable),
    ], capture_output=True, text=True, timeout=60)
    assert built.returncode == 0, built.stdout + built.stderr
    return executable


@pytest.mark.parametrize('mode', ['safe', 'nominal'])
@pytest.mark.parametrize('case', [
    'constant', 'plane', 'peak', 'continuity', 'boundary', 'gradient', 'layers', 'nonfinite',
])
def test_native_integer_cell_bilinear_contract(bilinear_probe, case, mode):
    result = subprocess.run([str(bilinear_probe), case, mode],
                            capture_output=True, text=True, timeout=5)
    assert result.returncode == 0, result.stdout + result.stderr
    assert f'PASS {case} {mode}' in result.stdout
