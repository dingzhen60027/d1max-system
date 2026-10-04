"""Exercise installed code from an unrelated CWD, without ROS or tools imports."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest


@pytest.fixture(scope='module')
def packaged_helpers(tmp_path_factory):
    package = Path(__file__).resolve().parents[1]
    root = package.parents[1]
    temporary = tmp_path_factory.mktemp('actual_pointcloud_package')
    build = temporary/'copy_install'
    env = os.environ.copy()
    env.pop('PYTHONPATH', None)
    env['PYTHONDONTWRITEBYTECODE'] = '1'
    subprocess.run([sys.executable, 'setup.py', 'build_py', '--build-lib', str(build)],
                   cwd=package, env=env, check=True, capture_output=True, text=True)
    # A copy install, not a link back to live editable source. Every helper is
    # the original implementation; no reduced PCD parser is tested here.
    original = root/'tools/pointcloud_preprocessing'
    installed = build/'d1max_pct_scan/pointcloud_helpers'
    for source in original.glob('*.py'):
        target = installed/source.name
        assert target.is_file() and not target.is_symlink()
        assert target.read_bytes() == source.read_bytes()
    return temporary, build, env


def run_copy(packaged_helpers, program, *args):
    temporary, build, environment = packaged_helpers
    environment = dict(environment, PYTHONPATH=str(build))
    result = subprocess.run([sys.executable, '-c', program, *map(str, args)],
                            cwd=temporary, env=environment, check=True,
                            capture_output=True, text=True)
    return json.loads(result.stdout)


def test_copy_installed_helpers_keep_relative_dependencies_and_no_tools(packaged_helpers):
    result = run_copy(packaged_helpers, '''
import importlib.util,json,sys
from pathlib import Path
from d1max_pct_scan.pointcloud_helpers import pcd_io,ground_path_bridge,flat_floor
assert importlib.util.find_spec('tools') is None
assert ground_path_bridge.read_pcd is pcd_io.read_pcd
assert ground_path_bridge._field is flat_floor._field
assert not any(key=='tools' or key.startswith('tools.') for key in sys.modules)
print(json.dumps({'path':str(Path(pcd_io.__file__).resolve())}))
''')
    assert '/copy_install/d1max_pct_scan/pointcloud_helpers/pcd_io.py' in result['path']


def test_real_source_identity_loads_original_binary_records_without_workspace_cwd(packaged_helpers):
    temporary, _, _ = packaged_helpers
    dtype = np.dtype([('x','<f4'),('y','<f4'),('z','<f4'),('intensity','<u2')])
    points = np.zeros(4,dtype=dtype)
    points['x'] = [0.,.125,.25,.375]
    points['intensity'] = [13,37,59,83]
    def binary(path, records):
        n = len(records)
        header = ('VERSION .7\nFIELDS x y z intensity\nSIZE 4 4 4 2\n'
                  'TYPE F F F U\nCOUNT 1 1 1 1\n'
                  f'WIDTH {n}\nHEIGHT 1\nPOINTS {n}\nDATA binary\n')
        path.write_bytes(header.encode('ascii') + records.tobytes())
    source, output, indices = (temporary/name for name in ('source.pcd','floor1.pcd','indices.npz'))
    binary(source, points)
    binary(output, points[[0,2]])
    support = np.column_stack([points[name][[0,1]] for name in ('x','y','z')])
    np.savez(indices, floor1_support_xyz=support,
             floor1_support_source_indices=np.array([0,1]), selected_source_indices=np.array([0,2]))
    digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    manifest = temporary/'manifest.json'
    manifest.write_text(json.dumps(dict(schema='d1max.source_identity_floor/v1', status='complete',
        xyz_modified=False, geometry_operation='source_identity', frame_id='d1max_loc_map',
        floor_id='floor1', source_coordinate_error_m=0., source_path=str(source),
        source_sha256=digest(source), output_file=output.name, output_sha256=digest(output),
        source_indices_file=indices.name, source_indices_sha256=digest(indices),
        forbidden_stair_xy=[[5.,5.],[6.,6.]])))
    result = run_copy(packaged_helpers, '''
import importlib.util,json,sys
from d1max_pct_scan.source_identity import SourceIdentityBridge
from d1max_pct_scan.pointcloud_helpers.pcd_io import read_pcd
assert importlib.util.find_spec('tools') is None
bridge=SourceIdentityBridge.from_artifacts(sys.argv[1])
cloud=read_pcd(sys.argv[2])
assert cloud.records['intensity'].tolist()==[13,59]
print(json.dumps({'support':bridge.support.tolist(),'dtype':cloud.records.dtype.descr}))
''', manifest, output)
    assert result['support'] == support.tolist()
    assert result['dtype'][-1] == ['intensity', '<u2']
