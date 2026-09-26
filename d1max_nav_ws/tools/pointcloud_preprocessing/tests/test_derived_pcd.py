"""Only explicitly requested Z may change in the independent planning output."""
import sys
from pathlib import Path
import numpy as np
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from pointcloud_preprocessing.pcd_io import PcdError, read_pcd, write_derived_z
from test_pcd_io import pcd


def test_derived_z_preserves_xy_intensity_and_source(tmp_path):
    source = pcd(tmp_path)
    before = source.read_bytes()
    cloud = read_pcd(source)
    output = tmp_path/'flat.pcd'
    write_derived_z(output, cloud, [2, 0], [0., .125])
    result = read_pcd(output)
    for name in ('x', 'y', 'intensity'):
        assert result.records[name].tobytes() == cloud.records[name][[2, 0]].tobytes()
    np.testing.assert_array_equal(result.records['z'], [0., .125])
    assert result.metadata['viewpoint'] == [0, 0, 0, 1, 0, 0, 0]
    assert source.read_bytes() == before


@pytest.mark.parametrize('values', [[0], [0, float('nan')], [0, float('inf')], [0, 1e60]])
def test_bad_z_never_creates_file(tmp_path, values):
    cloud = read_pcd(pcd(tmp_path))
    output = tmp_path/'bad.pcd'
    with pytest.raises(PcdError):
        write_derived_z(output, cloud, [0, 1], values)
    assert not output.exists()


def test_never_overwrites_source_or_existing(tmp_path):
    source = pcd(tmp_path)
    cloud = read_pcd(source)
    with pytest.raises(PcdError):
        write_derived_z(source, cloud, [0], [0.])
    output = tmp_path/'output.pcd'
    output.write_bytes(b'preserve')
    with pytest.raises(FileExistsError):
        write_derived_z(output, cloud, [0], [0.])
    assert output.read_bytes() == b'preserve'
