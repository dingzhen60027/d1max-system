"""Binary layout, raw-byte preservation and non-destructive IO contracts."""
import struct
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from pointcloud_preprocessing.pcd_io import PcdError, read_pcd, write_subset


def pcd(tmp_path, payload=None, **replacements):
    header = dict(VERSION="0.7", FIELDS="x y z intensity", SIZE="4 4 4 4",
                  TYPE="F F F F", COUNT="1 1 1 1", WIDTH="3", HEIGHT="1",
                  VIEWPOINT="1.25 -2 3.125 0.70710678 0 0 0.70710678", POINTS="3", DATA="binary")
    header.update(replacements)
    if payload is None:
        payload = struct.pack("<12f", 1,2,3,11, 4,5,6,22, 7,8,9,33)
    path = tmp_path / 'input.pcd'
    path.write_bytes(('\n'.join(f'{key} {value}' for key,value in header.items() if value is not None)+'\n').encode()+payload)
    return path


def test_xyzi_roundtrip_is_original_bytes_and_viewpoint(tmp_path):
    source = pcd(tmp_path)
    original = source.read_bytes()
    cloud = read_pcd(source)
    assert cloud.xyz.dtype == np.float64
    np.testing.assert_array_equal(cloud.xyz, [[1,2,3],[4,5,6],[7,8,9]])
    assert not cloud.records.flags.writeable
    output = tmp_path/'output.pcd'
    assert write_subset(output,cloud,[2,0]) == ['x','y','z','intensity']
    result=read_pcd(output)
    assert result.records.tobytes() == cloud.records[[2,0]].tobytes()
    assert result.metadata['header']['VIEWPOINT'] == cloud.metadata['header']['VIEWPOINT']
    assert source.read_bytes() == original


def test_mixed_sizes_counts_order_and_padding_preserved(tmp_path):
    dtype=np.dtype([('tag','u1'),('x','<f8'),('histogram','<u2',(3,)),('y','<f4'),
                    ('_','u1',(3,)),('z','<f8'),('signed','<i8')])
    records=np.zeros(3,dtype=dtype)
    records['tag']=[1,2,3]
    records['x']=[.1,.2,.3]
    records['y']=[1,2,3]
    records['z']=[-2,-3,-4]
    records['histogram']=[[6,7,8],[1,2,3],[65535,0,555]]
    records['_']=[[111,222,33]]*3
    records['signed']=[-5,2**60,-2**60]
    source=pcd(tmp_path,records.tobytes(),FIELDS='tag x histogram y _ z signed',
               SIZE='1 8 2 4 1 8 8',TYPE='U F U F U F I',COUNT='1 1 3 1 3 1 1')
    cloud=read_pcd(source)
    assert cloud.records.dtype == dtype
    output=tmp_path/'subset.pcd'
    write_subset(output,cloud,[True,False,True])
    result=read_pcd(output)
    assert result.records.tobytes() == records[[0,2]].tobytes()
    assert result.metadata['header']['COUNT'] == ['1','1','3','1','3','1','1']


def test_finite_mask_never_changes_nan_inf_or_nan_payload(tmp_path):
    # Noncanonical NaN bit-patterns must survive an unfiltered subset unchanged.
    words=np.asarray([0x7FC00071,0x40000000,0x40400000,0x7FC01234,
                      0x3F800000,0x7F800000,0x40400000,0x3F800000,
                      0x3F800000,0x40000000,0x40400000,0x3F800000],dtype='<u4')
    cloud=read_pcd(pcd(tmp_path,words.tobytes()))
    np.testing.assert_array_equal(cloud.finite_xyz_mask,[False,False,True])
    output=tmp_path/'all.pcd'
    write_subset(output,cloud,[0,1,2])
    assert read_pcd(output).records.tobytes() == words.tobytes()
    filtered=tmp_path/'finite.pcd'
    write_subset(filtered,cloud,cloud.finite_xyz_mask)
    assert read_pcd(filtered).records.tobytes() == words[8:].tobytes()


def test_zero_subset_roundtrip(tmp_path):
    cloud=read_pcd(pcd(tmp_path))
    output=tmp_path/'empty.pcd'
    write_subset(output,cloud,[])
    result=read_pcd(output)
    assert result.xyz.shape == (0,3)
    assert len(result.records) == 0
    assert result.metadata['width'] == 0 and result.metadata['height'] == 1


def test_organized_source_is_checked_then_subset_unorganized(tmp_path):
    cloud=read_pcd(pcd(tmp_path,WIDTH='1',HEIGHT='3'))
    output=tmp_path/'subset.pcd'
    write_subset(output,cloud,[1,2])
    result=read_pcd(output)
    assert result.metadata['width'] == 2 and result.metadata['height'] == 1


def test_optional_count_viewpoint_and_version_defaults(tmp_path):
    cloud=read_pcd(pcd(tmp_path,COUNT=None,VIEWPOINT=None,VERSION=None))
    assert cloud.metadata['header']['COUNT'] == ['1']*4
    assert cloud.metadata['viewpoint'] == [0,0,0,1,0,0,0]


@pytest.mark.parametrize('changes',[
    {'SIZE':'4 4'}, {'TYPE':'F F F'}, {'COUNT':'1 1 1'}, {'COUNT':'1 1 1 0'},
    {'WIDTH':'2'}, {'HEIGHT':'0'}, {'POINTS':'-1'}, {'FIELDS':'x x z intensity'},
    {'FIELDS':'x y z invalid-name'}, {'FIELDS':'x y intensity something'},
    {'SIZE':'2 4 4 4'}, {'TYPE':'Q F F F'}, {'COUNT':'2 1 1 1'},
    {'VIEWPOINT':'0 0 0 1 0 0'}, {'VIEWPOINT':'nan 0 0 1 0 0 0'},
    {'VERSION':'0.6'}, {'DATA':'ascii'}, {'DATA':'binary_compressed'},
])
def test_invalid_headers_rejected(tmp_path,changes):
    with pytest.raises(PcdError):
        read_pcd(pcd(tmp_path,**changes))


@pytest.mark.parametrize('payload',[b'\0'*47,b'\0'*49])
def test_truncated_or_trailing_payload_rejected(tmp_path,payload):
    with pytest.raises(PcdError,match='payload length'):
        read_pcd(pcd(tmp_path,payload))


def test_duplicate_header_entry_rejected(tmp_path):
    source=pcd(tmp_path)
    source.write_bytes(b'WIDTH 3\n'+source.read_bytes())
    with pytest.raises(PcdError,match='Duplicate'):
        read_pcd(source)


@pytest.mark.parametrize('indices',[[1.0],[0,0],[-1],[3],[[0]],True,[True,False]])
def test_invalid_subset_rejected_before_creating_output(tmp_path,indices):
    cloud=read_pcd(pcd(tmp_path))
    output=tmp_path/'invalid.pcd'
    with pytest.raises(PcdError):
        write_subset(output,cloud,indices)
    assert not output.exists()


def test_source_existing_file_and_symlink_are_never_overwritten(tmp_path):
    source=pcd(tmp_path)
    cloud=read_pcd(source)
    before=source.read_bytes()
    with pytest.raises(PcdError,match='differ'):
        write_subset(source,cloud,[0])
    output=tmp_path/'exists.pcd'
    output.write_bytes(b'keep-me')
    with pytest.raises(FileExistsError):
        write_subset(output,cloud,[0])
    assert output.read_bytes() == b'keep-me'
    link=tmp_path/'source-link.pcd'
    link.symlink_to(source)
    with pytest.raises(PcdError,match='differ'):
        write_subset(link,cloud,[0])
    dangling=tmp_path/'dangling.pcd'
    dangling.symlink_to(tmp_path/'missing.pcd')
    with pytest.raises(FileExistsError):
        write_subset(dangling,cloud,[0])
    assert source.read_bytes() == before
