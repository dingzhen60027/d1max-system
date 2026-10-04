"""A movable release must bind its own actual artifacts before ROS startup."""
import importlib.util
import json
from pathlib import Path
import shutil

import numpy as np
import pytest
import yaml

from d1max_pct_scan.source_identity import validate_package
from d1max_pct_scan.source_route import digest_file


def package(tmp_path):
    root = tmp_path/'release'; directory = root/'map/floor'
    directory.mkdir(parents=True)
    (root/'release.json').write_text(json.dumps(dict(schema=1, sealed_manifest='seal.json')))
    xyz = np.array([[0.,0.,0.],[.1,0.,0.],[0.,.1,0.],[.1,.1,0.]],dtype='<f4')
    pcd = (b'VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
           b'WIDTH 4\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS 4\nDATA binary\n')+xyz.tobytes()
    source = tmp_path/'original.pcd'; source.write_bytes(pcd)
    (directory/'processed_map.pcd').write_bytes(pcd)
    np.savez_compressed(directory/'source_indices.npz',selected_source_indices=np.arange(4),
        floor1_support_source_indices=np.arange(4),floor1_support_xyz=xyz.astype(float))
    manifest = dict(schema='d1max.source_identity_floor/v1',status='complete',xyz_modified=False,
        geometry_operation='source_identity',frame_id='d1max_loc_map',floor_id='floor1',
        source_coordinate_error_m=0.,source_path=str(source),source_sha256=digest_file(source),
        output_file='processed_map.pcd',output_sha256=digest_file(directory/'processed_map.pcd'),
        source_indices_file='source_indices.npz',source_indices_sha256=digest_file(directory/'source_indices.npz'),
        forbidden_stair_xy=[[5.,5.],[6.,6.]],physical_acceptance=False)
    (directory/'manifest.json').write_text(json.dumps(manifest))
    data = np.zeros((5,2,3,3),np.float32); data[4]=2.; data[4,0,0,0]=np.nan
    np.savez_compressed(directory/'tomogram.npz',data=data,resolution=.1,center=np.array([0.,0.]),
        slice_h0=0.,slice_dh=.5,source_pcd=str(directory/'processed_map.pcd'),
        source_sha256=manifest['output_sha256'],frame_id='d1max_loc_map',
        source_processing_manifest=str(directory/'manifest.json'),
        source_processing_manifest_sha256=digest_file(directory/'manifest.json'),
        original_source_pcd=str(source),original_source_sha256=manifest['source_sha256'],
        geometry_operation='source_identity',planning_only=False)
    route = dict(schema='d1max.source_identity_route/v1',frame_id='d1max_loc_map',floor_id='floor1',
        stairs_enabled=False,source_pcd=str(directory/'processed_map.pcd'),
        tomogram_path=str(directory/'tomogram.npz'),vendor_root=str(tmp_path),
        unknown_ceiling_policy='reject',minimum_headroom_m=.55,limits=dict(max_ground_step_m=.17))
    (directory/'route.yaml').write_text(yaml.safe_dump(route))
    return root,directory


def tool():
    target=Path(__file__).resolve().parents[3]/'tools/map/rebase_source_identity_package.py'
    spec=importlib.util.spec_from_file_location('rebase_source_identity_package',target)
    module=importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    return module


def test_small_real_original_records_and_tomogram_package_validate_without_ros(tmp_path):
    _,directory=package(tmp_path)
    bound=validate_package(directory)
    assert bound['bridge'].manifest['physical_acceptance'] is False
    assert bound['tomogram'].unknown_ceiling_policy=='reject'
    assert bound['route']['source_pcd']==str(directory/'processed_map.pcd')


def test_copied_absolute_route_fails_before_node_launch_then_explicit_rebase_preserves_geometry(tmp_path,monkeypatch):
    old,original=package(tmp_path)
    new=tmp_path/'new'; shutil.copytree(old,new)
    copied=new/'map/floor'
    with pytest.raises(ValueError,match='single_floor_source_identity_map_binding_invalid'):
        validate_package(copied)
    module=tool()
    monkeypatch.setattr(module,'vendorverify',lambda root:dict(native_libraries={}))
    monkeypatch.setattr(module,'prepare_native_environment',lambda root:{})
    before={name:digest_file(copied/name) for name in ('manifest.json','processed_map.pcd','source_indices.npz')}
    result=module.rebase(new,copied,vendor_root=tmp_path)
    assert not result['geometry_changed'] and not result['acceptance_changed']
    assert result['unchanged_files']==before
    assert result['before']['tomogram_sha256']!=result['after']['tomogram_sha256']
    assert result['after']['route_paths']['tomogram_path']==str(copied/'tomogram.npz')
    assert 'data' in result['unchanged_array_identity']
    assert validate_package(copied)['bridge'].manifest['physical_acceptance'] is False
    assert validate_package(original) # old package was never touched


@pytest.mark.parametrize('bad',['source','indices','manifest_binding','foreign_manifest','foreign_source','floor'])
def test_package_hash_or_path_mismatch_is_not_bypassed(bad,tmp_path):
    _,directory=package(tmp_path)
    if bad in ('source','indices'):
        path=directory/('processed_map.pcd' if bad=='source' else 'source_indices.npz')
        path.write_bytes(path.read_bytes()+b'tamper')
    elif bad=='floor':
        route=yaml.safe_load((directory/'route.yaml').read_text());route['floor_id']='floor2'
        (directory/'route.yaml').write_text(yaml.safe_dump(route))
    else:
        with np.load(directory/'tomogram.npz',allow_pickle=False) as archive:
            payload={k:archive[k].copy() for k in archive.files}
        if bad=='manifest_binding':payload['source_processing_manifest_sha256']=np.asarray('a'*64)
        elif bad=='foreign_manifest':payload['source_processing_manifest']=np.asarray(str(tmp_path/'other.json'))
        else:payload['source_pcd']=np.asarray(str(tmp_path/'other.pcd'))
        np.savez_compressed(directory/'tomogram.npz',**payload)
    with pytest.raises(ValueError): validate_package(directory)


def test_rebase_refuses_sealed_release_or_different_copied_bytes(tmp_path,monkeypatch):
    release,directory=package(tmp_path)
    module=tool()
    (release/'seal.json').write_text('{}')
    with pytest.raises(ValueError,match='sealed_release'):module.rebase(release,directory)
    unsealed=tmp_path/'new';shutil.copytree(release,unsealed)
    (unsealed/'seal.json').unlink()
    payload=(unsealed/'map/floor/tomogram.npz').read_bytes()
    (unsealed/'map/floor/tomogram.npz').write_bytes(payload+b'changed-archive')
    with pytest.raises(ValueError,match='copied_route_artifact_bytes_differ'):
        module.rebase(unsealed,unsealed/'map/floor')
