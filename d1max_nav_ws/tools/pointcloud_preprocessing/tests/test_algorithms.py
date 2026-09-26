from copy import deepcopy
from pathlib import Path
import json

import numpy as np
import pytest
from scipy.spatial import cKDTree
import yaml

from pointcloud_preprocessing.algorithms import filter_indices, protect_structures
from pointcloud_preprocessing.runner import validate, run, sha256
from pointcloud_preprocessing.pcd_io import read_pcd


CONFIG = Path(__file__).resolve().parents[1] / 'configs/sc_pgo_0919_structure.yaml'


def config():
    return yaml.safe_load(CONFIG.read_text())


def modules():
    return config()['modules']


def surface():
    x,y=np.meshgrid(np.arange(-.4,.41,.05),np.arange(-.4,.41,.05))
    return np.c_[x.ravel(),y.ravel(),np.zeros(x.size)]


def test_isolated_spike_above_flat_floor_not_rescued_by_flat_neighbors():
    floor=surface()
    xyz=np.vstack([floor,[0,0,.18],[3,3,3]])
    result=filter_indices(xyz,modules(),workers=1)
    assert set(result['removed_indices'])=={len(floor),len(floor)+1}
    assert len(result['kept_indices'])==len(floor)
    np.testing.assert_array_equal(xyz[result['kept_indices']],floor)


@pytest.mark.parametrize('rotation', [np.eye(3),np.array([[0,0,1],[0,1,0],[-1,0,0]]),
                                      np.array([[1,0,0],[0,.8,-.6],[0,.6,.8]])])
def test_sparse_measured_floors_walls_and_slopes_are_protected_without_flattening(rotation):
    xyz=surface()[::2]@rotation.T
    cfg=modules()
    cfg[0]['parameters']['radius_m']=.025
    result=filter_indices(xyz,cfg,workers=1)
    assert len(result['kept_indices'])==len(xyz)
    assert result['statistics']['protected_candidate_union']==len(xyz)


def test_thin_line_has_a_separate_geometric_protection():
    xyz=np.c_[np.zeros(31),np.zeros(31),np.linspace(-.75,.75,31)]
    cfg=modules()
    cfg[0]['parameters']['radius_m']=.025
    cfg[2]['parameters']['minimum_neighbors']=6
    result=filter_indices(xyz,cfg,workers=1)
    assert len(result['kept_indices'])==len(xyz)
    assert result['statistics']['line_supported_candidates']==len(xyz)


def test_no_semantic_deletion_of_a_dense_object_above_ground():
    floor=surface()
    x,y,z=np.meshgrid(np.arange(4)*.03,np.arange(4)*.03,np.arange(4)*.03)
    object_points=np.c_[x.ravel()+.1,y.ravel()+.1,z.ravel()+.5]
    result=filter_indices(np.vstack([floor,object_points]),modules(),workers=1)
    assert set(range(len(floor),len(floor)+len(object_points)))<=set(result['kept_indices'])


def test_nonfinite_preserved_in_removed_partition_and_neighbors_exclude_self():
    xyz=np.array([[0.,0,0],[1.,1,1],[np.nan,0,0]])
    result=filter_indices(xyz,modules(),workers=1)
    assert len(result['kept_indices'])==0
    assert result['removed_reasons'].tolist()==[2,2,1]
    assert result['statistics']['radius_count_excludes_self']


def test_disabled_filters_keep_all_finite_points():
    cfg=modules()
    cfg[0]['enabled']=cfg[1]['enabled']=False
    result=filter_indices(np.array([[0.,0,0],[1,2,3]]),cfg,workers=1)
    assert result['kept_indices'].tolist()==[0,1]


def test_protection_has_spatial_radius_not_infinite_planar_extrapolation():
    xyz=np.vstack([surface(),[4,4,0]])
    tree=cKDTree(xyz)
    candidates=np.zeros(len(xyz),bool);candidates[-1]=True
    cfg={**modules()[2]['parameters'],'enabled':True}
    a,b=protect_structures(xyz,tree,candidates,cfg,workers=1)
    assert not a.any() and not b.any()


@pytest.mark.parametrize('key,value', [('radius_m',float('nan')),('radius_m',-1),
                                      ('minimum_other_neighbors',True),('minimum_other_neighbors',2.3)])
def test_bad_parameters_fail_closed(key,value):
    cfg=config();cfg['modules'][0]['parameters'][key]=value
    with pytest.raises(ValueError): validate(cfg)


def test_unknown_module_parameter_is_not_silently_ignored():
    cfg=config();cfg['modules'][0]['parameters']['bad_key']=1
    with pytest.raises(ValueError): validate(cfg)


def test_field_exact_end_to_end_catalog_artifact_is_an_independent_version(tmp_path):
    points=np.vstack([surface(),[4.,4.,4.]])
    records=np.c_[points,np.arange(len(points))+.25].astype('<f4')
    source=tmp_path/'raw.pcd'
    header=('VERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n'
            f'WIDTH {len(records)}\nHEIGHT 1\nVIEWPOINT 1 2 3 1 0 0 0\nPOINTS {len(records)}\nDATA binary\n')
    source.write_bytes(header.encode()+records.tobytes())
    cfg=config();cfg['input']['path']=str(source);cfg['input']['sha256']=sha256(source)
    cfg['output']['root']=str(tmp_path/'processed');cfg['runtime']['workers']=1
    config_path=tmp_path/'pipeline.yaml';config_path.write_text(yaml.safe_dump(cfg))
    original=source.read_bytes()
    output,manifest=run(config_path)
    assert manifest['status']=='complete' and manifest['removed_points']==1
    assert source.read_bytes()==original
    cloud=read_pcd(output/'processed_map.pcd')
    assert cloud.records.tobytes()==records[:-1].tobytes()
    assert read_pcd(output/'audit/removed_points.pcd').records.tobytes()==records[-1:].tobytes()
    assert json.loads((output/'manifest.json').read_text())['source_unchanged']
    second,_=run(config_path)
    assert second!=output
    cfg['input']['sha256']='0'*64;config_path.write_text(yaml.safe_dump(cfg))
    with pytest.raises(ValueError,match='Source PCD changed'): run(config_path)
