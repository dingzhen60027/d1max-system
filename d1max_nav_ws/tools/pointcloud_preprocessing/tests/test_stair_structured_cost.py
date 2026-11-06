import copy

import numpy as np
import pytest

from d1max_pct_planner.cpu_tomography import tomogram_from_points, traversal_cost, _inflate, SENTINEL
from tools.pointcloud_preprocessing.stair_structured_support import build_structured_stair_support
from tools.pointcloud_preprocessing.stair_structured_cost import apply_structured_stair_cost


def fixture(*, hole=False, box=False):
    x,y = np.meshgrid(np.arange(-.70,.701,.04),np.arange(.04,3.961,.04))
    floor = np.c_[x.ravel(),y.ravel(),.5*y.ravel()+.015*np.sin(x.ravel()*6+y.ravel()*7)]
    if hole: floor=floor[(floor[:,1]<1.3)|(floor[:,1]>1.8)]
    sy,dz=np.meshgrid(np.arange(.02,3.99,.04),np.arange(.26,1.24,.04))
    walls=[np.c_[np.full(sy.size,side),sy.ravel(),.5*sy.ravel()+dz.ravel()] for side in (-.75,.75)]
    parts=[floor,*walls,floor+[0,0,2.]]
    if box:
        bx,by=np.meshgrid(np.arange(-.12,.13,.04),np.arange(1.9,2.13,.04))
        parts.append(np.c_[bx.ravel(),by.ravel(),.5*by.ravel()+.25])
    cloud=np.concatenate(parts).astype(np.float32).astype(float)
    ids=np.flatnonzero(abs(floor[:,0])<.24).astype(np.int64)
    mapping=dict(source_indices=ids,source_xyz=cloud[ids].copy(),planning_xyz=cloud[ids].copy(),surface_id=np.ones(len(ids),np.int32))
    models=[dict(id=1,name='lower',kind='flight_support_envelope',leg='stair_lower',center_xy=[0.,2.],
        uphill_axis_xy=[0.,1.],grade=.5,intercept_m=1.,along_range_m=[-1.94,1.94],
        interior_lateral_range_m=[-.24,.24],scope_minimum=[-.3,.04,0.],scope_maximum=[.3,3.96,2.1])]
    structure=build_structured_stair_support(cloud,cloud,np.arange(len(cloud),dtype=np.int64),mapping,models,
        source_sha256='a'*64,planning_sha256='b'*64)
    cfg=dict(resolution=.1,slice_dh=.5,ground_height=-.5,simplify_layers=False,
        traversability=dict(kernel_size=7,interval_min=.55,interval_free=.7,slope_max_rad=.6,
            step_max=.17,standable_ratio=.2,cost_barrier=50.,safe_margin=.1,inflation=.1))
    payload,_=tomogram_from_points(cloud,cfg)
    return cloud,structure,cfg,payload


def run(values, **overrides):
    cloud,structure,cfg,payload=values
    args=dict(center=payload['center'],resolution=.1,traversability=cfg['traversability'],
        model=structure['model'],support_arrays=structure['arrays'],original_source_xyz=cloud,
        planning_xyz=cloud,input_tomogram_sha256='c'*64,planning_sha256='b'*64)
    args.update(overrides)
    return apply_structured_stair_cost(payload['data'],**args)


def test_native_model_cost_changes_terrain_not_ground_ceiling_or_unrelated_cells():
    values=fixture(); before=values[3]['data'].copy(); result=run(values)
    selected=result['arrays']['eligible_terrain']; after=result['data']
    assert selected.any() and result['diagnostics']['changed_native_raw_layer_cells']>0
    assert np.array_equal(before[3:].view(np.uint32),after[3:].view(np.uint32))
    assert np.array_equal(before[0][~selected],after[0][~selected])
    assert np.array_equal(values[3]['data'],before,equal_nan=True)
    assert not np.any(after[0]>before[0])
    assert np.all(np.isfinite(before[4][selected]))
    assert np.all(before[4][selected]-before[3][selected]>=.55)
    # Every changed actual cell carries a literal source identity, not a model point.
    witness=result['arrays']['literal_source_witness'][selected]
    assert np.isin(witness,result['arrays']['qualified_source_indices']).all()
    assert np.array_equal(result['arrays']['qualified_source_xyz'],values[0][result['arrays']['qualified_source_indices']])
    halo=selected.copy()
    halo[:,1:]|=selected[:,:-1]; halo[:,:-1]|=selected[:,1:]
    halo[:,:,1:]|=selected[:,:,:-1]; halo[:,:,:-1]|=selected[:,:,1:]
    assert np.array_equal(before[1:3,:,~np.any(halo,axis=0)],after[1:3,:,~np.any(halo,axis=0)])
    assert [item['channel'] for item in result['diagnostics']['channels']]==['cost','gradient_x','gradient_y','ground','ceiling']


def test_real_box_and_side_walls_keep_native_occupied_costs():
    values=fixture(box=True); result=run(values); data=values[3]['data']; cloud=values[0]
    shape=np.array(data.shape[2:]); center=values[3]['center']
    xx,yy=np.meshgrid(np.arange(shape[0]),np.arange(shape[1]),indexing='ij')
    xy=center+(np.stack((xx,yy),axis=-1)-shape//2)*.1
    box=(abs(xy[:,:,0])<.13)&(abs(xy[:,:,1]-2.)<.14)
    walls=abs(xy[:,:,0])>.69
    assert box.any()
    assert not result['arrays']['eligible_terrain'][:,box].any()
    assert not result['arrays']['modified_raw_seed'][:,walls].any()
    assert np.array_equal(result['data'][0,:,box],data[0,:,box])
    ids=result['arrays']['qualified_source_indices']
    assert not ((abs(cloud[ids,0])<.08)&(abs(cloud[ids,1]-2.)<.08)).any()


def test_missing_source_gap_stays_unknown_and_cannot_become_a_model_bridge():
    values=fixture(hole=True); result=run(values); data=values[3]['data']; shape=np.array(data.shape[2:])
    xx,yy=np.meshgrid(np.arange(shape[0]),np.arange(shape[1]),indexing='ij')
    xy=values[3]['center']+(np.stack((xx,yy),axis=-1)-shape//2)*.1
    gap=(xy[:,:,1]>1.44)&(xy[:,:,1]<1.66)&(abs(xy[:,:,0])<.5)
    assert gap.any() and not result['arrays']['eligible_terrain'][:,gap].any()
    assert np.array_equal(result['data'][0,:,gap],data[0,:,gap])


def test_original_low_headroom_cannot_be_repaired_by_support_model():
    values=fixture(); data=values[3]['data']; cfg=values[2]['traversability']
    known=np.isfinite(data[3]); data[4][known]=data[3][known]+.4
    raw=traversal_cost(np.nan_to_num(data[3],nan=-float(SENTINEL)),
        np.nan_to_num(data[4],nan=float(SENTINEL)),.1,cfg)
    data[0]=np.stack([_inflate(row,.1,cfg['safe_margin'],cfg['inflation']) for row in raw])
    data[1:3]=0; data[1,:,1:-1]=data[0,:,2:]-data[0,:,:-2]
    data[2,:,:,1:-1]=data[0,:,:,2:]-data[0,:,:,:-2]
    result=run(values)
    assert not result['arrays']['eligible_terrain'].any()
    assert np.array_equal(data.view(np.uint32),result['data'].view(np.uint32))


def test_foreign_or_synthetic_source_witness_is_rejected():
    values=fixture(); arrays=copy.deepcopy(values[1]['arrays']); arrays['source_xyz'][0,2]+=.001
    with pytest.raises(ValueError,match='literal_support'): run(values,support_arrays=arrays)
    model=copy.deepcopy(values[1]['model']); model['model_samples_are_observations']=True
    with pytest.raises(ValueError,match='identity'): run(values,model=model)


def test_already_modified_tensor_or_inconsistent_gradient_is_rejected():
    values=fixture(); data=values[3]['data']; data[0,0,0,0]+=1.
    with pytest.raises(ValueError,match='original_native_tensor'): run(values)
    values=fixture(); values[3]['data'][1,0,0,0]=1.
    with pytest.raises(ValueError,match='gradient'): run(values)


def test_model_cannot_expand_support_radius_or_native_slope_limits():
    values=fixture(); model=copy.deepcopy(values[1]['model']); model['policy']['source_support_radius_m']=.2
    with pytest.raises(ValueError,match='support_bounds'):run(values,model=model)
    cfg=copy.deepcopy(values[2]['traversability']); cfg['slope_max_rad']=.01
    # Native input equation binding catches unapproved policy changes first.
    with pytest.raises(ValueError,match='original_native_tensor'):run(values,traversability=cfg)


@pytest.mark.parametrize('field,value', [('retained_entity_height_m',.25),
    ('retained_entity_top_m',.1),('retained_entity_radius_m',.001)])
def test_structural_prior_cannot_silently_erase_raised_entities(field,value):
    values=fixture(box=True); model=copy.deepcopy(values[1]['model']); model['policy'][field]=value
    with pytest.raises(ValueError,match='support_bounds'):run(values,model=model)
