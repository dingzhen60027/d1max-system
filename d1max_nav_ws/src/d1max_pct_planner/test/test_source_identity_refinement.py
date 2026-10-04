import numpy as np
import pytest

from d1max_pct_planner.source_identity_refinement import refine_source_identity, ObservedGround
from d1max_pct_planner.tomogram_map import TomogramMap,TomogramError


def scene(layers=2):
    data=np.zeros((5,layers,81,41),dtype=np.float32)
    data[4]=2.
    return TomogramMap(dict(data=data,resolution=.1,center=[0.,0.],slice_h0=0.,slice_dh=.5,
                            geometry_operation='source_identity'))


def test_real_slice_overlap_preserves_exact_endpoints_and_validates_entire_curve():
    t=scene();t.cost[0,45:]=50.;t.cost[1,:35]=50.
    p=np.array([[-2.,0.,0.],[0.,0.,0.],[2.,0.,0.]])
    r=refine_source_identity(t,p,[0,0,1])
    assert r['applied'] and not r['is_native_gpmp']
    assert r['path'][0]==p[0].tolist() and r['path'][-1]==p[-1].tolist()
    assert set(r['layer_ids'])=={0,1}
    t.validate_path(r['path'],r['layer_ids'])


def test_same_xy_upper_floor_cannot_replace_missing_lower_floor():
    t=scene();t.ground[1]=3.
    t.cost[0,40,:]=50.
    with pytest.raises(TomogramError):
        refine_source_identity(t,[[-2.,0.,0.],[2.,0.,0.]],[0,0])


def test_unknown_and_low_ceiling_remain_blocked():
    for which in ('ground','ceiling'):
        t=scene(1)
        if which=='ground':
            t.ground[0,40,:]=np.nan;t.ground_known[0,40,:]=False
        else:
            t.ceiling[0,40,:]=.2;t.headroom[0,40,:]=.2
        with pytest.raises(TomogramError):
            refine_source_identity(t,[[-2.,0.,0.],[2.,0.,0.]],[0,0])


def test_no_ground_filling_when_original_returns_missing():
    t=scene(1)
    source=ObservedGround([[-2.,0.,0.],[2.,0.,0.]])
    with pytest.raises(TomogramError):
        refine_source_identity(t,[[-2.,0.,0.],[2.,0.,0.]],[0,0],source_ground=source)


def test_original_ground_heights_are_not_flattened_or_pcd_max_outliers():
    t=scene(1)
    t.ground[0]=.08
    x=np.linspace(-2.,2.,401)
    ground=np.c_[x,np.zeros_like(x),.02*np.sin(x)]
    p=ground[[0,200,-1]]
    r=refine_source_identity(t,p,[0,0,0],source_ground=ObservedGround(ground))
    z=np.asarray(r['path'])[:,2]
    assert z.max()-z.min()>.03 and np.max(z)<.03
    t.validate_path(r['path'],r['layer_ids'])


def test_nonrigid_conditioned_map_cannot_use_identity_strategy():
    t=scene();t.provenance['geometry_operation']='flatten'
    with pytest.raises(TomogramError,match='source-identity'):
        refine_source_identity(t,[[-2.,0.,0.],[2.,0.,0.]],[0,0])
