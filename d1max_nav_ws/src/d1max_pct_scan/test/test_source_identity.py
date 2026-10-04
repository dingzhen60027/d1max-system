from copy import deepcopy
import numpy as np
import pytest

from d1max_pct_planner.tomogram_map import TomogramMap
from d1max_pct_scan.source_identity import SourceIdentityBridge
from d1max_pct_scan.source_route import SourceRouteBuilder,SourceSupport,RouteSnapshot


def make(ceiling=True,support=True):
    grid=np.linspace(-1.,1.,41)
    points=np.array([[x,y,0.] for x in grid for y in grid])
    bridge=SourceIdentityBridge(points,[[5.,5.],[6.,6.]],floor_id='floor1')
    data=np.zeros((5,2,30,30),np.float32)
    data[4]=2. if ceiling else np.nan
    tomo=TomogramMap(dict(data=data,resolution=.1,center=[0.,0.],slice_h0=0.,slice_dh=.5))
    builder=SourceRouteBuilder(bridge=bridge,source_map_sha256='a'*64,conditioning_sha256='b'*64,
        tomogram_sha256='c'*64,tomogram=tomo,
        support=SourceSupport({'floor1':points},'a'*64) if support else None)
    route=[[0.,0.,0.],[.2,0.,0.],[.4,0.,0.]]
    snapshot=builder.build(dict(route_type='same_floor',layer_ids=[0,0,1],source_layer_ids=[0,0,1]),
        route,['floor1']*3,map_version_id='map')
    return bridge,snapshot


def test_identity_keeps_xyz_layers_and_geometry_eligible_is_not_execution_authority():
    bridge,snapshot=make()
    v=snapshot.payload()
    assert v['xyz']==[[0.,0.,0.],[.2,0.,0.],[.4,0.,0.]]
    assert v['layer_ids']==[0,0,1] and v['execution_eligible']
    assert not v['geometry_evidence']['physical_execution_authorized']
    assert v['geometry_evidence']['observed_source_support_xyz']
    assert RouteSnapshot.create(v)==snapshot
    assert bridge.source_frame==bridge.planning_frame=='d1max_loc_map'


@pytest.mark.parametrize('kwargs',[dict(ceiling=False),dict(support=False)])
def test_missing_source_or_unknown_ceiling_stays_preview_only(kwargs):
    _,snapshot=make(**kwargs)
    assert not snapshot.payload()['execution_eligible']


def test_stair_or_second_floor_not_silently_selected_from_same_xy():
    bridge,_=make()
    with pytest.raises(ValueError): bridge.to_localization_ground([[0,0,3],[1,0,3]],['floor2']*2)
    with pytest.raises(ValueError): bridge.project_live_pose_to_ground([0.,0.,3.55],'floor1',body_height_interval_m=(.4,.7))
    with pytest.raises(ValueError): bridge.query([[5.5,5.5]])


def test_nonrigid_cannot_claim_identity_geometry_authority():
    _,snapshot=make(); v=snapshot.payload()
    v['geometry_evidence']['projection_kind']='non_rigid_ground_only_not_tf'
    with pytest.raises(ValueError): RouteSnapshot.create(v)
