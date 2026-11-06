"""Optimizer rejection must not erase the independently checked native search."""
from copy import deepcopy
import numpy as np
import pytest
from d1max_pct_planner.tomogram_map import TomogramMap, TomogramError
from d1max_pct_planner.tomogram_route import TomogramRoute


def scene(refinement='none'):
    data=np.zeros((5,2,21,21),dtype=np.float32)
    data[3,1]=3.;data[4,0]=2.;data[4,1]=5.
    tomo=TomogramMap(dict(data=data,resolution=.1,center=[0.,0.],slice_h0=0.,slice_dh=.5))
    tomo.cost[0,10,10]=50.;tomo.valid[0,10,10]=False
    start,goal=[-.6,0.,0.],[.6,0.,0.]
    raw=dict(path=np.array([start,goal]),layer_ids=np.array([0,0]),native_states=None,
        astar_path=np.array([start,[-.6,.4,0.],[.6,.4,0.],goal]),
        astar_layer_ids=np.zeros(4,dtype=int))
    class Native:
        calls=0
        def plan(self,*args,**kw):
            self.calls+=1
            return deepcopy(raw)
    route=TomogramRoute.__new__(TomogramRoute)
    route.tomogram=tomo;route.planner=Native();route.height_tolerance_m=.08
    route.path_refinement=refinement;route.refinement_corner_cut_m=.5
    return route,raw,start,goal


@pytest.mark.parametrize('refinement',['none','visibility_c2'])
def test_rejected_gpmp_uses_same_search_checked_from_exact_endpoints(refinement):
    route,raw,start,goal=scene(refinement)
    result=route.plan(start,goal,0,0)
    assert route.planner.calls==1  # No second search or replacement planner.
    assert result['path'][0]==start and result['path'][-1]==goal
    assert result['optimizer_rejection']['code']=='invalid_curve'
    assert 'GPMP rejected' in result['algorithm']
    route.tomogram.validate_path(result['path'],result['layer_ids'])
    assert result['cost_threshold']==20
    np.testing.assert_array_equal(raw['path'],[start,goal]) # Original rejected curve is not mutated.


@pytest.mark.parametrize('fault',['blocked','no_support','headroom','wrong_floor','bad_layers','no_search'])
def test_original_search_cannot_bypass_geometry_or_floor_rules(fault):
    route,raw,start,goal=scene()
    if fault=='blocked':raw['astar_path']=raw['path'].copy();raw['astar_layer_ids']=np.zeros(2,dtype=int)
    elif fault=='no_support':
        route.tomogram.ground[0,10,14]=np.nan;route.tomogram.ground_known[0,10,14]=False
    elif fault=='headroom':route.tomogram.headroom[0,10,14]=.1
    elif fault=='wrong_floor':
        raw['astar_layer_ids'][1:3]=1;raw['astar_path'][1:3,2]=3.
    elif fault=='bad_layers':raw['astar_layer_ids']=np.zeros(3)
    else:raw.pop('astar_path')
    with pytest.raises(TomogramError):route.plan(start,goal,0,0)


def test_original_search_exact_connector_is_not_snapped_to_make_it_pass():
    route,raw,start,goal=scene()
    route.tomogram.cost[0,4,12]=50.;route.tomogram.valid[0,4,12]=False
    raw['astar_path']=raw['astar_path'][1:];raw['astar_layer_ids']=np.zeros(3,dtype=int)
    with pytest.raises(TomogramError):route.plan(start,goal,0,0)
