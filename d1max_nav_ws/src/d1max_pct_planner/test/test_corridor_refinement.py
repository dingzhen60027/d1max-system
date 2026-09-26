import json

import numpy as np

from d1max_pct_planner.corridor_refinement import refine_corridor
from d1max_pct_planner.tomogram_map import TomogramMap


def scene(allowed=None, layers=1):
    coords=(np.arange(161)-80)*.1
    x,y=np.meshgrid(coords,coords,indexing='ij')
    mask=np.ones(x.shape,bool) if allowed is None else allowed(x,y)
    data=np.zeros((5,layers,161,161),np.float32)
    data[0]=np.where(mask,0,50)
    data[4]=2.
    return TomogramMap({'data':data,'resolution':.1,'center':[0.,0.],
                        'slice_h0':.15,'slice_dh':.5})


def wavy_line():
    x=np.linspace(-4,4,161)
    return np.c_[x,.15*np.sin(np.pi*(x+4)),np.zeros(len(x))]


def wavy_l():
    s=np.linspace(0,3,81)
    horizontal=np.c_[s-3,.09*np.sin(5*np.pi*s/3),np.zeros(len(s))]
    vertical=np.c_[.09*np.sin(5*np.pi*s/3),s,np.zeros(len(s))]
    return np.vstack((horizontal,vertical[1:]))


def test_wavy_open_corridor_becomes_exact_straight_xy_and_keeps_endpoints():
    tomo=scene();path=wavy_line();path[0,2]=.04;path[-1,2]=-.03
    before=path.copy();r=refine_corridor(tomo,path,np.zeros(len(path),int))
    assert r['applied'] and r['reason']=='checked_straight_line'
    output=np.array(r['path'])
    assert np.array_equal(output[[0,-1]],path[[0,-1]])
    assert np.array_equal(path,before)
    assert np.max(abs(output[:,1]))<1e-12
    assert np.max(np.linalg.norm(np.diff(output[:,:2],axis=0),axis=1))<=.100000001
    assert r['after_quality']['curvature_max_per_m']<1e-9
    assert r['after_quality']['total_abs_turn_rad']<r['before_quality']['total_abs_turn_rad']
    assert not r['is_native_gpmp']
    assert r['segments'][0]['kind']=='line'
    tomo.validate_path(output,r['layer_ids'])
    json.dumps(r,allow_nan=False)


def test_l_corridor_gets_checked_zero_endpoint_curvature_bezier_corners():
    tomo=scene(lambda x,y: ((x>=-3.6)&(x<=.6)&(abs(y)<=.6))|((abs(x)<=.6)&(y>=-.6)&(y<=3.6)))
    path=wavy_l();r=refine_corridor(tomo,path,np.zeros(len(path),int),corner_cut_m=1.5)
    assert r['applied'],r
    assert r['reason']=='checked_c2_corners'
    assert any(segment['kind']=='quintic_bezier' for segment in r['segments'])
    output=np.asarray(r['path']);tomo.validate_path(output,r['layer_ids'])
    assert np.max(np.linalg.norm(np.diff(output[:,:2],axis=0),axis=1))<=.100000001
    for segment in r['segments']:
        if segment['kind']!='quintic_bezier':continue
        coefficients=np.asarray(segment['coefficients_xy'])
        acceleration=np.polynomial.polynomial.polyder(coefficients,m=2,axis=0)
        np.testing.assert_allclose(np.polynomial.polynomial.polyval([0.,1.],acceleration),0,atol=1e-10)
    for key in ('xy_length_m','xyz_length_m','total_abs_turn_rad','curvature_p95_per_m','curvature_max_per_m'):
        assert r['after_quality'][key] <= r['before_quality'][key]+1e-5


def test_blocked_direct_connection_cannot_be_published_through_a_wall():
    tomo=scene(lambda x,y: ~((abs(x)<=.5)&(abs(y)<=1.)))
    path=np.array([[-3.,0.,0.],[-1.,-2.,0.],[1.,-2.,0.],[3.,0.,0.]])
    r=refine_corridor(tomo,path,[0]*len(path))
    assert r.get('reason')!='checked_straight_line'
    if r['applied']:
        tomo.validate_path(r['path'],r['layer_ids'])
        assert any(point[1]<-1 for point in r['path'])
    else:
        assert r['reason']=='no_valid_nonworsening_smooth_candidate'
        assert 'path' not in r


def test_invalid_native_input_never_gets_repaired_by_bypassing_validation():
    tomo=scene(lambda x,y: abs(x)>.3)
    r=refine_corridor(tomo,[[-2.,0.,0.],[2.,0.,0.]],[0,0])
    assert not r['applied']
    assert r['reason']=='invalid_input_path'
    assert 'path' not in r


def test_unknown_ground_is_not_treated_as_clear_space():
    tomo=scene();tomo.ground[:,80,:]=np.nan;tomo.ground_known[:,80,:]=False;tomo.valid[:,80,:]=False
    r=refine_corridor(tomo,[[-2.,0.,0.],[2.,0.,0.]],[0,0])
    assert not r['applied'] and r['reason']=='invalid_input_path'


def test_layer_transitions_are_not_refined_even_with_geometric_overlap():
    tomo=scene(layers=2)
    path=[[-1.,0.,0.],[0.,0.,0.],[1.,0.,0.]]
    tomo.validate_path(path,[0,0,1])
    r=refine_corridor(tomo,path,[0,0,1])
    assert not r['applied'] and r['reason']=='multi_layer_path_not_supported'


def test_all_corner_scales_fail_without_falling_back_to_a_sharp_polyline():
    tomo=scene(lambda x,y: ((abs(y)<.001)&(x<=0)&(x>=-3.1))|((abs(x)<.001)&(y>=0)&(y<=3.1)))
    path=np.array([[-3.,0.,0.],[0.,0.,0.],[0.,3.,0.]])
    tomo.validate_path(path,[0,0,0])
    r=refine_corridor(tomo,path,[0,0,0],corner_cut_m=1.5)
    assert not r['applied'] and r['reason']=='no_valid_nonworsening_smooth_candidate'
    assert len(r['candidate_failures'])==3
    assert 'path' not in r


def test_duplicates_do_not_lose_exact_endpoints():
    tomo=scene();path=wavy_line();path=np.vstack((path[0],path,path[-1]))
    r=refine_corridor(tomo,path,[0]*len(path))
    assert r['applied']
    assert r['path'][0]==path[0].tolist() and r['path'][-1]==path[-1].tolist()


def test_short_straight_line_remains_valid_and_exact():
    tomo=scene();path=[[.01,.02,0.],[.06,.02,0.]]
    r=refine_corridor(tomo,path,[0,0])
    assert r['applied'] and r['path'][0]==path[0] and r['path'][-1]==path[-1]


def test_pure_z_route_is_not_a_corridor():
    r=refine_corridor(scene(),[[0.,0.,0.],[0.,0.,.02]],[0,0])
    assert not r['applied'] and r['reason']=='degenerate_or_closed_xy_route'


def test_invalid_layer_and_nonfinite_points_have_explicit_rejection():
    assert not refine_corridor(scene(),[[0.,0.,0.],[1.,0.,0.]],[0,99])['applied']
    assert not refine_corridor(scene(),[[0.,0.,0.],[np.nan,0.,0.]],[0,0])['applied']


def test_bad_cut_has_explicit_rejection():
    for cut in [0,-1,np.nan,True]:
        r=refine_corridor(scene(),[[0.,0.,0.],[1.,0.,0.]],[0,0],cut)
        assert not r['applied'] and r['reason']=='invalid_corner_cut'


def test_soft_cost_tradeoff_is_audited_without_changing_hard_limits():
    tomo=scene();tomo.cost[0,:,80]=15.
    x=np.linspace(-4,4,121)
    path=np.c_[x,.4*np.sin(np.pi*(x+4)/8),np.zeros(len(x))]
    r=refine_corridor(tomo,path,[0]*len(path))
    assert r['applied'] and r['reason']=='checked_straight_line'
    assert r['after_cost_audit']['mean_touched_cell_cost'] > r['before_cost_audit']['mean_touched_cell_cost']
    assert r['after_cost_audit']['max_cost']==15
    assert r['after_cost_audit']['hard_cost_threshold']==20
    assert r['soft_cost_policy']=='prefer_geometric_simplicity_within_unchanged_hard_pct_mask'
    assert not r['physical_safety_certified']


def test_length_budget_precedes_expensive_boundary_validation():
    class NoValidationAllowed:
        def validate_path(self,*args,**kwargs):
            raise AssertionError('Oversized path must be rejected before grid walking')
    path=np.array([[0.,0.,0.],[2000.,0.,0.]])
    r=refine_corridor(NoValidationAllowed(),path,[0,0])
    assert not r['applied'] and r['reason']=='input_length_limit'


def test_direct_visibility_reuses_geometry_but_revalidates_changed_map(monkeypatch):
    tomo=scene();path=wavy_line()
    original=tomo.validate_path
    calls=[]

    def mutate_after_visibility(*args,**kwargs):
        result=original(*args,**kwargs)
        calls.append(1)
        if len(calls)==2:  # Input route, then fully expanded visibility line.
            tomo.cost[0,80,80]=50.
        return result

    monkeypatch.setattr(tomo,'validate_path',mutate_after_visibility)
    result=refine_corridor(tomo,path,np.zeros(len(path),int))
    assert not result['applied']
    assert result['candidate_failures'][0]['reason']=='invalid_curve'
    assert 'path' not in result


def test_direct_line_performs_only_one_geometry_expansion(monkeypatch):
    tomo=scene();path=wavy_line();calls=[]
    original=tomo.surface_ground_z_many

    def counted(*args,**kwargs):
        calls.append(1)
        return original(*args,**kwargs)

    monkeypatch.setattr(tomo,'surface_ground_z_many',counted)
    result=refine_corridor(tomo,path,np.zeros(len(path),int))
    assert result['applied'] and calls==[1]
    tomo.validate_path(result['path'],result['layer_ids'])
