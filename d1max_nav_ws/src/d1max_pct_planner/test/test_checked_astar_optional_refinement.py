import numpy as np
import pytest

from d1max_pct_planner.tomogram_map import TomogramMap,TomogramError
from d1max_pct_planner.tomogram_route import TomogramRoute
from d1max_pct_planner import source_identity_refinement


def planner():
    data=np.zeros((5,1,41,41),dtype=np.float32);data[4]=2.
    t=TomogramMap(dict(data=data,resolution=.1,center=[0.,0.],slice_h0=0.,slice_dh=.5,
        geometry_operation='source_identity'),unknown_ceiling_policy='reject')
    class Native:
        def plan_astar(self,*unused):
            return dict(path=np.array([[-1.,0.,0.],[0.,0.,0.],[1.,0.,0.]]),layer_ids=np.zeros(3,dtype=int))
    p=TomogramRoute.__new__(TomogramRoute)
    p.tomogram=t;p.planner=Native();p.refinement_corner_cut_m=1.5;p.source_ground=None
    p.native_runtime={};p.native_parameters={};p.planning_strategy='native_astar_checked_smooth'
    return p


def endpoints():
    return dict(xyz=[-1.,0.,0.],layer_id=0),dict(xyz=[1.,0.,0.],layer_id=0)


def test_quality_only_refinement_failure_retains_checked_native_route(monkeypatch):
    p=planner()
    def reject(*args):
        raise TomogramError('no_valid_smooth_candidate','relative curvature metric',
            candidate_failures=[dict(code='quality_not_improved',details=dict(metrics=['curvature_max_per_m']))])
    monkeypatch.setattr(source_identity_refinement,'refine_source_identity',reject)
    a,b=endpoints();result=p._plan_checked_astar(a,b,0.)
    assert result['path'][0]==a['xyz'] and result['path'][-1]==b['xyz']
    assert result['path_refinement']['applied'] is False
    assert result['path_refinement']['reason']=='quality_only_smoothing_fallback'
    assert result['curve_validation']=='native_polyline_cell_supercover_and_actual_slice_overlap'
    assert result['path_refinement']['continuity'].startswith('Geometric polyline')
    p.tomogram.validate_path(result['path'],result['layer_ids'])


@pytest.mark.parametrize('code,failures',[
    ('source_identity_ground_unobserved',[]),
    ('no_valid_smooth_candidate',[dict(code='pct_cost_blocked')]),
    ('no_valid_smooth_candidate',[dict(code='quality_not_improved'),dict(code='insufficient_headroom')]),
    ('refinement_output_limit',[]),
])
def test_geometry_support_and_resource_failures_never_fallback(monkeypatch,code,failures):
    p=planner()
    def reject(*args):raise TomogramError(code,'not a soft quality failure',candidate_failures=failures)
    monkeypatch.setattr(source_identity_refinement,'refine_source_identity',reject)
    a,b=endpoints()
    with pytest.raises(TomogramError) as error:p._plan_checked_astar(a,b,0.)
    assert error.value.code==code


def test_fallback_is_revalidated_not_cached_acceptance(monkeypatch):
    p=planner()
    def changed_map(*args):
        p.tomogram.cost[0,20,:]=50.
        raise TomogramError('no_valid_smooth_candidate','quality',candidate_failures=[dict(code='quality_not_improved')])
    monkeypatch.setattr(source_identity_refinement,'refine_source_identity',changed_map)
    a,b=endpoints()
    with pytest.raises(TomogramError) as error:p._plan_checked_astar(a,b,0.)
    assert error.value.code=='invalid_curve'
    assert any(reason['code']=='pct_cost_blocked' for reason in error.value.details['reasons'])
