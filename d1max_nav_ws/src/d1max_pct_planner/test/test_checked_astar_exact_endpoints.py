"""Production orchestration and refinement, only native search is injected."""
from types import SimpleNamespace

import numpy as np
import pytest

from d1max_pct_planner.tomogram_map import TomogramError
from d1max_pct_planner.tomogram_route import TomogramRoute
from test_source_identity_refinement import scene


def route_fixture(*, goal_delta=0.):
    tomo=scene(1)
    start=np.array([-2.,0.,0.]);goal=np.array([2.,0.,0.])
    native_goal=goal.copy();native_goal[0]+=goal_delta
    route=TomogramRoute.__new__(TomogramRoute)
    route.tomogram,route.height_tolerance_m=tomo,.08
    route.planning_strategy='native_astar_checked_smooth'
    route.refinement_corner_cut_m=1.5;route.source_ground=None
    route.native_runtime={};route.native_parameters={}
    route.planner=SimpleNamespace(plan_astar=lambda *args:dict(
        path=np.array([start,[0.,0.,0.],native_goal]),layer_ids=np.array([0,0,0])))
    return route,start,goal


@pytest.mark.parametrize('delta',[0.,np.spacing(2.),-np.spacing(2.),1e-10])
def test_lattice_near_duplicate_never_replaces_frozen_requested_goal(delta):
    route,start,goal=route_fixture(goal_delta=delta)
    result=route.plan(start,goal,0,0)
    assert np.array_equal(result['path'][0],start)
    assert np.array_equal(result['path'][-1],goal)
    route.tomogram.validate_path(result['path'],result['layer_ids'])


def test_real_refiner_endpoint_mutation_remains_rejected_even_for_one_ulp(monkeypatch):
    import d1max_pct_planner.source_identity_refinement as module
    route,start,goal=route_fixture(goal_delta=np.spacing(2.))
    refine=module.refine_source_identity
    def bad_refine(*args,**kwargs):
        result=refine(*args,**kwargs)
        result['path'][-1][0]=np.nextafter(result['path'][-1][0],np.inf)
        return result
    monkeypatch.setattr(module,'refine_source_identity',bad_refine)
    with pytest.raises(TomogramError,match='frozen endpoints'):
        route.plan(start,goal,0,0)


def test_preserving_exact_goal_does_not_bypass_blocked_connector():
    route,start,goal=route_fixture(goal_delta=np.spacing(2.))
    route.tomogram.cost[0,40,:]=50.
    with pytest.raises(TomogramError):
        route.plan(start,goal,0,0)
