"""The final production selector must not let shortcuts erase clearance."""
import numpy as np
import pytest

from d1max_pct_planner.tomogram_map import TomogramMap
from d1max_pct_planner.tomogram_route import TomogramRoute


def scene(layers=1):
    xy=(np.arange(181)-90)*.1
    x,y=np.meshgrid(xy,xy,indexing='ij')
    data=np.zeros((5,layers,181,181),np.float32)
    data[0]=np.where((abs(y)<=.8)&(abs(x)<=8.),0.,50.)
    data[4]=2.
    return TomogramMap(dict(data=data,resolution=.1,center=[0.,0.],slice_h0=.1,slice_dh=.5))


def candidate(tomo,points,reason):
    points=np.asarray(points,float)
    layers=np.zeros(len(points),int)
    return dict(path=points,layers=layers,checks=tomo.validate_path(points,layers),
        curve_checks=dict(curve_validation='checked_native_fixture',quintic_segments=0,
            curve_partition_points=len(points)),algorithm=reason,
        refinement=dict(applied=reason=='visibility',reason=reason))


def selector(tomo):
    route=TomogramRoute.__new__(TomogramRoute)
    route.tomogram=tomo
    return route


def test_shorter_wall_hugging_visibility_cannot_erase_native_center_corridor():
    tomo=scene();route=selector(tomo)
    native=candidate(tomo,[[-6.,-.6,0.],[-3.,0.,0.],[3.,0.,0.],[6.,-.6,0.]],'native')
    straight=candidate(tomo,[[-6.,-.6,0.],[6.,-.6,0.]],'visibility')
    chosen=route._select_floor_quality(native,straight)
    tomo.validate_path(chosen['path'],chosen['layers'])
    audit=chosen['refinement']['clearance_selection']
    assert audit['selected_quality']['score']<=audit['native']['score']+1e-9
    assert audit['selected_quality']['score']<audit['visibility']['score']
    xy=np.asarray(chosen['path'])
    assert abs(xy[np.argmin(abs(xy[:,0])),1])<.3
    np.testing.assert_array_equal(xy[[0,-1]],native['path'][[0,-1]])


def test_centering_soft_rejection_preserves_a_legal_native_route(monkeypatch):
    import d1max_pct_planner.corridor_centering as centering
    failures = [dict(blend=1., reason='pct_cost_blocked'),
                dict(blend='local', reason='local_offsets_cannot_repair_curve')]
    monkeypatch.setattr(centering,'center_corridor',lambda *a,**k:dict(
        applied=False,reason='bounded_quality_unavailable', candidate_failures=failures))
    tomo=scene();route=selector(tomo)
    native=candidate(tomo,[[-4.,0.,0.],[0.,.1,0.],[4.,0.,0.]],'native')
    direct=candidate(tomo,[[-4.,0.,0.],[4.,0.,0.]],'visibility')
    chosen=route._select_floor_quality(native,direct)
    assert chosen['algorithm']=='visibility'
    assert not chosen['refinement']['clearance_selection']['centering_applied']
    assert chosen['refinement']['clearance_selection']['centering_candidate_failures'] == failures
    tomo.validate_path(chosen['path'],chosen['layers'])


def test_multislice_route_does_not_receive_planar_centering(monkeypatch):
    import d1max_pct_planner.corridor_centering as centering
    def forbidden(*args,**kwargs):raise AssertionError('Do not center a layer transition')
    monkeypatch.setattr(centering,'center_corridor',forbidden)
    tomo=scene(layers=2);route=selector(tomo)
    original=candidate(tomo,[[-2.,0.,0.],[0.,0.,0.],[2.,0.,0.]],'native')
    original['layers'][-1]=1
    assert route._select_floor_quality(original,original) is original


def test_public_route_plan_reaches_final_clearance_selector(monkeypatch):
    tomo=scene();route=selector(tomo)
    route.height_tolerance_m=.08;route.path_refinement='visibility_c2';route.refinement_corner_cut_m=1.5
    raw=np.array([[-5.,-.6,0.],[-2.,0.,0.],[2.,0.,0.],[5.,-.6,0.]])
    class Native:
        calls=0
        def plan(self,*args,**kwargs):
            self.calls+=1
            return dict(path=raw.copy(),layer_ids=np.zeros(len(raw),int),native_states=None,native_sample_dt=None)
    route.planner=Native()
    selected=[];actual=route._select_floor_quality
    def choose(*args):
        result=actual(*args);selected.append(result);return result
    monkeypatch.setattr(route,'_select_floor_quality',choose)
    result=route.plan(raw[0],raw[-1],0,0)
    assert len(selected)==route.planner.calls==1
    assert 'clearance_selection' in result['path_refinement']
    assert result['path'][0]==raw[0].tolist() and result['path'][-1]==raw[-1].tolist()
    tomo.validate_path(result['path'],result['layer_ids'])


def test_native_seed_can_improve_more_than_centering_only_visibility_shortcut():
    import d1max_pct_planner.corridor_centering as centering
    tomo = scene(); route = selector(tomo)
    native = candidate(tomo, [[-3.,-.6,0.],[-1.5,-.2,0.],[1.5,-.2,0.],[3.,-.6,0.]], 'native')
    shortcut = candidate(tomo, [[-3.,-.6,0.],[3.,-.6,0.]], 'visibility')
    old_center = centering.center_corridor(tomo, shortcut['path'], shortcut['layers'])
    old_scores = [centering.route_clearance_quality(tomo, item['path'], item['layers'])['score']
                  for item in (native, shortcut)]
    if old_center['applied']:
        old_scores.append(old_center['score_after'])
    chosen = route._select_floor_quality(native, shortcut)
    audit = chosen['refinement']['clearance_selection']
    assert audit['selected'] == 'centered' and audit['centering_source'] == 'native'
    assert audit['selected_quality']['score'] < min(old_scores)-.1
    assert [item['source'] for item in audit['centering_attempts']] == ['visibility', 'native']
    np.testing.assert_array_equal(chosen['path'][[0,-1]], native['path'][[0,-1]])
    tomo.validate_path(chosen['path'], chosen['layers'])


def test_duplicate_seed_is_refined_once_and_already_centred_route_not_detoured(monkeypatch):
    import d1max_pct_planner.corridor_centering as centering
    real_center = centering.center_corridor
    calls = []
    def record(*args, **kwargs):
        calls.append(1)
        return real_center(*args, **kwargs)
    monkeypatch.setattr(centering, 'center_corridor', record)
    tomo = scene(); route = selector(tomo)
    original = candidate(tomo, [[-4.,0.,0.],[4.,0.,0.]], 'native')
    shortcut = candidate(tomo, original['path'].copy(), 'visibility')
    chosen = route._select_floor_quality(original, shortcut)
    assert len(calls) == 1
    np.testing.assert_array_equal(chosen['path'], original['path'])
