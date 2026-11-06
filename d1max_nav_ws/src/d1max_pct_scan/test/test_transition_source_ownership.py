"""Original terrain ownership is independent of the moving support window."""
from copy import deepcopy
from types import SimpleNamespace

import numpy as np
import pytest

from d1max_pct_scan.reference_support import (ConditionedReferenceSupport,
    _polyline_distance_squared, _unique_transition_owners, owned_stair_source_returns)
from d1max_pct_scan.source_route import SourceSupport
from d1max_pct_scan.continuous_reference_node import ReferenceCallbacks
from d1max_pct_scan.control_frame_contract import Rigid
from test_reference_support import transition_request, xyz


def provider(points, **floors):
    from scipy.spatial import cKDTree
    p=ConditionedReferenceSupport({'source_sha256':'a'*64},
        SourceSupport({'stairs':points,**floors},'a'*64))
    p.stair_index=SimpleNamespace(tree=cKDTree(points),points=np.asarray(points))
    return p


def owners(result, contexts):
    answer={}
    for c,part in zip(contexts,result):
        for point in part:
            assert tuple(point) not in answer
            answer[tuple(point)]=c[0]
    return answer


def test_exact_distance_uses_segment_interiors_not_vertex_nearness():
    rng=np.random.default_rng(11)
    path=np.r_[rng.normal(size=(21,3)),[[0.,0.,0.],[20.,0.,0.],[20.,0.,0.]]]
    points=np.r_[rng.normal(size=(31,3)),[[10.,.01,0.]]]
    distances=[]
    for point in points:
        row=[]
        for a,b in zip(path[:-1],path[1:]):
            d=b-a;r=np.dot(point-a,d)/np.dot(d,d) if np.dot(d,d) else 0.
            row.append(np.linalg.norm(point-a-np.clip(r,0.,1.)*d)**2)
        distances.append(min(row))
    np.testing.assert_allclose(_polyline_distance_squared(points,path),distances,atol=2e-14)


@pytest.mark.parametrize('reverse',[False,True])
def test_overlap_has_unique_owners_and_preserves_literal_union(reverse):
    lower=np.array([[x,0.,.1*x] for x in np.linspace(0.,3.,20)])
    upper=lower+[0.,.7,1.]
    points=np.r_[lower,upper];p=provider(points)
    contexts=[('lower','stair_lower','stair_up','stair'),('upper','stair_upper','stair_up','stair')]
    geometries=[lower,upper]
    if reverse:contexts=contexts[::-1];geometries=[g[::-1] for g in geometries[::-1]]
    assigned=_unique_transition_owners(p,[points,points],contexts,geometries)
    actual=owners(assigned,contexts)
    assert set(actual)==set(map(tuple,points))
    assert all(actual[tuple(point)]=='lower' for point in lower)
    assert all(actual[tuple(point)]=='upper' for point in upper)


def test_full_geometry_keeps_owner_stable_as_context_window_moves():
    lower=np.array([[0.,0.,0.],[10.,0.,0.]])
    upper=np.array([[0.,1.,0.],[10.,1.,0.]])
    points=np.array([[5.,.6,0.],[4.,.1,0.],[8.,.9,0.]])
    p=provider(points);contexts=[('a','stair_lower','stair_up','stair'),('b','stair_upper','stair_up','stair')]
    first=_unique_transition_owners(p,[points,points],contexts,[lower,upper])
    later=_unique_transition_owners(p,[points[:2],points[:2]],contexts,[lower,upper])
    assert owners(first,contexts)[tuple(points[0])]=='b'
    assert owners(later,contexts)[tuple(points[0])]=='b'
    assert owners(later,contexts)[tuple(points[1])]=='a'


@pytest.mark.parametrize('other,mode,kind', [('floor1','general','floor'),('stairs','stair','landing')])
def test_true_floor_or_landing_wins_exact_portal_tie(other,mode,kind):
    points=np.array([[0.,0.,0.],[0.,.1,0.],[-.2,0.,0.],[.2,0.,.1]])
    p=provider(points,floor1=points[:3])
    contexts=[('flight','stair_lower','stair_up','stair'),('portal',other,kind,mode)]
    paths=[np.array([[0.,0.,0.],[1.,0.,.5]]),np.array([[-1.,0.,0.],[0.,0.,0.]])]
    result=owners(_unique_transition_owners(p,[points,points[:3]],contexts,paths),contexts)
    assert result[(0.,0.,0.)]=='portal'
    assert result[(.2,0.,.1)]=='flight'


def test_same_xy_other_floor_cannot_borrow_ownership_or_contact_class():
    low=np.array([[0.,0.,0.],[.1,0.,0.]])
    high=low+[0.,0.,3.];p=provider(np.r_[low,high],floor1=low,floor2=high)
    contexts=[('a','floor1','floor','general'),('b','floor2','floor','general')]
    result=owners(_unique_transition_owners(p,[low,high],contexts,[high,low]),contexts)
    # Even an erroneous nearer reference cannot turn a source row into another floor.
    assert all(result[tuple(point)]=='a' for point in low)
    assert all(result[tuple(point)]=='b' for point in high)


@pytest.mark.parametrize('reverse',[False,True])
def test_callback_partition_before_indices_and_rigid_transform(reverse):
    node,req,literal=transition_request(reverse)
    contexts=[]
    raw=np.asarray([req[2].inverse().point((p.pose.position.x,p.pose.position.y,p.pose.position.z))
                    for p in req[0].reference.path.poses])-[0.,0.,.55]
    for ctx in req[5][4]:
        ids=np.flatnonzero(np.asarray(req[0].reference.point_segment_ids)==ctx[0])
        # Include the shared portal but freeze a whole segment, not selection radius.
        geometry=raw[max(0,ids[0]-1):min(len(raw),ids[-1]+2)].copy()
        geometry.flags.writeable=False
        contexts.append((*ctx,geometry))
    req=(*req[:5],(*req[5][:4],tuple(contexts)))
    node.support_map.stair_contact_source=frozenset(map(tuple,literal[literal[:,2]>.05]))
    _,support=node.build_support(req)
    assert support.verified
    source=np.asarray([req[2].inverse().point(tuple(map(float,p))) for p in xyz(support)])
    assert len(np.unique(np.round(source,12),axis=0))==len(source)
    assert list(support.stair_contact_indices)==sorted(set(support.stair_contact_indices))
    for i in support.stair_contact_indices:
        region=next(r for r in support.regions if r.begin_index<=i<r.end_index)
        assert region.required_mode=='stair'


def test_live_prepare_freezes_full_readonly_segment_views_without_copying_route():
    node,req,_=transition_request();reference=req[0].reference
    points=np.asarray([[float(i),0.,0.] for i in range(13)])
    points.flags.writeable=False
    segments=[dict(segment_id=c[0],floor_id=c[1],kind=c[2],required_mode=c[3],
                   begin_index=c[4],end_index=c[5]) for c in req[5][4]]
    core=SimpleNamespace(_points=points,_segments=segments,_segment=0,goal_ground_xyz=tuple(map(float,points[-1])),
        goal_heading=(False,0.,.15),anchor=SimpleNamespace(map_from_odom=Rigid((0.,0.,0.),(0.,0.,0.,1.))),
        body_height=.55,_progress=SimpleNamespace(body_source_xyz=(0.,0.,.55)),support_evidence=req[1])
    captured=[]
    n=SimpleNamespace(map_fresh=lambda:True,geometry_owner_fact=lambda:None,
        p={'transport_mode':'isolated_mock'},freshness_ns=600_000_000,
        transport=SimpleNamespace(pending_core=core,core=core,_pending_received_monotonic=1.,
            _pending_end_arc_m=2.,_pending_start_arc_m=0.,_pending_issue_arc_m=0.),
        prepared={},prepared_limit=8,mono=lambda:1.,
        deliver_proposal=lambda proposal:True,
        preparation=SimpleNamespace(submit=lambda key,value:captured.append(value)))
    n.prepared_for=lambda version:n.prepared.get(version.reference_generation)
    n.prepare_support=lambda record:ReferenceCallbacks.prepare_support(n,record)
    ReferenceCallbacks.prepare(n,reference)
    assert len(captured)==1
    for context,segment in zip(captured[0][5][4],segments):
        assert not context[6].flags.writeable and np.shares_memory(context[6],points)
        np.testing.assert_array_equal(context[6],points[segment['begin_index']:segment['end_index']+1])
        assert len(context[7])==len(segments)
    assert captured[0][5][4][0][7] is captured[0][5][4][1][7]


def test_newly_represented_third_segment_never_steals_previously_relabelled_source():
    paths=[np.array([[0.,float(i),0.],[10.,float(i),0.]]) for i in range(3)]
    points=np.array([[5.,.1,0.],[5.,1.1,0.],[5.,1.9,0.]])
    p=provider(points)
    domain=tuple((str(i),'stairs','stair_up','stair',0,1,path) for i,path in enumerate(paths))
    current=tuple((*context,domain) for context in domain[:2])
    later=tuple((*context,domain) for context in domain)
    before=owners(_unique_transition_owners(p,[points]*2,current,paths[:2]),current)
    after=owners(_unique_transition_owners(p,[points]*3,later,paths),later)
    assert tuple(points[2]) not in before  # Never mislabel future terrain as segment 1.
    assert after[tuple(points[2])]=='2'
    assert all(after[point]==owner for point,owner in before.items())


def cached_domain(paths, identity=('task','route')):
    for path in paths:path.flags.writeable=False
    domain=tuple((str(i),'stairs','stair_up','stair',0,len(path)-1,path) for i,path in enumerate(paths))
    return domain,tuple((*context,domain,identity) for context in domain)


def test_single_mixed_and_return_to_single_keep_identical_source_owner():
    paths=[np.array([[0.,float(i),0.],[10.,float(i),0.]]) for i in range(2)]
    points=np.array([[5.,.1,0.],[5.,.9,0.],[6.,.2,0.],[6.,.8,0.]])
    p=provider(points);domain,contexts=cached_domain(paths)
    single=owned_stair_source_returns(p,points,domain[0],domain,('task','route'))
    mixed=_unique_transition_owners(p,[points,points],contexts,paths)
    np.testing.assert_array_equal(single,mixed[0])
    np.testing.assert_array_equal(single,owned_stair_source_returns(p,points,domain[0],domain,('task','route')))
    assert set(map(tuple,mixed[1]))=={tuple(points[1]),tuple(points[3])}


def test_cache_is_content_bound_and_reused_by_equal_geometry_not_addresses():
    paths=[np.array([[0.,float(i),0.],[10.,float(i),0.]]) for i in range(2)]
    points=np.array([[5.,.1,0.],[5.,.9,0.]])
    p=provider(points);domain,contexts=cached_domain(paths)
    expected=_unique_transition_owners(p,[points]*2,contexts,paths)
    first=p._transition_owner_cache
    copies=[path.copy() for path in paths];domain,contexts=cached_domain(copies)
    _unique_transition_owners(p,[points]*2,contexts,copies)
    assert p._transition_owner_cache is first
    # Read-only view alone is insufficient: base mutation must invalidate too.
    copies[0].flags.writeable=True;copies[0][:,1]=2.;copies[0].flags.writeable=False
    moved=_unique_transition_owners(p,[points]*2,contexts,copies)
    assert p._transition_owner_cache is not first and len(moved[0])==0
    changed=p._transition_owner_cache
    domain,contexts=cached_domain(copies,('next-task','route'))
    _unique_transition_owners(p,[points]*2,contexts,copies)
    assert p._transition_owner_cache is not changed
    changed=p._transition_owner_cache
    p.source_support.source_hash='b'*64
    _unique_transition_owners(p,[points]*2,contexts,copies)
    assert p._transition_owner_cache is not changed


def test_cache_never_exceeds_bound_even_for_one_large_union_and_eviction_is_exact():
    points=np.c_[np.linspace(0.,10.,20017),np.zeros(20017),np.zeros(20017)]
    p=provider(points);paths=[np.array([[0.,0.,0.],[10.,0.,0.]]),np.array([[0.,1.,0.],[10.,1.,0.]])]
    domain,contexts=cached_domain(paths)
    first=_unique_transition_owners(p,[points]*2,contexts,paths)
    assert len(p._transition_owner_cache[1])==20000
    again=_unique_transition_owners(p,[points[:17]]*2,contexts,paths)
    np.testing.assert_array_equal(again[0],first[0][:17])
    assert len(p._transition_owner_cache[1])<=20000
    del p._transition_owner_cache
    cold=_unique_transition_owners(p,[points[:17]]*2,contexts,paths)
    np.testing.assert_array_equal(again[0],cold[0])


def test_single_segment_stair_preserves_original_order_and_object():
    points=np.array([[1.,0.,0.],[0.,0.,0.]])
    p=provider(points);domain,_=cached_domain([points.copy()])
    assert owned_stair_source_returns(p,points,domain[0],domain,('task','route')) is points
