"""Independent original riser observations never become terrain/qualification."""
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from d1max_pct_scan.source_route import SourceSupport, original_stair_record_lineage
from d1max_pct_scan.reference_support import (ConditionedReferenceSupport,
    original_stair_riser_candidates,canonical_riser_points)


def fixture(raw=None):
    lower=np.array([[0.,0.,0.],[1.,0.,.3],[2.,0.,.6]])
    upper=lower+[0.,1.,1.]
    floor=np.array([[-1.,0.,0.],[0.,0.,0.]])
    landing=np.array([[2.,0.,.6],[2.,1.,1.]])
    if raw is None:raw=np.array([[.5,0.,.05],[.5,1.,1.05],[2.,.5,.8],[-.8,0.,.1],[20.,0.,0.]])
    support=SourceSupport({'stairs':np.r_[lower,upper,landing], 'floor1':floor},'a'*64,
        raw_stair_evidence=raw,raw_stair_evidence_sha256='b'*64)
    p=ConditionedReferenceSupport({'source_sha256':'a'*64},support)
    segments=[('floor','floor1','floor','general',floor),('low','stair_lower','stair_up','stair',lower),
              ('landing','stairs','landing','stair',landing),('high','stair_upper','stair_up','stair',upper)]
    for *_,path in segments:path.flags.writeable=False
    domain=tuple((*c[:4],0,len(c[4])-1,c[4]) for c in segments)
    identity=('task','route')
    contexts=tuple((*c,domain,identity) for c in domain)
    return p,domain,contexts,raw


def test_independent_candidates_keep_literal_xyz_and_never_enter_terrain():
    p,domain,contexts,raw=fixture()
    ground=np.r_[domain[1][6],domain[2][6],domain[3][6]]
    before={key:tree.data.copy() for key,tree in p.source_support.trees.items()}
    result,owners=original_stair_riser_candidates(p,ground,source_hash='a'*64,contexts=contexts)
    np.testing.assert_array_equal(result,raw[:2]);assert owners==[1,3]
    assert all(tuple(point) in set(map(tuple,raw)) for point in result)
    assert not p.stair_contact_source
    for label,value in before.items():np.testing.assert_array_equal(p.source_support.trees[label].data,value)


def test_single_mixed_anchor_independent_owner_and_identity_change():
    p,domain,contexts,raw=fixture()
    single,owners=original_stair_riser_candidates(p,domain[1][6],source_hash='a'*64,
        context=domain[1],domain=domain,identity=('task','route'))
    np.testing.assert_array_equal(single,raw[:1]);assert owners==[0]
    mixed,mixed_owner=original_stair_riser_candidates(p,domain[1][6],source_hash='a'*64,contexts=contexts)
    assert tuple(single[0]) in set(map(tuple,mixed))
    assert mixed_owner[list(map(tuple,mixed)).index(tuple(single[0]))]==1
    cache=p._riser_owner_cache
    original_stair_riser_candidates(p,domain[1][6],source_hash='a'*64,
        context=domain[1],domain=domain,identity=('task-next','route'))
    assert p._riser_owner_cache is not cache


@pytest.mark.parametrize('case',['source_hash','floor','landing','missing_domain','no_independent_source'])
def test_no_qualification_inferred_from_missing_or_wrong_provenance(case):
    p,domain,contexts,raw=fixture()
    source='c'*64 if case=='source_hash' else 'a'*64
    context=domain[0] if case=='floor' else domain[2] if case=='landing' else domain[1]
    if case=='no_independent_source':p.source_support.raw_stair_evidence=None
    values,owners=original_stair_riser_candidates(p,domain[1][6],source_hash=source,
        context=context,domain=() if case=='missing_domain' else domain,identity=('task','route'))
    assert len(values)==0 and owners==[]


def test_large_packet_is_bounded_and_selection_contains_only_original_rows():
    raw=np.c_[np.linspace(.1,1.8,20033),np.zeros(20033),np.full(20033,.1)]
    p,domain,_,_=fixture(raw)
    values,owners=original_stair_riser_candidates(p,domain[1][6],source_hash='a'*64,
        context=domain[1],domain=domain,identity=('task','route'))
    assert len(values)==len(owners)==20000
    assert set(map(tuple,values)).issubset(set(map(tuple,raw)))
    assert len(p._riser_owner_cache[1])<=20000


def test_over_quota_keeps_complete_near_reference_patch_not_index_strides():
    from d1max_pct_scan.reference_support import _polyline_distance_squared
    y,z=np.meshgrid(np.linspace(-.1,.1,61),np.linspace(.1,.3,61))
    near=np.c_[np.ones(y.size),y.ravel(),z.ravel()]
    remote=np.c_[np.linspace(.1,1.9,21000),np.full(21000,.8),np.full(21000,.2)]
    raw=np.r_[remote,near]
    p,*_=fixture(raw)
    path=np.array([[0.,0.,.2],[2.,0.,.2]]);path.flags.writeable=False
    domain=(('flight','stairs','stair_up','stair',0,1,path),)
    selected,owners=original_stair_riser_candidates(p,path,source_hash='a'*64,
        context=domain[0],domain=domain,identity=('task','quota'))
    selected_rows=set(map(tuple,selected))
    assert len(selected)==20000 and owners==[0]*20000
    assert set(map(tuple,near))<=selected_rows<=set(map(tuple,raw))
    excluded=np.array([v for v in raw if tuple(v) not in selected_rows])
    assert _polyline_distance_squared(selected,path).max()<=_polyline_distance_squared(excluded,path).min()


def test_equal_distance_quota_ties_use_original_source_row_order():
    raw=np.c_[np.linspace(.1,1.9,20033),np.full(20033,.1),np.full(20033,.2)]
    raw=raw[np.random.default_rng(17).permutation(len(raw))]
    p,*_=fixture(raw)
    path=np.array([[0.,0.,.2],[2.,0.,.2]]);path.flags.writeable=False
    domain=(('flight','stairs','stair_up','stair',0,1,path),)
    selected,_=original_stair_riser_candidates(p,path,source_hash='a'*64,
        context=domain[0],domain=domain,identity=('task','quota'))
    assert set(map(tuple,selected))==set(map(tuple,raw[:20000]))
    again,_=original_stair_riser_candidates(p,path,source_hash='a'*64,
        context=domain[0],domain=domain,identity=('task','quota'))
    np.testing.assert_array_equal(selected,again)


def lineage_fixture(tmp_path, indices):
    path=tmp_path/'record_lineage.npz';np.savez(path,retained_source_indices=indices)
    digest=hashlib.sha256(path.read_bytes()).hexdigest()
    manifest={'support_consistency_repair':{'captured_at_build_inputs':{
        '/historical/frozen/package/record_lineage.npz':digest}}}
    return path,manifest


def test_lineage_uses_current_package_exact_recorded_hash_not_historical_location(tmp_path):
    path,manifest=lineage_fixture(tmp_path,np.array([0,2,4],dtype=np.int64))
    ids,actual,digest=original_stair_record_lineage(tmp_path/'manifest.json',manifest,5)
    np.testing.assert_array_equal(ids,[0,2,4]);assert actual==path and not ids.flags.writeable
    assert digest==hashlib.sha256(path.read_bytes()).hexdigest()
    manifest['support_consistency_repair']['captured_at_build_inputs'][
        '/historical/frozen/package/record_lineage.npz']='c'*64
    with pytest.raises(ValueError,match='lineage_changed'):
        original_stair_record_lineage(tmp_path/'manifest.json',manifest,5)


@pytest.mark.parametrize('indices',[[0,0],[0,6],[-1,2],[.1,2.]])
def test_lineage_rejects_duplicate_foreign_or_nonliteral_indices(tmp_path,indices):
    _,manifest=lineage_fixture(tmp_path,np.asarray(indices))
    with pytest.raises(ValueError,match='lineage_indices_invalid'):
        original_stair_record_lineage(tmp_path/'manifest.json',manifest,5)


def test_absent_old_lineage_contract_gives_no_independent_evidence(tmp_path):
    assert original_stair_record_lineage(tmp_path/'manifest.json',{},5) is None


def test_canonical_cache_preserves_exact_bytes_signed_zero_order_eviction_and_source_identity():
    from d1max_pct_scan.source_route import canonical
    p,*_=fixture()
    points=np.r_[np.random.default_rng(21).normal(size=(20001,3)),[[0.,-0.,1e-200],[-0.,0.,1e200]]]
    for values in (points,points[::-1],points[:21],points[:0]):
        assert canonical_riser_points(p,values)==canonical(values.tolist())
        assert len(p._riser_encoding_cache[1])<=20000
    old=p._riser_encoding_cache;p.source_support.raw_stair_evidence_sha256='c'*64
    assert canonical_riser_points(p,points[:2])==canonical(points[:2].tolist())
    assert p._riser_encoding_cache is not old


@pytest.mark.parametrize('reverse',[False,True])
def test_real_callback_keeps_riser_channel_separate_and_uses_same_anchor_version_hash(reverse):
    from copy import deepcopy
    from scipy.spatial import cKDTree
    from scipy.spatial.transform import Rotation
    from rclpy.serialization import serialize_message,deserialize_message
    from d1max_planning_interfaces.msg import SupportReference
    from d1max_pct_scan.control_frame_contract import Rigid
    from test_reference_support import transition_request,xyz
    node,req,literal=transition_request(reverse)
    assert 'stair_riser_xyz' in SupportReference.get_fields_and_field_types()
    old_inverse=req[2]
    raw_ground=np.asarray([old_inverse.inverse().point((p.pose.position.x,p.pose.position.y,p.pose.position.z))
        for p in req[0].reference.path.poses])-[0.,0.,.55]
    domain=[]
    for context in req[5][4]:
        ids=np.flatnonzero(np.asarray(req[0].reference.point_segment_ids)==context[0])
        path=raw_ground[max(0,ids[0]-1):min(len(raw_ground),ids[-1]+2)].copy();path.flags.writeable=False
        domain.append((*context,path))
    domain=tuple(domain);contexts=tuple((*c,domain,('task','route')) for c in domain)
    req=(*req[:5],(*req[5][:4],contexts))
    _,without=node.build_support(req)
    raw=literal[(literal[:,2]>.2)&(literal[:,2]<.8)][:9].copy();raw[:,2]-=.06
    node.support_map.source_support.raw_stair_evidence=cKDTree(raw)
    node.support_map.source_support.raw_stair_evidence_sha256='b'*64
    _,packet=node.build_support(req)
    assert packet.stair_riser_xyz and len(packet.stair_riser_xyz)==len(packet.stair_riser_owner)
    assert packet.support_ground_xyz==without.support_ground_xyz
    assert packet.stair_contact_indices==without.stair_contact_indices
    assert packet.support_hash!=without.support_hash
    recovered=np.asarray([old_inverse.inverse().point((p.x,p.y,p.z)) for p in packet.stair_riser_xyz])
    assert cKDTree(raw).query(recovered)[0].max()<1e-12
    assert all(packet.regions[i].required_mode=='stair' for i in packet.stair_riser_owner)
    assert packet.version==req[0].version and packet.source_stamp==req[0].source_stamp
    assert deserialize_message(serialize_message(packet),SupportReference)==packet
    new_inverse=Rigid((.2,-.1,.03),tuple(map(float,Rotation.from_euler('xyz',[.02,-.03,.4]).as_quat())))
    proposal=deepcopy(req[0]);proposal.version.anchor_id+='-new';proposal.version.anchor_revision+=1
    for p,ground in zip(proposal.reference.path.poses,raw_ground):
        p.pose.position.x,p.pose.position.y,p.pose.position.z=map(float,new_inverse.point(tuple(map(float,ground+[0.,0.,.55]))))
    _,corrected=node.build_support((proposal,req[1],new_inverse,req[3],req[4],req[5]))
    recovered2=np.asarray([new_inverse.inverse().point((p.x,p.y,p.z)) for p in corrected.stair_riser_xyz])
    np.testing.assert_allclose(recovered2,recovered,atol=1e-12)
    assert corrected.stair_riser_owner==packet.stair_riser_owner
    assert corrected.support_hash!=packet.support_hash and corrected.version==proposal.version
