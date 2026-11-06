"""Production map loaders and support callback; no ROS graph, SDK or native run."""
from copy import deepcopy
import json
from pathlib import Path

import numpy as np
import pytest

from d1max_pct_scan.continuous_reference import SupportEvidence
from d1max_pct_scan.continuous_reference_node import ReferenceCallbacks
from d1max_pct_scan.control_frame_contract import Rigid
from d1max_pct_scan.reference_support import (ConditionedReferenceSupport,
    load_reference_support, nearby_source_returns)
from d1max_pct_scan.source_identity import SourceIdentityBridge
from d1max_pct_scan.source_route import SourceRouteBuilder, SourceSupport, canonical, digest_file
from test_navigation_map_package import conditioned_package
from test_source_identity_package import package as identity_package


def parameters(fixture):
    manifest = json.loads(fixture['manifest'].read_text())
    return dict(session_id='session',map_version_id='map',
        expected_source_map_sha256=manifest['source_sha256'],
        expected_conditioning_sha256=digest_file(fixture['manifest']),
        expected_tomogram_sha256=digest_file(fixture['tomogram']),
        planning_manifest=str(fixture['manifest']),body_height_m=.55,
        body_height_calibration_id='fixture',transport_mode='isolated_mock',floor_id='floor1')


def callbacks(fixture):
    output=[]
    node=ReferenceCallbacks(parameters(fixture),lambda *value:output.append(value),lambda:10_000_000_000)
    assert not output  # Initialization neither publishes geometry nor opens a graph.
    return node


def request(source_hash, *, floor='floor1', kind='floor', mode='general', center=(0.,0.),
            eligible=False, inverse=None):
    from d1max_planning_interfaces.msg import ReferenceProposal
    proposal=ReferenceProposal()
    proposal.proposal_id='support-proposal'
    proposal.version.anchor_id='measured-rigid-anchor'
    proposal.source_stamp.sec=10
    proposal.reference.segment_id='segment'
    proposal.reference.segment_kind=kind
    proposal.reference.required_mode=mode
    evidence=SupportEvidence(source_hash,eligible,'whole_route_not_execution_eligible',
                             np.empty((0,3)),canonical([]))
    return (proposal,evidence,inverse or Rigid((0.,0.,0.),(0.,0.,0.,1.)),center,.55,
            ('segment',floor,kind,mode))


def xyz(support):
    return np.array([[point.x,point.y,point.z] for point in support.support_ground_xyz]).reshape(-1,3)


@pytest.mark.parametrize('floor,z', [('floor1',-.75),('floor2',3.)])
def test_conditioned_callback_uses_classified_original_floor_rows_without_route_authority(
        conditioned_package,floor,z):
    node=callbacks(conditioned_package)
    assert isinstance(node.support_map,ConditionedReferenceSupport)
    original=json.loads(conditioned_package['manifest'].read_text())['source_sha256']
    req=request(original,floor=floor)
    literal=deepcopy(req[0])
    _,support=node.build_support(req)
    assert support.verified and support.reason=='original_floor_support_classified'
    assert not req[1].execution_eligible and req[0]==literal
    assert len(support.support_ground_xyz)==25
    assert support.floor_id==floor and support.frame_id=='d1max_loc_odom'
    assert support.source_stamp==literal.source_stamp and support.version==literal.version
    assert support.support_map_sha256==original
    np.testing.assert_array_equal(xyz(support)[:,2],np.full(25,z))
    # PCT Z is -.65/3.15; neither its values nor bridge interpolation are returns.
    assert not np.any(np.isclose(xyz(support)[:,2],-.65 if floor=='floor1' else 3.15))
    assert support.support_xy_radius_m==.2 and support.max_support_step_m==.05
    assert support.max_support_slope_rad==.15 and support.body_reference_height_m==.55


def test_support_ground_uses_only_rigid_anchor_not_body_height_or_nonrigid_bridge(conditioned_package):
    node=callbacks(conditioned_package)
    source_hash=parameters(conditioned_package)['expected_source_map_sha256']
    inverse=Rigid((1.,2.,.3),(0.,0.,0.,1.))
    _,support=node.build_support(request(source_hash,floor='floor2',inverse=inverse))
    raw=nearby_source_returns(node.support_map,(0.,0.),6.,'floor2')
    np.testing.assert_allclose(xyz(support),raw+[1.,2.,.3])
    expected=canonical(dict(anchor='measured-rigid-anchor',map=source_hash,points=raw.tolist()))
    import hashlib
    assert support.support_hash==hashlib.sha256(expected.encode()).hexdigest()


@pytest.mark.parametrize('fault',['empty','source_hash','stair','unknown_mode','wrong_kind'])
def test_support_without_matching_classified_floor_remains_unverified(conditioned_package,fault):
    node=callbacks(conditioned_package)
    source_hash=parameters(conditioned_package)['expected_source_map_sha256']
    kwargs={}
    if fault=='empty':kwargs['center']=(100.,100.)
    elif fault=='source_hash':source_hash='f'*64
    elif fault=='stair':kwargs.update(floor='stair_lower',kind='stair_up',mode='stair')
    elif fault=='unknown_mode':kwargs['mode']='unverified'
    else:kwargs['kind']='unknown'
    _,support=node.build_support(request(source_hash,**kwargs))
    assert not support.verified
    if fault=='empty':
        assert not support.support_ground_xyz and support.reason=='original_support_returns_missing'


def test_nonempty_protected_stair_points_are_not_classified_floor_support():
    source_hash='a'*64
    raw=SourceSupport({'stairs':[[0.,0.,1.],[.1,0.,1.]],
        'floor1':[[0.,0.,0.],[.1,0.,0.]]},source_hash)
    provider=ConditionedReferenceSupport(dict(source_sha256=source_hash),raw)
    assert len(provider.nearby((0.,0.),6.,'stairs'))==2
    assert not provider.classified_ground(source_map_sha256=source_hash,
        floor_id='stairs',segment_kind='stair_up',required_mode='stair')
    with pytest.raises(ValueError,match='unknown_floor'):
        provider.nearby((0.,0.),6.,'foreign')


def test_stair_callback_transports_measured_surface_not_flat_floor_capability():
    from geometry_msgs.msg import PoseStamped
    from test_stair_support import fixture
    grid,literal=fixture()
    source_hash='a'*64
    node=object.__new__(ReferenceCallbacks)
    node.support_map=ConditionedReferenceSupport({'source_sha256':source_hash},
        SourceSupport({'stairs':literal},source_hash),grid)
    ground=literal[np.isclose(literal[:,1],0.) & (literal[:,2]<1.)]
    # Nontrivial rigid translation must be undone once for association and
    # applied once to the outgoing raw supports; body height never shifts them.
    inverse=Rigid((1.,2.,.3),(0.,0.,0.,1.))
    req=request(source_hash,floor='stair_lower',kind='stair_up',mode='stair',inverse=inverse)
    for point in ground:
        body=inverse.point(tuple(float(v) for v in point+[0.,0.,.55]))
        pose=PoseStamped()
        pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=body
        req[0].reference.path.poses.append(pose)
    _,support=node.build_support(req)
    assert support.verified and support.reason=='original_stair_surface_returns_associated'
    assert support.max_support_slope_rad==support.max_support_step_m==0.
    raw=xyz(support)-[1.,2.,.3]
    for point in raw:
        assert np.min(np.linalg.norm(literal-point,axis=1))<1e-12
    assert np.ptp(raw[:,2])<1. and not req[1].execution_eligible
    assert support.version==req[0].version and support.source_stamp==req[0].source_stamp


def transition_request(reverse=False):
    from geometry_msgs.msg import PoseStamped
    from test_stair_support import fixture
    grid,literal=fixture();source_hash='a'*64
    floor=literal[(literal[:,2]<.01)]
    node=object.__new__(ReferenceCallbacks)
    node.support_map=ConditionedReferenceSupport({'source_sha256':source_hash},
        SourceSupport({'stairs':literal,'floor1':floor},source_hash),grid)
    ground=literal[np.isclose(literal[:,1],0.) & (literal[:,2]<1.)]
    floor_mask=ground[:,2]<.01
    ids=np.where(floor_mask,'entry','stairs').tolist()
    if reverse:ground=ground[::-1];ids=ids[::-1]
    contexts=[('entry','floor1','floor','general',0,2),
              ('stairs','stair_lower','stair_down' if reverse else 'stair_up','stair',2,12)]
    if reverse:contexts=contexts[::-1]
    first=contexts[0]
    inverse=Rigid((1.,2.,.3),(0.,0.,0.,1.))
    req=list(request(source_hash,floor=first[1],kind=first[2],mode=first[3],inverse=inverse))
    req[5]=(*first[:4],tuple(contexts));proposal=req[0]
    proposal.reference.segment_id=first[0];proposal.version.segment_id=first[0]
    for point,sid in zip(ground,ids):
        ctx=next(c for c in contexts if c[0]==sid)
        pose=PoseStamped();body=inverse.point(tuple(float(v) for v in point+[0.,0.,.55]))
        pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=map(float,body)
        proposal.reference.path.poses.append(pose)
        proposal.reference.point_segment_ids.append(sid)
        proposal.reference.point_segment_kinds.append(ctx[2])
        proposal.reference.point_required_modes.append(ctx[3])
    return node,tuple(req),literal


@pytest.mark.parametrize('reverse',[False,True])
def test_mixed_stair_portal_keeps_each_original_source_and_measured_current_identity(reverse):
    node,req,literal=transition_request(reverse)
    frozen=deepcopy(req[0]);_,support=node.build_support(req)
    assert req[0]==frozen
    assert support.verified and support.reason=='original_transition_surface_returns_associated'
    assert support.version==frozen.version and support.source_stamp==frozen.source_stamp
    assert support.required_mode==frozen.reference.required_mode
    assert support.max_support_step_m==support.max_support_slope_rad==0.
    assert len(support.regions)==2 and support.regions[0].segment_id==frozen.version.segment_id
    assert support.regions[0].begin_index==0
    assert support.regions[0].end_index==support.regions[1].begin_index
    assert support.regions[1].end_index==len(support.support_ground_xyz)
    raw=xyz(support)-[1.,2.,.3]
    for point in raw:assert np.min(np.linalg.norm(literal-point,axis=1))<1e-12
    for region in support.regions:
        if region.required_mode=='general':
            assert np.allclose(raw[region.begin_index:region.end_index,2],0.)
    # Same XY on the upper floor never enters this lower-flight transition.
    assert np.max(raw[:,2])<1.


@pytest.mark.parametrize('fault',['hash','missing_stairs','wrong_label','noncontiguous'])
def test_mixed_stair_portal_does_not_invent_support_or_ignore_metadata(fault):
    node,req,_=transition_request()
    if fault=='hash':node.support_map.source_support.source_hash='b'*64
    elif fault=='missing_stairs':node.support_map.stair_index=None
    elif fault=='wrong_label':req[0].reference.point_required_modes[-1]='general'
    else:req[0].reference.point_segment_ids[-1]='entry'
    if fault in ('wrong_label','noncontiguous'):
        with pytest.raises(ValueError,match='transition_source_context'):
            node.build_support(req)
    else:
        _,support=node.build_support(req)
        assert not support.verified and support.reason=='transition_source_support_incomplete'


@pytest.mark.parametrize('field',['expected_source_map_sha256','expected_conditioning_sha256',
                                  'expected_tomogram_sha256'])
def test_reference_initialization_rejects_foreign_pinned_artifact(conditioned_package,field):
    params=parameters(conditioned_package);params[field]='f'*64
    with pytest.raises(ValueError,match='reference_support_'):
        ReferenceCallbacks(params,lambda *args:None,lambda:10_000_000_000)


def test_original_identity_loader_and_point_order_are_unchanged(tmp_path):
    _,directory=identity_package(tmp_path)
    manifest=directory/'manifest.json'
    fixture=dict(manifest=manifest,tomogram=directory/'tomogram.npz')
    node=callbacks(fixture)
    assert isinstance(node.support_map,SourceIdentityBridge)
    old=SourceIdentityBridge.from_artifacts(manifest)
    expected=old.support[old.tree.query_ball_point((0.,0.),6.)]
    actual=nearby_source_returns(node.support_map,(0.,0.),6.,'floor1')
    np.testing.assert_array_equal(actual,expected)
    source_hash=parameters(fixture)['expected_source_map_sha256']
    for eligible in (False,True):
        _,support=node.build_support(request(source_hash,eligible=eligible))
        assert support.verified is eligible
        assert support.reason=='whole_route_not_execution_eligible'


def test_loading_raw_support_never_changes_conditioned_snapshot_eligibility(conditioned_package):
    from d1max_pct_scan.navigation_map_package import validate_package
    bound=validate_package(conditioned_package['directory'])
    builder=SourceRouteBuilder.from_artifacts(conditioned_package['manifest'],bound['tomogram'],bound['bridge'])
    snapshot=builder.build(dict(route_type='same_floor',layer_ids=[1,1,1],source_layer_ids=[1,1,1]),
        [[-.1,0.,3.15],[0.,0.,3.15],[.1,0.,3.15]],['floor2']*3,map_version_id='map')
    before=snapshot.encoded
    node=callbacks(conditioned_package)
    _,support=node.build_support(request(snapshot.payload()['source_map_sha256'],floor='floor2'))
    assert support.verified
    assert snapshot.encoded==before and snapshot.payload()['execution_eligible'] is False
    assert all(not segment['execution_eligible'] for segment in snapshot.payload()['segments'])


@pytest.mark.parametrize('floor,source_z,planning_z,layer',[
    ('floor1',-.75,-.65,0),('floor2',3.,3.15,1)])
def test_typed_conditioned_floor_route_prepares_bound_reference_and_support_not_motion(
        conditioned_package,floor,source_z,planning_z,layer):
    from d1max_navigation_bt_interfaces.msg import RouteReference
    from d1max_pct_scan.navigation_map_package import validate_package
    from d1max_pct_scan.source_route_ros import to_message
    from test_continuous_reference_transport import state
    bound=validate_package(conditioned_package['directory'])
    builder=SourceRouteBuilder.from_artifacts(conditioned_package['manifest'],bound['tomogram'],bound['bridge'])
    route=builder.build(dict(route_type='same_floor',layer_ids=[layer]*3,source_layer_ids=[layer]*3),
        [[-.1,0.,planning_z],[0.,0.,planning_z],[.1,0.,planning_z]],[floor]*3,map_version_id='map')
    output=[]
    node=ReferenceCallbacks(parameters(conditioned_package),
        lambda key,value:output.append((key,deepcopy(value))),lambda:10_000_000_000,lambda:100.)
    measured=state(x=-.1)
    measured.global_odometry.pose.pose.position.z=source_z+.55
    node.on_navigation(measured)
    node.on_context_ack(node.context)
    node.on_map(dict(node.context,valid=True,source_stamp_ns=10_000_000_000,
        sources=[dict(sensor_id=i,integrated_stamp_ns=10_000_000_000) for i in (0,1)]))
    delivery=RouteReference(schema_version=2,session_id='session',task_id='task',route_id='route',
        route_hash=route.route_hash,delivery_sequence=1,active=True)
    delivery.source_stamp=deepcopy(measured.source_stamp)
    delivery.snapshot=to_message(route,session_id='session',task_id='task',route_id='route',
        epoch=1,seed_id='seed',stamp=delivery.source_stamp)
    node.on_route(delivery)
    supports=[value for key,value in output if key=='support']
    proposals=[value for key,value in output if key=='proposal']
    assert len(supports)==len(proposals)==1
    assert supports[0].verified and supports[0].floor_id==floor
    assert supports[0].version==proposals[0].version
    assert supports[0].version.route_hash==route.route_hash
    np.testing.assert_allclose(xyz(supports[0])[:,2],0.,atol=1e-12)
    assert proposals[0].reference.path.header.frame_id=='d1max_loc_odom'
    assert proposals[0].reference.path.poses[0].pose.position.z==pytest.approx(.55)
    assert not route.payload()['execution_eligible']
    assert node.permit is None and node.local_commit is None and node.transport.accepted is None
    assert not [key for key,_ in output if key in ('sdk','motion','permit','handoff')]


def test_actual_restored_map_reference_support_initializes_both_floors():
    directory=Path(__file__).resolve().parents[3]/'experiments/navigation_runtime/map/crossfloor_20261002'
    if not (directory/'manifest.json').is_file():
        pytest.skip('requires the restored 10-02 actual map package')
    fixture=dict(manifest=directory/'manifest.json',tomogram=directory/'pct/tomogram.npz')
    node=callbacks(fixture)
    source_hash=parameters(fixture)['expected_source_map_sha256']
    for floor in ('floor1','floor2'):
        point=node.support_map.points[floor][0]
        _,support=node.build_support(request(source_hash,floor=floor,center=tuple(point[:2])))
        assert support.verified and support.support_ground_xyz
        assert support.support_map_sha256==source_hash and len(support.support_ground_xyz)<=20000
    assert not node.support_map.classified_ground(source_map_sha256=source_hash,
        floor_id='stairs',segment_kind='stair_up',required_mode='stair')
