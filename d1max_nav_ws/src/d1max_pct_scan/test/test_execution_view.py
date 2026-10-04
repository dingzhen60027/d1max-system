from copy import deepcopy
from types import SimpleNamespace as N
from d1max_pct_scan.execution_view import CommittedView


def at(ns):return N(sec=ns//10**9,nanosec=ns%10**9)


def fixture():
    v=N(schema_version=3,session_id='s',task_id='t',route_id='r',route_hash='hash',map_version_id='map',
        localization_epoch=1,localization_seed_id='seed',reference_generation=1,segment_id='seg',
        anchor_id='a',anchor_revision=1,context_sequence=1,map_geometry_revision=1)
    p=N(version=v,transport_mode='isolated_mock',sequence=1,trajectory_id=2,geometry_committed=True,
        revoked=False,allowed=False,frame_id='d1max_loc_odom',source_stamp=at(1000000000),valid_until=at(1100000000))
    c=N(**{k:x for k,x in vars(v).items() if k not in ('reference_generation','schema_version')},
        schema_version=2,generation=1,trajectory=N(traj_id=2),frame_id='d1max_loc_odom')
    proof=N(version=v,transport_mode='isolated_mock',sequence=1,trajectory_id=2,valid=True,
        frame_id='d1max_loc_odom',valid_until=at(1100000000),check_end=at(1000000000))
    core=CommittedView('s','isolated_mock');core.permit_received(p);core.curve_received(c);core.proof_received(proof)
    return core,p,c,proof


def test_geometry_preview_does_not_require_motion_permission():
    core,p,_,_=fixture()
    assert not p.allowed
    assert core.selected(1000000001) is not None


def test_candidate_does_not_replace_committed_shape():
    core,_,c,_=fixture();new=deepcopy(c);new.trajectory.traj_id=3;core.curve_received(new)
    assert core.selected(1000000001)[0].trajectory.traj_id==2


def test_hold_preserves_committed_shape_but_revocation_and_heartbeat_expiry_remove_it():
    core,p,_,proof=fixture()
    assert core.selected(1350000000) is None
    proof.valid=False;proof.sequence=2;core.proof_received(proof)
    assert core.selected(1000000001) is not None
    proof.valid=True;proof.sequence=3;core.proof_received(proof)
    p.revoked=True;p.sequence=2;core.permit_received(p)
    assert core.selected(1000000001) is None


def test_wrong_anchor_or_task_cannot_appear_as_current():
    core,p,_,_=fixture();p.version.anchor_revision=2;p.sequence+=1;core.permit_received(p)
    assert core.selected(1000000001) is None
