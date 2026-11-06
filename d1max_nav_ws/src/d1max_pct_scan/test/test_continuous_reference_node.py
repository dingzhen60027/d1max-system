from copy import deepcopy
from types import SimpleNamespace
from d1max_localization.estimation_ros import stamp_time
import pytest
from d1max_planning_interfaces.msg import ReferenceReceipt,ExecutionPermit
from d1max_pct_scan.continuous_reference_node import (ReferenceCallbacks,committed_trajectory_status,
    candidate_preparation_status,version_of)
from d1max_pct_scan.continuous_reference_transport import set_stamp,nanoseconds
from d1max_pct_scan.execution_safety import version as execution_version_identity
from test_continuous_reference_transport import state,envelope,BASE_NS


def harness(preparation_ready=None,**overrides):
    clock=SimpleNamespace(ns=BASE_NS,mono=100.)
    output=[]
    p=dict(session_id='session',map_version_id='map',expected_source_map_sha256='a'*64,
        expected_tomogram_sha256='c'*64,expected_conditioning_sha256='b'*64,body_height_m=.55,
        body_height_calibration_id='fixture',transport_mode='isolated_mock',floor_id='floor1')
    p.update(overrides)
    c=ReferenceCallbacks(p,lambda k,v:output.append((k,deepcopy(v))),lambda:clock.ns,lambda:clock.mono,
        preparation_ready=preparation_ready)
    c.on_navigation(state())
    c.on_context_ack(c.context)
    c.on_map(dict(c.context,valid=True,source_stamp_ns=clock.ns,
        sources=[dict(sensor_id=i,integrated_stamp_ns=clock.ns) for i in (0,1)]))
    return c,clock,output


def local_geometry_commit(c,clock,proposal,sequence=1,trajectory=1,*,publish_proof=True,
                          snapshot_revision=17):
    from d1max_planning_interfaces.msg import LocalTrajectoryCommit,TrajectoryValidation
    current=c.local_commit
    message=LocalTrajectoryCommit(schema_version=1,version=deepcopy(proposal.version),trajectory_id=trajectory,
        expected_version=deepcopy(current.version if current else proposal.expected_version),
        expected_trajectory_id=current.trajectory_id if current else proposal.expected_trajectory_id,
        proposal_id=proposal.proposal_id,commit_sequence=sequence,native_validation_sequence=sequence,
        map_geometry_revision=snapshot_revision,accepted=True,
        disposition='committed',transport_mode='isolated_mock')
    set_stamp(message.source_stamp,clock.ns)
    body=c.transport.latest_local or c.transport.pair_local
    set_stamp(message.body_source_stamp,body.body.source_ns)
    for axis,value in zip('xyz',body.body.pose.xyz):setattr(message.measured_pose.position,axis,value)
    for axis,value in zip('xyzw',body.body.pose.xyzw):setattr(message.measured_pose.orientation,axis,value)
    if publish_proof:
        proof=local_commit_proof(c,clock,proposal,message)
        assert c.on_validation(proof)
    return message


def local_commit_proof(c,clock,proposal,message):
    from d1max_planning_interfaces.msg import TrajectoryValidation
    support=c.prepared_for(proposal.version).support
    proof=TrajectoryValidation(version=deepcopy(proposal.version),trajectory_id=message.trajectory_id,
        sequence=message.native_validation_sequence,proposal_id=proposal.proposal_id,
        map_snapshot_revision=message.map_geometry_revision,valid=True,whole_curve=True,
        frame_id='d1max_loc_odom',transport_mode='isolated_mock',
        collision_policy='official_inflated_geometry',support_reference_id=support.support_reference_id,
        support_hash=support.support_hash,checked_to_time=2.,curve_duration=2.)
    for name in ('source_stamp','check_begin','check_end'):
        setattr(proof,name,deepcopy(message.source_stamp))
    set_stamp(proof.valid_until,0)  # Formal SCAN state has no map-proof lease.
    proof.body_source_stamp=deepcopy(message.body_source_stamp)
    return proof


def test_local_executor_commit_installs_reference_without_sdk_or_bt_curve_permission():
    c,clock,out=harness(geometry_owner='local_executor');c.on_route(long_envelope())
    proposal=native_received(c);event=local_geometry_commit(c,clock,proposal)
    assert c.on_local_commit(event)
    assert c.transport.accepted.generation==proposal.version.reference_generation
    assert c.permit is None and not c.writer_unresolved()
    assert not c.on_permit(ExecutionPermit(version=proposal.version,geometry_committed=True))
    for i in range(1,43):drive(c,clock,t=.1*i,x=.025*i)
    assert c.proposal is not None and c.proposal.version!=event.version
    assert c.proposal.expected_version==event.version and c.proposal.expected_trajectory_id==event.trajectory_id
    assert not [v for k,v in out if k in ('handoff','sdk','motion')]


def test_same_reference_local_curve_commit_does_not_reinstall_or_rewind_progress():
    c,clock,_=harness(geometry_owner='local_executor');c.on_route(long_envelope())
    p=native_received(c);first=local_geometry_commit(c,clock,p)
    assert c.on_local_commit(first)
    drive(c,clock,t=.02,x=.02)
    core=c.transport.core;progress=deepcopy(core._progress)
    second=local_geometry_commit(c,clock,p,2,2)
    assert c.on_local_commit(second)
    assert c.transport.core is core and core._progress==progress
    assert c.local_commit.trajectory_id==2 and not c.on_local_commit(first)


def test_local_commit_before_native_ack_is_deferred_without_authority():
    c,clock,_=harness(geometry_owner='local_executor');c.on_route(envelope())
    p=deepcopy(c.proposal);event=local_geometry_commit(c,clock,p)
    assert not c.on_local_commit(event) and c.transport.accepted is None
    assert c.deferred_local_commit==event
    native_received(c)
    assert c.transport.accepted is not None and c.local_commit==event and c.permit is None


@pytest.mark.parametrize('bad',['task','hash','proposal','future','bodyfuture','stamp','predecessor',
    'trajectory','revision','nan','quaternion','negative','retired'])
def test_local_commit_rejects_wrong_or_noncanonical_fact_without_clearing_reference(bad):
    c,clock,_=harness(geometry_owner='local_executor');c.on_route(envelope())
    p=native_received(c);event=local_geometry_commit(c,clock,p)
    if bad=='task':event.version.task_id='foreign'
    elif bad=='hash':event.version.route_hash='foreign'
    elif bad=='proposal':event.proposal_id='foreign'
    elif bad=='future':set_stamp(event.source_stamp,clock.ns+1)
    elif bad=='bodyfuture':set_stamp(event.body_source_stamp,clock.ns+1)
    elif bad=='stamp':event.source_stamp.nanosec=1_000_000_000
    elif bad=='predecessor':event.expected_version.task_id='foreign'
    elif bad=='trajectory':event.expected_trajectory_id=20
    elif bad=='revision':event.map_geometry_revision+=1
    elif bad=='nan':event.measured_twist.linear.x=float('nan')
    elif bad=='quaternion':event.measured_pose.orientation.w=0.
    elif bad=='negative':event.accepted=False
    else:event.retired=True
    assert not c.on_local_commit(event)
    assert c.proposal==p and c.local_commit is None and c.transport.accepted is None


def test_retired_route_rejects_late_local_commit():
    c,clock,_=harness(geometry_owner='local_executor');c.on_route(envelope())
    p=native_received(c);event=local_geometry_commit(c,clock,p)
    c.on_route(envelope(sequence=2,active=False))
    assert not c.on_local_commit(event) and c.local_commit is None


def test_pending_reference_accepts_actual_current_curve_without_rewriting_proposal():
    c,clock,_=harness(geometry_owner='local_executor');c.on_route(long_envelope())
    first=native_received(c);assert c.on_local_commit(local_geometry_commit(c,clock,first))
    for i in range(1,43):drive(c,clock,t=.1*i,x=.025*i)
    pending=native_received(c);literal=deepcopy(c.proposal)
    refreshed=local_geometry_commit(c,clock,first,2,2)
    assert c.on_local_commit(refreshed)
    assert c.proposal==literal and c.transport.pending is not None
    new=local_geometry_commit(c,clock,pending,3,3)
    assert new.expected_trajectory_id==2 and pending.expected_trajectory_id==1
    assert c.on_local_commit(new)
    assert c.transport.accepted.generation==pending.version.reference_generation
    assert c.local_commit.trajectory_id==3 and c.transport.route.route_hash==literal.version.route_hash


def test_local_commit_binds_native_occupancy_revision_not_map_contract_revision():
    c,clock,_=harness(geometry_owner='local_executor');c.on_route(envelope())
    p=native_received(c);event=local_geometry_commit(c,clock,p,snapshot_revision=17)
    assert p.version.map_geometry_revision==1 and event.map_geometry_revision==17
    assert c.on_local_commit(event)
    assert c.local_commit==event and c.transport.accepted.generation==p.version.reference_generation


def test_local_commit_uses_original_positive_fact_without_reviving_latest_negative():
    c,clock,_=harness(geometry_owner='local_executor');c.on_route(envelope())
    p=native_received(c);event=local_geometry_commit(c,clock,p)
    positive=deepcopy(next(iter(c.local_commit_proofs.values())))
    drive(c,clock,t=.02,x=0.)
    negative=deepcopy(positive);negative.sequence=2;negative.valid=False;negative.reason='occupied'
    for name in ('source_stamp','check_begin','check_end','body_source_stamp'):
        set_stamp(getattr(negative,name),clock.ns)
    assert c.on_validation(negative)
    assert c.on_local_commit(event)  # geometry happened while seq 1 was valid
    key=(execution_version_identity(p.version),event.trajectory_id)
    assert c.validation_facts[key][0]==negative
    assert not c.tick()['spline_visual_valid'] and c.tick()['local_debug_phase']=='occupied'


def test_local_commit_before_delayed_proof_recovers_only_geometry_fact():
    c,clock,_=harness(geometry_owner='local_executor');c.on_route(envelope())
    p=native_received(c);event=local_geometry_commit(c,clock,p,publish_proof=False)
    proof=local_commit_proof(c,clock,p,event)
    assert not c.on_local_commit(event) and c.deferred_local_commit==event
    drive(c,clock,t=.6,x=0.)
    assert c.on_validation(proof)  # Late collision state, not a renewed motion lease.
    assert c.local_commit==event and c.transport.accepted is not None
    assert c.validation_facts and c.tick()['spline_visual_valid']
    assert c.permit is None and not c.writer_unresolved()
    assert nanoseconds(proof.source_stamp)==BASE_NS and nanoseconds(proof.valid_until)==0


def test_delayed_middle_commit_proof_reconciles_later_commit_before_rolling_window():
    """Native's reliable commits and proofs are independent topic streams."""
    c,clock,_=harness(geometry_owner='local_executor');c.on_route(long_envelope())
    p=native_received(c);first=local_geometry_commit(c,clock,p,1,1)
    assert c.on_local_commit(first)
    drive(c,clock,t=.02,x=.005)
    second=local_geometry_commit(c,clock,p,2,2,publish_proof=False)
    second_proof=local_commit_proof(c,clock,p,second)
    assert not c.on_local_commit(second)
    drive(c,clock,t=.04,x=.01)
    third=local_geometry_commit(c,clock,p,3,3,publish_proof=False)
    third.expected_version=deepcopy(second.version);third.expected_trajectory_id=2
    third_proof=local_commit_proof(c,clock,p,third)
    assert not c.on_local_commit(third)
    # Native has already completed 2 -> 3. Its result plus its original proof
    # is the geometry authority; Python must not re-run that CAS against 1.
    assert c.on_validation(third_proof) and c.local_commit==third
    # Neither a delayed geometry ACK nor its decision becomes a motion lease.
    drive(c,clock,t=.6,x=.01)
    assert c.on_validation(second_proof)
    assert c.local_commit==third and c.deferred_local_commit is None
    assert nanoseconds(third.source_stamp)==BASE_NS+40_000_000
    assert c.tick()['spline_visual_valid']
    drive(c,clock,t=.8,x=.3)
    assert c.proposal.expected_version==third.version
    assert c.proposal.expected_trajectory_id==3


def test_missing_middle_commit_recovers_native_snapshot_and_preserves_negative():
    c,clock,_=harness(geometry_owner='local_executor');c.on_route(long_envelope())
    p=native_received(c);first=local_geometry_commit(c,clock,p,1,1)
    assert c.on_local_commit(first)
    drive(c,clock,t=.02,x=.005)
    second=local_geometry_commit(c,clock,p,2,2,publish_proof=False)
    second_proof=local_commit_proof(c,clock,p,second)
    drive(c,clock,t=.04,x=.01)
    third=local_geometry_commit(c,clock,p,3,3,publish_proof=False)
    third.expected_version=deepcopy(second.version);third.expected_trajectory_id=2
    third_proof=local_commit_proof(c,clock,p,third)
    assert not c.on_local_commit(third) and c.local_commit==first
    assert c.on_validation(third_proof)
    assert c.local_commit==third
    assert c.on_validation(second_proof) and c.local_commit==third
    negative=deepcopy(third_proof);negative.sequence=4;negative.valid=False;negative.reason='occupied'
    assert c.on_validation(negative)
    assert not c.on_local_commit(second)
    assert c.local_commit==third
    assert not c.tick()['spline_visual_valid']
    assert c.tick()['local_debug_phase']=='occupied'
    key=(execution_version_identity(p.version),third.trajectory_id)
    assert c.validation_facts[key][0]==negative


def test_cancel_clears_out_of_order_commit_facts_without_reviving_old_route():
    c,clock,_=harness(geometry_owner='local_executor');c.on_route(long_envelope())
    p=native_received(c);first=local_geometry_commit(c,clock,p,1,1)
    assert c.on_local_commit(first)
    second=local_geometry_commit(c,clock,p,2,2,publish_proof=False)
    proof=local_commit_proof(c,clock,p,second)
    assert not c.on_local_commit(second)
    third=deepcopy(second);third.commit_sequence=3;third.trajectory_id=3
    third.expected_trajectory_id=2
    assert not c.on_local_commit(third)
    cancel=deepcopy(c.transport.route);cancel.delivery_sequence+=1;cancel.active=False
    c.on_route(cancel)
    assert c.local_commit is None and c.deferred_local_commit is None
    assert not c.on_validation(proof) and not c.on_local_commit(second)
    assert not c.reconcile_local_commit() and c.transport.accepted is None


def test_native_snapshot_recovers_across_rolling_generation_without_republishing_authority():
    c,clock,out=harness(geometry_owner='local_executor');c.on_route(long_envelope())
    first=native_received(c);assert c.on_local_commit(local_geometry_commit(c,clock,first))
    drive(c,clock,t=.6,x=.3)
    rolling=native_received(c)
    event=local_geometry_commit(c,clock,rolling,3,3,publish_proof=False)
    # Native refreshed the old window to curve 2; its ACK was never received.
    event.expected_version=deepcopy(first.version);event.expected_trajectory_id=2
    proof=local_commit_proof(c,clock,rolling,event)
    assert c.on_validation(proof)
    before=len(out)
    assert c.on_local_commit(event)
    assert c.local_commit==event and c.transport.accepted==rolling.reference
    assert c.transport.route.task_id==first.version.task_id
    assert c.transport.route.route_hash==first.version.route_hash
    assert not [kind for kind,_ in out[before:] if kind in ('proposal','handoff','commit','permit','sdk','motion')]


def test_rejected_rolling_predecessor_recovers_current_native_curve_watermark():
    c,clock,_=harness(geometry_owner='local_executor');c.on_route(long_envelope())
    first=native_received(c);assert c.on_local_commit(local_geometry_commit(c,clock,first))
    drive(c,clock,t=.6,x=.3)
    rejected=deepcopy(c.proposal)
    receipt=ReferenceReceipt(version=deepcopy(rejected.version),proposal_id=rejected.proposal_id,
        expected_version=deepcopy(rejected.expected_version),
        expected_trajectory_id=rejected.expected_trajectory_id,accepted=False,
        reason='rolling_reference_predecessor_changed_retry',transport_mode='isolated_mock')
    set_stamp(receipt.source_stamp,clock.ns);set_stamp(receipt.valid_until,clock.ns)
    assert not c.on_receipt(receipt) and c.transport.pending is None
    event=local_geometry_commit(c,clock,first,40,40,publish_proof=False)
    event.expected_trajectory_id=39
    assert c.on_validation(local_commit_proof(c,clock,first,event))
    assert c.on_local_commit(event)
    drive(c,clock,t=1.15,x=.4)
    assert c.proposal.version.reference_generation>rejected.version.reference_generation
    assert c.proposal.expected_version==event.version
    assert c.proposal.expected_trajectory_id==40
    assert c.transport.accepted==first.reference


@pytest.mark.parametrize('bad',['foreign_predecessor','future_predecessor','rollback_source','unproved','rejected_receipt'])
def test_native_snapshot_recovery_requires_known_identity_receipt_and_original_proof(bad):
    c,clock,_=harness(geometry_owner='local_executor');c.on_route(long_envelope())
    first=native_received(c);original=local_geometry_commit(c,clock,first)
    assert c.on_local_commit(original)
    drive(c,clock,t=.6,x=.3);rolling=native_received(c)
    event=local_geometry_commit(c,clock,rolling,3,3,publish_proof=False)
    event.expected_version=deepcopy(first.version);event.expected_trajectory_id=2
    if bad=='foreign_predecessor':event.expected_version.anchor_id='foreign'
    elif bad=='future_predecessor':event.expected_version=deepcopy(rolling.version);event.expected_version.reference_generation+=1
    elif bad=='rollback_source':set_stamp(event.source_stamp,BASE_NS-1)
    elif bad=='rejected_receipt':c.prepared_for(rolling.version).receipt.accepted=False
    if bad not in ('unproved','rollback_source'):
        assert c.on_validation(local_commit_proof(c,clock,rolling,event))
    assert not c.on_local_commit(event)
    assert c.local_commit==original and c.transport.accepted==first.reference


def test_deferred_commit_facts_are_bounded_and_newer_proved_native_snapshot_unblocks():
    c,clock,_=harness(geometry_owner='local_executor');c.on_route(long_envelope())
    p=native_received(c);assert c.on_local_commit(local_geometry_commit(c,clock,p))
    events=[]
    for i in range(2,25):
        event=local_geometry_commit(c,clock,p,i,i,publish_proof=False)
        event.expected_trajectory_id=i-1
        assert not c.on_local_commit(event)
        events.append(event)
    assert len(c.deferred_local_commits)==c.local_commit_fact_limit==16
    assert c.local_commit.trajectory_id==1
    # Old missing evidence cannot block an actually committed, fully evidenced
    # newer Native snapshot. No missing intermediate proof is manufactured.
    assert min(c.deferred_local_commits)==9 and max(c.deferred_local_commits)==24
    selected=events[-1]
    assert c.on_validation(local_commit_proof(c,clock,p,selected))
    assert c.local_commit==selected and not c.deferred_local_commits


@pytest.mark.parametrize('accepted',[True,False])
def test_first_observed_native_snapshot_before_reference_receipt_waits_without_losing_fact(accepted):
    c,clock,_=harness(geometry_owner='local_executor');c.on_route(long_envelope())
    p=deepcopy(c.proposal)
    event=local_geometry_commit(c,clock,p,3,3,publish_proof=False)
    event.expected_version=deepcopy(p.version);event.expected_trajectory_id=2
    assert not c.on_local_commit(event) and c.deferred_local_commit==event
    assert c.on_validation(local_commit_proof(c,clock,p,event)) and c.local_commit is None
    if accepted:
        native_received(c)
        assert c.local_commit==event and c.transport.accepted==p.reference
    else:
        receipt=ReferenceReceipt(version=deepcopy(p.version),proposal_id=p.proposal_id,
            expected_version=deepcopy(p.expected_version),expected_trajectory_id=p.expected_trajectory_id,
            accepted=False,reason='reference_rejected',transport_mode='isolated_mock')
        set_stamp(receipt.source_stamp,clock.ns);set_stamp(receipt.valid_until,clock.ns)
        assert not c.on_receipt(receipt)
        assert c.local_commit is None and c.transport.accepted is None
        assert not c.reconcile_local_commit()


def test_reference_diagnostics_report_receipt_reason_and_original_window_watermark():
    c,clock,_=harness(geometry_owner='local_executor');logs=[];c.diagnostic=logs.append
    c.on_route(long_envelope());p=native_received(c)
    assert c.on_local_commit(local_geometry_commit(c,clock,p))
    c.tick()
    assert any('"event": "receipt"' in line for line in logs)
    assert any('"event": "proposal"' in line and '"source_age_ms": 0.0' in line
        and '"same_anchor": false' in line for line in logs)
    assert any('"event": "window"' in line and '"issued_arc_m": 0.0' in line for line in logs)
    count=len(logs)
    for _ in range(100):c.tick()
    assert len(logs)==count  # No source advance/timer flood creates new facts.


def test_measured_progress_refresh_keeps_same_anchor_during_small_pair_corrections():
    """Use the real node/transport flow, not a separate scheduling model."""
    c,clock,_=harness(geometry_owner='local_executor',reference_horizon_m=4.)
    logs=[];c.diagnostic=logs.append
    c.on_route(long_envelope(10));first=native_received(c)
    assert c.on_local_commit(local_geometry_commit(c,clock,first))
    for i in range(1,7):
        t=.1*i;clock.ns=BASE_NS+int(t*1e9);clock.mono=100.+t
        c.on_map(dict(c.context,valid=True,source_stamp_ns=clock.ns,
            sources=[dict(sensor_id=j,integrated_stamp_ns=clock.ns) for j in (0,1)]))
        assert c.on_navigation(state(t=t,x=.05*i,correction=.001*i))
        c.tick()
    rolling=c.proposal
    assert rolling is not None and rolling.version.reference_generation>first.version.reference_generation
    assert rolling.version.anchor_id==first.version.anchor_id
    assert rolling.version.anchor_revision==first.version.anchor_revision
    assert rolling.version.map_geometry_revision==first.version.map_geometry_revision
    assert rolling.reference.path.poses[0].pose.position.x>=.25
    assert c.transport.core.candidate_anchor is not None
    assert not c.transport.correction_due()
    assert any('"event": "proposal"' in line and '"same_anchor": true' in line for line in logs)
    assert c.transport.accepted==first.reference
    assert native_received(c)==rolling
    assert c.on_local_commit(local_geometry_commit(c,clock,rolling,2,2))
    assert c.transport.accepted==rolling.reference and c.transport.core.candidate_anchor is not None


def test_ten_metre_rolling_with_missing_commits_then_cancel_and_replace_target():
    """Real ReferenceCallbacks, synthetic wire ordering; not a motion/bag test."""
    from d1max_pct_scan.source_route_ros import from_message,to_message
    from test_continuous_reference import snapshot
    c,clock,out=harness(geometry_owner='local_executor',reference_horizon_m=4.)
    route=long_envelope(20);c.on_route(route)
    current=native_received(c);seq=1
    assert c.on_local_commit(local_geometry_commit(c,clock,current,seq,seq))
    windows=1;recovered=0;overlaps=[];end=4.
    for i in range(1,668):
        drive(c,clock,t=.05*i,x=.015*i)
        if c.transport.pending is None:
            continue
        assert c.proposal.expected_trajectory_id==seq
        proposal=native_received(c)
        start=proposal.reference.path.poses[0].pose.position.x
        overlaps.append((end-start)/4.)
        assert overlaps[-1]>=.75
        if windows%3==0:
            # Native really advanced the old reference; deliberately drop
            # that result and original proof at this observing callback.
            seq+=1
            skipped=local_geometry_commit(c,clock,current,seq,seq,publish_proof=False)
            seq+=1
            event=local_geometry_commit(c,clock,proposal,seq,seq,publish_proof=False)
            event.expected_version=deepcopy(skipped.version)
            event.expected_trajectory_id=skipped.trajectory_id
            proof=local_commit_proof(c,clock,proposal,event)
            assert not c.on_local_commit(event)
            if windows%2:
                negative=deepcopy(proof);negative.sequence+=1;negative.valid=False;negative.reason='occupied'
                assert c.on_validation(negative)
                assert c.on_validation(proof)
                assert c.local_commit==event and not c.tick()['spline_visual_valid']
                assert c.tick()['local_debug_phase']=='occupied'
            else:
                assert c.on_validation(proof) and c.local_commit==event
            recovered+=1
        else:
            seq+=1;event=local_geometry_commit(c,clock,proposal,seq,seq)
            assert c.on_local_commit(event)
        current=proposal;windows+=1
        end=proposal.reference.path.poses[-1].pose.position.x
        assert c.transport.route==route and c.local_commit.trajectory_id==seq
    assert windows>=35 and recovered>=10
    assert c.transport.core._progress.confirmed_arc_m==pytest.approx(10.005)
    old=deepcopy(c.local_commit)
    cancel=deepcopy(route);cancel.delivery_sequence=2;cancel.active=False
    set_stamp(cancel.source_stamp,clock.ns)
    cancel.snapshot=to_message(from_message(route.snapshot),session_id=route.session_id,
        task_id=route.task_id,route_id=route.route_id,epoch=1,seed_id='seed',stamp=cancel.source_stamp)
    c.on_route(cancel)
    assert c.local_commit is None and not c.deferred_local_commits
    assert c.on_debug(reset_debug(c,clock,c.cancel_generation))
    replacement=deepcopy(route)
    replacement.task_id='replacement';replacement.route_id='replacement-route';replacement.delivery_sequence=3
    set_stamp(replacement.source_stamp,clock.ns)
    new_snapshot=snapshot(points=[[10.005+.5*x,0.,0.] for x in range(21)])
    replacement.route_hash=new_snapshot.route_hash
    replacement.snapshot=to_message(new_snapshot,session_id='session',task_id=replacement.task_id,
        route_id=replacement.route_id,epoch=1,seed_id='seed',stamp=replacement.source_stamp)
    c.on_route(replacement)
    assert not c.on_local_commit(old)
    next_proposal=native_received(c);seq+=1
    assert c.on_local_commit(local_geometry_commit(c,clock,next_proposal,seq,seq))
    assert c.local_commit.version.task_id=='replacement'
    for i in range(668,700):
        drive(c,clock,t=.05*i,x=.015*i)
        if c.transport.pending is not None:
            next_proposal=native_received(c);seq+=1
            assert c.on_local_commit(local_geometry_commit(c,clock,next_proposal,seq,seq))
    assert c.transport.route==replacement and not c.on_local_commit(old)
    assert c.transport.accepted.path.poses[0].pose.position.x>10.005
    # A completed collision state is not a timer lease. Advancing the clock
    # alone cannot invent a collision or reset the already committed window.
    drive(c,clock,t=35.5,x=.015*699)
    assert c.tick()['spline_visual_valid']
    assert not [kind for kind,_ in out if kind in ('handoff','commit','permit','sdk','motion')]
    print(f'reference_resume measured_m=10.005 windows={windows} lost_intermediate_recoveries={recovered} '
        f'min_overlap={min(overlaps):.6f} replacement_accepted=1 old_task_revival=0 '
        'proof_time_unchanged=1 negative_verdict_preserved=1')


@pytest.mark.parametrize('bad',['snapshot','sequence','proposal','support','proof_future',
                               'proof_stamp','legacy_lease','before_check','retired'])
def test_local_commit_native_fact_mismatch_never_installs_or_renews(bad):
    c,clock,_=harness(geometry_owner='local_executor');c.on_route(envelope())
    p=native_received(c);event=local_geometry_commit(c,clock,p,publish_proof=False)
    proof=local_commit_proof(c,clock,p,event)
    if bad=='snapshot':proof.map_snapshot_revision+=1
    elif bad=='sequence':proof.sequence+=1
    elif bad=='proposal':proof.proposal_id='foreign'
    elif bad=='support':proof.support_hash='f'*64
    elif bad=='proof_future':
        for name in ('check_begin','check_end','source_stamp'):set_stamp(getattr(proof,name),clock.ns+1)
    elif bad=='proof_stamp':proof.source_stamp.nanosec=1_000_000_000
    elif bad=='legacy_lease':
        set_stamp(proof.valid_until,clock.ns+250_000_000)
    elif bad=='before_check':
        drive(c,clock,t=.02,x=0.)
        for name in ('check_begin','check_end','source_stamp'):set_stamp(getattr(proof,name),clock.ns)
    else:c.on_route(envelope(sequence=2,active=False))
    try:c.on_validation(proof)
    except ValueError:pass
    assert not c.on_local_commit(event)
    assert c.local_commit is None and c.transport.accepted is None and c.permit is None


def test_historical_local_commit_proofs_are_bounded_without_live_lease_refresh():
    c,clock,_=harness(geometry_owner='local_executor');c.on_route(envelope())
    p=native_received(c);event=local_geometry_commit(c,clock,p,publish_proof=False)
    original=local_commit_proof(c,clock,p,event)
    for i in range(1,22):
        proof=deepcopy(original);proof.sequence=i;proof.trajectory_id=i
        assert c.on_validation(proof)
    assert len(c.local_commit_proofs)==16 and len(c.validation_facts)==c.prepared_limit
    assert not c.on_local_commit(event)  # an evicted fact isn't invented
    assert c.deferred_local_commit==event and c.local_commit is None


@pytest.mark.parametrize('legacy_lease_ns',[1,250_000_000,250_003_000,10_000_000_000])
def test_formal_state_observer_rejects_old_timed_proof_contract(legacy_lease_ns):
    """No mixing old short leases (or disguised longer leases) into state."""
    c,clock,_=harness(geometry_owner='local_executor');logs=[];c.diagnostic=logs.append
    c.on_route(long_envelope());p=native_received(c)
    event=local_geometry_commit(c,clock,p,publish_proof=False)
    proof=local_commit_proof(c,clock,p,event)
    set_stamp(proof.valid_until,clock.ns+legacy_lease_ns)
    original=deepcopy(proof)
    assert not c.on_validation(proof)
    assert not c.on_local_commit(event)
    assert c.local_commit is None and c.deferred_local_commit==event
    assert not c.local_commit_proofs and not c.validation_facts
    assert proof==original and nanoseconds(proof.check_end)==clock.ns
    assert c.permit is None


def test_formal_state_persists_until_new_scan_decision_not_clock_or_old_positive():
    c,clock,out=harness(geometry_owner='local_executor');c.on_route(long_envelope())
    p=native_received(c);commit=local_geometry_commit(c,clock,p)
    assert c.on_local_commit(commit)
    positive=deepcopy(next(iter(c.validation_facts.values()))[0])
    for elapsed in (.25,.6,1.5,3.):
        drive(c,clock,t=elapsed,x=0.)
        assert c.tick()['spline_visual_valid']
        assert c.local_commit==commit
    # This observer has never been a motion authority. A real newer SCAN
    # obstacle verdict, not a notification clock, changes its display state.
    negative=deepcopy(positive);negative.sequence+=1;negative.valid=False;negative.reason='occupied'
    for name in ('source_stamp','check_begin','check_end','body_source_stamp'):
        set_stamp(getattr(negative,name),clock.ns)
    assert c.on_validation(negative)
    assert not c.tick()['spline_visual_valid']
    c.on_validation(positive)
    assert not c.tick()['spline_visual_valid']
    assert c.tick()['local_debug_phase']=='occupied'
    assert not [k for k,_ in out if k in ('permit','handoff','commit','sdk','motion')]


@pytest.mark.parametrize('map_status',['missing','stale','updating'])
def test_formal_reference_delivery_does_not_wait_for_map_lease(map_status):
    c,clock,out=harness(geometry_owner='local_executor')
    if map_status=='missing':
        c.map_report=None
    elif map_status=='stale':
        c.map_received=clock.mono-1.
    else:
        c.map_report['valid']=False
    assert not c.map_fresh()
    assert c.reference_inputs_ready()
    assert c.on_route(long_envelope())
    assert c.proposal is not None and c.transport.pending is not None
    assert any(k=='proposal' for k,_ in out)
    # There is no route movement permission until the actual local executor
    # plans/commits and the task is authorized; this is geometry delivery only.
    assert not [k for k,_ in out if k in ('permit','handoff','commit','sdk','motion')]


def test_formal_reference_still_requires_real_context_ack():
    c,clock,out=harness(geometry_owner='local_executor')
    c.context_ready=False
    assert not c.reference_inputs_ready()
    c.on_route(long_envelope())
    assert c.proposal is None
    assert not [k for k,_ in out if k=='proposal']


def test_real_reference_entry_accepts_provider_coast_without_dropping_first_window():
    c,clock,out=harness()
    c.p['local_state_enabled']=True
    clock.ns+=20_000_000;clock.mono+=.02
    message=state(t=.02)
    message.posterior_stamp=stamp_time(9.63)
    message.imu_stamp=stamp_time(9.9254)
    message.extrapolation_sec=.0946
    source=nanoseconds(message.source_stamp)
    clock.ns+=25_900_000;clock.mono+=.0259
    assert c.on_local_navigation(local_state_from_pair(message))
    assert c.on_navigation(message)
    assert c.state.source_ns==source and c.state.imu_ns==9_925_400_000
    c.on_map(dict(c.context,valid=True,source_stamp_ns=clock.ns,
        sources=[dict(sensor_id=i,integrated_stamp_ns=clock.ns) for i in (0,1)]))
    c.on_route(envelope(t=.0459))
    assert c.proposal is not None
    assert c.proposal.reference.path.header.stamp==message.source_stamp
    assert [value for kind,value in out if kind=='proposal'][-1]==c.proposal
    assert c.transport.accepted is None and c.permit is None


@pytest.mark.parametrize('bad',['raw_coast','posterior','source_expired'])
def test_real_reference_entry_rejects_bad_provenance_without_replacing_owner(bad):
    c,clock,_=harness();c.on_route(envelope())
    state_before,proposal=c.state,c.proposal
    message=state(t=.02)
    clock.ns+=30_000_000;clock.mono+=.03
    if bad=='raw_coast':
        message.imu_stamp=stamp_time(9.919999999);message.extrapolation_sec=.1
    elif bad=='posterior':message.posterior_stamp=stamp_time(9.619999999)
    else:clock.ns=BASE_NS+120_000_001;clock.mono=100.120000001
    with pytest.raises(ValueError):c.on_navigation(message)
    assert c.state is state_before and c.proposal is proposal
    assert c.transport.accepted is None and c.permit is None


def test_same_clock_local_reset_fences_old_coasting_global_anchor():
    c,clock,_=harness();c.p['local_state_enabled']=True
    clock.ns+=25_900_000;clock.mono+=.0259
    local_reset=state(t=.02,epoch=2)
    assert c.on_local_navigation(local_state_from_pair(local_reset))
    reset_context=deepcopy(c.context)
    old=state(t=.02,epoch=1)
    old.posterior_stamp=stamp_time(9.63)
    old.imu_stamp=stamp_time(9.9254);old.extrapolation_sec=.0946
    assert not c.on_navigation(old)
    assert c.context==reset_context and c.local_inbox.latest.identity==('session',2,'seed')
    assert c.state.source_ns==BASE_NS
    # Only the genuine new same-time global pair may establish its anchor.
    assert c.on_navigation(local_reset)
    assert c.context['epoch']==2 and c.state.source_ns==nanoseconds(local_reset.source_stamp)
    assert not c.on_local_navigation(local_state_from_pair(old))
    assert c.transport.route is None and c.transport.accepted is None


def admit(c, sequence=1):
    proposal=c.proposal
    receipt=ReferenceReceipt(version=proposal.version,proposal_id=proposal.proposal_id,
        expected_version=proposal.expected_version,expected_trajectory_id=proposal.expected_trajectory_id,
        source_stamp=proposal.source_stamp,valid_until=proposal.valid_until,accepted=True,transport_mode='isolated_mock')
    assert c.on_receipt(receipt)
    permit=ExecutionPermit(version=proposal.version,source_stamp=proposal.source_stamp,
        valid_until=proposal.valid_until,phase='preview',geometry_committed=True,allowed=False,
        sequence=sequence,trajectory_id=sequence,transport_mode='isolated_mock')
    assert c.on_permit(permit)
    return permit


def test_async_support_does_not_delay_proposal_or_expire_admitted_static_geometry():
    from threading import Event
    from d1max_pct_scan.bounded_preparation import LatestPreparation
    c,clock,out=harness();c.p['local_state_enabled']=True
    started,release=Event(),Event()
    def prepare(request):
        started.set();assert release.wait(1.)
        return c.build_support(request)
    c.preparation=LatestPreparation(prepare)
    try:
        c.on_route(envelope());assert started.wait(1.)
        original=native_received(c)
        assert len([v for k,v in out if k=='proposal'])==1
        assert not [v for k,v in out if k=='support']
        clock.ns+=20_000_000;clock.mono+=.02
        assert c.on_local_navigation(local_state_from_pair(state(t=.02,x=.005)))
        assert [v for k,v in out if k=='body'][-1].header.stamp==state(t=.02).source_stamp
        # Admission is already factual. Static support is not a heartbeat.
        clock.ns+=500_000_000;clock.mono+=.5
        release.set()
        with c.preparation._cv:
            assert c.preparation._cv.wait_for(lambda:c.preparation._result is not None,timeout=1.)
        c.tick()
        supports=[v for k,v in out if k=='support']
        assert len(supports)==1 and supports[0].source_stamp==original.source_stamp
        assert c.proposal==original and c.receipt.accepted
    finally:release.set();assert c.close()


def test_async_support_uses_same_geometry_and_source_as_synchronous_owner():
    from d1max_pct_scan.bounded_preparation import LatestPreparation
    expected,_,sync=harness();expected.on_route(envelope())
    c,clock,out=harness();c.preparation=LatestPreparation(c.build_support)
    try:
        c.on_route(envelope())
        with c.preparation._cv:
            assert c.preparation._cv.wait_for(lambda:c.preparation._result is not None,timeout=1.)
        c.tick()
        assert [v for k,v in out if k=='proposal']==[v for k,v in sync if k=='proposal']
        assert [v for k,v in out if k=='support']==[v for k,v in sync if k=='support']
    finally:assert c.close()


def test_ready_support_wakes_owner_and_delivers_before_next_50_ms_tick():
    from threading import Event,get_ident
    expected,_,sync=harness();expected.on_route(envelope())
    owner=get_ident();wake=Event();notifications=[]
    def notify():notifications.append(get_ident());wake.set()
    c,clock,out=harness(async_support_preparation=True,preparation_ready=notify)
    try:
        c.on_route(envelope());assert wake.wait(1.)
        assert notifications and notifications[0]!=owner
        assert len([value for kind,value in out if kind=='proposal'])==1
        assert not [value for kind,value in out if kind=='support']
        original=deepcopy(c.proposal)
        # This is the serialized executor's guard callback, not a control or
        # status tick. Worker completion alone cannot publish any geometry.
        clock.ns+=1_000_000;clock.mono+=.001
        assert c.harvest_preparation()
        with c.support_delivery._cv:
            assert c.support_delivery._cv.wait_for(lambda:c.support_delivery._result is not None,timeout=1.)
        assert c.proposal==original
        for kind in ('proposal','support'):
            assert [value for key,value in out if key==kind]==[value for key,value in sync if key==kind]
        assert not c.harvest_preparation()  # A coalesced/repeated wake is idle.
        c.tick()
        assert len([value for key,value in out if key=='proposal'])==1
    finally:assert c.close()


@pytest.mark.parametrize('dispose',['cancel','expired','map_invalid'])
def test_ready_wake_fences_cancel_but_static_support_does_not_renew_motion(dispose):
    from threading import Event
    wake=Event()
    c,clock,out=harness(async_support_preparation=True,preparation_ready=wake.set)
    try:
        c.on_route(envelope());assert wake.wait(1.)
        original=native_received(c)
        if dispose=='cancel':
            c.on_route(envelope(sequence=2,active=False))
        elif dispose=='expired':
            clock.ns=nanoseconds(original.valid_until)+1
            clock.mono+=.501
        else:
            assert c.on_map(dict(c.context,valid=False,source_stamp_ns=clock.ns,
                sources=[dict(sensor_id=i,integrated_stamp_ns=clock.ns) for i in (0,1)]))
        assert c.harvest_preparation()==(dispose!='cancel')
        if dispose!='cancel':
            with c.support_delivery._cv:
                assert c.support_delivery._cv.wait_for(lambda:c.support_delivery._result is not None,timeout=1.)
            assert [value for kind,value in out if kind=='support'][0].source_stamp==original.source_stamp
            assert c.proposal==original and c.permit is None
        else:
            assert not [value for kind,value in out if kind=='support']
        assert len([value for kind,value in out if kind=='proposal'])==1
    finally:assert c.close()


def test_queued_old_completion_wake_cannot_preempt_replacement_goal():
    from threading import Event
    wake=Event()
    c,clock,out=harness(async_support_preparation=True,preparation_ready=wake.set)
    try:
        c.on_route(delivery_for('old',sequence=1));assert wake.wait(1.)
        old=deepcopy(c.proposal)
        c.on_route(delivery_for('old',sequence=2,active=False))
        c.on_debug(reset_debug(c,clock,c.cancel_generation))
        wake.clear()
        c.on_route(delivery_for('replacement',sequence=3));assert wake.wait(1.)
        assert c.harvest_preparation()
        delivered=[value for kind,value in out if kind=='proposal']
        assert len(delivered)==2 and delivered[-1].version.task_id=='replacement'
        assert delivered[-1].version!=old.version
        assert not c.harvest_preparation()
        assert c.transport.route.task_id=='replacement'
    finally:assert c.close()


def test_cached_support_preserves_literal_hash_and_never_reparses_full_route(monkeypatch):
    import hashlib
    from d1max_pct_scan.continuous_reference import SupportEvidence, _immutable_array
    from d1max_pct_scan.source_route import RouteSnapshot, canonical
    c,_,out=harness();c.on_route(envelope())
    core=c.transport.pending_core or c.transport.core
    points=[[0,0,-0.0],[1.0,0,0]]
    core.support_evidence=SupportEvidence('a'*64,False,'fixture',_immutable_array(points),canonical(points))
    monkeypatch.setattr(RouteSnapshot,'payload',lambda _:pytest.fail('full route reparsed in window owner'))
    c.prepare(c.transport.pending)
    support=[value for kind,value in out if kind=='support'][-1]
    expected=canonical(dict(map='a'*64,anchor=c.proposal.version.anchor_id,points=points))
    assert support.support_hash==hashlib.sha256(expected.encode()).hexdigest()
    assert len(support.support_ground_xyz)==2


def test_cancel_during_async_support_retirement_cannot_publish_old_proposal():
    from threading import Event
    from d1max_pct_scan.bounded_preparation import LatestPreparation
    c,clock,out=harness();started,release,done=Event(),Event(),Event()
    def prepare(request):
        started.set();assert release.wait(1.)
        result=c.build_support(request);done.set();return result
    c.preparation=LatestPreparation(prepare)
    try:
        c.on_route(envelope());assert started.wait(1.)
        c.on_route(envelope(sequence=2,active=False))
        release.set();assert done.wait(1.)
        c.tick()
        assert len([v for k,v in out if k=='proposal'])==1
        assert [v for k,v in out if k=='cancel_reference']
        assert not [v for k,v in out if k=='support']
        assert c.transport.route is None
    finally:release.set();assert c.close()


def test_real_callbacks_prepare_once_receipt_is_not_commit_no_duplicate_on_correction():
    c,clock,out=harness();c.on_route(envelope())
    assert c.proposal.reference.path.header.frame_id=='d1max_loc_odom'
    assert c.proposal.reference.point_reference=='body_center'
    assert c.transport.accepted is None
    p=admit(c)
    accepted=deepcopy(c.transport.accepted)
    clock.ns+=100_000_000;clock.mono+=.1
    c.on_navigation(state(t=.1,x=.05,correction=.01))
    assert c.transport.accepted==accepted and c.transport.core.candidate_anchor
    assert len([x for x in out if x[0]=='proposal'])==1
    assert not p.allowed


def test_wrong_or_uncommitted_permit_cannot_admit_candidate():
    c,clock,out=harness();c.on_route(envelope())
    p=c.proposal
    permit=ExecutionPermit(version=p.version,source_stamp=p.source_stamp,valid_until=p.valid_until,
        geometry_committed=False,sequence=1,transport_mode='isolated_mock')
    assert not c.on_permit(permit)
    permit.geometry_committed=True
    assert not c.on_permit(permit) # no receipt
    assert c.transport.accepted is None


def test_route_progress_only_reports_accepted_core_original_body_source():
    c,clock,out=harness();c.on_route(envelope())
    assert not [v for k,v in out if k=='route_progress']
    p=admit(c)
    reports=[v for k,v in out if k=='route_progress']
    assert len(reports)==1
    r=reports[-1]
    assert r.schema_version==1 and r.version==p.version
    assert r.body_source_stamp==r.odom_body_pose.header.stamp==state().source_stamp
    assert r.odom_body_pose.pose==state().local_odometry.pose.pose
    assert r.anchor_source_stamp==state().source_stamp
    assert r.measured_arc_m==r.confirmed_arc_m==pytest.approx(0.)
    assert r.source_map_body_xyz.z==pytest.approx(.55)
    assert not c.publish_route_progress()
    clock.ns+=100_000_000;clock.mono+=.1
    c.on_navigation(state(t=.1,x=.05,correction=.1))
    latest=[v for k,v in out if k=='route_progress'][-1]
    assert latest.version==p.version  # correction is staged, not the incumbent anchor
    assert latest.source_map_body_xyz.x==latest.measured_arc_m==pytest.approx(.05)
    assert latest.map_from_odom==r.map_from_odom
    assert latest.body_source_stamp==state(t=.1).source_stamp
    assert not c.publish_route_progress()


def test_local_only_progress_survives_global_gap_and_has_no_restamped_source():
    from d1max_planning_interfaces.msg import LocalNavigationState
    c,clock,out=harness();c.p['local_state_enabled']=True;c.on_route(envelope());admit(c)
    global_source=deepcopy(c.state.source_ns)
    for i in range(1,11):
        clock.ns=BASE_NS+i*100_000_000;clock.mono=100.+i*.1
        pair=state(t=i*.1,x=i*.03)
        m=LocalNavigationState(schema_version=2,session_id='session',map_version_id='map',
            localization_epoch=1,localization_seed_id='seed',usable=True,
            local_odometry=pair.local_odometry,source_stamp=pair.source_stamp,
            posterior_stamp=pair.posterior_stamp,imu_stamp=pair.imu_stamp,
            valid_until=stamp_time((BASE_NS+i*100_000_000+100_000_000)*1e-9))
        assert c.on_local_navigation(m)
    r=[v for k,v in out if k=='route_progress'][-1]
    assert c.state.source_ns==global_source
    assert r.measured_arc_m==pytest.approx(.3)
    assert r.body_source_stamp==m.source_stamp and r.odom_body_pose.pose==m.local_odometry.pose.pose
    before=len([v for k,v in out if k=='route_progress'])
    assert not c.on_local_navigation(m)
    assert len([v for k,v in out if k=='route_progress'])==before


def local_state_from_pair(pair, *, usable=True, reason=''):
    from d1max_planning_interfaces.msg import LocalNavigationState
    return LocalNavigationState(schema_version=2,session_id=pair.session_id,
        map_version_id=pair.map_version_id,localization_epoch=pair.localization_epoch,
        localization_seed_id=pair.localization_seed_id,usable=usable,reason=reason,
        local_odometry=deepcopy(pair.local_odometry),source_stamp=deepcopy(pair.source_stamp),
        posterior_stamp=deepcopy(pair.posterior_stamp),imu_stamp=deepcopy(pair.imu_stamp),
        extrapolation_sec=pair.extrapolation_sec,
        valid_until=stamp_time(pair.source_stamp.sec+pair.source_stamp.nanosec*1e-9+.1))


def test_global_pair_ahead_of_local_cannot_replace_literal_route_progress_source():
    c,clock,out=harness();c.p['local_state_enabled']=True;c.on_route(envelope());permit=admit(c)
    assert not [v for k,v in out if k=='route_progress']
    assert c.on_local_navigation(local_state_from_pair(state()))
    first=[v for k,v in out if k=='route_progress'][-1]
    for i in range(1,6):
        clock.ns=BASE_NS+i*100_000_000;clock.mono=100.+i*.1
        pair=state(t=i*.1,x=i*.03,correction=.1)
        count=len([v for k,v in out if k=='route_progress'])
        assert c.on_navigation(pair)
        assert c.transport.core.candidate_anchor  # Real pairs still stage the anchor.
        heartbeat=deepcopy(permit);heartbeat.sequence=i+1
        heartbeat.source_stamp=deepcopy(pair.source_stamp)
        set_stamp(heartbeat.valid_until,clock.ns+400_000_000)
        assert c.on_permit(heartbeat)
        assert not c.publish_route_progress()  # Permit/timer paths cannot bypass the source fence.
        assert len([v for k,v in out if k=='route_progress'])==count
        assert c.on_local_navigation(local_state_from_pair(pair))
        current=[v for k,v in out if k=='route_progress'][-1]
        assert current.body_source_stamp==pair.source_stamp
        assert current.odom_body_pose.pose==pair.local_odometry.pose.pose
        assert current.map_from_odom==first.map_from_odom
        assert current.measured_arc_m==pytest.approx(i*.03)
        assert len([v for k,v in out if k=='route_progress'])==count+1
        assert not c.publish_route_progress()  # Exact duplicates never renew the observation.


def test_global_projection_same_source_wrong_local_pose_cannot_publish_progress():
    c,clock,out=harness();c.p['local_state_enabled']=True;c.on_route(envelope());admit(c)
    assert c.on_local_navigation(local_state_from_pair(state()))
    clock.ns+=100_000_000;clock.mono+=.1
    pair=state(t=.1,x=.03)
    assert c.on_navigation(pair)
    wrong_pose=local_state_from_pair(state(t=.1,x=.031))
    assert c.on_local_navigation(wrong_pose)
    assert not c.publish_route_progress()
    reports=[v for k,v in out if k=='route_progress']
    assert len(reports)==1 and reports[0].body_source_stamp==state().source_stamp
    clock.ns+=20_000_000;clock.mono+=.02
    actual=local_state_from_pair(state(t=.12,x=.036))
    assert c.on_local_navigation(actual)
    reports=[v for k,v in out if k=='route_progress']
    assert len(reports)==2 and reports[-1].body_source_stamp==actual.source_stamp
    assert reports[-1].measured_arc_m==pytest.approx(.036)


def test_global_pair_cannot_bypass_explicit_unusable_local_progress_source():
    c,clock,out=harness();c.p['local_state_enabled']=True;c.on_route(envelope());admit(c)
    assert c.on_local_navigation(local_state_from_pair(state()))
    clock.ns+=100_000_000;clock.mono+=.1
    pair=state(t=.1,x=.03)
    assert not c.on_local_navigation(local_state_from_pair(pair,usable=False,reason='imu_age'))
    assert c.on_navigation(pair)
    assert not c.publish_route_progress()
    reports=[v for k,v in out if k=='route_progress']
    assert len(reports)==1 and reports[0].body_source_stamp==state().source_stamp


def test_pending_anchor_does_not_report_reprojected_candidate_as_task_motion():
    c,clock,out=harness();c.on_route(envelope());admit(c)
    old=deepcopy(c.transport.accepted)
    clock.ns+=100_000_000;clock.mono+=.1
    c.on_navigation(state(t=.1,correction=.1))
    c.transport.propose_window(current_source_ns=clock.ns,now_monotonic=clock.mono)
    assert c.transport.pending_core.anchor!=c.transport.core.anchor
    assert c.transport.pending_core._progress.confirmed_arc_m==pytest.approx(.1)
    r=[v for k,v in out if k=='route_progress'][-1]
    assert r.version.anchor_id==old.anchor_id and r.confirmed_arc_m==pytest.approx(0.)
    assert r.version.reference_generation==old.generation
    c.transport.quarantined=True
    assert not c.publish_route_progress()


def test_cancelled_route_cannot_refresh_old_progress():
    c,clock,out=harness();c.on_route(envelope());admit(c)
    count=len([v for k,v in out if k=='route_progress'])
    c.on_route(envelope(sequence=2,active=False))
    assert not c.publish_route_progress()
    assert len([v for k,v in out if k=='route_progress'])==count


def test_stale_one_sensor_does_not_allow_reference_even_fresh_map_heartbeat():
    c,clock,out=harness()
    c.map_report['sources'][1]['integrated_stamp_ns']-=600_000_000
    c.on_route(envelope())
    assert c.transport.route is not None and c.proposal is None
    assert c.error=='reference_requires_acknowledged_fresh_native_map'
    assert not [v for k,v in out if k=='proposal']
    c.map_report['sources'][1]['integrated_stamp_ns']=clock.ns
    c.tick()
    assert c.proposal is not None
    assert c.error==''
    assert c.transport.route.source_stamp==envelope().source_stamp


def test_current_native_pending_debug_is_reported_without_claiming_valid_trajectory():
    from d1max_planning_interfaces.msg import LocalPlanDebug
    c,clock,_=harness();c.on_route(envelope())
    proposal=c.proposal
    receipt=ReferenceReceipt(version=proposal.version,proposal_id=proposal.proposal_id,
        expected_version=proposal.expected_version,expected_trajectory_id=proposal.expected_trajectory_id,
        source_stamp=proposal.source_stamp,valid_until=proposal.valid_until,accepted=True,transport_mode='isolated_mock')
    assert c.on_receipt(receipt)
    debug=LocalPlanDebug(session_id='session',generation=c.proposal.version.reference_generation,
        valid=False,phase='waiting_observed_space')
    debug.header.frame_id='d1max_loc_odom';set_stamp(debug.header.stamp,clock.ns)
    assert c.on_debug(debug)
    status=c.tick()
    assert status['local_debug_fresh'] and status['local_debug_generation']==debug.generation
    assert status['native_pending_phase']=='waiting_observed_space'
    assert not status['spline_visual_valid'] and not status['active_reference']
    clock.ns+=450_000_000;clock.mono+=.45
    status=c.tick()
    assert not status['local_debug_fresh']
    assert status['native_pending_phase']=='waiting_observed_space'
    assert not status['spline_visual_valid'] and not status['active_reference']
    assert c.debug.header.stamp==debug.header.stamp # no fabricated heartbeat
    c.debug.generation+=1
    assert c.tick()['native_pending_phase']==''


def curve_debug(c,clock,*,generation=None,plan_id=None,valid=True,phase='accepted'):
    from d1max_planning_interfaces.msg import LocalPlanDebug
    reference=c.transport.accepted or c.transport.pending
    message=LocalPlanDebug(session_id='session',
        generation=reference.generation if generation is None else generation,
        plan_id=(c.permit.trajectory_id if c.permit else 1) if plan_id is None else plan_id,
        valid=valid,phase=phase,checked_map_source_stamp_ns=clock.ns,
        checked_body_source_stamp_ns=clock.ns,checked_map_revision=1,
        checked_context_sequence=c.context['sequence'])
    message.header.frame_id='d1max_loc_odom';set_stamp(message.header.stamp,clock.ns)
    return message


def curve_validation(c,clock,*,debug=None,valid=True,reason='official_inflated_geometry',sequence=None):
    from d1max_planning_interfaces.msg import TrajectoryValidation
    from d1max_pct_scan.continuous_reference_node import version_of
    debug=debug or curve_debug(c,clock)
    reference=c.transport.accepted or c.transport.pending
    if c.transport.pending and debug.generation==c.transport.pending.generation:
        reference=c.transport.pending
    message=TrajectoryValidation(version=version_of(reference),trajectory_id=debug.plan_id,
        sequence=sequence if sequence is not None else 1+max(
            (fact[0].sequence for fact in c.validation_facts.values()),default=0),
        proposal_id=f'{reference.task_id}.{reference.generation}',map_snapshot_revision=debug.checked_map_revision,
        valid=valid,reason=reason,whole_curve=True,frame_id='d1max_loc_odom',
        transport_mode='isolated_mock',collision_policy='official_inflated_geometry')
    for name in ('source_stamp','check_begin','check_end'):
        set_stamp(getattr(message,name),clock.ns)
    set_stamp(message.valid_until,0 if c.geometry_owner=='local_executor' else clock.ns+250_000_000)
    set_stamp(message.body_source_stamp,debug.checked_body_source_stamp_ns)
    for name in ('front_ray_source_stamp','rear_ray_source_stamp'):
        set_stamp(getattr(message,name),debug.checked_map_source_stamp_ns)
    return message


def test_failed_candidate_preserves_exact_current_proof_and_following_phase():
    from d1max_pct_scan.bt_follow_policy import classify_follow
    c,clock,_=harness();c.on_route(long_envelope());permit=admit(c)
    current=curve_debug(c,clock);assert c.on_debug(current)
    assert c.on_validation(curve_validation(c,clock,debug=current))
    c.transport.propose_window(current_source_ns=clock.ns,now_monotonic=clock.mono)
    candidate=native_received(c)
    clock.ns+=20_000_000;clock.mono+=.02
    failed=curve_debug(c,clock,generation=candidate.version.reference_generation,
        valid=False,phase='failed_optimization')
    assert c.on_debug(failed)
    status=c.tick()
    assert status['spline_visual_valid'] and status['local_debug_phase']=='accepted'
    assert status['local_debug_generation']==permit.version.reference_generation
    assert status['last_spline_id']==permit.trajectory_id
    assert status['native_pending_phase']=='failed_optimization'
    assert status['native_pending_debug_generation']==candidate.version.reference_generation
    assert classify_follow(pose_ready=True,map_ready=True,committed=True,
        subscriber_ready=True,same_reference=True,scan=status).phase=='following'
    # A subsequent current proof must preserve the candidate's own outcome.
    clock.ns+=20_000_000;clock.mono+=.02
    refreshed=curve_debug(c,clock)
    assert c.on_validation(curve_validation(c,clock,debug=refreshed))
    assert c.on_debug(refreshed)
    assert c.tick()['native_pending_phase']=='failed_optimization'
    assert c.debug_facts[(current.generation,current.plan_id)][0].checked_body_source_stamp_ns==clock.ns


def test_current_negative_proof_wins_and_delayed_positive_cannot_revive_it():
    c,clock,_=harness();c.on_route(envelope());admit(c)
    positive=curve_debug(c,clock);assert c.on_debug(positive)
    assert c.on_validation(curve_validation(c,clock,debug=positive))
    clock.ns+=20_000_000;clock.mono+=.02
    negative=curve_debug(c,clock,valid=False,phase='failed_current_validation')
    assert c.on_debug(negative)
    assert c.on_validation(curve_validation(c,clock,debug=negative,
        valid=False,reason='failed_current_validation'))
    assert not c.on_debug(positive)
    status=c.tick()
    assert not status['spline_visual_valid']
    assert status['local_debug_phase']=='failed_current_validation'
    assert status['last_spline_id']==negative.plan_id


@pytest.mark.parametrize('expired',['header','receipt'])
def test_candidate_reports_cannot_extend_current_debug_or_proof_delivery_freshness(expired):
    c,clock,_=harness();c.on_route(long_envelope());admit(c)
    current=curve_debug(c,clock)
    assert c.on_validation(curve_validation(c,clock,debug=current))
    assert c.on_debug(current)
    c.transport.propose_window(current_source_ns=clock.ns,now_monotonic=clock.mono)
    candidate=native_received(c)
    if expired=='header':clock.ns+=c.freshness_ns+1
    if expired=='receipt':clock.mono+=c.transport.freshness+.001
    c.on_map(dict(c.context,valid=True,source_stamp_ns=clock.ns,
        sources=[dict(sensor_id=i,integrated_stamp_ns=clock.ns) for i in (0,1)]))
    assert c.on_debug(curve_debug(c,clock,generation=candidate.version.reference_generation,
        valid=False,phase='failed_optimization'))
    status=c.tick()
    assert status['native_pending_debug_fresh']
    assert not status['spline_visual_valid']
    fact,received=c.debug_facts[(current.generation,current.plan_id)]
    assert fact==current and received==100.


def test_curve_identity_requires_owner_commit_and_uses_only_matching_plan_id():
    c,clock,_=harness();c.on_route(envelope());proposal=native_received(c)
    first=curve_debug(c,clock,plan_id=1);assert c.on_debug(first)
    assert c.on_validation(curve_validation(c,clock,debug=first))
    assert not c.tick()['spline_visual_valid'] # Native candidate is not owner selection.
    assert fresh_commit(c,clock,proposal)
    assert c.tick()['spline_visual_valid']
    clock.ns+=20_000_000;clock.mono+=.02
    assert c.on_debug(curve_debug(c,clock,plan_id=2))
    assert c.tick()['last_spline_id']==1 # Same generation can contain another curve.
    switched=deepcopy(c.permit);switched.trajectory_id=3;switched.sequence+=1
    assert c.on_permit(switched)
    assert not c.tick()['spline_visual_valid'] # A proof of curve 1 or 2 is not proof of 3.
    third=curve_debug(c,clock,plan_id=3)
    assert c.on_debug(third)
    assert c.on_validation(curve_validation(c,clock,debug=third))
    assert c.tick()['spline_visual_valid'] and c.tick()['last_spline_id']==3


def candidate_proof(c,clock,*,plan_id=1,valid=True,reason='official_inflated_geometry'):
    debug=curve_debug(c,clock,plan_id=plan_id)
    proof=curve_validation(c,clock,debug=debug,valid=valid,reason=reason)
    proof.support_reference_id=c.proposal.proposal_id
    proof.support_hash='d'*64
    proof.checked_to_time=proof.curve_duration=2.
    return proof


def test_initial_typed_candidate_readiness_is_not_committed_visual_or_following():
    from d1max_pct_scan.bt_follow_policy import classify_follow
    c,clock,_=harness();c.on_route(envelope());proposal=native_received(c)
    proof=candidate_proof(c,clock);assert c.on_validation(proof)
    status=c.tick()
    assert status['candidate_admission_ready'] and status['candidate_plan_id']==1
    assert status['candidate_phase']=='candidate_ready_waiting_owner_commit'
    assert status['candidate_validation_sequence']==proof.sequence
    assert status['candidate_validation_valid_until_ns']==BASE_NS+250_000_000
    assert status['last_spline_id']==-1 and not status['spline_visual_valid']
    assert not status['active_reference'] and not status['motion_enabled']
    assert c.transport.accepted is None and c.permit is None
    assert classify_follow(pose_ready=True,map_ready=True,committed=True,
        subscriber_ready=True,same_reference=True,scan=status).phase!='following'
    # Only a separate typed owner commit adopts the candidate. This test does
    # not pretend that a diagnostic status automatically causes that commit.
    assert fresh_commit(c,clock,proposal)
    status=c.tick()
    assert status['spline_visual_valid'] and status['last_spline_id']==1
    assert not status['candidate_admission_ready']


def test_initial_negative_then_positive_and_latest_candidate_negative_are_literal():
    c,clock,_=harness();c.on_route(envelope());native_received(c)
    negative=candidate_proof(c,clock,valid=False,reason='source_expired_during_check')
    assert c.on_validation(negative)
    assert c.tick()['candidate_phase']=='source_expired_during_check'
    clock.ns+=20_000_000;clock.mono+=.02
    positive=candidate_proof(c,clock);assert c.on_validation(positive)
    assert c.tick()['candidate_admission_ready']
    assert c.transport.accepted is None
    clock.ns+=20_000_000;clock.mono+=.02
    newer_negative=candidate_proof(c,clock,plan_id=2,valid=False,reason='occupied')
    assert c.on_validation(newer_negative)
    assert c.on_debug(curve_debug(c,clock,plan_id=2,valid=True))
    status=c.tick()
    assert not status['candidate_admission_ready'] and status['candidate_plan_id']==2
    assert status['candidate_phase']=='occupied' and not status['spline_visual_valid']
    previous=c.validation_facts[(execution_version_identity(positive.version),1)]
    assert c.on_validation(positive) # Idempotent duplicate does not refresh its receipt.
    assert c.validation_facts[(execution_version_identity(positive.version),1)]==previous
    assert not c.tick()['candidate_admission_ready']


def candidate_policy_fixture():
    c,clock,_=harness();c.on_route(envelope());native_received(c)
    return dict(reference=deepcopy(c.transport.pending),proposal=deepcopy(c.proposal),
        receipt=deepcopy(c.receipt),proof=candidate_proof(c,clock),proof_received=clock.mono,
        context_sequence=c.context['sequence'],quarantined=False,now_ns=clock.ns,
        now_monotonic=clock.mono,freshness_s=c.transport.freshness,
        transport_mode=c.p['transport_mode'])


@pytest.mark.parametrize('fence',['wrong_version','wrong_proposal','negative_receipt',
    'wrong_predecessor','receipt_after_original_deadline','future_receipt','context','quarantine',
    'proof_expired','proof_receipt_expired','proof_future','proof_source_mismatch',
    'proof_floor_before_reference','suffix','missing_support','invalid_domain'])
def test_candidate_diagnostic_requires_original_receipt_and_fresh_exact_whole_proof(fence):
    facts=candidate_policy_fixture();assert candidate_preparation_status(**facts).valid
    if fence=='wrong_version':facts['proof'].version.map_geometry_revision+=1
    elif fence=='wrong_proposal':facts['proof'].proposal_id+='other'
    elif fence=='negative_receipt':facts['receipt'].accepted=False
    elif fence=='wrong_predecessor':facts['receipt'].expected_trajectory_id=99
    elif fence=='receipt_after_original_deadline':
        set_stamp(facts['receipt'].source_stamp,nanoseconds(facts['proposal'].valid_until)+1)
        facts['now_ns']+=500_000_000
    elif fence=='future_receipt':set_stamp(facts['receipt'].source_stamp,facts['now_ns']+1)
    elif fence=='context':facts['context_sequence']+=1
    elif fence=='quarantine':facts['quarantined']=True
    elif fence=='proof_expired':facts['now_ns']+=250_000_000
    elif fence=='proof_receipt_expired':facts['now_monotonic']+=facts['freshness_s']+.001
    elif fence=='proof_future':
        set_stamp(facts['proof'].check_end,facts['now_ns']+1)
        facts['proof'].source_stamp=deepcopy(facts['proof'].check_end)
    elif fence=='proof_source_mismatch':set_stamp(facts['proof'].source_stamp,facts['now_ns']-1)
    elif fence=='proof_floor_before_reference':set_stamp(facts['reference'].path.header.stamp,facts['now_ns']+1)
    elif fence=='suffix':facts['proof'].whole_curve=False;facts['proof'].remaining_curve=True
    elif fence=='missing_support':facts['proof'].support_hash=''
    else:facts['proof'].valid_start_time=3.
    assert not candidate_preparation_status(**facts).valid


def test_candidate_receipt_is_durable_but_native_proof_deadline_is_not_renewed():
    c,clock,_=harness();c.on_route(envelope());native_received(c)
    receipt=deepcopy(c.receipt);proposal=deepcopy(c.proposal)
    clock.ns+=450_000_000;clock.mono+=.45
    proof=candidate_proof(c,clock);assert c.on_validation(proof)
    status=c.tick()
    assert status['candidate_admission_ready'] # New native check, not a renewed delivery.
    assert c.receipt==receipt and c.proposal==proposal and not status['spline_visual_valid']
    clock.ns+=250_000_000;clock.mono+=.25
    assert not c.tick()['candidate_admission_ready']
    assert c.validation_facts[(execution_version_identity(proof.version),1)][0]==proof


def test_duplicate_debug_delivery_does_not_renew_monotonic_proof_age():
    c,clock,_=harness();c.on_route(envelope());admit(c)
    proof=curve_debug(c,clock);assert c.on_debug(proof)
    assert c.on_validation(curve_validation(c,clock,debug=proof))
    clock.mono+=c.transport.freshness+.001
    assert c.on_debug(proof)
    c.on_map(dict(c.context,valid=True,source_stamp_ns=clock.ns,
        sources=[dict(sensor_id=i,integrated_stamp_ns=clock.ns) for i in (0,1)]))
    assert not c.tick()['spline_visual_valid']
    assert c.debug_facts[(proof.generation,proof.plan_id)][1]==100.


def test_repeated_candidate_attempts_keep_debug_storage_bounded_without_evicting_current():
    c,clock,_=harness();c.on_route(long_envelope());permit=admit(c)
    proof=curve_debug(c,clock);assert c.on_debug(proof)
    assert c.on_validation(curve_validation(c,clock,debug=proof))
    c.transport.propose_window(current_source_ns=clock.ns,now_monotonic=clock.mono)
    candidate=native_received(c)
    for plan_id in range(2,34):
        clock.ns+=1;clock.mono+=1e-9
        assert c.on_debug(curve_debug(c,clock,generation=candidate.version.reference_generation,
            plan_id=plan_id,valid=False,phase='failed_optimization'))
        assert c.on_validation(curve_validation(c,clock,debug=c.debug,valid=False,reason='curve_check_budget_exhausted'))
    assert len(c.debug_facts)==c.prepared_limit
    assert len(c.validation_facts)==c.prepared_limit
    assert c.debug_facts[(proof.generation,proof.plan_id)][0]==proof
    assert c.tick()['spline_visual_valid'] and c.tick()['last_spline_id']==permit.trajectory_id


@pytest.mark.parametrize('change',['cancel','epoch','local_identity','execution_binding'])
def test_native_debug_facts_are_cleared_on_identity_retirement(change):
    c,clock,_=harness();c.on_route(envelope());admit(c)
    assert c.on_debug(curve_debug(c,clock))
    assert c.on_validation(curve_validation(c,clock,debug=c.debug))
    assert c.tick()['spline_visual_valid']
    if change=='cancel':c.on_route(envelope(sequence=2,active=False))
    elif change=='epoch':
        clock.ns+=20_000_000;clock.mono+=.02
        c.on_navigation(state(t=.02,epoch=2))
    elif change=='local_identity':
        c.p['local_state_enabled']=True
        assert c.on_local_navigation(local_state_from_pair(state()))
        clock.ns+=20_000_000;clock.mono+=.02
        c.on_local_navigation(local_state_from_pair(state(t=.02,epoch=2)))
    else:
        changed=deepcopy(c.permit);changed.sequence+=1;changed.sdk_arm_generation+=1
        assert c.on_permit(changed)
    assert not c.debug_facts and not c.validation_facts and not c.tick()['spline_visual_valid']


def test_positive_debug_without_native_proof_waits_for_exact_validation():
    c,clock,_=harness();c.on_route(envelope());admit(c)
    debug=curve_debug(c,clock);assert c.on_debug(debug)
    status=c.tick()
    assert not status['spline_visual_valid'] and status['local_debug_phase']=='waiting_current_validation'
    assert c.on_validation(curve_validation(c,clock,debug=debug))
    assert c.tick()['spline_visual_valid']


def test_debug_heartbeat_cannot_extend_original_native_proof_deadline():
    c,clock,_=harness();c.on_route(envelope());admit(c)
    debug=curve_debug(c,clock);assert c.on_debug(debug)
    proof=curve_validation(c,clock,debug=debug)
    # Native captures check_end and computes valid_until with separate clock
    # reads. Preserve its exact deadline rather than reconstructing 250 ms.
    set_stamp(proof.valid_until,clock.ns+250_001_000)
    assert c.on_validation(proof)
    clock.ns+=249_000_000;clock.mono+=.249
    heartbeat=deepcopy(debug);set_stamp(heartbeat.header.stamp,clock.ns)
    assert c.on_debug(heartbeat)
    assert c.tick()['spline_visual_valid']
    clock.ns+=1_001_000;clock.mono+=.001001
    status=c.tick()
    assert status['local_debug_fresh'] # Header and all source timestamps are still fresh.
    assert not status['spline_visual_valid'] and status['local_debug_phase']=='waiting_current_validation'
    assert status['local_validation_valid_until_ns']==BASE_NS+250_001_000
    assert c.validation_facts[c.committed_validation_key()][0]==proof


def test_native_negative_fences_newer_debug_header_and_old_positive_validation():
    c,clock,_=harness();c.on_route(envelope());admit(c)
    debug=curve_debug(c,clock);assert c.on_debug(debug)
    positive=curve_validation(c,clock,debug=debug);assert c.on_validation(positive)
    clock.ns+=20_000_000;clock.mono+=.02
    negative=curve_validation(c,clock,valid=False,reason='occupied')
    assert c.on_validation(negative)
    heartbeat=deepcopy(debug);set_stamp(heartbeat.header.stamp,clock.ns)
    assert c.on_debug(heartbeat)
    assert not c.on_validation(positive)
    newer_sequence_old_source=deepcopy(positive);newer_sequence_old_source.sequence=negative.sequence+1
    assert not c.on_validation(newer_sequence_old_source)
    status=c.tick()
    assert not status['spline_visual_valid'] and status['local_debug_phase']=='occupied'
    assert status['local_validation_sequence']==negative.sequence
    # Only another genuine native check recovers this curve; debug is diagnostic.
    clock.ns+=20_000_000;clock.mono+=.02
    refreshed=curve_debug(c,clock)
    assert c.on_validation(curve_validation(c,clock,debug=refreshed))
    assert c.on_debug(refreshed) and c.tick()['spline_visual_valid']


@pytest.mark.parametrize('diagnostic',['missing','stale_receipt','old_negative','other_context'])
def test_exact_committed_positive_proof_does_not_wait_for_diagnostic(diagnostic):
    from d1max_pct_scan.bt_follow_policy import classify_follow
    c,clock,_=harness();c.on_route(envelope());permit=admit(c)
    if diagnostic!='missing':
        debug=curve_debug(c,clock,valid=diagnostic!='old_negative',
            phase='source_expired_during_check' if diagnostic=='old_negative' else 'accepted')
        if diagnostic=='other_context':debug.checked_context_sequence=0
        assert c.on_debug(debug)
        if diagnostic=='stale_receipt':
            key=debug.generation,debug.plan_id
            c.debug_facts[key]=(debug,clock.mono-c.transport.freshness-.001)
    proof=curve_validation(c,clock)
    assert c.on_validation(proof)
    status=c.tick()
    assert status['spline_visual_valid'] and status['local_debug_phase']=='accepted'
    assert status['last_spline_id']==permit.trajectory_id
    assert status['local_validation_sequence']==proof.sequence
    assert status['local_validation_valid_until_ns']==BASE_NS+250_000_000
    assert not status['motion_enabled'] and not c.permit.allowed
    if diagnostic=='missing':assert not status['local_debug_fresh']
    if diagnostic=='old_negative':
        assert status['native_debug_phase']=='source_expired_during_check'
        assert not status['native_debug_valid']
    assert classify_follow(pose_ready=True,map_ready=True,committed=True,
        subscriber_ready=True,same_reference=True,scan=status).phase=='following'


def test_same_curve_new_positive_recovers_earlier_negative_before_debug_heartbeat():
    c,clock,_=harness();c.on_route(envelope());admit(c)
    debug=curve_debug(c,clock,valid=False,phase='source_expired_during_check')
    assert c.on_debug(debug)
    negative=curve_validation(c,clock,debug=debug,valid=False,reason='source_expired_during_check')
    assert c.on_validation(negative)
    assert not c.tick()['spline_visual_valid']
    clock.ns+=20_000_000;clock.mono+=.02
    proof=curve_validation(c,clock)
    assert c.on_validation(proof)
    status=c.tick()
    assert status['spline_visual_valid'] and status['local_debug_phase']=='accepted'
    assert status['native_debug_phase']=='source_expired_during_check'
    assert c.debug_facts[(debug.generation,debug.plan_id)][0]==debug
    assert c.validation_facts[c.committed_validation_key()][0]==proof


def committed_policy_fixture():
    c,clock,_=harness();c.on_route(envelope());admit(c)
    return dict(reference=deepcopy(c.transport.accepted),permit=deepcopy(c.permit),
        proof=curve_validation(c,clock),proof_received=clock.mono,
        context_sequence=c.context['sequence'],quarantined=False,now_ns=clock.ns,
        now_monotonic=clock.mono,freshness_s=c.transport.freshness,
        transport_mode=c.p['transport_mode'])


@pytest.mark.parametrize('identity',['task_id','route_hash','map_version_id','localization_epoch',
    'localization_seed_id','reference_generation','anchor_id','anchor_revision',
    'context_sequence','map_geometry_revision','trajectory_id','frame_id','transport_mode'])
def test_pure_committed_policy_requires_exact_full_proof_identity(identity):
    facts=committed_policy_fixture();assert committed_trajectory_status(**facts).valid
    proof=facts['proof']
    target=proof if identity in ('trajectory_id','frame_id','transport_mode') else proof.version
    before=getattr(target,identity)
    setattr(target,identity,before+1 if isinstance(before,int) else before+'.other')
    result=committed_trajectory_status(**facts)
    assert not result.valid and result.phase=='waiting_current_validation'
    assert result.plan_id==facts['permit'].trajectory_id


@pytest.mark.parametrize('fence',['revoked','not_committed','permit_expired','permit_future',
    'proof_floor','quarantined','current_context','permit_version','permit_transport'])
def test_pure_committed_policy_preserves_permission_identity_and_retirement_fences(fence):
    facts=committed_policy_fixture();assert committed_trajectory_status(**facts).valid
    if fence=='revoked':facts['permit'].revoked=True
    elif fence=='not_committed':facts['permit'].geometry_committed=False
    elif fence=='permit_expired':set_stamp(facts['permit'].valid_until,facts['now_ns'])
    elif fence=='permit_future':set_stamp(facts['permit'].source_stamp,facts['now_ns']+1)
    elif fence=='proof_floor':facts['permit'].validation_sequence=facts['proof'].sequence+1
    elif fence=='quarantined':facts['quarantined']=True
    elif fence=='current_context':facts['context_sequence']+=1
    elif fence=='permit_version':facts['permit'].version.map_geometry_revision+=1
    else:facts['permit'].transport_mode='live'
    assert not committed_trajectory_status(**facts).valid


@pytest.mark.parametrize('expired',['original_deadline','receipt','future_end','source_mismatch','old_check'])
def test_pure_committed_policy_never_extends_proof_by_receipt_debug_or_clock(expired):
    facts=committed_policy_fixture();proof=facts['proof'];original=deepcopy(proof)
    if expired=='original_deadline':
        facts['now_ns']+=250_000_000;facts['now_monotonic']+=.25
    elif expired=='receipt':facts['now_monotonic']+=facts['freshness_s']+.000001
    elif expired=='future_end':
        set_stamp(proof.check_end,facts['now_ns']+1);proof.source_stamp=deepcopy(proof.check_end)
    elif expired=='source_mismatch':set_stamp(proof.source_stamp,facts['now_ns']-1)
    else:
        # Exact original Unix ns, no float restamp. A newer reference's minimum
        # source cannot be satisfied by an earlier native check.
        set_stamp(facts['reference'].path.header.stamp,facts['now_ns']+1)
    assert not committed_trajectory_status(**facts).valid
    if expired in ('original_deadline','receipt','old_check'):assert proof==original


def test_newer_candidate_proof_is_not_authority_for_current_committed_curve():
    c,clock,_=harness();c.on_route(envelope());admit(c)
    current=curve_validation(c,clock);assert c.on_validation(current)
    c.transport.propose_window(current_source_ns=clock.ns,now_monotonic=clock.mono)
    candidate=native_received(c)
    clock.ns+=20_000_000;clock.mono+=.02
    next_curve=curve_debug(c,clock,generation=candidate.version.reference_generation,plan_id=9)
    assert c.on_validation(curve_validation(c,clock,debug=next_curve))
    # A newer candidate never supplies or renews the incumbent's proof.
    status=c.tick();assert status['spline_visual_valid'] and status['last_spline_id']==1
    assert status['local_validation_sequence']==current.sequence
    clock.ns=BASE_NS+250_000_000;clock.mono=100.25
    status=c.tick();assert not status['spline_visual_valid']
    assert status['last_spline_id']==1 and status['local_validation_sequence']==current.sequence


@pytest.mark.parametrize('metadata',['body_source','front_source','rear_source'])
def test_sensor_metadata_does_not_override_original_native_proof_deadline(metadata):
    c,clock,_=harness();c.on_route(envelope());admit(c)
    debug=curve_debug(c,clock)
    proof=curve_validation(c,clock,debug=debug)
    field={'body_source':'body_source_stamp','front_source':'front_ray_source_stamp',
        'rear_source':'rear_ray_source_stamp'}[metadata]
    allowance=c.freshness_ns if metadata=='body_source' else c.map_evidence_age_ns
    set_stamp(getattr(proof,field),clock.ns-allowance-1)
    assert c.on_validation(proof)
    assert c.on_debug(debug)
    assert c.tick()['spline_visual_valid'] # Native's deadline is the source-age verdict.
    clock.ns+=250_000_000;clock.mono+=.25
    assert not c.tick()['spline_visual_valid']
    assert c.tick()['local_validation_valid_until_ns']==BASE_NS+250_000_000


def test_stale_map_relay_blocks_new_reference_input_without_revoking_issued_native_proof():
    c,clock,_=harness();c.on_route(long_envelope());admit(c)
    current=deepcopy(c.transport.accepted)
    debug=curve_debug(c,clock);assert c.on_debug(debug)
    assert c.on_validation(curve_validation(c,clock,debug=debug))
    c.map_received-=c.transport.freshness+.001
    assert not c.map_fresh()
    c.propose_window()
    assert c.error=='reference_requires_acknowledged_fresh_native_map'
    assert c.transport.accepted==current and c.transport.pending is None
    status=c.tick()
    assert not status['sensor_ready'] and status['spline_visual_valid']
    clock.ns+=250_000_000;clock.mono+=.25
    assert not c.tick()['spline_visual_valid']


def test_positive_renewal_topic_order_does_not_interrupt_current_curve():
    c,clock,_=harness();c.on_route(envelope());admit(c)
    debug=curve_debug(c,clock);assert c.on_debug(debug)
    assert c.on_validation(curve_validation(c,clock,debug=debug))
    clock.ns+=20_000_000;clock.mono+=.02
    assert c.on_validation(curve_validation(c,clock)) # Proof before diagnostic heartbeat.
    assert c.tick()['spline_visual_valid']
    clock.ns+=20_000_000;clock.mono+=.02
    assert c.on_debug(curve_debug(c,clock)) # Heartbeat before the next proof.
    assert c.tick()['spline_visual_valid']
    assert c.on_validation(curve_validation(c,clock,valid=False,reason='occupied'))
    assert not c.tick()['spline_visual_valid'] and c.tick()['local_debug_phase']=='occupied'


def test_candidate_negative_validation_does_not_revoke_current_curve_proof():
    c,clock,_=harness();c.on_route(long_envelope());admit(c)
    debug=curve_debug(c,clock);assert c.on_debug(debug)
    assert c.on_validation(curve_validation(c,clock,debug=debug))
    c.transport.propose_window(current_source_ns=clock.ns,now_monotonic=clock.mono)
    candidate=native_received(c)
    clock.ns+=20_000_000;clock.mono+=.02
    failed=curve_debug(c,clock,generation=candidate.version.reference_generation,valid=False,phase='failed_optimization')
    assert c.on_validation(curve_validation(c,clock,debug=failed,valid=False,reason='curve_check_budget_exhausted'))
    assert c.on_debug(failed)
    status=c.tick()
    assert status['spline_visual_valid'] and status['local_debug_phase']=='accepted'
    assert status['native_pending_phase']=='failed_optimization'


@pytest.mark.parametrize('foreign',['curve','version'])
def test_validation_requires_exact_committed_version_and_curve(foreign):
    c,clock,_=harness();c.on_route(envelope());admit(c)
    debug=curve_debug(c,clock);assert c.on_debug(debug)
    proof=curve_validation(c,clock,debug=debug)
    if foreign=='curve':proof.trajectory_id+=1
    else:proof.version.anchor_id='foreign-anchor'
    assert c.on_validation(proof)==(foreign=='curve')
    assert not c.tick()['spline_visual_valid']


def test_candidate_expiry_keeps_committed_reference_and_context_ack_is_exact():
    c,clock,out=harness();c.on_route(envelope());admit(c)
    old=deepcopy(c.transport.accepted)
    c.transport.propose_window(current_source_ns=clock.ns,now_monotonic=clock.mono)
    proposal=deepcopy(c.proposal)
    clock.ns+=500_000_000;clock.mono+=.5;c.tick()
    assert c.transport.accepted==old and c.proposal==proposal
    assert c.transport.pending is not None  # missing ACK is not a retirement fact
    assert not reject_delivery(c,clock,proposal)
    assert c.transport.pending is None and c.transport.accepted==old
    assert not c.on_context_ack(dict(c.context,epoch=2))


def test_cancel_same_source_stamp_new_sequence_retires_owner_without_motion():
    c,clock,out=harness();c.on_route(envelope());admit(c)
    c.on_route(envelope(sequence=2,active=False))
    assert c.transport.accepted is None
    assert c.permit is None
    assert [v for k,v in out if k=='cancel_reference'][-1].path.poses==[]


def test_soft_delivery_refresh_acknowledges_latest_integer_sequence_without_moving_curve():
    c,clock,out=harness();c.on_route(envelope());admit(c)
    old=deepcopy(c.transport.accepted)
    clock.ns+=1;clock.mono+=.000000001
    c.on_route(envelope(t=1e-9,sequence=2))
    status=c.tick()
    assert status['owner_reference_stamp_ns']==BASE_NS+1
    assert status['owner_delivery_sequence']==2
    assert status['owner_task_id']=='task' and status['owner_route_id']=='route'
    assert status['owner_route_hash']==old.route_hash and c.transport.accepted==old


def test_cancel_before_map_ack_cannot_resurrect_cached_active_route():
    c,clock,out=harness();c.context_ready=False
    c.on_route(envelope())
    assert c.proposal is None and c.transport.route is not None
    c.on_route(envelope(sequence=2,active=False))
    c.on_context_ack(c.context);c.tick()
    assert c.transport.route is None
    assert not [v for k,v in out if k=='proposal']
    assert c.on_route(envelope()) is None


def test_goal_yaw_uses_frozen_map_goal_and_fixed_anchor_not_window_tangent():
    import math
    from d1max_pct_scan.source_route_ros import from_message,to_message
    c,clock,out=harness()
    message=state(t=.01)
    message.global_odometry.pose.pose.orientation.z=math.sin(.3/2)
    message.global_odometry.pose.pose.orientation.w=math.cos(.3/2)
    clock.ns+=10_000_000;clock.mono+=.01;c.on_navigation(message)
    route=envelope(t=.01)
    snapshot=from_message(route.snapshot).with_goal(has_goal_yaw=True,goal_yaw=.8,goal_yaw_tolerance_rad=.12)
    route.snapshot=to_message(snapshot,session_id='session',task_id='task',route_id='route',
        epoch=1,seed_id='seed',stamp=route.source_stamp)
    route.route_hash=snapshot.route_hash
    c.on_route(route)
    assert c.proposal.has_goal_yaw
    assert c.proposal.goal_yaw==pytest.approx(.5)
    assert c.proposal.goal_yaw_tolerance_rad==pytest.approx(.12)


def correction(c,clock,*,t,offset):
    clock.ns=BASE_NS+int(t*1e9);clock.mono=100.+t
    c.on_map(dict(c.context,valid=True,source_stamp_ns=clock.ns,
        sources=[dict(sensor_id=i,integrated_stamp_ns=clock.ns) for i in (0,1)]))
    return c.on_navigation(state(t=t,correction=offset))


def test_accumulated_map_correction_prepares_new_anchor_then_atomic_owner_commit():
    c,clock,out=harness();c.on_route(envelope());old_permit=admit(c)
    old_core=c.transport.core;old=deepcopy(c.transport.accepted)
    assert correction(c,clock,t=.1,offset=.03)
    assert c.proposal is None
    assert correction(c,clock,t=.6,offset=.12)
    candidate=c.proposal
    assert candidate is not None and candidate.version.anchor_id!=old.anchor_id
    assert candidate.expected_version==old_permit.version
    assert candidate.expected_trajectory_id==old_permit.trajectory_id
    assert candidate.reference.route_hash==old.route_hash
    assert candidate.reference.task_id==old.task_id
    assert candidate.goal_position.x==pytest.approx(1.88)
    assert candidate.reference.path.poses[0].pose.position.z==pytest.approx(.55)
    assert c.transport.core is old_core and c.transport.accepted==old
    new_permit=admit(c,sequence=2)
    assert c.transport.core is not old_core
    assert c.transport.core.anchor.anchor_id==new_permit.version.anchor_id
    assert c.transport.core.snapshot.route_hash==old_core.snapshot.route_hash
    assert c.transport.core._progress.measured_arc_m==pytest.approx(.12)
    assert old_core.anchor.anchor_id==old.anchor_id # old curve was never moved


def test_rejected_reanchor_retains_incumbent_and_late_old_commit_cannot_overwrite_new():
    c,clock,out=harness();c.on_route(envelope());old_permit=admit(c)
    old_core=c.transport.core;old=deepcopy(c.transport.accepted)
    correction(c,clock,t=.6,offset=.12)
    rejected=deepcopy(c.proposal)
    negative=ReferenceReceipt(version=rejected.version,proposal_id=rejected.proposal_id,
        expected_version=rejected.expected_version,expected_trajectory_id=rejected.expected_trajectory_id,
        source_stamp=rejected.source_stamp,valid_until=rejected.valid_until,
        accepted=False,transport_mode='isolated_mock')
    assert not c.on_receipt(negative)
    assert c.transport.core is old_core and c.transport.accepted==old
    assert c.transport.pending_core is None
    correction(c,clock,t=1.2,offset=.13)
    assert c.proposal and c.proposal.version!=rejected.version
    current=admit(c,sequence=3)
    committed_core=c.transport.core;committed=deepcopy(c.transport.accepted)
    late=deepcopy(current);late.version=rejected.version;late.sequence=4
    assert not c.on_permit(late)
    assert c.transport.core is committed_core and c.transport.accepted==committed
    late.version=old_permit.version
    assert not c.on_permit(late)


def test_expired_or_unprojectable_reanchor_never_replaces_incumbent():
    c,clock,out=harness();c.on_route(envelope());admit(c)
    old_core=c.transport.core;old=deepcopy(c.transport.accepted)
    correction(c,clock,t=.6,offset=.12)
    assert c.transport.pending_core is not old_core
    proposal=deepcopy(c.proposal)
    clock.ns+=500_000_000;clock.mono+=.5;c.tick()
    assert c.proposal==proposal and c.transport.core is old_core
    assert c.transport.accepted==old
    assert not reject_delivery(c,clock,proposal)
    assert c.transport.pending_core is None
    correction(c,clock,t=1.2,offset=.8)
    assert c.proposal is None and c.transport.pending_core is None
    assert c.transport.core is old_core and c.transport.accepted==old
    assert c.error=='measured_body_outside_current_semantic_segment'


@pytest.mark.parametrize('committed_without_progress',[False,True])
def test_local_owner_recovers_new_anchor_when_committed_old_projection_is_missing(committed_without_progress):
    c,clock,out=harness(geometry_owner='local_executor',local_state_enabled=True)
    c.on_route(long_envelope())
    first=native_received(c)
    assert c.on_local_commit(local_geometry_commit(c,clock,first))
    old=deepcopy(c.transport.accepted);old_core=c.transport.core
    clock.ns=BASE_NS+1_000_000_000;clock.mono=101.
    pair=state(t=1.)
    pair.local_odometry.pose.pose.position.z=.95
    c.map_report=None  # Map status must not hold a body/anchor-derived window.
    # Old-anchor projection is unavailable; the independently acquired global
    # pair still provides a same-identity corrected anchor. No stale progress
    # is republished and the incumbent curve is not moved.
    assert c.on_navigation(pair) is False
    c.on_local_navigation(local_state_from_pair(pair))
    t=c.transport
    assert c.proposal is not None
    if committed_without_progress:
        # Reproduce the production outcome of an already-successful owner
        # commit whose immutable preparation can no longer project the body.
        t._install_committed_window(old,deepcopy(old_core),received_monotonic=100.,
            end_arc_m=t._accepted_end_arc_m,start_arc_m=t._accepted_start_arc_m,
            issue_arc_m=t._accepted_issue_arc_m,current_source_ns=clock.ns,
            now_monotonic=clock.mono,owner_committed=True)
        assert t.core._progress is None
    c.on_map(dict(c.context,valid=True,source_stamp_ns=clock.ns,
        sources=[dict(sensor_id=i,integrated_stamp_ns=clock.ns) for i in (0,1)]))
    c.tick()
    assert c.proposal is not None
    assert c.proposal.version.anchor_id!=old.anchor_id
    assert c.proposal.version.task_id==old.task_id
    assert c.proposal.version.route_hash==old.route_hash
    assert c.proposal.reference.path.poses[0].pose.position.z==pytest.approx(.95)
    assert t.accepted==old and t.core.anchor.anchor_id==old.anchor_id
    assert not [v for k,v in out if k in ('handoff','sdk','motion')]


def test_missing_projection_without_correcting_anchor_does_not_publish_new_reference():
    c,clock,out=harness(geometry_owner='local_executor',local_state_enabled=True)
    c.on_route(long_envelope());first=native_received(c)
    assert c.on_local_commit(local_geometry_commit(c,clock,first))
    old=deepcopy(c.transport.accepted)
    clock.ns=BASE_NS+1_000_000_000;clock.mono=101.
    pair=state(t=1.)
    pair.local_odometry.pose.pose.position.z=.95
    pair.global_odometry.pose.pose.position.z=.95
    assert c.on_navigation(pair) is False
    c.on_local_navigation(local_state_from_pair(pair))
    c.on_map(dict(c.context,valid=True,source_stamp_ns=clock.ns,
        sources=[dict(sensor_id=i,integrated_stamp_ns=clock.ns) for i in (0,1)]))
    c.tick()
    assert c.proposal is None and c.transport.accepted==old
    assert c.transport.core.candidate_anchor is None


def reset_debug(c,clock,generation):
    from d1max_planning_interfaces.msg import LocalPlanDebug
    message=LocalPlanDebug(session_id='session',generation=generation,phase='cancelled',valid=False)
    message.header.frame_id='d1max_loc_odom';set_stamp(message.header.stamp,clock.ns)
    return message


def delivery_for(task, *, sequence, active=True, t=0., epoch=1):
    from d1max_pct_scan.source_route_ros import from_message,to_message
    message=envelope(t=t,sequence=sequence,active=active)
    message.task_id=task;message.route_id=task+'-route'
    message.snapshot=to_message(from_message(message.snapshot),session_id='session',
        task_id=message.task_id,route_id=message.route_id,epoch=epoch,
        seed_id='seed',stamp=message.source_stamp)
    return message


def delivery_receipts(output):
    return [message for kind,message in output if kind=='route_receipt']


def test_empty_hard_reset_recovers_with_new_context_not_nonexistent_native_cancel():
    from d1max_planning_interfaces.msg import RouteDeliveryReceipt
    c,clock,out=harness();c.p['local_state_enabled']=True
    old_context=deepcopy(c.context)
    clock.ns+=100_000_000;clock.mono+=.1
    assert not c.on_local_navigation(local_state_from_pair(state(t=.1),usable=False,reason='lio_reset'))
    assert c.transport.quarantined and c.transport.empty_ownership
    assert not c.on_context_ack(old_context)
    clock.ns+=20_000_000;clock.mono+=.02
    c.on_navigation(state(t=.12))
    c.on_local_navigation(local_state_from_pair(state(t=.12)))
    assert c.transport.quarantined
    assert c.on_context_ack(c.context)
    assert not c.transport.quarantined
    c.on_map(dict(c.context,valid=True,source_stamp_ns=clock.ns,
        sources=[dict(sensor_id=i,integrated_stamp_ns=clock.ns) for i in (0,1)]))
    c.on_route(delivery_for('new',sequence=1,t=.12))
    assert delivery_receipts(out)[-1].status==RouteDeliveryReceipt.ACCEPTED
    assert c.proposal.version.task_id=='new' and c.permit is None
    assert not [m for k,m in out if k=='cancel_reference']


def test_rejected_unadmitted_route_cancel_is_terminal_and_fences_queued_active():
    from d1max_planning_interfaces.msg import RouteDeliveryReceipt
    c,clock,out=harness();c.transport.quarantined=True
    with pytest.raises(ValueError,match='quarantined'):
        c.on_route(delivery_for('never-native',sequence=1))
    assert delivery_receipts(out)[-1].status==RouteDeliveryReceipt.REJECTED
    c.on_route(delivery_for('never-native',sequence=2,active=False))
    receipt=delivery_receipts(out)[-1]
    assert receipt.status==RouteDeliveryReceipt.RETIRED
    assert not receipt.native_retirement_required and not receipt.native_retirement_confirmed
    assert receipt.native_generation==0 and c.pending_route_retirement is None
    c.transport.quarantined=False
    with pytest.raises(ValueError,match='retired_committed_route'):
        c.on_route(delivery_for('never-native',sequence=3))
    assert not [m for k,m in out if k=='proposal']


def test_early_proposal_cancel_waits_native_retirement_even_when_support_not_ready():
    from threading import Event
    from d1max_planning_interfaces.msg import RouteDeliveryReceipt
    from d1max_pct_scan.bounded_preparation import LatestPreparation
    c,clock,out=harness();started,release,done=Event(),Event(),Event()
    def prepare(request):
        started.set();assert release.wait(1.)
        result=c.build_support(request);done.set();return result
    c.preparation=LatestPreparation(prepare)
    try:
        c.on_route(delivery_for('worker',sequence=1));assert started.wait(1.)
        c.on_route(delivery_for('worker',sequence=2,active=False))
        receipt=delivery_receipts(out)[-1]
        assert receipt.status==RouteDeliveryReceipt.ACCEPTED
        assert c.native_route_exposures and c.pending_route_retirement['required']
        release.set();assert done.wait(1.);c.tick()
        assert len([m for k,m in out if k=='proposal'])==1
        assert [m for k,m in out if k=='cancel_reference']
        assert not [m for k,m in out if k=='support']
        c.on_debug(reset_debug(c,clock,c.cancel_generation))
        assert delivery_receipts(out)[-1].status==RouteDeliveryReceipt.RETIRED
        assert c.transport.route is None
    finally:release.set();assert c.close()


def test_native_exposure_survives_prepared_clear_and_requires_exact_cancel_barrier():
    from d1max_planning_interfaces.msg import RouteDeliveryReceipt
    c,clock,out=harness();c.on_route(delivery_for('native',sequence=1))
    identity=('native','native-route',envelope().route_hash)
    assert identity in c.native_route_exposures
    c.clear_prepared();c.on_route(delivery_for('native',sequence=2,active=False))
    assert c.pending_route_retirement['required']
    assert delivery_receipts(out)[-1].status==RouteDeliveryReceipt.ACCEPTED
    c.on_debug(reset_debug(c,clock,c.cancel_generation-1))
    assert c.pending_route_retirement is not None
    c.on_debug(reset_debug(c,clock,c.cancel_generation))
    receipt=delivery_receipts(out)[-1]
    assert receipt.status==RouteDeliveryReceipt.RETIRED
    assert receipt.native_retirement_required and receipt.native_retirement_confirmed
    assert receipt.native_generation==c.cancel_generation and identity not in c.native_route_exposures


def test_delayed_native_cancel_ack_is_software_fact_not_current_curve_proof():
    from d1max_planning_interfaces.msg import RouteDeliveryReceipt
    c,clock,out=harness();c.on_route(delivery_for('native',sequence=1));old_permit=admit(c)
    c.on_route(delivery_for('native',sequence=2,active=False))
    ack=reset_debug(c,clock,c.cancel_generation);original=deepcopy(ack)
    clock.ns+=600_000_000;clock.mono+=.6
    assert c.on_debug(ack)
    receipt=delivery_receipts(out)[-1]
    assert receipt.status==RouteDeliveryReceipt.RETIRED and receipt.native_retirement_confirmed
    assert receipt.native_generation==ack.generation and receipt.source_stamp==original.header.stamp
    assert c.pending_route_retirement is None and not c.native_route_exposures
    assert c.native_retirement['source_ns']==nanoseconds(original.header.stamp)
    assert c.debug is None and not c.debug_facts and not c.validation_facts
    assert c.permit is None and not c.on_permit(old_permit)
    assert c.transport.accepted is None and c.transport.route is None
    assert ack==original and not c.tick()['spline_visual_valid']


@pytest.mark.parametrize('bad',['session','frame','generation_old','generation_new','future',
    'noncanonical','negative','before_command','valid','accepted_phase'])
def test_delayed_native_cancel_ack_requires_exact_current_cancellation(bad):
    c,clock,out=harness();c.on_route(delivery_for('native',sequence=1))
    c.on_route(delivery_for('native',sequence=2,active=False))
    pending=deepcopy(c.pending_route_retirement)
    ack=reset_debug(c,clock,c.cancel_generation)
    clock.ns+=600_000_000;clock.mono+=.6
    if bad=='session':ack.session_id='foreign'
    elif bad=='frame':ack.header.frame_id='d1max_loc_map'
    elif bad=='generation_old':ack.generation-=1
    elif bad=='generation_new':ack.generation+=1
    elif bad=='future':set_stamp(ack.header.stamp,clock.ns+1)
    elif bad=='noncanonical':ack.header.stamp.nanosec=1_000_000_000
    elif bad=='negative':ack.header.stamp.sec=-1
    elif bad=='before_command':set_stamp(ack.header.stamp,nanoseconds(pending['command'].source_stamp)-1)
    elif bad=='valid':ack.valid=True
    else:ack.phase='accepted'
    assert not c.on_debug(ack)
    assert c.pending_route_retirement==pending and c.native_retirement is None
    assert c.debug is None and not c.debug_facts and not c.validation_facts
    assert len(delivery_receipts(out))==1 and c.permit is None


def retired_context_waiting_for_new_ack():
    """Actual reset/cancel callbacks, with the native ACK already consumed."""
    c,clock,out=harness();c.p['local_state_enabled']=True
    c.on_local_navigation(local_state_from_pair(state()))
    c.on_route(envelope());old_permit=admit(c)
    clock.ns+=100_000_000;clock.mono+=.1
    assert not c.on_navigation(state(t=.1,epoch=2))
    c.on_local_navigation(local_state_from_pair(state(t=.1,epoch=2)))
    c.on_route(envelope(t=.1,sequence=2,active=False))
    ack=reset_debug(c,clock,c.cancel_generation)
    assert c.on_debug(ack)
    assert c.pending_route_retirement is None and not c.native_route_exposures
    assert c.transport.quarantined and not c.context_ready
    saved=deepcopy(c.native_retirement)
    clock.ns+=3_100_000_000;clock.mono+=3.1
    fresh=state(t=3.2,epoch=2)
    c.on_navigation(fresh);c.on_local_navigation(local_state_from_pair(fresh))
    assert c.native_retirement==saved and c.transport.quarantined
    return c,clock,out,old_permit,ack


def test_native_retirement_survives_three_second_new_context_wait_without_resuming_old_task():
    c,clock,out,old_permit,old_ack=retired_context_waiting_for_new_ack()
    original_pair=deepcopy(c.transport._reset_pair)
    assert c.local_inbox.usable(now_ns=clock.ns,monotonic=clock.mono,
        receipt_timeout_s=c.transport.freshness)
    assert c.on_context_ack(deepcopy(c.context))
    assert not c.transport.quarantined and c.transport.empty_ownership
    assert c.transport.latest_pair==original_pair
    assert c.proposal is None and c.receipt is None and c.permit is None and c.native_retirement is None
    assert c.debug is None and not c.validation_facts
    assert not c.on_permit(old_permit)
    with pytest.raises(ValueError,match='retired_committed_route'):
        c.on_route(envelope(t=3.2,sequence=3))
    c.on_map(dict(c.context,valid=True,source_stamp_ns=clock.ns,
        sources=[dict(sensor_id=i,integrated_stamp_ns=clock.ns) for i in (0,1)]))
    c.on_route(delivery_for('new',sequence=3,t=3.2,epoch=2))
    proposal,owner=deepcopy(c.proposal),deepcopy(c.transport.route)
    assert proposal.version.task_id=='new' and proposal.expected_trajectory_id==-1
    assert not c.on_debug(old_ack)
    assert c.proposal==proposal and c.transport.route==owner and c.permit is None
    assert c.native_retirement is None and c.debug is None


@pytest.mark.parametrize('identity_change',['seed','epoch'])
@pytest.mark.parametrize('ack_order',['before_new_identity','after_new_identity'])
@pytest.mark.parametrize('state_order',['local_first','global_first'])
def test_old_identity_reset_retirement_survives_new_identity_delivery_order(
        identity_change,ack_order,state_order):
    """A completed cancellation belongs to its operation, not the old TF seed.

    Production publishes the hard notice under the OLD identity before a new
    verified global pair exists. Native can finish cancellation during that
    interval. Neither topic ordering nor the new context may erase that fact.
    """
    from d1max_pct_scan.source_route_ros import from_message,to_message
    c,clock,out=harness(geometry_owner='local_executor',local_state_enabled=True)
    c.on_local_navigation(local_state_from_pair(state()))
    c.on_route(envelope());proposal=native_received(c)
    old_commit=local_geometry_commit(c,clock,proposal)
    assert c.on_local_commit(old_commit)
    clock.ns+=100_000_000;clock.mono+=.1
    assert not c.on_local_navigation(local_state_from_pair(
        state(t=.1),usable=False,reason='localization_reset'))
    c.on_route(envelope(t=.1,sequence=2,active=False))
    cancelled=reset_debug(c,clock,c.cancel_generation)
    if ack_order=='before_new_identity':
        assert c.on_debug(cancelled)
        original=deepcopy(c.native_retirement)
        assert c.on_debug(cancelled)  # Duplicate ACK does not renew its receipt.
        assert c.native_retirement==original
    clock.ns+=600_000_000;clock.mono+=.6
    fresh=state(t=.7,epoch=2 if identity_change=='epoch' else 1)
    if identity_change=='seed':fresh.localization_seed_id='new-seed'
    callbacks=(lambda:c.on_local_navigation(local_state_from_pair(fresh)),
               lambda:c.on_navigation(fresh))
    for callback in callbacks if state_order=='local_first' else reversed(callbacks):
        callback()
    assert c.transport.quarantined and not c.context_ready
    assert c.on_context_ack(deepcopy(c.context))
    if ack_order=='after_new_identity':
        assert c.transport.quarantined  # New context cannot invent retirement.
        assert c.on_debug(cancelled)
    assert not c.transport.quarantined and c.transport.empty_ownership
    assert c.native_retirement is None and c.transport._retired_generation==0
    assert c.proposal is None and c.permit is None and c.local_commit is None
    assert not c.on_local_commit(old_commit) and not c.on_debug(cancelled)
    with pytest.raises(ValueError,match='retired_committed_route'):
        c.on_route(envelope(t=.7,sequence=3))

    # A new task is admitted, but the previous task's completed cancellation
    # cannot stand in for its own outstanding Native operation.
    new=delivery_for('new',sequence=3,t=.7,epoch=fresh.localization_epoch)
    new.snapshot=to_message(from_message(new.snapshot),session_id='session',
        task_id=new.task_id,route_id=new.route_id,epoch=fresh.localization_epoch,
        seed_id=fresh.localization_seed_id,stamp=new.source_stamp)
    c.on_route(new)
    assert c.proposal.version.task_id=='new' and c.proposal.expected_trajectory_id==-1
    assert c.permit is None and not c.on_debug(cancelled)
    clock.ns+=20_000_000;clock.mono+=.02
    new_cancel=deepcopy(new);new_cancel.active=False;new_cancel.delivery_sequence=4
    set_stamp(new_cancel.source_stamp,clock.ns)
    new_cancel.snapshot.path.header.stamp=deepcopy(new_cancel.source_stamp)
    c.on_route(new_cancel)
    assert c.cancel_generation>cancelled.generation
    pending=deepcopy(c.pending_route_retirement)
    assert not c.on_debug(cancelled)
    wrong_session=reset_debug(c,clock,c.cancel_generation);wrong_session.session_id='other'
    assert not c.on_debug(wrong_session)
    before_command=reset_debug(c,clock,c.cancel_generation)
    set_stamp(before_command.header.stamp,clock.ns-1)
    assert not c.on_debug(before_command)
    assert c.pending_route_retirement==pending
    assert c.on_debug(reset_debug(c,clock,c.cancel_generation))
    assert c.pending_route_retirement is None and not c.native_route_exposures
    assert not [message for kind,message in out if kind in ('handoff','sdk','motion')]


@pytest.mark.parametrize('missing',['context_ack','fresh_local','native_exposure','pending_retirement',
    'writer_outcome','active_slot','retired_generation','cancel_generation','retired_source','future_retirement'])
def test_historical_native_retirement_cannot_bypass_new_context_and_empty_owner_contract(missing):
    c,clock,out,old_permit,ack=retired_context_waiting_for_new_ack()
    if missing=='fresh_local':clock.ns+=100_000_001;clock.mono+=.100000001
    elif missing=='native_exposure':c.native_route_exposures.add(('foreign','route','f'*64))
    elif missing=='pending_retirement':c.pending_route_retirement=dict(required=True,generation=99)
    elif missing=='writer_outcome':
        # A literal unresolved writer claim must still block the empty-context
        # fallback, even after its native planner operation was retired.
        record=SimpleNamespace(handoffs={((('execution',1,'sdk',1,'isolated_mock')),'handoff',1):object()},
            acknowledgements={})
        c.prepared[99]=record
        assert c.writer_unresolved()
    elif missing=='active_slot':c.transport.pending=deepcopy(old_permit)
    elif missing=='retired_generation':c.native_retirement['generation']+=1
    elif missing=='cancel_generation':c.cancel_generation+=1
    elif missing=='retired_source':c.native_retirement['source_ns']=c.transport._retired_stamp_ns-1
    elif missing=='future_retirement':c.native_retirement['source_ns']=clock.ns+1
    if missing!='context_ack':c.on_context_ack(deepcopy(c.context))
    assert c.transport.quarantined and not c.try_recover_context()
    assert c.permit is None and not c.on_permit(old_permit)


def test_rejected_b_cancellation_during_a_native_retirement_cannot_deadlock_or_clear_a():
    from d1max_planning_interfaces.msg import RouteDeliveryReceipt
    c,clock,out=harness();c.on_route(delivery_for('a',sequence=1))
    c.on_route(delivery_for('a',sequence=2,active=False))
    pending=deepcopy(c.pending_route_retirement)
    c.on_route(delivery_for('b',sequence=3))
    assert delivery_receipts(out)[-1].status==RouteDeliveryReceipt.REJECTED
    c.on_route(delivery_for('b',sequence=4,active=False))
    receipt=delivery_receipts(out)[-1]
    assert receipt.task_id=='b' and receipt.status==RouteDeliveryReceipt.RETIRED
    assert not receipt.native_retirement_required and c.pending_route_retirement==pending
    c.on_debug(reset_debug(c,clock,pending['generation']))
    receipt=delivery_receipts(out)[-1]
    assert receipt.task_id=='a' and receipt.status==RouteDeliveryReceipt.RETIRED
    assert c.pending_route_retirement is None and not c.native_route_exposures
    with pytest.raises(ValueError,match='retired_committed_route'):
        c.on_route(delivery_for('b',sequence=5))


def test_cached_delivery_reaffirms_manager_fact_without_restamping_sensor_or_geometry():
    c,clock,out=harness();command=delivery_for('a',sequence=1);c.on_route(command)
    first=delivery_receipts(out)[-1];proposal=deepcopy(c.proposal)
    clock.ns+=600_000_000;clock.mono+=.6;c.on_route(command)
    current=delivery_receipts(out)[-1]
    assert nanoseconds(current.receipt_stamp)==clock.ns
    assert current.source_stamp==first.source_stamp==command.source_stamp
    assert current.delivery_sequence==first.delivery_sequence
    assert c.proposal==proposal and len([m for k,m in out if k=='proposal'])==1


def test_unadmitted_foreign_cancel_cannot_erase_current_native_route():
    from d1max_planning_interfaces.msg import RouteDeliveryReceipt
    c,clock,out=harness();c.on_route(delivery_for('a',sequence=1));proposal=deepcopy(c.proposal)
    c.on_route(delivery_for('b',sequence=2,active=False))
    assert delivery_receipts(out)[-1].status==RouteDeliveryReceipt.RETIRED
    assert c.proposal==proposal and c.transport.ingress.active_identity[0]=='a'
    assert c.pending_route_retirement is None and len(c.native_route_exposures)==1


def test_cached_delivery_cannot_acknowledge_a_different_map_version():
    c,clock,out=harness();command=delivery_for('a',sequence=1);c.on_route(command)
    before=len(delivery_receipts(out));command.snapshot.map_version_id='other-map'
    with pytest.raises(ValueError,match='receipt_identity_invalid|snapshot_hash_mismatch'):
        c.on_route(command)
    assert len(delivery_receipts(out))==before


def test_hard_reset_requires_old_retirement_and_new_context_ack_then_new_task_only():
    from d1max_pct_scan.source_route_ros import from_message,to_message
    c,clock,out=harness();c.on_route(envelope());old_permit=admit(c)
    old_context=deepcopy(c.context)
    clock.ns+=100_000_000;clock.mono+=.1
    assert not c.on_navigation(state(t=.1,epoch=2))
    assert c.transport.quarantined and c.permit is None and c.transport.core._progress is None
    assert not c.on_permit(old_permit)
    assert not c.on_context_ack(old_context)
    assert c.on_context_ack(c.context)
    assert c.transport.quarantined # ACK alone cannot retire old task
    c.on_route(envelope(t=.1,sequence=2,active=False))
    c.on_debug(reset_debug(c,clock,c.cancel_generation-1))
    assert c.transport.quarantined # wrong generation cannot recover
    c.on_debug(reset_debug(c,clock,c.cancel_generation))
    assert not c.transport.quarantined and c.transport.core is None and c.transport.route is None
    assert c.permit is None and c.receipt is None and c.transport.accepted is None
    assert not c.on_permit(old_permit)
    with pytest.raises(ValueError,match='retired_committed_route'):
        c.on_route(envelope(t=.1,sequence=3))
    new=envelope(t=.1,sequence=3)
    new.task_id,new.route_id='new-task','new-route'
    new.snapshot=to_message(from_message(new.snapshot),session_id='session',task_id=new.task_id,
        route_id=new.route_id,epoch=2,seed_id='seed',stamp=new.source_stamp)
    c.on_map(dict(c.context,valid=True,source_stamp_ns=clock.ns,
        sources=[dict(sensor_id=i,integrated_stamp_ns=clock.ns) for i in (0,1)]))
    c.on_route(new)
    assert c.proposal.version.task_id=='new-task' and c.proposal.version.localization_epoch==2
    assert c.permit is None # a new owner confirmation is still required
    assert c.proposal.expected_trajectory_id==-1


@pytest.mark.parametrize('missing',['context_ack','fresh_body','native_ack'])
def test_hard_reset_cannot_recover_with_partial_or_stale_evidence(missing):
    c,clock,out=harness();c.on_route(envelope());admit(c)
    clock.ns+=100_000_000;clock.mono+=.1;c.on_navigation(state(t=.1,epoch=2))
    c.on_route(envelope(t=.1,sequence=2,active=False))
    if missing!='context_ack':c.on_context_ack(c.context)
    if missing=='fresh_body':clock.ns+=500_000_000;clock.mono+=.5
    if missing!='native_ack':c.on_debug(reset_debug(c,clock,c.cancel_generation))
    assert c.transport.quarantined and c.transport.core is None
    assert not c.try_recover_context()


def native_received(c):
    p=c.proposal
    ack=ReferenceReceipt(version=p.version,proposal_id=p.proposal_id,expected_version=p.expected_version,
        expected_trajectory_id=p.expected_trajectory_id,source_stamp=p.source_stamp,valid_until=p.valid_until,
        accepted=True,transport_mode='isolated_mock')
    assert c.on_receipt(ack)
    return deepcopy(p)


def fresh_commit(c,clock,proposal,sequence=1):
    p=ExecutionPermit(version=proposal.version,phase='preview',geometry_committed=True,allowed=False,
        sequence=sequence,trajectory_id=1,transport_mode='isolated_mock')
    set_stamp(p.source_stamp,clock.ns);set_stamp(p.valid_until,clock.ns+250_000_000)
    return c.on_permit(p)


def test_native_receipt_geometry_survives_delivery_deadline_until_fresh_owner_commit():
    # graph06: native accepted generation1, owner commit arrived 477ms after
    # proposal source. A 400ms delivery TTL must not discard already received
    # immutable geometry and start a generation flood.
    c,clock,out=harness();c.on_route(envelope());proposal=native_received(c)
    for i in range(1,31):
        correction(c,clock,t=.05*i,offset=0.)
        c.tick()
    assert c.transport.generation==1 and c.transport.pending is not None
    assert c.transport.accepted is None
    assert len([v for k,v in out if k=='proposal'])==1
    assert not [v for k,v in out if k=='cancel_reference']
    assert fresh_commit(c,clock,proposal)
    assert c.transport.accepted.generation==1
    assert c.transport.accepted.path.header.stamp==proposal.source_stamp # not renewed
    assert c.permit is not None and not c.permit.allowed


def test_received_geometry_is_not_permission_when_current_pose_is_stale():
    c,clock,out=harness();c.on_route(envelope());proposal=native_received(c)
    clock.ns+=600_000_000;clock.mono+=.6;c.tick()
    assert fresh_commit(c,clock,proposal)
    assert c.transport.accepted==proposal.reference and c.transport.pending is None
    assert c.transport.core._progress is None and not c.publish_route_progress()
    assert not c.tick()['spline_visual_valid']
    assert not [v for k,v in out if k=='route_progress']
    correction(c,clock,t=.65,offset=0.)
    assert c.transport.core._progress is not None
    assert [v for k,v in out if k=='route_progress'][-1].body_source_stamp==state(t=.65).source_stamp


def test_missing_receipt_retry_is_bounded_and_never_a_cancel():
    c,clock,out=harness();c.on_route(envelope())
    for i in range(1,61):
        correction(c,clock,t=.05*i,offset=0.);c.tick()
    assert c.transport.generation==1 and len(c.prepared)==1
    assert c.transport.accepted is None
    assert not [v for k,v in out if k=='cancel_reference']


def reject_delivery(c,clock,proposal):
    """A native refusal of the exact request, not a local timeout inference."""
    message=ReferenceReceipt(version=deepcopy(proposal.version),proposal_id=proposal.proposal_id,
        expected_version=deepcopy(proposal.expected_version),
        expected_trajectory_id=proposal.expected_trajectory_id,accepted=False,
        transport_mode='isolated_mock',reason='reference_deadline_expired_before_admission')
    set_stamp(message.source_stamp,clock.ns);set_stamp(message.valid_until,clock.ns+500_000_000)
    return c.on_receipt(message)


def test_missing_receipt_past_eight_retry_cycles_keeps_one_exact_request_and_support():
    # Before the fix, each 510 ms cycle created another delivered record; after
    # 8 cycles the production prepared ledger was full forever, even with fresh
    # local/global state and both sensor sources. Retry the ORIGINAL request.
    c,clock,out=harness();c.p['local_state_enabled']=True
    assert c.on_local_navigation(local_state_from_pair(state()))
    c.on_route(envelope());proposal=deepcopy(c.proposal)
    support=deepcopy([value for kind,value in out if kind=='support'][-1])
    for i in range(1,21):
        t=i*.51;correction(c,clock,t=t,offset=0.)
        assert c.on_local_navigation(local_state_from_pair(state(t=t)))
        c.tick()
        assert len(c.prepared)==1 and c.proposal==proposal
        assert not c.prepared_for(proposal.version).retirement_requested
        assert c.transport.generation==proposal.version.reference_generation==1
        assert c.transport.pending is not None and c.transport.accepted is None
        assert c.permit is None and not c.transport.quarantined
    requests=[value for kind,value in out if kind=='proposal']
    supports=[value for kind,value in out if kind=='support']
    assert len(requests)==21 and len(supports)==1
    assert all(value==proposal for value in requests)
    assert all(value==support for value in supports)
    assert c.error.startswith('waiting_native_reference_receipt:')
    assert not [value for kind,value in out if kind=='cancel_reference']


def test_retry_admission_receipt_is_original_fact_not_renewed_motion_proof():
    c,clock,out=harness();c.on_route(envelope());proposal=deepcopy(c.proposal)
    for i in range(1,6):
        correction(c,clock,t=i*.51,offset=0.);c.tick()
    receipt=ReferenceReceipt(version=deepcopy(proposal.version),proposal_id=proposal.proposal_id,
        expected_version=deepcopy(proposal.expected_version),
        expected_trajectory_id=proposal.expected_trajectory_id,accepted=True,
        source_stamp=deepcopy(proposal.source_stamp),valid_until=deepcopy(proposal.valid_until),
        transport_mode='isolated_mock')
    assert c.on_receipt(receipt)  # durable original admission, delivered late
    assert not c.prepared_for(proposal.version).retirement_requested
    count=len([value for kind,value in out if kind=='proposal'])
    correction(c,clock,t=3.2,offset=0.);c.tick()
    assert len([value for kind,value in out if kind=='proposal'])==count
    assert c.receipt==receipt and c.proposal==proposal and c.transport.generation==1
    assert c.on_receipt(receipt)  # duplicate has no freshness side effect
    assert c.permit is None and not c.tick()['spline_visual_valid']
    assert fresh_commit(c,clock,proposal)
    assert c.transport.accepted==proposal.reference and not c.permit.allowed
    assert not c.tick()['spline_visual_valid']  # no fresh native curve proof


def test_expired_never_admitted_request_rebuilds_only_after_exact_native_rejection():
    c,clock,out=harness();c.on_route(envelope());proposal=deepcopy(c.proposal)
    correction(c,clock,t=.51,offset=0.);c.tick()
    bad=ReferenceReceipt(version=deepcopy(proposal.version),proposal_id=proposal.proposal_id+'-foreign',
        source_stamp=deepcopy(proposal.source_stamp),valid_until=deepcopy(proposal.valid_until),
        accepted=False,transport_mode='isolated_mock')
    assert not c.on_receipt(bad) and c.proposal==proposal and len(c.prepared)==1
    assert not reject_delivery(c,clock,proposal)
    assert c.proposal is None and not c.prepared and c.transport.pending is None
    route=c.transport.route
    correction(c,clock,t=1.02,offset=0.);c.tick()
    new=deepcopy(c.proposal)
    assert new is not None and new.version.reference_generation==2
    assert new.source_stamp!=proposal.source_stamp and new.proposal_id!=proposal.proposal_id
    assert c.transport.route==route and new.reference.route_hash==proposal.reference.route_hash
    assert not c.on_receipt(bad) and not reject_delivery(c,clock,proposal)
    assert c.proposal==new and len(c.prepared)==1


def test_missing_admission_receipt_and_unknown_writer_outcome_never_release_ledger():
    from d1max_planning_interfaces.msg import ExecutionHandoffGrant
    c,clock,out=harness();c.on_route(envelope());proposal=deepcopy(c.proposal)
    candidate=ExecutionPermit(version=deepcopy(proposal.version),allowed=True,geometry_committed=False,
        execution_id='execution',control_epoch=1,sdk_session='sdk',sdk_arm_generation=1,
        sequence=1,trajectory_id=1,transport_mode='isolated_mock')
    grant=ExecutionHandoffGrant(schema_version=2,handoff_id='unknown-writer-outcome',sequence=1,
        candidate=candidate,source_stamp=deepcopy(proposal.source_stamp))
    set_stamp(grant.valid_until,clock.ns+200_000_000)
    set_stamp(grant.transition_deadline,clock.ns+100_000_000)
    assert c.on_handoff(grant)
    record=c.prepared_for(proposal.version);claims=deepcopy(record.handoffs)
    for i in range(1,11):
        correction(c,clock,t=i*.51,offset=0.);c.tick()
    assert c.writer_unresolved() and c.proposal==proposal and record.handoffs==claims
    assert len(c.prepared)==1 and c.transport.pending is not None
    assert not reject_delivery(c,clock,proposal)  # refusal is not a writer outcome
    assert c.proposal==proposal and c.prepared_for(proposal.version) is record
    assert record.handoffs==claims and c.writer_unresolved()


def test_cancel_stops_pending_retry_and_late_old_receipt_cannot_revive_new_task():
    c,clock,out=harness();c.on_route(envelope());proposal=deepcopy(c.proposal)
    correction(c,clock,t=.51,offset=0.);c.tick()
    c.on_route(envelope(t=.51,sequence=2,active=False))
    count=len([value for kind,value in out if kind=='proposal'])
    clock.ns+=510_000_000;clock.mono+=.51;c.tick()
    assert len([value for kind,value in out if kind=='proposal'])==count
    assert not c.prepared and c.proposal is None
    c.on_debug(reset_debug(c,clock,c.cancel_generation))
    correction(c,clock,t=1.03,offset=0.)
    c.on_route(delivery_for('new',sequence=3,t=1.03))
    new=deepcopy(c.proposal)
    assert new is not None and new.version.task_id=='new'
    receipt=ReferenceReceipt(version=deepcopy(proposal.version),proposal_id=proposal.proposal_id,
        expected_version=deepcopy(proposal.expected_version),
        expected_trajectory_id=proposal.expected_trajectory_id,accepted=True,
        source_stamp=deepcopy(proposal.source_stamp),valid_until=deepcopy(proposal.valid_until),
        transport_mode='isolated_mock')
    assert not c.on_receipt(receipt) and c.proposal==new and len(c.prepared)==1


def test_late_epoch_or_seed_cannot_replace_new_atomic_context():
    c,clock,out=harness()
    clock.ns+=100_000_000;clock.mono+=.1
    assert c.on_navigation(state(t=.1,epoch=2))
    current=deepcopy(c.context);body_source=c.state.source_ns
    assert not c.on_navigation(state(t=.09,epoch=1))
    clock.ns+=20_000_000;clock.mono+=.02
    assert not c.on_navigation(state(t=.11,epoch=1))
    message=state(t=.09,epoch=2);message.localization_seed_id='old-seed'
    assert not c.on_navigation(message)
    assert c.context==current and c.state.source_ns==body_source


@pytest.mark.parametrize('local_event',['hard_reset','valid_new_epoch','same_epoch_new_seed'])
def test_local_identity_watermark_fences_delayed_foreign_global_context(local_event):
    c,clock,out=harness();c.p['local_state_enabled']=True
    clock.ns+=100_000_000;clock.mono+=.1
    reset=state(t=.1,epoch=2 if local_event!='same_epoch_new_seed' else 1)
    if local_event=='same_epoch_new_seed':reset.localization_seed_id='new-seed'
    local=local_state_from_pair(reset,usable=local_event!='hard_reset',reason='lio_reset')
    c.on_local_navigation(local)
    context=deepcopy(c.context);source=c.state.source_ns
    before=len([value for kind,value in out if kind=='context'])
    clock.ns+=20_000_000;clock.mono+=.02
    # The old global callback is newer than the preceding global stream, but
    # older than the already received reset when a seed changed in one epoch.
    delayed=state(t=.09 if local_event=='same_epoch_new_seed' else .11,epoch=1)
    assert not c.on_navigation(delayed)
    assert c.context==context and c.state.source_ns==source
    assert len([value for kind,value in out if kind=='context'])==before


def test_same_identity_global_can_lag_local_and_still_supply_a_real_anchor():
    c,clock,out=harness();c.p['local_state_enabled']=True
    clock.ns+=100_000_000;clock.mono+=.1
    assert c.on_local_navigation(local_state_from_pair(state(t=.1,x=.02)))
    identity=deepcopy(c.context)
    assert c.on_navigation(state(t=.09,x=.018,correction=.12))
    assert c.context==identity and c.state.source_ns==state(t=.09).source_stamp.sec*10**9+state(t=.09).source_stamp.nanosec
    assert c.local_inbox.latest.source_ns==clock.ns


def test_new_global_identity_can_lead_local_without_reusing_old_anchor():
    c,clock,out=harness();c.p['local_state_enabled']=True
    assert c.on_local_navigation(local_state_from_pair(state()))
    clock.ns+=100_000_000;clock.mono+=.1
    assert c.on_navigation(state(t=.1,epoch=2))
    assert c.context['epoch']==2 and not c.context_ready and c.permit is None


def long_envelope(length=5):
    from d1max_pct_scan.source_route_ros import to_message
    from test_continuous_reference import snapshot
    route=snapshot(points=[[float(x),0.,0.] for x in range(length+1)])
    message=envelope()
    message.route_hash=route.route_hash
    message.snapshot=to_message(route,session_id='session',task_id='task',route_id='route',
        epoch=1,seed_id='seed',stamp=message.source_stamp)
    return message


def drive(c,clock,*,t,x):
    clock.ns=BASE_NS+int(t*1e9);clock.mono=100.+t
    c.on_map(dict(c.context,valid=True,source_stamp_ns=clock.ns,
        sources=[dict(sensor_id=i,integrated_stamp_ns=clock.ns) for i in (0,1)]))
    assert c.on_navigation(state(t=t,x=x))
    c.tick()


def test_late_owner_commit_cannot_exhaust_window_before_next_reissue():
    # long_straight_00_newnative: generation 3 was proposed ~1 m into its
    # predecessor but committed ~0.9 m later. Commit-relative progress then
    # needed 1 m more, native reached the 2 m window end first and waited
    # forever. Rolling now starts after 25 cm; a late commit must still use
    # that original publication arc rather than restart its distance clock.
    c,clock,out=harness();c.on_route(long_envelope());admit(c)
    assert c.transport.accepted.path.poses[-1].pose.position.x==pytest.approx(2.)
    assert c.transport.remaining_window_arc_m()==pytest.approx(2.)
    t=x=0.
    while c.transport.pending is None:
        t+=.1;x+=.025;drive(c,clock,t=t,x=x)
        assert x<.30, 'next window must roll after 25 cm measured advance'
    assert x==pytest.approx(.25,abs=.03)
    second=native_received(c)
    end=second.reference.path.poses[-1].pose.position.x
    assert end==pytest.approx(x+2.,abs=.03)
    while x<1.9:  # native accepted geometry, owner commit arrives late
        t+=.1;x+=.025;drive(c,clock,t=t,x=x)
    assert c.transport.pending is not None
    assert fresh_commit(c,clock,second,sequence=2)
    assert c.transport.accepted.generation==second.version.reference_generation
    assert c.transport.remaining_window_arc_m()==pytest.approx(end-x,abs=.03)
    before=len([v for k,v in out if k=='proposal'])
    while c.transport.pending is None:
        t+=.1;x+=.025;drive(c,clock,t=t,x=x)
        assert x<end-.25, 'late commit must not delay the next window to the window end'
    assert x==pytest.approx(1.95,abs=.03)
    assert len([v for k,v in out if k=='proposal'])==before+1


def test_window_covering_segment_end_is_not_reissued_without_correction():
    c,clock,out=harness();c.on_route(long_envelope(length=2));admit(c)
    assert c.transport.covers_current_segment_end()
    for i in range(1,30):
        drive(c,clock,t=.1*i,x=.025*i)
    assert c.transport.pending is None
    assert len([v for k,v in out if k=='proposal'])==1


def segmented_envelope(*,landing=False):
    from test_continuous_reference import segmented_snapshot
    from d1max_pct_scan.source_route_ros import to_message
    route=segmented_snapshot(landing=landing)
    message=envelope();message.route_hash=route.route_hash
    message.snapshot=to_message(route,session_id='session',task_id='task',route_id='route',
        epoch=1,seed_id='seed',stamp=message.source_stamp)
    return message


@pytest.mark.parametrize('landing',[False,True])
def test_formal_same_floor_portal_creates_next_candidate_without_stop_or_global_replan(landing):
    c,clock,out=harness();message=segmented_envelope(landing=landing)
    c.on_route(message);admit(c)
    first=deepcopy(c.transport.accepted);route=deepcopy(c.transport.route)
    # Even within the ordinary refresh cooldown, a real portal sample must
    # not be lost. The one pending slot prevents repeated boundary requests.
    c.last_proposal_monotonic=100.79
    drive(c,clock,t=.8,x=.95)
    second=c.proposal
    assert second is not None and second.reference.segment_id=='corridor_b'
    assert second.version.route_hash==first.route_hash
    assert second.reference.path.header.stamp==state(t=.8).source_stamp
    assert c.transport.core._segment==0 and c.transport.accepted==first
    assert c.transport.pending_core._segment==1
    assert c.transport.route==route and not c.transport.core._stationary()
    support=[v for k,v in out if k=='support'][-1]
    assert support.floor_id=='floor1'
    assert support.segment_kind==('landing' if landing else 'floor')
    assert support.required_mode=='general' and support.version==second.version
    count=len([v for k,v in out if k=='proposal'])
    c.refresh_window();c.refresh_window()
    assert len([v for k,v in out if k=='proposal'])==count
    # Native admission alone is not reference ownership. The current owner
    # commits the candidate before the source-bound progress changes segment.
    admit(c,sequence=2)
    assert c.transport.accepted.segment_id=='corridor_b'
    assert c.transport.core._progress.source_ns==clock.ns
    assert c.transport.route==route and c.transport.snapshot.route_hash==message.route_hash


def test_formal_stair_boundary_does_not_auto_advance_even_after_actual_stop():
    c,clock,out=harness();c.on_route(envelope(crossfloor=True));admit(c)
    first=deepcopy(c.transport.accepted)
    for t in (.8,.9,1.):drive(c,clock,t=t,x=.95)
    assert c.transport.core._stationary()
    assert c.transport.pending is None and c.transport.accepted==first
    assert len([v for k,v in out if k=='proposal'])==1


def test_support_uses_bound_stair_segment_not_static_floor_parameter():
    from d1max_pct_scan.continuous_reference import ModeObservation
    c,clock,out=harness();c.on_route(envelope(crossfloor=True));admit(c)
    drive(c,clock,t=.8,x=.95)
    core=c.transport.core
    mode=ModeObservation(core.context,'task',core.snapshot.route_hash,'stair_lower','stair',clock.ns,clock.mono)
    assert c.transport.request_segment_transition(current_source_ns=clock.ns,
        now_monotonic=clock.mono,mode=mode)=='stair_lower'
    support=[v for k,v in out if k=='support'][-1]
    assert support.floor_id=='stair_lower' and support.segment_kind=='stair_up'
    assert support.required_mode=='stair' and not support.verified
    assert c.transport.accepted.segment_id=='lower_floor'  # No automatic mode/geometry authorization.


def test_support_floor_identity_comes_from_route_not_session_ui_parameter():
    from d1max_pct_scan.source_route import RouteSnapshot
    from d1max_pct_scan.source_route_ros import to_message
    from test_continuous_reference import snapshot
    value=snapshot().payload();value['point_floor_ids']=['floor2']*len(value['xyz'])
    value['segments'][0]['floor_id']='floor2'
    route=RouteSnapshot.create(value)
    message=envelope();message.route_hash=route.route_hash
    message.snapshot=to_message(route,session_id='session',task_id='task',route_id='route',
        epoch=1,seed_id='seed',stamp=message.source_stamp)
    c,_,out=harness();assert c.p['floor_id']=='floor1'
    c.on_route(message)
    support=[v for k,v in out if k=='support'][-1]
    assert support.floor_id=='floor2' and support.segment_kind=='floor' and support.required_mode=='general'
    assert not support.verified  # Correct semantics do not authorize another physical floor.
