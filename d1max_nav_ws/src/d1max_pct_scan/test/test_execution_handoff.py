from copy import deepcopy
from types import SimpleNamespace as N

import pytest

from test_execution_safety import fixture, motion_proof, NOW, stamp
from d1max_pct_scan.execution_handoff import ExecutionHandoffAdmission


def vec(x=0.,y=0.,z=0.):return N(x=x,y=y,z=z)
def pose(x=0.):return N(position=vec(x),orientation=N(x=0.,y=0.,z=0.,w=1.))
def twist(x=.2):return N(linear=vec(x),angular=vec(z=.1))


def complete_permit(p,*,sequence=None,when=NOW):
    p=deepcopy(p);p.frame_id='d1max_loc_odom';p.confirmation_id='confirmed-'+p.execution_id
    p.source_stamp=stamp(when);p.valid_until=stamp(when+200_000_000)
    if sequence is not None:p.sequence=sequence
    return p


def next_authority(h,*,when=NOW+50_000_000):
    p=complete_permit(h.writer_authority,sequence=h.core.last_permit_sequence+1,when=when)
    p.version.task_id='next-task';p.version.route_id='next-route';p.version.route_hash='d'*64
    p.version.reference_generation+=1;p.execution_id='next-execution';p.control_epoch+=1
    p.sdk_session='new-sdk-session';p.sdk_arm_generation=1;p.trajectory_id=5
    p.allowed=True;p.revoked=False;p.geometry_committed=True;p.phase='tracking'
    return p


def retire_authority(h,*,when=NOW+30_000_000):
    p=complete_permit(h.applied or h.writer_authority,sequence=h.core.last_permit_sequence+1,when=when)
    p.allowed=False;p.revoked=True;p.phase='terminal'
    assert h.on_permit(p,when)
    return p


def ack(p,*,g=None,sequence=1,applied=True,when=NOW):
    return N(schema_version=1,handoff_id=g.handoff_id if g else '',grant_sequence=g.sequence if g else 0,
        sequence=sequence,previous_commit_sequence=g.expected_commit_sequence if g else 0,
        commit_sequence=(g.expected_commit_sequence+int(applied)) if g else 1,
        incumbent_trajectory_id=g.incumbent.trajectory_id if g else 0,
        incumbent_version=deepcopy(g.incumbent.version) if g else deepcopy(p.version),
        candidate_trajectory_id=p.trajectory_id,candidate_version=deepcopy(p.version),
        permit_sequence=p.sequence,execution_id=p.execution_id,sdk_session=p.sdk_session,
        control_epoch=p.control_epoch,sdk_arm_generation=p.sdk_arm_generation,
        transport_mode=p.transport_mode,applied_at=stamp(when),body_source_stamp=stamp(when),
        measured_pose=N(header=N(frame_id='d1max_loc_odom',stamp=stamp(when)),pose=pose()),
        write_submitted=True,write_acknowledged=False,applied=applied)


def ready():
    c,p,d,curve,*_=fixture();p.phase='tracking'
    c.permits.clear();c.last_permit_sequence=0;assert c.on_permit(p)
    h=ExecutionHandoffAdmission(c,enabled=True)
    assert h.on_ack(ack(p),NOW)
    b=deepcopy(p);b.sequence=2;b.trajectory_id=3;b.version.anchor_revision+=1
    b.geometry_committed=False
    g=N(schema_version=2,transition_mode=0,handoff_id='handoff-1',sequence=1,expected_commit_sequence=1,
        incumbent=deepcopy(p),candidate=b,source_stamp=stamp(),valid_until=stamp(NOW+200_000_000),
        transition_deadline=stamp(NOW+200_000_000),retain_incumbent_until=stamp(NOW+200_000_000),
        revoked=False,reason='conditional_not_applied')
    assert h.on_grant(g,NOW)
    curve=deepcopy(curve);curve.version=deepcopy(b.version);curve.trajectory_id=b.trajectory_id
    assert c.on_validation(curve)
    new=deepcopy(d);new.version=deepcopy(b.version);new.trajectory_id=b.trajectory_id;new.permit_sequence=b.sequence
    m=N(schema_version=1,handoff_id=g.handoff_id,grant_sequence=g.sequence,expected_commit_sequence=1,
        demand=new,entry_source_stamp=stamp(),measured_pose=N(header=N(frame_id='d1max_loc_odom',stamp=stamp()),pose=pose()),
        measured_twist=twist(),curve_entry_pose=pose(),curve_entry_twist=twist(),curve_time=0.,
        entry_admission=N(sequence=1,version=deepcopy(b.version),trajectory_id=b.trajectory_id,
            validation_sequence=3,body_source_stamp=stamp(),checked_at=stamp(),valid_until=stamp(NOW+100_000_000),accepted=True),
        position_tolerance_m=.0125,velocity_tolerance_mps=.05,position_error_m=0.,velocity_error_mps=0.)
    proof=motion_proof(new);proof.handoff_id=g.handoff_id;proof.entry_admission_sequence=1
    proof.entry_curve_pose=deepcopy(m.curve_entry_pose);proof.entry_curve_twist=deepcopy(m.curve_entry_twist)
    proof.entry_curve_time=0.
    return h,p,d,g,m,proof


def test_conditional_admission_never_changes_active_geometry_and_ack_promotes_original_lease():
    h,p,d,g,m,proof=ready()
    assert h.core.permits[-1]==p and h.applied==p
    assert h.ordinary.demand(d,NOW).allowed
    assert h.prepared_demand(m,NOW).reason=='waiting_command_sweep'
    prepared,out=h.motion_proof(proof,NOW+20_000_000)
    assert prepared and out.allowed and out.output.demand.safety_checked
    assert out.output.demand.source_stamp==m.demand.source_stamp
    assert h.core.permits[-1]==p and h.commit_sequence==1
    a=ack(g.candidate,g=g,sequence=2,when=NOW+30_000_000)
    assert h.on_ack(a,NOW+40_000_000)
    committed=deepcopy(g.candidate);committed.geometry_committed=True
    assert h.applied==committed and h.commit_sequence==2 and h.core.permits[-1]==committed
    assert h.core.permits[-1].valid_until==g.candidate.valid_until
    assert h.candidate is None and not h.prepared
    original_lease=deepcopy(h.core.permits[-1])
    repeated=deepcopy(a);repeated.sequence=3
    assert h.on_ack(repeated,NOW+60_000_000)
    assert h.core.permits[-1]==original_lease and h.commit_sequence==2
    delayed=deepcopy(p);delayed.sequence=8;delayed.source_stamp=stamp(NOW+40_000_000)
    assert not h.on_permit(delayed) and h.applied==committed


@pytest.mark.parametrize('change',['id','expected','source','pose','velocity','entry_id','native_entry','no_raw','negative','expired'])
def test_prepared_dangerous_or_foreign_evidence_never_clears_incumbent(change):
    h,p,d,g,m,proof=ready();now=NOW+20_000_000
    if change=='id':m.handoff_id='foreign'
    elif change=='expected':m.expected_commit_sequence=0
    elif change=='source':m.entry_source_stamp=stamp(NOW+1)
    elif change=='pose':m.measured_pose.pose.position.x=.0125001
    elif change=='velocity':m.measured_twist.linear.x=.250001
    elif change=='entry_id':m.entry_admission.sequence+=1
    elif change=='native_entry':proof.entry_curve_time=.1
    elif change=='no_raw':h.core.raw[1].clear()
    elif change=='negative':proof.valid=False
    else:now=NOW+201_000_000
    assert h.core.on_motion_validation(proof)
    out=h.prepared_demand(m,now)
    assert not out.allowed and out.output is None
    assert h.applied==p and h.core.permits[-1]==p and h.commit_sequence==1


def test_zero_prepared_requires_true_collision_and_entry_proof():
    h,p,d,g,m,proof=ready();m.demand.velocity=twist(0.);m.demand.velocity.angular.z=0.
    assert not h.prepared_demand(m,NOW).allowed
    proof=motion_proof(m.demand);proof.handoff_id=g.handoff_id;proof.entry_admission_sequence=1
    proof.entry_curve_pose=m.curve_entry_pose;proof.entry_curve_twist=m.curve_entry_twist;proof.entry_curve_time=0.
    proof.valid=False;h.core.on_motion_validation(proof)
    assert not h.prepared_demand(m,NOW).allowed
    assert h.applied==p


def test_zero_prepared_with_actual_free_sweep_is_a_conditional_offer_not_a_commit():
    h,p,d,g,m,proof=ready();m.demand.velocity=twist(0.);m.demand.velocity.angular.z=0.
    proof=motion_proof(m.demand);proof.handoff_id=g.handoff_id;proof.entry_admission_sequence=1
    proof.entry_curve_pose=m.curve_entry_pose;proof.entry_curve_twist=m.curve_entry_twist;proof.entry_curve_time=0.
    assert h.core.on_motion_validation(proof)
    result=h.prepared_demand(m,NOW)
    assert result.allowed and result.output.demand.safety_checked and h.applied==p
    assert result.output.demand.velocity.linear.x==0. and h.commit_sequence==1


def test_repeat_grant_does_not_extend_budget_and_expired_negative_ack_allows_retirement():
    h,p,d,g,m,proof=ready();assert h.on_grant(deepcopy(g),NOW+10_000_000)
    renewed=deepcopy(g);renewed.transition_deadline=stamp(NOW+300_000_000)
    assert not h.on_grant(renewed,NOW+10_000_000)
    a=ack(g.candidate,g=g,sequence=2,applied=False,when=NOW+250_000_000)
    assert h.on_ack(a,NOW+250_000_000)
    assert h.commit_sequence==1 and h.applied==p and h.candidate is None
    assert not h.on_grant(g,NOW+250_000_000)


def test_revoked_late_write_fact_advances_identity_but_never_restores_authority():
    h,p,d,g,m,proof=ready();revoked=deepcopy(p);revoked.sequence=4;revoked.revoked=True
    assert h.on_permit(revoked)
    a=ack(g.candidate,g=g,sequence=2,when=NOW+10_000_000)
    assert h.on_ack(a,NOW+100_000_000)
    committed=deepcopy(g.candidate);committed.geometry_committed=True
    assert h.applied==committed and h.commit_sequence==2 and not h.core.permits
    assert not h.ordinary.demand(m.demand,NOW+100_000_000).allowed


@pytest.mark.parametrize('field',['sdk_session','control_epoch','sdk_arm_generation','execution_id','transport_mode'])
def test_foreign_negative_ack_cannot_retire_current_conditional_transaction(field):
    h,p,d,g,m,proof=ready();a=ack(g.candidate,g=g,sequence=2,applied=False)
    setattr(a,field,999 if field.endswith('epoch') or field.endswith('generation') else 'foreign')
    assert not h.on_ack(a,NOW) and h.candidate is not None and h.commit_sequence==1


def test_known_writer_error_advances_identity_but_requires_hard_hold_not_old_fallback():
    h,p,d,g,m,proof=ready();a=ack(g.candidate,g=g,sequence=2);a.write_submitted=False
    assert h.on_ack(a,NOW)
    assert h.commit_sequence==2 and h.applied.trajectory_id==g.candidate.trajectory_id
    assert not h.core.permits
    heartbeat=deepcopy(g.candidate);heartbeat.sequence+=5;heartbeat.geometry_committed=True
    assert not h.on_permit(heartbeat)


@pytest.mark.parametrize('change',['old_commit','foreign_task','after_deadline','expired_fact','frame','duplicate'])
def test_ack_cannot_invent_a_write_or_revive_another_identity(change):
    h,p,d,g,m,proof=ready();a=ack(g.candidate,g=g,sequence=2,when=NOW+10_000_000);now=NOW+20_000_000
    if change=='old_commit':a.previous_commit_sequence=0
    elif change=='foreign_task':a.candidate_version.task_id='foreign'
    elif change=='after_deadline':a.applied_at=stamp(NOW+201_000_000);now=NOW+201_000_000
    elif change=='expired_fact':now=NOW+400_000_000
    elif change=='frame':a.measured_pose.header.frame_id='map'
    else:a.sequence=1
    assert not h.on_ack(a,now) and h.commit_sequence==1 and h.applied==p


def stationary_ready():
    h,p,d,g,m,proof=ready()
    assert h.on_ack(ack(g.candidate,g=g,sequence=2,applied=False),NOW)
    hold=deepcopy(p);hold.sequence=10;hold.allowed=False;hold.phase='holding'
    assert h.on_permit(hold)
    g=deepcopy(g);g.handoff_id='stationary-2';g.sequence=2;g.transition_mode=1
    g.incumbent=deepcopy(hold);g.candidate.sequence=11
    g.retain_incumbent_until=stamp(NOW)
    e=N(schema_version=1,version=deepcopy(p.version),sequence=1,writer_commit_sequence=1,
        execution_id=p.execution_id,control_epoch=p.control_epoch,sdk_session=p.sdk_session,
        sdk_arm_generation=p.sdk_arm_generation,transport_mode=p.transport_mode,
        applied_trajectory_id=p.trajectory_id,zero_write_sequence=1,zero_ack_at=stamp(NOW-800_000_000),
        mc_raw_stamp_ns=123000,mc_clock_epoch='mock-clock',time_basis='isolated_simulated_source_clock',
        source_stamp=stamp(NOW),received_stamp=stamp(NOW),capture_lower_bound=stamp(NOW-10_000_000),
        capture_upper_bound=stamp(NOW),mc_capture_delay_bound_sec=.01,
        valid_until=stamp(NOW+250_000_000),stationary_samples=35,stationary_duration_sec=.7,
        measured_linear_mps=0.,measured_angular_radps=0.,nonzero_blocked=True,usable=True,
        physical_acceptance_verified=False)
    g.stationary_evidence=deepcopy(e)
    return h,p,g,e


def test_stationary_reentry_uses_shared_noise_policy_without_weakening_source_evidence():
    h,p,g,e=stationary_ready()
    h.core.stationary_policy=dict(linear_threshold_mps=.04,angular_threshold_radps=.05,
        reentry_duration_s=.6,minimum_new_samples=3)
    e.measured_linear_mps=.038;g.stationary_evidence=deepcopy(e)
    assert h.on_stationary(e) and h.on_grant(g,NOW)
    h,p,g,e=stationary_ready()
    h.core.stationary_policy=dict(linear_threshold_mps=.04,angular_threshold_radps=.05,
        reentry_duration_s=.6,minimum_new_samples=3)
    e.measured_linear_mps=.041;g.stationary_evidence=deepcopy(e)
    assert h.on_stationary(e) and not h.on_grant(g,NOW)


def test_stationary_reentry_requires_independent_original_sdk_evidence_and_no_old_motion():
    h,p,g,e=stationary_ready()
    assert not h.on_grant(g,NOW)
    assert h.on_stationary(e) and h.on_grant(g,NOW)
    assert not h.core.permits[-1].allowed and h.applied.trajectory_id==p.trajectory_id
    heartbeat=deepcopy(g.incumbent);heartbeat.sequence+=4
    assert h.on_permit(heartbeat) and h.candidate is not None
    a=ack(g.candidate,g=g,sequence=3,when=NOW+20_000_000)
    assert h.on_ack(a,NOW+30_000_000)
    assert h.commit_sequence==2 and h.core.permits[-1].allowed
    assert h.core.permits[-1].valid_until==g.candidate.valid_until


@pytest.mark.parametrize('change',['unusable','mc_moving','yaw_moving','two_samples','short_window',
    'before_zero_ack','different_raw_clock','foreign_sdk','wrong_commit','old_can_move','retained_motion','expiry'])
def test_stationary_reentry_dangerous_or_unbound_witness_is_rejected(change):
    h,p,g,e=stationary_ready()
    if change=='unusable':e.usable=False
    elif change=='mc_moving':e.measured_linear_mps=.031
    elif change=='yaw_moving':e.measured_angular_radps=.051
    elif change=='two_samples':e.stationary_samples=2
    elif change=='short_window':e.stationary_duration_sec=.599
    elif change=='before_zero_ack':e.zero_ack_at=e.capture_lower_bound
    elif change=='different_raw_clock':e.mc_clock_epoch=''
    elif change=='foreign_sdk':e.sdk_session='foreign'
    elif change=='wrong_commit':e.writer_commit_sequence+=1
    elif change=='old_can_move':g.incumbent.allowed=True
    elif change=='retained_motion':g.retain_incumbent_until=stamp(NOW+1)
    else:e.valid_until=stamp(NOW-1)
    g.stationary_evidence=deepcopy(e)
    assert h.on_stationary(e)==(change!='foreign_sdk')
    assert not h.on_grant(g,NOW) and h.commit_sequence==1


def test_negative_stationary_evidence_is_not_overridden_by_an_older_cached_witness():
    h,p,g,e=stationary_ready();assert h.on_stationary(e)
    lost=deepcopy(e);lost.sequence+=1;lost.usable=False
    assert h.on_stationary(lost) and not h.on_grant(g,NOW)


def test_applied_new_identity_invalidates_stop_witness_without_revoking_delayed_write_fact():
    h,p,g,e=stationary_ready();assert h.on_stationary(e) and h.on_grant(g,NOW)
    later=deepcopy(e);later.sequence+=1;later.usable=False
    later.version=deepcopy(g.candidate.version);later.applied_trajectory_id=g.candidate.trajectory_id
    later.writer_commit_sequence=2
    assert h.on_stationary(later) and h.candidate is not None
    assert h.on_ack(ack(g.candidate,g=g,sequence=3),NOW)
    assert h.commit_sequence==2 and h.core.permits[-1].allowed


def test_retired_execution_starts_independent_ack_and_owner_grant_highwaters():
    h,p,d,g,m,proof=ready()
    assert h.on_ack(ack(g.candidate,g=g,sequence=16,when=NOW+10_000_000),NOW+20_000_000)
    next_old=deepcopy(g);next_old.handoff_id='old-handoff-8';next_old.sequence=8
    next_old.expected_commit_sequence=2;next_old.incumbent=deepcopy(h.applied)
    next_old.candidate=deepcopy(h.applied);next_old.candidate.geometry_committed=False
    next_old.candidate.sequence=3;next_old.candidate.trajectory_id=4
    next_old.candidate.version.anchor_revision+=1
    assert h.on_grant(next_old,NOW+20_000_000)
    assert (h.last_ack_sequence,h.last_grant_sequence,h.commit_sequence)==(16,8,2)
    retire_authority(h)
    new=next_authority(h);assert h.on_permit(new,NOW+50_000_000)
    assert (h.last_ack_sequence,h.last_grant_sequence,h.commit_sequence)==(0,0,0)
    assert h.applied is None and h.grant is None
    assert h.on_ack(ack(new,sequence=2,when=NOW+50_000_000),NOW+60_000_000)
    new_grant=deepcopy(g);new_grant.handoff_id='next-execution:handoff:1';new_grant.sequence=1
    new_grant.incumbent=deepcopy(h.applied);new_grant.candidate=deepcopy(new)
    new_grant.candidate.sequence+=1;new_grant.candidate.trajectory_id=6
    new_grant.candidate.version.anchor_revision+=1;new_grant.candidate.geometry_committed=False
    new_grant.source_stamp=stamp(NOW+60_000_000)
    new_grant.transition_deadline=stamp(NOW+200_000_000)
    new_grant.valid_until=stamp(NOW+200_000_000)
    new_grant.retain_incumbent_until=stamp(NOW+200_000_000)
    assert h.on_grant(new_grant,NOW+60_000_000)
    assert h.commit_sequence==1 and h.last_ack_sequence==2 and h.last_grant_sequence==1
    assert h.applied.valid_until==new.valid_until


@pytest.mark.parametrize('kind',['initial','handoff_positive','handoff_negative'])
def test_old_large_ack_cannot_poison_or_roll_back_new_execution(kind):
    h,p,d,g,m,proof=ready();retire_authority(h)
    new=next_authority(h);assert h.on_permit(new,NOW+50_000_000)
    assert h.on_ack(ack(new,sequence=2,when=NOW+50_000_000),NOW+60_000_000)
    snapshot=(h.last_ack_sequence,h.last_grant_sequence,h.commit_sequence,deepcopy(h.applied),deepcopy(h.core.permits))
    old=ack(p if kind=='initial' else g.candidate,g=None if kind=='initial' else g,
        sequence=1000,applied=kind!='handoff_negative',when=NOW+60_000_000)
    assert not h.on_ack(old,NOW+60_000_000)
    assert snapshot==(h.last_ack_sequence,h.last_grant_sequence,h.commit_sequence,h.applied,h.core.permits)


@pytest.mark.parametrize('change',['unretired','same_task','same_execution','same_epoch','map','session',
    'sdk','arm','confirmation','frame','context','future_one_ns','expired','long_lease','holding','uncommitted','no_clock'])
def test_new_execution_requires_complete_fresh_positive_forward_authority(change):
    h,p,d,g,m,proof=ready()
    if change!='unretired':retire_authority(h)
    new=next_authority(h);now=NOW+50_000_000
    if change=='same_task':new.version.task_id=p.version.task_id
    elif change=='same_execution':new.execution_id=p.execution_id
    elif change=='same_epoch':new.control_epoch=p.control_epoch
    elif change=='map':new.version.map_version_id='foreign'
    elif change=='session':new.version.session_id='foreign'
    elif change=='sdk':new.sdk_session=''
    elif change=='arm':new.sdk_arm_generation=0
    elif change=='confirmation':new.confirmation_id=''
    elif change=='frame':new.frame_id='foreign'
    elif change=='context':new.version.context_sequence=0
    elif change=='future_one_ns':new.source_stamp=stamp(now+1)
    elif change=='expired':new.valid_until=stamp(now)
    elif change=='long_lease':new.valid_until=stamp(now+750_000_001)
    elif change=='holding':new.allowed=False;new.phase='holding'
    elif change=='uncommitted':new.geometry_committed=False
    elif change=='no_clock':now=None
    snapshot=(h.last_ack_sequence,h.last_grant_sequence,h.commit_sequence,deepcopy(h.applied),
        h.core.last_permit_sequence,deepcopy(h.writer_authority),deepcopy(h.core.permits))
    assert not h.on_permit(new,now)
    assert snapshot==(h.last_ack_sequence,h.last_grant_sequence,h.commit_sequence,h.applied,
        h.core.last_permit_sequence,h.writer_authority,h.core.permits)


def test_retirement_keeps_late_initial_heartbeat_write_fact_under_hold():
    c,p,*_=fixture();p=complete_permit(p);p.phase='tracking'
    c.permits.clear();c.last_permit_sequence=0
    h=ExecutionHandoffAdmission(c,enabled=True);assert h.on_permit(p,NOW)
    heartbeat=complete_permit(p,sequence=2,when=NOW+10_000_000)
    assert h.on_permit(heartbeat,NOW+10_000_000)
    retired=retire_authority(h,when=NOW+30_000_000)
    assert h.commit_sequence==0 and h.last_ack_sequence==0
    assert h.on_ack(ack(heartbeat,sequence=16,when=NOW+20_000_000),NOW+50_000_000)
    assert h.applied==heartbeat and h.commit_sequence==1 and h.last_ack_sequence==16
    assert not h.core.permits and h.applied.source_stamp==heartbeat.source_stamp
    assert h.applied.valid_until==heartbeat.valid_until
    restored=complete_permit(heartbeat,sequence=retired.sequence+1,when=NOW+50_000_000)
    assert not h.on_permit(restored,NOW+50_000_000)
    new=next_authority(h,when=NOW+60_000_000)
    assert h.on_permit(new,NOW+60_000_000)
    assert h.commit_sequence==0 and h.last_ack_sequence==0
    assert h.on_ack(ack(new,sequence=2,when=NOW+60_000_000),NOW+70_000_000)


def test_retirement_keeps_late_handoff_fact_and_cannot_restore_old_positive_lease():
    h,p,d,g,m,proof=ready();retired=retire_authority(h)
    assert h.on_ack(ack(g.candidate,g=g,sequence=16,when=NOW+20_000_000),NOW+50_000_000)
    assert h.commit_sequence==2 and h.last_ack_sequence==16 and not h.core.permits
    restored=complete_permit(h.applied,sequence=retired.sequence+1,when=NOW+50_000_000)
    assert not h.on_permit(restored,NOW+50_000_000)
    assert h.commit_sequence==2 and h.last_ack_sequence==16 and not h.core.permits


def test_same_execution_heartbeat_and_holding_replan_do_not_restart_writer_counters():
    h,p,d,g,m,proof=ready()
    assert h.on_ack(ack(g.candidate,g=g,sequence=16,when=NOW+10_000_000),NOW+20_000_000)
    heartbeat=complete_permit(h.applied,sequence=4,when=NOW+20_000_000)
    assert h.on_permit(heartbeat,NOW+20_000_000)
    holding=deepcopy(heartbeat);holding.sequence+=1;holding.allowed=False;holding.phase='holding'
    assert h.on_permit(holding,NOW+30_000_000)
    assert (h.last_ack_sequence,h.last_grant_sequence,h.commit_sequence)==(16,1,2)
    assert h.retired_authority is None


def test_new_scope_rejects_old_grant_and_stationary_before_their_highwaters():
    h,p,g,e=stationary_ready();assert h.on_stationary(e);retire_authority(h)
    new=next_authority(h);assert h.on_permit(new,NOW+50_000_000)
    assert h.on_ack(ack(new,sequence=2,when=NOW+50_000_000),NOW+60_000_000)
    old_grant=deepcopy(g);old_grant.sequence=1000
    old_evidence=deepcopy(e);old_evidence.sequence=1000
    assert not h.on_grant(old_grant,NOW+60_000_000)
    assert not h.on_stationary(old_evidence)
    assert h.last_grant_sequence==0 and not h.stationary and h.commit_sequence==1


def test_disabled_writer_handoff_preserves_ordinary_permission_behavior():
    c,p,*_=fixture();h=ExecutionHandoffAdmission(c,enabled=False)
    p=complete_permit(p,sequence=2);p.phase='tracking'
    assert h.on_permit(p,NOW)
    new=deepcopy(p);new.sequence+=1;new.version.task_id='next-task'
    new.execution_id='next-execution';new.control_epoch+=1
    assert h.on_permit(new,NOW)
    assert h.writer_authority is None and h.commit_sequence==h.last_ack_sequence==0
    assert c.permits[-1]==new
