from copy import deepcopy
import pytest
from d1max_pct_scan.command_admission import CommandAdmission
from test_execution_safety import fixture,motion_proof,NOW,stamp


def pending():
    core,permit,demand,*_=fixture()
    core.motion_proofs.clear()
    queue=CommandAdmission(core)
    result=queue.demand(demand,NOW)
    assert not result.allowed and result.output is None and result.reason=='waiting_command_sweep'
    return core,queue,permit,demand


def test_async_proof_keeps_original_command_and_source_time():
    core,queue,permit,demand=pending()
    permit=deepcopy(permit);permit.sequence+=1;core.on_permit(permit)
    out=queue.proof(motion_proof(demand),NOW+20_000_000)
    assert out.allowed and out.output.motion_validation_sequence==1
    assert out.output.source_stamp==demand.source_stamp and out.output.valid_until==demand.valid_until
    assert out.output.velocity==demand.velocity and not queue.pending


@pytest.mark.parametrize('change',['model','velocity','body_time','ray_watermark','expired','invalid'])
def test_actual_command_proof_rejects_dangerous_mismatches(change):
    _,queue,_,demand=pending();proof=motion_proof(demand)
    if change=='model':proof.braking_model_sha256='d'*64
    elif change=='velocity':proof.velocity.angular.z=.3
    elif change=='body_time':proof.demand_body_source_stamp=stamp(NOW+1)
    elif change=='ray_watermark':proof.rear_ray_source_stamp=stamp(NOW-1)
    elif change=='expired':proof.valid_until=stamp(NOW)
    else:proof.valid=False;proof.reason='occupied_on_braking_sweep'
    result=queue.proof(proof,NOW)
    assert not result.allowed
    assert result.output is not None and result.output.hold
    assert result.output.velocity.linear.x==result.output.velocity.angular.z==0


def test_late_proof_cannot_refresh_old_source_or_revive_revoked_curve():
    core,queue,permit,demand=pending()
    assert not queue.proof(motion_proof(demand),NOW+110_000_000).allowed
    assert not queue.pending
    core,queue,permit,demand=pending()
    permit=deepcopy(permit);permit.sequence+=1;permit.revoked=True;core.on_permit(permit)
    result=queue.proof(motion_proof(demand),NOW)
    assert not result.allowed and result.output is None


def test_new_pending_does_not_clear_previous_verified_command_and_is_bounded():
    core,permit,demand,*_=fixture();queue=CommandAdmission(core)
    assert queue.demand(demand,NOW).allowed
    for seq in range(2,50):
        future=deepcopy(demand);future.sequence=seq
        result=queue.demand(future,NOW)
        assert not result.allowed and result.output is None
    assert len(queue.pending)==8
    assert not queue.proof(motion_proof(demand),NOW).allowed


def test_invalid_proof_cannot_be_overwritten_for_same_command():
    core,queue,_,demand=pending();proof=motion_proof(demand);proof.valid=False
    queue.proof(proof,NOW)
    proof.sequence+=1;proof.valid=True
    assert not core.on_motion_validation(proof)


@pytest.mark.parametrize('change',['curve','phase','hold','version'])
def test_late_command_proof_cannot_cross_latest_execution_boundary(change):
    core,queue,permit,demand=pending();permit=deepcopy(permit);permit.sequence+=1
    if change=='curve':permit.trajectory_id+=1
    elif change=='phase':permit.phase='ALIGNING'
    elif change=='hold':permit.allowed=False
    else:permit.version.anchor_revision+=1
    core.on_permit(permit)
    result=queue.proof(motion_proof(demand),NOW)
    assert not result.allowed and result.output is None


def phase_fixture(phase_ms=30):
    core,permit,demand,prior,*_=fixture()
    core.motion_proofs.clear()
    prior=deepcopy(prior);prior.sequence=4;prior.valid_until=stamp(NOW+5_000_000)
    assert core.on_validation(prior)
    demand=deepcopy(demand);demand.validation_sequence=4
    demand.valid_until=stamp(NOW+100_000_000)
    queue=CommandAdmission(core)
    waiting=queue.demand(demand,NOW)
    assert waiting.reason=='waiting_command_sweep' and waiting.output is None
    now=NOW+phase_ms*1_000_000
    current=deepcopy(prior);current.sequence=5
    current.source_stamp=current.check_end=current.body_source_stamp=stamp(now)
    current.check_begin=stamp(now-100_000);current.valid_until=stamp(NOW+180_000_000)
    sweep=motion_proof(demand);sweep.trajectory_validation_sequence=5
    sweep.check_begin=stamp(now-50_000);sweep.check_end=sweep.body_source_stamp=stamp(now)
    sweep.valid_until=stamp(NOW+100_000_000)
    return core,queue,demand,current,sweep,now


@pytest.mark.parametrize('phase_ms',[10,19,29,39,49])
def test_fifty_ms_writer_phase_can_use_new_native_evidence_without_restamping_control(phase_ms):
    core,queue,demand,current,sweep,now=phase_fixture(phase_ms)
    core.on_validation(current)
    out=queue.proof(sweep,now)
    assert out.allowed and out.output is not None
    assert out.output.source_stamp==demand.source_stamp
    assert out.output.body_source_stamp==demand.body_source_stamp
    assert out.output.valid_until==demand.valid_until==stamp(NOW+100_000_000)
    assert out.output.validation_sequence==4 and sweep.trajectory_validation_sequence==5
    assert sweep.valid_until==stamp(NOW+100_000_000)
    # The old proof's 5 ms deadline has not moved. Both independent current
    # sweep and unchanged command deadlines still cover the 50 ms writer tick.
    assert core.proof_history[next(iter(core.proof_history))][-2].valid_until==stamp(NOW+5_000_000)
    assert not queue.pending


def test_command_proof_before_its_named_curve_proof_waits_then_resolves_original_pending():
    core,queue,demand,current,sweep,now=phase_fixture()
    waiting=queue.proof(sweep,now)
    assert waiting.reason=='waiting_native_curve_proof' and waiting.output is None
    assert len(queue.pending)==1
    results=queue.validation(current,now+1_000_000)
    assert len(results)==1 and results[0].allowed
    assert results[0].output.source_stamp==demand.source_stamp
    assert results[0].output.valid_until==demand.valid_until
    assert not queue.pending


@pytest.mark.parametrize('change',['intervening_negative','latest_negative','old_revision','control_expired',
                                  'body_expired','raw_missing','raw_expired','foreign_curve'])
def test_new_sweep_cannot_resurrect_stale_negative_or_foreign_commands(change):
    core,queue,demand,current,sweep,now=phase_fixture()
    if change=='intervening_negative':
        negative=deepcopy(current);negative.valid=False;core.on_validation(negative)
        current.sequence=sweep.trajectory_validation_sequence=6
    elif change=='latest_negative':
        current.valid=False
    elif change=='old_revision':
        sweep.trajectory_validation_sequence=3
    elif change=='control_expired':
        now=NOW+101_000_000
    elif change=='body_expired':
        original=next(iter(queue.pending.values()))
        original.body_source_stamp=stamp(NOW-80_000_000)
        sweep.demand_body_source_stamp=deepcopy(original.body_source_stamp)
    elif change=='raw_missing':
        core.raw[1].clear()
    elif change=='raw_expired':
        sweep.front_ray_source_stamp=stamp(NOW-600_000_000)
    elif change=='foreign_curve':
        sweep.trajectory_id+=1
    core.on_validation(current)
    result=queue.proof(sweep,now)
    assert not result.allowed
    assert result.output is None or result.output.hold


def test_missing_curve_delivery_cannot_hold_pending_past_original_control_source_ttl():
    core,queue,demand,current,sweep,now=phase_fixture()
    assert queue.proof(sweep,now).reason=='waiting_native_curve_proof'
    assert queue.validation(current,NOW+101_000_000)==()
    assert not queue.pending


def test_late_intervening_negative_fences_old_demand_without_replacing_latest_snapshot():
    core,queue,demand,current,sweep,now=phase_fixture()
    negative=deepcopy(current);negative.valid=False
    current.sequence=sweep.trajectory_validation_sequence=6
    assert core.on_validation(current)
    assert core.on_validation(negative)
    assert next(iter(core.proofs.values())).sequence==6
    result=queue.proof(sweep,now)
    assert not result.allowed and result.output.hold
    assert result.reason=='native_swept_collision_proof_missing_or_revoked'
    # A new controller decision made after the negative revision may recover;
    # delivering a positive snapshot cannot renew the old command itself.
    replacement=deepcopy(demand);replacement.sequence+=1;replacement.validation_sequence=6
    waiting=queue.demand(replacement,now)
    assert waiting.reason=='waiting_command_sweep'
    replacement_sweep=motion_proof(replacement)
    replacement_sweep.trajectory_validation_sequence=6
    assert queue.proof(replacement_sweep,now).allowed


@pytest.mark.parametrize('phase_ms',list(range(6,31)))
def test_floor_expiring_in_callback_queue_waits_for_actual_sweep_instead_of_consuming_command(phase_ms):
    core,queue,demand,current,sweep,now=phase_fixture(phase_ms)
    # Reproduce the production queue gap: the controller generated this exact
    # demand while floor4 was fresh, but gate delivery occurs after its 5ms
    # deadline. It remains an unapproved command until native finishes using5.
    queue.pending.clear()
    delayed=queue.demand(demand,now)
    assert delayed.reason=='waiting_command_sweep' and delayed.output is None
    assert core.last_demand==0 and len(queue.pending)==1
    core.on_validation(current)
    accepted=queue.proof(sweep,now+1_000_000)
    assert accepted.allowed and accepted.output.source_stamp==demand.source_stamp
    assert accepted.output.valid_until==demand.valid_until
    assert accepted.output.validation_sequence==4


def test_missing_sweep_wait_does_not_ignore_known_negative_or_original_command_expiry():
    core,queue,demand,current,sweep,now=phase_fixture()
    negative=deepcopy(current);negative.valid=False
    queue.pending.clear();assert core.on_validation(negative)
    denied=queue.demand(demand,now)
    assert denied.reason=='native_swept_collision_proof_missing_or_revoked'
    assert denied.output.hold and not queue.pending
    core,queue,demand,current,sweep,now=phase_fixture()
    expired=queue.demand(demand,NOW+101_000_000)
    assert expired.reason=='motion_demand_expired' and expired.output is None
    assert not queue.pending


def upgraded_fixture():
    core,permit,demand,curve,*_=fixture();queue=CommandAdmission(core)
    accepted=queue.demand(demand,NOW)
    assert accepted.allowed
    proof=motion_proof(demand);proof.sequence=2
    proof.check_end=proof.body_source_stamp=stamp(NOW+30_000_000)
    proof.valid_until=stamp(NOW+95_000_000)
    return core,queue,permit,demand,curve,proof


@pytest.mark.parametrize('phase_ms',[30,49,79,90])
def test_real_positive_upgrade_preserves_entire_original_control_lease(phase_ms):
    core,queue,permit,demand,curve,proof=upgraded_fixture()
    out=queue.proof(proof,NOW+phase_ms*1_000_000)
    assert out.allowed and out.output.motion_validation_sequence==2
    assert out.output.source_stamp==demand.source_stamp
    assert out.output.body_source_stamp==demand.body_source_stamp
    assert out.output.valid_until==demand.valid_until
    assert out.output.velocity==demand.velocity
    assert out.output.permit_sequence==permit.sequence
    assert not core.admit_evidence_upgrade(demand,NOW+phase_ms*1_000_000).allowed
    assert not core.admit(demand,NOW+phase_ms*1_000_000).allowed


@pytest.mark.parametrize('change',['expired','revoke','hold','phase','curve','negative',
                                  'late_negative','mutated_velocity','mutated_time','newer_command'])
def test_evidence_upgrade_cannot_revive_or_rewrite_a_command(change):
    core,queue,permit,demand,curve,proof=upgraded_fixture();now=NOW+40_000_000
    if change=='expired':now=NOW+101_000_000
    elif change in ('revoke','hold','phase','curve'):
        p=deepcopy(permit);p.sequence+=1
        if change=='revoke':p.revoked=True
        elif change=='hold':p.allowed=False
        elif change=='phase':p.phase='HOLDING'
        else:p.trajectory_id+=1
        core.on_permit(p)
    elif change in ('negative','late_negative'):
        bad=deepcopy(proof);bad.valid=False
        if change=='late_negative':
            positive=deepcopy(proof);positive.sequence=3
            assert core.on_motion_validation(positive)
        assert core.on_motion_validation(bad)
    elif change=='mutated_velocity':queue.recent.velocity.linear.x=.21
    elif change=='mutated_time':queue.recent.source_stamp=stamp(NOW+1)
    else:
        d=deepcopy(demand);d.sequence+=1
        assert core.on_motion_validation(motion_proof(d))
        assert queue.demand(d,NOW+1).allowed
    out=queue.proof(proof,now)
    assert not out.allowed and out.output is None
