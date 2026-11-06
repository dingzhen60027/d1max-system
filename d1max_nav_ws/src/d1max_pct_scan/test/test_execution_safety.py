from copy import deepcopy
from types import SimpleNamespace as N
import pytest
from d1max_pct_scan.execution_safety import ExecutionSafety

NOW=10_000_000_000
def stamp(ns=NOW): return N(sec=ns//10**9,nanosec=ns%10**9)
def fixture(kinematic_profile='forward_yaw'):
    v=N(schema_version=3,session_id='s',task_id='task',route_id='route',route_hash='a'*64,
        map_version_id='map',localization_epoch=1,localization_seed_id='seed',reference_generation=1,
        segment_id='floor1',anchor_id='anchor',anchor_revision=1,context_sequence=1,map_geometry_revision=1)
    common=dict(version=v,execution_id='execution',control_epoch=1,sdk_session='sdk',sdk_arm_generation=1,
        sequence=1,source_stamp=stamp(),valid_until=stamp(NOW+200_000_000),trajectory_id=2,
        validation_sequence=3,transport_mode='isolated_mock')
    permit=N(**common,allowed=True,revoked=False,geometry_committed=True,phase='TRACKING')
    demand=N(**common,permit_sequence=1,body_source_stamp=stamp(),hold=False,safety_checked=False,
        safety_source_stamp=stamp(0),reason='',velocity=N(linear=N(x=.2,y=0.,z=0.),angular=N(x=0.,y=0.,z=.1)))
    proof=N(version=v,trajectory_id=2,sequence=3,source_stamp=stamp(),check_begin=stamp(NOW-10_000_000),
        check_end=stamp(),valid_until=stamp(NOW+100_000_000),body_source_stamp=stamp(),front_ray_source_stamp=stamp(NOW-100_000_000),
        rear_ray_source_stamp=stamp(NOW-90_000_000),valid=True,whole_curve=True,
        remaining_curve=False,checked_from_time=0.,checked_to_time=5.,curve_duration=5.,
        valid_start_time=0.,reverse_margin_m=0.,
        frame_id='d1max_loc_odom',collision_policy='official_inflated_geometry',transport_mode='isolated_mock',
        support_reference_id='support',support_hash='b'*64)
    stationary_policy=dict(profile='general_low_speed',linear_threshold_mps=.03,
        angular_threshold_radps=.05,stationary_duration_s=1.,reentry_duration_s=.6,
        minimum_new_samples=3,mc_expected_hz=50.,mc_min_hz=10.,mc_max_hz=200.,
        measured_static_linear_bound_mps=.02,measured_static_angular_bound_radps=.03)
    core=ExecutionSafety('s','isolated_mock',braking_model_sha256='c'*64,
        stationary_policy=stationary_policy,kinematic_profile=kinematic_profile)
    assert core.on_permit(permit)
    assert core.on_validation(proof)
    assert core.on_motion_validation(motion_proof(demand))
    # Keep the fixture's existing tuple shape; no second sensor observer is
    # needed by ordinary commands or by prepared handoff commands.
    return core,permit,demand,proof,None,None


def motion_proof(demand):
    return N(**{k:deepcopy(getattr(demand,k)) for k in ('version','execution_id','control_epoch',
        'sdk_session','sdk_arm_generation','trajectory_id','permit_sequence','velocity','transport_mode')},
        sequence=1,demand_sequence=demand.sequence,trajectory_validation_sequence=demand.validation_sequence,
        demand_source_stamp=deepcopy(demand.source_stamp),demand_body_source_stamp=deepcopy(demand.body_source_stamp),
        demand_valid_until=deepcopy(demand.valid_until),body_source_stamp=stamp(),
        front_ray_source_stamp=stamp(NOW-100_000_000),rear_ray_source_stamp=stamp(NOW-90_000_000),
        check_begin=stamp(NOW-10_000_000),check_end=stamp(),valid_until=stamp(NOW+80_000_000),
        braking_model_sha256='c'*64,frame_id='d1max_loc_odom',map_snapshot_revision=1,valid=True,reason='observed_free')

def test_scan_decision_is_bound_without_sensor_delivery_restamping_or_mutating_input():
    core,_,d,_,_,_=fixture(); original=deepcopy(d)
    result=core.admit(d,NOW)
    assert result.allowed and result.output.safety_checked
    assert result.output.safety_source_stamp==stamp(NOW-100_000_000)
    assert result.output.source_stamp==d.source_stamp and d==original
    assert not core.admit(d,NOW).allowed

@pytest.mark.parametrize('field,value', [('valid',False),('whole_curve',False),
    ('collision_policy','official'),('frame_id','d1max_loc_map'),('support_hash',''),
    ('valid_until',stamp(NOW-1))])
def test_scan_negative_expired_or_foreign_trajectory_decision_never_authorizes(field,value):
    c,p,d,proof,_,_=fixture(); setattr(proof,field,value); proof.sequence+=1
    p.validation_sequence=d.validation_sequence=proof.sequence; p.sequence+=1; d.permit_sequence=p.sequence
    c.on_validation(proof); c.on_permit(p)
    command=motion_proof(d);command.sequence+=1;c.on_motion_validation(command)
    assert not c.admit(d,NOW).allowed

def test_new_collision_rejection_retires_old_proof():
    c,_,d,proof,_,_=fixture(); proof=deepcopy(proof); proof.sequence+=1; proof.valid=False
    c.on_validation(proof)
    assert not c.admit(d,NOW).allowed
    stop=c.stop_for_rejection(d,NOW,'collision')
    assert stop.allowed and stop.output.hold and not stop.output.safety_checked
    assert stop.output.velocity.linear.x==stop.output.velocity.angular.z==0

def test_stop_is_not_new_authorization_or_extended_lease():
    c,p,d,_,_,_=fixture(); p=deepcopy(p); p.sequence+=1; p.revoked=True; c.on_permit(p)
    assert not c.stop_for_rejection(d,NOW,'revoked').allowed
    c,_,d,_,_,_=fixture()
    assert not c.stop_for_rejection(d,NOW+300_000_000,'stale').allowed

@pytest.mark.parametrize('field', ['task_id','route_hash','anchor_id','localization_seed_id','map_version_id'])
def test_foreign_or_uncommitted_execution_never_sends(field):
    c,_,d,_,_,_=fixture(); d=deepcopy(d); setattr(d.version,field,'foreign')
    assert not c.admit(d,NOW).allowed and not c.stop_for_rejection(d,NOW,'mismatch').allowed

@pytest.mark.parametrize('component,axis,value', [('linear','x',-.01),('linear','x',.31),
    ('linear','y',.01),('linear','z',.01),('angular','z',.51),('angular','z',float('nan'))])
def test_forward_yaw_only_and_physical_limits(component,axis,value):
    c,_,d,_,_,_=fixture(); setattr(getattr(d.velocity,component),axis,value)
    assert not c.admit(d,NOW).allowed


@pytest.mark.parametrize('vx,vy,wz',[(0.,.2,0.),(-.15,.1,.2),(.2,.2,-.2)])
def test_holonomic_exact_three_axis_demand_passes_unchanged(vx,vy,wz):
    c,_,d,_,_,_=fixture('holonomic_xyyaw')
    d.sequence+=1;d.velocity.linear.x=vx;d.velocity.linear.y=vy;d.velocity.angular.z=wz
    proof=motion_proof(d);proof.sequence+=1
    assert c.on_motion_validation(proof)
    original=deepcopy(d)
    result=c.admit(d,NOW)
    assert result.allowed and result.output.velocity==original.velocity
    assert result.output.source_stamp==original.source_stamp and d==original


def test_diagonal_speed_is_capped_by_norm_not_per_axis():
    c,_,d,_,_,_=fixture('holonomic_xyyaw')
    d.sequence+=1;d.velocity.linear.x=d.velocity.linear.y=.25
    proof=motion_proof(d);proof.sequence+=1;c.on_motion_validation(proof)
    assert not c.admit(d,NOW).allowed


def test_lateral_command_still_requires_exact_fresh_scan_decision():
    c,_,d,_,_,_=fixture('holonomic_xyyaw')
    d.velocity.linear.x=0.;d.velocity.linear.y=.2
    assert not c.admit(d,NOW).allowed  # old proof bound to different velocity
    proof=motion_proof(d);proof.sequence+=1;proof.valid_until=stamp(NOW-1)
    c.on_motion_validation(proof)
    assert not c.admit(d,NOW).allowed

def test_native_decision_delivery_never_renews_original_control_deadline():
    c,_,d,proof,_,_=fixture()
    proof=deepcopy(proof);proof.sequence+=1;proof.valid_until=stamp(NOW+2*10**9)
    c.on_validation(proof)
    assert not c.admit(d,NOW+10**9).allowed


@pytest.mark.parametrize('kind',['trajectory','motion'])
@pytest.mark.parametrize('field',['body_source_stamp','front_ray_source_stamp','rear_ray_source_stamp'])
def test_scan_owns_sensor_source_admission_without_downstream_second_observer(kind,field):
    c,p,d,proof,_,_=fixture()
    c.sensor_max_age_ns=1_000_000
    if kind=='trajectory':
        proof=deepcopy(proof);proof.sequence+=1
        setattr(proof,field,stamp(NOW-700_000_000));c.on_validation(proof)
        command=motion_proof(d);command.sequence+=1
        command.trajectory_validation_sequence=proof.sequence
    else:
        command=motion_proof(d);command.sequence+=1
        setattr(command,field,stamp(NOW-700_000_000))
    assert c.on_motion_validation(command)
    result=c.admit(d,NOW)
    assert result.allowed and result.output.safety_checked
    assert result.output.source_stamp==d.source_stamp
    assert result.output.valid_until==d.valid_until
    assert not hasattr(c,'raw') and not hasattr(c,'on_rays')


def test_legacy_bound_sensor_parameter_still_rejects_invalid_configuration():
    for invalid in (True,0.,float('nan'),.7,1.1):
        with pytest.raises(ValueError):
            ExecutionSafety('s','isolated_mock',braking_model_sha256='c'*64,sensor_source_age_s=invalid)

def test_bounded_caches():
    c,p,_,proof,_,_=fixture()
    for i in range(2,40):
        p=deepcopy(p); p.sequence=i; c.on_permit(p)
        proof=deepcopy(proof); proof.trajectory_id=i; c.on_validation(proof)
    assert len(c.permits)==8 and len(c.proofs)==8


def test_newer_positive_proof_does_not_invalidate_unexpired_commit():
    c,_,d,proof,_,_=fixture()
    proof=deepcopy(proof);proof.sequence+=1;c.on_validation(proof)
    assert c.admit(d,NOW).allowed


def renewed_fixture():
    c,p,d,proof,_,_=fixture(); now=NOW+120_000_000
    d=deepcopy(d);d.sequence=2;d.validation_sequence=4
    d.source_stamp=d.body_source_stamp=stamp(now)
    newer=deepcopy(proof);newer.sequence=4
    newer.check_begin=stamp(now-1_000_000);newer.check_end=newer.body_source_stamp=stamp(now)
    newer.valid_until=stamp(NOW+190_000_000);c.on_validation(newer)
    command=motion_proof(d);command.check_begin=stamp(now-1_000_000)
    command.check_end=command.body_source_stamp=stamp(now);command.valid_until=stamp(NOW+180_000_000)
    c.on_motion_validation(command)
    return c,p,d,newer,now


def test_authority_lease_accepts_fresh_exact_proof_above_floor_when_old_proof_expired():
    c,p,d,proof,now=renewed_fixture()
    result=c.admit(d,now)
    assert result.allowed and result.output.validation_sequence==4
    assert p.validation_sequence==3 and result.output.permit_sequence==p.sequence
    assert result.output.valid_until==d.valid_until  # No deadline restamping.


def test_newer_proof_does_not_extend_authority_lease():
    c,p,d,proof,now=renewed_fixture()
    assert not c.admit(d,NOW+201_000_000).allowed


def test_new_collision_fences_proof_even_above_owner_floor():
    c,p,d,proof,now=renewed_fixture()
    negative=deepcopy(proof);negative.sequence=5;negative.valid=False
    c.on_validation(negative)
    assert not c.admit(d,now).allowed


def test_proof_below_new_authority_floor_is_rejected():
    c,p,d,proof,now=renewed_fixture()
    p=deepcopy(p);p.sequence=2;p.validation_sequence=5;c.on_permit(p)
    d.permit_sequence=2
    assert not c.admit(d,now).allowed


def test_newer_same_curve_permission_keeps_exact_unexpired_old_permission():
    c,p,d,_,_,_=fixture()
    p=deepcopy(p);p.sequence+=1;c.on_permit(p)
    assert c.admit(d,NOW).allowed


@pytest.mark.parametrize('field,value',[('trajectory_id',99),('phase','ALIGNING')])
def test_new_curve_or_phase_fences_old_permission(field,value):
    c,p,d,_,_,_=fixture()
    p=deepcopy(p);p.sequence+=1;setattr(p,field,value);c.on_permit(p)
    assert not c.admit(d,NOW).allowed


def test_negative_proof_is_fence_even_if_later_environment_becomes_clear():
    c,_,d,proof,_,_=fixture()
    proof=deepcopy(proof);proof.sequence+=1;proof.valid=False;c.on_validation(proof)
    proof=deepcopy(proof);proof.sequence+=1;proof.valid=True;c.on_validation(proof)
    assert not c.admit(d,NOW).allowed


def suffix_fixture():
    c,p,d,proof,_,_=fixture()
    proof.whole_curve=False;proof.remaining_curve=True
    proof.checked_from_time=1.;proof.valid_start_time=2.;proof.reverse_margin_m=.15;proof.sequence+=1
    p.sequence+=1;p.validation_sequence=d.validation_sequence=proof.sequence;d.permit_sequence=p.sequence
    c.on_validation(proof);c.on_permit(p)
    command=motion_proof(d);command.sequence=2;c.on_motion_validation(command)
    return c,p,d,proof


def test_suffix_uses_native_measured_domain_without_second_progress_observer():
    c,_,d,_=suffix_fixture()
    result=c.admit(d,NOW)
    assert result.allowed and result.output.safety_checked
    assert not hasattr(c,'progress') and not hasattr(c,'on_progress')


@pytest.mark.parametrize('field,value',[('checked_to_time',4.),('checked_from_time',3.),
    ('reverse_margin_m',.14),('curve_duration',float('nan')),('remaining_curve',False)])
def test_bad_suffix_is_not_whole_curve_proof(field,value):
    c,p,d,proof=suffix_fixture();setattr(proof,field,value);proof.sequence+=1
    p.validation_sequence=d.validation_sequence=proof.sequence;p.sequence+=1;d.permit_sequence=p.sequence
    c.on_validation(proof);c.on_permit(p)
    assert not c.admit(d,NOW).allowed


def test_suffix_still_requires_committed_owner_geometry():
    c,p,d,_=suffix_fixture()
    p.geometry_committed=False;p.sequence+=1;d.permit_sequence=p.sequence;c.on_permit(p)
    assert not c.admit(d,NOW).allowed


@pytest.mark.parametrize('change',['missing','negative','expired','foreign','source','velocity'])
def test_suffix_geometry_alone_cannot_replace_exact_native_command_verdict(change):
    c,_,d,_=suffix_fixture()
    c.motion_proofs.clear()
    command=motion_proof(d);command.sequence=3
    if change=='negative':command.valid=False;command.reason='motion_sweep_occupied'
    elif change=='expired':command.valid_until=stamp(NOW)
    elif change=='foreign':command.version=deepcopy(command.version);command.version.anchor_revision+=1
    elif change=='source':command.demand_body_source_stamp=stamp(NOW-1)
    elif change=='velocity':command.velocity.linear.x+=.01
    if change!='missing':assert c.on_motion_validation(command)
    assert not c.admit(d,NOW).allowed


def test_suffix_native_verdict_survives_progress_delivery_gap_without_renewing_original_sources():
    c,p,d,proof=suffix_fixture()
    c.command_pipeline_bound_ns=200_000_000
    now=NOW+120_000_000
    proof=deepcopy(proof);proof.sequence+=1
    proof.source_stamp=proof.check_end=stamp(now);proof.check_begin=stamp(now-1_000_000)
    proof.valid_until=stamp(NOW+180_000_000)
    assert c.on_validation(proof)
    command=motion_proof(d);command.sequence=3;command.trajectory_validation_sequence=proof.sequence
    command.check_end=stamp(now);command.check_begin=stamp(now-500_000)
    command.valid_until=stamp(NOW+170_000_000)
    assert c.on_motion_validation(command)
    original=deepcopy(d)
    result=c.admit(d,now)
    assert result.allowed
    assert d==original and result.output.source_stamp==original.source_stamp
    assert result.output.body_source_stamp==original.body_source_stamp
    assert result.output.valid_until==original.valid_until
    assert not c.admit_evidence_upgrade(d,NOW+171_000_000).allowed


def test_new_negative_curve_verdict_fences_suffix_despite_previous_positive_motion():
    c,_,d,proof=suffix_fixture()
    negative=deepcopy(proof);negative.sequence+=1;negative.valid=False
    assert c.on_validation(negative)
    assert not c.admit(d,NOW).allowed
