from copy import deepcopy
import math
from types import SimpleNamespace as N
import numpy as np
import pytest
from scipy.interpolate import BSpline
from d1max_pct_scan.execution_view import CommittedView,curve_key,remaining_spline_style


def at(ns):return N(sec=ns//10**9,nanosec=ns%10**9)


def fixture():
    v=N(schema_version=3,session_id='s',task_id='t',route_id='r',route_hash='hash',map_version_id='map',
        localization_epoch=1,localization_seed_id='seed',reference_generation=1,segment_id='seg',
        anchor_id='a',anchor_revision=1,context_sequence=1,map_geometry_revision=1)
    p=N(version=v,transport_mode='isolated_mock',sequence=1,trajectory_id=2,geometry_committed=True,
        revoked=False,allowed=False,frame_id='d1max_loc_odom',source_stamp=at(1000000000),valid_until=at(1100000000))
    c=N(**{k:x for k,x in vars(v).items() if k not in ('reference_generation','schema_version')},
        schema_version=2,generation=1,trajectory=N(traj_id=2,order=3,
            knots=[i*.2-.6 for i in range(11)],
            pos_pts=[N(x=i*.2,y=.03*i*i,z=.5) for i in range(7)]),frame_id='d1max_loc_odom')
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


def test_mainline_committed_shape_survives_temporary_heartbeat_gap_without_motion_claim():
    old,p,c,proof=fixture()
    core=CommittedView('s','isolated_mock',durable_geometry=True)
    core.permit_received(p);core.curve_received(c);core.proof_received(proof)
    selected=core.selected(2000000000)
    assert selected is not None and selected[2]==0.
    assert not p.allowed
    p.sequence+=1;p.revoked=True;core.permit_received(p)
    assert core.selected(2000000000) is None


def observed(curve,*,source=1_000_000_000,time=.23):
    return N(**{name:deepcopy(getattr(curve,name)) for name in (
        'schema_version','session_id','task_id','route_id','route_hash','map_version_id',
        'localization_epoch','localization_seed_id','generation','segment_id',
        'anchor_id','anchor_revision','context_sequence')},
        trajectory_id=curve.trajectory.traj_id,header=N(frame_id=curve.frame_id,stamp=at(source)),
        curve_time=time,arc_length=time,s_committed=time,valid=True,holding=False)


def mainline():
    _,p,c,proof=fixture()
    core=CommittedView('s','isolated_mock',durable_geometry=True)
    core.permit_received(p);core.curve_received(c);core.proof_received(proof)
    return core,p,c,proof


def test_formal_remainder_begins_exactly_at_actual_progress_not_old_origin_or_body_pose():
    core,_,curve,_=mainline();raw=curve.trajectory;before=deepcopy(raw)
    progress=observed(curve,time=.237);core.progress_received(progress,1_050_000_000)
    style=core.display_style(curve,1_050_000_000)
    points=[[p.x,p.y,p.z] for p in raw.pos_pts]
    spline=BSpline(raw.knots,points,3,extrapolate=False)
    np.testing.assert_allclose(style['points'][0],spline(raw.knots[3]+progress.curve_time))
    assert style['times'][0]==progress.curve_time
    assert np.all(style['times']>=progress.curve_time)
    assert np.linalg.norm(style['points'][0]-spline(raw.knots[3]))>.1
    assert raw==before and core.progress[(curve_key(curve,tagged=True),raw.traj_id)].header.stamp==progress.header.stamp


def test_trim_preserves_native_remaining_samples_and_whole_curve_colour_scale():
    from d1max_pct_scan.scan_visual_style import official_spline_style
    _,_,curve,_=mainline();raw=curve.trajectory
    args=dict(order=raw.order,knots=raw.knots,points=[[p.x,p.y,p.z] for p in raw.pos_pts])
    original=official_spline_style(**args);part=remaining_spline_style(**args,measured_time=.237)
    keep=original['times']>.237
    for name in ('points','colors','speeds','times'):
        np.testing.assert_array_equal(part[name][1:],original[name][keep])
    assert part['times'][0]==.237


def test_progress_gap_hold_and_repeated_receipt_freeze_last_measured_trim_without_advancing():
    core,_,curve,_=mainline();progress=observed(curve)
    core.progress_received(progress,1_050_000_000)
    original=core.display_style(curve,1_050_000_000)
    progress.curve_time=.6 # Repeated source is not a new measured state.
    core.progress_received(progress,1_100_000_000)
    invalid=observed(curve,source=1_100_000_000,time=.7);invalid.valid=False;invalid.holding=True
    core.progress_received(invalid,1_100_000_000)
    held=core.display_style(curve,10_000_000_000)
    np.testing.assert_array_equal(held['points'],original['points'])
    assert next(iter(core.progress.values())).header.stamp==at(1_000_000_000)


def test_new_source_can_report_real_small_backtracking_without_source_regression():
    core,_,curve,_=mainline()
    core.progress_received(observed(curve,time=.3),1_050_000_000)
    core.progress_received(observed(curve,source=1_100_000_000,time=.28),1_110_000_000)
    core.progress_received(observed(curve,source=1_050_000_000,time=.6),1_110_000_000)
    assert core.display_style(curve,1_110_000_000)['times'][0]==.28


@pytest.mark.parametrize('name',[
    'session_id','task_id','route_id','route_hash','map_version_id','localization_epoch',
    'localization_seed_id','generation','segment_id','anchor_id','anchor_revision',
    'context_sequence','trajectory_id',
])
def test_all_tracking_progress_identity_fields_must_match_committed_curve(name):
    core,_,curve,_=mainline();progress=observed(curve);value=getattr(progress,name)
    setattr(progress,name,value+1 if isinstance(value,int) else value+'_other')
    core.progress_received(progress,1_050_000_000)
    assert core.display_style(curve,1_050_000_000) is None


@pytest.mark.parametrize('change',[
    {'schema_version':1},{'valid':False},{'curve_time':math.nan},{'curve_time':math.inf},
    {'curve_time':-.01},{'curve_time':2.},{'arc_length':math.nan},{'s_committed':-1.},
])
def test_invalid_progress_does_not_fabricate_a_trim(change):
    core,_,curve,_=mainline();progress=observed(curve)
    for name,value in change.items():setattr(progress,name,value)
    core.progress_received(progress,1_050_000_000)
    assert core.display_style(curve,1_050_000_000) is None


@pytest.mark.parametrize('source',[0,500_000_000,1_071_000_000])
def test_first_progress_must_have_original_recent_nonfuture_source(source):
    core,_,curve,_=mainline();core.progress_received(observed(curve,source=source),1_050_000_000)
    assert core.display_style(curve,1_050_000_000) is None


def test_progress_in_other_frame_is_not_drawn_on_odom_curve():
    core,_,curve,_=mainline();progress=observed(curve);progress.header.frame_id='map'
    core.progress_received(progress,1_050_000_000)
    assert core.display_style(curve,1_050_000_000) is None


def test_progress_without_committed_geometry_and_future_candidate_cannot_replace_current_trim():
    core,permit,curve,_=mainline()
    core.progress_received(observed(curve),1_050_000_000)
    new=deepcopy(curve);new.trajectory.traj_id+=1
    core.progress_received(observed(new,time=.5),1_050_000_000) # no committed geometry
    core.curve_received(new)
    assert core.display_style(new,1_050_000_000) is None
    assert core.selected(1_050_000_000)[0].trajectory.traj_id==curve.trajectory.traj_id
    core.progress_received(observed(new,source=1_100_000_000,time=.5),1_110_000_000)
    assert core.selected(1_110_000_000)[0].trajectory.traj_id==curve.trajectory.traj_id
    permit.trajectory_id=new.trajectory.traj_id;permit.sequence+=1;core.permit_received(permit)
    assert core.selected(1_110_000_000)[0].trajectory.traj_id==new.trajectory.traj_id
    assert core.display_style(new,1_110_000_000)['times'][0]==.5


def test_missing_map_geometry_field_never_allows_progress_to_cross_ambiguous_revision():
    core,_,curve,_=mainline();core.progress_received(observed(curve),1_050_000_000)
    changed=deepcopy(curve);changed.map_geometry_revision+=1;core.curve_received(changed)
    core.progress_received(observed(changed,source=1_100_000_000,time=.6),1_110_000_000)
    assert core.display_style(changed,1_110_000_000) is None
    assert core.display_style(curve,1_110_000_000)['times'][0]==.23


def test_legacy_preview_retains_full_curve_and_original_permit_lease_without_progress():
    core,_,curve,_=fixture()
    assert core.display_style(curve,1_050_000_000)['times'][0]==0.
    assert core.selected(1_350_000_000) is None


def test_real_endpoint_renders_no_fake_continuation():
    core,_,curve,_=mainline();raw=curve.trajectory
    duration=raw.knots[len(raw.pos_pts)]-raw.knots[raw.order]
    core.progress_received(observed(curve,time=duration),1_050_000_000)
    style=core.display_style(curve,1_050_000_000)
    assert len(style['points'])==2
    np.testing.assert_array_equal(style['points'][0],style['points'][1])
    np.testing.assert_array_equal(style['times'],[duration,duration])


def test_progress_cache_stays_bounded_for_many_committed_replacements():
    core,_,curve,_=mainline()
    for identifier in range(2,25):
        curve.trajectory.traj_id=identifier;core.curve_received(curve)
        core.progress_received(observed(curve),1_050_000_000)
    assert len(core.curves)==len(core.progress)==8
