from copy import deepcopy
from dataclasses import replace

import pytest
from nav_msgs.msg import Odometry
from d1max_planning_interfaces.msg import NavigationState
from d1max_navigation_bt_interfaces.msg import RouteReference

from d1max_pct_scan.continuous_reference_transport import (
    ContinuousReferenceTransport, ReferenceReceipt, nanoseconds, set_stamp,
    STAGING_REFERENCE_TOPIC, STAGING_NATIVE_PARAMETERS)
from d1max_pct_scan.source_route_ros import to_message
from test_continuous_reference import snapshot, BASE_NS


def state(*, t=0., x=0., correction=0., epoch=1, usable=True):
    message = NavigationState()
    message.schema_version, message.session_id, message.map_version_id = 2, 'session', 'map'
    message.localization_epoch, message.localization_seed_id = epoch, 'seed'
    message.usable = usable
    for name in ('source_stamp', 'posterior_stamp', 'imu_stamp'):
        set_stamp(getattr(message, name), BASE_NS+int(t*1e9))
    for name, frame, px in (('global_odometry', 'd1max_loc_map', x+correction),
                             ('local_odometry', 'd1max_loc_odom', x)):
        odom = Odometry()
        odom.header.frame_id, odom.child_frame_id = frame, 'd1max_loc_base_link'
        odom.header.stamp = deepcopy(message.source_stamp)
        odom.pose.pose.position.x, odom.pose.pose.position.z = float(px), .55
        odom.pose.pose.orientation.w = 1.
        setattr(message, name, odom)
    return message


def envelope(*, t=0., sequence=1, active=True, crossfloor=False):
    route = snapshot(crossfloor)
    result = RouteReference()
    result.schema_version, result.session_id, result.task_id = 2, 'session', 'task'
    result.route_id, result.route_hash = 'route', route.route_hash
    result.delivery_sequence, result.active = sequence, active
    set_stamp(result.source_stamp, BASE_NS+int(t*1e9))
    result.snapshot = to_message(route, session_id='session', task_id='task', route_id='route',
                                epoch=1, seed_id='seed', stamp=result.source_stamp)
    return result


def transport(**kwargs):
    output = []
    value = ContinuousReferenceTransport(session_id='session', map_version_id='map',
        expected_hashes=dict(source_map_sha256='a'*64, tomogram_sha256='c'*64, conditioning_sha256='b'*64),
        body_height_m=.55, body_height_calibration_id='preview-unverified', publish_staging=output.append,**kwargs)
    return value, output


@pytest.mark.parametrize('bad',[-1.,.49,4.01,float('nan'),True])
def test_reference_horizon_is_explicit_and_bounded(bad):
    with pytest.raises(ValueError):
        transport(reference_horizon_m=bad)


def test_two_metre_reference_window_does_not_implicitly_publish_four_metres():
    value,output=transport(reference_horizon_m=2.)
    route=snapshot(points=[[float(x),0.,0.] for x in range(5)])
    message=envelope()
    message.route_hash=route.route_hash
    message.snapshot=to_message(route,session_id='session',task_id='task',route_id='route',
        epoch=1,seed_id='seed',stamp=message.source_stamp)
    value.on_route(message,current_source_ns=BASE_NS,now_monotonic=100.)
    proposal=on_state(value)
    assert proposal.path.poses[-1].pose.position.x==2.
    assert acknowledge(value,proposal)
    assert not value.covers_current_segment_end()


def test_terminal_coverage_is_geometry_only_and_reanchoring_still_due():
    value,output=transport(reference_horizon_m=2.)
    on_route(value)
    proposal=on_state(value)
    assert not value.covers_current_segment_end()  # candidate is not current
    assert acknowledge(value,proposal)
    assert value.covers_current_segment_end()
    on_state(value,t=.1,correction=.12)
    assert value.correction_due()


def on_state(value, *, t=0., **kwargs):
    return value.on_navigation(state(t=t, **kwargs), received_monotonic=100.+t,
        current_source_ns=BASE_NS+int(t*1e9), now_monotonic=100.+t)


def on_route(value, *, t=0., **kwargs):
    return value.on_route(envelope(t=t, **kwargs), current_source_ns=BASE_NS+int(t*1e9), now_monotonic=100.+t)


def receipt(message, accepted=True):
    return ReferenceReceipt(**{key:getattr(message, key) for key in (
        'session_id','task_id','route_id','route_hash','generation','anchor_id','context_sequence','segment_id')},
        source_ns=nanoseconds(message.path.header.stamp), accepted=accepted)


def acknowledge(value, message, *, t=0., accepted=True):
    return value.on_reference_receipt(receipt(message, accepted),
        current_source_ns=BASE_NS+int(t*1e9), now_monotonic=100.+t)


def test_real_wire_callbacks_create_one_staged_odom_reference_not_an_owner_or_motion_permit():
    value, output = transport()
    on_route(value)
    assert not output
    message = on_state(value)
    assert len(output) == 1 and message.schema_version == 2
    assert message.path.header.frame_id == 'd1max_loc_odom'
    assert message.point_reference == 'body_center'
    assert STAGING_NATIVE_PARAMETERS['fsm.reference_path_z_offset'] == 0.
    assert message.path.poses[0].pose.position.z == pytest.approx(.55)
    assert message.anchor_id and message.anchor_revision == 1
    assert len(message.point_segment_ids) == len(message.path.poses)
    assert set(message.point_segment_ids) == {'floor'}
    assert value.accepted is None and not value.status['execution_eligible']
    assert STAGING_REFERENCE_TOPIC.endswith('/bt/staging/reference_path')
    assert acknowledge(value, message)
    assert value.accepted == message and not value.status['physical_stop_confirmed']


def test_navigation_first_and_soft_delivery_refresh_do_not_duplicate_native_reference():
    value, output = transport()
    on_state(value)
    message = on_route(value, t=.01)
    assert message and len(output) == 1
    on_route(value, t=.02, sequence=2)
    assert len(output) == 1 and value.core.anchor.revision == 1
    on_state(value, t=.1, x=.05)
    assert value.core.candidate_anchor is None  # ordinary body motion, same map <- odom


@pytest.mark.parametrize('invalid_half',['global_odometry','local_odometry'])
def test_pair_normalization_cannot_relax_original_raw_quaternion_admission(invalid_half):
    value,output=transport();on_route(value)
    original=on_state(value)
    assert acknowledge(value,original)
    pair,core,accepted,generation=value.latest_pair,value.core,value.accepted,value.generation
    published=len(output)
    invalid=state(t=.1,x=.05)
    # checked_pose would accept this small norm error, but the original
    # Rigid squared-norm admission limit is 1e-6 and must still reject it.
    getattr(invalid,invalid_half).pose.pose.orientation.w=1.0000006
    with pytest.raises(ValueError,match='unit quaternion required'):
        value.on_navigation(invalid,received_monotonic=100.1,
            current_source_ns=BASE_NS+100_000_000,now_monotonic=100.1)
    assert value.latest_pair is pair and value.core is core
    assert value.accepted is accepted and value.generation==generation
    assert value.pending is None and core.candidate_anchor is None
    assert len(output)==published


def test_map_corrections_and_motion_update_progress_not_reference_or_anchor():
    value, output = transport()
    on_route(value)
    original = on_state(value)
    assert acknowledge(value, original)
    for index in range(1, 9):
        on_state(value, t=index*.05, x=index*.02, correction=index*.01)
    assert len(output) == 1 and value.accepted == original
    assert value.core.anchor.anchor_id == original.anchor_id
    assert value.core.candidate_anchor.anchor_id != original.anchor_id
    assert value.status['confirmed_arc_m'] == pytest.approx(.16)
    assert value.accepted.route_hash == value.snapshot.route_hash


def test_pending_candidate_never_overwrites_accepted_without_matching_receipt():
    value, output = transport()
    on_route(value)
    original = on_state(value)
    assert acknowledge(value, original)
    on_state(value, t=.1, x=.05)
    new = value.propose_window(current_source_ns=BASE_NS+100_000_000, now_monotonic=100.1)
    assert new.generation == original.generation+1
    assert value.accepted == original
    assert not value.on_reference_receipt(replace(receipt(new), route_hash='f'*64),
        current_source_ns=BASE_NS+100_000_000, now_monotonic=100.1)
    assert value.pending == new and value.accepted == original
    assert acknowledge(value, new, t=.1, accepted=False)
    assert value.accepted == original and value.pending is None


@pytest.mark.parametrize('corruption', ['pair_time','posterior_stale','imu_stale','unusable','quaternion','wrong_map'])
def test_invalid_real_wire_does_not_forge_source_clock_or_reference(corruption):
    value, output = transport()
    on_route(value)
    msg = state()
    if corruption == 'pair_time': msg.global_odometry.header.stamp.nanosec = 1
    elif corruption == 'posterior_stale': msg.posterior_stamp.sec -= 1
    elif corruption == 'imu_stale': msg.imu_stamp.sec -= 1
    elif corruption == 'unusable': msg.usable = False
    elif corruption == 'quaternion': msg.local_odometry.pose.pose.orientation.w = .9
    else: msg.map_version_id = 'another-map'
    with pytest.raises(ValueError):
        value.on_navigation(msg, received_monotonic=100., current_source_ns=BASE_NS, now_monotonic=100.)
    assert not output and value.accepted is None and value.core is None


def test_duplicate_navigation_does_not_renew_old_measurement_receipt():
    value, _ = transport()
    on_route(value)
    candidate = on_state(value)
    value.on_navigation(state(), received_monotonic=100.4, current_source_ns=BASE_NS+400_000_000, now_monotonic=100.4)
    assert value.latest_pair[0].received_monotonic == 100.
    on_state(value, t=.6)  # current body is fresh, but candidate source remains old
    assert not acknowledge(value, candidate, t=.6)
    assert value.pending is None and value.accepted is None


def test_epoch_reset_requires_retirement_not_inplace_reanchor():
    value, output = transport()
    on_route(value)
    original = on_state(value)
    with pytest.raises(ValueError, match='context_reset'):
        on_state(value, t=.1, epoch=2)
    assert value.quarantined and len(output) == 1
    assert value.pending is None  # old prepared work is invalid after a hard reset
    assert value.core._progress is None
    with pytest.raises(ValueError, match='quarantined'):
        on_state(value, t=.2)
    cancel = on_route(value, t=.2, sequence=2, active=False)
    assert not cancel.path.poses and value.core is None
    assert value.quarantined  # cancellation does not authorize the new epoch


def test_cancel_stages_empty_reference_with_same_route_identity_never_stop_confirmation():
    value, output = transport()
    on_route(value)
    original = on_state(value)
    cancel = on_route(value, t=.1, sequence=2, active=False)
    assert len(output) == 2 and not cancel.path.poses
    assert cancel.route_hash == original.route_hash and cancel.task_id == original.task_id
    assert cancel.generation > original.generation and cancel.anchor_id == original.anchor_id
    assert value.core is None and value.pending is None and value.accepted is None
    assert not value.status['physical_stop_confirmed']
    assert not acknowledge(value, original, t=.1)
    assert cancel.point_reference == 'body_center'


def test_current_segment_metadata_cannot_label_entire_crossfloor_route_as_floor():
    value, output = transport()
    on_route(value, crossfloor=True)
    first = on_state(value)
    assert first.segment_id == 'lower_floor'
    assert set(first.point_segment_ids) == {'lower_floor'}
    assert first.path.poses[-1].pose.position.x == pytest.approx(1.)
    assert first.path.poses[-1].pose.position.z == pytest.approx(.55)
    assert first.required_mode == 'general'


def test_staging_profile_cannot_apply_body_height_twice():
    with pytest.raises(ValueError, match='staging_provenance'):
        ContinuousReferenceTransport(session_id='session', map_version_id='map',
            expected_hashes=dict(source_map_sha256='a'*64, tomogram_sha256='c'*64, conditioning_sha256='b'*64),
            body_height_m=.55, body_height_calibration_id='preview-unverified', publish_staging=lambda m: None,
            native_reference_path_z_offset=.55)


def test_adjacent_stair_transition_stages_new_mode_only_after_measured_stop_and_new_sample():
    value, output = transport()
    on_route(value, crossfloor=True)
    first = on_state(value)
    assert acknowledge(value, first)
    on_state(value, t=.8, x=.95)
    on_state(value, t=.9, x=.95)
    on_state(value, t=1., x=.95)
    assert value.request_segment_transition(current_source_ns=BASE_NS+1_000_000_000,
        now_monotonic=101.) == 'stair_lower'
    assert len(output) == 1 and value.accepted == first
    stair = on_state(value, t=1.1, x=1.)
    assert len(output) == 2
    assert stair.required_mode == 'stair' and stair.segment_id == 'stair_lower'
    assert set(stair.point_segment_ids) == {'stair_lower'}
    assert stair.route_hash == first.route_hash and stair.anchor_id == first.anchor_id
    assert value.accepted == first and value.pending == stair
    assert acknowledge(value, stair, t=1.1)
    assert value.accepted == stair
