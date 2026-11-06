"""Independent production odom contracts; no ROS context, nodes or SDK."""
from copy import deepcopy

import pytest
from nav_msgs.msg import Odometry
from d1max_planning_interfaces.msg import LocalNavigationState
from d1max_localization.estimation_ros import stamp_time
from d1max_localization.navigation_wire import local_body_message
from d1max_pct_scan.local_navigation_state import (
    LocalNavigationInbox, LocalNavigationHistoryInbox, history_identity)
from d1max_pct_scan.ray_projection import RayProjectorCore, AwaitingCoverage
from test_ray_projection import CTX, EPOCH, IDENTITY, decode, feed, raw_points


def packet(source_ns=10_000_000_000, *, epoch=1, seed='seed', usable=True,
           reason='local_navigation_unavailable'):
    local = Odometry()
    local.header.frame_id, local.child_frame_id = 'd1max_loc_odom', 'd1max_loc_base_link'
    local.header.stamp.sec, local.header.stamp.nanosec = divmod(source_ns, 10**9)
    local.pose.pose.orientation.w = 1.
    imu = (source_ns-20_000_000)*1e-9
    message = local_body_message(local=local, session_id='session', map_version_id='map',
        epoch=epoch, seed_id=seed, posterior_stamp=(source_ns-40_000_000)*1e-9,
        imu_stamp=imu, extrapolation_sec=(source_ns-round(imu*1e9))*1e-9)
    if not usable:
        message.usable, message.reason = False, reason
        # A real unavailable event need not retain any expired geometry.
        message.local_odometry = Odometry()
        message.posterior_stamp, message.imu_stamp = stamp_time(0.), stamp_time(0.)
    return message


def accept(inbox, message, *, now_ns=None, mono=100.):
    source = message.source_stamp.sec*10**9+message.source_stamp.nanosec
    return inbox.accept(message, session_id='session', map_version_id='map',
        now_ns=source if now_ns is None else now_ns, monotonic=mono)


def test_local_wire_roundtrips_real_source_without_a_global_member_or_restamp():
    from rclpy.serialization import serialize_message, deserialize_message
    source = EPOCH+160_000_137
    message = packet(source)
    decoded = deserialize_message(serialize_message(message), LocalNavigationState)
    assert decoded.source_stamp == decoded.local_odometry.header.stamp
    assert decoded.source_stamp.sec*10**9+decoded.source_stamp.nanosec == source
    assert not hasattr(decoded, 'global_odometry')
    inbox = LocalNavigationInbox()
    assert accept(inbox, decoded, now_ns=source+10_000_000)
    assert inbox.latest.state.source_ns == source
    assert inbox.latest.state.posterior_ns < source
    assert inbox.usable(now_ns=source+10_000_000, monotonic=100.01, receipt_timeout_s=.4)


def test_soft_global_unavailability_cannot_enter_or_renew_the_independent_local_inbox():
    from d1max_pct_scan.atomic_navigation_inbox import AtomicNavigationInbox
    from test_continuous_reference_transport import state
    source = EPOCH+160_000_000
    local = LocalNavigationInbox()
    assert accept(local, packet(source))
    measured = local.latest
    global_pair = AtomicNavigationInbox()
    global_notice = state(usable=False)
    global_notice.reason = 'filter_stale'
    # Same source instant/session/map: rejection below must be because the
    # global wire has different ownership, not because its clock is far away.
    global_notice.source_stamp = deepcopy(packet(source).source_stamp)
    global_source = global_notice.source_stamp.sec*10**9+global_notice.source_stamp.nanosec
    assert global_pair.accept(global_notice, session_id='session', map_version_id='map',
        now_ns=global_source, monotonic=100.)
    assert not global_pair.usable(now_ns=global_source, monotonic=100., receipt_timeout_s=.4)
    assert accept(local, global_notice, now_ns=global_source) is None  # wrong wire schema
    assert local.latest is measured
    assert local.usable(now_ns=source+50_000_000, monotonic=100.05, receipt_timeout_s=.4)


@pytest.mark.parametrize('reason', ['hard_localization_lost', 'lio_reset', 'localization_reset'])
def test_hard_reset_discards_history_and_queued_good_packet_cannot_revive(reason):
    inbox = LocalNavigationInbox()
    old = packet()
    assert accept(inbox, old)
    revoked = accept(inbox, packet(10_020_000_000, usable=False, reason=reason), mono=100.02)
    assert revoked.hard_failure
    assert history_identity(inbox) is None
    assert not inbox.usable(now_ns=10_020_000_000, monotonic=100.02, receipt_timeout_s=.4)
    assert accept(inbox, old, now_ns=10_020_000_000, mono=100.03) is None
    assert inbox.latest is revoked


@pytest.mark.parametrize('change', ['epoch', 'seed'])
def test_estimator_identity_change_is_explicit_not_an_old_task_recovery(change):
    inbox = LocalNavigationInbox()
    assert accept(inbox, packet())
    original = history_identity(inbox)
    newer = packet(10_020_000_000, epoch=2 if change == 'epoch' else 1,
                   seed='new-seed' if change == 'seed' else 'seed')
    event = accept(inbox, newer, mono=100.02)
    assert event.identity != original
    assert history_identity(inbox) == event.identity
    # History consumers must compare this identity; no old context is renamed.
    assert event.identity != ('session', 1, 'seed')
    if change == 'epoch':
        assert accept(inbox, packet(10_040_000_000), mono=100.04) is None
        assert inbox.latest is event


def test_fresh_new_epoch_with_earlier_source_is_not_hidden_by_old_epoch_watermark():
    inbox=LocalNavigationInbox();assert accept(inbox,packet())
    reset=packet(9_980_000_000,epoch=2,seed='reset')
    event=accept(inbox,reset,now_ns=10_020_000_000,mono=100.02)
    assert event and event.identity==('session',2,'reset')
    assert accept(inbox,packet(10_030_000_000),mono=100.03) is None
    assert inbox.latest is event


@pytest.mark.parametrize('source', [10_000_000_000, 9_990_000_000, 10_010_000_000])
def test_duplicate_or_reordered_source_cannot_restore_soft_unavailable(source):
    inbox = LocalNavigationInbox()
    assert accept(inbox, packet())
    unavailable = accept(inbox, packet(10_010_000_000, usable=False), mono=100.01)
    assert unavailable.state is None and not unavailable.hard_failure
    assert accept(inbox, packet(source), now_ns=10_020_000_000, mono=100.6) is None
    assert inbox.latest is unavailable
    assert not inbox.usable(now_ns=10_020_000_000, monotonic=100.6, receipt_timeout_s=.4)
    # A genuinely newer measurement is admissible; restoring execution still
    # requires the BT's separate stable source-time gate.
    assert accept(inbox, packet(10_030_000_000), mono=100.61)


@pytest.mark.parametrize('bad', ['frame', 'time', 'nan_pose', 'nan_twist', 'quaternion', 'imu', 'posterior'])
def test_invalid_local_packet_does_not_advance_measurement_or_receipt_watermarks(bad):
    inbox = LocalNavigationInbox()
    assert accept(inbox, packet())
    prior = inbox.latest
    incoming = packet(10_020_000_000)
    if bad == 'frame': incoming.local_odometry.header.frame_id = 'map'
    elif bad == 'time':
        incoming.local_odometry.header.stamp = deepcopy(incoming.local_odometry.header.stamp)
        incoming.local_odometry.header.stamp.nanosec += 1
    elif bad == 'nan_pose': incoming.local_odometry.pose.pose.position.x = float('nan')
    elif bad == 'nan_twist': incoming.local_odometry.twist.twist.linear.y = float('nan')
    elif bad == 'quaternion': incoming.local_odometry.pose.pose.orientation.w = 0.
    elif bad == 'imu': incoming.imu_stamp = stamp_time(10.03)
    else: incoming.posterior_stamp = stamp_time(10.03)
    assert accept(inbox, incoming, mono=100.02) is None
    assert inbox.latest is prior
    assert not inbox.usable(now_ns=10_101_000_000, monotonic=100.101, receipt_timeout_s=.4)


def test_soft_expired_history_projects_covered_old_rays_but_never_authorizes_newer_scan():
    source = EPOCH+160_000_000
    inbox = LocalNavigationInbox()
    assert accept(inbox, packet(source))
    assert not inbox.usable(now_ns=EPOCH+300_000_000, monotonic=100.14, receipt_timeout_s=.4)
    assert history_identity(inbox) == ('session', 1, 'seed')
    value = RayProjectorCore(projection_frame='odom')
    value.reset(CTX)
    value.set_extrinsics(body_to_tracking=IDENTITY, ray_to_tracking=IDENTITY)
    feed(value, count=15)  # geometry coverage exists, but is not source authorization
    old = decode(raw_points())
    covered = value.project(old, CTX, now_ns=EPOCH+300_000_000, authorized_pose_ns=source)
    assert covered.start_ns == EPOCH and covered.end_ns <= source
    start = EPOCH+140_000_000
    newer = decode(raw_points(start=start), header_ns=start)
    with pytest.raises(AwaitingCoverage, match='alignment'):
        value.project(newer, CTX, now_ns=EPOCH+300_000_000, authorized_pose_ns=source)
    # A soft revocation preserves history identity, not a renewed pose watermark.
    notice = packet(source+1, usable=False)
    assert accept(inbox, notice, now_ns=EPOCH+300_000_000, mono=100.15)
    assert history_identity(inbox) == ('session', 1, 'seed')
    assert not inbox.usable(now_ns=EPOCH+300_000_000, monotonic=100.15, receipt_timeout_s=.4)


def test_task_continuity_never_renews_expired_motion_evidence():
    inbox = LocalNavigationInbox()
    assert accept(inbox, packet())
    now = 10_101_000_000  # Original IMU is now 121 ms old.
    assert inbox.task_usable(now_ns=now, monotonic=100.101, receipt_timeout_s=.4)
    assert not inbox.usable(now_ns=now, monotonic=100.101, receipt_timeout_s=.4)
    assert accept(inbox, packet(), now_ns=now, mono=100.101) is None
    # Task observation does not subtract delivery from the raw posterior again;
    # the original source/receipt 400 ms bounds remain, with no control renewal.
    assert inbox.task_usable(now_ns=10_370_000_000, monotonic=100.37, receipt_timeout_s=.4)
    assert not inbox.usable(now_ns=10_370_000_000, monotonic=100.37, receipt_timeout_s=.4)
    assert inbox.task_usable(now_ns=10_400_000_000, monotonic=100.39, receipt_timeout_s=.4)
    assert not inbox.task_usable(now_ns=10_400_000_001, monotonic=100.39, receipt_timeout_s=.4)
    assert not inbox.task_usable(now_ns=10_390_000_000, monotonic=100.400001, receipt_timeout_s=.4)


def test_provider_coast_and_delivery_are_not_two_raw_imu_age_checks():
    # Actual failure: 94.6 ms valid coast + 25.9 ms delivery was rejected as
    # a 120.5 ms old raw IMU, although the provider accepted this estimate.
    m=packet();m.imu_stamp=stamp_time(9.9054);m.extrapolation_sec=.0946
    inbox=LocalNavigationInbox()
    assert accept(inbox,m,now_ns=10_025_900_000)
    assert inbox.usable(now_ns=10_050_000_000,monotonic=100.05,receipt_timeout_s=.4)
    assert not inbox.usable(now_ns=10_100_000_001,monotonic=100.1,receipt_timeout_s=.4)
    assert inbox.latest.state.imu_ns==9_905_400_000
    assert inbox.latest.state.valid_until_ns==10_100_000_000
    assert accept(inbox,m,now_ns=10_050_000_000) is None


@pytest.mark.parametrize('bad',['schema','expiry','oversized_lease','raw_coast'])
def test_original_provider_lease_is_not_a_blanket_freshness_bypass(bad):
    m=packet()
    if bad=='schema':m.schema_version=1
    elif bad=='expiry':m.valid_until=stamp_time(10.)
    elif bad=='oversized_lease':m.valid_until=stamp_time(10.100000001)
    else:m.imu_stamp=stamp_time(9.899);m.extrapolation_sec=.101
    assert accept(LocalNavigationInbox(),m,now_ns=10_001_000_000) is None


@pytest.mark.parametrize('reason', ['imu_temporarily_unavailable', 'lio_reset'])
def test_explicit_producer_loss_pauses_task_without_consuming_old_geometry(reason):
    inbox = LocalNavigationInbox()
    assert accept(inbox, packet())
    assert accept(inbox, packet(10_020_000_000, usable=False, reason=reason), mono=100.02)
    assert not inbox.task_usable(now_ns=10_020_000_000, monotonic=100.02, receipt_timeout_s=.4)


@pytest.mark.parametrize('delay_ns', [100_000_001, 120_000_000, 250_000_000, 400_000_000])
def test_history_admits_original_late_measurement_without_current_control_permission(delay_ns):
    message = packet()
    history, control = LocalNavigationHistoryInbox(), LocalNavigationInbox()
    original = deepcopy(message)
    now = 10_000_000_000+delay_ns
    event = accept(history, message, now_ns=now)
    assert event and event.state.source_ns == 10_000_000_000
    assert event.state.valid_until_ns == 10_100_000_000
    assert event.state.imu_ns == 9_980_000_000
    assert message == original
    assert accept(control, message, now_ns=now) is None
    assert not hasattr(history, 'usable') and not hasattr(history, 'task_usable')
    assert accept(history, message, now_ns=now, mono=101.) is None
    assert history.latest is event and history.latest.received == 100.


@pytest.mark.parametrize('delay_ns', [-1, 400_000_001, 500_000_000])
def test_history_does_not_admit_future_or_unbounded_late_measurements(delay_ns):
    assert accept(LocalNavigationHistoryInbox(), packet(),
        now_ns=10_000_000_000+delay_ns) is None


@pytest.mark.parametrize('bad', ['lease', 'coast', 'posterior', 'frame', 'pose', 'twist'])
def test_late_history_still_requires_original_producer_geometry_and_source_bounds(bad):
    message = packet()
    if bad == 'lease': message.valid_until = stamp_time(10.100000001)
    elif bad == 'coast': message.imu_stamp = stamp_time(9.899); message.extrapolation_sec = .101
    elif bad == 'posterior': message.posterior_stamp = stamp_time(9.599)
    elif bad == 'frame': message.local_odometry.header.frame_id = 'map'
    elif bad == 'pose': message.local_odometry.pose.pose.position.x = float('nan')
    else: message.local_odometry.twist.twist.angular.z = float('nan')
    assert accept(LocalNavigationHistoryInbox(), message, now_ns=10_120_000_000) is None


def test_history_hard_reset_prevents_queued_geometry_from_reviving_old_context():
    inbox = LocalNavigationHistoryInbox()
    assert accept(inbox, packet(), now_ns=10_120_000_000)
    reset = packet(10_100_000_000, usable=False, reason='lio_reset')
    event = accept(inbox, reset, now_ns=10_200_000_000)
    assert event.hard_failure and history_identity(inbox) is None
    assert accept(inbox, packet(10_080_000_000), now_ns=10_200_000_000) is None
    assert inbox.latest is event
    newer = packet(10_100_000_001, epoch=2, seed='new')
    assert accept(inbox, newer, now_ns=10_200_000_002)
    assert accept(inbox, packet(10_120_000_000), now_ns=10_220_000_002) is None
    assert history_identity(inbox) == ('session', 2, 'new')
