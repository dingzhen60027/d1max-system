"""Explicit recovery episodes, independent of ROS and native map internals."""
import pytest
from d1max_pct_scan.bt_follow_policy import (FollowRecovery, FollowStage,
    classify_follow, validate_follow_policy)


GOOD = FollowStage('following', 'following')
BLOCKED = FollowStage('recovering_local_trajectory', 'failed_optimization', 'trajectory')


@pytest.mark.parametrize('key,value', [('follow_map_wait_s', float('nan')),
    ('follow_map_wait_s', -1), ('follow_reference_wait_s', True),
    ('follow_stable_reset_s', .01), ('follow_trajectory_wait_s', 999)])
def test_recovery_policy_cannot_be_disabled_or_unbounded(key, value):
    with pytest.raises(ValueError):
        validate_follow_policy({key: value})


def test_stable_reset_must_fit_episode():
    with pytest.raises(ValueError, match='stable_reset_must_fit_episode'):
        validate_follow_policy(dict(follow_stable_reset_s=3., follow_recovery_episode_s=1.))


def test_candidate_failure_does_not_override_current_proved_curve():
    stage = classify_follow(pose_ready=True, map_ready=True, committed=True,
        subscriber_ready=True, same_reference=True,
        scan={'spline_visual_valid': True, 'local_debug_phase': 'failed_optimization'})
    assert stage == FollowStage('following', 'following_validated_local_trajectory')


def test_delivered_but_unvalidated_reference_reports_native_wait_not_delivery_failure():
    stage=classify_follow(pose_ready=True,map_ready=True,committed=True,
        subscriber_ready=True,same_reference=False,scan={},pending_native_phase='waiting_observed_space')
    assert stage==FollowStage('recovering_local_trajectory','waiting_observed_space','trajectory')
    assert stage.budget and stage.phase!='following'


def test_proved_curve_never_overrides_wrong_reference_or_stale_map():
    for field in ('pose_ready', 'map_ready', 'same_reference'):
        args = dict(pose_ready=True, map_ready=True, committed=True,
                    subscriber_ready=True, same_reference=True, scan={'spline_visual_valid': True})
        args[field] = False
        assert classify_follow(**args).budget


def test_stationary_healthy_preview_has_no_progress_timeout():
    policy = FollowRecovery({})
    for tick in range(1000):
        assert policy.observe(GOOD, tick, evidence_stamp=tick+1) == ''
    assert policy.since is None and not policy.waits


def test_persistent_blockage_has_bounded_recovery_without_global_retry():
    policy = FollowRecovery({})
    for tick in range(20):
        assert not policy.observe(BLOCKED, tick)
    assert policy.observe(BLOCKED, 20).startswith('follow_trajectory_timeout:')


def test_alternating_reasons_does_not_renew_recovery_deadline():
    policy = FollowRecovery({})
    stages = [FollowStage('waiting_local_map', 'missing', 'map'),
              FollowStage('waiting_localization', 'pose', 'pose'), BLOCKED]
    reason = ''
    for tick in range(31):
        reason = policy.observe(stages[tick % 3], tick)
    assert reason.startswith('follow_recovery_episode_timeout:')


def test_one_good_packet_cannot_extend_indefinite_wait():
    policy = FollowRecovery({})
    reason = ''
    for tick in range(61):
        reason = policy.observe(GOOD if tick % 2 else BLOCKED, tick*.5, evidence_stamp=tick+1)
    assert reason.startswith('follow_recovery_episode_timeout:')


def test_stable_genuine_recovery_starts_new_episode_not_new_task():
    policy = FollowRecovery({})
    assert not policy.observe(BLOCKED, 0.)
    assert not policy.observe(BLOCKED, 10.)
    for t in (10.1, 10.5, 10.9):
        assert not policy.observe(GOOD, t, evidence_stamp=100+t)
    assert policy.since is None and not policy.waits
    assert not policy.observe(BLOCKED, 40.)
    assert not policy.observe(BLOCKED, 59.)
    assert policy.observe(BLOCKED, 60.).startswith('follow_trajectory_timeout:')


def test_repeated_good_report_is_not_sustained_recovery():
    policy = FollowRecovery({})
    policy.observe(BLOCKED, 0.)
    for t in range(1, 31):
        reason = policy.observe(GOOD, t, evidence_stamp=123.)
    assert reason.startswith('follow_recovery_episode_timeout:')


@pytest.mark.parametrize('phase', ['reference_rejected_frame', 'reference_rejected_geometry',
                                 'failed_reference_geometry', 'emergency_stop'])
def test_contract_failure_has_no_retry_loop(phase):
    stage = classify_follow(pose_ready=True, map_ready=True, committed=True,
        subscriber_ready=True, same_reference=True,
        scan={'spline_visual_valid': False, 'local_debug_phase': phase})
    assert FollowRecovery({}).observe(stage, 0.) == 'follow_contract_failure:'+phase
