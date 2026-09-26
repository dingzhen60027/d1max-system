"""Pure high-rate pose evidence admission; no middleware or robot endpoints."""

import pytest

from d1max_localization.estimation.pose_status import continuous_pose_status


def evidence(**changes):
    value=dict(schema=1, epoch=3, seed_id='seed', frame_id='map', body_frame='body',
               received_at_unix=10., output_stamp_sec=9.98, pose_timeout_sec=.08,
               valid=True, pose_valid=True, fault='', reset_pending=False,
               motion_control_enabled=False, quality='tracking', reason='new_output')
    value.update(changes)
    return value


def check(value=None, **changes):
    context=dict(now=10., mono=100., received_mono=99.99, epoch=3, seed='seed',
                 confirmed_seed='seed', confirmations=3, local_fault='',
                 map_frame='map', body_frame='body')
    context.update(changes)
    return continuous_pose_status(evidence() if value is None else value, **context)


def test_matching_state_is_not_a_dependency_of_fresh_output():
    assert check() == (True, 'tracking', 'fresh_continuous_pose')
    assert check(evidence(quality='coasting')) == (True, 'coasting', 'fresh_continuous_pose')
    # No raw LIO posterior freshness, SDK heartbeat or latest matching bool is
    # accepted as a substitute authority for this high-rate estimate.
    assert not check(evidence(pose_valid=False, reason='waiting_map'))[0]


@pytest.mark.parametrize('changes', [
    {'epoch': 2}, {'epoch': True}, {'seed_id': 'old'}, {'frame_id': 'odom'},
    {'body_frame': 'tracking'}, {'fault': 'clock_reset'}, {'reset_pending': True},
    {'reset_pending': None}, {'pose_valid': None}, {'valid': False},
    {'output_stamp_sec': 9.90}, {'output_stamp_sec': 10.02},
    {'output_stamp_sec': float('nan')}, {'pose_timeout_sec': .5},
    {'pose_timeout_sec': True}, {'received_at_unix': 9.8},
    {'received_at_unix': 10.02}, {'schema': 0},
])
def test_invalid_pose_evidence_never_authorizes(changes):
    assert not check(evidence(**changes))[0]


@pytest.mark.parametrize('changes', [
    {'epoch': 4}, {'seed': 'new'}, {'confirmed_seed': 'old'}, {'confirmed_seed': None},
    {'confirmations': 2}, {'confirmations': True}, {'local_fault': 'pose_jump'},
    {'received_mono': 99.8}, {'received_mono': 100.01}, {'now': 9.8},
])
def test_current_context_and_both_clocks_remain_mandatory(changes):
    assert not check(**changes)[0]


def test_status_heartbeats_cannot_extend_original_pose_time():
    for now in (10., 10.02, 10.04, 10.059):
        assert check(evidence(received_at_unix=now), now=now)[0]
    assert not check(evidence(received_at_unix=10.061), now=10.061)[0]
    assert not check(evidence(output_stamp_sec=10.15, received_at_unix=10.17),
                     now=10.17, received_mono=99.8)[0]
