"""SDK preparation hints are passive text, never an execution retry or grant."""
from copy import deepcopy
import importlib.util
import os

import pytest

if os.environ.get('D1MAX_BT_PRESENTATION_BEFORE'):
    spec = importlib.util.spec_from_file_location(
        'before_bt_presentation', os.environ['D1MAX_BT_PRESENTATION_BEFORE'])
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    tree_presentation = module.tree_presentation
else:
    from d1max_pct_scan.bt_presentation import tree_presentation


REASON = 'sdk_grant_rejected:fixed_stair_requires_selected_stair_mode'
HINT = '未执行：请确认爬楼梯模式、站立、低速档、机头前向'


@pytest.mark.parametrize('reason_field', ('reason', 'worker_reason'))
def test_fixed_stair_rejection_names_every_required_observed_state(reason_field):
    state = dict(phase='failed', root_status='FAILURE', task_id='task-1',
                 active_node='FollowRoute', nodes=[dict(name='FollowRoute', status='FAILURE')],
                 transitions=[dict(node='FollowRoute', previous='RUNNING', current='FAILURE')])
    state[reason_field] = REASON
    before = deepcopy(state)
    shown = tree_presentation(state)
    assert shown['label'] == HINT
    assert len(shown['label']) <= 30  # Actual RViz task-label bound.
    assert REASON in shown['detail']
    assert '待状态更新后重新提交目标' in shown['detail']
    assert '登阶模式运动状态' in shown['detail']
    assert shown['root_status'] == 'FAILURE'
    assert shown['task_id'] == state['task_id']
    assert shown['nodes'][0]['status'] == 'FAILURE'
    assert not {'ready', 'success', 'motion_enabled', 'retry'} & shown.keys()
    assert state == before


@pytest.mark.parametrize('other', (
    'sdk_grant_rejected:writer_not_ready_or_prior_not_stopped',
    'sdk_grant_rejected:general_runtime_parameters_invalid',
    'sdk_grant_rejected:stair_move_speed_and_hold_unverified',
    'sdk_grant_rejected:control_lost',
    REASON + '_different_reason',
))
def test_other_sdk_rejections_keep_their_original_cause(other):
    shown = tree_presentation(dict(phase='failed', root_status='FAILURE', reason=other))
    assert shown['label'] == '任务失败'
    assert shown['detail'] == other


@pytest.mark.parametrize('phase,label', (
    ('running', '执行任务'), ('succeeded', '任务完成'),
    ('canceled', '已取消'), ('switching_goal', '切换目标'),
))
def test_old_rejection_cannot_relabel_new_or_completed_task(phase, label):
    shown = tree_presentation(dict(phase=phase, reason=REASON, root_status='RUNNING'))
    assert shown['label'] == label
    assert shown['detail'] == REASON


def test_hint_preserves_other_worker_details_and_existing_output_bounds():
    shown = tree_presentation(dict(phase='failed', reason=REASON,
                                  worker_reason='worker_original_detail', active_node='FollowRoute'))
    assert shown['label'] == HINT
    assert 'worker_original_detail' in shown['detail']
    assert shown['detail'].startswith('FollowRoute · ')
    assert len(shown['detail']) < 500


@pytest.mark.parametrize('state', (None, {}, [], 'invalid'))
def test_missing_status_still_waits_without_inventing_mode_failure(state):
    assert tree_presentation(state)['label'] == '等待任务状态'
