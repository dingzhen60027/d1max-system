"""Bounded, passive rendering of the real tree, never a navigation authority."""


def awaiting_goal_owner(status, *, pending=False, active=False, task_id=None):
    """A new heartbeat from the old task is not an ACK/result for a new goal.

    Only UUID-scoped Action feedback supplies the submitted goal's task ID.
    This affects presentation, never the retained owner cache or execution.
    A reopened view (no local outstanding request) reads the current owner.
    """
    return bool((pending or active) and
                (not task_id or not isinstance(status, dict) or
                 status.get('task_id') != task_id))


def tree_presentation(status):
    def text(value, limit=160):
        return value[:limit] if isinstance(value, str) else ''
    if not isinstance(status, dict) or not status:
        return dict(label='等待任务状态', detail='', nodes=[], transitions=[])
    phase = text(status.get('phase'))
    root = text(status.get('root_status'))
    labels = {'idle': '等待目标', 'running': '执行任务', 'paused': '等待输入恢复',
              'accepted': '目标已接收', 'computing_route': '计算全局路线',
              'computing': '计算全局路线', 'following_route': '沿路线规划',
              'waiting_worker': '等待全局规划器',
              'following': '沿路线规划', 'waiting_localization': '等待定位',
              'waiting_local_trajectory': '等待局部有效轨迹',
              'waiting_local_map': '等待局部地图', 'waiting_reference': '等待路线接收',
              'recovering_local_trajectory': '局部重新规划',
              'local_contract_failed': '局部接口异常',
              'switching_goal': '切换目标',
              'canceling': '正在取消', 'cancelling': '正在取消',
              'retiring': '等待旧任务退出', 'retiring_task': '等待任务退出', 'succeeded': '任务完成',
              'failed': '任务失败', 'canceled': '已取消', 'cancelled': '已取消'}
    label = labels.get(phase, {'SUCCESS': '任务完成', 'FAILURE': '任务失败',
                              'IDLE': '等待目标', 'RUNNING': '执行任务'}.get(root, '任务状态'))
    active = text(status.get('active_node'))
    reason = text(status.get('reason'), 240)
    worker_reason = text(status.get('worker_reason'), 240)
    # This SDK code covers the entire observed working state, not mode alone:
    # mode=STAIR, MOTION_STATUS_STAIR, low gear and head-forward must match.
    # MOTION_STATUS_STAND_UP means the transition, not the required ready state.
    # Render an action hint only for a failed task; a retained old reason must
    # not overwrite a later running/successful task. This grants no authority.
    stair_state_rejected = (
        phase == 'failed' and
        'sdk_grant_rejected:fixed_stair_requires_selected_stair_mode'
        in (reason, worker_reason))
    if worker_reason and worker_reason != reason:
        reason = ' / '.join(x for x in (reason, worker_reason) if x)
    if stair_state_rejected:
        label = '未执行：请确认爬楼梯模式、站立、低速档、机头前向'
        reason = ('SDK 尚未确认所需工作状态。请先在遥控器选择爬楼梯模式，'
                  '确认已站立并显示“登阶模式运动状态”，以及低速档和机头前向；'
                  '待状态更新后重新提交目标。'
                  '原始原因：' + reason)
    task = text(status.get('task_id'), 128)
    nodes = status.get('nodes')
    transitions = status.get('transitions')
    # Never propagate arbitrary nested/oversized data into the panel's 16 KiB contract.
    nodes = [dict(name=text(n.get('name')), status=text(n.get('status'), 16))
             for n in nodes[:24] if isinstance(n, dict)] if isinstance(nodes, list) else []
    transitions = [dict(name=text(n.get('node', n.get('name'))), previous=text(n.get('previous'), 16),
                        current=text(n.get('current'), 16))
                   for n in transitions[-8:] if isinstance(n, dict)] if isinstance(transitions, list) else []
    return dict(label=label, root_status=root, active_node=active, task_id=task,
        detail=' · '.join(x for x in (active, reason) if x), nodes=nodes, transitions=transitions)
