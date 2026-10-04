"""Owner event invariants, independent of ROS and planner choice."""
import pytest

from d1max_pct_scan.navigation_task_policy import (
    LocalAction, RouteAction, TaskEvent, task_transition,
)


@pytest.mark.parametrize('event', [TaskEvent.INPUT_LOST, TaskEvent.INPUT_RECOVERED,
    TaskEvent.REFERENCE_REFRESH, TaskEvent.BODY_PROGRESS, TaskEvent.LOCAL_RETRY,
    TaskEvent.LOCAL_TRAJECTORY_INVALID])
def test_tracking_health_and_motion_cannot_replan_a_committed_route(event):
    assert task_transition(event, route_committed=True).route is RouteAction.RETAIN


@pytest.mark.parametrize('committed', [False, True])
@pytest.mark.parametrize('event', [TaskEvent.CANCEL, TaskEvent.IDENTITY_CHANGED])
def test_terminal_events_retire_both_owners(event, committed):
    decision = task_transition(event, route_committed=committed)
    assert decision.route is RouteAction.CLEAR and decision.local is LocalAction.CLEAR


@pytest.mark.parametrize('committed', [False, True])
def test_new_user_request_is_the_only_compute_event(committed):
    for event in TaskEvent:
        decision = task_transition(event, route_committed=committed)
        assert (decision.route is RouteAction.COMPUTE) == (event is TaskEvent.NEW_GOAL)


@pytest.mark.parametrize('event', [TaskEvent.INPUT_RECOVERED, TaskEvent.REFERENCE_REFRESH,
                                  TaskEvent.BODY_PROGRESS])
def test_pending_result_still_needs_start_and_context_validation(event):
    assert task_transition(event, route_committed=False).route is RouteAction.VALIDATE_PENDING


def test_pause_recovery_retry_and_invalid_curve_have_distinct_effects():
    assert task_transition(TaskEvent.INPUT_LOST, route_committed=True).local is LocalAction.SUSPEND
    assert task_transition(TaskEvent.INPUT_RECOVERED, route_committed=True).local is LocalAction.REVALIDATE
    assert task_transition(TaskEvent.LOCAL_RETRY, route_committed=True).local is LocalAction.KEEP_VALID
    assert task_transition(TaskEvent.LOCAL_TRAJECTORY_INVALID, route_committed=True).local is LocalAction.CLEAR


@pytest.mark.parametrize('event,committed', [('input_lost', True), (None, True),
                                           (TaskEvent.INPUT_LOST, 1)])
def test_implicit_or_unknown_events_fail_closed(event, committed):
    with pytest.raises(ValueError):
        task_transition(event, route_committed=committed)
