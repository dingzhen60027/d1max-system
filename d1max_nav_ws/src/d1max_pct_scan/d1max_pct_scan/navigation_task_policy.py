"""Event policy shared by task ownership and local-follow adapters.

There is deliberately no second state machine, timer, ROS node or algorithm
here. The global owner stores the goal and committed route; the local owner
stores current tracking/proof state. They use the same event semantics instead
of treating a sensor heartbeat, failed candidate or robot motion as a new goal.

This is an event-driven contract, not a BehaviorTree.CPP/Nav2 integration.
"""
from dataclasses import dataclass
from enum import Enum


POLICY_ID = 'event_driven_fixed_route_v1'


class TaskEvent(str, Enum):
    NEW_GOAL = 'new_goal'
    CANCEL = 'cancel'
    IDENTITY_CHANGED = 'identity_changed'
    INPUT_LOST = 'input_lost'
    INPUT_RECOVERED = 'input_recovered'
    REFERENCE_REFRESH = 'reference_refresh'
    BODY_PROGRESS = 'body_progress'
    LOCAL_RETRY = 'local_retry'
    LOCAL_TRAJECTORY_INVALID = 'local_trajectory_invalid'


class RouteAction(str, Enum):
    COMPUTE = 'compute'
    RETAIN = 'retain'
    VALIDATE_PENDING = 'validate_pending'
    CLEAR = 'clear'


class LocalAction(str, Enum):
    KEEP_VALID = 'keep_valid'
    SUSPEND = 'suspend'
    REVALIDATE = 'revalidate'
    CLEAR = 'clear'


@dataclass(frozen=True)
class TaskDecision:
    route: RouteAction
    local: LocalAction


def task_transition(event: TaskEvent, *, route_committed: bool) -> TaskDecision:
    """Decide ownership effects, never sensor/collision/execution admission.

    RETAIN means immutable route geometry/identity, not permission to drive.
    KEEP_VALID means an existing trajectory still needs its independent,
    current native collision and source-time proof. No old proof is renewed.
    A moved start matters only BEFORE the first route commit. A new explicit
    operator planning request enters through NEW_GOAL, not through recovery.
    """
    if not isinstance(event, TaskEvent) or type(route_committed) is not bool:
        raise ValueError('explicit_task_event_and_route_state_required')
    if event is TaskEvent.NEW_GOAL:
        return TaskDecision(RouteAction.COMPUTE, LocalAction.CLEAR)
    if event in (TaskEvent.CANCEL, TaskEvent.IDENTITY_CHANGED):
        return TaskDecision(RouteAction.CLEAR, LocalAction.CLEAR)
    if event is TaskEvent.INPUT_LOST:
        return TaskDecision(RouteAction.RETAIN, LocalAction.SUSPEND)
    if event in (TaskEvent.INPUT_RECOVERED, TaskEvent.REFERENCE_REFRESH):
        return TaskDecision(RouteAction.RETAIN if route_committed else
                            RouteAction.VALIDATE_PENDING, LocalAction.REVALIDATE)
    if event is TaskEvent.BODY_PROGRESS:
        return TaskDecision(RouteAction.RETAIN if route_committed else
                            RouteAction.VALIDATE_PENDING, LocalAction.KEEP_VALID)
    if event is TaskEvent.LOCAL_RETRY:
        return TaskDecision(RouteAction.RETAIN, LocalAction.KEEP_VALID)
    if event is TaskEvent.LOCAL_TRAJECTORY_INVALID:
        return TaskDecision(RouteAction.RETAIN, LocalAction.CLEAR)
    raise ValueError('unsupported_task_event')
