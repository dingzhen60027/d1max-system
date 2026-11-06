"""Pure task-follower workflow status, never motion or collision authority.

The adapter owns ordered message admission, cancellation and measured arrival.
This projection consumes its already admitted typed messages and rechecks the
current task/time binding. A durable route receipt does not expire, whereas a
local status cannot renew its original body deadline. No scan/display JSON is
an input; ordinary local failure remains a recoverable workflow state.
"""
from dataclasses import dataclass
import math

from .bt_follow_policy import FollowStage


@dataclass(frozen=True)
class FormalFollowContext:
    session_id: str
    task_id: str
    route_id: str
    route_hash: str
    map_version_id: str
    localization_epoch: int
    localization_seed_id: str
    transport_mode: str


@dataclass(frozen=True)
class FormalFollowStatus:
    stage: FollowStage
    # Real body source, not the executor's reporting heartbeat, counts recovery.
    evidence_source_ns: int = 0
    trajectory_id: int = -1


def route_delivery_rejection_stage(receipt):
    """Only an explicit unowned, prior-retirement refusal is retryable.

    A local candidate failure AFTER route acceptance is not this contract.
    Retrying asks the same receiver to re-evaluate its real retirement barrier;
    the rejection itself is neither retirement proof nor permission to prepare.
    """
    reason = 'reference_delivery_rejected:'+str(receipt.reason)[:120]
    retryable = (receipt.reason == 'prior_route_retirement_pending'
        and not receipt.native_retirement_required
        and not receipt.native_retirement_confirmed and receipt.native_generation == 0)
    return FollowStage('waiting_reference' if retryable else 'local_contract_failed',
        reason, 'reference' if retryable else '', terminal=not retryable)


def _stamp_ns(stamp):
    if (type(stamp.sec) is not int or type(stamp.nanosec) is not int
            or stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000):
        raise ValueError('invalid_follow_status_stamp')
    return stamp.sec*1_000_000_000+stamp.nanosec


def _receipt_matches(context, delivery, receipt):
    if receipt is None or delivery is None:
        return False
    try:
        identity = (receipt.session_id, receipt.task_id, receipt.route_id, receipt.route_hash)
        return (receipt.schema_version == 1 and receipt.active is True
            and receipt.status in (1, 2)  # RouteDeliveryReceipt ACCEPTED / REJECTED.
            and identity == (context.session_id, context.task_id, context.route_id, context.route_hash)
            and (*identity, receipt.delivery_sequence, _stamp_ns(receipt.source_stamp)) == delivery)
    except (AttributeError, TypeError, ValueError):
        return False


def _local_matches(context, local):
    try:
        version = local.version
        return (local.schema_version == 1 and version.schema_version == 3
            and all(getattr(version, name) == getattr(context, name) for name in (
                'session_id', 'task_id', 'route_id', 'route_hash', 'map_version_id',
                'localization_epoch', 'localization_seed_id'))
            and local.transport_mode == context.transport_mode)
    except AttributeError:
        return False


def classify_formal_follow(*, context, pose_ready, published, subscriber_ready,
                           active_delivery, active_receipt, local_status,
                           local_status_received, now_ns, now_monotonic):
    """Project admitted RouteDeliveryReceipt/LocalExecutionStatus, without ROS.

    ``active_delivery`` is the adapter's exact six-tuple (session, task, route,
    hash, sequence, original source ns). Receipt ordering/retirement and local
    sequence/execution-control admission stay in their existing callbacks.
    The function is stateless: duplicates return the SAME evidence timestamp,
    never a receipt- or timer-dated replacement for recovery stability tests.
    Initial-reference map admission remains outside this projection, avoiding
    a cycle where publishing the first reference requires an existing curve.
    """
    def waiting(reason, *, phase='recovering_local_trajectory', budget='trajectory'):
        return FormalFollowStatus(FollowStage(phase, reason, budget))

    matched_receipt = _receipt_matches(context, active_delivery, active_receipt)
    if matched_receipt and active_receipt.status == 2:
        # A permanent software contract error does not become an indefinite
        # localization wait when a sensor happens to be temporarily unavailable.
        return FormalFollowStatus(route_delivery_rejection_stage(active_receipt))
    if not pose_ready:
        return waiting('waiting_current_localization', phase='waiting_localization', budget='pose')
    if not published:
        return waiting('delivering_committed_route' if subscriber_ready else 'waiting_reference_subscriber',
                       phase='waiting_reference', budget='reference')
    if not matched_receipt:
        return waiting('waiting_reference_acceptance', phase='waiting_reference', budget='reference')
    if local_status is None:
        return waiting('waiting_native_reference')
    if not _local_matches(context, local_status):
        return waiting('waiting_current_local_executor_status')
    local = local_status
    try:
        source = _stamp_ns(local.source_stamp)
        receipt_age = now_monotonic-local_status_received
        current = (type(now_ns) is int and 0 < source <= now_ns
            and now_ns-source <= 500_000_000 and math.isfinite(receipt_age)
            and 0 <= receipt_age <= .5)
        if local.ready:
            body, until = _stamp_ns(local.body_source_stamp), _stamp_ns(local.body_valid_until)
            current = (current and 0 < body <= source <= now_ns <= until
                and 0 <= until-body <= 100_000_000)
    except (AttributeError, TypeError, ValueError, OverflowError):
        current = False
    if not current:
        return waiting('waiting_current_local_executor_status')
    if (local.ready is True and local.holding is False and local.software_retired is False
            and local.trajectory_id >= 0 and local.commit_sequence > 0):
        if local.phase == 'preview':
            # Geometry is ready for the BT's independent initial authorization,
            # but no task movement has begun. Do not label preview as following.
            return FormalFollowStatus(FollowStage('waiting_execution', 'trajectory_ready_for_execution'))
        return FormalFollowStatus(FollowStage('following', 'following_local_executor_trajectory'),
                                  body, local.trajectory_id)
    # Neither diagnostic strings nor holding/retired states are a task abort
    # or an arrival certificate. The adapter's explicit terminal fences own it.
    return waiting(str(local.reason or local.phase or 'waiting_native_trajectory')[:120])
