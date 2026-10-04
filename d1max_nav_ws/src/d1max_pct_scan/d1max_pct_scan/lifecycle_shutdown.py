"""Drain the task owner while its downstream action servers are still alive.

This is not a stop acknowledgement: software retirement and physical stopping
are separate facts. Importing this module never creates a ROS context.
"""
import json
import math
import time


def drained_status(value, *, session_id, requested_at, now):
    stamp = value.get('stamp')
    return (value.get('schema') == 2 and value.get('session_id') == session_id
            and type(stamp) in (int, float) and math.isfinite(stamp)
            and requested_at <= stamp <= now + .1 and now-stamp <= .6
            and value.get('lifecycle_active') is False
            and value.get('lifecycle_draining') is False
            and value.get('lifecycle_quarantined') is False)


def drain_task_owner(session_id, *, budget_s=6.):
    """Bounded shutdown client, called ONLY by the already-running supervisor.

    It shares that supervisor's domain/router and cannot restart any component.
    Failure is recorded as unconfirmed, never converted into physical success.
    """
    if not session_id or not 0 < budget_s <= 10.:
        raise ValueError('invalid_shutdown_contract')
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from std_msgs.msg import String
    from d1max_navigation_bt_interfaces.srv import PrepareTransition
    context, node, executor = Context(), None, None
    result = dict(request_accepted=False, software_retired=False,
                  physical_stop_confirmed=False, reason='task_owner_drain_timeout')
    started = time.monotonic()
    try:
        rclpy.init(context=context)
        node = rclpy.create_node('d1max_shutdown_observer', context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        requested_at = node.get_clock().now().nanoseconds*1e-9
        def observe(message):
            try:
                value = json.loads(message.data)
                if result['request_accepted'] and drained_status(value,
                        session_id=session_id, requested_at=requested_at,
                        now=node.get_clock().now().nanoseconds*1e-9):
                    result.update(software_retired=True,
                                  physical_stop_confirmed=value.get('physical_stop_confirmed') is True,
                                  reason=('software_retired_and_physical_stop_confirmed'
                                    if value.get('physical_stop_confirmed') is True
                                    else 'software_retired_not_physical_stop'))
            except (ValueError, TypeError, AttributeError):
                pass
        node.create_subscription(String, '/d1max/live_planning/bt/status', observe,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        client = node.create_client(PrepareTransition, '/d1max/live_planning/bt/prepare_transition')
        future = None
        while time.monotonic()-started < budget_s:
            if future is None and client.service_is_ready():
                request = PrepareTransition.Request()
                request.schema_version, request.reason = 2, 'owned_supervisor_shutdown'
                request.session_id = session_id
                future = client.call_async(request)
            executor.spin_once(timeout_sec=.05)
            if future is not None and future.done() and not result['request_accepted']:
                response = future.result()
                if response is None or response.schema_version != 2 or not response.accepted:
                    result['reason'] = 'task_owner_drain_rejected'
                    break
                result['request_accepted'] = True
                # ACK means accepted for drain, not that dependencies retired.
            if result['software_retired']:
                break
    except Exception as error:
        result['reason'] = 'task_owner_drain_error:'+type(error).__name__
    finally:
        if executor is not None:
            executor.shutdown(timeout_sec=.1)
        if node is not None:
            node.destroy_node()
        if context.ok():
            context.shutdown()
    result['elapsed_s'] = time.monotonic()-started
    return result
