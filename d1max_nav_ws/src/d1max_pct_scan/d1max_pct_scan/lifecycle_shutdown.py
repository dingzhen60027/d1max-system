"""Drain the task owner while its downstream action servers are still alive.

This is not a stop acknowledgement: software retirement and physical stopping
are separate facts. Importing this module never creates a ROS context.
"""
import json
import math
import time


class OriginalClockSource:
    """Retain only original /clock integers; a regression is terminal here."""
    def __init__(self):
        self.nanoseconds = 0
        self.fault = ''

    def observe(self, message):
        if self.fault:
            return False
        try:
            stamp = message.clock
            if (type(stamp.sec) is not int or not 0 <= stamp.sec <= 2147483647
                    or type(stamp.nanosec) is not int or not 0 <= stamp.nanosec < 1000000000):
                return False
            source = stamp.sec * 1000000000 + stamp.nanosec
            if source <= 0:
                return False
            if source < self.nanoseconds:
                self.fault = 'clock_regressed'
                return False
            self.nanoseconds = source
            return True
        except AttributeError:
            return False


def drained_status(value, *, session_id, requested_at, now):
    stamp = value.get('stamp')
    return (value.get('schema') == 2 and value.get('session_id') == session_id
            and type(stamp) in (int, float) and math.isfinite(stamp)
            and requested_at <= stamp <= now + .1 and now-stamp <= .6
            and value.get('lifecycle_active') is False
            and value.get('lifecycle_draining') is False
            and value.get('lifecycle_quarantined') is False)


def drain_task_owner(session_id, *, budget_s=6., use_sim_time=False):
    """Bounded shutdown client, called ONLY by the already-running supervisor.

    It shares that supervisor's domain/router and cannot restart any component.
    Failure is recorded as unconfirmed, never converted into physical success.
    """
    if not session_id or not 0 < budget_s <= 10. or type(use_sim_time) is not bool:
        raise ValueError('invalid_shutdown_contract')
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from std_msgs.msg import String
    from d1max_navigation_bt_interfaces.srv import PrepareTransition
    context, node, executor = Context(), None, None
    result = dict(request_sent=False, request_accepted=False, software_retired=False,
                  physical_stop_confirmed=False, reason='task_owner_drain_timeout',
                  requested_at=None, requested_source_ns=None,
                  timeout_stage='clock' if use_sim_time else 'service')
    started = time.monotonic()
    try:
        rclpy.init(context=context)
        node = rclpy.create_node('d1max_shutdown_observer', context=context,
            use_global_arguments=False,
            parameter_overrides=[Parameter('use_sim_time', value=use_sim_time)])
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        original_clock = OriginalClockSource()
        if use_sim_time:
            from rosgraph_msgs.msg import Clock
            # Humble's built-in ROSClock subscription is volatile. This extra
            # observer receives the publisher's retained ORIGINAL last clock;
            # it never sets/overrides the node clock or manufactures a tick.
            clock_subscription = node.create_subscription(Clock, '/clock', original_clock.observe,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        def source_ns():
            if use_sim_time:
                return 0 if original_clock.fault else original_clock.nanoseconds
            return node.get_clock().now().nanoseconds
        requested_at = None
        def observe(message):
            try:
                value = json.loads(message.data)
                if (result['request_accepted'] and requested_at is not None and source_ns() > 0
                        and drained_status(value,
                        session_id=session_id, requested_at=requested_at,
                        now=source_ns()*1e-9)):
                    result.update(software_retired=True,
                                  physical_stop_confirmed=value.get('physical_stop_confirmed') is True,
                                  reason=('software_retired_and_physical_stop_confirmed'
                                    if value.get('physical_stop_confirmed') is True
                                    else 'software_retired_not_physical_stop'), timeout_stage=None)
            except (ValueError, TypeError, AttributeError):
                pass
        node.create_subscription(String, '/d1max/live_planning/bt/status', observe,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        client = node.create_client(PrepareTransition, '/d1max/live_planning/bt/prepare_transition')
        future = None
        while time.monotonic()-started < budget_s:
            current_source_ns = source_ns()
            source_now = current_source_ns*1e-9
            if future is None:
                result['timeout_stage'] = 'clock' if current_source_ns <= 0 else 'service'
            if future is None and current_source_ns > 0 and client.service_is_ready():
                # A new simulation clock starts at zero until its first /clock
                # callback. Never turn that zero into a barrier which would
                # admit an inactive latched status from before this request.
                # The total budget remains the original steady wall deadline.
                requested_at = source_now
                request = PrepareTransition.Request()
                request.schema_version, request.reason = 2, 'owned_supervisor_shutdown'
                request.session_id = session_id
                future = client.call_async(request)
                result.update(request_sent=True, requested_at=requested_at,
                              requested_source_ns=current_source_ns, timeout_stage='response')
            executor.spin_once(timeout_sec=.05)
            if original_clock.fault:
                result.update(reason='task_owner_drain_clock_regressed', timeout_stage='clock_regressed',
                              software_retired=False, physical_stop_confirmed=False)
                break
            if future is not None and future.done() and not result['request_accepted']:
                response = future.result()
                if response is None or response.schema_version != 2 or not response.accepted:
                    result['reason'] = 'task_owner_drain_rejected'
                    result['timeout_stage'] = None
                    break
                result['request_accepted'] = True
                result['timeout_stage'] = 'retirement'
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
