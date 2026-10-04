"""Observe the real BT input admission without changing its decisions."""
from collections import Counter, deque
import json
import time


def main():
    from d1max_pct_scan.isolated_zenoh import validate_environment
    validate_environment()
    import rclpy
    from std_msgs.msg import String
    from d1max_pct_scan.bt_adapters import NavigationBTAdapters
    from d1max_pct_scan.atomic_projection_state import stamp_ns, validate

    class ObservedAdapters(NavigationBTAdapters):
        def __init__(self):
            self.input_outcomes=Counter()
            self.use_outcomes=Counter()
            self.input_delta=deque(maxlen=10000)
            self.last_timing=0.
            super().__init__()
            self.timing_pub=self.create_publisher(String,'/d1max/replay/bt_input_timing',2)

        def on_atomic_state(self,message):
            before=self.atomic_inbox.latest
            now_ns=self.get_clock().now().nanoseconds
            source=stamp_ns(message.source_stamp)
            super().on_atomic_state(message)
            accepted=self.atomic_inbox.latest is not before
            self.input_delta.append((now_ns-source)*1e-9)
            if accepted:
                reason='accepted_usable' if message.usable else 'accepted_unavailable'
            elif source>now_ns:
                reason='rejected_future_source'
            elif before is not None and source<=before.source_ns:
                reason='rejected_old_or_duplicate'
            else:
                try:
                    validate(message,session_id=self.p['session_id'],
                        map_version_id=self.p['map_version_id'],now_ns=now_ns)
                    reason='rejected_other_identity'
                except (ValueError,TypeError,AttributeError) as error:
                    reason=str(error)
            self.input_outcomes[reason]+=1

        def tick(self):
            # Diagnostic only. The original identity()/tick() still own
            # readiness; do not hold an expired state or alter freshness.
            now=self.get_clock().now().nanoseconds
            event=self.atomic_inbox.latest
            if event is None:
                reason='no_state'
            elif event.state is None:
                reason='explicit_unavailable'
            elif now<event.state.source_ns:
                reason='clock_before_source'
            elif now-event.state.imu_ns>100_000_000:
                reason='imu_age_at_use'
            elif now-event.state.posterior_ns>400_000_000:
                reason='posterior_age_at_use'
            elif not self.atomic_inbox.usable(now_ns=now,monotonic=time.monotonic(),
                    receipt_timeout_s=self.p['freshness_s']):
                reason='state_or_receipt_age_at_use'
            else:
                reason='usable'
            self.use_outcomes[reason]+=1
            super().tick()
            if time.monotonic()-self.last_timing>=1.:
                from d1max_localization.realtime_navigation_output import quantiles_ms
                self.last_timing=time.monotonic()
                self.timing_pub.publish(String(data=json.dumps(dict(
                    incoming=dict(self.input_outcomes),at_use=dict(self.use_outcomes),
                    source_age_at_arrival_ms=quantiles_ms(self.input_delta),
                    minimum_source_age_at_arrival_ms=1000*min(self.input_delta) if self.input_delta else None),allow_nan=False)))

    rclpy.init()
    node=ObservedAdapters()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__=='__main__':
    main()
