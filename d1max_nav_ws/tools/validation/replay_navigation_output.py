"""Observe the real navigation publisher in an isolated recorded session.

No estimator or admission behavior is changed. A rejected atomic pair records
its original time/frame provenance instead of disappearing behind a generic
sticky last_error. This is never a production launch entry.
"""
import json
import os
import signal
import time


def main():
    from d1max_pct_scan.isolated_zenoh import validate_environment
    validate_environment()
    import rclpy
    if os.environ.get('D1MAX_REPLAY_INTEGRATED_OUTPUT') == '1':
        from d1max_localization.realtime_navigation_output import RealtimeNavigationOutput as NavigationOutput
    else:
        from d1max_localization.navigation_output import NavigationOutput

    class ObservedNavigationOutput(NavigationOutput):
        def __init__(self):
            super().__init__()
            if self.wire_session.get('input_source') != 'isolated_recorded_rosbag':
                raise ValueError('recorded_session_required')
            self.last_diagnostic = 0.

        def publish_global(self, local, pose, pc):
            try:
                super().publish_global(local, pose, pc)
            except ValueError as error:
                now = time.monotonic()
                if now - self.last_diagnostic >= .5:
                    self.last_diagnostic = now
                    self.get_logger().error('atomic_pair_rejected:' + json.dumps(dict(
                        error=str(error), target_ns=round(local.stamp * 1e9),
                        posterior_ns=round(local.source_stamp * 1e9),
                        imu_ns=round(local.imu_stamp * 1e9),
                        unsupported_sec=local.extrapolation,
                        expected_unsupported_sec=max(0., local.stamp-local.imu_stamp),
                        epoch=self.core.epoch, seed=self.core.key(),
                        map_frame=self.p['map_frame'], odom_frame=self.p['odom_frame'],
                        body_frame=self.p['body_frame']), allow_nan=False))
                raise

    rclpy.init()
    node = ObservedNavigationOutput()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
