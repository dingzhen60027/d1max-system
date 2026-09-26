"""Bag-only seed admission adapter; never used by the live localization launch.

No SDK telemetry is fabricated. Only the initial-pose head-direction prerequisite
is omitted because the recorded bag has no RobotState. All estimator, matching,
freshness and navigation-readiness checks remain the production implementation.
"""
import json
import os
from pathlib import Path
import signal
import threading
import yaml


def validate_isolation():
    if os.environ.get('RMW_IMPLEMENTATION') != 'rmw_zenoh_cpp':
        raise RuntimeError('Replay requires Zenoh')
    if os.environ.get('ROS_DOMAIN_ID') != '24':
        raise RuntimeError('Unexpected replay domain')
    if os.environ.get('ZENOH_CONFIG_OVERRIDE') or os.environ.get('ZENOH_SESSION_CONFIG'):
        raise RuntimeError('Replay refuses transport overrides')
    config = yaml.safe_load(Path(os.environ['ZENOH_SESSION_CONFIG_URI']).read_text())
    if (config.get('mode') != 'client'
            or config.get('connect', {}).get('endpoints') != ['tcp/127.0.0.1:17449']
            or config.get('listen', {}).get('endpoints')
            or config.get('scouting', {}).get('multicast', {}).get('enabled') is not False
            or config.get('scouting', {}).get('gossip', {}).get('enabled') is not False):
        raise RuntimeError('Replay must use isolated loopback router 17449')


def main():
    validate_isolation()
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from d1max_localization.lio_localizer import LioLocalizer

    class ReplayLocalizer(LioLocalizer):
        def __init__(self):
            super().__init__()
            if self.session.get('mode') != 'RECORDED_BAG_LOCALIZATION_TEST':
                raise RuntimeError('Explicit replay session is required')
            self.get_logger().warning('BAG TEST ONLY: SDK/head state absent; initial pose is tracking-reference. No SDK or controls.')

        def seed_ready(self):
            local_ready = (not self.p.get('navigation_output_enabled') or
                self.fresh(self.navigation_at, .6)
                and self.navigation.get('epoch') == self.core.local_epoch
                and self.navigation.get('local_fresh') is True)
            return local_ready and self.front_ready() and self.core.local_fresh(self.now_s())

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = ReplayLocalizer()
        while not stop.is_set() and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.05)
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if node is not None:
            node.close()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
