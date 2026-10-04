"""Isolated bag boundary around the real GlobalLioLocalizer, never live launch.

The bag lacks SDK RobotState. Only tracking-reference seed admission omits
that prerequisite; no SDK state is fabricated. All fine matching and public
pose admission still use the production implementations.
"""
import json
import signal
import time


def main():
    from d1max_pct_scan.isolated_zenoh import validate_environment
    validate_environment()
    import rclpy
    from rclpy.clock import Clock, ClockType
    from std_msgs.msg import String
    from d1max_localization.global_lio_localizer import GlobalLioLocalizer

    class RecordedGlobalLocalizer(GlobalLioLocalizer):
        def __init__(self):
            super().__init__()
            if self.session.get('input_source') != 'isolated_recorded_rosbag':
                raise ValueError('recorded_session_required')
            self.pause_until = 0.
            self.saw_real_ready = False
            self.create_subscription(String, '/d1max/replay/status', self.on_replay, 1)
            # Native search watchdog/polling must run while virtual bag time
            # is paused. This wall timer is exclusive to the replay boundary.
            self.timer.cancel()
            self.timer = self.create_timer(.2, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

        def on_frontend(self, message):
            super().on_frontend(message)
            if (self.frontend.get('ready') and not self.frontend.get('fault')
                    and self.frontend.get('epoch') == self.core.local_epoch):
                self.saw_real_ready = True

        def on_replay(self, message):
            v = json.loads(message.data)
            if v.get('session_id') == self.session['id']:
                self.pause_until = time.monotonic() + .5 if v.get('paused') else 0.

        def seed_ready(self):
            # A clock pause is not a broken IMU stream. The frontend's wall
            # input watchdog may report waiting, but it must have genuinely
            # initialized before this bounded virtual-time pause is accepted.
            paused = time.monotonic() < getattr(self, 'pause_until', 0.)
            frontend = self.front_ready() or (paused and getattr(self, 'saw_real_ready', False)
                and self.core.stream_valid and not self.core.fault
                and self.frontend.get('epoch') == self.core.local_epoch
                and not self.frontend.get('fault'))
            local = (not self.p.get('navigation_output_enabled') or
                self.fresh(self.navigation_at, .6)
                and self.navigation.get('epoch') == self.core.local_epoch
                and self.navigation.get('local_fresh') is True)
            return frontend and local and self.core.local_fresh(self.now_s())

    rclpy.init()
    node = RecordedGlobalLocalizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
