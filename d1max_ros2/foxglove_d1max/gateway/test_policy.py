import unittest
from policy import Policy

CONFIG = dict(state_stale_seconds=2.5, lease_seconds=1.5, command_timeout_seconds=0.25,
              max_forward=0.30, max_lateral=0.20, max_yaw=0.50)


class SafetyTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.0
        self.p = Policy(CONFIG, live=True, monotonic=lambda: self.now)
        self.p.robot = dict(control_source=2, motion_status=5, sport_mode=1,
                            software_emergency_status=1, hardware_emergency_status=1)
        self.p.behavior = dict(fault_latched=False, ready_for_navigation=True,
                              control_adapter="guarded_console_v1", sdk_has_control=True,
                              goal_status="IDLE", goal_id=0, available_actions={"stand": ""})
        self.p.robot_at = self.p.behavior_at = self.p.connection_at = self.now
        self.p.connected = True

    def command(self, **kwargs):
        return dict(session=self.p.session, stamp=100.0, seq=1, x=0.2, y=0.0, yaw=0.0, **kwargs)

    def test_startup_never_armed(self):
        self.assertFalse(self.p.armed)
        self.assertIsNone(self.p.velocity(self.command(), 100.0))

    def test_readonly_rejects_every_action(self):
        self.p.live = False
        for action in ("stand", "soft_estop", "halt", "recover_estop"):
            self.assertTrue(self.p.action_reason(action))

    def test_replay_rejects_all(self):
        self.p.arm()
        self.p.replay = True
        self.assertFalse(self.p.armed)
        self.assertTrue(self.p.action_reason("soft_estop"))
        self.assertIsNone(self.p.velocity(self.command(), 100.0))

    def test_lease_expiry_does_not_rearm_on_keepalive(self):
        self.p.arm()
        self.assertEqual(self.p.velocity(self.command(), 100), [0.2, 0.0, 0.0])
        self.now += 1.6
        self.assertTrue(self.p.renew())
        self.assertTrue(self.p.should_stop())

    def test_command_timeout(self):
        self.p.arm()
        self.p.velocity(self.command(), 100)
        self.now += 0.26
        self.assertTrue(self.p.should_stop())
        self.p.stopped()
        self.assertFalse(self.p.should_stop())

    def test_reject_recorded_wrong_session_stale_future_and_duplicate(self):
        self.p.arm()
        command = self.command()
        for field, value in (("session", "oldsession"), ("stamp", 99), ("stamp", 101), ("seq", -1), ("x", float("nan")), ("x", 2.0), ("x", True)):
            bad = {**command, field: value}
            self.assertIsNone(self.p.velocity(bad, 100))
        self.assertIsNotNone(self.p.velocity(command, 100))
        self.assertIsNone(self.p.velocity(command, 100))

    def test_stale_state_blocks_motion_but_stop_still_requestable(self):
        self.p.arm()
        self.p.robot_at = 0
        self.assertFalse(self.p.armed)
        self.assertTrue(self.p.action_reason("stand"))
        self.assertEqual(self.p.action_reason("soft_estop"), "")

    def test_unknown_fault_state_blocks_arm(self):
        self.p.behavior = {}
        self.assertTrue(self.p.arm())

    def test_emergency_and_lost_control_block_velocity(self):
        for field, value in (("control_source", 1), ("software_emergency_status", 2), ("hardware_emergency_status", 0), ("motion_status", 2)):
            original = self.p.robot[field]
            self.p.robot[field] = value
            self.p.arm()
            self.assertIsNone(self.p.velocity(self.command(), 100))
            self.p.robot[field] = original

    def test_sessions_unique(self):
        other = Policy(CONFIG)
        self.assertNotEqual(self.p.session, other.session)

    def test_transition_does_not_expire_lease_but_blocks_commands(self):
        self.p.arm()
        self.p.behavior["goal_status"] = "TRANSITIONING"
        self.p.robot["motion_status"] = 1
        self.assertEqual(self.p.renew(), "")
        self.assertTrue(self.p.action_reason("stand"))
        self.assertIsNone(self.p.velocity(self.command(), 100))

    def test_ack_gap_blocks_until_matching_feedback(self):
        self.p.arm()
        self.p.pending_goal_id = 3
        self.assertTrue(self.p.busy)
        self.p.behavior.update(goal_id=3, goal_status="SUCCEEDED")
        self.assertFalse(self.p.busy)

    def test_backend_denial_and_unknown_action(self):
        self.p.arm()
        self.p.behavior["available_actions"]["stand"] = "站立中"
        self.assertEqual(self.p.action_reason("stand"), "站立中")
        self.assertTrue(self.p.action_reason("unknown"))

    def test_telemetry_observer_never_arms(self):
        self.p.behavior["control_adapter"] = "observer"
        self.assertTrue(self.p.arm())

    def test_stop_on_control_loss_even_before_behavior_update(self):
        self.p.arm()
        self.p.velocity(self.command(), 100)
        self.p.robot["control_source"] = 1
        self.assertTrue(self.p.should_stop())

    def test_transport_timeout_stays_locked(self):
        self.p.arm()
        self.p.transport_error = "unknown execution"
        self.assertFalse(self.p.armed)
        self.assertTrue(self.p.arm())
        self.assertEqual(self.p.action_reason("soft_estop"), "")


if __name__ == "__main__":
    unittest.main()
