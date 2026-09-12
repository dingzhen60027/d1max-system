import unittest
from scripts.monitor_health import mc_health


class McHealthTests(unittest.TestCase):
    def setUp(self):
        self.report = dict(source='sdk_mc', received_at_unix=100., stream_fresh=True,
                           observed_hz=50., rate_ok=True, acknowledged=True, ack_on=True, state='streaming')

    def test_complete_stream_is_ready(self):
        self.assertTrue(mc_health(self.report, 100.2)['mc_ready'])

    def test_handoff_is_visible_but_not_a_substitute_for_mc(self):
        owner = {'enabled': True, 'state': 'confirmed', 'confirmed': True}
        report = {**self.report, 'ownership': owner, 'stream_fresh': False, 'observed_hz': 0.}
        health = mc_health(report, 100.2)
        self.assertEqual(health['speed_report']['ownership'], owner)
        self.assertFalse(health['mc_ready'])

    def test_incomplete_stream_is_not_ready(self):
        for extra in ({'stream_fresh':False}, {'acknowledged':False}, {'ack_on':False},
                      {'rate_ok':False}, {'source':'robot_state'}, {'received_at_unix':98.},
                      {'received_at_unix':101.}, {'observed_hz':float('nan')}):
            self.assertFalse(mc_health({**self.report, **extra}, 100.2)['mc_ready'])
        for value in (None, [], {}, {'received_at_unix': 'bad'}):
            self.assertFalse(mc_health(value, 100.2)['mc_ready'])

    def test_disconnect_replay_and_stale_file_fail_closed(self):
        for connected, replay in ((False, False), (True, True)):
            self.assertFalse(mc_health(self.report, 100.2, connected, replay)['mc_ready'])
        self.assertEqual(mc_health(self.report, 102.)['speed_report'], {})

    def test_zero_hz_retry_status_is_visible_without_localization(self):
        r = {**self.report, 'stream_fresh':False, 'observed_hz':0., 'state':'retry_cooldown', 'next_retry_sec':12.}
        health = mc_health(r, 100.2)
        self.assertFalse(health['mc_ready'])
        self.assertEqual(health['speed_report']['next_retry_sec'], 12.)


if __name__ == '__main__':
    unittest.main()
