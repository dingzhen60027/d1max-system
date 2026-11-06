import json
from pathlib import Path
import tempfile
import unittest
from d1max_pct_scan.diagnostic_session import trace_environment


class DiagnosticSessionTest(unittest.TestCase):
    def test_index_and_fresh_retry_no_configuration_change(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root/'session.json').write_text('{"id":"test"}')
            original = {'RMW_IMPLEMENTATION':'rmw_zenoh_cpp', 'ROS_DOMAIN_ID':'24'}
            session = {'id':'test', 'expected_sdk_session':'sdk-same', 'version_id':'map-same'}
            first = trace_environment(root, session, {'tracker':['unchanged']}, environment=original)
            second = trace_environment(root, session, {'tracker':['unchanged']}, environment=original)
            self.assertNotEqual(first['D1MAX_TRACE_DIRECTORY'], second['D1MAX_TRACE_DIRECTORY'])
            self.assertEqual({k:v for k,v in first.items() if k != 'D1MAX_TRACE_DIRECTORY'}, original)
            index = json.loads((Path(first['D1MAX_TRACE_DIRECTORY'])/'session-index.json').read_text())
            self.assertEqual(index['expected_sdk_session'], 'sdk-same')
            self.assertEqual(index['component_trace_limit_bytes'], 17039360)
            self.assertEqual(index['task_trace_retention']['task_buckets'], 4)
            self.assertEqual(index['task_trace_retention']['post_task_seconds'], 10)
            self.assertEqual(index['component_logs']['tracker'], str(root/'tracker.log'))
            self.assertNotIn('D1MAX_TRACE_DIRECTORY', original)

    def test_disk_failure_does_not_block_launch(self):
        with tempfile.TemporaryDirectory() as tmp:
            blocked = Path(tmp)/'file'
            blocked.write_text('keep')
            self.assertEqual(trace_environment(blocked, {'id':'s'}, {}, environment={'TEST':'1'}), {'TEST':'1'})
            self.assertEqual(blocked.read_text(), 'keep')


if __name__ == '__main__':
    unittest.main()
