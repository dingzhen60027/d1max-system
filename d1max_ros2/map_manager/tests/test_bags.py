"""All writes use temporary fixtures; no ROS node, robot or production bag touched."""
import fcntl
import json
import os
import signal
import subprocess
import sys
import tempfile
import threading
import unittest
from pathlib import Path
from unittest.mock import patch, Mock

import yaml
from fastapi import FastAPI
from fastapi.testclient import TestClient

from backend.bags.api import create_bag_router
from backend.bags.common import atomic_json, identity, owned_alive
from backend.bags.library import BagLibrary
from backend.bags.runtime import BagRecorder
from backend.bags.worker import stop_child

APP = Path(__file__).resolve().parents[1]


def fixture_bag(root, name='test_bag'):
    path = root / name
    path.mkdir()
    (path / 'part_0.db3').write_bytes(b'fixture: not a real database')
    (path / 'metadata.yaml').write_text(yaml.safe_dump({'rosbag2_bagfile_information': {
        'storage_identifier': 'sqlite3', 'duration': {'nanoseconds': 2000000000},
        'starting_time': {'nanoseconds_since_epoch': 1000000000}, 'message_count': 100,
        'relative_file_paths': ['part_0.db3'], 'topics_with_message_count': [
            {'topic_metadata': {'name': '/imu_driver/imu_central', 'type': 'sensor_msgs/msg/Imu'}, 'message_count': 100}]
    }}))
    return path


class BagTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='d1max-bag-unit-')
        self.root = Path(self.temp.name)
        self.data = self.root / 'data'; self.data.mkdir()
        self.bags = self.root / 'bags'; self.bags.mkdir()
        self.active = None
        self.library = BagLibrary([self.bags], self.data, lambda: self.active)
        self.path = fixture_bag(self.bags)
        self.key = self.library.key(self.path)

    def tearDown(self):
        self.temp.cleanup()

    def test_existing_bag_metadata_without_reading_database(self):
        item = self.library.detail(self.key)
        self.assertEqual(item['status'], 'ready')
        self.assertEqual(item['duration_sec'], 2)
        self.assertEqual(item['message_count'], 100)
        self.assertEqual(item['topics'][0]['count'], 100)

    def test_rename_note_archive_restore_do_not_modify_bag(self):
        before = (self.path / 'metadata.yaml').read_bytes()
        self.library.update(self.key, {'name': '一楼楼梯', 'note': 'test', 'archived': True})
        self.assertTrue(self.library.list()[0]['archived'])
        self.assertEqual(self.library.detail(self.key)['name'], '一楼楼梯')
        self.library.update(self.key, {'archived': False})
        self.assertFalse(self.library.list()[0]['archived'])
        self.assertEqual((self.path / 'metadata.yaml').read_bytes(), before)

    def test_delete_requires_archive_then_deletes_exact_directory(self):
        other = fixture_bag(self.bags, 'untouched')
        with self.assertRaises(RuntimeError): self.library.delete([self.key])
        self.library.update(self.key, {'archived': True})
        self.library.delete([self.key])
        self.assertFalse(self.path.exists())
        self.assertTrue(other.is_dir())
        self.assertTrue(self.bags.is_dir())

    def test_batch_prevalidates_every_target(self):
        other = fixture_bag(self.bags, 'not_archived')
        self.library.update(self.key, {'archived': True})
        with self.assertRaises(RuntimeError): self.library.delete([self.key, self.library.key(other)])
        self.assertTrue(self.path.exists())

    def test_recording_protected(self):
        self.library.update(self.key, {'archived': True})
        self.active = str(self.path)
        with self.assertRaises(RuntimeError): self.library.delete([self.key])
        with self.assertRaises(RuntimeError): self.library.update(self.key, {'name': 'no'})
        with self.assertRaises(RuntimeError): self.library.file(self.key, 'part_0.db3')

    def test_other_reader_protected(self):
        self.library.update(self.key, {'archived': True})
        with patch('backend.bags.library.open_users', return_value=[123]):
            with self.assertRaises(RuntimeError): self.library.delete([self.key])

    def test_symlinks_and_traversal_rejected(self):
        (self.bags / 'escape').symlink_to(self.path, target_is_directory=True)
        self.assertEqual(len(self.library.list()), 1)
        secret = self.root / 'secret'; secret.write_text('private')
        (self.path / 'link.db3').symlink_to(secret)
        with self.assertRaises(FileNotFoundError): self.library.file(self.key, 'link.db3')
        with self.assertRaises(FileNotFoundError): self.library.file(self.key, '../../secret')
        self.library.update(self.key, {'archived': True})
        with self.assertRaises(ValueError): self.library.delete([self.key])
        self.assertEqual(secret.read_text(), 'private')

    def test_incomplete_and_missing_chunk_visible(self):
        (self.path / 'metadata.yaml').unlink()
        self.assertEqual(self.library.detail(self.key)['status'], 'incomplete')
        other = fixture_bag(self.bags, 'missing_chunk')
        (other / 'part_0.db3').unlink()
        self.assertEqual(self.library.detail(self.library.key(other))['status'], 'incomplete')

    def test_manifest_failure_not_success(self):
        atomic_json(self.path / 'd1max_recording.json', {'error': '传感器断流'})
        self.assertEqual(self.library.detail(self.key)['status'], 'incomplete')

    def test_malformed_metadata_and_empty_bag_stay_visible(self):
        (self.path / 'metadata.yaml').write_text('rosbag2_bagfile_information: []')
        self.assertEqual(self.library.list()[0]['status'], 'incomplete')
        (self.path / 'metadata.yaml').write_text('rosbag2_bagfile_information:\n  duration: invalid\n')
        self.assertEqual(self.library.list()[0]['status'], 'incomplete')

    def test_stop_waits_for_sigint_before_escalation(self):
        process = Mock()
        process.poll.return_value = None
        self.assertFalse(stop_child(process, 7))
        process.send_signal.assert_called_once_with(signal.SIGINT)
        process.wait.assert_called_once_with(timeout=7)
        process = Mock()
        process.poll.return_value = None
        process.wait.side_effect = [subprocess.TimeoutExpired('test', 7), None]
        self.assertTrue(stop_child(process, 7))
        self.assertEqual([call.args[0] for call in process.send_signal.call_args_list], [signal.SIGINT, signal.SIGTERM])

    def test_api_confirm_validation_and_download(self):
        recorder = Mock()
        recorder.output_root = self.bags
        recorder.snapshot.return_value = {'status': 'idle', 'busy': False}
        recorder.config.return_value = {}
        app = FastAPI(); app.include_router(create_bag_router(self.library, recorder, threading.RLock()))
        with TestClient(app) as client:
            self.assertEqual(client.get('/api/bags/overview').status_code, 200)
            self.assertEqual(client.post('/api/bags/recording/stop', headers={'Origin': 'http://evil.example'}).status_code, 403)
            self.assertEqual(client.post('/api/bags/delete', json={'ids': [self.key]}).status_code, 422)
            self.assertEqual(client.patch('/api/bags/' + self.key, json={'name': '  '}).status_code, 422)
            self.assertEqual(client.get('/api/bags/' + self.key + '/file', params={'name': '../secret'}).status_code, 404)
            self.assertEqual(client.get('/api/bags/' + self.key + '/file', params={'name': 'metadata.yaml'}).status_code, 200)
            self.assertEqual(client.patch('/api/bags/' + self.key, json={'archived': True}).status_code, 200)
            self.assertEqual(client.post('/api/bags/delete', json={'ids': [self.key], 'permanent': True}).status_code, 200)

    def recorder(self):
        return BagRecorder(APP, self.root, self.root, self.data / 'runtime', self.bags)

    def test_duplicate_start_external_recorder_and_lock_conflicts(self):
        recorder = self.recorder()
        with patch.object(recorder, 'snapshot', return_value={'busy': True}):
            with self.assertRaises(RuntimeError): recorder.start('a', ['core'])
        with patch.object(recorder, 'other_recorders', return_value=['123']):
            with self.assertRaises(RuntimeError): recorder.start('a', ['core'])
        with (recorder.root / 'recorder.lock').open('a+') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            with self.assertRaises(RuntimeError): recorder.start('a', ['core'])

    def test_low_disk_and_invalid_groups(self):
        recorder = self.recorder()
        with patch('backend.bags.runtime.shutil.disk_usage', return_value=Mock(free=100)):
            with self.assertRaises(RuntimeError): recorder.start('a', ['core'])
        for groups in ([], ['core', 'unknown']):
            with self.assertRaises(ValueError): recorder.start('a', groups)

    def test_stale_pid_never_kills_unrelated_process(self):
        recorder = self.recorder()
        path = recorder.root / 'job.json'; atomic_json(path, {})
        atomic_json(recorder.root / 'active.json', {'pid': os.getpid(), 'start_ticks': identity(os.getpid()),
                    'job_path': str(path), 'output_path': str(self.path)})
        atomic_json(recorder.root / 'status.json', {'status': 'recording'})
        self.assertFalse(owned_alive(json.loads((recorder.root / 'active.json').read_text())))
        with patch('os.killpg') as kill:
            self.assertEqual(recorder.snapshot()['status'], 'interrupted')
            recorder.stop(); recorder.close()
            kill.assert_not_called()

    def test_repeated_stop_is_idempotent(self):
        recorder = self.recorder()
        path = recorder.root / 'job.json'
        atomic_json(recorder.root / 'active.json', {'job_path': str(path), 'output_path': str(self.path)})
        with patch('backend.bags.runtime.owned_alive', return_value=True):
            recorder.stop(); recorder.stop()
            self.assertTrue(path.with_name('stop.request').is_file())
            self.assertEqual(recorder.snapshot()['status'], 'stopping')


if __name__ == '__main__':
    unittest.main()
