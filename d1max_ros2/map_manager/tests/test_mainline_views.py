"""Owned RViz lifecycle contracts; all subprocesses and systemd are mocked."""
import os
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

from backend.mainline_views import (CORE_OWNER, CORE_UNIT, VIEW_OWNER, VIEW_UNIT, VIEW_UNITS,
    LEGACY_VIEW_UNITS, VIEW_CONTRACT, LAYOUT_REQUEST_FILE, LAYOUT_STATE_FILE, OwnedNavigationViews)


class MockSystem:
    @staticmethod
    def populated(unit):
        return unit.get('populated') == '1'


class MainlineViewTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.nav = Path(temporary.name) / 'nav'
        self.release = self.nav / 'experiments/sealed'
        self.release.mkdir(parents=True)
        self.directory = Path(temporary.name) / 'sessions/current'
        self.directory.mkdir(parents=True)
        self.entry = self.nav / 'tools/single_floor_entry.sh'
        self.entry.parent.mkdir()
        self.entry.write_text('mock entry only')
        self.entry.chmod(0o700)
        self.state = dict(id='session-1', directory=str(self.directory), release_root=str(self.release),
                          pid=123, invocation_id='core-invocation', views={}, view_contract=VIEW_CONTRACT)
        self.units = {name: dict(ActiveState='inactive', Description='', MainPID='0', InvocationID='',
                                ControlGroup='', ExecMainStatus='0', Result='success')
                      for name in [CORE_UNIT, VIEW_UNIT, *LEGACY_VIEW_UNITS.values()]}
        self.units[CORE_UNIT].update(ActiveState='active', Description=CORE_OWNER + self.state['id'],
                                     MainPID='123', InvocationID='core-invocation')
        self.calls = []
        self.launch_count = 0
        self.fail_launch = False
        self.bad_launch_identity = False
        self.stop_residual = False
        self.views = OwnedNavigationViews(self.nav, MockSystem(), self.run_mock)
        self.acknowledge = True
        self.on_request = None
        self.request_count = 0
        self.original_writer = self.views._write_request
        self.views._write_request = self.write_request
        self.environment = patch.dict(os.environ, {'DISPLAY': ':1', 'XAUTHORITY': '/tmp/auth',
                                                   'UNRELATED_SECRET': 'do-not-forward',
                                                   'D1MAX_NAV_TRANSPORT': 'isolated_mock'}, clear=True)
        self.environment.start()
        self.addCleanup(self.environment.stop)

    def write_request(self, directory, request):
        self.original_writer(directory, request)
        self.request_count += 1
        if self.acknowledge:
            path = directory / LAYOUT_STATE_FILE
            path.write_text(json.dumps({**request, 'phase': 'applied',
                                       'applied_at_unix': time.time(), 'rviz_pid': 800}))
            path.chmod(0o600)
        if self.on_request:
            self.on_request()

    def run_mock(self, command, **options):
        self.calls.append((command, options))
        if command[:3] == ['systemctl', '--user', 'show']:
            return subprocess.CompletedProcess(command, 0,
                '\n'.join(key + '=' + value for key, value in self.units[command[3]].items()))
        if command[0] == 'systemd-run':
            self.launch_count += 1
            if self.fail_launch:
                raise subprocess.TimeoutExpired(command, 10)
            unit = next(value.split('=', 1)[1] for value in command if value.startswith('--unit='))
            description = next(value.split('=', 2)[2] for value in command
                               if value.startswith('--property=Description='))
            self.units[unit].update(ActiveState='active', Description=description,
                                    MainPID=str(500 + self.launch_count), InvocationID='view-' + str(self.launch_count))
            if self.bad_launch_identity:
                self.units[unit]['InvocationID'] = ''
        if command[:3] == ['systemctl', '--user', 'stop']:
            self.units[command[3]].update(ActiveState='inactive', MainPID='0',
                                         populated='1' if self.stop_residual else '0')
        self.assertTrue(options.get('check'))
        self.assertLessEqual(options['timeout'], 16)
        return subprocess.CompletedProcess(command, 0, '')

    def open(self, layout='global'):
        record = self.views.open(self.state, layout)
        self.state['views']['single'] = record
        return record

    def launches(self):
        return [command for command, _ in self.calls if command[0] == 'systemd-run']

    def stops(self):
        return [command for command, _ in self.calls if command[:3] == ['systemctl', '--user', 'stop']]

    def legacy(self, layout):
        name = LEGACY_VIEW_UNITS[layout]
        description = VIEW_OWNER + self.state['id'] + ':' + layout
        self.units[name].update(ActiveState='active', Description=description,
                                MainPID='900', InvocationID='legacy-' + layout)
        self.state['views'][layout] = dict(unit=name, layout=layout, core_id=self.state['id'],
            directory=str(self.directory), release_root=str(self.release), description=description,
            pid=900, invocation_id='legacy-' + layout, phase='running')

    def test_open_uses_same_sealed_mainline_and_explicit_live_environment(self):
        record = self.open()
        command = self.launches()[0]
        self.assertEqual(command[-6:], [str(self.entry), 'view', '--session', str(self.directory), '--layout', 'global'])
        self.assertEqual(record['core_id'], self.state['id'])
        self.assertEqual(record['invocation_id'], 'view-1')
        self.assertIn('--property=BindsTo=' + CORE_UNIT, command)
        self.assertIn('--property=KillMode=control-group', command)
        self.assertNotIn('--property=PartOf=' + CORE_UNIT, command)
        self.assertIn('--setenv=D1MAX_NAV_ROOT=' + str(self.nav), command)
        self.assertIn('--setenv=D1MAX_RELEASE=' + str(self.release), command)
        self.assertIn('--setenv=D1MAX_NAV_TRANSPORT=live', command)
        self.assertIn('--setenv=DISPLAY=:1', command)
        self.assertIn('--unit=' + VIEW_UNIT, command)
        self.assertIn('--setenv=D1MAX_NAV_RVIZ_VIEWER_ID=' + record['viewer_id'], command)
        self.assertFalse(any('UNRELATED_SECRET' in value or 'isolated_mock' in value for value in command))
        self.assertFalse(any(value in ('connect', 'run', 'prepare', 'seal') for value in command))

    def test_view_uses_session_bound_public_entry_not_a_new_selection(self):
        entry = self.nav / 'tools/navigation_entry.sh'
        entry.write_text('mock generic dispatcher only'); entry.chmod(0o700)
        self.state.update(entrypoint=str(entry),
                          entrypoint_sha256=hashlib.sha256(entry.read_bytes()).hexdigest(),
                          compatibility_entrypoint_sha256=hashlib.sha256(self.entry.read_bytes()).hexdigest())
        self.open()
        self.assertEqual(self.launches()[0][-6], str(entry))
        self.assertEqual(self.units[CORE_UNIT]['Description'], CORE_OWNER + self.state['id'])

    def test_changed_or_unpinned_public_entry_cannot_open_a_view(self):
        entry = self.nav / 'tools/navigation_entry.sh'
        entry.write_text('mock generic dispatcher only'); entry.chmod(0o700)
        self.state['entrypoint'] = str(entry)
        with self.assertRaisesRegex(ValueError, 'view_entrypoint_changed'):
            self.open()
        self.state['entrypoint_sha256'] = hashlib.sha256(entry.read_bytes()).hexdigest()
        entry.write_text('changed dispatcher')
        with self.assertRaisesRegex(ValueError, 'view_entrypoint_changed'):
            self.open()
        self.assertEqual(self.launches(), [])

    def test_public_entry_dependency_change_does_not_switch_view_implementation(self):
        entry = self.nav / 'tools/navigation_entry.sh'
        entry.write_text('mock generic dispatcher only'); entry.chmod(0o700)
        self.state.update(entrypoint=str(entry),
                          entrypoint_sha256=hashlib.sha256(entry.read_bytes()).hexdigest())
        with self.assertRaisesRegex(ValueError, 'view_entrypoint_dependency_changed'):
            self.open()
        self.state['compatibility_entrypoint_sha256'] = hashlib.sha256(self.entry.read_bytes()).hexdigest()
        self.entry.write_text('changed implementation dispatcher')
        with self.assertRaisesRegex(ValueError, 'view_entrypoint_dependency_changed'):
            self.open()
        self.assertEqual(self.launches(), [])

    def test_duplicate_open_is_idempotent_and_can_recover_exact_owned_record(self):
        original = self.open()
        duplicate = self.open()
        self.assertNotEqual(duplicate['request_id'], original['request_id'])
        self.state['views'] = {}
        recovered = self.open()
        for record in (duplicate, recovered):
            for key in ('unit', 'pid', 'invocation_id', 'viewer_id', 'layout', 'rviz_pid'):
                self.assertEqual(record[key], original[key])
        self.assertEqual(self.launch_count, 1)

    def test_two_layouts_switch_the_same_process_and_invocation(self):
        global_view = self.open('global')
        local_view = self.open('local')
        self.assertEqual(global_view['unit'], VIEW_UNIT)
        for key in ('unit', 'pid', 'invocation_id', 'viewer_id'):
            self.assertEqual(global_view[key], local_view[key])
        self.assertEqual(local_view['layout'], 'local')
        self.assertEqual(local_view['layout_state'], 'applied')
        self.assertEqual(self.open('global')['pid'], local_view['pid'])
        self.assertEqual(self.launch_count, 1)
        self.assertFalse(self.stops())

    def test_concurrent_duplicate_open_and_switch_have_one_launch_and_one_writer(self):
        entered = threading.Event()
        release = threading.Event()
        result = []

        def hold():
            entered.set()
            self.assertTrue(release.wait(3))

        self.on_request = hold
        first = threading.Thread(target=lambda: result.append(self.views.open(self.state, 'global')))
        first.start()
        self.addCleanup(lambda: first.join(3))
        self.addCleanup(release.set)
        self.assertTrue(entered.wait(3))
        for layout in ('global', 'local'):
            with self.assertRaisesRegex(ValueError, 'view_lifecycle_busy'):
                self.views.open(self.state, layout)
        # A second Web object shares the same private file lock.
        other = OwnedNavigationViews(self.nav, MockSystem(), self.run_mock)
        with self.assertRaisesRegex(ValueError, 'view_lifecycle_busy'):
            other.open(self.state, 'local')
        release.set()
        first.join(3)
        self.assertFalse(first.is_alive())
        self.on_request = None
        self.state['views']['single'] = result[0]
        self.assertEqual(self.open('local')['pid'], result[0]['pid'])
        self.assertEqual(self.launch_count, 1)
        self.assertEqual(self.request_count, 2)
        self.assertFalse(self.stops())

    def test_failed_switch_is_pending_without_relaunch_or_navigation_change(self):
        original = self.open()
        self.acknowledge = False
        with patch('backend.mainline_views.time.monotonic', side_effect=[0, 3]):
            pending = self.open('local')
        self.assertEqual(pending['layout_state'], 'pending')
        self.assertEqual(pending['layout'], 'global')
        self.assertEqual(pending['requested_layout'], 'local')
        self.assertEqual(pending['pid'], original['pid'])
        self.assertEqual(self.launch_count, 1)
        self.assertFalse(self.stops())
        self.assertEqual(self.units[CORE_UNIT]['MainPID'], '123')

    def test_delayed_ack_recovers_pending_switch_with_same_pid(self):
        self.open()
        self.acknowledge = False
        with patch('backend.mainline_views.time.monotonic', side_effect=[0, 3]):
            pending = self.open('local')
        request = json.loads((self.directory / LAYOUT_REQUEST_FILE).read_text())
        ack_path = self.directory / LAYOUT_STATE_FILE
        ack_path.write_text(json.dumps({**request, 'phase': 'applied',
                                      'applied_at_unix': time.time(), 'rviz_pid': 800}))
        ack_path.chmod(0o600)
        self.acknowledge = True
        applied = self.open('local')
        self.assertEqual(applied['layout_state'], 'applied')
        self.assertEqual(applied['pid'], pending['pid'])
        self.assertEqual(self.launch_count, 1)
        self.assertEqual(self.request_count, 3)

    def test_old_session_allows_one_global_view_and_rejects_local_switch(self):
        self.state.pop('view_contract')
        original = self.open()
        with self.assertRaisesRegex(ValueError, 'view_layout_switch_unsupported'):
            self.open('local')
        self.assertEqual(self.open()['pid'], original['pid'])
        self.assertEqual(self.launch_count, 1)
        self.assertFalse(self.stops())

    def test_old_owned_view_blocks_new_launch_until_explicit_cleanup(self):
        self.legacy('global')
        self.legacy('local')
        with self.assertRaisesRegex(ValueError, 'legacy_view_conflict_needs_upgrade'):
            self.open('global')
        self.assertFalse(self.launches())
        self.assertFalse(self.stops())
        self.views.stop(self.state)
        self.open('local')
        self.assertEqual(self.launch_count, 1)

    def test_two_legacy_views_with_foreign_replacement_never_partially_cleanup(self):
        self.legacy('global')
        self.legacy('local')
        self.units[LEGACY_VIEW_UNITS['local']]['InvocationID'] = 'replacement'
        with self.assertRaisesRegex(ValueError, 'view_owner_mismatch:local'):
            self.views.stop(self.state)
        self.assertFalse(self.stops())

    def test_core_or_view_replacement_during_ack_is_rejected_without_relaunch(self):
        original = self.open()
        self.on_request = lambda: self.units[VIEW_UNIT].update(InvocationID='replacement')
        with self.assertRaisesRegex(ValueError, 'view_owner_mismatch:single'):
            self.open('local')
        self.units[VIEW_UNIT]['InvocationID'] = original['invocation_id']
        self.on_request = lambda: self.units[CORE_UNIT].update(InvocationID='replacement')
        with self.assertRaisesRegex(ValueError, 'navigation_core_owner_mismatch'):
            self.open('global')
        self.assertEqual(self.launch_count, 1)
        self.assertFalse(self.stops())

    def test_layout_request_is_private_bounded_and_binds_the_view_nonce(self):
        record = self.open()
        path = self.directory / LAYOUT_REQUEST_FILE
        request = json.loads(path.read_text())
        self.assertEqual(set(request), {'schema', 'session_id', 'viewer_id', 'request_id', 'layout', 'stamp'})
        self.assertEqual(request['viewer_id'], record['viewer_id'])
        self.assertEqual(request['session_id'], self.state['id'])
        self.assertEqual(path.stat().st_mode & 0o777, 0o600)
        self.assertLess(path.stat().st_size, 4096)

    def test_non_private_or_symlink_request_is_not_overwritten(self):
        self.open()
        path = self.directory / LAYOUT_REQUEST_FILE
        path.chmod(0o666)
        with self.assertRaisesRegex(ValueError, 'view_layout_file_not_private'):
            self.open('local')
        path.unlink()
        outside = self.directory.parent / 'outside.json'
        outside.write_text('{}')
        path.symlink_to(outside)
        with self.assertRaises(OSError):
            self.open('local')
        self.assertEqual(outside.read_text(), '{}')
        self.assertEqual(self.launch_count, 1)
        self.assertFalse(self.stops())

    def test_fifo_layout_file_is_rejected_without_blocking(self):
        self.open()
        path = self.directory / LAYOUT_STATE_FILE
        path.unlink()
        os.mkfifo(path, 0o600)
        with self.assertRaisesRegex(ValueError, 'view_layout_file_not_private'):
            self.open('local')
        self.assertEqual(self.launch_count, 1)
        self.assertFalse(self.stops())

    def test_wrong_nonce_or_request_ack_does_not_confirm_switch(self):
        self.open()
        self.acknowledge = False

        def wrong_ack():
            path = self.directory / LAYOUT_STATE_FILE
            request = json.loads((self.directory / LAYOUT_REQUEST_FILE).read_text())
            path.write_text(json.dumps({**request, 'viewer_id': 'f' * 32,
                'phase': 'applied', 'applied_at_unix': time.time(), 'rviz_pid': 800}))
            path.chmod(0o600)

        self.on_request = wrong_ack
        with patch('backend.mainline_views.time.monotonic', side_effect=[0, 3]):
            self.assertEqual(self.open('local')['layout_state'], 'pending')
        self.assertEqual(self.launch_count, 1)
        self.assertFalse(self.stops())

    def test_ack_from_replaced_rviz_child_is_not_accepted(self):
        self.open()

        def replaced():
            path = self.directory / LAYOUT_STATE_FILE
            ack = json.loads(path.read_text())
            ack['rviz_pid'] = 999
            path.write_text(json.dumps(ack))

        self.on_request = replaced
        with self.assertRaisesRegex(ValueError, 'view_owner_mismatch:single'):
            self.open('local')
        self.assertEqual(self.launch_count, 1)
        self.assertFalse(self.stops())

    def test_saved_session_hash_change_prevents_layout_request(self):
        path = self.directory / 'session.json'
        path.write_text('{"id":"session-1"}')
        self.state['session_sha256'] = hashlib.sha256(path.read_bytes()).hexdigest()
        self.open()
        path.write_text('{"id":"changed"}')
        with self.assertRaisesRegex(ValueError, 'view_session_binding_changed'):
            self.open('local')
        self.assertEqual(self.request_count, 1)
        self.assertEqual(self.launch_count, 1)

    def test_closed_view_can_reopen_without_cancelling_core(self):
        original = self.open()
        self.units[VIEW_UNITS['global']].update(ActiveState='inactive', MainPID='0')
        reopened = self.open()
        self.assertNotEqual(reopened['invocation_id'], original['invocation_id'])
        self.assertEqual(self.units[CORE_UNIT]['MainPID'], '123')
        self.assertFalse(self.stops())

    def test_view_close_does_not_send_navigation_cancel_or_stop_sdk(self):
        self.open()
        self.views.stop(self.state)
        self.assertEqual(self.stops(), [['systemctl', '--user', 'stop', VIEW_UNITS['global']]])
        self.assertEqual(self.units[CORE_UNIT]['ActiveState'], 'active')

    def test_core_closed_prevents_open_but_still_allows_view_cleanup(self):
        self.open()
        self.units[CORE_UNIT].update(ActiveState='inactive', MainPID='0')
        with self.assertRaisesRegex(ValueError, 'navigation_core_not_running'):
            self.views.open(self.state, 'local')
        self.views.stop(self.state)
        self.assertEqual(len(self.stops()), 1)

    def test_wrong_core_identity_or_invocation_prevents_open(self):
        for key, wrong in [('Description', CORE_OWNER + 'other'), ('MainPID', '999'),
                           ('InvocationID', 'new-core')]:
            with self.subTest(key=key):
                original = self.units[CORE_UNIT][key]
                self.units[CORE_UNIT][key] = wrong
                with self.assertRaisesRegex(ValueError, 'navigation_core_owner_mismatch'):
                    self.views.open(self.state, 'global')
                self.units[CORE_UNIT][key] = original
        self.assertFalse(self.launches())

    def test_active_foreign_view_never_adopted_or_stopped(self):
        self.units[VIEW_UNITS['global']].update(ActiveState='active', Description=VIEW_OWNER + 'other:global',
                                               MainPID='700', InvocationID='foreign')
        with self.assertRaisesRegex(ValueError, 'view_owner_mismatch:single'):
            self.views.open(self.state, 'global')
        with self.assertRaisesRegex(ValueError, 'view_owner_mismatch:single'):
            self.views.stop(self.state)
        self.assertFalse(self.launches())
        self.assertFalse(self.stops())

    def test_replacement_same_description_old_record_invocation_is_not_stopped(self):
        self.open()
        self.units[VIEW_UNITS['global']]['InvocationID'] = 'new-invocation'
        with self.assertRaisesRegex(ValueError, 'view_owner_mismatch:single'):
            self.views.open(self.state, 'global')
        with self.assertRaisesRegex(ValueError, 'view_owner_mismatch:single'):
            self.views.stop(self.state)
        self.assertFalse(self.stops())

    def test_replacement_same_invocation_wrong_pid_is_not_stopped(self):
        self.open()
        self.units[VIEW_UNITS['global']]['MainPID'] = '900'
        with self.assertRaisesRegex(ValueError, 'view_owner_mismatch:single'):
            self.views.stop(self.state)
        self.assertFalse(self.stops())

    def test_stop_preflights_all_groups_before_touching_own_view(self):
        self.open('global')
        self.units[LEGACY_VIEW_UNITS['local']].update(ActiveState='active', Description='foreign',
                                              MainPID='800', InvocationID='foreign')
        with self.assertRaisesRegex(ValueError, 'view_owner_mismatch:local'):
            self.views.stop(self.state)
        self.assertFalse(self.stops())

    def test_cleanup_of_both_legacy_owned_views_validates_before_stop(self):
        self.legacy('global')
        self.legacy('local')
        result = self.views.stop(self.state)
        self.assertEqual(set(result), {'global', 'local'})
        self.assertTrue(all(record['phase'] == 'stopped' for record in result.values()))
        self.assertEqual(self.views.stop(self.state), {})
        self.assertEqual(len(self.stops()), 2)

    def test_no_display_is_clear_warning_without_stopping_core(self):
        os.environ.pop('DISPLAY')
        with self.assertRaisesRegex(ValueError, '^desktop_unavailable$'):
            self.views.open(self.state, 'global')
        self.assertFalse(self.launches())
        self.assertFalse(self.stops())

    def test_existing_view_is_idempotent_even_after_web_loses_display(self):
        record = self.open()
        os.environ.pop('DISPLAY')
        duplicate = self.open()
        self.assertEqual(duplicate['pid'], record['pid'])
        self.assertEqual(duplicate['layout_state'], 'applied')
        self.assertEqual(self.launch_count, 1)

    def test_unknown_layout_rejected_before_external_operation(self):
        with self.assertRaisesRegex(ValueError, 'unknown_view_layout'):
            self.views.open(self.state, 'other')
        self.assertFalse(self.calls)

    def test_missing_release_or_invalid_binding_does_not_launch(self):
        for key, value in [('id', '../other'), ('directory', 'relative'), ('release_root', str(self.directory))]:
            with self.subTest(key=key):
                original = self.state[key]
                self.state[key] = value
                with self.assertRaisesRegex(ValueError, 'view_session_binding_invalid'):
                    self.views.open(self.state, 'global')
                self.state[key] = original
        self.assertFalse(self.launches())

    def test_new_launch_missing_invocation_is_unconfirmed_without_kill_or_retry(self):
        self.bad_launch_identity = True
        with self.assertRaisesRegex(ValueError, 'view_identity_unverified:single'):
            self.views.open(self.state, 'global')
        self.assertEqual(self.launch_count, 1)
        self.assertFalse(self.stops())

    def test_uncertain_launch_is_not_retried_or_cleanup_killed(self):
        self.fail_launch = True
        with self.assertRaises(subprocess.TimeoutExpired):
            self.views.open(self.state, 'global')
        self.assertEqual(self.launch_count, 1)
        self.assertFalse(self.stops())

    def test_residual_group_is_not_reported_stopped(self):
        self.open()
        self.stop_residual = True
        with self.assertRaisesRegex(ValueError, 'view_stop_unconfirmed:single'):
            self.views.stop(self.state)
        self.assertEqual(self.units[CORE_UNIT]['ActiveState'], 'active')

    def test_saved_exact_deactivating_view_is_cleanup_owned_without_core(self):
        self.open()
        self.units[CORE_UNIT].update(ActiveState='inactive', MainPID='0')
        self.units[VIEW_UNITS['global']].update(ActiveState='deactivating', MainPID='0', populated='1')
        result = self.views.stop(self.state)
        self.assertEqual(result['single']['phase'], 'stopped')
        self.assertEqual(self.stops(), [['systemctl', '--user', 'stop', VIEW_UNITS['global']]])

    def test_residual_view_without_saved_invocation_is_not_adopted_for_stop(self):
        self.open()
        self.state['views'] = {}
        self.units[VIEW_UNITS['global']].update(ActiveState='inactive', MainPID='0', populated='1')
        with self.assertRaisesRegex(ValueError, 'view_identity_unverified:single'):
            self.views.stop(self.state)
        self.assertFalse(self.stops())

    def test_deactivating_replacement_invocation_not_stopped(self):
        self.open()
        self.units[VIEW_UNITS['global']].update(ActiveState='deactivating', MainPID='0',
                                               populated='1', InvocationID='replacement')
        with self.assertRaisesRegex(ValueError, 'view_owner_mismatch:single'):
            self.views.stop(self.state)
        self.assertFalse(self.stops())

    def test_non_private_lifecycle_lock_is_rejected(self):
        path = self.nav / 'log/.navigation-view-switch.lock'
        path.parent.mkdir()
        path.write_text('')
        path.chmod(0o666)
        with self.assertRaisesRegex(ValueError, 'view_lifecycle_lock_not_private'):
            self.views.open(self.state, 'global')
        self.assertFalse(self.calls)

    def test_busy_lifecycle_does_not_launch(self):
        self.views.lock.acquire()
        try:
            with self.assertRaisesRegex(ValueError, 'view_lifecycle_busy'):
                self.views.open(self.state, 'global')
        finally:
            self.views.lock.release()
        self.assertFalse(self.calls)

    def test_invalid_desktop_value_not_forwarded(self):
        os.environ['DISPLAY'] = ':1\ninvalid'
        with self.assertRaisesRegex(ValueError, 'desktop_environment_invalid'):
            self.views.open(self.state, 'global')
        self.assertFalse(self.launches())


if __name__ == '__main__':
    unittest.main()
