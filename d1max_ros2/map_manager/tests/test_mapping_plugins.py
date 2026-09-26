"""No robot, ROS nodes, or real systemd mutations."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch
import yaml
from backend.mapping.configuration import MolaConfig, parse_config, validate_bag, config_yaml
from backend.mapping.registry import MappingRuntime
from backend.mapping.mola_runtime import MolaRuntime, MARKER, atomic_json
from backend.mapping.worker import prepare, Worker, Cancelled, check_native_log, loop_summary
from backend.mapping.artifacts import completed_artifacts

APP = Path(__file__).resolve().parents[1]


class MappingTests(unittest.TestCase):
    def test_schema_blocks_expressions_unknown_fields_planar_and_nonfinite(self):
        for change in ({'arbitrary_command': 'touch /tmp/no'},
                       {'input': {'bag_path': '/tmp/$(touch nope)'}},
                       {'input': {'lidar_topic': '/lidar.*'}},
                       {'loop_closure': {'assume_planar_world': True}},
                       {'sensors': {'imu_pose': [0, 0, 0, float('nan'), 0, 0]}}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                parse_config(yaml.safe_dump(change))
        with self.assertRaises(ValueError):
            parse_config('!!python/object/apply:os.system [echo nope]')

    def test_config_roundtrip(self):
        config = parse_config((APP / 'config/mapping/mola_lio_lc.yaml').read_text())
        self.assertEqual(parse_config(config_yaml(config)), config)
        self.assertFalse(config.loop_closure.assume_planar_world)

    def test_bad_types_and_missing_bag_are_actionable(self):
        for value in (None, {}, 1):
            with self.assertRaises(ValueError):
                parse_config(value)
        with self.assertRaises(ValueError):
            parse_config('input: null', '/bag')
        with self.assertRaises(ValueError):
            validate_bag(MolaConfig(input={'bag_path': '/does-not-exist-d1max-bag'}))

    def test_native_exit_zero_does_not_hide_internal_failure(self):
        with tempfile.TemporaryDirectory() as tmp:
            log = Path(tmp) / 'lio.log'
            log.write_text('[INFO] reading bag\n[ERROR|LidarOdometry] Exception:\nMessage: RKNN failed\n')
            with self.assertRaisesRegex(RuntimeError, 'RKNN failed'):
                check_native_log(log)
            log.write_text('Total accepted loop closures: 4 (GNC: 3 inliers, 1 outliers rejected)\n')
            self.assertEqual(loop_summary(log), {'icp_accepted': 4, 'gnc_inliers': 3, 'gnc_outliers': 1})
            log.write_text('no statistics\n')
            self.assertIsNone(loop_summary(log))

    def test_cancelled_stage_does_not_launch_process(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / 'task.yaml').write_text(config_yaml(MolaConfig()))
            atomic_json(root / 'manifest.json', {})
            worker = Worker(root, APP)
            (root / 'cancel.request').touch()
            with patch('backend.mapping.worker.subprocess.Popen') as popen, self.assertRaises(Cancelled):
                worker.stage('lio', 5, ['never-run'])
            popen.assert_not_called()

    def test_snapshot_keeps_terminal_status_without_runtime_log(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); run_id = 'c' * 32
            (root / run_id).mkdir()
            atomic_json(root / 'state.json', {'id': run_id})
            atomic_json(root / run_id / 'manifest.json', {'status': 'complete', 'progress': 100})
            system = Mock()
            system.show.return_value = {'ActiveState': 'inactive'}
            system.populated.return_value = False
            self.assertEqual(MolaRuntime(root, APP, system=system).snapshot()['status'], 'complete')

    def test_profile_api_and_mapping_localization_exclusion(self):
        import backend.app as app
        from fastapi.testclient import TestClient
        client = TestClient(app.app)
        with patch.object(app, 'runtime_manager', Mock()) as runtime, patch.object(app, 'grid_workspace', Mock()) as grid, patch.object(app, 'localization_runtime', Mock()) as localization, patch.object(app, 'navigation_runtime', Mock()) as navigation:
            runtime.mola.availability.return_value = {'available': True}
            grid.overview.return_value = {'job': {'running': False}}
            localization.pinned_id = None
            navigation.snapshot.return_value = {'busy':False}
            self.assertEqual(client.get('/api/mapping/profiles/mola_lio_lc').status_code, 200)
            self.assertEqual(client.post('/api/mapping/profiles/mola_lio_lc/validate', json={'yaml': 'input: null', 'bag_path': '/bag'}).status_code, 422)
            valid = client.post('/api/mapping/profiles/mola_lio_lc/validate', json={'yaml': 'profile: mola_lio_lc', 'bag_path': '/tmp/bag'})
            self.assertEqual(valid.json()['config']['input']['bag_path'], '/tmp/bag')
            runtime.start.return_value = {'status': 'running'}
            self.assertEqual(client.post('/api/runtime/start', json={'algorithm': 'mola_lio_lc', 'options': {'bag_path': '/tmp/bag'}}).status_code, 202)
            runtime.start.assert_called_once_with('mola_lio_lc', {'bag_path': '/tmp/bag'})
            localization.pinned_id = 'active-floor'
            self.assertEqual(client.post('/api/runtime/start', json={'algorithm': 'mola_lio_lc'}).status_code, 409)
            localization.pinned_id = None
            navigation.snapshot.return_value = {'busy':True}
            self.assertEqual(client.post('/api/runtime/start', json={'algorithm': 'mola_lio_lc'}).status_code, 409)

    def test_incomplete_bag_is_not_modified(self):
        with tempfile.TemporaryDirectory() as tmp:
            config = MolaConfig(input={'bag_path': tmp})
            with self.assertRaises(ValueError):
                validate_bag(config)
            self.assertEqual(list(Path(tmp).iterdir()), [])

    def test_registry_preserves_default_routes_and_guards_switching(self):
        legacy, mola = Mock(), Mock()
        legacy.snapshot.return_value = {'status': 'idle'}
        mola.snapshot.return_value = {'status': 'idle'}
        manager = MappingRuntime(Path('/unused'), Path('/unused/state'), APP, legacy=legacy, mola=mola)
        manager.start('faster_lio_pgo')
        legacy.start.assert_called_once_with('faster_lio_pgo')
        manager.start('mola_lio_lc', {'bag_path': '/bag'})
        mola.start.assert_called_once_with({'bag_path': '/bag'})
        for state in ('running', 'stopping', 'detached', 'conflict'):
            mola.snapshot.return_value = {'status': state}
            with self.assertRaises(RuntimeError):
                manager.start('faster_lio')
        with self.assertRaises(ValueError):
            manager.start('injected')

    def test_runtime_never_stops_unknown_service(self):
        with tempfile.TemporaryDirectory() as tmp:
            system = Mock()
            system.show.return_value = {'ActiveState': 'active', 'Description': 'somebody else'}
            runtime = MolaRuntime(tmp, APP, system=system)
            with self.assertRaises(RuntimeError):
                runtime.stop()
            system.stop.assert_not_called()

    def test_ownership_recovery_and_cancel(self):
        with tempfile.TemporaryDirectory() as tmp:
            root, run_id = Path(tmp), 'a' * 32
            session = root / run_id
            session.mkdir()
            atomic_json(root / 'state.json', {'id': run_id, 'status': 'running'})
            atomic_json(session / 'manifest.json', {'status': 'running'})
            (session / 'runtime.log').touch()
            system = Mock()
            active = {'ActiveState': 'active', 'Description': MARKER + run_id, 'MainPID': '123'}
            system.show.return_value = active
            runtime = MolaRuntime(root, APP, system=system)
            self.assertEqual(runtime.recover()['status'], 'running')
            system.populated.return_value = False
            system.show.side_effect = [active, {'ActiveState': 'inactive'}, {'ActiveState': 'inactive'}]
            runtime.stop()
            self.assertTrue((session / 'cancel.request').is_file())
            system.stop.assert_called_once()

    def test_complete_artifact_allowlist_and_no_false_loop_success(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); session = root / ('b' * 32); session.mkdir()
            (session / 'raw.pcd').write_bytes(b'pcd')
            (session / 'optimized.pcd').symlink_to(session / 'raw.pcd')
            atomic_json(session / 'manifest.json', {'artifacts': [
                {'file': 'raw.pcd', 'role': 'mola_frontend', 'status': 'complete'},
                {'file': 'optimized.pcd', 'role': 'mola_optimized', 'status': 'complete'},
                {'file': '../outside.pcd', 'role': 'mola_frontend', 'status': 'complete'}]})
            result = list(completed_artifacts(root))
            self.assertEqual(len(result), 1)
            self.assertEqual(result[0][1], 'mola_frontend')

    def test_mola_maps_scanned_paired_and_archived_without_changing_selection(self):
        import backend.app as app
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); maps = root / 'maps'; maps.mkdir()
            run = root / 'mola' / ('e' * 32); run.mkdir(parents=True)
            pcd = 'VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA ascii\n0 0 0\n'
            for stem in ('raw', 'optimized'):
                (run / f'{stem}.pcd').write_text(pcd)
            atomic_json(run / 'manifest.json', {'status':'complete', 'started_at':'2026-09-13T16:00:00+08:00',
                'loop_counts':{'icp_accepted':2, 'gnc_inliers':1, 'gnc_outliers':1}, 'loop_validation':'reported',
                'artifacts':[{'file':'raw.pcd','role':'mola_frontend','status':'complete'},
                             {'file':'optimized.pcd','role':'mola_optimized','status':'complete'}]})
            state = {'items':{}, 'active_id':None}
            with patch.object(app,'DATA_ROOT',root), patch.object(app,'MAPS_ROOT',maps), patch.object(app,'scan_processed_maps',return_value=[]), patch.object(app,'read_state',return_value=state):
                items, _ = app.scan_maps()
                self.assertEqual(len(items),2)
                self.assertEqual(app.comparison_for(items[0],items)['id'],items[1]['id'])
                self.assertFalse(any(i['active'] for i in items))
                optimized = next(i for i in items if i['role']=='mola_optimized')
                self.assertEqual(optimized['accepted_loops'],1)
                self.assertIn('地图质量待核对', optimized['issues'][0])
                state['items'][optimized['id']] = {'archived':True}
                archived, _ = app.scan_maps()
                self.assertEqual(sum(i['archived'] for i in archived),1)
                self.assertTrue((run / 'optimized.pcd').exists())

    def test_trusted_pipeline_single_deskew_and_multifloor_parameters(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            from backend.mapping.toolchain import prefix
            if not (prefix(APP) / 'opt/ros/humble/share/mola_lidar_odometry').is_dir():
                self.skipTest('isolated binaries not installed')
            prepare(root, APP, MolaConfig())
            front = yaml.safe_load((root / 'frontend.yaml').read_text())
            loop = yaml.safe_load((root / 'loop.yaml').read_text())
            self.assertTrue(front['params']['simplemap']['save_deskewed_scans'])
            self.assertTrue(front['params']['simplemap']['measure_from_last_kf_only'])
            self.assertEqual(front['params']['simplemap']['min_nearby_poses_occupied'], 1)
            self.assertEqual(loop['observations_generator'][0]['params']['process_sensor_labels_regex'], '^deskewed$')
            exporter = yaml.safe_load((root / 'export.yaml').read_text())
            self.assertEqual(exporter['generators'][0]['params']['process_sensor_labels_regex'], '^deskewed$')
            self.assertEqual(loop['params']['min_icp_goodness'], .75)
            self.assertFalse(loop['params']['assume_planar_world'])
            self.assertFalse(loop['params']['use_gnss'])
            self.assertNotIn('mp2p_icp_filters::FilterDeskew', [f['class_name'] for f in loop['observations_filter']])
