"""No ROS/systemd/SDK: every external operation below is a mock subprocess."""
import json
import os
from pathlib import Path
import subprocess
import signal
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import Mock,patch
from fastapi import FastAPI,HTTPException
from fastapi.testclient import TestClient
from backend.single_floor import SingleFloorRuntime,MonitorStatusProbe,UNIT,OWNER,digest,persist,create_single_floor_router,LifecycleCancelled
from backend.mainline_views import OwnedNavigationViews,VIEW_UNIT
from backend.navigation_session import NavigationSessionRuntime,create_navigation_session_router


class System:
    def __init__(self):self.conflict=False;self.residual=False
    def show(self,name):
        if name=='d1max-web-managed.service':return {'ActiveState':'active','MainPID':str(os.getpid())}
        return {'ActiveState':'active' if self.conflict else 'inactive'}
    def populated(self,_):return self.residual


class ExistingMonitor:
    def __init__(self):
        self.ready=True;self.calls=0;self.session='existing-sdk';self.health={};self.start_requests=[]
    def manager(self,path='/v1/status',body=None):
        self.calls+=1
        if path=='/v1/monitor/start':
            self.start_requests.append(body);self.ready=True
        return {'instance':'fixture-manager','monitor':{'active':self.ready,'phase':'running' if self.ready else 'stopped',
            'health':{'sdk_fresh':True,'lidar_fresh':True,'replay':False,
            'sdk_session':self.session,'sdk_session_source_stamp':time.time(),'execution_v3_enabled':False,
            'execution_protocol':'task_follower_v1','execution_task_writer_enabled':True,
            'execution_record_valid':True,**self.health}}}


class MonitorStatusProbeTests(unittest.TestCase):
    def test_slow_connect_must_not_hold_the_lifecycle_state_lock(self):
        with tempfile.TemporaryDirectory() as directory:
            nav=Path(directory)/'nav';nav.mkdir()
            entered=threading.Event();release=threading.Event();done=threading.Event()
            monitor=Mock()
            def status(*args,**kwargs):
                entered.set();release.wait(2.)
                return {'instance':'fixture-instance','monitor':{'active':False,'phase':'stopped','health':{}}}
            monitor.manager.side_effect=status
            system=System()
            runner=lambda *a,**kw:subprocess.CompletedProcess(a,0,'ActiveState=inactive\nMainPID=0')
            probe=Mock();probe.snapshot.return_value={'readiness':'missing_activation'}
            connection=Mock();connection.snapshot.return_value={'active':False,'phase':'stopped','health':{}}
            runtime=SingleFloorRuntime(Path(directory)/'data',nav,monitor,system=system,runner=runner,
                release_probe=probe,monitor_probe=connection,views=Mock())
            def request():
                try:runtime.connect()
                except HTTPException:pass
                finally:done.set()
            caller=threading.Thread(target=request);caller.start()
            try:
                self.assertTrue(entered.wait(1.))
                acquired=runtime.lock.acquire(timeout=.15)
                if acquired:runtime.lock.release()
                self.assertTrue(acquired,'manager HTTP is holding the navigation state lock')
            finally:
                release.set();caller.join(2.)
                if runtime.worker:runtime.worker.join(2.)
            self.assertTrue(done.is_set())

    def test_private_admission_and_digest_fifo_fail_without_stalling_lifecycle(self):
        with tempfile.TemporaryDirectory() as directory:
            fifo=Path(directory)/'replaced-admission';os.mkfifo(fifo,0o600)
            code="""import sys
from backend.single_floor import digest,private_json,regular_text
try:globals()[sys.argv[1]](sys.argv[2])
except ValueError:sys.exit(0)
sys.exit(1)
"""
            for reader in ('digest','private_json','regular_text'):
                result=subprocess.run([sys.executable,'-c',code,reader,str(fifo)],
                    cwd=Path(__file__).resolve().parents[1],capture_output=True,text=True,timeout=3.)
                self.assertEqual(result.returncode,0,result.stderr)

    def test_reader_thread_failure_does_not_permanently_lock_checking(self):
        monitor=ExistingMonitor();probe=MonitorStatusProbe(monitor)
        with patch('backend.single_floor.threading.Thread.start',side_effect=RuntimeError('no_thread')):
            self.assertEqual(probe.snapshot()['phase'],'unavailable')
        self.assertFalse(probe.reading);self.assertEqual(monitor.calls,0)
        probe.checked=0.
        self.assertTrue(SingleFloorRuntime._data_ready(probe.snapshot()))
        self.assertEqual(monitor.calls,1)

    def test_real_ui_poll_interval_gets_new_source_not_previous_poll(self):
        clock={'wall':1800000000.,'steady':100.}
        monitor=ExistingMonitor()
        with patch('backend.single_floor.time.time',side_effect=lambda:clock['wall']), \
             patch('backend.single_floor.time.monotonic',side_effect=lambda:clock['steady']):
            probe=MonitorStatusProbe(monitor)
            for _ in range(8):
                value=probe.snapshot()
                self.assertTrue(SingleFloorRuntime._data_ready(value))
                self.assertEqual(value['health']['sdk_session_source_stamp'],clock['wall'])
                clock['wall']+=1.5;clock['steady']+=1.5
        self.assertEqual(monitor.calls,8)

    def test_slow_manager_is_bounded_and_never_stacks_workers(self):
        release=threading.Event();entered=threading.Event()
        def slow_read():
            entered.set();release.wait(2.)
            return {'monitor':{'active':False,'phase':'stopped','health':{}}}
        monitor=Mock();monitor.manager.side_effect=slow_read
        probe=MonitorStatusProbe(monitor);probe.WAIT_SECONDS=.02
        try:
            for _ in range(4):
                started=time.perf_counter();value=probe.snapshot()
                self.assertLess(time.perf_counter()-started,.15)
                self.assertEqual(value['phase'],'checking')
                self.assertFalse(SingleFloorRuntime._data_ready(value))
            self.assertTrue(entered.is_set())
            self.assertEqual(monitor.manager.call_count,1)
        finally:
            release.set()
            with probe.changed:self.assertTrue(probe.changed.wait_for(lambda:not probe.reading,1.))

    def test_explicit_update_cannot_be_overwritten_by_older_inflight_get(self):
        release=threading.Event()
        monitor=Mock();monitor.manager.side_effect=lambda:(release.wait(2.) or True) and {'monitor':{'active':False,'phase':'stopped','health':{}}}
        probe=MonitorStatusProbe(monitor);probe.WAIT_SECONDS=.02
        try:
            self.assertEqual(probe.snapshot()['phase'],'checking')
            current=ExistingMonitor().manager()['monitor']
            current['health']['sdk_session']='new-connection'
            probe.update(current)
            release.set()
            with probe.changed:self.assertTrue(probe.changed.wait_for(lambda:not probe.reading,1.))
            value=probe.snapshot()
            self.assertEqual(value['health']['sdk_session'],'new-connection')
            self.assertTrue(SingleFloorRuntime._data_ready(value))
        finally:release.set()

    def test_fresh_manager_read_does_not_renew_expired_source_identity(self):
        monitor=ExistingMonitor();original=time.time()-2.
        monitor.health['sdk_session_source_stamp']=original
        value=MonitorStatusProbe(monitor).snapshot()
        self.assertEqual(value['health']['sdk_session_source_stamp'],original)
        self.assertFalse(SingleFloorRuntime._data_ready(value))
        self.assertEqual(SingleFloorRuntime._data_blocker(value),'等待新的 SDK 会话状态')
        for field,value_override in [('replay',True),('sdk_fresh',False),('lidar_fresh',False)]:
            monitor.health={field:value_override}
            self.assertFalse(SingleFloorRuntime._data_ready(MonitorStatusProbe(monitor).snapshot()))
        self.assertFalse(SingleFloorRuntime._data_ready(ExistingMonitor().manager()['monitor'],'foreign'))


class SingleFloorTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.nav=Path(self.tmp.name)/'nav';self.nav.mkdir()
        self.system=System();self.monitor=ExistingMonitor();self.calls=[];self.envs=[];self.drain=True;self.pid=321
        self.prepared_changes={}
        self.startup_phase='started';self.startup_children=True;self.startup_changes={}
        self.unit=dict(ActiveState='inactive',Description='',MainPID='0',InvocationID='',ExecMainStatus='0')
        self.release_probe=Mock();self.release_probe.snapshot.return_value={'readiness':'ready','selected_id':'release','configured_id':'release','reason':''}
        self.monitor_probe=Mock();self.monitor_probe.snapshot.return_value={'active':False,'phase':'stopped','health':{}}
        self.views=Mock();self.views.open.return_value={'layout':'global','pid':432,'invocation_id':'view-invocation'}
        self.runtime=SingleFloorRuntime(Path(self.tmp.name)/'data',self.nav,self.monitor,system=self.system,runner=self.run_mock,
            release_probe=self.release_probe,monitor_probe=self.monitor_probe,views=self.views,helper_runner=self.run_mock)
        self.release=self.nav/'experiments/release';self.release.mkdir(parents=True)
        self.entry=self.nav/'tools/single_floor_entry.sh';self.entry.parent.mkdir();self.entry.write_text('mock only');self.entry.chmod(0o700)
        self.generic_entry=self.nav/'tools/navigation_entry.sh'
        self.preflight=self.nav/'tools/release/single_floor_entry_preflight.py'
        self.preflight.parent.mkdir();self.preflight.write_text('mock only; not executed')
        self.descriptor=self.release/'release.json'
        self.descriptor.write_text(json.dumps({'schema':1,'sealed_manifest':'release_manifest.json'}))
        self.record=self.release/'physical.json';self.record.write_text('not a physical record; subprocess prepare is mocked')
        self.seal=self.release/'release_manifest.json'
        self.seal.write_text(json.dumps({'schema':3,'files':{str(p):digest(p) for p in (self.entry,self.preflight,self.descriptor)}}))
        self.activation=dict(schema_version=3,profile='single_floor_live',purpose='execution',release_root=str(self.release),
            entrypoint=str(self.entry),entrypoint_sha256=digest(self.entry),release_manifest=str(self.seal),
            release_manifest_sha256=digest(self.seal),physical_acceptance_record=str(self.record),
            physical_acceptance_record_sha256=digest(self.record),expected_sdk_session='existing-sdk')
        persist(self.runtime.activation,self.activation)

    def run_mock(self,cmd,**kwargs):
        self.calls.append(cmd)
        if cmd[0] in (str(self.entry),str(self.generic_entry)):self.envs.append((kwargs.get('env') or {}).get('D1MAX_RELEASE'))
        if cmd[:3]==['systemctl','--user','show']:
            return subprocess.CompletedProcess(cmd,0,'\n'.join(k+'='+v for k,v in self.unit.items()))
        if cmd[0] in (str(self.entry),str(self.generic_entry)) and cmd[1]=='prepare':
            directory=Path(cmd[cmd.index('--session')+1]);directory.mkdir()
            purpose=cmd[cmd.index('--purpose')+1]
            session=dict(id=directory.name,pipeline_contract='single_floor_v3',transport_mode='live',
                purpose=purpose,motion_control_enabled=purpose=='execution',view_contract='single_window_layout_v1',
                expected_sdk_session=cmd[cmd.index('--expected-sdk-session')+1],runtime_policy={'worker_startup_timeout_s':60.})
            session.update(self.prepared_changes)
            persist(directory/'session.json',session)
        if cmd[0]=='systemd-run':
            desc=next(x for x in cmd if x.startswith('--property=Description=')).split('=',2)[2]
            self.unit.update(ActiveState='active',Description=desc,MainPID=str(self.pid),InvocationID='invocation')
            state=self.runtime._state()
            if self.startup_phase is not None:
                startup=dict(schema=1,session_id=state['id'],phase=self.startup_phase,
                    children_started=self.startup_children,error='navigation_lock_not_private' if self.startup_phase=='failed' else '',
                    recorded_at_unix=time.time())
                startup.update(self.startup_changes)
                persist(Path(state['directory'])/'core_startup.json',startup)
            if self.startup_phase=='failed':
                # --collect removes the old invocation and its exit properties.
                self.unit.update(ActiveState='inactive',Description='',MainPID='0',InvocationID='')
        if cmd[:3]==['systemctl','--user','stop']:
            self.unit.update(ActiveState='inactive',MainPID='0')
            state=self.runtime._state()
            persist(Path(state['directory'])/'shutdown.json',dict(session_id=state['id'],request_accepted=True,software_retired=self.drain))
        return subprocess.CompletedProcess(cmd,0,'')

    def settle(self,*,raise_error=True):
        with self.runtime.changed:
            self.assertTrue(self.runtime.changed.wait_for(lambda:self.runtime.operation is None,3.))
        state=self.runtime.snapshot(connection=True)
        if raise_error and state.get('worker_error'):
            raise HTTPException(state.get('worker_status_code') or 503,state['worker_error'])
        return state

    def start_settled(self):
        self.runtime.start()
        return self.settle()

    def stop_settled(self):
        self.runtime.stop()
        return self.settle()

    def navigation_activation(self, *, execution=False):
        self.activation.pop('purpose')
        self.activation['launch_policy']='navigation'
        if not execution:
            for key in ('physical_acceptance_record','physical_acceptance_record_sha256','expected_sdk_session'):
                self.activation.pop(key)
        persist(self.runtime.activation,self.activation)

    def runtime_navigation_activation(self):
        self.navigation_activation()
        record=self.release/'runtime.json'
        record.write_text(json.dumps(dict(record_kind='runtime_motion_profile',physical_acceptance_verified=False)))
        seal=json.loads(self.seal.read_text());seal['files'][str(record)]=digest(record)
        self.seal.write_text(json.dumps(seal))
        self.activation.update(runtime_motion_profile=str(record),runtime_motion_profile_sha256=digest(record),
            release_manifest_sha256=digest(self.seal))
        persist(self.runtime.activation,self.activation)
        self.monitor.health['execution_profile_sha256']=digest(record)
        return record

    def test_runtime_profile_starts_full_graph_without_report_or_fixed_sdk_session(self):
        record=self.runtime_navigation_activation()
        self.monitor.session='actual-current-sdk'
        state=self.start_settled()
        self.assertEqual(state['phase'],'running')
        self.assertTrue(state['motion_capable'])
        prepare=next(x for x in self.calls if len(x)>1 and x[1]=='prepare')
        self.assertEqual(prepare[prepare.index('--motion-profile')+1],str(record))
        self.assertEqual(prepare[prepare.index('--expected-sdk-session')+1],'actual-current-sdk')
        self.assertNotIn('--acceptance-record',prepare)
        self.assertEqual(self.monitor.calls,1)
        self.assertFalse(any('connect' in x for c in self.calls for x in c))

    def test_runtime_profile_wrong_sdk_configuration_cannot_start_an_inconsistent_graph(self):
        self.runtime_navigation_activation()
        self.monitor.health['execution_profile_sha256']='b'*64
        with self.assertRaises(HTTPException):self.start_settled()
        self.assertFalse(any(x[0]=='systemd-run' for x in self.calls))

    def test_navigation_policy_starts_same_entry_and_binds_actual_data_session(self):
        self.navigation_activation()
        self.monitor.session='current-data-session'
        self.monitor.health={'execution_v3_enabled':False,'execution_record_valid':False}
        state=self.start_settled()
        self.assertEqual(state['phase'],'running')
        prepare=next(x for x in self.calls if len(x)>1 and x[1]=='prepare')
        self.assertEqual(prepare[prepare.index('--expected-sdk-session')+1],'current-data-session')
        self.assertNotIn('--acceptance-record',prepare)
        self.assertFalse(state['motion_capable'])
        saved=json.loads(self.runtime.activation.read_text())
        self.assertEqual(saved['launch_policy'],'navigation')
        self.assertNotIn('purpose',saved)
        self.assertNotIn('expected_sdk_session',saved)
        self.assertEqual(self.monitor.calls,1)

    def test_navigation_policy_uses_execution_when_record_and_session_explicitly_bound(self):
        self.navigation_activation(execution=True)
        state=self.start_settled()
        prepare=next(x for x in self.calls if len(x)>1 and x[1]=='prepare')
        self.assertEqual(prepare[prepare.index('--purpose')+1],'execution')
        self.assertEqual(prepare[prepare.index('--acceptance-record')+1],str(self.record))
        self.assertTrue(state['motion_capable'])
        self.assertNotIn('purpose',json.loads(self.runtime.activation.read_text()))

    def test_navigation_policy_record_mismatch_cannot_fall_back_to_preview(self):
        self.navigation_activation(execution=True)
        self.record.write_text('changed operator binding')
        self.runtime.start()
        with self.assertRaises(HTTPException):self.settle()
        self.assertFalse(any(x[0]=='systemd-run' for x in self.calls))

    def test_navigation_policy_new_binding_does_not_upgrade_running_capability(self):
        self.navigation_activation()
        state=self.start_settled()
        self.activation.update(physical_acceptance_record=str(self.record),
            physical_acceptance_record_sha256=digest(self.record),expected_sdk_session='existing-sdk')
        persist(self.runtime.activation,self.activation)
        current=self.runtime.snapshot(connection=True)
        self.assertEqual(current['purpose'],state['purpose'])
        self.assertFalse(current['motion_capable'])

    def test_navigation_policy_next_sdk_binding_cannot_rebind_running_execution(self):
        self.navigation_activation(execution=True)
        self.start_settled()
        self.activation['expected_sdk_session']='next-sdk'
        persist(self.runtime.activation,self.activation)
        self.monitor.session='next-sdk'
        self.monitor_probe.snapshot.return_value=self.monitor.manager()['monitor']
        current=self.runtime.snapshot(connection=True)
        self.assertEqual(current['phase'],'running')
        self.assertFalse(current['execution_available'])
        self.assertIn('SDK 会话已变化',current['start_blocker'])

    def test_navigation_policy_bad_next_config_does_not_disable_current_binding(self):
        self.navigation_activation(execution=True)
        self.start_settled()
        self.activation.pop('expected_sdk_session')
        persist(self.runtime.activation,self.activation)
        self.monitor_probe.snapshot.return_value=self.monitor.manager()['monitor']
        current=self.runtime.snapshot(connection=True)
        self.assertEqual(current['phase'],'running')
        self.assertTrue(current['execution_available'])
        self.assertFalse(current['release_configured'])

    def test_start_is_fixed_profile_and_never_connects(self):
        state=self.start_settled();self.assertEqual(state['phase'],'running');self.assertEqual(state['pid'],321)
        self.assertEqual(self.monitor.calls,1)
        start=next(x for x in self.calls if x[0]=='systemd-run')
        self.assertIn('--property=KillMode=mixed',start)
        self.assertEqual(start[-4:],[str(self.entry),'run','--session',self.runtime._state()['directory']])
        self.assertFalse(any('connect' in x or 'start-motion' in x for c in self.calls for x in c))
        self.assertEqual(state['purpose'],'execution')
        prepare=next(x for x in self.calls if x[:2]==[str(self.entry),'prepare'])
        self.assertEqual(prepare[prepare.index('--purpose')+1],'execution')
        self.assertIn('--acceptance-record',prepare)

    def planning_activation(self):
        self.activation['purpose']='planning_only'
        self.activation.pop('physical_acceptance_record',None)
        self.activation.pop('physical_acceptance_record_sha256',None)
        persist(self.runtime.activation,self.activation)

    def test_planning_only_needs_no_motion_record_or_sdk_execution_capability(self):
        self.planning_activation();self.record.unlink()
        self.monitor.health.update(execution_record_valid=False,execution_v3_enabled=False)
        before=self.runtime.snapshot()
        self.assertEqual(before['purpose'],'planning_only')
        self.assertFalse(before['motion_capable']);self.assertFalse(before['execution_available'])
        state=self.start_settled()
        self.assertEqual(state['phase'],'running');self.assertEqual(state['purpose'],'planning_only')
        self.assertFalse(state['motion_capable']);self.assertFalse(state['execution_available'])
        prepare=next(x for x in self.calls if x[:2]==[str(self.entry),'prepare'])
        self.assertEqual(prepare[prepare.index('--purpose')+1],'planning_only')
        self.assertNotIn('--acceptance-record',prepare)
        self.assertFalse(any('connect' in x or 'start-motion' in x for c in self.calls for x in c))
        stopped=self.stop_settled()
        self.assertEqual(stopped['phase'],'stopped');self.assertFalse(stopped['motion_capable'])

    def test_planning_only_still_requires_sealed_entry_and_release(self):
        self.planning_activation();self.entry.write_text('modified')
        with self.assertRaises(HTTPException) as error:self.start_settled()
        self.assertEqual(error.exception.detail,'导航启动配置未通过校验')
        self.assertNotIn('物理验收',error.exception.detail)
        self.assertEqual(self.monitor.calls,0)
        self.assertFalse(any(x[0]=='systemd-run' for x in self.calls))

    def test_planning_only_requires_explicit_sdk_session_binding(self):
        self.planning_activation();self.activation.pop('expected_sdk_session')
        persist(self.runtime.activation,self.activation)
        with self.assertRaises(HTTPException):self.start_settled()
        self.assertEqual(self.monitor.calls,0)

    def test_planning_only_still_requires_current_actual_data_session(self):
        self.planning_activation()
        for field,value in [('sdk_fresh',False),('lidar_fresh',False),('replay',True),
                            ('sdk_session','foreign'),('sdk_session_source_stamp',time.time()-10.),
                            ('sdk_session_source_stamp',None)]:
            with self.subTest(field=field,value=value):
                self.monitor.health={field:value}
                with self.assertRaises(HTTPException):self.start_settled()
        self.assertFalse(any(x[0]=='systemd-run' for x in self.calls))

    def test_planning_only_keeps_conflict_and_duplicate_locks(self):
        self.planning_activation();self.system.conflict=True
        with self.assertRaises(HTTPException):self.start_settled()
        self.system.conflict=False;self.start_settled()
        with self.assertRaises(HTTPException):self.start_settled()
        self.assertEqual(sum(x[0]=='systemd-run' for x in self.calls),1)

    def test_activation_purpose_is_explicit_and_cannot_default_to_execution(self):
        for value in [None,'',False,'preview','isolated_mock']:
            with self.subTest(value=value):
                self.activation['purpose']=value;persist(self.runtime.activation,self.activation)
                self.assertFalse(self.runtime.snapshot()['release_configured'])
                with self.assertRaises(HTTPException):self.start_settled()
        self.activation.pop('purpose');persist(self.runtime.activation,self.activation)
        with self.assertRaises(HTTPException):self.start_settled()
        self.assertFalse(any(x[0]=='systemd-run' for x in self.calls))

    def test_prepared_session_cannot_escalate_or_ignore_planning_purpose(self):
        self.planning_activation()
        for changes in [dict(purpose='execution'),dict(purpose=None),dict(motion_control_enabled=True),
                        dict(motion_control_enabled=0),dict(expected_sdk_session='other')]:
            with self.subTest(changes=changes):
                self.prepared_changes=changes
                with self.assertRaises(HTTPException):self.start_settled()
        self.assertFalse(any(x[0]=='systemd-run' for x in self.calls))

    def test_running_purpose_cannot_be_changed_by_next_activation(self):
        self.planning_activation();self.start_settled()
        self.activation['purpose']='execution';persist(self.runtime.activation,self.activation)
        state=self.runtime.snapshot()
        self.assertEqual(state['purpose'],'planning_only')
        self.assertFalse(state['motion_capable']);self.assertFalse(state['execution_available'])

    def next_execution_activation(self):
        self.activation.update(purpose='execution',physical_acceptance_record=str(self.record),
            physical_acceptance_record_sha256=digest(self.record),expected_sdk_session=self.monitor.session)
        persist(self.runtime.activation,self.activation)

    def retired_preview_with_purpose_pollution(self):
        """Reproduce the old start() bug without starting any real process."""
        self.planning_activation();self.start_settled();self.stop_settled()
        state=self.runtime._state();directory=Path(state['directory'])
        drain=dict(session_id=state['id'],request_accepted=True,software_retired=True,
            physical_stop_confirmed=False,navigator_started=True,recorded_at_unix=time.time(),
            process_cleanup=dict(all_direct_children_exited=True,all_owned_groups_exited=True))
        persist(directory/'shutdown.json',drain)
        # The admitted session stays unchanged. Only the mutable Web state was
        # polluted when selecting the next execution-capable graph.
        state.update(purpose='execution',phase='stopped',quarantined=False)
        persist(self.runtime.state_path,state);self.next_execution_activation()
        self.unit.update(ActiveState='inactive',Description='',MainPID='0',InvocationID='')
        self.monitor_probe.snapshot.return_value=self.monitor.manager()['monitor']
        return state,drain

    def test_clean_retirement_preview_to_execution_preserves_old_identity(self):
        self.planning_activation();self.start_settled();self.stop_settled()
        old=self.runtime._state();session_path=Path(old['directory'])/'session.json'
        old_bytes=session_path.read_bytes();self.next_execution_activation()
        self.assertEqual(self.start_settled()['phase'],'running')
        current=self.runtime._state()
        self.assertNotEqual(current['id'],old['id']);self.assertEqual(current['purpose'],'execution')
        self.assertEqual(session_path.read_bytes(),old_bytes)
        self.assertEqual(old['purpose'],'planning_only')
        self.assertEqual(sum(c[0]=='systemd-run' for c in self.calls),2)
        self.assertFalse(any('connect' in value for command in self.calls for value in command))

    def test_clean_retirement_execution_to_preview_preserves_old_identity(self):
        self.start_settled();self.stop_settled()
        old=self.runtime._state();session_path=Path(old['directory'])/'session.json'
        old_bytes=session_path.read_bytes();self.planning_activation()
        self.assertEqual(self.start_settled()['phase'],'running')
        current=self.runtime._state()
        self.assertNotEqual(current['id'],old['id']);self.assertEqual(current['purpose'],'planning_only')
        self.assertEqual(session_path.read_bytes(),old_bytes)
        self.assertEqual(old['purpose'],'execution')
        self.assertEqual(sum(c[0]=='systemd-run' for c in self.calls),2)

    def test_next_prepare_failure_restores_prior_admitted_identity(self):
        self.planning_activation();self.start_settled();self.stop_settled()
        old=self.runtime._state();self.next_execution_activation()
        def fail_prepare(command,**kwargs):
            if command[:2]==[str(self.entry),'prepare']:
                self.calls.append(command)
                raise subprocess.CalledProcessError(1,command,stderr='mock preparation failure')
            return self.run_mock(command,**kwargs)
        self.runtime.helper_runner=fail_prepare
        with self.assertRaises(HTTPException):self.start_settled()
        current=self.runtime._state()
        for field in ('id','directory','session_sha256','purpose','release_manifest_sha256'):
            self.assertEqual(current[field],old[field])
        self.assertEqual(current['phase'],'stopped');self.assertFalse(current['quarantined'])
        self.assertEqual(sum(c[0]=='systemd-run' for c in self.calls),1)
        self.runtime.helper_runner=self.run_mock
        self.assertEqual(self.start_settled()['phase'],'running')
        self.assertEqual(self.runtime._state()['purpose'],'execution')

    def test_retired_preview_purpose_pollution_recovers_from_immutable_evidence(self):
        polluted,_=self.retired_preview_with_purpose_pollution()
        session_path=Path(polluted['directory'])/'session.json';old_bytes=session_path.read_bytes()
        recovered=self.runtime._inactive_reconciliation(polluted,self.runtime._unit())
        self.assertEqual(recovered['purpose'],'planning_only')
        self.assertTrue(recovered['stop_report']['software_retired'])
        self.assertTrue(recovered['stop_report']['cleanup_confirmed'])
        self.assertFalse(recovered['stop_report']['physical_stop_confirmed'])
        self.assertEqual(self.runtime._state()['purpose'],'execution')  # A read does not rewrite state.
        self.assertEqual(self.start_settled()['phase'],'running')
        self.assertNotEqual(self.runtime._state()['id'],polluted['id'])
        self.assertEqual(self.runtime._state()['purpose'],'execution')
        self.assertEqual(session_path.read_bytes(),old_bytes)

    def test_purpose_pollution_recovery_rejects_changed_session_hash(self):
        polluted,_=self.retired_preview_with_purpose_pollution()
        polluted['session_sha256']='0'*64;persist(self.runtime.state_path,polluted)
        with self.assertRaises(HTTPException):self.start_settled()
        self.assertEqual(sum(c[0]=='systemd-run' for c in self.calls),1)

    def test_purpose_pollution_recovery_rejects_wrong_drain_identity(self):
        polluted,drain=self.retired_preview_with_purpose_pollution()
        drain['session_id']='foreign-session';persist(Path(polluted['directory'])/'shutdown.json',drain)
        with self.assertRaises(HTTPException):self.start_settled()
        self.assertEqual(sum(c[0]=='systemd-run' for c in self.calls),1)

    def test_purpose_pollution_recovery_rejects_active_core(self):
        polluted,_=self.retired_preview_with_purpose_pollution()
        self.unit.update(ActiveState='active',Description=OWNER+polluted['id'],MainPID='321',InvocationID='invocation')
        with self.assertRaises(HTTPException):self.start_settled()
        self.assertEqual(sum(c[0]=='systemd-run' for c in self.calls),1)

    def test_purpose_pollution_recovery_rejects_residual_view_group(self):
        self.retired_preview_with_purpose_pollution();show=self.system.show
        self.system.show=lambda name: {'ActiveState':'inactive','MainPID':'99'} if name==VIEW_UNIT else show(name)
        with self.assertRaises(HTTPException):self.start_settled()
        self.assertEqual(sum(c[0]=='systemd-run' for c in self.calls),1)

    def test_execution_rejects_unaccepted_record_and_missing_task_writer(self):
        for field in ['execution_record_valid','execution_task_writer_enabled']:
            with self.subTest(field=field):
                self.monitor.health={field:False}
                with self.assertRaises(HTTPException):self.start_settled()
        self.assertFalse(any(x[0]=='systemd-run' for x in self.calls))

    def test_old_curve_protocol_cannot_start_or_automatically_reconnect(self):
        self.monitor.health.update(execution_protocol='curve_cas_v3',execution_v3_enabled=True,
                                  execution_task_writer_enabled=False)
        self.monitor_probe.snapshot.return_value=self.monitor.manager()['monitor']
        overview=self.runtime.snapshot(connection=True)
        self.assertFalse(overview['can_start'])
        self.assertIn('旧控制协议',overview['start_blocker'])
        with self.assertRaises(HTTPException):self.start_settled()
        self.assertFalse(self.monitor.start_requests)
        self.assertFalse(any(x[0]=='systemd-run' for x in self.calls))
        self.runtime.connect();state=self.settle(raise_error=False)
        self.assertIn('旧控制协议',state['worker_error'])
        self.assertFalse(self.monitor.start_requests)

    def test_activated_release_is_named_to_every_entry_call(self):
        self.start_settled()
        self.assertEqual(self.envs,[str(self.release.resolve())]*2)
        start=next(x for x in self.calls if x[0]=='systemd-run')
        self.assertIn('--setenv=D1MAX_RELEASE='+str(self.release.resolve()),start)

    def test_duplicate_start_never_launches_second_process(self):
        self.start_settled()
        with self.assertRaises(HTTPException):self.start_settled()
        self.assertEqual(sum(x[0]=='systemd-run' for x in self.calls),1)

    def test_missing_activation_rejected_before_external_launch(self):
        self.runtime.activation.unlink()
        with self.assertRaises(HTTPException):self.start_settled()
        self.assertFalse(any(x[0]=='systemd-run' for x in self.calls))

    def test_changed_sealed_entry_rejected(self):
        self.entry.write_text('changed')
        with self.assertRaises(HTTPException):self.start_settled()
        self.assertEqual(self.monitor.calls,0)

    def use_generic_entry(self):
        self.generic_entry.write_text('mock generic dispatcher only');self.generic_entry.chmod(0o700)
        self.activation.update(entrypoint=str(self.generic_entry),entrypoint_sha256=digest(self.generic_entry),
            compatibility_entrypoint_sha256=digest(self.entry))
        persist(self.runtime.activation,self.activation)

    def test_generic_entry_accepts_unchanged_legacy_seal_and_pins_running_entry(self):
        seal_before=self.seal.read_bytes();descriptor_before=self.descriptor.read_bytes()
        self.use_generic_entry()
        state=self.start_settled()
        self.assertEqual(state['phase'],'running')
        self.assertEqual(self.runtime._state()['entrypoint'],str(self.generic_entry))
        self.assertEqual(self.runtime._state()['compatibility_entrypoint_sha256'],digest(self.entry))
        self.assertEqual(state['mainline']['runtime_entrypoint'],str(self.generic_entry))
        self.assertEqual(state['mainline']['runtime_entrypoint_sha256'],digest(self.generic_entry))
        self.assertEqual(self.seal.read_bytes(),seal_before)
        self.assertEqual(self.descriptor.read_bytes(),descriptor_before)
        prepare=next(x for x in self.calls if x[:2]==[str(self.generic_entry),'prepare'])
        self.assertIn('--session',prepare)
        start=next(x for x in self.calls if x[0]=='systemd-run')
        self.assertEqual(start[-4:],[str(self.generic_entry),'run','--session',self.runtime._state()['directory']])

    def test_generic_entry_still_rejects_changed_activation_hash_and_legacy_implementation(self):
        self.use_generic_entry();self.generic_entry.write_text('changed generic dispatcher')
        with self.assertRaises(HTTPException):self.start_settled()
        self.assertEqual(self.monitor.calls,0)
        self.use_generic_entry();self.entry.write_text('changed sealed implementation')
        with self.assertRaises(HTTPException):self.start_settled()
        self.assertEqual(self.monitor.calls,0)
        self.assertFalse(any(x[0]=='systemd-run' for x in self.calls))

    def test_generic_entry_requires_explicit_compatibility_dependency_hash(self):
        self.use_generic_entry();self.activation.pop('compatibility_entrypoint_sha256')
        persist(self.runtime.activation,self.activation)
        with self.assertRaises(ValueError):self.runtime._activation()
        with self.assertRaises(HTTPException):self.start_settled()
        self.assertEqual(self.monitor.calls,0)
        self.assertFalse(any(x[0]=='systemd-run' for x in self.calls))

    def test_generic_frozen_release_pins_workspace_dependency_and_frozen_seal(self):
        tools=self.release/'snapshot/tools';tools.mkdir(parents=True)
        frozen_entry=tools/'single_floor_entry.sh';frozen_entry.write_bytes(self.entry.read_bytes());frozen_entry.chmod(0o700)
        frozen_checker=tools/'release/single_floor_entry_preflight.py';frozen_checker.parent.mkdir()
        frozen_checker.write_text('mock frozen checker; never executed')
        self.descriptor.write_text(json.dumps({'schema':1,'sealed_manifest':self.seal.name,
            'tools_source_root':'snapshot/tools','template_source_root':'snapshot'}))
        self.seal.write_text(json.dumps({'schema':3,'files':{str(p):digest(p) for p in (frozen_entry,frozen_checker,self.descriptor)},
            'actual_tools_root':str(tools),'startup_closure':{'schema':1,'fixture':True}}))
        self.activation['release_manifest_sha256']=digest(self.seal)
        self.use_generic_entry()
        # Only the release's frozen implementation appears in this seal.
        self.assertNotIn(str(self.entry),json.loads(self.seal.read_text())['files'])
        with patch('backend.mainline_release.subprocess.run',return_value=subprocess.CompletedProcess([],0,'{}','')) as checker:
            self.assertEqual(self.start_settled()['phase'],'running')
            self.assertEqual(self.runtime._state()['compatibility_entrypoint_sha256'],digest(self.entry))
            sealed_before=self.seal.read_bytes();frozen_before=frozen_entry.read_bytes()
            self.entry.write_text('changed workspace dependency, frozen release unchanged')
            with self.assertRaisesRegex(ValueError,'activated_compatibility_entrypoint_changed'):
                self.runtime._activation()
            self.assertEqual(checker.call_count,0)
            self.assertEqual(sum('startup-check' in command for command in self.calls),1)
            self.assertEqual(self.seal.read_bytes(),sealed_before)
            self.assertEqual(frozen_entry.read_bytes(),frozen_before)

    def test_only_exact_root_dispatcher_names_can_be_activated(self):
        for entry in (self.nav/'tools/other_entry.sh',self.release/'snapshot/tools/single_floor_entry.sh'):
            with self.subTest(entry=entry):
                entry.parent.mkdir(parents=True,exist_ok=True);entry.write_text('mock only');entry.chmod(0o700)
                self.activation.update(entrypoint=str(entry),entrypoint_sha256=digest(entry))
                persist(self.runtime.activation,self.activation)
                with self.assertRaises(HTTPException):self.start_settled()
        self.generic_entry.symlink_to(self.entry)
        self.activation.update(entrypoint=str(self.generic_entry),entrypoint_sha256=digest(self.generic_entry))
        persist(self.runtime.activation,self.activation)
        with self.assertRaises(HTTPException):self.start_settled()
        self.assertEqual(self.monitor.calls,0)
        self.assertFalse(any(x[0]=='systemd-run' for x in self.calls))

    def test_missing_physical_record_rejected(self):
        self.record.unlink()
        with self.assertRaises(HTTPException):self.start_settled()

    def test_activation_must_be_private(self):
        self.runtime.activation.chmod(0o644)
        with self.assertRaises(HTTPException):self.start_settled()

    def test_live_data_required_no_auto_connect(self):
        self.monitor.ready=False
        with self.assertRaises(HTTPException):self.start_settled()
        self.assertFalse(any(x[0]=='systemd-run' for x in self.calls))

    def test_other_sdk_session_does_not_start_or_reconnect(self):
        self.monitor.session='other'
        with self.assertRaises(HTTPException):self.start_settled()
        self.assertFalse(any(x[0]=='systemd-run' for x in self.calls))

    def test_conflicting_existing_localization_not_stopped(self):
        self.system.conflict=True
        with self.assertRaises(HTTPException):self.start_settled()
        self.assertFalse(any(x[:3]==['systemctl','--user','stop'] for x in self.calls))

    def test_stop_requires_drain_and_exact_owner(self):
        self.start_settled();self.unit['MainPID']='777'
        with self.assertRaises(HTTPException):self.stop_settled()
        self.assertFalse(any(x[:3]==['systemctl','--user','stop'] for x in self.calls))
        self.unit['MainPID']='321';self.assertEqual(self.stop_settled()['phase'],'stopped')

    def test_missing_stop_proof_quarantines_relaunch(self):
        self.start_settled();self.drain=False
        with self.assertRaises(HTTPException):self.stop_settled()
        self.assertTrue(self.runtime.snapshot()['quarantined'])
        with self.assertRaises(HTTPException):self.start_settled()

    def test_unexpected_exit_cannot_be_called_clean(self):
        self.start_settled();self.unit.update(ActiveState='failed',MainPID='0',ExecMainStatus='9')
        state=self.runtime.snapshot();self.assertEqual(state['phase'],'failed');self.assertEqual(state['exit_code'],'9')
        with self.assertRaises(HTTPException):self.start_settled()

    def test_no_targets_or_approval_api(self):
        app=FastAPI();shared_lock=threading.Lock()
        app.include_router(create_navigation_session_router(self.runtime,shared_lock,lambda:False))
        app.include_router(create_single_floor_router(self.runtime,shared_lock,lambda:False))
        client=TestClient(app)
        for prefix in ('/api/navigation-session','/api/single-floor'):
            with self.subTest(prefix=prefix):
                self.assertEqual(client.post(prefix+'/start',json={'goal':[1,2,3]}).status_code,422)
                self.assertEqual(client.post(prefix+'/start',json={'purpose':'execution'}).status_code,422)
                self.assertEqual(client.post(prefix+'/start',json={},headers={'origin':'https://foreign.invalid'}).status_code,403)
                for suffix in ('/goal','/confirm','/initial-pose','/execution-permit'):
                    self.assertEqual(client.post(prefix+suffix,json={}).status_code,404)
                self.assertEqual(client.get(prefix+'/overview').status_code,200)

    def test_navigation_session_name_is_same_class_and_both_apis_share_state(self):
        self.assertIs(NavigationSessionRuntime,SingleFloorRuntime)
        app=FastAPI();shared_lock=threading.Lock()
        app.include_router(create_navigation_session_router(self.runtime,shared_lock,lambda:False))
        app.include_router(create_single_floor_router(self.runtime,shared_lock,lambda:False))
        client=TestClient(app)
        started=client.post('/api/navigation-session/start',json={})
        self.assertEqual(started.status_code,202)
        self.settle()
        legacy=client.get('/api/single-floor/overview').json()
        self.assertEqual(legacy['operation_id'],started.json()['operation_id'])
        self.assertEqual(legacy['phase'],'running')
        self.assertEqual(client.post('/api/single-floor/start',json={}).status_code,409)
        self.assertEqual(sum(x[0]=='systemd-run' for x in self.calls),1)
        self.assertEqual(client.post('/api/single-floor/stop',json={}).status_code,202)
        self.settle()
        self.assertEqual(client.get('/api/navigation-session/overview').json()['phase'],'stopped')
        self.assertEqual(UNIT,'d1max-single-floor-navigation.service')

    def test_new_and_legacy_apis_share_runtime_and_task_locks(self):
        app=FastAPI();shared_lock=threading.Lock();busy=[False]
        for factory in (create_navigation_session_router,create_single_floor_router):
            app.include_router(factory(self.runtime,shared_lock,lambda:busy[0]))
        client=TestClient(app)
        shared_lock.acquire()
        try:
            for prefix in ('/api/navigation-session','/api/single-floor'):
                for suffix in ('/start','/connect'):
                    self.assertEqual(client.post(prefix+suffix,json={}).status_code,409)
        finally:shared_lock.release()
        self.runtime.lock.acquire()
        try:
            for prefix in ('/api/navigation-session','/api/single-floor'):
                self.assertEqual(client.post(prefix+'/start',json={}).status_code,409)
                self.assertEqual(client.post(prefix+'/stop',json={}).status_code,409)
        finally:self.runtime.lock.release()
        busy[0]=True
        for prefix in ('/api/navigation-session','/api/single-floor'):
            self.assertEqual(client.post(prefix+'/start',json={}).status_code,409)
        self.assertFalse(any(x[0]=='systemd-run' for x in self.calls))

    def test_app_binds_both_route_names_to_one_runtime_and_unchanged_storage(self):
        # Import alone constructs holders in a temporary root; external calls
        # fail immediately if accidentally introduced into app composition.
        script="""
import inspect
import subprocess
def forbidden(*args, **kwargs):
    raise AssertionError('app import must not run external commands')
subprocess.run = forbidden
import backend.app as app
from backend.navigation_session import NavigationSessionRuntime
from backend.single_floor import SingleFloorRuntime
assert NavigationSessionRuntime is SingleFloorRuntime
assert app.navigation_session_runtime is app.single_floor_runtime
assert app.navigation_session_runtime.root == app.DATA_ROOT / 'single_floor'
assert app.navigation_session_runtime.state_path == app.DATA_ROOT / 'single_floor/state.json'
def flattened_routes(router):
    for route in router.routes:
        included = getattr(route, 'original_router', None)
        if included is not None:
            yield from flattened_routes(included)
        else:
            yield route
for prefix in ('/api/navigation-session', '/api/single-floor'):
    routes = [route for route in flattened_routes(app.app) if getattr(route, 'path', '').startswith(prefix + '/')]
    assert len(routes) == 5
    for route in routes:
        scope = inspect.getclosurevars(route.endpoint).nonlocals
        assert scope['runtime'] is app.navigation_session_runtime
        if 'shared_lock' in scope:
            assert scope['shared_lock'] is app.processing_lock
            assert scope['other_busy'] is app.navigation_session_other_busy
"""
        environment={**os.environ,'D1MAX_MAP_MANAGER_DATA':str(Path(self.tmp.name)/'web-data'),
            'D1MAX_NAV_ROOT':str(self.nav),'D1MAX_BAG_DIR':str(self.nav/'bags'),
            'D1MAX_BAG_LIBRARY_ROOTS':str(self.nav/'bags')}
        result=subprocess.run([sys.executable,'-c',script],env=environment,capture_output=True,text=True,check=False)
        self.assertEqual(result.returncode,0,result.stderr)

    def test_scope_reports_sealed_route_coverage_only(self):
        # Without a sealed preflight route the UI gets no invented coverage.
        self.assertIsNone(self.runtime.snapshot()['scope'])
        route=self.release/'map/route.yaml';route.parent.mkdir()
        route.write_text('floor_id: floor1\nstairs_enabled: false\n')
        session=self.release/'preflight/session.json';session.parent.mkdir()
        session.write_text(json.dumps({'crossfloor_route_config':str(route)}))
        self.seal.write_text(json.dumps({'schema':3,'files':{str(self.entry):digest(self.entry)},
            'preflight_session':str(session)}))
        self.assertEqual(self.runtime.snapshot()['scope'],{'floors':['floor1'],'stairs_enabled':False})
        self.assertEqual(self.runtime.snapshot()['release']['scope'],{'floors':['floor1'],'stairs_enabled':False})
        route.write_text('floor_id: floor1\nfloor_ids: [floor1, floor2]\nstairs_enabled: true\n')
        self.assertEqual(self.runtime.snapshot()['scope'],{'floors':['floor1','floor2'],'stairs_enabled':True})
        outside=Path(self.tmp.name)/'outside.yaml';outside.write_text('floor_id: floor9\n')
        session.write_text(json.dumps({'crossfloor_route_config':str(outside)}))
        self.assertIsNone(self.runtime.snapshot()['scope'])

    def test_snapshot_reports_mainline_and_exact_running_release(self):
        self.release_probe.snapshot.return_value['manifest_sha256']=self.activation['release_manifest_sha256']
        self.start_settled()
        state=self.runtime.snapshot()
        self.assertEqual(state['mainline']['entry_module'],'d1max_pct_scan.navigation_session')
        self.assertEqual(state['mainline']['entrypoint'],str(self.generic_entry))
        self.assertEqual(state['mainline']['runtime_entrypoint'],str(self.entry))
        self.assertEqual(state['mainline']['task_owner'],'BehaviorTree.CPP')
        self.assertEqual(state['release']['running_id'],'release')
        self.assertEqual(state['release']['running_manifest_sha256'],self.activation['release_manifest_sha256'])
        self.assertTrue(state['release']['running_matches_configured'])
        self.release_probe.snapshot.return_value.update(configured_id='next-release')
        self.assertEqual(self.runtime.snapshot()['release']['running_id'],'release')
        self.assertFalse(self.runtime.snapshot()['release']['running_matches_configured'])

    def test_same_release_path_new_configuration_does_not_relabel_existing_core(self):
        self.start_settled()
        original=self.runtime._state()['release_manifest_sha256']
        self.release_probe.snapshot.return_value.update(manifest_sha256='b'*64)
        release=self.runtime.snapshot()['release']
        self.assertEqual(release['running_id'],release['configured_id'])
        self.assertEqual(release['running_manifest_sha256'],original)
        self.assertEqual(release['manifest_sha256'],'b'*64)
        self.assertFalse(release['running_matches_configured'])

    def test_collected_core_or_preparing_helper_is_not_reported_as_running_release(self):
        self.start_settled()
        self.unit.update(ActiveState='inactive',MainPID='0')
        release=self.runtime.snapshot()['release']
        self.assertIsNone(release['running_id'])
        self.assertIsNone(release['running_manifest_sha256'])
        self.assertIsNone(release['running_matches_configured'])

    def test_readiness_requires_verified_release_and_actual_data(self):
        data={'active':True,'phase':'running','health':{'sdk_fresh':True,'lidar_fresh':True,'replay':False,
            'sdk_session':'existing-sdk','sdk_session_source_stamp':time.time(),
            'execution_protocol':'task_follower_v1','execution_task_writer_enabled':True,
            'execution_record_valid':True}}
        self.monitor_probe.snapshot.return_value=data
        self.assertTrue(self.runtime.snapshot(connection=True)['can_start'])
        for readiness in ['invalid','checking','missing_activation']:
            self.release_probe.snapshot.return_value['readiness']=readiness
            self.assertFalse(self.runtime.snapshot(connection=True)['can_start'])
        self.release_probe.snapshot.return_value['readiness']='ready'
        data['health']['sdk_session_source_stamp']=True
        self.assertFalse(self.runtime.snapshot(connection=True)['can_start'])
        data['health']['sdk_session_source_stamp']=time.time()-2
        self.assertFalse(self.runtime.snapshot(connection=True)['can_start'])

    def test_preview_binds_new_data_session_only_at_start(self):
        self.planning_activation()
        self.activation['sdk_session_policy']='bind_current_on_start'
        self.activation.pop('expected_sdk_session')
        persist(self.runtime.activation,self.activation)
        self.monitor.session='new-sdk-session'
        self.start_settled()
        prepare=next(c for c in self.calls if c[:2]==[str(self.entry),'prepare'])
        self.assertEqual(prepare[prepare.index('--expected-sdk-session')+1],'new-sdk-session')
        self.assertNotIn('expected_sdk_session',json.loads(self.runtime.activation.read_text()))
        self.assertFalse(any('connect' in v for c in self.calls for v in c))

    def test_execution_cannot_bind_arbitrary_next_session(self):
        self.activation['sdk_session_policy']='bind_current_on_start'
        persist(self.runtime.activation,self.activation)
        with self.assertRaises(HTTPException):self.start_settled()
        self.assertFalse(any(c[0]=='systemd-run' for c in self.calls))

    def test_view_failure_keeps_core_running(self):
        self.views.open.side_effect=ValueError('desktop_unavailable')
        state=self.start_settled()
        self.assertEqual(state['phase'],'running')
        self.assertTrue(state['view_error'])
        self.assertFalse(any(c[:3]==['systemctl','--user','stop'] for c in self.calls))

    def test_type_exec_does_not_open_view_without_core_started_ack(self):
        self.startup_phase=None;self.prepared_changes["runtime_policy"]={"worker_startup_timeout_s":.01}
        with self.assertRaises(HTTPException) as error:self.start_settled()
        self.assertEqual(error.exception.status_code,503)
        self.assertIn('core_startup_timeout',error.exception.detail)
        self.views.open.assert_not_called()
        self.assertTrue(self.runtime._state()['quarantined'])
        self.assertNotEqual(self.runtime._state()['phase'],'running')

    def test_rviz_opens_only_after_matching_started_ack(self):
        def open_after_ack(state,layout):
            ack=json.loads((Path(state['directory'])/'core_startup.json').read_text())
            self.assertEqual(ack['phase'],'started');self.assertTrue(ack['children_started'])
            self.assertEqual(ack['session_id'],state['id'])
            return {'layout':layout,'pid':432,'invocation_id':'view-invocation'}
        self.views.open.side_effect=open_after_ack
        self.assertEqual(self.start_settled()['phase'],'running')
        self.views.open.assert_called_once()

    def test_unstarted_preview_failure_returns_exact_cause_and_retries_without_quarantine(self):
        self.planning_activation();self.startup_phase='failed';self.startup_children=False
        with self.assertRaises(HTTPException) as error:self.start_settled()
        self.assertEqual(error.exception.status_code,409)
        self.assertIn('navigation_lock_not_private',error.exception.detail)
        self.assertNotIn('桌面',error.exception.detail)
        self.assertNotIn('RViz',error.exception.detail)
        state=self.runtime._state();self.assertEqual(state['phase'],'stopped')
        self.assertFalse(state['quarantined']);self.assertFalse(state['stop_report']['physical_stop_confirmed'])
        self.assertFalse(state['stop_report']['software_started']);self.assertFalse(state['stop_report']['software_retired'])
        self.views.open.assert_not_called();self.views.stop.assert_not_called()
        self.assertFalse(any(c[:3]==['systemctl','--user','stop'] for c in self.calls))
        self.startup_phase='started';self.startup_children=True
        self.assertEqual(self.start_settled()['phase'],'running')

    def test_snapshot_and_idempotent_stop_reconcile_collected_unstarted_preview(self):
        self.planning_activation();self.start_settled()
        state=self.runtime._state()
        persist(Path(state['directory'])/'core_startup.json',dict(schema=1,session_id=state['id'],
            phase='failed',children_started=False,error='navigation_lock_not_private',recorded_at_unix=time.time()))
        state.update(phase='needs_review',quarantined=True);persist(self.runtime.state_path,state)
        self.unit.update(ActiveState='inactive',Description='',MainPID='0',InvocationID='')
        self.monitor_probe.snapshot.return_value=self.monitor.manager()['monitor']
        overview=self.runtime.snapshot(connection=True)
        self.assertEqual(overview['phase'],'stopped');self.assertFalse(overview['quarantined'])
        self.assertTrue(overview['can_start']);self.assertEqual(overview['startup_error'],'navigation_lock_not_private')
        # Status does not mutate a running operation; explicit stop persists cleanup.
        self.assertEqual(self.runtime._state()['phase'],'needs_review')
        self.assertEqual(self.stop_settled()['phase'],'stopped')
        self.assertEqual(self.stop_settled()['phase'],'stopped')
        self.assertEqual(self.runtime._state()['phase'],'stopped')
        self.views.stop.assert_not_called()

    def test_unstarted_execution_failure_recovers_without_retirement_or_motion_claim(self):
        self.startup_phase='failed';self.startup_children=False
        with self.assertRaises(HTTPException) as error:self.start_settled()
        self.assertEqual(error.exception.status_code,409)
        self.assertIn('navigation_lock_not_private',error.exception.detail)
        self.assertIn('未启动运行组件',error.exception.detail)
        state=self.runtime._state();self.assertEqual(state['phase'],'stopped')
        self.assertFalse(state['quarantined']);self.assertEqual(state['purpose'],'execution')
        report=state['stop_report']
        self.assertTrue(report['cleanup_confirmed'])
        self.assertFalse(report['software_started']);self.assertFalse(report['software_retired'])
        self.assertFalse(report['physical_stop_confirmed'])
        self.assertEqual(report['reason'],'startup_failed_before_graph')
        session_path=Path(state['directory'])/'session.json';admission=session_path.read_bytes()
        self.monitor_probe.snapshot.return_value=self.monitor.manager()['monitor']
        overview=self.runtime.snapshot(connection=True)
        self.assertTrue(overview['can_start']);self.assertFalse(overview['execution_available'])
        self.views.open.assert_not_called();self.views.stop.assert_not_called()
        self.assertFalse(any(command[:3]==['systemctl','--user','stop'] for command in self.calls))
        self.startup_phase='started';self.startup_children=True
        self.assertEqual(self.start_settled()['phase'],'running')
        self.assertNotEqual(self.runtime._state()['id'],state['id'])
        self.assertEqual(session_path.read_bytes(),admission)
        self.assertEqual(sum(command[0]=='systemd-run' for command in self.calls),2)
        self.assertFalse(self.monitor.start_requests)

    def collected_unstarted_execution_failure(self):
        self.startup_phase='failed';self.startup_children=False
        with self.assertRaises(HTTPException):self.start_settled()
        state=self.runtime._state()
        # An older Web can have cached quarantine for the same unstarted graph.
        state.update(phase='needs_review',quarantined=True)
        persist(self.runtime.state_path,state)
        return state

    def test_collected_unstarted_execution_recovery_is_read_only_and_stop_is_idempotent(self):
        state=self.collected_unstarted_execution_failure()
        admission=(Path(state['directory'])/'session.json').read_bytes()
        self.monitor_probe.snapshot.return_value=self.monitor.manager()['monitor']
        overview=self.runtime.snapshot(connection=True)
        self.assertEqual(overview['phase'],'stopped');self.assertTrue(overview['can_start'])
        self.assertFalse(overview['quarantined']);self.assertFalse(overview['execution_available'])
        self.assertEqual(self.runtime._state()['phase'],'needs_review')
        for _ in range(2):
            stopped=self.stop_settled()
            self.assertEqual(stopped['phase'],'stopped')
            self.assertTrue(stopped['stop_report']['cleanup_confirmed'])
            self.assertFalse(stopped['stop_report']['software_retired'])
            self.assertFalse(stopped['stop_report']['physical_stop_confirmed'])
        self.assertEqual((Path(state['directory'])/'session.json').read_bytes(),admission)
        self.views.stop.assert_not_called()
        self.assertEqual(sum(command[0]=='systemd-run' for command in self.calls),1)
        self.assertFalse(any(command[:3]==['systemctl','--user','stop'] for command in self.calls))

    def test_unstarted_execution_recovery_requires_private_matching_failed_ack(self):
        state=self.collected_unstarted_execution_failure()
        path=Path(state['directory'])/'core_startup.json';startup=json.loads(path.read_text())
        variants=[{'session_id':'other'},{'schema':2},{'schema':True},
            {'recorded_at_unix':time.time()-60},{'recorded_at_unix':time.time()+60},
            {'children_started':True},{'children_started':'false'},
            {'phase':'starting'},{'phase':'spawning','children_started':True},
            {'phase':'started','children_started':True}]
        self.monitor_probe.snapshot.return_value=self.monitor.manager()['monitor']
        for changes in variants:
            with self.subTest(changes=changes):
                persist(path,{**startup,**changes})
                overview=self.runtime.snapshot(connection=True)
                self.assertNotEqual(overview['phase'],'stopped');self.assertTrue(overview['quarantined'])
                self.assertFalse(overview['can_start'])
        persist(path,startup);path.chmod(0o644)
        self.assertFalse(self.runtime.snapshot(connection=True)['can_start'])
        path.unlink()
        self.assertFalse(self.runtime.snapshot(connection=True)['can_start'])
        self.assertEqual(sum(command[0]=='systemd-run' for command in self.calls),1)

    def test_unstarted_execution_recovery_rejects_changed_admission_or_motion_binding(self):
        state=self.collected_unstarted_execution_failure()
        path=Path(state['directory'])/'session.json';session=json.loads(path.read_text())
        self.monitor_probe.snapshot.return_value=self.monitor.manager()['monitor']
        state['session_sha256']='0'*64;persist(self.runtime.state_path,state)
        self.assertFalse(self.runtime.snapshot(connection=True)['can_start'])
        with self.assertRaises(HTTPException):self.start_settled()
        for motion in (False,0,'true'):
            with self.subTest(motion=motion):
                persist(path,{**session,'motion_control_enabled':motion})
                state['session_sha256']=digest(path);persist(self.runtime.state_path,state)
                self.assertFalse(self.runtime.snapshot(connection=True)['can_start'])
                with self.assertRaises(HTTPException):self.start_settled()
        persist(path,session);state['session_sha256']=digest(path);persist(self.runtime.state_path,state)
        persist(path,{**session,'purpose':'planning_only','motion_control_enabled':False})
        self.assertFalse(self.runtime.snapshot(connection=True)['can_start'])
        with self.assertRaises(HTTPException):self.start_settled()
        self.assertEqual(sum(command[0]=='systemd-run' for command in self.calls),1)

    def test_unstarted_execution_recovery_rejects_active_or_populated_core_and_views(self):
        state=self.collected_unstarted_execution_failure()
        self.monitor_probe.snapshot.return_value=self.monitor.manager()['monitor']
        self.system.residual=True
        self.assertFalse(self.runtime.snapshot(connection=True)['can_start'])
        self.system.residual=False
        self.unit.update(ActiveState='active',Description=OWNER+state['id'],MainPID='321',InvocationID='invocation')
        self.assertFalse(self.runtime.snapshot(connection=True)['can_start'])
        self.unit.update(ActiveState='inactive',Description='',MainPID='0',InvocationID='')
        show=self.system.show
        for view in ({'ActiveState':'active','MainPID':'0'},{'ActiveState':'inactive','MainPID':'99'}):
            with self.subTest(view=view):
                self.system.show=lambda name: view if name==VIEW_UNIT else show(name)
                overview=self.runtime.snapshot(connection=True)
                self.assertNotEqual(overview['phase'],'stopped');self.assertFalse(overview['can_start'])
        self.system.show=show
        self.views.stop.assert_not_called()
        self.assertEqual(sum(command[0]=='systemd-run' for command in self.calls),1)

    def test_spawned_failure_without_retirement_does_not_auto_clear(self):
        self.planning_activation();self.startup_phase='failed';self.startup_children=True
        with self.assertRaises(HTTPException) as error:self.start_settled()
        self.assertIn('退出与退役待核对',error.exception.detail)
        self.assertTrue(self.runtime._state()['quarantined'])
        with self.assertRaises(HTTPException):self.stop_settled()
        self.assertNotEqual(self.runtime.snapshot()['phase'],'stopped')
        self.views.open.assert_not_called()

    def partial_spawn_cleanup(self,*,navigator_started=False):
        self.planning_activation();self.start_settled();state=self.runtime._state()
        persist(Path(state['directory'])/'core_startup.json',dict(schema=1,session_id=state['id'],
            phase='failed',children_started=True,error='critical_component_exited',recorded_at_unix=time.time()))
        drain=dict(session_id=state['id'],recorded_at_unix=time.time(),navigator_started=navigator_started,
            request_accepted=False,software_retired=False,physical_stop_confirmed=False,
            process_cleanup=dict(all_direct_children_exited=True,all_owned_groups_exited=True))
        persist(Path(state['directory'])/'shutdown.json',drain)
        state.update(phase='needs_review',quarantined=True);persist(self.runtime.state_path,state)
        self.unit.update(ActiveState='inactive',Description='',MainPID='0',InvocationID='')
        return state,drain

    def test_partial_planning_spawn_recovers_from_real_cleanup_not_drain_claim(self):
        state,drain=self.partial_spawn_cleanup()
        self.monitor_probe.snapshot.return_value=self.monitor.manager()['monitor']
        overview=self.runtime.snapshot(connection=True)
        self.assertEqual(overview['phase'],'stopped');self.assertTrue(overview['can_start'])
        self.assertFalse(overview['quarantined'])
        report=overview['stop_report']
        self.assertTrue(report['cleanup_confirmed']);self.assertEqual(report['process_cleanup'],drain['process_cleanup'])
        self.assertFalse(report['software_started']);self.assertFalse(report['request_accepted'])
        self.assertFalse(report['software_retired']);self.assertFalse(report['physical_stop_confirmed'])
        self.assertEqual(self.runtime._state()['phase'],'needs_review')  # GET remains read-only.
        self.assertEqual(self.stop_settled()['phase'],'stopped')
        self.assertEqual(self.stop_settled()['phase'],'stopped')
        self.assertEqual(self.runtime._state()['stop_report']['process_cleanup'],report['process_cleanup'])
        self.assertFalse(self.runtime._state()['stop_report']['software_retired'])

    def test_partial_cleanup_with_navigator_spawn_keeps_actual_retirement_false(self):
        self.partial_spawn_cleanup(navigator_started=True)
        report=self.runtime.snapshot()['stop_report']
        self.assertTrue(report['software_started']);self.assertFalse(report['software_retired'])
        self.assertFalse(report['physical_stop_confirmed'])

    def test_partial_cleanup_requires_private_fresh_complete_same_session_proof(self):
        state,drain=self.partial_spawn_cleanup()
        path=Path(state['directory'])/'shutdown.json'
        variants=[{**drain,'session_id':'other'}, {**drain,'recorded_at_unix':time.time()-60},
            {**drain,'recorded_at_unix':time.time()+60}, {**drain,'recorded_at_unix':True},
            {key:value for key,value in drain.items() if key!='recorded_at_unix'},
            {**drain,'navigator_started':'false'}, {**drain,'software_retired':'false'},
            {**drain,'process_cleanup':{'all_direct_children_exited':False,'all_owned_groups_exited':True}},
            {**drain,'process_cleanup':{'all_direct_children_exited':True,'all_owned_groups_exited':False}},
            {**drain,'process_cleanup':{'all_direct_children_exited':True,'all_owned_groups_exited':'true'}}]
        for invalid in variants:
            with self.subTest(invalid=invalid):
                persist(path,invalid)
                overview=self.runtime.snapshot()
                self.assertNotEqual(overview['phase'],'stopped');self.assertTrue(overview['quarantined'])
        persist(path,drain);path.chmod(0o644)
        self.assertNotEqual(self.runtime.snapshot()['phase'],'stopped')

    def test_partial_cleanup_cannot_clear_execution_or_residual_view(self):
        state,drain=self.partial_spawn_cleanup()
        show=self.system.show
        self.system.show=lambda name: {'ActiveState':'inactive','MainPID':'9'} if name=='d1max-navigation-view.service' else show(name)
        self.assertNotEqual(self.runtime.snapshot()['phase'],'stopped')
        self.system.show=show
        session_path=Path(state['directory'])/'session.json'
        session=json.loads(session_path.read_text());session.update(purpose='execution',motion_control_enabled=True)
        persist(session_path,session);state.update(purpose='execution',session_sha256=digest(session_path))
        persist(self.runtime.state_path,state)
        self.assertNotEqual(self.runtime.snapshot()['phase'],'stopped')
        self.assertTrue(self.runtime.snapshot()['quarantined'])

    def test_collected_core_can_cleanup_its_exact_view_before_reconciliation(self):
        state,_=self.partial_spawn_cleanup()
        occupied={'value':True};show=self.system.show
        self.system.show=lambda name: {'ActiveState':'active','MainPID':'900'} if name=='d1max-navigation-view.service' and occupied['value'] else show(name)
        self.views.stop.side_effect=lambda _:occupied.update(value=False)
        self.assertNotEqual(self.runtime.snapshot()['phase'],'stopped')
        self.assertEqual(self.stop_settled()['phase'],'stopped')
        self.views.stop.assert_called_once()
        self.assertEqual(self.views.stop.call_args.args[0]['id'],state['id'])

    def test_collected_spawned_failure_with_exact_drain_can_restart_without_physical_claim(self):
        self.start_settled();state=self.runtime._state()
        persist(Path(state['directory'])/'shutdown.json',dict(session_id=state['id'],request_accepted=True,
            software_retired=True,physical_stop_confirmed=False))
        self.unit.update(ActiveState='inactive',Description='',MainPID='0',InvocationID='')
        self.assertEqual(self.runtime.snapshot()['phase'],'stopped')
        stopped=self.stop_settled();self.assertEqual(stopped['phase'],'stopped')
        self.assertTrue(stopped['stop_report']['software_retired'])
        self.assertFalse(stopped['stop_report']['physical_stop_confirmed'])
        self.assertFalse(stopped['quarantined'])

    def test_mismatched_stale_or_inconsistent_ack_never_reports_running(self):
        self.prepared_changes["runtime_policy"]={"worker_startup_timeout_s":.001}
        for changes in ({'session_id':'other'},{'schema':True},{'recorded_at_unix':time.time()-60},
                        {'recorded_at_unix':time.time()+60},{'children_started':False}):
            with self.subTest(changes=changes):
                self.startup_changes=changes
                if self.runtime.state_path.exists():self.runtime.state_path.unlink()
                self.unit.update(ActiveState='inactive',Description='',MainPID='0',InvocationID='')
                with self.assertRaises(HTTPException):self.start_settled()
                self.assertNotEqual(self.runtime._state()['phase'],'running')
        self.views.open.assert_not_called()

    def test_preview_failed_ack_cannot_clear_a_residual_or_foreign_view_group(self):
        self.planning_activation();self.startup_phase='failed';self.startup_children=False
        show=self.system.show
        def occupied(name):
            if name=='d1max-navigation-view.service':return {'ActiveState':'active','MainPID':'900'}
            return show(name)
        self.system.show=occupied
        with self.assertRaises(HTTPException):self.start_settled()
        self.assertFalse(any(command[0]=='systemd-run' for command in self.calls))
        self.assertNotEqual(self.runtime.snapshot()['phase'],'stopped')
        self.views.stop.assert_not_called()

    def test_repeated_start_failure_view_reopen_exit_and_new_sdk_session_keep_one_owner(self):
        self.runtime_navigation_activation()
        self.monitor_probe.snapshot.return_value=self.monitor.manager()['monitor']
        original=self.runtime.helper_runner
        failed={'done':False}
        def prepare_once_fails(command,**kwargs):
            if command[1]=='prepare' and not failed['done']:
                failed['done']=True
                raise subprocess.CalledProcessError(1,command,stderr='offline preparation failure')
            return original(command,**kwargs)
        self.runtime.helper_runner=prepare_once_fails
        with self.assertRaises(HTTPException):self.start_settled()
        self.assertEqual(self.runtime.snapshot()['phase'],'stopped')
        self.assertFalse(self.runtime.snapshot()['quarantined'])
        self.assertFalse(any(command[0]=='systemd-run' for command in self.calls))
        identifiers=set()
        for cycle in range(3):
            self.monitor.session='data-session-'+str(cycle)
            self.monitor_probe.snapshot.return_value=self.monitor.manager()['monitor']
            self.pid=321+cycle
            running=self.start_settled();admitted=self.runtime._state()
            self.assertEqual(running['phase'],'running')
            self.assertNotIn(admitted['id'],identifiers);identifiers.add(admitted['id'])
            session=json.loads((Path(admitted['directory'])/'session.json').read_text())
            self.assertEqual(session['expected_sdk_session'],self.monitor.session)
            with self.assertRaises(HTTPException):self.runtime.start()
            # A failed or closed presentation cannot restart the core or SDK.
            self.views.open.side_effect=ValueError('offline view closed')
            with self.assertRaises(HTTPException):self.runtime.open_view('global')
            self.assertEqual(self.runtime.snapshot()['phase'],'running')
            self.views.open.side_effect=None
            self.assertEqual(self.runtime.open_view('global')['phase'],'running')
            stopped=self.stop_settled()
            self.assertEqual(stopped['phase'],'stopped')
            self.assertTrue(stopped['stop_report']['software_retired'])
            self.assertFalse(stopped['stop_report']['physical_stop_confirmed'])
        self.assertEqual(sum(command[0]=='systemd-run' for command in self.calls),3)
        self.assertEqual(sum(command[:3]==['systemctl','--user','stop'] for command in self.calls),3)
        self.assertFalse(any('connect' in value for command in self.calls for value in command))

    def test_invalid_unstarted_evidence_does_not_clear_preview_quarantine(self):
        self.planning_activation();self.startup_phase='failed';self.startup_children=False
        self.prepared_changes["runtime_policy"]={"worker_startup_timeout_s":.001}
        for changes in ({'session_id':'other'},{'recorded_at_unix':time.time()-60},
                        {'children_started':'false'},{'schema':2}):
            with self.subTest(changes=changes):
                self.startup_changes=changes
                if self.runtime.state_path.exists():self.runtime.state_path.unlink()
                self.unit.update(ActiveState='inactive',Description='',MainPID='0',InvocationID='')
                with self.assertRaises(HTTPException):self.start_settled()
                self.assertTrue(self.runtime._state()['quarantined'])
                self.assertNotEqual(self.runtime.snapshot()['phase'],'stopped')

    def test_no_ack_and_inactive_empty_group_is_not_evidence_of_clean_start_failure(self):
        self.planning_activation();self.start_settled();state=self.runtime._state()
        (Path(state['directory'])/'core_startup.json').unlink()
        self.unit.update(ActiveState='inactive',Description='',MainPID='0',InvocationID='')
        self.assertEqual(self.runtime.snapshot()['phase'],'failed')
        with self.assertRaises(HTTPException):self.stop_settled()
        self.assertNotEqual(self.runtime.snapshot()['phase'],'stopped')
        self.assertNotEqual(self.runtime._state()['phase'],'stopped')

    def test_non_private_startup_file_is_not_accepted_as_recovery_evidence(self):
        self.planning_activation();self.start_settled();state=self.runtime._state()
        path=Path(state['directory'])/'core_startup.json'
        persist(path,dict(schema=1,session_id=state['id'],phase='failed',children_started=False,
            error='navigation_lock_not_private',recorded_at_unix=time.time()))
        path.chmod(0o644)
        self.unit.update(ActiveState='inactive',Description='',MainPID='0',InvocationID='')
        self.assertEqual(self.runtime.snapshot()['phase'],'failed')

    def test_current_session_changed_recovery_is_explicit_not_internal_error(self):
        self.planning_activation();self.start_settled();state=self.runtime._state()
        self.unit.update(ActiveState='inactive',Description='',MainPID='0',InvocationID='')
        persist(Path(state['directory'])/'session.json',{'id':'changed'})
        with self.assertRaises(HTTPException) as start:self.start_settled()
        self.assertEqual(start.exception.status_code,409)
        with self.assertRaises(HTTPException) as stop:self.stop_settled()
        self.assertEqual(stop.exception.status_code,409)

    def test_view_failure_reports_actual_owner_reason_not_desktop_guess(self):
        self.views.open.side_effect=ValueError('navigation_core_not_running')
        state=self.start_settled()
        self.assertIn('navigation_core_not_running',state['view_error'])
        self.assertNotIn('桌面',state['view_error'])

    def test_session_tampering_blocks_cleanup_and_stale_drain_does_not_retire(self):
        self.planning_activation();self.start_settled();state=self.runtime._state()
        self.unit.update(ActiveState='inactive',MainPID='0')
        persist(Path(state['directory'])/'shutdown.json',dict(session_id='other',request_accepted=True,software_retired=True))
        self.assertEqual(self.runtime.snapshot()['phase'],'failed')
        persist(Path(state['directory'])/'session.json',{'id':'tampered'})
        self.assertEqual(self.runtime.snapshot()['phase'],'conflict')

    def test_starting_ack_keeps_phase_starting_and_never_claims_running(self):
        self.start_settled();state=self.runtime._state()
        persist(Path(state['directory'])/'core_startup.json',dict(schema=1,session_id=state['id'],
            phase='starting',children_started=False,error='',recorded_at_unix=time.time()))
        self.assertEqual(self.runtime.snapshot()['phase'],'starting')

    def test_open_view_requires_core_and_exact_session(self):
        with self.assertRaises(HTTPException):self.runtime.open_view('local')
        self.start_settled();self.runtime.open_view('local')
        self.assertEqual(self.views.open.call_args.args[1],'local')
        self.assertEqual(self.runtime._state()['view_contract'],'single_window_layout_v1')
        self.assertEqual(set(self.runtime._state()['views']),{'single'})
        state=self.runtime._state()
        persist(Path(state['directory'])/'session.json',{'id':'tampered'})
        with self.assertRaises(HTTPException):self.runtime.open_view('global')

    def test_physical_stop_is_distinct_from_software_exit(self):
        self.start_settled();state=self.stop_settled()
        self.assertEqual(state['phase'],'stopped')
        self.assertTrue(state['stop_report']['software_retired'])
        self.assertFalse(state['stop_report']['physical_stop_confirmed'])
        self.views.stop.assert_called_once()

    def execution_exit_cleanup_proof(self,state):
        return dict(session_id=state['id'],request_accepted=True,software_retired=False,
            physical_stop_confirmed=False,navigator_started=True,recorded_at_unix=time.time(),
            reason='task_owner_drain_timeout',process_cleanup=dict(
                all_direct_children_exited=True,all_owned_groups_exited=True))

    def collected_execution_with_drain_timeout(self):
        self.start_settled();state=self.runtime._state()
        state.update(phase='needs_review',quarantined=True,stop_requested_at_unix=time.time(),
            worker_operation='stop',worker_error='进程已退出，但停止与退役未确认',worker_status_code=409)
        persist(self.runtime.state_path,state)
        drain=self.execution_exit_cleanup_proof(state)
        persist(Path(state['directory'])/'shutdown.json',drain)
        self.unit.update(ActiveState='inactive',Description='',MainPID='0',InvocationID='')
        self.monitor_probe.snapshot.return_value=self.monitor.manager()['monitor']
        return state,drain

    def assert_execution_cleanup_preserves_failed_retirement(self,overview,drain,*,cleanup_reason='execution_owned_stop_cleanup'):
        self.assertEqual(overview['phase'],'stopped');self.assertFalse(overview['quarantined'])
        self.assertTrue(overview['can_start']);self.assertFalse(overview['busy'])
        report=overview['stop_report']
        self.assertTrue(report['cleanup_confirmed']);self.assertTrue(report['request_accepted'])
        self.assertFalse(report['software_retired']);self.assertFalse(report['physical_stop_confirmed'])
        self.assertEqual(report['reason'],drain['reason'])
        self.assertEqual(report['cleanup_reason'],cleanup_reason)
        self.assertEqual(report['task_retirement_reason'],drain['reason'])
        self.assertEqual(report['process_cleanup'],drain['process_cleanup'])
        self.assertEqual(overview['worker_error'],'');self.assertEqual(overview['worker_operation'],'')
        self.assertIsNone(overview['worker_status_code'])
        self.assertFalse(overview['execution_available'])

    def test_execution_stop_cleans_processes_without_claiming_task_or_physical_retirement(self):
        self.start_settled();original=self.runtime.runner;proof={}
        self.monitor_probe.snapshot.return_value=self.monitor.manager()['monitor']
        def runner(command,**kwargs):
            result=original(command,**kwargs)
            if command[:3]==['systemctl','--user','stop']:
                state=self.runtime._state();proof.update(self.execution_exit_cleanup_proof(state))
                persist(Path(state['directory'])/'shutdown.json',proof)
            return result
        self.runtime.runner=runner
        stopped=self.stop_settled()
        self.assert_execution_cleanup_preserves_failed_retirement(stopped,proof)
        self.assertFalse(self.runtime._state()['quarantined'])
        self.assertEqual(sum(command[:3]==['systemctl','--user','stop'] for command in self.calls),1)
        self.assertFalse(any('connect' in value for command in self.calls for value in command))

    def test_collected_explicit_execution_cleanup_recovers_and_next_start_gets_new_identity(self):
        state,drain=self.collected_execution_with_drain_timeout()
        overview=self.runtime.snapshot(connection=True)
        self.assert_execution_cleanup_preserves_failed_retirement(overview,drain)
        self.assertEqual(self.runtime._state()['phase'],'needs_review')  # GET remains read-only.
        stopped=self.stop_settled();self.assert_execution_cleanup_preserves_failed_retirement(stopped,drain)
        self.assertEqual(self.runtime._state()['worker_error'],'')
        self.assertEqual(self.start_settled()['phase'],'running')
        self.assertNotEqual(self.runtime._state()['id'],state['id'])
        self.assertEqual(sum(command[0]=='systemd-run' for command in self.calls),2)

    def test_execution_cleanup_rejects_wrong_stale_incomplete_or_untyped_evidence(self):
        state,drain=self.collected_execution_with_drain_timeout()
        path=Path(state['directory'])/'shutdown.json'
        variants=[{**drain,'session_id':'other'},
            {**drain,'recorded_at_unix':state['launch_requested_at_unix']-1},
            {**drain,'recorded_at_unix':state['stop_requested_at_unix']-1},
            {**drain,'recorded_at_unix':time.time()+60},
            {**drain,'recorded_at_unix':True},
            {key:value for key,value in drain.items() if key!='recorded_at_unix'},
            {key:value for key,value in drain.items() if key!='process_cleanup'},
            {**drain,'request_accepted':False}, {**drain,'software_retired':'false'},
            {**drain,'physical_stop_confirmed':'false'}, {**drain,'navigator_started':'true'},
            {**drain,'process_cleanup':{'all_direct_children_exited':False,'all_owned_groups_exited':True}},
            {**drain,'process_cleanup':{'all_direct_children_exited':True,'all_owned_groups_exited':False}},
            {**drain,'process_cleanup':{'all_direct_children_exited':True,'all_owned_groups_exited':'true'}}]
        for invalid in variants:
            with self.subTest(invalid=invalid):
                persist(path,invalid)
                overview=self.runtime.snapshot(connection=True)
                self.assertNotEqual(overview['phase'],'stopped');self.assertTrue(overview['quarantined'])
                self.assertFalse(overview['can_start'])
        persist(path,drain);path.chmod(0o644)
        self.assertNotEqual(self.runtime.snapshot()['phase'],'stopped')
        self.assertFalse(self.runtime.snapshot(connection=True)['can_start'])
        self.assertEqual(sum(command[0]=='systemd-run' for command in self.calls),1)

    def test_execution_cleanup_rejects_changed_admitted_session_hash(self):
        state,_=self.collected_execution_with_drain_timeout()
        state['session_sha256']='0'*64;persist(self.runtime.state_path,state)
        overview=self.runtime.snapshot(connection=True)
        self.assertNotEqual(overview['phase'],'stopped');self.assertFalse(overview['can_start'])
        with self.assertRaises(HTTPException):self.start_settled()
        self.assertEqual(sum(command[0]=='systemd-run' for command in self.calls),1)

    def test_execution_cleanup_rejects_residual_or_active_groups(self):
        state,_=self.collected_execution_with_drain_timeout()
        self.system.residual=True
        self.assertNotEqual(self.runtime.snapshot()['phase'],'stopped')
        self.assertFalse(self.runtime.snapshot(connection=True)['can_start'])
        self.system.residual=False;show=self.system.show
        self.system.show=lambda name: {'ActiveState':'inactive','MainPID':'99'} if name==VIEW_UNIT else show(name)
        self.assertNotEqual(self.runtime.snapshot()['phase'],'stopped')
        self.assertFalse(self.runtime.snapshot(connection=True)['can_start'])
        self.system.show=show
        self.unit.update(ActiveState='active',Description=OWNER+state['id'],MainPID='321',InvocationID='invocation')
        self.assertNotEqual(self.runtime.snapshot()['phase'],'stopped')
        self.assertFalse(self.runtime.snapshot(connection=True)['can_start'])
        self.assertEqual(sum(command[0]=='systemd-run' for command in self.calls),1)

    def test_unrequested_execution_exit_recovers_web_without_claiming_task_retirement(self):
        state,drain=self.collected_execution_with_drain_timeout()
        state.pop('stop_requested_at_unix')
        state.update(phase='running',quarantined=False,worker_operation='',worker_error='',worker_status_code=None)
        persist(self.runtime.state_path,state)
        state_bytes=self.runtime.state_path.read_bytes()
        proof_path=Path(state['directory'])/'shutdown.json';proof_bytes=proof_path.read_bytes()
        before=len(self.calls)
        overview=self.runtime.snapshot(connection=True)
        self.assert_execution_cleanup_preserves_failed_retirement(
            overview,drain,cleanup_reason='execution_owned_process_cleanup')
        self.assertFalse(overview['can_view']);self.assertFalse(overview['can_stop'])
        self.assertEqual(self.runtime.state_path.read_bytes(),state_bytes)  # GET is read-only.
        self.assertEqual(proof_path.read_bytes(),proof_bytes)
        self.assertTrue(all(command[:3]==['systemctl','--user','show'] for command in self.calls[before:]))
        # Relaunch is a NEW graph/admission, never resurrection of the old task.
        self.assertEqual(self.start_settled()['phase'],'running')
        self.assertNotEqual(self.runtime._state()['id'],state['id'])
        self.assertEqual(sum(command[0]=='systemd-run' for command in self.calls),2)
        self.assertEqual(proof_path.read_bytes(),proof_bytes)

    def test_unrequested_execution_cleanup_needs_exact_fresh_complete_exit_proof(self):
        state,drain=self.collected_execution_with_drain_timeout()
        state.pop('stop_requested_at_unix')
        state.update(phase='running',quarantined=False,worker_operation='')
        persist(self.runtime.state_path,state)
        path=Path(state['directory'])/'shutdown.json'
        for invalid in ({**drain,'session_id':'foreign'},
                        {**drain,'recorded_at_unix':state['launch_requested_at_unix']-1},
                        {key:value for key,value in drain.items() if key!='process_cleanup'},
                        {**drain,'process_cleanup':{'all_direct_children_exited':False,'all_owned_groups_exited':True}},
                        {**drain,'process_cleanup':{'all_direct_children_exited':True,'all_owned_groups_exited':False}},
                        {**drain,'request_accepted':False}):
            with self.subTest(invalid=invalid):
                persist(path,invalid)
                overview=self.runtime.snapshot(connection=True)
                self.assertFalse(overview['can_start']);self.assertTrue(overview['quarantined'])
                self.assertNotEqual(overview['phase'],'stopped')
        persist(path,drain)
        self.system.residual=True
        self.assertFalse(self.runtime.snapshot(connection=True)['can_start'])
        self.system.residual=False
        show=self.system.show
        self.system.show=lambda name:{'ActiveState':'active','MainPID':'99'} if name==VIEW_UNIT else show(name)
        self.assertFalse(self.runtime.snapshot(connection=True)['can_start'])
        self.system.show=show
        altered={**state,'session_sha256':'0'*64};persist(self.runtime.state_path,altered)
        self.assertFalse(self.runtime.snapshot(connection=True)['can_start'])
        self.assertEqual(sum(command[0]=='systemd-run' for command in self.calls),1)

    def test_unrequested_execution_cleanup_cannot_end_an_inflight_operation(self):
        state,_=self.collected_execution_with_drain_timeout()
        state.pop('stop_requested_at_unix');state.update(phase='running',quarantined=False,worker_operation='')
        persist(self.runtime.state_path,state)
        self.runtime.operation={'kind':'start','cancel':threading.Event()}
        try:
            overview=self.runtime.snapshot(connection=True)
            self.assertTrue(overview['busy']);self.assertFalse(overview['can_start'])
            self.assertNotEqual(overview['phase'],'stopped')
        finally:self.runtime.operation=None

    def test_repeated_execution_stop_preserves_first_request_and_original_cleanup_proof(self):
        state,drain=self.collected_execution_with_drain_timeout()
        first_request=state['stop_requested_at_unix'];future=time.time()+10.
        proof_path=Path(state['directory'])/'shutdown.json';proof_bytes=proof_path.read_bytes()
        with patch('backend.single_floor.time.time',return_value=future):
            self.monitor_probe.snapshot.return_value=self.monitor.manager()['monitor']
            for _ in range(2):
                stopped=self.stop_settled()
                self.assert_execution_cleanup_preserves_failed_retirement(stopped,drain)
                self.assertEqual(self.runtime._state()['stop_requested_at_unix'],first_request)
        self.assertEqual(proof_path.read_bytes(),proof_bytes)
        self.assertFalse(any(command[:3]==['systemctl','--user','stop'] for command in self.calls))

    def test_web_close_after_execution_cleanup_preserves_first_stop_request(self):
        state,drain=self.collected_execution_with_drain_timeout()
        first_request=state['stop_requested_at_unix'];future=time.time()+10.
        with patch('backend.single_floor.time.time',return_value=future):
            self.monitor_probe.snapshot.return_value=self.monitor.manager()['monitor']
            self.stop_settled();self.runtime.close()
            self.assert_execution_cleanup_preserves_failed_retirement(self.settle(),drain)
        self.assertEqual(self.runtime._state()['stop_requested_at_unix'],first_request)
        self.assertTrue(self.runtime.closed)
        self.assertFalse(any(command[:3]==['systemctl','--user','stop'] for command in self.calls))

    def test_execution_cleanup_accepts_later_cached_repeat_request_without_restamping_proof(self):
        state,drain=self.collected_execution_with_drain_timeout()
        # Old Web close() rewrote the cached request after the supervisor had
        # already recorded its original exit. This is not newer stop evidence.
        state['stop_requested_at_unix']=drain['recorded_at_unix']+2.
        persist(self.runtime.state_path,state)
        proof_path=Path(state['directory'])/'shutdown.json';original=proof_path.read_bytes()
        with patch('backend.single_floor.time.time',return_value=drain['recorded_at_unix']+10.):
            self.monitor_probe.snapshot.return_value=self.monitor.manager()['monitor']
            self.assert_execution_cleanup_preserves_failed_retirement(
                self.runtime.snapshot(connection=True),drain)
            self.assert_execution_cleanup_preserves_failed_retirement(self.stop_settled(),drain)
        self.assertEqual(proof_path.read_bytes(),original)
        self.assertFalse(any(command[:3]==['systemctl','--user','stop'] for command in self.calls))

    def test_execution_cleanup_does_not_use_invalid_optional_stop_metadata_as_evidence(self):
        state,drain=self.collected_execution_with_drain_timeout()
        proof_path=Path(state['directory'])/'shutdown.json';original=proof_path.read_bytes()
        for invalid in (time.time()+60.,state['launch_requested_at_unix']-1.,True,'timestamp',float('inf'),float('nan')):
            with self.subTest(stop_requested_at_unix=invalid):
                persist(self.runtime.state_path,{**state,'stop_requested_at_unix':invalid})
                overview=self.runtime.snapshot(connection=True)
                # Matching fresh accepted shutdown + actual empty groups prove
                # cleanup independently; malformed Web stop metadata proves
                # neither a stop request nor task/physical retirement.
                self.assert_execution_cleanup_preserves_failed_retirement(
                    overview,drain,cleanup_reason='execution_owned_process_cleanup')
                self.assertEqual(proof_path.read_bytes(),original)

    def test_uncertain_launch_quarantines_future_starts(self):
        original=self.runtime.runner
        def fail(cmd,**kw):
            if cmd[0]=='systemd-run':raise subprocess.TimeoutExpired(cmd,10)
            return original(cmd,**kw)
        self.runtime.runner=fail
        with self.assertRaises(HTTPException):self.start_settled()
        self.assertTrue(self.runtime.snapshot()['quarantined'])
        with self.assertRaises(HTTPException):self.start_settled()

    def test_connect_is_explicit_and_cannot_reconnect_active_core_or_monitor(self):
        self.runtime.connect()
        with self.assertRaises(HTTPException):self.settle()
        self.assertEqual(self.monitor.start_requests,[])
        self.monitor.ready=False
        self.runtime.connect()
        self.settle()
        self.assertEqual(len(self.monitor.start_requests),1)
        self.assertEqual(self.monitor.start_requests[0]['instance'],'fixture-manager')
        self.monitor_probe.update.assert_called_once()
        self.monitor.ready=True;self.start_settled()
        with self.assertRaises(HTTPException):self.runtime.connect()
        self.assertEqual(len(self.monitor.start_requests),1)

    def test_slow_connection_http_admission_overview_and_cancel_remain_available(self):
        self.monitor.ready=False
        entered=threading.Event();release=threading.Event();original=self.monitor.manager
        def manager(path='/v1/status',body=None):
            if path=='/v1/status':
                entered.set();self.assertTrue(release.wait(3.))
            return original(path,body)
        self.monitor.manager=manager
        app=FastAPI();shared=threading.Lock()
        app.include_router(create_navigation_session_router(self.runtime,shared,lambda:False))
        client=TestClient(app)
        try:
            started=time.perf_counter();accepted=client.post('/api/navigation-session/connect',json={})
            self.assertLess(time.perf_counter()-started,.5)
            self.assertEqual(accepted.status_code,202);self.assertTrue(entered.wait(1.))
            self.assertFalse(shared.locked());self.assertEqual(accepted.json()['phase'],'connecting')
            started=time.perf_counter();overview=client.get('/api/navigation-session/overview')
            self.assertLess(time.perf_counter()-started,.5)
            self.assertTrue(overview.json()['can_stop']);self.assertFalse(overview.json()['can_start'])
            self.assertFalse(overview.json()['can_connect'])
            self.assertEqual(client.post('/api/navigation-session/connect',json={}).status_code,409)
            self.assertEqual(client.post('/api/navigation-session/start',json={}).status_code,409)
            started=time.perf_counter();cancelled=client.post('/api/navigation-session/stop',json={})
            self.assertLess(time.perf_counter()-started,.5)
            self.assertEqual(cancelled.status_code,202);self.assertEqual(cancelled.json()['phase'],'stopping')
            operation=self.runtime.operation
        finally:release.set()
        self.assertEqual(self.settle()['phase'],'stopped')
        self.assertEqual(self.monitor.start_requests,[])  # Cancellation preceded the only write.
        self.monitor_probe.update.assert_not_called()
        with self.assertRaises(LifecycleCancelled):self.runtime._commit(operation,{'phase':'running'})
        self.runtime.connect();self.assertEqual(self.settle()['phase'],'stopped')
        self.assertEqual(len(self.monitor.start_requests),1)
        self.assertNotEqual(self.monitor.start_requests[0]['request_id'],operation['id'])
        self.assertFalse(any(command[0]=='systemd-run' for command in self.calls))

    def test_cancelled_inflight_connect_result_does_not_overwrite_cache_or_reconnect(self):
        self.monitor.ready=False
        entered=threading.Event();release=threading.Event();original=self.monitor.manager
        def manager(path='/v1/status',body=None):
            result=original(path,body)
            if path=='/v1/monitor/start':
                entered.set();self.assertTrue(release.wait(3.))
            return result
        self.monitor.manager=manager
        try:
            self.runtime.connect();self.assertTrue(entered.wait(1.));operation=self.runtime.operation
            self.runtime.stop()
            # A fresher status read is independent of this cancelled response.
            replacement={'active':False,'phase':'stopped','health':{'sdk_session':'newer-observation'}}
            self.monitor_probe.snapshot.return_value=replacement
        finally:release.set()
        self.assertEqual(self.settle()['phase'],'stopped')
        self.monitor_probe.update.assert_not_called()
        self.assertEqual(self.runtime.snapshot(connection=True)['connection'],replacement)
        self.assertEqual(len(self.monitor.start_requests),1)
        with self.assertRaises(LifecycleCancelled):self.runtime._commit(operation,{'phase':'running'})
        self.assertFalse(any(command[:3]==['systemctl','--user','stop'] for command in self.calls))
        self.assertFalse(any(command[0]=='systemd-run' for command in self.calls))

    def test_web_close_can_cancel_slow_connect_without_taking_the_network_state_lock(self):
        self.monitor.ready=False
        entered=threading.Event();release=threading.Event();original=self.monitor.manager
        def manager(path='/v1/status',body=None):
            result=original(path,body)
            if path=='/v1/monitor/start':entered.set();self.assertTrue(release.wait(3.))
            return result
        self.monitor.manager=manager
        closed=threading.Event()
        try:
            self.runtime.connect();self.assertTrue(entered.wait(1.));operation=self.runtime.operation
            closer=threading.Thread(target=lambda:(self.runtime.close(),closed.set()));closer.start()
            with self.runtime.changed:
                self.assertTrue(self.runtime.changed.wait_for(lambda:operation['cancel'].is_set(),.5))
            self.assertTrue(self.runtime.closed)
            self.assertEqual(self.runtime.snapshot()['phase'],'stopping')
        finally:
            release.set()
            if 'closer' in locals():closer.join(2.)
        self.assertTrue(closed.is_set());self.assertEqual(self.settle()['phase'],'stopped')
        self.monitor_probe.update.assert_not_called()
        with self.assertRaises(HTTPException):self.runtime.connect()
        with self.assertRaises(HTTPException):self.runtime.start()
        self.assertEqual(len(self.monitor.start_requests),1)

    def test_connect_failure_remains_retryable_and_success_keeps_retired_navigation_identity(self):
        self.start_settled();retired=self.stop_settled();prior=self.runtime._state()
        self.monitor.ready=False;original=self.monitor.manager
        def failed(path='/v1/status',body=None):raise HTTPException(503,'offline manager unavailable')
        self.monitor.manager=failed
        self.runtime.connect()
        with self.assertRaises(HTTPException):self.settle()
        failed_state=self.runtime.snapshot()
        self.assertEqual(failed_state['phase'],'stopped');self.assertFalse(failed_state['quarantined'])
        self.assertEqual(failed_state['worker_operation'],'connect')
        self.assertEqual(self.runtime._state()['id'],prior['id'])
        self.monitor.manager=original;self.runtime.connect();connected=self.settle()
        self.assertEqual(connected['phase'],'stopped');self.assertEqual(connected['worker_error'],'')
        self.assertEqual(self.runtime._state()['id'],prior['id'])
        self.assertEqual(connected['stop_report'],retired['stop_report'])
        self.assertFalse(connected['stop_report']['physical_stop_confirmed'])
        self.assertEqual(sum(command[0]=='systemd-run' for command in self.calls),1)

    def test_retired_preview_api_does_not_start_historical_stack(self):
        from backend.live_planning import create_live_planning_router
        legacy=Mock()
        app=FastAPI();app.include_router(create_live_planning_router(legacy,threading.Lock(),threading.Lock(),lambda:False,retired=True))
        response=TestClient(app).post('/api/live-planning/start',json={})
        self.assertEqual(response.status_code,410);legacy.start.assert_not_called()

    def test_http_accepts_prepare_and_cancel_without_long_transaction_lock(self):
        entered=threading.Event();release=threading.Event();original=self.runtime.helper_runner
        def helper(command,**kwargs):
            if command[1]=='prepare':entered.set();self.assertTrue(release.wait(3.))
            return original(command,**kwargs)
        self.runtime.helper_runner=helper
        app=FastAPI();shared=threading.Lock()
        app.include_router(create_navigation_session_router(self.runtime,shared,lambda:False))
        client=TestClient(app)
        try:
            before=time.perf_counter();response=client.post('/api/navigation-session/start',json={})
            self.assertEqual(response.status_code,202);self.assertLess(time.perf_counter()-before,.5)
            self.assertTrue(entered.wait(1.));operation=self.runtime.operation
            self.assertFalse(shared.locked());self.assertTrue(response.json()['busy'])
            overview=client.get('/api/navigation-session/overview').json()
            self.assertEqual(overview['phase'],'preparing');self.assertTrue(overview['can_stop'])
            self.assertTrue(all(key in overview for key in ('can_start','can_stop','can_view','can_connect','snapshot_at_unix')))
            self.assertEqual(client.post('/api/navigation-session/stop',json={}).status_code,202)
            self.assertEqual(self.runtime.snapshot()['phase'],'stopping')
            with self.assertRaises(HTTPException):self.runtime.start()
        finally:release.set()
        self.assertEqual(self.settle()['phase'],'stopped')
        self.assertFalse(any(command[0]=='systemd-run' for command in self.calls))
        with self.assertRaises(LifecycleCancelled):self.runtime._commit(operation,{'phase':'running'})

    def test_cancel_during_seal_does_not_launch_or_reuse_old_generation(self):
        entered=threading.Event();release=threading.Event();original=self.runtime.helper_runner
        def helper(command,**kwargs):
            if command[1]=='seal':entered.set();self.assertTrue(release.wait(3.))
            return original(command,**kwargs)
        self.runtime.helper_runner=helper
        try:
            self.runtime.start();self.assertTrue(entered.wait(1.));old=self.runtime.operation
            self.runtime.stop();cancel_generation=self.runtime.generation
        finally:release.set()
        self.assertEqual(self.settle()['phase'],'stopped')
        self.runtime.helper_runner=original
        self.assertEqual(self.start_settled()['phase'],'running')
        self.assertGreater(self.runtime.generation,cancel_generation)
        with self.assertRaises(LifecycleCancelled):self.runtime._commit(old,{'phase':'running'})
        self.assertEqual(sum(command[0]=='systemd-run' for command in self.calls),1)

    def test_cancel_after_launch_before_pid_commit_recovers_exact_core_identity(self):
        entered=threading.Event();release=threading.Event();original=self.runtime.runner
        def runner(command,**kwargs):
            result=original(command,**kwargs)
            if command[0]=='systemd-run':entered.set();self.assertTrue(release.wait(3.))
            return result
        self.runtime.runner=runner
        try:
            self.runtime.start();self.assertTrue(entered.wait(1.))
            self.assertEqual(self.runtime._state()['pid'],0)
            self.runtime.stop()
        finally:release.set()
        self.assertEqual(self.settle()['phase'],'stopped')
        self.views.open.assert_not_called()
        self.assertEqual(sum(command[:3]==['systemctl','--user','stop'] for command in self.calls),1)

    def test_cancel_after_uncertain_launch_never_stops_foreign_replacement(self):
        entered=threading.Event();release=threading.Event();original=self.runtime.runner
        def runner(command,**kwargs):
            result=original(command,**kwargs)
            if command[0]=='systemd-run':entered.set();self.assertTrue(release.wait(3.))
            return result
        self.runtime.runner=runner
        try:
            self.runtime.start();self.assertTrue(entered.wait(1.));self.runtime.stop()
            self.unit.update(Description=OWNER+'other',MainPID='999',InvocationID='replacement')
        finally:release.set()
        state=self.settle(raise_error=False)
        self.assertTrue(state['worker_error']);self.assertFalse(state['owned'])
        self.assertFalse(any(command[:3]==['systemctl','--user','stop'] for command in self.calls))
        self.assertTrue(self.runtime._state()['quarantined'])

    def test_completed_uncertain_launch_is_stoppable_with_admitted_exact_identity(self):
        original=self.runtime.runner
        def runner(command,**kwargs):
            result=original(command,**kwargs)
            if command[0]=='systemd-run':raise subprocess.TimeoutExpired(command,10)
            return result
        self.runtime.runner=runner
        with self.assertRaises(HTTPException):self.start_settled()
        self.assertEqual(self.runtime._state()['pid'],0);self.assertIsNone(self.runtime.operation)
        self.assertTrue(self.runtime.snapshot()['can_stop']);self.assertFalse(self.runtime.snapshot()['can_view'])
        self.assertEqual(self.runtime.snapshot()['phase'],'needs_review')
        self.assertEqual(self.stop_settled()['phase'],'stopped')
        self.assertEqual(sum(command[:3]==['systemctl','--user','stop'] for command in self.calls),1)

    def test_terminal_core_without_ack_fails_immediately_but_stays_unknown(self):
        self.startup_phase=None;original=self.runtime.runner
        def runner(command,**kwargs):
            result=original(command,**kwargs)
            if command[0]=='systemd-run':self.unit.update(ActiveState='inactive',Description='',MainPID='0',InvocationID='')
            return result
        self.runtime.runner=runner;before=time.perf_counter()
        with self.assertRaises(HTTPException) as error:self.start_settled()
        self.assertLess(time.perf_counter()-before,.5)
        self.assertIn('core_startup_exited_without_ack',error.exception.detail)
        self.assertTrue(self.runtime._state()['quarantined']);self.views.open.assert_not_called()

    def test_owned_preview_stop_cleanup_preserves_failed_drain_facts_after_started_ack(self):
        self.planning_activation();self.start_settled();original=self.runtime.runner
        def runner(command,**kwargs):
            result=original(command,**kwargs)
            if command[:3]==['systemctl','--user','stop']:
                state=self.runtime._state()
                persist(Path(state['directory'])/'shutdown.json',dict(session_id=state['id'],recorded_at_unix=time.time(),
                    navigator_started=True,request_accepted=False,software_retired=False,physical_stop_confirmed=False,
                    process_cleanup=dict(all_direct_children_exited=True,all_owned_groups_exited=True)))
            return result
        self.runtime.runner=runner
        stopped=self.stop_settled();self.assertEqual(stopped['phase'],'stopped')
        self.assertTrue(stopped['stop_report']['software_started']);self.assertTrue(stopped['stop_report']['cleanup_confirmed'])
        self.assertFalse(stopped['stop_report']['request_accepted']);self.assertFalse(stopped['stop_report']['software_retired'])
        self.assertFalse(stopped['stop_report']['physical_stop_confirmed'])

    def test_unrequested_preview_exit_cannot_use_cleanup_as_retirement(self):
        self.planning_activation();self.start_settled();state=self.runtime._state()
        persist(Path(state['directory'])/'shutdown.json',dict(session_id=state['id'],recorded_at_unix=time.time(),
            navigator_started=True,request_accepted=False,software_retired=False,physical_stop_confirmed=False,
            process_cleanup=dict(all_direct_children_exited=True,all_owned_groups_exited=True)))
        self.unit.update(ActiveState='inactive',Description='',MainPID='0',InvocationID='')
        self.assertNotEqual(self.runtime.snapshot()['phase'],'stopped')
        self.assertTrue(self.runtime.snapshot()['quarantined'])

    def test_worker_thread_start_failure_rolls_back_operation_reservation(self):
        with patch('backend.single_floor.threading.Thread.start',side_effect=RuntimeError('cannot_start_thread')):
            with self.assertRaises(HTTPException) as error:self.runtime.start()
        self.assertEqual(error.exception.status_code,503);self.assertIsNone(self.runtime.operation)
        self.assertFalse(self.runtime.snapshot()['busy'])
        self.assertEqual(self.runtime.snapshot()['worker_error'],'lifecycle_worker_unavailable')
        self.assertFalse(any(command[0]=='systemd-run' for command in self.calls))

    def test_persist_failure_does_not_leave_permanent_worker_reservation(self):
        with patch('backend.single_floor.persist',side_effect=OSError('offline_disk_error')):
            with self.assertRaises(HTTPException):self.runtime.start()
        self.assertIsNone(self.runtime.operation);self.assertFalse(self.runtime.snapshot()['busy'])

    def test_real_owned_local_helper_is_cancelled_and_reaped_on_close(self):
        entered=threading.Event();processes=[]
        def popen(command,**kwargs):
            process=subprocess.Popen([sys.executable,'-c','import time; time.sleep(30)'],**kwargs)
            processes.append(process);entered.set();return process
        def cleanup(children,**kwargs):
            self.assertEqual(kwargs,dict(interrupt_s=6.,terminate_s=1.,kill_s=1.))
            for _,process in children:
                try:os.killpg(process.pid,signal.SIGTERM)
                except ProcessLookupError:pass
                process.wait(2.)
            return dict(all_direct_children_exited=all(process.poll() is not None for _,process in children),
                        all_owned_groups_exited=True)
        self.runtime.helper_runner=None;self.runtime.popen=popen;self.runtime.process_cleanup=cleanup
        try:
            self.runtime.start();self.assertTrue(entered.wait(1.));before=time.perf_counter()
            self.runtime.close();self.assertLess(time.perf_counter()-before,1.)
            self.assertEqual(self.runtime.snapshot()['phase'],'stopped');self.assertIsNone(self.runtime.helper)
            self.assertTrue(all(process.poll() is not None for process in processes))
            for process in processes:
                with self.assertRaises(ProcessLookupError):os.killpg(process.pid,0)
            with self.assertRaises(HTTPException):self.runtime.start()
            self.assertFalse(any(command[0]=='systemd-run' for command in self.calls))
        finally:
            for process in processes:
                if process.poll() is None:os.killpg(process.pid,signal.SIGKILL);process.wait(2.)

    def test_unconfirmed_helper_cleanup_never_claims_stopped_and_owned_retry_can_finish(self):
        processes=[];cleanup_calls=[]
        def popen(command,**kwargs):
            process=subprocess.Popen([sys.executable,'-c','pass'],**kwargs);processes.append(process);return process
        def cleanup(children,**kwargs):
            cleanup_calls.append(True)
            if len(cleanup_calls)==1:return dict(all_direct_children_exited=False,all_owned_groups_exited=False)
            for _,process in children:
                self.assertIsNotNone(process.poll())
                with self.assertRaises(ProcessLookupError):os.killpg(process.pid,0)
            return dict(all_direct_children_exited=True,all_owned_groups_exited=True)
        self.runtime.helper_runner=None;self.runtime.popen=popen;self.runtime.process_cleanup=cleanup
        self.runtime.start();failed=self.settle(raise_error=False)
        self.assertTrue(failed['busy']);self.assertTrue(failed['quarantined']);self.assertEqual(failed['phase'],'needs_review')
        self.assertTrue(failed['can_stop'])
        self.assertIsNotNone(self.runtime.helper)
        stopped=self.stop_settled();self.assertEqual(stopped['phase'],'stopped');self.assertIsNone(self.runtime.helper)
        self.assertTrue(stopped['stop_report']['cleanup_confirmed']);self.assertFalse(stopped['stop_report']['software_retired'])
        self.assertFalse(stopped['stop_report']['physical_stop_confirmed'])
        self.assertFalse(any(command[0]=='systemd-run' for command in self.calls))

    def test_view_identity_arriving_during_cancel_is_retained_for_cleanup(self):
        entered=threading.Event();release=threading.Event();view_busy={'value':False};records=[]
        class View(OwnedNavigationViews):
            def open(inner,state,layout,*,on_record=None):
                record=dict(unit=VIEW_UNIT,core_id=state['id'],pid=432,invocation_id='view-invocation')
                view_busy['value']=True;on_record(record);entered.set()
                if not release.wait(3.):raise ValueError('offline_view_wait_timeout')
                return record
            def preflight_stop(inner,state):return state.get('views',{})
            def stop(inner,state):
                records.append(state['views']['single']);view_busy['value']=False;return {}
        show=self.system.show
        self.system.show=lambda name: {'ActiveState':'active','MainPID':'432'} if name==VIEW_UNIT and view_busy['value'] else show(name)
        self.runtime.views=View(self.nav,self.system,self.run_mock)
        try:
            self.runtime.start();self.assertTrue(entered.wait(1.));self.runtime.stop()
            self.assertEqual(self.runtime._state()['phase'],'stopping')
            self.assertEqual(self.runtime._state()['views']['single']['invocation_id'],'view-invocation')
        finally:release.set()
        self.assertEqual(self.settle()['phase'],'stopped');self.assertEqual(len(records),1)
        command=next(command for command in self.calls if command[:3]==['systemctl','--user','stop'])
        self.assertEqual(command[3:],[UNIT,VIEW_UNIT])


if __name__=='__main__':unittest.main()
