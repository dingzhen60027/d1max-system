"""No ROS/systemd/SDK: every external operation below is a mock subprocess."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import Mock,patch
from fastapi import FastAPI,HTTPException
from fastapi.testclient import TestClient
from backend.single_floor import SingleFloorRuntime,MonitorStatusProbe,UNIT,OWNER,digest,persist,create_single_floor_router
from backend.navigation_session import NavigationSessionRuntime,create_navigation_session_router


class System:
    def __init__(self):self.conflict=False;self.residual=False
    def show(self,name):
        if name=='d1max-web-managed.service':return {'ActiveState':'active','MainPID':str(os.getpid())}
        return {'ActiveState':'active' if self.conflict else 'inactive'}
    def populated(self,_):return self.residual


class ExistingMonitor:
    def __init__(self):
        self.ready=True;self.calls=0;self.session='existing-sdk';self.health={}
    def manager(self):
        self.calls+=1
        return {'monitor':{'active':self.ready,'health':{'sdk_fresh':True,'lidar_fresh':True,'replay':False,
            'sdk_session':self.session,'sdk_session_source_stamp':time.time(),'execution_v3_enabled':True,
            'execution_record_valid':True,**self.health}}}


class SingleFloorTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.nav=Path(self.tmp.name)/'nav';self.nav.mkdir()
        self.system=System();self.monitor=ExistingMonitor();self.calls=[];self.envs=[];self.drain=True;self.pid=321
        self.prepared_changes={}
        self.unit=dict(ActiveState='inactive',Description='',MainPID='0',InvocationID='',ExecMainStatus='0')
        self.release_probe=Mock();self.release_probe.snapshot.return_value={'readiness':'ready','selected_id':'release','configured_id':'release','reason':''}
        self.monitor_probe=Mock();self.monitor_probe.snapshot.return_value={'active':False,'phase':'stopped','health':{}}
        self.views=Mock();self.views.open.return_value={'layout':'global','pid':432,'invocation_id':'view-invocation'}
        self.runtime=SingleFloorRuntime(Path(self.tmp.name)/'data',self.nav,self.monitor,system=self.system,runner=self.run_mock,
            release_probe=self.release_probe,monitor_probe=self.monitor_probe,views=self.views)
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
                expected_sdk_session=cmd[cmd.index('--expected-sdk-session')+1])
            session.update(self.prepared_changes)
            persist(directory/'session.json',session)
        if cmd[0]=='systemd-run':
            desc=next(x for x in cmd if x.startswith('--property=Description=')).split('=',2)[2]
            self.unit.update(ActiveState='active',Description=desc,MainPID=str(self.pid),InvocationID='invocation')
        if cmd[:3]==['systemctl','--user','stop']:
            self.unit.update(ActiveState='inactive',MainPID='0')
            state=self.runtime._state()
            (Path(state['directory'])/'shutdown.json').write_text(json.dumps(dict(session_id=state['id'],request_accepted=True,software_retired=self.drain)))
        return subprocess.CompletedProcess(cmd,0,'')

    def test_start_is_fixed_profile_and_never_connects(self):
        state=self.runtime.start();self.assertEqual(state['phase'],'running');self.assertEqual(state['pid'],321)
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
        state=self.runtime.start()
        self.assertEqual(state['phase'],'running');self.assertEqual(state['purpose'],'planning_only')
        self.assertFalse(state['motion_capable']);self.assertFalse(state['execution_available'])
        prepare=next(x for x in self.calls if x[:2]==[str(self.entry),'prepare'])
        self.assertEqual(prepare[prepare.index('--purpose')+1],'planning_only')
        self.assertNotIn('--acceptance-record',prepare)
        self.assertFalse(any('connect' in x or 'start-motion' in x for c in self.calls for x in c))
        stopped=self.runtime.stop()
        self.assertEqual(stopped['phase'],'stopped');self.assertFalse(stopped['motion_capable'])

    def test_planning_only_still_requires_sealed_entry_and_release(self):
        self.planning_activation();self.entry.write_text('modified')
        with self.assertRaises(HTTPException) as error:self.runtime.start()
        self.assertIn('只规划发布版本',error.exception.detail)
        self.assertNotIn('物理验收',error.exception.detail)
        self.assertEqual(self.monitor.calls,0)
        self.assertFalse(any(x[0]=='systemd-run' for x in self.calls))

    def test_planning_only_requires_explicit_sdk_session_binding(self):
        self.planning_activation();self.activation.pop('expected_sdk_session')
        persist(self.runtime.activation,self.activation)
        with self.assertRaises(HTTPException):self.runtime.start()
        self.assertEqual(self.monitor.calls,0)

    def test_planning_only_still_requires_current_actual_data_session(self):
        self.planning_activation()
        for field,value in [('sdk_fresh',False),('lidar_fresh',False),('replay',True),
                            ('sdk_session','foreign'),('sdk_session_source_stamp',time.time()-10.),
                            ('sdk_session_source_stamp',None)]:
            with self.subTest(field=field,value=value):
                self.monitor.health={field:value}
                with self.assertRaises(HTTPException):self.runtime.start()
        self.assertFalse(any(x[0]=='systemd-run' for x in self.calls))

    def test_planning_only_keeps_conflict_and_duplicate_locks(self):
        self.planning_activation();self.system.conflict=True
        with self.assertRaises(HTTPException):self.runtime.start()
        self.system.conflict=False;self.runtime.start()
        with self.assertRaises(HTTPException):self.runtime.start()
        self.assertEqual(sum(x[0]=='systemd-run' for x in self.calls),1)

    def test_activation_purpose_is_explicit_and_cannot_default_to_execution(self):
        for value in [None,'',False,'preview','isolated_mock']:
            with self.subTest(value=value):
                self.activation['purpose']=value;persist(self.runtime.activation,self.activation)
                self.assertFalse(self.runtime.snapshot()['release_configured'])
                with self.assertRaises(HTTPException):self.runtime.start()
        self.activation.pop('purpose');persist(self.runtime.activation,self.activation)
        with self.assertRaises(HTTPException):self.runtime.start()
        self.assertFalse(any(x[0]=='systemd-run' for x in self.calls))

    def test_prepared_session_cannot_escalate_or_ignore_planning_purpose(self):
        self.planning_activation()
        for changes in [dict(purpose='execution'),dict(purpose=None),dict(motion_control_enabled=True),
                        dict(motion_control_enabled=0),dict(expected_sdk_session='other')]:
            with self.subTest(changes=changes):
                self.prepared_changes=changes
                with self.assertRaises(HTTPException):self.runtime.start()
        self.assertFalse(any(x[0]=='systemd-run' for x in self.calls))

    def test_running_purpose_cannot_be_changed_by_next_activation(self):
        self.planning_activation();self.runtime.start()
        self.activation['purpose']='execution';persist(self.runtime.activation,self.activation)
        state=self.runtime.snapshot()
        self.assertEqual(state['purpose'],'planning_only')
        self.assertFalse(state['motion_capable']);self.assertFalse(state['execution_available'])

    def test_execution_still_rejects_unaccepted_sdk_record_and_missing_v3(self):
        for field in ['execution_record_valid','execution_v3_enabled']:
            with self.subTest(field=field):
                self.monitor.health={field:False}
                with self.assertRaises(HTTPException):self.runtime.start()
        self.assertFalse(any(x[0]=='systemd-run' for x in self.calls))

    def test_activated_release_is_named_to_every_entry_call(self):
        self.runtime.start()
        self.assertEqual(self.envs,[str(self.release.resolve())]*2)
        start=next(x for x in self.calls if x[0]=='systemd-run')
        self.assertIn('--setenv=D1MAX_RELEASE='+str(self.release.resolve()),start)

    def test_duplicate_start_never_launches_second_process(self):
        self.runtime.start()
        with self.assertRaises(HTTPException):self.runtime.start()
        self.assertEqual(sum(x[0]=='systemd-run' for x in self.calls),1)

    def test_missing_activation_rejected_before_external_launch(self):
        self.runtime.activation.unlink()
        with self.assertRaises(HTTPException):self.runtime.start()
        self.assertFalse(any(x[0]=='systemd-run' for x in self.calls))

    def test_changed_sealed_entry_rejected(self):
        self.entry.write_text('changed')
        with self.assertRaises(HTTPException):self.runtime.start()
        self.assertEqual(self.monitor.calls,0)

    def use_generic_entry(self):
        self.generic_entry.write_text('mock generic dispatcher only');self.generic_entry.chmod(0o700)
        self.activation.update(entrypoint=str(self.generic_entry),entrypoint_sha256=digest(self.generic_entry),
            compatibility_entrypoint_sha256=digest(self.entry))
        persist(self.runtime.activation,self.activation)

    def test_generic_entry_accepts_unchanged_legacy_seal_and_pins_running_entry(self):
        seal_before=self.seal.read_bytes();descriptor_before=self.descriptor.read_bytes()
        self.use_generic_entry()
        state=self.runtime.start()
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
        with self.assertRaises(HTTPException):self.runtime.start()
        self.assertEqual(self.monitor.calls,0)
        self.use_generic_entry();self.entry.write_text('changed sealed implementation')
        with self.assertRaises(HTTPException):self.runtime.start()
        self.assertEqual(self.monitor.calls,0)
        self.assertFalse(any(x[0]=='systemd-run' for x in self.calls))

    def test_generic_entry_requires_explicit_compatibility_dependency_hash(self):
        self.use_generic_entry();self.activation.pop('compatibility_entrypoint_sha256')
        persist(self.runtime.activation,self.activation)
        with self.assertRaises(ValueError):self.runtime._activation()
        with self.assertRaises(HTTPException):self.runtime.start()
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
            self.assertEqual(self.runtime.start()['phase'],'running')
            self.assertEqual(self.runtime._state()['compatibility_entrypoint_sha256'],digest(self.entry))
            sealed_before=self.seal.read_bytes();frozen_before=frozen_entry.read_bytes()
            self.entry.write_text('changed workspace dependency, frozen release unchanged')
            with self.assertRaisesRegex(ValueError,'activated_compatibility_entrypoint_changed'):
                self.runtime._activation()
            self.assertEqual(checker.call_count,1)
            self.assertEqual(self.seal.read_bytes(),sealed_before)
            self.assertEqual(frozen_entry.read_bytes(),frozen_before)

    def test_only_exact_root_dispatcher_names_can_be_activated(self):
        for entry in (self.nav/'tools/other_entry.sh',self.release/'snapshot/tools/single_floor_entry.sh'):
            with self.subTest(entry=entry):
                entry.parent.mkdir(parents=True,exist_ok=True);entry.write_text('mock only');entry.chmod(0o700)
                self.activation.update(entrypoint=str(entry),entrypoint_sha256=digest(entry))
                persist(self.runtime.activation,self.activation)
                with self.assertRaises(HTTPException):self.runtime.start()
        self.generic_entry.symlink_to(self.entry)
        self.activation.update(entrypoint=str(self.generic_entry),entrypoint_sha256=digest(self.generic_entry))
        persist(self.runtime.activation,self.activation)
        with self.assertRaises(HTTPException):self.runtime.start()
        self.assertEqual(self.monitor.calls,0)
        self.assertFalse(any(x[0]=='systemd-run' for x in self.calls))

    def test_missing_physical_record_rejected(self):
        self.record.unlink()
        with self.assertRaises(HTTPException):self.runtime.start()

    def test_activation_must_be_private(self):
        self.runtime.activation.chmod(0o644)
        with self.assertRaises(HTTPException):self.runtime.start()

    def test_live_data_required_no_auto_connect(self):
        self.monitor.ready=False
        with self.assertRaises(HTTPException):self.runtime.start()
        self.assertFalse(any(x[0]=='systemd-run' for x in self.calls))

    def test_other_sdk_session_does_not_start_or_reconnect(self):
        self.monitor.session='other'
        with self.assertRaises(HTTPException):self.runtime.start()
        self.assertFalse(any(x[0]=='systemd-run' for x in self.calls))

    def test_conflicting_existing_localization_not_stopped(self):
        self.system.conflict=True
        with self.assertRaises(HTTPException):self.runtime.start()
        self.assertFalse(any(x[:3]==['systemctl','--user','stop'] for x in self.calls))

    def test_stop_requires_drain_and_exact_owner(self):
        self.runtime.start();self.unit['MainPID']='777'
        with self.assertRaises(HTTPException):self.runtime.stop()
        self.assertFalse(any(x[:3]==['systemctl','--user','stop'] for x in self.calls))
        self.unit['MainPID']='321';self.assertEqual(self.runtime.stop()['phase'],'stopped')

    def test_missing_stop_proof_quarantines_relaunch(self):
        self.runtime.start();self.drain=False
        with self.assertRaises(HTTPException):self.runtime.stop()
        self.assertTrue(self.runtime.snapshot()['quarantined'])
        with self.assertRaises(HTTPException):self.runtime.start()

    def test_unexpected_exit_cannot_be_called_clean(self):
        self.runtime.start();self.unit.update(ActiveState='failed',MainPID='0',ExecMainStatus='9')
        state=self.runtime.snapshot();self.assertEqual(state['phase'],'failed');self.assertEqual(state['exit_code'],'9')
        with self.assertRaises(HTTPException):self.runtime.start()

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
        legacy=client.get('/api/single-floor/overview').json()
        self.assertEqual(legacy['session_id'],started.json()['session_id'])
        self.assertEqual(legacy['phase'],'running')
        self.assertEqual(client.post('/api/single-floor/start',json={}).status_code,409)
        self.assertEqual(sum(x[0]=='systemd-run' for x in self.calls),1)
        self.assertEqual(client.post('/api/single-floor/stop',json={}).status_code,200)
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
        self.runtime.start()
        state=self.runtime.snapshot()
        self.assertEqual(state['mainline']['entry_module'],'d1max_pct_scan.navigation_session')
        self.assertEqual(state['mainline']['entrypoint'],str(self.generic_entry))
        self.assertEqual(state['mainline']['runtime_entrypoint'],str(self.entry))
        self.assertEqual(state['mainline']['task_owner'],'BehaviorTree.CPP')
        self.assertEqual(state['release']['running_id'],'release')
        self.assertEqual(state['release']['running_manifest_sha256'],self.activation['release_manifest_sha256'])
        self.release_probe.snapshot.return_value.update(configured_id='next-release')
        self.assertEqual(self.runtime.snapshot()['release']['running_id'],'release')

    def test_readiness_requires_verified_release_and_actual_data(self):
        data={'active':True,'phase':'running','health':{'sdk_fresh':True,'lidar_fresh':True,'replay':False,
            'sdk_session':'existing-sdk','sdk_session_source_stamp':time.time()}}
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
        self.runtime.start()
        prepare=next(c for c in self.calls if c[:2]==[str(self.entry),'prepare'])
        self.assertEqual(prepare[prepare.index('--expected-sdk-session')+1],'new-sdk-session')
        self.assertNotIn('expected_sdk_session',json.loads(self.runtime.activation.read_text()))
        self.assertFalse(any('connect' in v for c in self.calls for v in c))

    def test_execution_cannot_bind_arbitrary_next_session(self):
        self.activation['sdk_session_policy']='bind_current_on_start'
        persist(self.runtime.activation,self.activation)
        with self.assertRaises(HTTPException):self.runtime.start()
        self.assertFalse(any(c[0]=='systemd-run' for c in self.calls))

    def test_view_failure_keeps_core_running(self):
        self.views.open.side_effect=ValueError('desktop_unavailable')
        state=self.runtime.start()
        self.assertEqual(state['phase'],'running')
        self.assertTrue(state['view_error'])
        self.assertFalse(any(c[:3]==['systemctl','--user','stop'] for c in self.calls))

    def test_open_view_requires_core_and_exact_session(self):
        with self.assertRaises(HTTPException):self.runtime.open_view('local')
        self.runtime.start();self.runtime.open_view('local')
        self.assertEqual(self.views.open.call_args.args[1],'local')
        self.assertEqual(self.runtime._state()['view_contract'],'single_window_layout_v1')
        self.assertEqual(set(self.runtime._state()['views']),{'single'})
        state=self.runtime._state()
        persist(Path(state['directory'])/'session.json',{'id':'tampered'})
        with self.assertRaises(HTTPException):self.runtime.open_view('global')

    def test_physical_stop_is_distinct_from_software_exit(self):
        self.runtime.start();state=self.runtime.stop()
        self.assertEqual(state['phase'],'stopped')
        self.assertTrue(state['stop_report']['software_retired'])
        self.assertFalse(state['stop_report']['physical_stop_confirmed'])
        self.views.stop.assert_called_once()

    def test_uncertain_launch_quarantines_future_starts(self):
        original=self.runtime.runner
        def fail(cmd,**kw):
            if cmd[0]=='systemd-run':raise subprocess.TimeoutExpired(cmd,10)
            return original(cmd,**kw)
        self.runtime.runner=fail
        with self.assertRaises(HTTPException):self.runtime.start()
        self.assertTrue(self.runtime.snapshot()['quarantined'])
        with self.assertRaises(HTTPException):self.runtime.start()

    def test_connect_is_explicit_and_cannot_reconnect_active_core_or_monitor(self):
        self.monitor.connect=Mock(return_value={'active':True,'phase':'running','health':{}})
        with self.assertRaises(HTTPException):self.runtime.connect()
        self.monitor.connect.assert_not_called()
        self.monitor.ready=False
        self.runtime.connect()
        self.monitor.connect.assert_called_once()
        self.monitor_probe.update.assert_called_once()
        self.monitor.ready=True;self.runtime.start()
        with self.assertRaises(HTTPException):self.runtime.connect()
        self.monitor.connect.assert_called_once()

    def test_retired_preview_api_does_not_start_historical_stack(self):
        from backend.live_planning import create_live_planning_router
        legacy=Mock()
        app=FastAPI();app.include_router(create_live_planning_router(legacy,threading.Lock(),threading.Lock(),lambda:False,retired=True))
        response=TestClient(app).post('/api/live-planning/start',json={})
        self.assertEqual(response.status_code,410);legacy.start.assert_not_called()


if __name__=='__main__':unittest.main()
