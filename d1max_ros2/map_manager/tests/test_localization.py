"""Unit-only lifecycle tests; no systemd writes and no SDK/robot access."""
import json
import fcntl
import os
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import Mock
from fastapi import FastAPI, HTTPException
from fastapi.testclient import TestClient
from backend.localization import LocalizationRuntime, InitialRequest, UNIT, LIVE_VIEW_UNIT, MARKER, create_localization_router

class FakeSystem:
    def __init__(self):self.unit={'ActiveState':'inactive','MainPID':'0'};self.live_unit={'ActiveState':'inactive','MainPID':'0'};self.starts=0;self.stops=0;self.fail=False;self.leak=False
    def show(self,unit):
        if self.fail:raise RuntimeError('unavailable')
        if unit=='d1max-web-managed.service':return {'MainPID':str(os.getpid())}
        return self.live_unit if unit==LIVE_VIEW_UNIT else self.unit
    def populated(self,unit):return unit.get('ActiveState')=='active' or self.leak
    def start(self,session,*args):self.starts+=1;self.unit={'ActiveState':'active','Description':MARKER+session.name,'MainPID':'123'}
    def stop(self):self.stops+=1;self.unit['ActiveState']='inactive'

class LocalizationTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory(prefix='d1max-loc-unit-');self.addCleanup(self.tmp.cleanup);self.root=Path(self.tmp.name)
        self.system=FakeSystem();self.runtime=LocalizationRuntime(self.root/'sessions',self.root,self.root/'nav',system=self.system)
        self.runtime.config.parent.mkdir(parents=True);self.runtime.config.write_text('localization_supervisor:\n  ros__parameters: {}\n')
        self.runtime.script.parent.mkdir(parents=True);self.runtime.script.write_text('test fixture only')
        executable=self.runtime.nav_root/'install/d1max_localization/lib/d1max_localization/fused_icp_matcher';executable.parent.mkdir(parents=True);executable.touch()
        self.runtime.manager=Mock(return_value={'monitor':{'active':True,'health':{'sdk_fresh':True,'lidar_fresh':True,'replay':False}}})
        self.version={'id':'grid-'+'a'*24,'selected':True,'archived':False,'complete':True,'map_yaml_path':str(self.root/'map.yaml'),'name':'test'}
        (self.root/'localization.pcd').write_bytes(b'read only fixture')
    def health(self,**fields):
        value={'session_id':self.runtime.state['id'],'wall_time':time.time(),**fields}
        (self.runtime.root/self.runtime.state['id']/'status.json').write_text(json.dumps(value));return value
    def test_start_once_stop_and_map_file_unchanged(self):
        before=(self.root/'localization.pcd').read_bytes()
        self.runtime.start(self.version);self.assertEqual(self.runtime.pinned_id,self.version['id'])
        with self.assertRaises(HTTPException):self.runtime.start(self.version)
        self.assertEqual(self.system.starts,1)
        self.health(localized=True)
        self.runtime.stop();self.runtime.stop();self.assertEqual(self.system.stops,1);self.assertIsNone(self.runtime.pinned_id)
        self.assertEqual(self.runtime.snapshot(False)['health'],{})
        self.assertEqual(before,(self.root/'localization.pcd').read_bytes())
    def test_lio_backend_requires_installed_executable_frontend_and_coordinator(self):
        self.runtime.config.write_text('localization_pipeline:\n  ros__parameters:\n    backend: lio_pcd\n')
        self.assertFalse(self.runtime.snapshot(False)['installed'])
        for suffix in ('faster_lio/lib/faster_lio/run_mapping_online','d1max_localization/lib/d1max_localization/lio_localizer'):
            path=self.runtime.nav_root/'install'/suffix;path.parent.mkdir(parents=True,exist_ok=True);path.touch()
            self.assertFalse(self.runtime.snapshot(False)['installed']);path.chmod(0o700)
        self.assertTrue(self.runtime.snapshot(False)['installed'])
        self.assertEqual(self.runtime.snapshot(False)['backend'],'lio_pcd')
    def test_unknown_pipeline_cannot_start(self):
        self.runtime.config.write_text('localization_pipeline:\n  ros__parameters:\n    backend: unknown\n')
        with self.assertRaises(HTTPException):self.runtime.start(self.version)
        self.assertEqual(self.system.starts,0)
    def test_live_visualization_unit_blocks_web_localization_start(self):
        for state in ('active','activating','deactivating','reloading'):
            self.system.live_unit={'ActiveState':state,'MainPID':'777'}
            before=self.runtime.state.copy()
            with self.assertRaises(HTTPException) as caught:self.runtime.start(self.version)
            self.assertEqual(caught.exception.status_code,409)
            self.assertEqual(self.system.starts,0)
            self.assertEqual(self.runtime.state,before)
            self.assertFalse(any(path.is_dir() and len(path.name)==32 for path in self.runtime.root.iterdir()))
        self.system.live_unit={'ActiveState':'inactive','MainPID':'0'}
        self.runtime.start(self.version)
        self.assertEqual(self.system.starts,1)
    def test_shared_startup_lease_blocks_concurrent_launch_before_any_side_effect(self):
        lock=self.runtime.launch_lease_path
        lock.parent.mkdir(parents=True,exist_ok=True)
        with lock.open('a') as stream:
            fcntl.flock(stream,fcntl.LOCK_EX|fcntl.LOCK_NB)
            with self.assertRaises(HTTPException) as caught:self.runtime.start(self.version)
            self.assertEqual(caught.exception.status_code,409)
            self.assertEqual(self.system.starts,0)
            self.assertEqual(self.runtime.state['phase'],'stopped')
            fcntl.flock(stream,fcntl.LOCK_UN)
        self.runtime.start(self.version)
        self.assertEqual(self.system.starts,1)
    def test_unknown_unit_is_not_stopped_or_adopted(self):
        self.system.unit={'ActiveState':'active','Description':'someone else'}
        self.assertEqual(self.runtime.snapshot(False)['phase'],'conflict')
        with self.assertRaises(HTTPException):self.runtime.stop()
        with self.assertRaises(HTTPException):self.runtime.start(self.version)
        self.assertEqual((self.system.starts,self.system.stops),(0,0))
    def test_systemd_failure_closes_start_gate(self):
        self.system.fail=True
        self.assertEqual(self.runtime.snapshot(False)['phase'],'conflict')
        with self.assertRaises(HTTPException):self.runtime.start(self.version)
        self.assertEqual(self.system.starts,0)
    def test_residual_children_keep_map_pinned(self):
        self.runtime.start(self.version);self.system.leak=True
        with self.assertRaises(HTTPException):self.runtime.stop()
        self.assertEqual(self.runtime.pinned_id,self.version['id'])
        with self.assertRaises(HTTPException):self.runtime.start(self.version)
    def test_initial_mailbox_ack_expiry_and_wrong_health(self):
        self.runtime.start(self.version);request=InitialRequest(x=1,y=2,z=.4,yaw=0)
        with self.assertRaises(HTTPException):self.runtime.initial_pose(request)
        self.health();first=self.runtime.initial_pose(request);self.assertFalse(first['accepted'])
        command=json.loads((self.runtime.root/self.runtime.state['id']/'initial_pose.json').read_text())
        self.assertEqual(command['reference'],'body')
        self.assertNotIn('heading_frame',command)
        with self.assertRaises(HTTPException):self.runtime.initial_pose(request)
        self.health(command_result={'id':first['request_id'],'accepted':False})
        self.assertEqual(self.runtime.initial_pose(request)['status'],'queued')
        self.health(session_id='another session')
        self.assertFalse(self.runtime.snapshot(False)['health'])
    def test_replay_missing_sensors_or_unselected_version_refused(self):
        for health in ({'sdk_fresh':True,'lidar_fresh':True,'replay':True},{'sdk_fresh':False,'lidar_fresh':True,'replay':False}):
            self.runtime.manager.return_value={'monitor':{'active':True,'health':health}}
            with self.assertRaises(HTTPException):self.runtime.start(self.version)
        self.assertEqual(self.system.starts,0)
    def test_initial_pose_context_headers_accept_matching_session_and_map(self):
        self.runtime.start(self.version);self.health()
        grid=Mock();grid.lock=threading.RLock();grid.job={'running':False}
        app=FastAPI();app.include_router(create_localization_router(self.runtime,grid,threading.RLock(),lambda:False))
        with TestClient(app) as client:
            response=client.post('/api/localization/initial-pose',json={'x':1,'y':2,'z':0,'yaw':0},headers={
                'X-D1max-Session-Id':self.runtime.state['id'],
                'X-D1max-Map-Version':self.version['id']})
            self.assertEqual(response.status_code,202)
            command=json.loads((self.runtime.root/self.runtime.state['id']/'initial_pose.json').read_text())
            self.assertEqual(command['session_id'],self.runtime.state['id'])
            self.assertEqual(command['reference'],'body')
            self.assertEqual(response.json()['request_id'],command['id'])
    def test_initial_pose_rejects_stale_context_without_mailbox_write(self):
        self.runtime.start(self.version);self.health()
        grid=Mock();grid.lock=threading.RLock();grid.job={'running':False}
        app=FastAPI();app.include_router(create_localization_router(self.runtime,grid,threading.RLock(),lambda:False))
        pose={'x':1,'y':2,'z':0,'yaw':0}
        with TestClient(app) as client:
            checked=client.get('/api/localization/overview').json()
            self.runtime.stop();self.runtime.start(self.version);self.health()
            mailbox=self.runtime.root/self.runtime.state['id']/'initial_pose.json'
            stale_headers={'X-D1max-Session-Id':checked['id'],'X-D1max-Map-Version':checked['version_id']}
            self.assertEqual(client.post('/api/localization/initial-pose',json=pose,headers=stale_headers).status_code,409)
            self.assertFalse(mailbox.exists())
            for headers in ({'X-D1max-Map-Version':'grid-'+'b'*24},{'X-D1max-Session-Id':''}):
                self.assertEqual(client.post('/api/localization/initial-pose',json=pose,headers=headers).status_code,409)
                self.assertFalse(mailbox.exists())
            # Old browser clients without the optional context headers remain valid.
            self.assertEqual(client.post('/api/localization/initial-pose',json=pose).status_code,202)
            before=mailbox.read_bytes()
            self.assertEqual(client.post('/api/localization/initial-pose',json=pose,headers=stale_headers).status_code,409)
            self.assertEqual(mailbox.read_bytes(),before)
    def test_api_csrf_guards_busy_guard_and_bad_pose(self):
        grid=Mock();grid.lock=threading.RLock();grid.job={'running':False};grid.item.return_value=self.version
        busy=[False];app=FastAPI();app.include_router(create_localization_router(self.runtime,grid,threading.RLock(),lambda:busy[0]))
        with TestClient(app) as client:
            self.assertEqual(client.post('/api/localization/stop',content='{}').status_code,415)
            self.assertEqual(client.post('/api/localization/stop',json={},headers={'Origin':'https://untrusted.example'}).status_code,403)
            self.assertEqual(client.post('/api/localization/initial-pose',json={'x':1,'y':2,'z':0,'yaw':9}).status_code,422)
            self.assertEqual(client.post('/api/localization/initial-pose',json={'x':1,'y':2,'z':0,'yaw':0,'reference':'guess_ground'}).status_code,422)
            busy[0]=True
            self.assertEqual(client.post('/api/localization/start',json={'version_id':self.version['id']}).status_code,409)
            self.assertEqual(self.system.starts,0)
            self.assertNotIn('token',client.get('/api/localization/overview').text)

if __name__=='__main__':unittest.main()
