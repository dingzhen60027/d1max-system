"""Web Nav2 contracts only: fake CLI, no ROS, no systemd, no robot calls."""
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from fastapi import FastAPI, HTTPException
from fastapi.testclient import TestClient

from backend.navigation import NavigationRuntime, NavigationCommand, StartNavigation, create_navigation_router


class NavigationTests(unittest.TestCase):
    def setUp(self):
        self.temp = TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name); (self.root/'start_navigation.sh').touch()
        self.version = {'id':'grid-'+'a'*24,'selected':True,'archived':False,'complete':True}
        self.active = False; self.mode = 'sim'; self.motion = False; self.calls = []
        self.local = Mock()
        self.local.snapshot.return_value = {'phase':'running','version_id':self.version['id'],
            'connection':{'active':True,'health':{'sdk_fresh':True,'lidar_fresh':True,'replay':False}}}
        self.runtime = NavigationRuntime(self.root/'state',self.root,self.local,runner=self.run_cli)

    def run_cli(self,command,**kwargs):
        args=command[2:]; self.calls.append(args)
        if args[0]=='status':
            value={'unit':{'ActiveState':'active' if self.active else 'inactive','Description':'D1MAX_NAVIGATION_V1:test'},
                'last_session':{'navigation_session_id':'c'*32,'version_id':self.version['id'],'mode':self.mode,'enable_motion':self.motion}}
        elif args[0]=='start':
            self.active=True; self.mode=args[args.index('--mode')+1]; self.motion='--enable-motion' in args
            value={'navigation_session_id':'c'*32,'session':'/fake/session.json'}
        elif args[0]=='stop':
            self.active=False; return SimpleNamespace(returncode=0,stdout='Stopped',stderr='')
        elif args[0]=='command': value={'runtime_available':True,'gate':{'armed':False},'accepted':True}
        else: raise AssertionError(command)
        return SimpleNamespace(returncode=0,stdout=json.dumps(value),stderr='')

    def start(self,**kwargs):
        return self.runtime.start(StartNavigation(version_id=self.version['id'],show_rviz=False,**kwargs),self.version)

    def test_start_is_simulation_by_default_and_duplicate_rejected(self):
        self.start(); launch=next(c for c in self.calls if c[0]=='start')
        self.assertEqual(launch,['start','--mode','sim','--web-owned','--headless'])
        with self.assertRaises(HTTPException): self.start()
        self.runtime.stop(); self.assertFalse(self.runtime.snapshot()['busy'])

    def test_simulation_never_enables_motion(self):
        with self.assertRaises(HTTPException): self.start(enable_motion=True)
        self.assertFalse(any(c[0]=='start' for c in self.calls))

    def test_rviz_enabled_by_default(self):
        self.runtime.start(StartNavigation(version_id=self.version['id']),self.version)
        launch=next(c for c in self.calls if c[0]=='start')
        self.assertNotIn('--headless',launch)

    def test_live_requires_existing_same_map_localization_and_real_data(self):
        for change in ({'phase':'stopped'},{'version_id':'grid-'+'b'*24},{'connection':{'active':True,'health':{'sdk_fresh':True,'lidar_fresh':True,'replay':True}}}):
            original=self.local.snapshot.return_value.copy(); self.local.snapshot.return_value.update(change)
            with self.assertRaises(HTTPException): self.start(mode='live',enable_motion=True)
            self.local.snapshot.return_value=original
        self.start(mode='live',enable_motion=True)
        self.assertTrue(self.motion)
        self.assertFalse(any('--operation' in c and c[c.index('--operation')+1]=='arm' for c in self.calls))

    def command(self,operation='disarm',**kwargs):
        return NavigationCommand(operation=operation,session_id='c'*32,version_id=self.version['id'],**kwargs)

    def test_stale_session_map_rejected_without_command(self):
        self.start(); self.calls.clear()
        for change in ({'session_id':'d'*32},{'version_id':'grid-'+'b'*24}):
            value=self.command().model_copy(update=change)
            with self.assertRaises(HTTPException): self.runtime.command(value)
        self.assertFalse(any(c[0]=='command' and c[c.index('--operation')+1]!='status' for c in self.calls))

    def test_safety_command_forwards_only_session_and_map(self):
        self.start(); self.runtime.command(self.command())
        args=self.calls[-1]
        self.assertEqual(args,['command','--operation','disarm','--session-id','c'*32,'--version-id',self.version['id']])
        with self.assertRaises(HTTPException): self.runtime.command(self.command('arm'))

    def test_close_stops_only_this_web_session(self):
        self.active=True; self.runtime.close(); self.assertTrue(self.active)
        self.active=False; self.start(); self.runtime.close(); self.assertFalse(self.active)

    def test_api_csrf_busy_guard_and_payload_rejection(self):
        grid=Mock();grid.lock=threading.RLock();grid.job={'running':False};grid.item.return_value=self.version
        busy=[False];app=FastAPI();app.include_router(create_navigation_router(self.runtime,grid,threading.RLock(),lambda:busy[0]))
        with TestClient(app) as client:
            self.assertEqual(client.post('/api/navigation/stop',content='{}').status_code,415)
            self.assertEqual(client.post('/api/navigation/stop',json={},headers={'Origin':'https://evil.test'}).status_code,403)
            busy[0]=True
            self.assertEqual(client.post('/api/navigation/start',json={'version_id':self.version['id']}).status_code,409)
            self.assertEqual(client.post('/api/navigation/command',json={'operation':'shell','session_id':'a','version_id':self.version['id']}).status_code,422)
            self.assertEqual(client.post('/api/navigation/start',json={'version_id':self.version['id'],'speed':2}).status_code,422)
            for operation in ('goal','cancel'):
                self.assertEqual(client.post('/api/navigation/command',json={'operation':operation,'session_id':'c'*32,'version_id':self.version['id']}).status_code,422)
            self.assertEqual(client.post('/api/navigation/command',json={'operation':'arm','session_id':'c'*32,'version_id':self.version['id'],'x':1,'y':2,'yaw':0}).status_code,422)


if __name__=='__main__': unittest.main()
