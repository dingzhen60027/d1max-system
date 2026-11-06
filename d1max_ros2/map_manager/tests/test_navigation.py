"""Retired Web Nav2 entry: fake CLI, no ROS, systemd or robot calls."""
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
        self.root = Path(self.temp.name); script = self.root/'scripts/navigation/start_navigation.sh'
        script.parent.mkdir(parents=True); script.touch()
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

    def existing_session(self, *, web_owned=False, live=False):
        # An already-running historical process, not a new alternative task.
        self.active=True; self.mode='live' if live else 'sim'; self.motion=live
        if web_owned:
            self.runtime.owner_path.write_text(json.dumps({'session_id':'c'*32}))

    def test_old_start_cannot_create_another_task_even_when_idle(self):
        for mode in ('sim','live'):
            for enabled in (False,True):
                with self.assertRaises(HTTPException) as error:
                    self.runtime.start(StartNavigation(version_id=self.version['id'],mode=mode,
                                                       enable_motion=enabled),self.version)
                self.assertEqual(error.exception.status_code,410)
        self.assertEqual(self.calls,[])
        self.assertFalse(self.runtime.owner_path.exists())
        self.local.snapshot.assert_not_called()

    def test_old_arm_cannot_reactivate_existing_live_task(self):
        self.existing_session(live=True)
        with self.assertRaises(HTTPException) as error:
            self.runtime.command(self.command('arm'))
        self.assertEqual(error.exception.status_code,410)
        self.assertEqual(self.calls,[])

    def test_old_status_and_stop_still_retire_existing_process(self):
        self.existing_session()
        self.assertEqual(self.runtime.snapshot()['phase'],'running')
        self.runtime.stop()
        self.assertFalse(self.runtime.snapshot()['busy'])

    def command(self,operation='disarm',**kwargs):
        return NavigationCommand(operation=operation,session_id='c'*32,version_id=self.version['id'],**kwargs)

    def test_stale_session_map_rejected_without_command(self):
        self.existing_session(); self.calls.clear()
        for change in ({'session_id':'d'*32},{'version_id':'grid-'+'b'*24}):
            value=self.command().model_copy(update=change)
            with self.assertRaises(HTTPException): self.runtime.command(value)
        self.assertFalse(any(c[0]=='command' and c[c.index('--operation')+1]!='status' for c in self.calls))

    def test_safety_command_forwards_only_session_and_map(self):
        self.existing_session(); self.runtime.command(self.command())
        args=self.calls[-1]
        self.assertEqual(args,['command','--operation','disarm','--session-id','c'*32,'--version-id',self.version['id']])
        with self.assertRaises(HTTPException): self.runtime.command(self.command('arm'))

    def test_close_stops_only_this_web_session(self):
        self.active=True; self.runtime.close(); self.assertTrue(self.active)
        self.existing_session(web_owned=True); self.runtime.close(); self.assertFalse(self.active)

    def test_api_csrf_busy_guard_and_payload_rejection(self):
        grid=Mock();grid.lock=threading.RLock();grid.job={'running':False};grid.item.return_value=self.version
        busy=[False];app=FastAPI();app.include_router(create_navigation_router(self.runtime,grid,threading.RLock(),lambda:busy[0]))
        with TestClient(app) as client:
            self.assertEqual(client.post('/api/navigation/stop',content='{}').status_code,415)
            self.assertEqual(client.post('/api/navigation/stop',json={},headers={'Origin':'https://evil.test'}).status_code,403)
            busy[0]=True
            self.assertEqual(client.post('/api/navigation/start',json={'version_id':self.version['id']}).status_code,410)
            grid.item.assert_not_called()
            self.assertEqual(self.calls,[])
            self.assertEqual(client.post('/api/navigation/command',json={'operation':'shell','session_id':'a','version_id':self.version['id']}).status_code,422)
            self.assertEqual(client.post('/api/navigation/start',json={'version_id':self.version['id'],'speed':2}).status_code,422)
            for operation in ('goal','cancel'):
                self.assertEqual(client.post('/api/navigation/command',json={'operation':operation,'session_id':'c'*32,'version_id':self.version['id']}).status_code,422)
            self.assertEqual(client.post('/api/navigation/command',json={'operation':'arm','session_id':'c'*32,'version_id':self.version['id'],'x':1,'y':2,'yaw':0}).status_code,422)


if __name__=='__main__': unittest.main()
