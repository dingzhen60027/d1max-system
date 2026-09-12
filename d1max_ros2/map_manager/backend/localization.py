"""Web-owned localization lifecycle. A dedicated systemd cgroup owns all ROS nodes.

Only fixed start/stop/initial-pose/monitor-connect operations. Never robot motion.
"""
from __future__ import annotations
import fcntl
import json
import math
import os
from pathlib import Path
import re
import subprocess
import threading
import time
import uuid
from typing import Literal
from contextlib import contextmanager
from urllib.request import Request, build_opener, ProxyHandler
from urllib.error import HTTPError
import yaml
from fastapi import APIRouter, HTTPException, Depends, Request as ApiRequest
from fastapi.responses import FileResponse
from pydantic import BaseModel, ConfigDict, Field
from .grid_maps import atomic_json

UNIT='d1max-localization-managed.service'
MARKER='D1MAX_LOCALIZATION_V1:'

class StartRequest(BaseModel):
    model_config=ConfigDict(extra='forbid')
    version_id:str=Field(pattern=r'^grid-[a-f0-9]{24}$')

class InitialRequest(BaseModel):
    model_config=ConfigDict(extra='forbid',allow_inf_nan=False)
    x:float=Field(ge=-100000,le=100000)
    y:float=Field(ge=-100000,le=100000)
    z:float=Field(ge=-100,le=100)
    yaw:float=Field(ge=-math.pi,le=math.pi)
    reference:Literal['body','tracking']='body'

class Systemd:
    def show(self,unit):
        result=subprocess.run(['systemctl','--user','show',unit,'--property=LoadState,ActiveState,SubState,MainPID,Description,ControlGroup,Result'],capture_output=True,text=True,timeout=3)
        value=dict(line.split('=',1) for line in result.stdout.splitlines() if '=' in line)
        if not value:raise RuntimeError('systemd 用户服务不可用')
        return value
    def populated(self,value):
        group=value.get('ControlGroup','')
        if not group:return False
        path=(Path('/sys/fs/cgroup')/group.lstrip('/')).resolve()
        if not path.is_relative_to('/sys/fs/cgroup'):raise RuntimeError('非法 cgroup')
        try:return 'populated 1' in (path/'cgroup.events').read_text()
        except FileNotFoundError:return False
    def start(self,session,script,config,map_pcd):
        subprocess.run(['systemd-run','--user','--collect','--unit='+UNIT,
            '--property=Description='+MARKER+session.name,'--property=KillMode=control-group','--property=KillSignal=SIGINT',
            '--property=TimeoutStopSec=12','--property=SendSIGKILL=yes','--property=BindsTo=d1max-web-managed.service',
            '--property=PartOf=d1max-web-managed.service','--property=After=d1max-web-managed.service',
            '--property=StandardOutput=append:'+str(session/'runtime.log'),'--property=StandardError=append:'+str(session/'runtime.log'),
            '/usr/bin/bash',str(script),str(config),str(session),str(map_pcd)],capture_output=True,text=True,check=True,timeout=6)
    def stop(self):
        subprocess.run(['systemctl','--user','stop',UNIT],capture_output=True,text=True,check=True,timeout=18)

class LocalizationRuntime:
    def __init__(self,root,project_root,nav_root,*,system=None):
        self.root=Path(root).resolve();self.root.mkdir(parents=True,exist_ok=True)
        self.project_root=Path(project_root).resolve();self.nav_root=Path(nav_root).resolve();self.system=system or Systemd()
        self.config=self.nav_root/'src/d1max_localization/config/localization.yaml'
        self.script=self.project_root/'d1max_ros2/map_manager/scripts/start_localization.sh'
        self.state_path=self.root/'state.json';self.lock=threading.RLock();self.state={'phase':'stopped','id':None,'version_id':None,'error':None}
        if self.state_path.is_file():
            try:
                value=json.loads(self.state_path.read_text())
                if value.get('id') and not re.fullmatch('[a-f0-9]{32}',value['id']):raise ValueError('bad session')
                self.state.update(value)
            except (ValueError,AttributeError,TypeError):self.state['phase']='conflict';self.state['error']='定位状态文件损坏，拒绝覆盖'

    @property
    def pinned_id(self):
        return self.state.get('version_id') if self.state.get('phase') in {'starting','running','stopping','detached','conflict'} else None

    @contextmanager
    def transaction(self):
        with self.lock, (self.root/'lifecycle.lock').open('a') as stream:
            fcntl.flock(stream,fcntl.LOCK_EX)
            try:yield
            finally:fcntl.flock(stream,fcntl.LOCK_UN)

    def persist(self):atomic_json(self.state_path,self.state)

    def owned(self,unit):
        return bool(self.state.get('id')) and unit.get('Description')==MARKER+self.state['id']

    def manager(self,path='/v1/status',body=None):
        # Credential remains server-side, is never returned to the browser or logs.
        credential=self.project_root/'d1max_ros2/foxglove_d1max/config/manager.local.json'
        try:
            if credential.stat().st_mode & 0o077:raise ValueError('credential permissions')
            token=json.loads(credential.read_text())['token']
            request=Request('http://127.0.0.1:8771'+path,data=None if body is None else json.dumps(body).encode(),headers={'Authorization':'Bearer '+token,'Content-Type':'application/json'})
            with build_opener(ProxyHandler({})).open(request,timeout=3) as response:return json.load(response)
        except HTTPError as error:
            try:reason=json.load(error).get('error','连接管理请求被拒绝')
            except (ValueError,AttributeError):reason='连接管理请求被拒绝'
            raise HTTPException(409,reason) from None
        except (OSError,ValueError,KeyError):raise HTTPException(503,'本机连接管理器不可用；请检查 d1max-session-manager.service') from None

    def connect(self):
        status=self.manager()
        return self.manager('/v1/monitor/start',{'request_id':uuid.uuid4().hex,'instance':status['instance']})['monitor']

    def snapshot(self,connection=True):
        with self.lock:
            health={};logs=[]
            try:
                unit=self.system.show(UNIT);alive=unit.get('ActiveState') in {'active','activating','deactivating'} or self.system.populated(unit)
                if alive and not self.owned(unit):
                    self.state.update(phase='conflict',error='定位服务被未知会话占用；不会清理或接管')
                elif alive:
                    self.state['phase']='stopping' if unit.get('ActiveState')=='deactivating' else 'running'
                elif self.state.get('phase') in {'starting','running','stopping','detached'}:
                    self.state.update(phase='failed',error='定位进程已退出，请核对日志后手动启动')
                if self.state.get('id'):
                    directory=self.root/self.state['id']
                    try:
                        candidate=json.loads((directory/'status.json').read_text())
                        if alive and self.owned(unit) and unit.get('ActiveState')=='active' and candidate.get('session_id')==self.state['id'] and 0<=time.time()-candidate.get('wall_time',0)<2.:
                            health=candidate
                    except (OSError,ValueError,TypeError):pass
                    try:
                        with (directory/'runtime.log').open('rb') as stream:
                            stream.seek(0,2);stream.seek(max(0,stream.tell()-12000));logs=stream.read().decode('utf-8',errors='replace').splitlines()[-50:]
                    except OSError:pass
            except (OSError,RuntimeError,subprocess.SubprocessError):unit={};self.state.update(phase='conflict',error='无法确认定位服务状态，禁止重复启动')
            try:backend=yaml.safe_load(self.config.read_text()).get('localization_pipeline',{}).get('ros__parameters',{}).get('backend','legacy_ekf')
            except (OSError,ValueError,AttributeError,yaml.YAMLError):backend='unavailable'
            installed=(self.nav_root/'install/d1max_localization/lib/d1max_localization/fused_icp_matcher').is_file()
            if backend=='lio_pcd':
                installed=installed and (self.nav_root/'install/faster_lio/lib/faster_lio/run_mapping_online').is_file() and (self.nav_root/'install/d1max_localization/lib/d1max_localization/lio_localizer').is_file()
                installed=installed and all(os.access(self.nav_root/path,os.X_OK) for path in ('install/faster_lio/lib/faster_lio/run_mapping_online','install/d1max_localization/lib/d1max_localization/lio_localizer'))
            result={**self.state,'unit':UNIT,'pid':int(unit.get('MainPID') or 0),'health':health,'logs':logs,'backend':backend,
                    'installed':installed and backend in {'lio_pcd','legacy_ekf'},
                    'config_path':str(self.config),'foxglove_url':'ws://127.0.0.1:8769','navigation_ready':False}
        if connection:
            try:result['connection']=self.manager()['monitor']
            except HTTPException as error:result['connection']={'phase':'unavailable','error':str(error.detail),'active':False}
        return result

    def recover(self):
        self.snapshot(connection=False)

    def start(self,version):
        with self.transaction():
            current=self.snapshot(connection=False)
            if current['phase'] in {'running','starting','stopping','conflict','detached'}:raise HTTPException(409,'定位已运行、状态转换中或存在进程冲突，请先停止并核对')
            web=self.system.show('d1max-web-managed.service')
            if int(web.get('MainPID') or 0)!=os.getpid():raise HTTPException(409,'请使用受托管的 Web 启动定位，以保证关闭 Web 时清理全部定位节点')
            connection=self.manager()['monitor']
            if not connection.get('active') or not connection.get('health',{}).get('sdk_fresh') or not connection.get('health',{}).get('lidar_fresh') or connection.get('health',{}).get('replay') is not False:
                raise HTTPException(409,'请先连接数据链路，并等待 SDK 和雷达实时数据；回放不能作为实机定位启动条件')
            if not current['installed'] or not self.script.is_file() or not self.config.is_file():raise HTTPException(409,'定位程序尚未构建，请先完成安装')
            if not version['selected'] or version['archived'] or not version['complete']:raise HTTPException(409,'只能启动当前选用且完整的 2D 版本对应定位点云')
            map_pcd=Path(version['map_yaml_path']).parent/'localization.pcd'
            identifier=uuid.uuid4().hex;session=self.root/identifier;session.mkdir()
            value=yaml.safe_load(self.config.read_text())
            (session/'localization.yaml').write_text(yaml.safe_dump(value,allow_unicode=True,sort_keys=False))
            atomic_json(session/'session.json',{'id':identifier,'version_id':version['id'],'map_pcd':str(map_pcd),'created_at':time.time()})
            self.state={'id':identifier,'phase':'starting','version_id':version['id'],'map_name':version['name'],'started_at':time.time(),'error':None}
            self.persist()
            try:self.system.start(session,self.script,session/'localization.yaml',map_pcd)
            except (OSError,subprocess.SubprocessError):
                self.state.update(phase='failed',error='定位服务启动未确认；请查看状态和日志，不会自动重试');self.persist()
                raise HTTPException(503,self.state['error']) from None
            return self.snapshot(connection=False)

    def stop(self):
        with self.transaction():
            unit=self.system.show(UNIT)
            alive=unit.get('ActiveState') in {'active','activating','deactivating'} or self.system.populated(unit)
            if alive:
                if not self.owned(unit):raise HTTPException(409,'拒绝停止非当前工作区拥有的定位会话')
                self.state['phase']='stopping';self.persist();self.system.stop()
                after=self.system.show(UNIT)
                if self.system.populated(after) or after.get('ActiveState') in {'active','activating','deactivating'}:
                    raise HTTPException(409,'定位进程组尚未清空，禁止重复启动')
            self.state.update(phase='stopped',error=None,stopped_at=time.time());self.persist()
            return self.snapshot(connection=False)

    def initial_pose(self,request):
        with self.transaction():
            current=self.snapshot(connection=False)
            if current['phase']!='running' or not current['health']:raise HTTPException(409,'定位服务尚未就绪')
            mailbox=self.root/self.state['id']/'initial_pose.json'
            if mailbox.exists():
                previous=json.loads(mailbox.read_text())
                if time.time()-previous.get('created_at',0)<5 and (current['health'].get('command_result') or {}).get('id')!=previous.get('id'):
                    raise HTTPException(409,'上一个初值请求尚未确认，请稍后核对结果')
            command={'id':uuid.uuid4().hex,'session_id':self.state['id'],'created_at':time.time(),**request.model_dump()}
            atomic_json(self.root/self.state['id']/'initial_pose.json',command)
            return {'request_id':command['id'],'accepted':False,'status':'queued','message':'等待定位节点核对，仅设置匹配初值，不移动机器人'}

    def close(self):
        try:self.stop()
        except (OSError,RuntimeError,subprocess.SubprocessError,HTTPException):pass  # BindsTo + KillMode enforce cleanup on Web unit exit.

def create_localization_router(runtime,grid,shared_lock,other_busy):
    def check_write(request:ApiRequest):
        if request.method in {'GET','HEAD','OPTIONS'}:return
        if request.headers.get('content-type','').split(';')[0]!='application/json':raise HTTPException(415,'定位操作必须使用 JSON 请求')
        origin=request.headers.get('origin')
        if origin and origin not in {'http://127.0.0.1:8766','http://localhost:8766','http://127.0.0.1:5173','http://localhost:5173'}:
            raise HTTPException(403,'不允许来自其他网站的定位操作')
    router=APIRouter(prefix='/api/localization',dependencies=[Depends(check_write)])
    @router.get('/overview')
    def overview():return runtime.snapshot()
    @router.get('/config')
    def config():return FileResponse(runtime.config,filename='d1max-localization.yaml')
    @router.post('/connect',status_code=202)
    def connect():return runtime.connect()
    @router.post('/start',status_code=202)
    def start(request:StartRequest):
        with shared_lock,grid.lock:
            if other_busy() or grid.job['running']:raise HTTPException(409,'建图或点云处理任务运行中，请先结束再启动定位')
            return runtime.start(grid.item(request.version_id))
    @router.post('/stop')
    def stop():return runtime.stop()
    @router.post('/initial-pose',status_code=202)
    def initial(request:InitialRequest):return runtime.initial_pose(request)
    return router
