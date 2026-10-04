"""The shared Web lifecycle implementation for the unique BT navigation session.

Only private, pinned activation can select a release. Web never owns a goal or
execution permit and never falls back to the historical preview launcher.
"""
from contextlib import contextmanager
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import threading
import time
import uuid
from typing import Literal

from fastapi import APIRouter, Depends, HTTPException, Request
from .live_planning import ALLOWED_ORIGINS, EmptyRequest
from .localization import Systemd
from .mainline_release import MainlineReleaseProbe,root_dispatchers,verify_release_files
from .mainline_views import OwnedNavigationViews

UNIT='d1max-single-floor-navigation.service'
OWNER='D1MAX-SINGLE-FLOOR:'
ACTIVE={'active','activating','deactivating','reloading'}
PURPOSES={'planning_only','execution'}
CONFLICTS=('d1max-live-planning-view.service','d1max-localization-managed.service',
    'd1max-nav2-localization-test.service','d1max-pct-preview.service','d1max-pct-scan.service')
MAINLINE={'entry_module':'d1max_pct_scan.navigation_session','task_owner':'BehaviorTree.CPP'}


class MonitorStatusProbe:
    """One bounded read-only worker. Slow manager GETs cannot stall every poll."""
    def __init__(self, localization):
        self.localization=localization;self.lock=threading.Lock()
        self.reading=False;self.checked=0.;self.value={};self.generation=0

    def _read(self,generation):
        value={'active':False,'phase':'unavailable','error':'连接管理器不可用'}
        try:
            value=self.localization.manager()['monitor']
            if not isinstance(value,dict):raise ValueError('monitor_object_required')
        except (HTTPException,OSError,ValueError,KeyError,TypeError):
            value={'active':False,'phase':'unavailable','error':'连接管理器不可用'}
        finally:
            with self.lock:
                if generation==self.generation:
                    self.value=value;self.checked=time.monotonic()
                self.reading=False

    def update(self,value):
        with self.lock:
            self.generation+=1;self.value=dict(value);self.checked=time.monotonic()

    def snapshot(self):
        with self.lock:
            age=time.monotonic()-self.checked
            if not self.reading and age>=1.5:
                self.reading=True
                threading.Thread(target=self._read,args=(self.generation,),daemon=True,name='mainline-monitor-read').start()
            if not self.checked or age>3.:
                return {'active':False,'phase':'checking','health':{},'read_only':True}
            value=dict(self.value)
            value['health']=dict(value['health']) if isinstance(value.get('health'),dict) else {}
            value['read_only']=True
            value['observed_at_unix']=time.time()-age
            return value


def digest(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda:stream.read(1048576),b''):h.update(block)
    return h.hexdigest()


def private_json(path,limit=1048576):
    path=Path(path)
    stat=path.stat()
    if path.is_symlink() or not path.is_file() or stat.st_uid!=os.getuid() or stat.st_mode&0o077 or stat.st_size>limit:
        raise ValueError('private_owned_json_required')
    value=json.loads(path.read_text())
    if not isinstance(value,dict):raise ValueError('object_required')
    return value


def persist(path,value):
    temporary=path.with_suffix('.tmp')
    fd=os.open(temporary,os.O_WRONLY|os.O_CREAT|os.O_TRUNC|os.O_NOFOLLOW,0o600)
    with os.fdopen(fd,'w') as stream:json.dump(value,stream)
    os.replace(temporary,path)


class SingleFloorRuntime:
    def __init__(self,root,nav_root,localization_runtime,*,system=None,runner=None,release_probe=None,monitor_probe=None,views=None):
        self.root=Path(root).resolve();self.root.mkdir(parents=True,exist_ok=True)
        self.nav=Path(nav_root).resolve();self.localization=localization_runtime
        self.activation=self.root/'activation.json';self.state_path=self.root/'state.json'
        self.system=system or Systemd();self.runner=runner or subprocess.run
        self.lock=threading.Lock()
        self.release_probe=release_probe or MainlineReleaseProbe(self.nav,self.activation)
        self.monitor_probe=monitor_probe or MonitorStatusProbe(self.localization)
        self.views=views or OwnedNavigationViews(self.nav,self.system,self.runner)

    def _state(self):
        return private_json(self.state_path) if self.state_path.exists() else {}

    @contextmanager
    def transaction(self):
        if not self.lock.acquire(False):raise HTTPException(409,'导航服务正在切换')
        try:
            lease=self.nav/'log/.localization-start.lock';lease.parent.mkdir(parents=True,exist_ok=True)
            with lease.open('a') as stream:
                try:fcntl.flock(stream,fcntl.LOCK_EX|fcntl.LOCK_NB)
                except BlockingIOError:raise HTTPException(409,'另一个定位服务正在切换') from None
                yield
        finally:self.lock.release()

    def _activation(self):
        a=private_json(self.activation)
        if a.get('schema_version')!=3 or a.get('profile')!='single_floor_live':raise ValueError('activation_schema')
        if a.get('purpose') not in PURPOSES:raise ValueError('explicit_activation_purpose_required')
        release=Path(a['release_root']).resolve(strict=True)
        if not release.is_relative_to(self.nav/'experiments'):raise ValueError('explicit_release_root_required')
        entry=Path(a['entrypoint'])
        seal=Path(a['release_manifest'])
        if (entry.is_symlink() or seal.is_symlink() or not entry.is_absolute() or not seal.is_absolute()
                or entry not in root_dispatchers(self.nav) or entry.resolve(strict=True)!=entry
                or not seal.resolve(strict=True).is_relative_to(release)
                or not os.access(entry,os.X_OK)):raise ValueError('sealed_entrypoint_required')
        if digest(seal)!=a['release_manifest_sha256'] or digest(entry)!=a['entrypoint_sha256']:
            raise ValueError('activated_release_changed')
        legacy_entry=self.nav/'tools/single_floor_entry.sh'
        # The generic wrapper execs this workspace dispatcher even when the
        # selected release seals a separate frozen copy of its implementation.
        if entry==self.nav/'tools/navigation_entry.sh':
            if (legacy_entry.is_symlink() or not legacy_entry.is_file()
                    or legacy_entry.resolve(strict=True)!=legacy_entry
                    or not os.access(legacy_entry,os.X_OK)
                    or digest(legacy_entry)!=a.get('compatibility_entrypoint_sha256')):
                raise ValueError('activated_compatibility_entrypoint_changed')
        manifest=json.loads(seal.read_text())
        if not isinstance(manifest,dict):raise ValueError('release_manifest_object_required')
        files=manifest.get('files')
        if manifest.get('schema')!=3 or not isinstance(files,dict) or not files:
            raise ValueError('release_files_not_sealed')
        # Private activation pins either current root dispatcher. Releases
        # retain their original sealed implementation and descriptor contract;
        # the generic wrapper does not relabel or replace legacy sealed files.
        descriptor_path=release/'release.json'
        if descriptor_path.is_file():
            descriptor=json.loads(descriptor_path.read_text())
            if (descriptor.get('schema')!=1 or descriptor.get('sealed_manifest')!=seal.name
                    or files.get(str(descriptor_path))!=digest(descriptor_path)):
                raise ValueError('release_descriptor_not_sealed')
            if 'tools_source_root' in descriptor:
                relative=Path(descriptor['tools_source_root'])
                if relative.is_absolute() or '..' in relative.parts:
                    raise ValueError('frozen_tools_outside_release')
                tools=(release/relative).resolve(strict=True)
                frozen_entry=tools/'single_floor_entry.sh'
                if (not tools.is_relative_to(release) or frozen_entry.is_symlink()
                        or manifest.get('actual_tools_root')!=str(tools)
                        or files.get(str(frozen_entry))!=digest(frozen_entry)
                        or 'startup_closure' not in manifest):
                    raise ValueError('frozen_entry_not_sealed')
            elif files.get(str(legacy_entry))!=digest(legacy_entry):
                raise ValueError('release_files_not_sealed')
        elif files.get(str(legacy_entry))!=digest(legacy_entry):
            raise ValueError('release_files_not_sealed')
        verify_release_files(self.nav,release,seal,expected_manifest_sha256=a['release_manifest_sha256'])
        if a['purpose']=='execution':
            record=Path(a['physical_acceptance_record'])
            if not record.is_absolute() or digest(record)!=a['physical_acceptance_record_sha256']:
                raise ValueError('physical_record_missing_or_changed')
        policy=a.get('sdk_session_policy','fixed')
        if policy not in ('fixed','bind_current_on_start'):
            raise ValueError('sdk_session_policy_invalid')
        if policy=='bind_current_on_start':
            if a['purpose']!='planning_only':raise ValueError('execution_requires_fixed_sdk_session')
        elif not re.fullmatch(r'[A-Za-z0-9_.-]{1,128}',a['expected_sdk_session']):
            raise ValueError('existing_sdk_session_required')
        return a

    def _unit(self):
        # Include PID, invocation and exit evidence, not merely a service name.
        r=self.runner(['systemctl','--user','show',UNIT,
            '--property=ActiveState,Description,MainPID,InvocationID,ControlGroup,ExecMainStatus,Result'],
            check=True,capture_output=True,text=True,timeout=3)
        value=dict(line.split('=',1) for line in r.stdout.splitlines() if '=' in line)
        if value.get('ActiveState') not in ACTIVE|{'inactive','failed'}:raise ValueError('unit_state_unverified')
        return value

    def _owned(self,state,unit):
        if unit.get('Description')!=OWNER+state.get('id',''):return False
        if state.get('invocation_id') and unit.get('InvocationID')!=state['invocation_id']:return False
        pid=int(unit.get('MainPID') or 0)
        return not pid or pid==state.get('pid')

    @staticmethod
    def _session_scope(session,release):
        import yaml
        route_path=Path(session['crossfloor_route_config']).resolve(strict=True)
        if not route_path.is_relative_to(release):return None
        route=yaml.safe_load(route_path.read_text())
        floors=route.get('floor_ids') or [route['floor_id']]
        return dict(floors=[str(f) for f in floors],stairs_enabled=route.get('stairs_enabled') is True)

    def _scope(self):
        """Display-only coverage of the activated release; never a launch input.

        The UI names what the sealed map actually covers instead of a fixed
        profile label, so a later multi-floor release needs no UI change.
        """
        try:
            a=private_json(self.activation)
            release=Path(a['release_root']).resolve(strict=True)
            seal=Path(a['release_manifest']).resolve(strict=True)
            if not release.is_relative_to(self.nav/'experiments') or not seal.is_relative_to(release):return None
            # The route the sealed preflight session actually planned on.
            session=json.loads(Path(json.loads(seal.read_text())['preflight_session']).read_text())
            return self._session_scope(session,release)
        except (OSError,ValueError,KeyError,TypeError,AttributeError,ImportError):
            return None

    def _connection(self):
        return self.monitor_probe.snapshot()

    @staticmethod
    def _data_ready(monitor,expected_session=None):
        health=monitor.get('health',{})
        if not isinstance(health,dict):return False
        stamp=health.get('sdk_session_source_stamp')
        return (monitor.get('active') is True and health.get('sdk_fresh') is True
                and health.get('lidar_fresh') is True and health.get('replay') is False
                and type(stamp) in (int,float) and 0<=time.time()-stamp<=1.
                and isinstance(health.get('sdk_session'),str)
                and re.fullmatch(r'[A-Za-z0-9_.-]{1,128}',health['sdk_session']) is not None
                and (expected_session is None or health.get('sdk_session')==expected_session))

    def snapshot(self,*,connection=False):
        try:
            state=self._state();unit=self._unit()
            busy=unit['ActiveState'] in ACTIVE or self.system.populated(unit)
            owned=bool(state) and self._owned(state,unit)
            phase=('running' if unit['ActiveState']=='active' else 'stopping') if busy and owned else 'conflict' if busy else state.get('phase','stopped')
            if not busy and phase in ('running','starting','stopping'):phase='failed'
            # The running process keeps its admitted purpose even if an admin
            # stages a different activation for the next session. Display-only
            # capability is not execution confirmation or a motion permit.
            try: configured_purpose=private_json(self.activation).get('purpose')
            except (OSError,ValueError,KeyError,TypeError):configured_purpose=None
            if configured_purpose not in PURPOSES:configured_purpose=None
            purpose=state.get('purpose') if owned and (busy or phase in ('failed','needs_review')) else configured_purpose
            if purpose not in PURPOSES:purpose=None
            release=self.release_probe.snapshot()
            configured=release.get('readiness')=='ready' and configured_purpose is not None
            monitor=self._connection() if connection else {'active':False,'phase':'unchecked','health':{}}
            try:
                activation=private_json(self.activation)
                dynamic=activation.get('purpose')=='planning_only' and activation.get('sdk_session_policy')=='bind_current_on_start'
                expected_session=None if dynamic else activation.get('expected_sdk_session')
            except (OSError,ValueError,KeyError,TypeError):expected_session=None;dynamic=False
            data_ready=self._data_ready(monitor,expected_session) if expected_session or dynamic else False
            blocker=(release.get('reason') if not configured else
                     '请先连接机器狗并等待雷达、运动数据' if not data_ready else
                     '当前服务需要先停止或核对退出结果' if busy or phase!='stopped' or state.get('quarantined') else '')
            scope=state.get('scope') if owned and busy else self._scope()
            release={**release,'scope':scope,'running_id':Path(state['release_root']).name if owned and busy and state.get('release_root') else None,
                     'running_manifest_sha256':state.get('release_manifest_sha256') if owned and busy else None}
            return dict(profile='single_floor_live',phase=phase,busy=busy,session_id=state.get('id'),
                pid=int(unit.get('MainPID') or 0) if owned else None,exit_code=unit.get('ExecMainStatus') if owned else state.get('exit_code'),
                release_configured=configured,owned=owned,
                quarantined=state.get('quarantined',False) or phase=='failed',
                navigation_controls='rviz',sdk_auto_connect=False,scope=scope,purpose=purpose,
                motion_capable=purpose=='execution',execution_available=configured and purpose=='execution' and data_ready
                    and monitor.get('health',{}).get('execution_v3_enabled') is True
                    and monitor.get('health',{}).get('execution_record_valid') is True,
                mainline={**MAINLINE,'entrypoint':str(self.nav/'tools/navigation_entry.sh'),
                    'runtime_entrypoint':state.get('entrypoint',str(self.nav/'tools/single_floor_entry.sh')) if owned and busy else None,
                    'runtime_entrypoint_sha256':state.get('entrypoint_sha256') if owned and busy else None},
                release=release,connection=monitor,start_blocker=blocker or '',
                view_error=state.get('view_error','') if owned and busy else '',
                stop_report=state.get('stop_report'),
                can_start=configured and data_ready and not busy and phase=='stopped' and not state.get('quarantined'),
                snapshot_at_unix=time.time())
        except (OSError,ValueError,KeyError,TypeError,RuntimeError,subprocess.SubprocessError):
            return dict(profile='single_floor_live',phase='conflict',busy=True,error='服务所有权或退出状态待核对',
                purpose=None,scope=None,motion_capable=False,execution_available=False,owned=False,
                mainline={**MAINLINE,'entrypoint':str(self.nav/'tools/navigation_entry.sh')},
                release={},can_start=False,start_blocker='服务所有权或退出状态待核对',snapshot_at_unix=time.time())

    def connect(self):
        # Explicit operator action, never part of start() or a status read.
        with self.transaction():
            current=self.snapshot()
            if current['busy'] or current.get('quarantined'):
                raise HTTPException(409,'导航运行中或退出待核对，不重连数据会话')
            for name in CONFLICTS:
                unit=self.system.show(name)
                if unit.get('ActiveState') in ACTIVE or self.system.populated(unit):
                    raise HTTPException(409,'请先停止已有定位或规划服务')
            monitor=self.localization.manager()['monitor']
            if monitor.get('active') is True:
                raise HTTPException(409,'已有机器狗数据会话，不重复连接')
            if (monitor.get('phase') in {'checking','connecting','disconnecting','starting','stopping'}
                    or monitor.get('health',{}).get('replay') is True):
                raise HTTPException(409,'连接切换中或正在回放，不重连')
            connected=self.localization.connect()
            if not isinstance(connected,dict):raise HTTPException(503,'连接结果待核对')
            self.monitor_probe.update(connected)
            return self.snapshot(connection=True)

    def _open_view(self,state,layout):
        if layout not in ('global','local'):raise HTTPException(422,'无效 RViz 布局')
        try:
            record=self.views.open(state,layout)
            state.setdefault('views',{})['single']=record;state['view_error']=''
            persist(self.state_path,state)
        except (OSError,ValueError,KeyError,TypeError,subprocess.SubprocessError):
            state['view_error']='RViz 未打开，请检查桌面环境后重开视图'
            persist(self.state_path,state)
            raise HTTPException(409,state['view_error']) from None

    def open_view(self,layout):
        with self.transaction():
            state=self._state();unit=self._unit()
            if unit.get('ActiveState')!='active' or not self._owned(state,unit):
                raise HTTPException(409,'请先启动本次主线导航服务')
            directory=Path(state['directory'])
            if directory.parent!=self.root/'sessions' or digest(directory/'session.json')!=state['session_sha256']:
                raise HTTPException(409,'本次导航会话绑定变化，不能打开视图')
            self._open_view(state,layout)
            return self.snapshot(connection=True)

    def start(self):
        with self.transaction():
            current=self.snapshot()
            if current['busy'] or current.get('quarantined'):raise HTTPException(409,'服务未清理或停止结果待核对')
            try:a=self._activation()
            except (OSError,ValueError,KeyError,TypeError):
                message=('执行版本或物理验收记录未通过校验' if current.get('purpose')=='execution' else
                         '只规划发布版本未通过校验' if current.get('purpose')=='planning_only' else
                         '未配置有效发布版本和明确用途')
                raise HTTPException(409,message) from None
            web=self.system.show('d1max-web-managed.service')
            if web.get('ActiveState')!='active' or int(web.get('MainPID') or 0)!=os.getpid():
                raise HTTPException(409,'请通过受托管 Web 启动导航服务')
            for name in CONFLICTS:
                state=self.system.show(name)
                if state.get('ActiveState') in ACTIVE or self.system.populated(state):raise HTTPException(409,'请先停止已有定位或规划服务')
            # Read-only existing monitor check. No start/connect/reconnect API.
            monitor=self.localization.manager()['monitor'];health=monitor.get('health',{})
            if not (monitor.get('active') is True and health.get('sdk_fresh') is True and health.get('lidar_fresh') is True and health.get('replay') is False):
                raise HTTPException(409,'需要已有实机数据会话；此入口不会连接 SDK')
            if a.get('sdk_session_policy')=='bind_current_on_start':
                if not self._data_ready(monitor):raise HTTPException(409,'当前数据会话身份或源时间未确认')
                a={**a,'expected_sdk_session':health['sdk_session']}
            if (health.get('sdk_session')!=a['expected_sdk_session']
                    or type(health.get('sdk_session_source_stamp')) not in (int,float)
                    or not 0<=time.time()-health['sdk_session_source_stamp']<=1.):
                raise HTTPException(409,'现有 SDK 数据会话不匹配或已过期；不会自动重连')
            if a['purpose']=='execution' and (health.get('execution_v3_enabled') is not True
                    or health.get('execution_record_valid') is not True):
                raise HTTPException(409,'现有 SDK 会话未绑定已验收执行版本；不会自动重连')
            identifier=uuid.uuid4().hex;directory=self.root/'sessions'/identifier
            directory.parent.mkdir(parents=True,exist_ok=True)
            entry=a['entrypoint']
            # The entry is release-agnostic; the activated release is named explicitly.
            release=str(Path(a['release_root']).resolve(strict=True))
            env={**os.environ,'D1MAX_RELEASE':release,'D1MAX_NAV_ROOT':str(self.nav),'D1MAX_NAV_TRANSPORT':'live'}
            operation='prepare';launched=False
            try:
                command=[entry,'prepare','--session',str(directory),'--transport-mode','live',
                    '--purpose',a['purpose'],'--expected-sdk-session',a['expected_sdk_session']]
                if a['purpose']=='execution':command+=['--acceptance-record',a['physical_acceptance_record']]
                self.runner(command,check=True,capture_output=True,text=True,timeout=40,env=env)
                operation='seal'
                self.runner([entry,'seal','--session',str(directory)],check=True,capture_output=True,text=True,timeout=30,env=env)
                session=private_json(directory/'session.json')
                if (session.get('pipeline_contract')!='single_floor_v3' or session.get('transport_mode')!='live'
                        or session.get('purpose')!=a['purpose']
                        or session.get('expected_sdk_session')!=a['expected_sdk_session']
                        or session.get('motion_control_enabled') is not (a['purpose']=='execution')):
                    raise ValueError('prepared_profile_mismatch')
                state=dict(id=session['id'],directory=str(directory),session_sha256=digest(directory/'session.json'),
                    phase='starting',pid=0,invocation_id='',quarantined=False,purpose=a['purpose'],
                    release_root=release,release_manifest_sha256=a['release_manifest_sha256'],
                    entrypoint=a['entrypoint'],entrypoint_sha256=a['entrypoint_sha256'],
                    compatibility_entrypoint_sha256=a.get('compatibility_entrypoint_sha256'),
                    view_contract=session.get('view_contract'))
                try:state['scope']=self._session_scope(session,Path(release))
                except (OSError,ValueError,KeyError,TypeError,AttributeError,ImportError):state['scope']=None
                persist(self.state_path,state)
                operation='launch';launched=True
                self.runner(['systemd-run','--user','--collect','--unit='+UNIT,
                    '--property=Description='+OWNER+session['id'],'--property=Type=exec',
                    '--property=KillMode=mixed','--property=KillSignal=SIGTERM','--property=TimeoutStopSec=30',
                    '--property=BindsTo=d1max-web-managed.service','--property=PartOf=d1max-web-managed.service',
                    '--property=After=d1max-web-managed.service','--setenv=D1MAX_RELEASE='+release,
                    '--setenv=D1MAX_NAV_ROOT='+str(self.nav),
                    '--setenv=D1MAX_NAV_TRANSPORT=live',
                    '--property=StandardOutput=append:'+str(directory/'service.log'),
                    '--property=StandardError=append:'+str(directory/'service.log'),
                    entry,'run','--session',str(directory)],check=True,capture_output=True,text=True,timeout=10)
                unit=self._unit();state.update(pid=int(unit.get('MainPID') or 0),invocation_id=unit.get('InvocationID',''))
                if unit.get('Description')!=OWNER+session['id'] or state['pid']<=0 or not state['invocation_id']:raise ValueError('start_identity_unconfirmed')
                state['phase']='running';persist(self.state_path,state)
            except (OSError,ValueError,KeyError,TypeError,subprocess.SubprocessError):
                # Never kill an unverified unit or retry an uncertain launch.
                if launched:
                    state.update(phase='needs_review',quarantined=True)
                    persist(self.state_path,state)
                    raise HTTPException(503,'启动结果待确认，请检查本次会话；禁止重复启动') from None
                message='主线会话准备失败，未启动导航进程' if operation=='prepare' else '主线会话校验失败，未启动导航进程'
                raise HTTPException(409,message) from None
            try:self._open_view(state,'global')
            except HTTPException:pass  # Display failure is not a core failure.
            return self.snapshot(connection=True)

    def stop(self):
        with self.transaction():
            state=self._state();unit=self._unit()
            busy=unit['ActiveState'] in ACTIVE or self.system.populated(unit)
            if not busy:
                if state and self._owned(state,unit):self.views.stop(state)
                return self.snapshot(connection=True)
            if not state or not self._owned(state,unit):raise HTTPException(409,'所有权未核实，不停止其他进程')
            directory=Path(state['directory'])
            if directory.parent!=self.root/'sessions' or digest(directory/'session.json')!=state['session_sha256']:
                raise HTTPException(409,'会话绑定变化，停止结果需人工核对')
            state['phase']='stopping';persist(self.state_path,state)
            # KillMode=mixed signals only the supervisor first. Its drain runs
            # while perception/tracker and the external SDK writer remain alive.
            try:self.runner(['systemctl','--user','stop',UNIT],check=True,capture_output=True,text=True,timeout=35)
            except (OSError,subprocess.SubprocessError):raise HTTPException(503,'停止未确认，不允许重复启动') from None
            after=self._unit()
            if after['ActiveState'] in ACTIVE or self.system.populated(after):raise HTTPException(409,'本次进程组尚未清空')
            try:drain=json.loads((directory/'shutdown.json').read_text())
            except (OSError,ValueError):drain={}
            clean=(drain.get('session_id')==state['id'] and drain.get('request_accepted') is True
                   and drain.get('software_retired') is True)
            state['stop_report']={
                'software_retired':clean,
                'physical_stop_confirmed':drain.get('session_id')==state['id'] and drain.get('physical_stop_confirmed') is True,
            }
            try:self.views.stop(state)
            except (OSError,ValueError,KeyError,TypeError,subprocess.SubprocessError):
                clean=False;state['view_error']='本次视图进程退出未确认'
            state.update(phase='stopped' if clean else 'needs_review',quarantined=not clean,exit_code=after.get('ExecMainStatus'))
            persist(self.state_path,state)
            if not clean:raise HTTPException(409,'进程已退出，但停止与退役未确认，需人工核对')
            return self.snapshot(connection=True)

    def close(self):
        try:self.stop()
        except (HTTPException,OSError,ValueError,KeyError,subprocess.SubprocessError):pass


def _create_session_router(runtime,shared_lock,other_busy,*,prefix):
    def write_check(request:Request):
        if request.method=='GET':return
        if request.headers.get('content-type','').split(';')[0]!='application/json':raise HTTPException(415,'需要 JSON 请求')
        if request.headers.get('origin') and request.headers['origin'] not in ALLOWED_ORIGINS:raise HTTPException(403,'请求来源不允许')
    router=APIRouter(prefix=prefix,dependencies=[Depends(write_check)])
    @router.get('/overview')
    def overview():return runtime.snapshot(connection=True)
    @router.post('/view/{layout}',status_code=202)
    def view(layout:Literal['global','local'],_request:EmptyRequest):return runtime.open_view(layout)
    @router.post('/connect',status_code=202)
    def connect(_request:EmptyRequest):
        if not shared_lock.acquire(False):raise HTTPException(409,'其他任务正在启动')
        try:
            if other_busy():raise HTTPException(409,'请先停止建图、录包或其他导航')
            return runtime.connect()
        finally:shared_lock.release()
    @router.post('/start',status_code=202)
    def start(_request:EmptyRequest):
        if not shared_lock.acquire(False):raise HTTPException(409,'其他任务正在启动')
        try:
            if other_busy():raise HTTPException(409,'请先停止建图、录包或其他导航')
            return runtime.start()
        finally:shared_lock.release()
    @router.post('/stop')
    def stop(_request:EmptyRequest):return runtime.stop()
    return router


def create_single_floor_router(runtime,shared_lock,other_busy):
    """Compatibility routes bound to the caller's existing session runtime."""
    return _create_session_router(runtime,shared_lock,other_busy,prefix='/api/single-floor')
