"""The shared Web lifecycle implementation for the unique BT navigation session.

Only private, pinned activation can select a release. Web never owns a goal or
execution permit and never falls back to the historical preview launcher.
"""
from contextlib import contextmanager
import fcntl
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import re
import stat
import subprocess
import threading
import time
import uuid
from typing import Literal

from fastapi import APIRouter, Depends, HTTPException, Request
from pydantic import BaseModel, ConfigDict
from .live_planning import ALLOWED_ORIGINS, EmptyRequest
from .localization import Systemd
from .mainline_release import (MainlineReleaseProbe,root_dispatchers,verify_release_files,
                              activation_launch_config)
from .mainline_views import OwnedNavigationViews,VIEW_UNIT,LEGACY_VIEW_UNITS

UNIT='d1max-single-floor-navigation.service'
OWNER='D1MAX-SINGLE-FLOOR:'
ACTIVE={'active','activating','deactivating','reloading'}
PURPOSES={'planning_only','execution'}
CONFLICTS=('d1max-live-planning-view.service','d1max-localization-managed.service',
    'd1max-nav2-localization-test.service','d1max-pct-preview.service','d1max-pct-scan.service')
MAINLINE={'entry_module':'d1max_pct_scan.navigation_session','task_owner':'BehaviorTree.CPP'}
TASK_WRITER_PROTOCOL='task_follower_v1'


class NavigationPurposeRequest(BaseModel):
    model_config=ConfigDict(extra='forbid')
    # Empty requests retain the admitted launch configuration for old clients.
    # The current UI always sends one of the two explicit purposes.
    purpose: Literal['planning_only','execution'] | None = None


class MonitorStatusProbe:
    """Fresh admission status, with one worker and a bounded request wait.

    Overview must not return the previous UI poll's sample: the UI polls more
    slowly than the SDK identity's one-second lifetime. Source timestamps are
    never rewritten, and start() independently reads the manager again.
    """
    REFRESH_SECONDS=.25
    WAIT_SECONDS=.20
    CACHE_SECONDS=3.

    def __init__(self, localization):
        self.localization=localization;self.lock=threading.Lock()
        self.changed=threading.Condition(self.lock)
        self.reading=False;self.checked=0.;self.value={};self.generation=0

    def _read(self,generation):
        value={'active':False,'phase':'unavailable','error':'连接管理器不可用'}
        try:
            value=self.localization.manager()['monitor']
            if not isinstance(value,dict):raise ValueError('monitor_object_required')
        except (HTTPException,OSError,ValueError,KeyError,TypeError):
            value={'active':False,'phase':'unavailable','error':'连接管理器不可用'}
        finally:
            with self.changed:
                if generation==self.generation:
                    self.value=value;self.checked=time.monotonic()
                self.reading=False
                self.changed.notify_all()

    def update(self,value):
        with self.changed:
            self.generation+=1;self.value=dict(value);self.checked=time.monotonic()
            self.changed.notify_all()

    def snapshot(self):
        with self.changed:
            age=time.monotonic()-self.checked
            if not self.reading and age>=self.REFRESH_SECONDS:
                self.reading=True
                try:threading.Thread(target=self._read,args=(self.generation,),daemon=True,name='mainline-monitor-read').start()
                except RuntimeError:
                    self.reading=False;self.value={'active':False,'phase':'unavailable','health':{}}
                    self.checked=time.monotonic();self.changed.notify_all()
                age=time.monotonic()-self.checked
            if self.reading and age>=self.REFRESH_SECONDS:
                checked=self.checked
                # Condition releases the lock so the worker can commit. A slow
                # or failed manager cannot create more workers or block a poll
                # for its full HTTP timeout.
                self.changed.wait_for(lambda:not self.reading or self.checked!=checked,
                                      timeout=self.WAIT_SECONDS)
                age=time.monotonic()-self.checked
            if not self.checked or age>self.CACHE_SECONDS or (self.reading and age>=self.REFRESH_SECONDS):
                return {'active':False,'phase':'checking','health':{},'read_only':True}
            value=dict(self.value)
            value['health']=dict(value['health']) if isinstance(value.get('health'),dict) else {}
            value['read_only']=True
            value['observed_at_unix']=time.time()-age
            return value


def digest(path):
    h=hashlib.sha256()
    fd=os.open(path,os.O_RDONLY|os.O_NONBLOCK|os.O_CLOEXEC)
    with os.fdopen(fd,'rb') as stream:
        if not stat.S_ISREG(os.fstat(stream.fileno()).st_mode):
            raise ValueError('regular_file_required')
        for block in iter(lambda:stream.read(1048576),b''):h.update(block)
    return h.hexdigest()


def private_json(path,limit=1048576):
    path=Path(path)
    fd=os.open(path,os.O_RDONLY|os.O_NOFOLLOW|os.O_NONBLOCK|os.O_CLOEXEC)
    with os.fdopen(fd,'r') as stream:
        info=os.fstat(stream.fileno())
        if (not stat.S_ISREG(info.st_mode) or info.st_uid!=os.getuid()
                or info.st_mode&0o077 or info.st_size>limit):
            raise ValueError('private_owned_json_required')
        raw=stream.read(limit+1)
        if len(raw)>limit:raise ValueError('private_owned_json_required')
        value=json.loads(raw)
    if not isinstance(value,dict):raise ValueError('object_required')
    return value


def regular_text(path,limit=1048576):
    """Bounded display/config read; a special file is not readable evidence."""
    fd=os.open(path,os.O_RDONLY|os.O_NOFOLLOW|os.O_NONBLOCK|os.O_CLOEXEC)
    with os.fdopen(fd,'r') as stream:
        info=os.fstat(stream.fileno())
        if not stat.S_ISREG(info.st_mode) or info.st_size>limit:
            raise ValueError('bounded_regular_file_required')
        raw=stream.read(limit+1)
        if len(raw)>limit:raise ValueError('bounded_regular_file_required')
        return raw


def persist(path,value):
    temporary=path.with_suffix('.tmp')
    fd=os.open(temporary,os.O_WRONLY|os.O_CREAT|os.O_TRUNC|os.O_NOFOLLOW,0o600)
    with os.fdopen(fd,'w') as stream:json.dump(value,stream)
    os.replace(temporary,path)


class LifecycleCancelled(Exception):
    """The current Web lifecycle was cancelled, not a navigation fault."""


class SingleFloorRuntime:
    STARTUP_ACK_POLL_SECONDS=.1
    STOP_COMMAND_TIMEOUT_SECONDS=35
    def __init__(self,root,nav_root,localization_runtime,*,system=None,runner=None,release_probe=None,monitor_probe=None,views=None,
                 helper_runner=None,popen=None,process_cleanup=None):
        self.root=Path(root).resolve();self.root.mkdir(parents=True,exist_ok=True)
        self.nav=Path(nav_root).resolve();self.localization=localization_runtime
        self.activation=self.root/'activation.json';self.state_path=self.root/'state.json'
        self.system=system or Systemd();self.runner=runner or subprocess.run
        self.lock=threading.RLock();self.changed=threading.Condition(self.lock)
        self.operation=None;self.worker=None;self.generation=0;self.closed=False
        self.helper_runner=helper_runner;self.popen=popen or subprocess.Popen
        self.process_cleanup=process_cleanup;self.helper=None;self.helper_operation=None
        self.release_probe=release_probe or MainlineReleaseProbe(self.nav,self.activation)
        self.monitor_probe=monitor_probe or MonitorStatusProbe(self.localization)
        self.views=views or OwnedNavigationViews(self.nav,self.system,self.runner)

    def _state(self):
        with self.lock:
            return private_json(self.state_path) if self.state_path.exists() else {}

    def _check_operation(self,operation):
        with self.lock:
            if (self.operation is not operation or operation['cancel'].is_set()
                    or operation['generation']!=self.generation):raise LifecycleCancelled()

    def _commit(self,operation,state,*,cleanup=False):
        with self.lock:
            if self.operation is not operation:raise LifecycleCancelled()
            if not cleanup:self._check_operation(operation)
            state={**state,'operation_id':operation['id'],'generation':self.generation}
            persist(self.state_path,state)
            return state

    @contextmanager
    def _startup_lease(self):
        # Only startup competes for this cross-workflow lease. Cancellation and
        # status never wait for it or for prepare/seal/hash verification.
        lease=self.nav/'log/.localization-start.lock';lease.parent.mkdir(parents=True,exist_ok=True)
        with lease.open('a') as stream:
            try:fcntl.flock(stream,fcntl.LOCK_EX|fcntl.LOCK_NB)
            except BlockingIOError:raise HTTPException(409,'另一个定位服务正在切换') from None
            yield

    def _helper_cleanup_function(self,operation):
        if self.process_cleanup is not None:return self.process_cleanup
        if not operation.get('cleanup_function'):
            # Pin the pure helper in this activation's sealed manifest, not the
            # mutable development tree or an imported ROS/navigation runtime.
            path=Path(operation['cleanup_path'])
            if path.is_symlink() or digest(path)!=operation['cleanup_sha256']:
                raise ValueError('web_helper_cleanup_binding_changed')
            spec=importlib.util.spec_from_file_location('_d1max_owned_process_cleanup',path)
            module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
            operation['cleanup_function']=module.stop_owned_children
        return operation['cleanup_function']

    def _cleanup_helper(self,operation,process):
        result=self._helper_cleanup_function(operation)([('web_prepare',process)],interrupt_s=6.,terminate_s=1.,kill_s=1.)
        if not (result.get('all_direct_children_exited') is True and result.get('all_owned_groups_exited') is True):
            raise RuntimeError('web_helper_cleanup_unconfirmed')
        return result

    def _run_helper(self,operation,command,**kwargs):
        """Own cancellable local preflight/prepare/seal processes in Web's cgroup."""
        self._check_operation(operation)
        if self.helper_runner is not None:  # Explicit offline-test injection only.
            result=self.helper_runner(command,**kwargs);self._check_operation(operation);return result
        self._helper_cleanup_function(operation)
        timeout=kwargs.pop('timeout');check=kwargs.pop('check',False)
        kwargs.pop('capture_output',None)
        process=self.popen(command,stdout=subprocess.PIPE,stderr=subprocess.PIPE,start_new_session=True,**kwargs)
        with self.lock:self.helper=process;self.helper_operation=operation
        deadline=time.monotonic()+timeout
        try:
            while True:
                self._check_operation(operation)
                remaining=deadline-time.monotonic()
                if remaining<=0:raise subprocess.TimeoutExpired(command,timeout)
                try:stdout,stderr=process.communicate(timeout=min(self.STARTUP_ACK_POLL_SECONDS,remaining));break
                except subprocess.TimeoutExpired:pass
            self._check_operation(operation)
            result=subprocess.CompletedProcess(command,process.returncode,stdout,stderr)
            if check and result.returncode:raise subprocess.CalledProcessError(result.returncode,command,stdout,stderr)
            return result
        finally:
            # Even a successful direct child can leave a process group behind.
            try:self._cleanup_helper(operation,process)
            except Exception:
                operation['helper_cleanup_unconfirmed']=True
                raise
            with self.lock:
                if self.helper is process:self.helper=None;self.helper_operation=None

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

    @contextmanager
    def _admission(self):
        if not self.lock.acquire(False):raise HTTPException(409,'导航状态正在更新')
        try:yield
        finally:self.lock.release()

    def _launch_config(self,purpose=None):
        a=activation_launch_config(private_json(self.activation))
        if purpose is not None:
            if purpose not in PURPOSES:raise ValueError('navigation_purpose_invalid')
            if purpose=='execution' and a.get('purpose')!='execution':
                raise ValueError('navigation_execution_not_configured')
            a={**a,'purpose':purpose}
        return a

    def _requested_purpose(self,purpose):
        if purpose is not None and purpose not in PURPOSES:
            raise HTTPException(422,'无效导航模式')
        try:return self._launch_config(purpose).get('purpose')
        except (OSError,ValueError,KeyError,TypeError):
            if purpose is not None:raise HTTPException(409,'当前发布版本未配置该导航模式') from None
            return None

    def _activation(self,operation=None):
        a=self._launch_config(operation.get('requested_purpose') if operation else None)
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
        if operation and self.helper_runner is None and self.process_cleanup is None:
            files=json.loads(regular_text(seal,33554432)).get('files',{})
            helpers=[name for name in files if name.endswith('/site-packages/d1max_pct_scan/live_runtime.py')]
            if len(helpers)!=1:raise ValueError('sealed_web_helper_cleanup_missing')
            operation.update(cleanup_path=helpers[0],cleanup_sha256=files[helpers[0]])
        verify_release_files(self.nav,release,seal,expected_manifest_sha256=a['release_manifest_sha256'],
            runner=(lambda command,**kwargs:self._run_helper(operation,command,**kwargs)) if operation else None,
            check_cancelled=(lambda:self._check_operation(operation)) if operation else None,require_admitted_tools=True)
        if a['purpose']=='execution':
            record=Path(a['motion_profile_record'])
            if not record.is_absolute() or digest(record)!=a['motion_profile_sha256']:
                raise ValueError('motion_profile_missing_or_changed')
            if a.get('runtime_motion_profile'):
                files=json.loads(regular_text(seal,33554432)).get('files',{})
                profile=json.loads(regular_text(record))
                if (not record.resolve(strict=True).is_relative_to(release)
                        or files.get(str(record.resolve()))!=a['motion_profile_sha256']
                        or profile.get('record_kind')!='runtime_motion_profile'
                        or profile.get('physical_acceptance_verified') is not False):
                    raise ValueError('sealed_runtime_motion_profile_required')
        policy=a.get('sdk_session_policy','fixed')
        if policy not in ('fixed','bind_current_on_start'):
            raise ValueError('sdk_session_policy_invalid')
        if policy=='bind_current_on_start':
            if a['purpose']!='planning_only' and not a.get('runtime_motion_profile'):
                raise ValueError('execution_requires_bound_motion_profile')
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

    def _adopt_owned_core(self,state,unit):
        if state and not state.get('pid') and not state.get('invocation_id'):
            self._bound_session(state)
            if (unit.get('Description')==OWNER+state['id'] and int(unit.get('MainPID') or 0)>0 and unit.get('InvocationID')):
                return {**state,'pid':int(unit['MainPID']),'invocation_id':unit['InvocationID']}
        return state

    def _admitted_session(self,state):
        """Read the immutable admission, independently of the next launch draft."""
        directory=Path(state['directory'])
        if (not directory.is_absolute() or directory.is_symlink()
                or directory.parent!=self.root/'sessions'
                or digest(directory/'session.json')!=state['session_sha256']):
            raise ValueError('core_session_binding_changed')
        session=private_json(directory/'session.json')
        if (session.get('id')!=state.get('id') or session.get('purpose') not in PURPOSES
                or session.get('pipeline_contract')!='single_floor_v3'
                or session.get('transport_mode')!='live'
                or session.get('motion_control_enabled') is not (session.get('purpose')=='execution')):
            raise ValueError('core_session_binding_changed')
        return directory,session

    def _bound_session(self,state):
        """Active ownership must also match the admitted purpose exactly."""
        directory,session=self._admitted_session(state)
        if session['purpose']!=state.get('purpose'):
            raise ValueError('core_session_binding_changed')
        return directory,session

    def _startup_evidence(self,state):
        try:
            directory,_=self._bound_session(state)
            value=private_json(directory/'core_startup.json',16384)
            phase=value.get('phase');stamp=value.get('recorded_at_unix')
            requested=state.get('launch_requested_at_unix')
            if (type(value.get('schema')) is not int or value['schema']!=1
                    or value.get('session_id')!=state['id']
                    or phase not in ('starting','spawning','started','failed')
                    or type(value.get('children_started')) is not bool
                    or not isinstance(value.get('error'),str) or len(value['error'])>4096
                    or type(stamp) not in (int,float) or not math.isfinite(stamp)
                    or stamp>time.time()+.1
                    or (requested is not None and (type(requested) not in (int,float)
                        or not math.isfinite(requested) or stamp<requested-.1))
                    or (phase=='starting' and value['children_started'])
                    or (phase in ('spawning','started') and not value['children_started'])):
                return None
            return value
        except (OSError,ValueError,KeyError,TypeError):
            return None

    def _all_groups_empty(self,unit):
        if (unit.get('ActiveState') not in ('inactive','failed')
                or int(unit.get('MainPID') or 0)!=0 or self.system.populated(unit)):
            return False
        for name in (VIEW_UNIT,*LEGACY_VIEW_UNITS.values()):
            view=self.system.show(name)
            if (view.get('ActiveState') not in ('inactive','failed')
                    or int(view.get('MainPID') or 0)!=0 or self.system.populated(view)):
                return False
        return True

    def _views_busy(self):
        return any(view.get('ActiveState') in ACTIVE or int(view.get('MainPID') or 0)>0
                   or self.system.populated(view) for view in
                   (self.system.show(name) for name in (VIEW_UNIT,*LEGACY_VIEW_UNITS.values())))

    @staticmethod
    def _startup_error(value):
        # Return a stable diagnostic, not raw launcher stderr or credentials.
        error=value.get('error','') if value else ''
        error=re.sub(r'^[A-Za-z_][A-Za-z0-9_]*Error:\s*','',error)
        match=re.match(r'[A-Za-z_][A-Za-z0-9_.-]{0,95}',error)
        return match.group(0) if match else 'core_startup_failed'

    def _inactive_reconciliation(self,state,unit):
        """Process cleanup is not a claim of physical stopping.

        Explicitly unstarted graphs have no task or motion work to retire.
        Partial planning-only startup can also recover from actual
        process cleanup, without changing the owner's drain/physical facts.
        An execution-session shutdown may also prove process cleanup without
        an owner drain ACK or a preceding Web stop request. The task and
        physical-stop facts remain unchanged; a new graph never restores that
        session's authorization. An inactive, proven-empty graph must not leave
        both Web start and exit disabled merely because its owner drain timed out.
        Read-only overview uses this result without writing or sending actions.
        """
        if not state or not self._all_groups_empty(unit):return None
        directory,session=self._admitted_session(state)
        # Older Web code copied the *next* purpose into the retired state's
        # cache. Only an unchanged private admission and independently proven
        # inactive/empty groups may repair that redundant cached field. This
        # local copy is returned only after the existing exit proof succeeds;
        # it cannot adopt a live process or restore execution authorization.
        corrected={**state,'purpose':session['purpose']}
        reconciled=self._inactive_session_exit(corrected,directory,session)
        if reconciled is None and state.get('purpose')!=session['purpose']:
            raise ValueError('retired_session_purpose_recovery_unconfirmed')
        return reconciled

    def _inactive_session_exit(self,state,directory,session):
        """Return a retired state only from the admitted session's exit proof."""
        startup=self._startup_evidence(state)
        # A graph capability is not a task permit. The private, session-bound
        # failed ACK proves no child was spawned; the caller has independently
        # verified all core/view groups empty and the unchanged admission.
        if (state.get('purpose') in PURPOSES and session.get('purpose')==state['purpose']
                and session.get('motion_control_enabled') is (state['purpose']=='execution')
                and startup and startup['phase']=='failed' and startup['children_started'] is False):
            return {**state,'phase':'stopped','quarantined':False,'views':{},'view_error':'',
                    'startup_error':self._startup_error(startup),'startup_report':startup,
                    'stop_report':{'software_started':False,'software_retired':False,
                        'cleanup_confirmed':True,'physical_stop_confirmed':False,
                        'reason':'startup_failed_before_graph'}}
        # A collected unit has no remaining InvocationID.  Match its immutable
        # session instead of treating a generic inactive unit as exit success.
        try:drain=private_json(directory/'shutdown.json',16384)
        except (OSError,ValueError):return None
        if drain.get('session_id')!=state['id']:
            return None
        stamp=drain.get('recorded_at_unix');requested=state.get('launch_requested_at_unix')
        valid_stamp=type(stamp) in (int,float) and math.isfinite(stamp) and stamp<=time.time()+.1
        valid_baseline=type(requested) in (int,float) and math.isfinite(requested)
        fresh=valid_stamp and valid_baseline and stamp>=requested-.1
        if stamp is not None and (not valid_stamp or requested is not None and not fresh):return None
        cleanup=drain.get('process_cleanup')
        stop_requested=state.get('stop_requested_at_unix')
        explicit_stop=(type(stop_requested) in (int,float) and math.isfinite(stop_requested)
            and valid_stamp and stamp>=stop_requested-.1
            and (state.get('phase')=='stopping' or state.get('worker_operation')=='stop'))
        # Repeated exit/old Web close can arrive AFTER this same session has
        # already written its complete cleanup proof. That newer request does
        # not invalidate software cleanup; it supplies no new physical-stop
        # evidence. Admission identity + launch freshness + empty real groups
        # are still mandatory above and below.
        execution_stop_requested=(type(stop_requested) in (int,float)
            and math.isfinite(stop_requested) and valid_baseline
            and requested-.1<=stop_requested<=time.time()+.1)
        preview_exit=startup and (startup['phase']=='failed' and startup['children_started'] is True
            or explicit_stop and startup['phase'] in ('starting','spawning','started','failed'))
        # Retiring an old admission must not erase a later, failed NEW start.
        # Only an exit error which this cleanup actually resolves is cleared.
        resolved_error=({'worker_error':'','worker_operation':'','worker_status_code':None}
            if state.get('worker_operation')=='stop' else {})
        preview_cleanup=(state.get('purpose')=='planning_only'
            and session.get('motion_control_enabled') is False and preview_exit)
        execution_cleanup=(state.get('purpose')=='execution'
            and session.get('motion_control_enabled') is True
            and startup and startup['phase'] in ('starting','spawning','started','failed')
            and drain.get('request_accepted') is True)
        execution_cleanup_reason=('execution_owned_stop_cleanup' if execution_stop_requested else
            'execution_owned_process_cleanup')
        if ((preview_cleanup or execution_cleanup)
                and fresh and type(drain.get('navigator_started')) is bool
                and all(type(drain.get(key)) is bool for key in
                        ('request_accepted','software_retired','physical_stop_confirmed'))
                and isinstance(cleanup,dict)
                and cleanup.get('all_direct_children_exited') is True
                and cleanup.get('all_owned_groups_exited') is True):
            return {**state,'phase':'stopped','quarantined':False,'views':{},'view_error':'',
                    **resolved_error,
                    'startup_error':self._startup_error(startup) if startup['phase']=='failed' else state.get('startup_error',''),
                    'startup_report':startup,
                    'stop_report':{'software_started':drain['navigator_started'],
                        'request_accepted':drain['request_accepted'],
                        'software_retired':drain['software_retired'],
                        'cleanup_confirmed':True,'process_cleanup':cleanup,
                        'physical_stop_confirmed':drain['physical_stop_confirmed'],
                        'task_retirement_reason':drain.get('reason',''),
                        'cleanup_reason':execution_cleanup_reason if execution_cleanup else 'owned_process_cleanup',
                        'reason':(drain.get('reason',execution_cleanup_reason) if execution_cleanup else
                            'planning_only_owned_stop_cleanup' if explicit_stop else 'startup_failed_after_partial_spawn')}}
        if drain.get('request_accepted') is not True or drain.get('software_retired') is not True:
            return None
        return {**state,'phase':'stopped','quarantined':False,'views':{},'view_error':'',
                **resolved_error,
                'stop_report':{'software_started':drain.get('navigator_started',True) is True,'software_retired':True,
                    'cleanup_confirmed':True,'physical_stop_confirmed':drain.get('physical_stop_confirmed') is True}}

    def _wait_for_startup(self,state,operation=None):
        _,session=self._bound_session(state)
        budget=session.get('runtime_policy',{}).get('worker_startup_timeout_s')
        if type(budget) not in (int,float) or not math.isfinite(budget) or budget<=0:
            raise ValueError('session_startup_policy_missing')
        deadline=time.monotonic()+budget
        while True:
            if operation:self._check_operation(operation)
            evidence=self._startup_evidence(state)
            unit=self._unit()
            if evidence and evidence['phase']=='failed':
                # The final writer may precede systemd's empty-group update.
                # Never mark an active/unknown group retired from JSON alone.
                reconciled=self._inactive_reconciliation(state,unit)
                if reconciled:return evidence,unit,reconciled
                if unit['ActiveState'] not in ACTIVE:return evidence,unit,None
            if evidence and evidence['phase']=='started':
                if (not state.get('pid') and not state.get('invocation_id')
                        and unit.get('Description')==OWNER+state['id']
                        and int(unit.get('MainPID') or 0)>0 and unit.get('InvocationID')):
                    state.update(pid=int(unit['MainPID']),invocation_id=unit['InvocationID'])
                if (unit['ActiveState']=='active' and self._owned(state,unit)
                        and int(unit.get('MainPID') or 0)>0 and unit.get('InvocationID')):
                    return evidence,unit,None
                raise ValueError('core_startup_identity_unconfirmed')
            if unit['ActiveState'] in ('inactive','failed') and not self.system.populated(unit):
                raise ValueError('core_startup_exited_without_ack')
            if time.monotonic()>=deadline:raise ValueError('core_startup_timeout')
            time.sleep(min(self.STARTUP_ACK_POLL_SECONDS,max(0.,deadline-time.monotonic())))

    @staticmethod
    def _session_scope(session,release):
        import yaml
        route_path=Path(session['crossfloor_route_config']).resolve(strict=True)
        if not route_path.is_relative_to(release):return None
        route=yaml.safe_load(regular_text(route_path))
        capabilities=session.get('map_capabilities')
        if capabilities is not None:
            floors=capabilities['floor_ids']
            stairs=capabilities['stairs_planning_enabled']
        else:
            floors=route.get('floor_ids') or [route['floor_id']]
            stairs=route.get('stairs_enabled') is True
        return dict(floors=[str(f) for f in floors],stairs_enabled=stairs)

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
            session_path=Path(json.loads(regular_text(seal,33554432))['preflight_session']).resolve(strict=True)
            if not session_path.is_relative_to(release):return None
            session=json.loads(regular_text(session_path))
            return self._session_scope(session,release)
        except (OSError,ValueError,KeyError,TypeError,AttributeError,ImportError):
            return None

    def _connection(self):
        return self.monitor_probe.snapshot()

    @staticmethod
    def _data_blocker(monitor,expected_session=None):
        health=monitor.get('health',{})
        if monitor.get('phase')=='checking':return '正在更新机器狗连接状态'
        if monitor.get('active') is not True:return '请先连接机器狗'
        if not isinstance(health,dict):return '等待机器狗状态数据'
        if health.get('replay') is not False:return '需要实机数据，当前为回放或状态未确认'
        if health.get('sdk_fresh') is not True:return '等待 SDK 状态数据'
        if health.get('lidar_fresh') is not True:return '等待前后雷达数据'
        stamp=health.get('sdk_session_source_stamp')
        if not (isinstance(health.get('sdk_session'),str)
                and re.fullmatch(r'[A-Za-z0-9_.-]{1,128}',health['sdk_session'])):
            return '等待 SDK 会话身份'
        if expected_session is not None and health['sdk_session']!=expected_session:
            return 'SDK 会话已变化，请核对启动绑定'
        if not (type(stamp) in (int,float) and 0<=time.time()-stamp<=1.):
            return '等待新的 SDK 会话状态'
        return ''

    @staticmethod
    def _data_ready(monitor,expected_session=None):
        return not SingleFloorRuntime._data_blocker(monitor,expected_session)

    @staticmethod
    def _writer_blocker(health,purpose):
        if purpose=='planning_only':
            # The mainline task writer forcibly disables automatic takeover in
            # the SDK constructor. It is safe to reuse this *disarmed capability*
            # without reconnecting: preview never grants a task or emits Move.
            # A known older per-curve writer is not the same connection contract.
            if isinstance(health,dict) and (health.get('execution_v3_enabled') is True
                    or health.get('execution_protocol')=='curve_cas_v3'):
                return '当前 SDK 使用旧运动通道，请先停止数据连接，再连接当前主线'
            return ''
        if purpose!='execution':return ''
        if (not isinstance(health,dict) or health.get('execution_protocol')!=TASK_WRITER_PROTOCOL
                or health.get('execution_task_writer_enabled') is not True):
            return '当前 SDK 使用旧控制协议，请先停止数据连接，再重新连接加载当前导航 SDK'
        if health.get('execution_record_valid') is not True:return 'SDK 运动配置尚未加载'
        return ''

    def snapshot(self,*,connection=False):
        try:
            with self.lock:
                state=self._state();operation=self.operation
            unit=self._unit()
            core_busy=unit['ActiveState'] in ACTIVE or self.system.populated(unit)
            views_busy=self._views_busy()
            busy=core_busy or views_busy or operation is not None or self.helper is not None
            if state and not busy and state.get('session_sha256'):
                reconciled=self._inactive_reconciliation(state,unit)
                if reconciled:state=reconciled
            if core_busy:state=self._adopt_owned_core(state,unit)
            owned=bool(state) and self._owned(state,unit)
            startup=self._startup_evidence(state) if state.get('session_sha256') else None
            phase=(('running' if startup and startup['phase']=='started' else 'starting')
                   if unit['ActiveState']=='active' and state.get('phase')!='stopping' else 'stopping') \
                  if core_busy and owned else 'conflict' if core_busy else state.get('phase','stopped')
            if operation is not None:phase=state.get('phase','preparing')
            elif core_busy and owned and state.get('quarantined'):phase='needs_review'
            elif views_busy and not core_busy:phase='needs_review' if state else 'conflict'
            if not busy and phase in ('running','starting','stopping'):phase='failed'
            # The running process keeps its admitted purpose even if an admin
            # stages a different activation for the next session. Display-only
            # capability is not execution confirmation or a motion permit.
            try: activation=activation_launch_config(private_json(self.activation))
            except (OSError,ValueError,KeyError,TypeError):activation={}
            configured_purpose=activation.get('purpose')
            if configured_purpose not in PURPOSES:configured_purpose=None
            purpose=(operation.get('requested_purpose') or state.get('purpose')) if operation is not None else \
                state.get('purpose') if owned and (busy or phase in ('failed','needs_review')) else configured_purpose
            if purpose not in PURPOSES:purpose=None
            release=self.release_probe.snapshot()
            configured=release.get('readiness')=='ready' and configured_purpose is not None
            monitor=self._connection() if connection else {'active':False,'phase':'unchecked','health':{}}
            running_binding=owned and bool(state.get('session_sha256')) and (busy or phase in ('failed','needs_review'))
            admission_valid=False
            try:
                if running_binding:
                    # The next launch configuration must not rebind the SDK
                    # identity or capability of this already admitted core.
                    _,admitted=self._bound_session(state)
                    admission_valid=True
                    expected_session=admitted.get('expected_sdk_session');dynamic=False
                else:
                    dynamic=activation.get('sdk_session_policy')=='bind_current_on_start'
                    expected_session=None if dynamic else activation.get('expected_sdk_session')
            except (OSError,ValueError,KeyError,TypeError):expected_session=None;dynamic=False
            data_blocker=self._data_blocker(monitor,expected_session) if expected_session or dynamic else 'SDK 启动绑定未配置'
            if not data_blocker:data_blocker=self._writer_blocker(monitor.get('health',{}),purpose)
            data_ready=not data_blocker
            blocker=(release.get('reason') if not configured else
                     data_blocker if not data_ready else
                     '当前服务需要先停止或核对退出结果' if busy or phase!='stopped' or state.get('quarantined') else '')
            scope=state.get('scope') if owned and busy else self._scope()
            running_id=Path(state['release_root']).name if owned and core_busy and state.get('release_root') else None
            running_manifest=state.get('release_manifest_sha256') if owned and core_busy else None
            configured_manifest=release.get('manifest_sha256')
            # The selected directory can be replaced while an older process
            # remains alive. This is admission identity, not a claim that an
            # already-running process has hot-loaded the configured bytes.
            running_matches=(running_id==release.get('configured_id') and running_manifest==configured_manifest
                if running_id and running_manifest and configured_manifest else None)
            release={**release,'scope':scope,'running_id':running_id,
                     'running_manifest_sha256':running_manifest,
                     'running_matches_configured':running_matches}
            selectable=not busy and phase=='stopped' and not state.get('quarantined') and not self.closed
            connectable=selectable and monitor.get('active') is False and monitor.get('phase') not in \
                {'checking','connecting','disconnecting','starting','stopping','unchecked','unavailable'} \
                and monitor.get('health',{}).get('replay') is not True
            capabilities={}
            for choice in ('planning_only','execution'):
                available=configured and (choice=='planning_only' or configured_purpose=='execution')
                choice_blocker=self._data_blocker(monitor,expected_session) if expected_session or dynamic else 'SDK 启动绑定未配置'
                if not choice_blocker:choice_blocker=self._writer_blocker(monitor.get('health',{}),choice)
                capabilities[choice]=dict(can_start=available and selectable and not choice_blocker,
                    can_connect=connectable and (choice=='planning_only' or configured_purpose=='execution'),
                    start_blocker=(release.get('reason') if not configured else
                        '当前发布版本未配置运动执行' if not available else choice_blocker
                        or ('当前服务需要先停止或核对退出结果' if not selectable else '')))
            if any(c['can_start'] for c in capabilities.values()):blocker=''
            return dict(profile='single_floor_live',phase=phase,busy=busy,session_id=state.get('id'),
                pid=int(unit.get('MainPID') or 0) if owned else None,exit_code=unit.get('ExecMainStatus') if owned else state.get('exit_code'),
                release_configured=configured,owned=owned,
                quarantined=state.get('quarantined',False) or phase=='failed',
                navigation_controls='rviz',sdk_auto_connect=False,scope=scope,purpose=purpose,
                motion_capable=purpose=='execution',execution_available=phase=='running' and running_binding and purpose=='execution' and data_ready
                    and monitor.get('health',{}).get('execution_protocol')==TASK_WRITER_PROTOCOL
                    and monitor.get('health',{}).get('execution_task_writer_enabled') is True
                    and monitor.get('health',{}).get('execution_record_valid') is True,
                mainline={**MAINLINE,'entrypoint':str(self.nav/'tools/navigation_entry.sh'),
                    'runtime_entrypoint':state.get('entrypoint',str(self.nav/'tools/single_floor_entry.sh')) if owned and busy else None,
                    'runtime_entrypoint_sha256':state.get('entrypoint_sha256') if owned and busy else None},
                release=release,connection=monitor,start_blocker=blocker or '',
                view_error=state.get('view_error','') if owned and busy else '',
                worker_error=state.get('worker_error',''),worker_operation=state.get('worker_operation',''),
                worker_status_code=state.get('worker_status_code'),
                operation_id=state.get('operation_id'),generation=state.get('generation'),
                startup_error=state.get('startup_error',''),
                startup_report=startup or state.get('startup_report'),
                stop_report=state.get('stop_report'),
                can_select_purpose=selectable,purpose_capabilities=capabilities,
                connection_requested_purpose=state.get('connection_purpose'),
                can_start=any(c['can_start'] for c in capabilities.values()),
                can_stop=bool(operation is not None and operation['kind'] in ('start','connect','view') and not operation['cancel'].is_set()
                    or operation is None and state and (owned and core_busy or views_busy or self.helper is not None)),
                # This is permission to open a display, not proof that RViz or
                # the functional graph is ready. A start already owns one
                # automatic open; clicks during that start coalesce with it.
                can_view=owned and unit['ActiveState']=='active' and int(unit.get('MainPID') or 0)>0
                    and bool(unit.get('InvocationID')) and admission_valid
                    and phase in ('starting','running') and not state.get('quarantined')
                    and not (startup and startup['phase']=='failed')
                    and (operation is None or operation['kind']=='start' and not operation['cancel'].is_set()),
                can_connect=any(c['can_connect'] for c in capabilities.values()),
                snapshot_at_unix=time.time())
        except (OSError,ValueError,KeyError,TypeError,RuntimeError,subprocess.SubprocessError):
            return dict(profile='single_floor_live',phase='conflict',busy=True,error='服务所有权或退出状态待核对',
                purpose=None,scope=None,motion_capable=False,execution_available=False,owned=False,
                mainline={**MAINLINE,'entrypoint':str(self.nav/'tools/navigation_entry.sh')},
                release={},can_start=False,can_stop=False,can_view=False,can_connect=False,
                can_select_purpose=False,purpose_capabilities={p:dict(can_start=False,can_connect=False) for p in PURPOSES},
                worker_error=state.get('worker_error','') if 'state' in locals() else '',
                worker_operation=state.get('worker_operation','') if 'state' in locals() else '',
                worker_status_code=state.get('worker_status_code') if 'state' in locals() else None,
                start_blocker='服务所有权或退出状态待核对',snapshot_at_unix=time.time())

    def connect(self,purpose=None):
        # Explicit operator action, never part of start() or a status read.
        with self._admission():
            if self.closed or self.operation is not None:
                raise HTTPException(409,'导航服务正在切换或 Web 正在退出')
        current=self.snapshot()
        with self._admission():
            if self.closed or self.operation is not None:
                raise HTTPException(409,'导航服务正在切换或 Web 正在退出')
            if current['busy'] or current.get('quarantined') or current['phase']!='stopped':
                raise HTTPException(409,'导航运行中或退出待核对，不重连数据会话')
            requested=self._requested_purpose(purpose) or 'planning_only'
            self._accept('connect',self._state(),requested_purpose=requested)
        return self.snapshot(connection=True)

    def _connect_owned(self,lifecycle):
        # One existing lifecycle worker serializes connect/start/stop. HTTP and
        # the cross-workflow lease do not hold the state lock: overview, cancel
        # and Web close remain available while the manager is slow. This uses
        # the existing manager API, including its instance-bound start request;
        # it does not create a navigation task or acquire robot control.
        with self._startup_lease():
            self._check_operation(lifecycle)
            unit=self._unit()
            if unit['ActiveState'] in ACTIVE or self.system.populated(unit) or self._views_busy():
                raise HTTPException(409,'已有导航进程，不重连数据会话')
            for name in CONFLICTS:
                unit=self.system.show(name)
                if unit.get('ActiveState') in ACTIVE or self.system.populated(unit):
                    raise HTTPException(409,'请先停止已有定位或规划服务')
                self._check_operation(lifecycle)
            status=self.localization.manager()
            self._check_operation(lifecycle)
            monitor=status['monitor']
            if not isinstance(monitor,dict):raise HTTPException(503,'连接状态待核对')
            if monitor.get('active') is True:
                protocol_blocker=self._writer_blocker(monitor.get('health',{}),lifecycle['requested_purpose'])
                if protocol_blocker:raise HTTPException(409,protocol_blocker)
                raise HTTPException(409,'已有机器狗数据会话，不重复连接')
            health=monitor.get('health',{})
            if monitor.get('active') is not False or not isinstance(health,dict):
                raise HTTPException(503,'连接状态待核对')
            if (monitor.get('phase') in {'checking','connecting','disconnecting','starting','stopping'}
                    or health.get('replay') is True):
                raise HTTPException(409,'连接切换中或正在回放，不重连')
            instance=status.get('instance')
            if not isinstance(instance,str) or not instance:raise HTTPException(503,'连接管理器身份待核对')
            self._check_operation(lifecycle)
            connected=self.localization.manager('/v1/monitor/start',
                {'request_id':lifecycle['id'],'instance':instance})['monitor']
            self._check_operation(lifecycle)
            if not isinstance(connected,dict):raise HTTPException(503,'连接结果待核对')
            with self.lock:
                # Cancellation of an in-flight manager request never means
                # disconnecting the SDK. Its late response cannot publish new
                # lifecycle/cache state or resurrect the cancelled operation.
                self._check_operation(lifecycle)
                self.monitor_probe.update(connected)
                self._commit(lifecycle,{**lifecycle['prior'],'phase':'stopped','quarantined':False,
                    'connection_purpose':lifecycle['requested_purpose'],
                    'worker_error':'','worker_operation':'','worker_status_code':None})

    def _open_view(self,state,layout,*,lifecycle=None):
        if layout not in ('global','local'):raise HTTPException(422,'无效 RViz 布局')
        if lifecycle:self._check_operation(lifecycle)
        def remember(record):
            # A late viewer identity is cleanup authority, not startup success.
            with self.lock:
                if lifecycle and self.operation is not lifecycle:raise LifecycleCancelled()
                current=self._state() if lifecycle else state
                current.setdefault('views',{})['single']=record
                if lifecycle:self._commit(lifecycle,current,cleanup=True)
                else:persist(self.state_path,current)
        try:
            if isinstance(self.views,OwnedNavigationViews):record=self.views.open(state,layout,on_record=remember)
            else:record=self.views.open(state,layout)
            if lifecycle:self._check_operation(lifecycle)
            state.setdefault('views',{})['single']=record;state['view_error']=''
            if lifecycle:self._commit(lifecycle,state)
            else:persist(self.state_path,state)
        except (OSError,ValueError,KeyError,TypeError,RuntimeError,subprocess.SubprocessError) as error:
            state['view_error']='RViz 未打开（'+self._startup_error({'error':str(error)})+'）'
            if lifecycle:self._commit(lifecycle,state)
            else:persist(self.state_path,state)
            raise HTTPException(409,state['view_error']) from None

    def open_view(self,layout):
        if layout not in ('global','local'):raise HTTPException(422,'无效 RViz 布局')
        with self._admission():
            generation=self.generation;operation=self.operation;state=self._state()
            if (self.closed or operation is not None and
                    (operation['kind']!='start' or operation['cancel'].is_set())):
                raise HTTPException(409,'导航服务正在切换')
        # systemd/session reads and all display work stay outside the state
        # lock, so overview and an explicit stop remain immediately available.
        unit=self._unit()
        if (unit.get('ActiveState')!='active' or not int(unit.get('MainPID') or 0) or not unit.get('InvocationID')
                or not self._owned(state,unit)
                or state.get('phase') not in ('starting','running') or state.get('quarantined')):
            raise HTTPException(409,'请先启动本次主线导航服务')
        try: self._bound_session(state)
        except (OSError,ValueError,KeyError,TypeError):
            raise HTTPException(409,'本次导航会话绑定变化，不能打开视图') from None
        startup=self._startup_evidence(state)
        if startup and startup['phase']=='failed':raise HTTPException(409,'本次导航运行图启动失败')
        with self._admission():
            if self.closed or self.generation!=generation or self.operation is not operation:
                raise HTTPException(409,'导航服务正在切换')
            if operation is None:
                self._accept('view',state,view_layout=layout)
            # During start the sealed, active core already has one automatic
            # global open queued on this same worker. Never launch a competitor.
            elif layout!='global':
                raise HTTPException(409,'启动期间请等待同一 RViz 窗口打开后再切换布局')
        return self.snapshot(connection=True)

    def _start_owned(self,lifecycle):
        with self._startup_lease():
            self._check_operation(lifecycle)
            prior=lifecycle['prior']
            if prior.get('session_sha256'):
                try:reconciled=self._inactive_reconciliation(prior,self._unit())
                except (OSError,ValueError,KeyError,TypeError,RuntimeError,subprocess.SubprocessError):
                    raise HTTPException(409,'上次导航会话绑定或退出状态待核对，不重复启动') from None
                if reconciled:prior=reconciled
            unit=self._unit()
            if unit['ActiveState'] in ACTIVE or self.system.populated(unit) or self._views_busy() or prior.get('quarantined'):
                raise HTTPException(409,'服务未清理或停止结果待核对')
            try:a=self._activation(lifecycle)
            except (OSError,ValueError,KeyError,TypeError):
                raise HTTPException(409,'导航启动配置未通过校验') from None
            self._check_operation(lifecycle)
            web=self.system.show('d1max-web-managed.service')
            if web.get('ActiveState')!='active' or int(web.get('MainPID') or 0)!=os.getpid():
                raise HTTPException(409,'请通过受托管 Web 启动导航服务')
            for name in CONFLICTS:
                state=self.system.show(name)
                if state.get('ActiveState') in ACTIVE or self.system.populated(state):raise HTTPException(409,'请先停止已有定位或规划服务')
            # Read-only existing monitor check. No start/connect/reconnect API.
            monitor=self.localization.manager()['monitor'];health=monitor.get('health',{})
            dynamic=a.get('sdk_session_policy')=='bind_current_on_start'
            if not self._data_ready(monitor,None if dynamic else a['expected_sdk_session']):
                raise HTTPException(409,'现有实机数据会话未确认或已过期；不会连接或重连 SDK')
            if dynamic:
                a={**a,'expected_sdk_session':health['sdk_session']}
            protocol_blocker=self._writer_blocker(health,a['purpose'])
            if protocol_blocker:raise HTTPException(409,protocol_blocker)
            if (a['purpose']=='execution' and a.get('runtime_motion_profile') and
                    health.get('execution_profile_sha256')!=a['motion_profile_sha256']):
                raise HTTPException(409,'SDK 与导航运动配置不一致')
            identifier=uuid.uuid4().hex;directory=self.root/'sessions'/identifier
            directory.parent.mkdir(parents=True,exist_ok=True)
            entry=a['entrypoint']
            # The entry is release-agnostic; the activated release is named explicitly.
            release=str(Path(a['release_root']).resolve(strict=True))
            env={**os.environ,'D1MAX_RELEASE':release,'D1MAX_NAV_ROOT':str(self.nav),'D1MAX_NAV_TRANSPORT':'live'}
            operation='prepare';launched=False;display_attempted=False
            try:
                command=[entry,'prepare','--session',str(directory),'--transport-mode','live',
                    '--purpose',a['purpose'],'--expected-sdk-session',a['expected_sdk_session'],'--seal']
                if a['purpose']=='execution':
                    command+=['--motion-profile' if a.get('runtime_motion_profile') else '--acceptance-record',a['motion_profile_record']]
                self._run_helper(lifecycle,command,check=True,capture_output=True,text=True,timeout=70,env=env)
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
                    view_contract=session.get('view_contract'),launch_requested_at_unix=time.time())
                try:state['scope']=self._session_scope(session,Path(release))
                except (OSError,ValueError,KeyError,TypeError,AttributeError,ImportError):state['scope']=None
                state=self._commit(lifecycle,state)
                lifecycle['admitted']=True
                operation='launch'
                self._check_operation(lifecycle)
                launched=True;lifecycle['launched']=True
                self.runner(['systemd-run','--user','--collect','--unit='+UNIT,
                    '--property=Description='+OWNER+session['id'],'--property=Type=exec',
                    '--property=KillMode=mixed','--property=KillSignal=SIGTERM','--property=TimeoutStopSec=30',
                    # An existing SDK session is a prerequisite, never a start
                    # dependency: it can disappear during prepare/seal. Explicit
                    # monitor stops drain the core first via reverse ordering;
                    # monitor restarts must not resurrect this bound session.
                    '--property=BindsTo=d1max-web-managed.service',
                    '--property=Requisite=d1max-monitor-managed.service',
                    '--property=StopPropagatedFrom=d1max-monitor-managed.service',
                    '--property=PartOf=d1max-web-managed.service',
                    '--property=After=d1max-web-managed.service d1max-monitor-managed.service',
                    '--setenv=D1MAX_RELEASE='+release,
                    '--setenv=D1MAX_NAV_ROOT='+str(self.nav),
                    '--setenv=D1MAX_NAV_TRANSPORT=live',
                    '--property=StandardOutput=append:'+str(directory/'service.log'),
                    '--property=StandardError=append:'+str(directory/'service.log'),
                    entry,'run','--session',str(directory)],check=True,capture_output=True,text=True,timeout=10)
                unit=self._unit()
                if (unit.get('Description')==OWNER+session['id'] and int(unit.get('MainPID') or 0)>0
                        and unit.get('InvocationID')):
                    state.update(pid=int(unit['MainPID']),invocation_id=unit['InvocationID'])
                    state=self._commit(lifecycle,state)
                # The sealed session and exact active core are enough to
                # start its display. Core verification/spawn continues in its
                # own process; a display is never functional readiness or a
                # motion grant. This same worker retains late viewer records
                # before honouring cancellation and cleaning up.
                if unit.get('ActiveState')=='active' and self._owned(state,unit):
                    self._check_operation(lifecycle)
                    display_attempted=True
                    try:self._open_view(state,'global',lifecycle=lifecycle)
                    except HTTPException:pass  # Display failure is not a core failure.
                    self._check_operation(lifecycle)
                operation='startup'
                startup,unit,reconciled=self._wait_for_startup(state,lifecycle)
                state['startup_report']=startup
                if startup['phase']=='failed':
                    state=reconciled or {**state,'phase':'needs_review','quarantined':True,
                        'startup_error':self._startup_error(startup)}
                    state=self._commit(lifecycle,state)
                    raise HTTPException(409,'导航运行图启动失败（'+self._startup_error(startup)+'）'
                        +('；未启动运行组件，可修复后重新启动' if reconciled and not startup['children_started']
                          else '；本次进程已清理，可修复后重新启动' if reconciled else '；退出与退役待核对'))
                state['phase']='running';state=self._commit(lifecycle,state)
                if not display_attempted:
                    # Type=exec may still show activating on the first query.
                    # Once the exact core has ACKed startup, make its one
                    # automatic display attempt; never retry an earlier failure.
                    self._check_operation(lifecycle)
                    display_attempted=True
                    try:self._open_view(state,'global',lifecycle=lifecycle)
                    except HTTPException:pass
            except (OSError,ValueError,KeyError,TypeError,subprocess.SubprocessError) as error:
                # Never kill an unverified unit or retry an uncertain launch.
                if launched:
                    state.update(phase='needs_review',quarantined=True)
                    state=self._commit(lifecycle,state)
                    if operation=='startup':
                        raise HTTPException(503,'导航运行图启动未确认（'+self._startup_error({'error':str(error)})
                            +'）；请核对本次会话，不重复启动') from None
                    raise HTTPException(503,'启动结果待确认，请检查本次会话；禁止重复启动') from None
                message='主线会话准备失败，未启动导航进程' if operation=='prepare' else '主线会话校验失败，未启动导航进程'
                raise HTTPException(409,message) from None
            self._check_operation(lifecycle)

    def _stop_owned(self,lifecycle):
        # A single lifecycle worker owns both start and retirement; HTTP merely
        # changes cancellation intent under the short state lock.
        with self._stop_scope(lifecycle):
            state=self._state();unit=self._unit()
            busy=unit['ActiveState'] in ACTIVE or self.system.populated(unit)
            if not busy:
                if lifecycle.get('local_prepare_cleanup') and not state.get('session_sha256') and self._all_groups_empty(unit):
                    self._commit(lifecycle,{**state,'phase':'stopped','quarantined':False,
                        'stop_report':{'software_started':False,'software_retired':False,'physical_stop_confirmed':False,
                            'cleanup_confirmed':True,'process_cleanup':lifecycle['local_prepare_cleanup'],
                            'reason':'owned_prepare_cleanup'}},cleanup=True)
                    return
                try:reconciled=self._inactive_reconciliation(state,unit) if state.get('session_sha256') else None
                except (OSError,ValueError,KeyError,TypeError,RuntimeError,subprocess.SubprocessError):
                    raise HTTPException(409,'会话绑定或退出状态待核对，未标记已清理') from None
                if reconciled:
                    self._commit(lifecycle,reconciled,cleanup=True)
                    return
                if state.get('session_sha256'):
                    # --collect may have removed the core's old Description and
                    # InvocationID. Viewer cleanup still validates its own exact
                    # invocation against this immutable admitted session.
                    try:
                        self._bound_session(state)
                        self.views.stop(state)
                        reconciled=self._inactive_reconciliation(state,self._unit())
                    except (OSError,ValueError,KeyError,TypeError,RuntimeError,subprocess.SubprocessError):
                        raise HTTPException(409,'会话视图所有权或退出状态待核对，未标记已清理') from None
                    if reconciled:
                        self._commit(lifecycle,reconciled,cleanup=True)
                        return
                    if (lifecycle['kind']=='start' and lifecycle.get('admitted') and lifecycle.get('launched')
                            and lifecycle['cancel'].is_set() and self._all_groups_empty(self._unit())):
                        # This launch may have been collected before its startup
                        # or shutdown ACK. Empty owned groups prove process
                        # cleanup, not whether its graph ran or retired a task.
                        # Keep the NEW admission and those unknown facts; the
                        # previous session's stopped phase proves none of them.
                        startup=self._startup_evidence(state)
                        self._commit(lifecycle,{**state,'phase':'stopped','quarantined':False,
                            'views':{},'view_error':'','startup_report':startup,
                            'startup_error':self._startup_error(startup) if startup and startup['phase']=='failed' else state.get('startup_error',''),
                            'stop_report':{'session_id':state['id'],
                                'software_started':startup['children_started'] if startup else None,
                                'request_accepted':False,'software_retired':False,'physical_stop_confirmed':False,
                                'cleanup_confirmed':True,'cleanup_evidence':'observed_empty_core_and_view_groups',
                                'reason':'cancelled_launch_process_cleanup'}},cleanup=True)
                        return
                if (state.get('id')==lifecycle['prior'].get('id')
                        and lifecycle['prior'].get('phase')=='stopped' and not lifecycle['prior'].get('quarantined')
                        and self._all_groups_empty(self._unit())):
                    self._commit(lifecycle,{**self._state(),'phase':'stopped','quarantined':False},cleanup=True)
                    return
                raise HTTPException(409,'进程已退出，但本次清理与退役证据待核对')
            state=self._adopt_owned_core(state,unit)
            if not state or not self._owned(state,unit):raise HTTPException(409,'所有权未核实，不停止其他进程')
            directory=Path(state['directory'])
            if directory.parent!=self.root/'sessions' or digest(directory/'session.json')!=state['session_sha256']:
                raise HTTPException(409,'会话绑定变化，停止结果需人工核对')
            state['phase']='stopping';state.setdefault('stop_requested_at_unix',time.time())
            state=self._commit(lifecycle,state,cleanup=True)
            # KillMode=mixed signals only the supervisor first. Its drain runs
            # while perception/tracker and the external SDK writer remain alive.
            view_records=self.views.preflight_stop(state) if isinstance(self.views,OwnedNavigationViews) else {}
            units=[UNIT,*[record['unit'] for record in view_records.values()]]
            try:self.runner(['systemctl','--user','stop',*units],check=True,capture_output=True,text=True,timeout=self.STOP_COMMAND_TIMEOUT_SECONDS)
            except (OSError,subprocess.SubprocessError):raise HTTPException(503,'停止未确认，不允许重复启动') from None
            after=self._unit()
            if after['ActiveState'] in ACTIVE or self.system.populated(after):raise HTTPException(409,'本次进程组尚未清空')
            try:drain=private_json(directory/'shutdown.json',16384)
            except (OSError,ValueError):drain={}
            clean=False
            state['stop_report']={
                'software_retired':drain.get('session_id')==state['id'] and drain.get('software_retired') is True,
                'physical_stop_confirmed':drain.get('session_id')==state['id'] and drain.get('physical_stop_confirmed') is True,
            }
            try:self.views.stop(state)
            except (OSError,ValueError,KeyError,TypeError,subprocess.SubprocessError):
                clean=False;state['view_error']='本次视图进程退出未确认'
            try:reconciled=self._inactive_reconciliation(state,after)
            except (OSError,ValueError,KeyError,TypeError,RuntimeError,subprocess.SubprocessError):reconciled=None
            if reconciled:state=reconciled;clean=True
            state.update(phase='stopped' if clean else 'needs_review',quarantined=not clean,exit_code=after.get('ExecMainStatus'))
            self._commit(lifecycle,state,cleanup=True)
            if not clean:raise HTTPException(409,'进程已退出，但停止与退役未确认，需人工核对')

    def _cancel_owned(self,lifecycle):
        if lifecycle.get('launched') or lifecycle['kind'] in ('stop','view'):
            self._stop_owned(lifecycle)
            return
        unit=self._unit()
        if not self._all_groups_empty(unit) or self.helper is not None:
            raise HTTPException(409,'准备取消后进程清理待核对')
        state={**lifecycle['prior'],'phase':'stopped','quarantined':False,
               'worker_error':'','worker_operation':'','worker_status_code':None}
        self._commit(lifecycle,state,cleanup=True)

    @contextmanager
    def _stop_scope(self,lifecycle):
        # No startup lease or long-held state lock: this is the same worker's
        # retirement phase. Retry only its own retained helper handle if needed.
        if self.helper is not None:
            previous=self.helper_operation
            proof=self._cleanup_helper(previous,self.helper)
            if previous['kind']=='start' and not previous.get('admitted') and not previous.get('launched'):
                lifecycle['local_prepare_cleanup']=proof
            with self.lock:self.helper=None;self.helper_operation=None
        yield

    def _lifecycle_worker(self,lifecycle):
        try:
            if lifecycle['kind']=='start':self._start_owned(lifecycle)
            elif lifecycle['kind']=='connect':self._connect_owned(lifecycle)
            elif lifecycle['kind']=='view':
                self._check_operation(lifecycle)
                try:self._open_view(self._state(),lifecycle['view_layout'],lifecycle=lifecycle)
                except HTTPException:pass  # Keep display errors separate from core health.
                self._check_operation(lifecycle)
            else:self._stop_owned(lifecycle)
        except LifecycleCancelled:
            try:self._cancel_owned(lifecycle)
            except (HTTPException,OSError,ValueError,KeyError,TypeError,RuntimeError,subprocess.SubprocessError) as error:
                self._worker_failure(lifecycle,error,retiring=True)
        except (HTTPException,OSError,ValueError,KeyError,TypeError,RuntimeError,subprocess.SubprocessError) as error:
            if lifecycle['cancel'].is_set():
                try:self._cancel_owned(lifecycle)
                except (HTTPException,OSError,ValueError,KeyError,TypeError,RuntimeError,subprocess.SubprocessError) as cleanup_error:
                    self._worker_failure(lifecycle,cleanup_error,retiring=True)
            else:self._worker_failure(lifecycle,error)
        finally:
            with self.changed:
                if self.operation is lifecycle:self.operation=None
                self.changed.notify_all()

    def _worker_failure(self,lifecycle,error,*,retiring=False):
        with self.lock:
            if self.operation is not lifecycle:return
            state=self._state()
            if (not lifecycle.get('admitted') and lifecycle['kind'] in ('start','connect') and self.helper is None
                    and not lifecycle.get('helper_cleanup_unconfirmed') and not retiring):
                state={**lifecycle['prior'],'phase':'stopped','quarantined':False}
            elif state.get('phase') not in ('stopped','needs_review'):
                state.update(phase='needs_review',quarantined=True)
            state.update(worker_error=error.detail if isinstance(error,HTTPException) else self._startup_error({'error':str(error)}),
                         worker_operation='stop' if retiring or lifecycle['kind']=='stop' else lifecycle['kind'],
                         worker_status_code=error.status_code if isinstance(error,HTTPException) else 503)
            self._commit(lifecycle,state,cleanup=True)

    def _accept(self,kind,state,*,requested_purpose=None,view_layout=None):
        self.generation+=1
        lifecycle=dict(id=uuid.uuid4().hex,generation=self.generation,kind=kind,cancel=threading.Event(),prior=state,
                       state=state,view_layout=view_layout,requested_purpose=requested_purpose,
                       admitted=bool(state.get('session_sha256')) if kind in ('stop','view') else False)
        self.operation=lifecycle
        accepted=({'purpose':requested_purpose} if kind=='start' else {**state})
        if kind!='view':
            accepted.update(phase={'start':'preparing','stop':'stopping','connect':'connecting'}[kind],quarantined=False,
                            worker_error='',worker_operation='',worker_status_code=None,startup_error='')
        if kind=='stop':accepted.setdefault('stop_requested_at_unix',time.time())
        try:
            self._commit(lifecycle,accepted)
            self.worker=threading.Thread(target=self._lifecycle_worker,args=(lifecycle,),daemon=True,name='mainline-lifecycle')
            self.worker.start()
        except Exception:
            self.operation=None;self.worker=None
            failure=({'view_error':'RViz 未打开（view_worker_unavailable）'} if kind=='view' else
                {'worker_error':'lifecycle_worker_unavailable','worker_operation':kind,'worker_status_code':503})
            try:persist(self.state_path,{**state,**failure,'generation':self.generation})
            except (OSError,ValueError,TypeError):pass
            self.changed.notify_all()
            raise HTTPException(503,'导航操作未受理，请核对 Web 状态') from None

    def start(self,purpose=None):
        with self._admission():
            if self.closed or self.operation is not None:raise HTTPException(409,'导航服务正在切换或 Web 正在退出')
        current=self.snapshot()
        with self._admission():
            if self.closed:raise HTTPException(409,'Web 正在退出')
            if self.operation is not None:raise HTTPException(409,'导航服务正在切换')
            if current['busy'] or current.get('quarantined'):raise HTTPException(409,'服务未清理或停止结果待核对')
            state=self._state()
            if state.get('session_sha256'):
                try:reconciled=self._inactive_reconciliation(state,self._unit())
                except (OSError,ValueError,KeyError,TypeError,RuntimeError,subprocess.SubprocessError):
                    raise HTTPException(409,'上次导航会话绑定或退出状态待核对，不重复启动') from None
                if reconciled:state=reconciled
            purpose=self._requested_purpose(purpose)
            # The retired admission remains immutable until a NEW session has
            # been prepared. In particular a preview->execution switch must
            # not change lifecycle.prior or the failure/cancellation fallback.
            self._accept('start',state,requested_purpose=purpose)
        return self.snapshot(connection=True)

    def _request_stop(self):
        if self.operation is not None:
            if self.operation['kind'] in ('start','connect','view') and not self.operation['cancel'].is_set():
                self.generation+=1;self.operation['cancel'].set()
                state=self._state();state.update(phase='stopping')
                if self.operation['kind'] in ('start','view'):state['stop_requested_at_unix']=time.time()
                self._commit(self.operation,state,cleanup=True)
        else:
            state=self._state();unit=self._unit()
            if unit['ActiveState'] in ACTIVE or self.system.populated(unit):
                try:state=self._adopt_owned_core(state,unit)
                except (OSError,ValueError,KeyError,TypeError):raise HTTPException(409,'会话绑定或进程身份待核对') from None
                if not state or not self._owned(state,unit):raise HTTPException(409,'所有权未核实，不停止其他进程')
            if not state and not self._views_busy():return
            self._accept('stop',state)

    def stop(self):
        with self._admission():self._request_stop()
        return self.snapshot(connection=True)

    def close(self):
        deadline=time.monotonic()+30.  # Existing managed Web TimeoutStopSec.
        with self.lock:
            self.closed=True
            try:self._request_stop()
            except (HTTPException,OSError,ValueError,KeyError,subprocess.SubprocessError):pass
        worker=self.worker
        if worker:worker.join(max(0.,deadline-time.monotonic()))


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
    def connect(_request:NavigationPurposeRequest):
        if not shared_lock.acquire(False):raise HTTPException(409,'其他任务正在启动')
        try:
            if other_busy():raise HTTPException(409,'请先停止建图、录包或其他导航')
            return runtime.connect(_request.purpose)
        finally:shared_lock.release()
    @router.post('/start',status_code=202)
    def start(_request:NavigationPurposeRequest):
        if not shared_lock.acquire(False):raise HTTPException(409,'其他任务正在启动')
        try:
            if other_busy():raise HTTPException(409,'请先停止建图、录包或其他导航')
            return runtime.start(_request.purpose)
        finally:shared_lock.release()
    @router.post('/stop',status_code=202)
    def stop(_request:EmptyRequest):return runtime.stop()
    return router


def create_single_floor_router(runtime,shared_lock,other_busy):
    """Compatibility routes bound to the caller's existing session runtime."""
    return _create_session_router(runtime,shared_lock,other_busy,prefix='/api/single-floor')
