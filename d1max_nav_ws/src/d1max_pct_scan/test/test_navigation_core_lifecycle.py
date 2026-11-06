"""Owned process fixtures only: no ROS context, navigation or SDK is started."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest

from d1max_pct_scan import single_floor_session as entry
from d1max_pct_scan import live_runtime, live_view_reload, lifecycle_shutdown


class Child:
    def __init__(self, code=None):
        self.returncode=code
        self.pid=10001

    def poll(self):
        return self.returncode


@pytest.fixture
def graph_fixture(tmp_path,monkeypatch):
    session=dict(id='owned-lifecycle-test',transport_mode='live',purpose='planning_only',
        execution_purpose='planning_only',mode='SINGLE_FLOOR_PLANNING_ONLY',
        motion_control_enabled=False)
    commands={'early':['fixture-early'],'navigator':['fixture-navigator']}
    children=[];events=[]
    monkeypatch.setenv('RMW_IMPLEMENTATION','rmw_zenoh_cpp')
    monkeypatch.setenv('D1MAX_NAV_TRANSPORT','live')
    monkeypatch.setattr(entry,'verify',lambda _: (session,commands))
    monkeypatch.setattr(entry.signal,'signal',lambda sig,handler:events.append(('signal',sig)))
    def popen(cmd,**kwargs):
        assert kwargs['start_new_session'] is True
        assert kwargs['env']['RMW_IMPLEMENTATION']=='rmw_zenoh_cpp'
        trace_directory=Path(kwargs['env']['D1MAX_TRACE_DIRECTORY'])
        assert trace_directory.is_relative_to(tmp_path/'diagnostics')
        assert json.loads((trace_directory/'session-index.json').read_text())['session_id']==session['id']
        child=Child(42 if not children else None)
        children.append(child)
        return child
    monkeypatch.setattr(entry.subprocess,'Popen',popen)
    def cleanup(owned,**budgets):
        events.append(('cleanup',[name for name,_ in owned],budgets))
        return dict(all_direct_children_exited=True,all_owned_groups_exited=True,
            children=[],elapsed_s=0.,signals=[])
    monkeypatch.setattr(live_runtime,'stop_owned_children',cleanup)
    def drain(_):
        events.append(('drain',))
        return dict(request_accepted=True,software_retired=True,physical_stop_confirmed=False)
    monkeypatch.setattr(lifecycle_shutdown,'drain_task_owner',drain)
    original_write=live_view_reload.write_owned_json
    def write(path,value):
        events.append(('write',Path(path).name,value.copy()))
        original_write(path,value)
    monkeypatch.setattr(live_view_reload,'write_owned_json',write)
    return tmp_path,commands,children,events


def test_critical_early_exit_never_publishes_started_ack(graph_fixture):
    directory,_,_,events=graph_fixture
    with pytest.raises(RuntimeError,match='critical_component_exited'):
        entry._run_locked(directory)
    phases=[event[2]['phase'] for event in events
        if event[0]=='write' and event[1]=='core_startup.json']
    assert phases==['spawning']
    report=json.loads((directory/'shutdown.json').read_text())
    assert report['navigator_started'] is True and report['navigator_alive_at_drain'] is True
    assert report['request_accepted'] is True and report['software_retired'] is True
    assert report['physical_stop_confirmed'] is False
    assert report['process_cleanup']['all_owned_groups_exited'] is True
    names=[event[0] for event in events]
    assert names.index('drain')<names.index('cleanup')
    assert next(i for i,event in enumerate(events)
        if event[0]=='write' and event[1]=='shutdown.json')>names.index('cleanup')
    budgets=next(event[2] for event in events if event[0]=='cleanup')
    assert budgets==dict(interrupt_s=6.,terminate_s=1.,kill_s=1.)
    assert 6.+sum(budgets.values())<30.
    assert (directory/'shutdown.json').stat().st_mode&0o777==0o600


def test_partial_spawn_without_navigator_does_not_wait_for_nonexistent_owner(graph_fixture,monkeypatch):
    directory,_,children,events=graph_fixture
    def popen(*args,**kwargs):
        if children:raise OSError('second_child_exec_failed')
        children.append(Child())
        return children[-1]
    monkeypatch.setattr(entry.subprocess,'Popen',popen)
    with pytest.raises(OSError,match='second_child_exec_failed'):
        entry._run_locked(directory)
    assert not any(event[0]=='drain' for event in events)
    report=json.loads((directory/'shutdown.json').read_text())
    assert report['navigator_started'] is False
    assert report['navigator_alive_at_drain'] is False
    assert report['reason']=='task_owner_never_started'
    assert report['request_accepted'] is False and report['software_retired'] is False
    assert report['physical_stop_confirmed'] is False
    assert report['process_cleanup']['all_direct_children_exited'] is True
    assert report['process_cleanup']['all_owned_groups_exited'] is True


def test_started_ack_is_spawn_liveness_not_a_later_exit_success(graph_fixture,monkeypatch):
    directory,_,children,events=graph_fixture
    class LaterExit(Child):
        def __init__(self):
            super().__init__()
            self.polls=0
        def poll(self):
            self.polls+=1
            if self.polls>=2:self.returncode=17
            return self.returncode
    def popen(*args,**kwargs):
        children.append(LaterExit() if not children else Child())
        return children[-1]
    monkeypatch.setattr(entry.subprocess,'Popen',popen)
    with pytest.raises(RuntimeError,match='critical_component_exited'):
        entry._run_locked(directory)
    phases=[event[2]['phase'] for event in events
        if event[0]=='write' and event[1]=='core_startup.json']
    assert phases==['spawning','started']
    report=json.loads((directory/'shutdown.json').read_text())
    assert report['process_cleanup']['all_owned_groups_exited'] is True
    assert report['physical_stop_confirmed'] is False


def test_drain_error_does_not_prevent_actual_group_cleanup(graph_fixture,monkeypatch):
    directory,_,_,events=graph_fixture
    def failed_drain(_):raise RuntimeError('no_task_owner_ack')
    monkeypatch.setattr(lifecycle_shutdown,'drain_task_owner',failed_drain)
    with pytest.raises(RuntimeError,match='critical_component_exited'):
        entry._run_locked(directory)
    report=json.loads((directory/'shutdown.json').read_text())
    assert report['request_accepted'] is False and report['software_retired'] is False
    assert report['physical_stop_confirmed'] is False
    assert report['reason']=='task_owner_drain_error:RuntimeError'
    assert report['process_cleanup']['all_owned_groups_exited'] is True
    assert any(event[0]=='cleanup' for event in events)


def test_dead_navigator_is_not_treated_as_accepted_retirement(graph_fixture,monkeypatch):
    directory,_,children,events=graph_fixture
    def popen(*args,**kwargs):
        children.append(Child(23))
        return children[-1]
    monkeypatch.setattr(entry.subprocess,'Popen',popen)
    with pytest.raises(RuntimeError,match='critical_component_exited'):
        entry._run_locked(directory)
    assert not any(event[0]=='drain' for event in events)
    report=json.loads((directory/'shutdown.json').read_text())
    assert report['navigator_started'] is True
    assert report['navigator_alive_at_drain'] is False
    assert report['reason']=='task_owner_not_alive'
    assert report['software_retired'] is False and report['physical_stop_confirmed'] is False


@pytest.mark.parametrize('raises', [False,True])
def test_process_cleanup_failure_never_turns_into_cleanup_or_physical_success(graph_fixture,monkeypatch,raises):
    directory,_,_,events=graph_fixture
    def incomplete(*args,**kwargs):
        events.append(('cleanup',))
        if raises:raise PermissionError('cleanup_rejected')
        return dict(all_direct_children_exited=True,all_owned_groups_exited=False,
            children=[dict(name='early',pid=10001,returncode=0,group_alive=True)])
    monkeypatch.setattr(live_runtime,'stop_owned_children',incomplete)
    with pytest.raises(RuntimeError,match='critical_component_exited'):
        entry._run_locked(directory)
    report=json.loads((directory/'shutdown.json').read_text())
    # These are separate original facts: task retirement can succeed while a
    # process group remains. Neither fact is physical stopping.
    assert report['request_accepted'] is True and report['software_retired'] is True
    assert report['physical_stop_confirmed'] is False
    assert report['process_cleanup']['all_owned_groups_exited'] is False
    if raises:assert report['process_cleanup']['all_direct_children_exited'] is False


def test_shutdown_disk_failure_cannot_skip_group_cleanup_or_handler_restore(graph_fixture,monkeypatch):
    directory,_,_,events=graph_fixture
    original_write=live_view_reload.write_owned_json
    def write(path,value):
        if Path(path).name=='shutdown.json':raise OSError('shutdown_disk_full')
        original_write(path,value)
    monkeypatch.setattr(live_view_reload,'write_owned_json',write)
    with pytest.raises(OSError,match='shutdown_disk_full'):
        entry._run_locked(directory)
    assert any(event[0]=='cleanup' for event in events)
    # Both handlers are restored even if final evidence cannot be published.
    assert len([event for event in events if event[0]=='signal'])==4


def _wait_for_file(path,timeout_s=3.):
    deadline=time.monotonic()+timeout_s
    while not path.exists() and time.monotonic()<deadline:time.sleep(.01)
    assert path.exists()


def _last_resort(children):
    for child in children:
        try:os.killpg(child.pid,signal.SIGKILL)
        except ProcessLookupError:pass
        child.wait(timeout=2.)


def test_real_batch_helper_has_shared_not_per_child_phase_budget(tmp_path):
    children=[]
    try:
        for index in range(4):
            ready=tmp_path/('ready-'+str(index))
            program=('import signal,time,pathlib,sys; '
                'signal.signal(signal.SIGINT,signal.SIG_IGN); '
                'signal.signal(signal.SIGTERM,signal.SIG_IGN); '
                'pathlib.Path(sys.argv[1]).touch(); time.sleep(30)')
            child=subprocess.Popen([sys.executable,'-c',program,str(ready)],start_new_session=True)
            children.append(child)
            _wait_for_file(ready)
        report=live_runtime.stop_owned_children([('fixture-'+str(i),child)
            for i,child in enumerate(children)],interrupt_s=.1,terminate_s=.1,kill_s=.4)
        assert report['all_direct_children_exited'] is True
        assert report['all_owned_groups_exited'] is True
        assert report['signals']==['SIGINT','SIGTERM','SIGKILL']
        assert report['elapsed_s']<1.2
    finally:_last_resort(children)


def test_real_helper_retires_group_after_direct_parent_exited(tmp_path):
    ready=tmp_path/'grandchild-ready'
    program='''import os,signal,time,pathlib,sys
pid=os.fork()
if pid==0:
    signal.signal(signal.SIGINT,signal.SIG_IGN)
    pathlib.Path(sys.argv[1]).write_text(str(os.getpid()))
    time.sleep(30)
    os._exit(0)
else:
    while not pathlib.Path(sys.argv[1]).exists():time.sleep(.01)
    os._exit(0)
'''
    parent=subprocess.Popen([sys.executable,'-c',program,str(ready)],start_new_session=True)
    try:
        _wait_for_file(ready)
        parent.wait(timeout=2.)
        assert parent.poll()==0 and live_runtime.group_alive(parent)
        report=live_runtime.stop_owned_children([('exited-parent',parent)],
            interrupt_s=.1,terminate_s=.2,kill_s=.5)
        assert report['all_direct_children_exited'] is True
        assert report['all_owned_groups_exited'] is True
        assert 'SIGTERM' in report['signals']
    finally:_last_resort([parent])


@pytest.fixture
def view_fixture(tmp_path,monkeypatch):
    from contextlib import contextmanager
    from d1max_pct_scan import view_layout_requests
    children=[];streams=[];events=[]
    monkeypatch.setenv('RMW_IMPLEMENTATION','rmw_zenoh_cpp')
    monkeypatch.setattr(entry,'verify_view',lambda _: dict(id='view-fixture',transport_mode='live'))
    monkeypatch.setattr(entry.signal,'signal',lambda sig,handler:events.append(('signal',sig)))
    monkeypatch.setattr(view_layout_requests,'viewer_environment',lambda *args:dict(VIEW_FIXTURE='1'))
    @contextmanager
    def window(*args):
        events.append(('window_acquired',))
        try:yield 77
        finally:events.append(('window_released',))
    monkeypatch.setattr(entry,'view_layout_lock',window)
    class DisplayLease:
        def __exit__(self,*args):events.append(('display_released',))
    monkeypatch.setattr(entry,'_try_display_lease',lambda _: (DisplayLease(),88))
    def popen(cmd,**options):
        assert options['start_new_session'] is True
        streams.append(options['stdout'])
        if not children:
            assert options['pass_fds']==(77,) and options['env']==dict(VIEW_FIXTURE='1')
        else:assert options['pass_fds']==(88,77)
        children.append(Child())
        return children[-1]
    monkeypatch.setattr(entry.subprocess,'Popen',popen)
    monkeypatch.setattr(entry.time,'sleep',lambda _: setattr(children[0],'returncode',0))
    def cleanup(owned,**budgets):
        events.append(('cleanup',[name for name,_ in owned],budgets))
        return dict(all_direct_children_exited=True,all_owned_groups_exited=True)
    monkeypatch.setattr(live_runtime,'stop_owned_children',cleanup)
    monkeypatch.setattr(lifecycle_shutdown,'drain_task_owner',lambda *args:
        pytest.fail('view must not drain or cancel the navigation task'))
    return tmp_path,children,streams,events


def test_view_uses_same_owned_batch_without_releasing_window_early(view_fixture):
    directory,_,streams,events=view_fixture
    entry.view(directory,'global')
    cleanups=[event for event in events if event[0]=='cleanup']
    assert len(cleanups)==1
    assert cleanups[0][1]==['rviz','view','map_layers','execution_view']
    assert cleanups[0][2]==dict(interrupt_s=6.,terminate_s=1.,kill_s=1.)
    assert sum(cleanups[0][2].values())<12.
    names=[event[0] for event in events]
    assert names.index('cleanup')<names.index('display_released')<names.index('window_released')
    assert all(stream.closed for stream in streams)
    assert len([event for event in events if event[0]=='signal'])==4
    assert not (directory/'shutdown.json').exists()


def test_view_cleanup_error_still_releases_all_local_resources(view_fixture,monkeypatch):
    directory,_,streams,events=view_fixture
    def cleanup_error(*args,**kwargs):
        events.append(('cleanup',))
        raise PermissionError('owned_view_cleanup_failed')
    monkeypatch.setattr(live_runtime,'stop_owned_children',cleanup_error)
    with pytest.raises(PermissionError,match='owned_view_cleanup_failed'):entry.view(directory,'local')
    assert all(stream.closed for stream in streams)
    assert ('display_released',) in events and ('window_released',) in events
    assert len([event for event in events if event[0]=='signal'])==4
    assert not (directory/'shutdown.json').exists()


def test_view_partial_publisher_spawn_uses_shared_cleanup_and_keeps_fd_ownership(view_fixture,monkeypatch):
    directory,children,streams,events=view_fixture
    original_popen=entry.subprocess.Popen
    def failed_popen(*args,**kwargs):
        if len(children)==2:
            streams.append(kwargs['stdout'])
            raise OSError('map_layer_exec_failed')
        return original_popen(*args,**kwargs)
    monkeypatch.setattr(entry.subprocess,'Popen',failed_popen)
    with pytest.raises(OSError,match='map_layer_exec_failed'):entry.view(directory,'global')
    assert next(event[1] for event in events if event[0]=='cleanup')==['rviz','view']
    assert all(stream.closed for stream in streams)
    assert ('display_released',) in events and ('window_released',) in events
    assert len([event for event in events if event[0]=='signal'])==4


def test_resource_unwind_restores_all_even_when_one_log_close_fails(monkeypatch):
    events=[]
    class Stream:
        def __init__(self,name,fails=False):self.name,self.fails=name,fails
        def close(self):
            events.append(('closed',self.name))
            if self.fails:raise OSError('log_close_failed')
    class Lease:
        def __exit__(self,*args):events.append(('released',))
    monkeypatch.setattr(entry.signal,'signal',lambda sig,handler:events.append(('signal',sig)))
    with pytest.raises(OSError,match='log_close_failed'):
        entry.release_supervisor_resources([Stream('a'),Stream('b',True),Stream('c')],
            {signal.SIGTERM:None,signal.SIGINT:None},Lease())
    assert {event[1] for event in events if event[0]=='closed'}=={'a','b','c'}
    assert ('released',) in events
    assert {event[1] for event in events if event[0]=='signal'}=={signal.SIGTERM,signal.SIGINT}
