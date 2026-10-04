"""Terminal owner evidence cannot replace source-timed geometric arrival."""
from copy import deepcopy
import pytest
from d1max_planning_interfaces.msg import ExecutionPermit,ExecutionVersion,StopReport
from test_bt_action_adapters import harness,commit,time_msg
from test_continuous_reference_transport import state


def fixture(monkeypatch):
    node,clock=harness(monkeypatch);commit(node,clock)
    node.p.update(pipeline_contract='single_floor_v3',map_version_id='map',transport_mode='isolated_mock')
    version=ExecutionVersion(schema_version=3,session_id='session',task_id=node.snapshot['task_id'],
        route_id=node.snapshot['route_id'],route_hash=node.snapshot['source_snapshot'].route_hash,
        map_version_id='map',localization_epoch=1,localization_seed_id='seed')
    permit=ExecutionPermit(version=version,execution_id='run',control_epoch=1,sdk_session='sdk',
        sdk_arm_generation=1,sequence=1,source_stamp=time_msg(clock.wall),valid_until=time_msg(clock.wall+.4),
        phase='terminal',reason='goal_reached',transport_mode='isolated_mock')
    node.on_execution_permit(permit)
    stop=StopReport(version=deepcopy(version),execution_id='run',control_epoch=1,sdk_session='sdk',
        sdk_arm_generation=1,sequence=1,source_stamp=time_msg(clock.wall),measured_stop_confirmed=True,
        nonzero_blocked=True,transport_mode='isolated_mock')
    node.on_stop_report(stop)
    node.local_ready=lambda _:False # planner already retired is not an arrival blocker
    return node,clock


def measured(node,clock,x=2.):
    node.on_atomic_state(state(t=clock.wall-10.,x=x))


def test_measured_terminal_stop_at_goal_finishes_without_live_scan(monkeypatch):
    node,clock=fixture(monkeypatch);measured(node,clock)
    node.follow_tick()
    assert node.follow['cancel']['reason']=='goal_reached'


@pytest.mark.parametrize('case',['outside','stale_pose','stale_stop','old_sdk','pre_stop_pose','yaw'])
def test_terminal_event_does_not_override_actual_arrival_failure(monkeypatch,case):
    node,clock=fixture(monkeypatch)
    if case=='outside':measured(node,clock,x=3.)
    elif case=='stale_pose':pass
    elif case=='stale_stop':
        measured(node,clock);node.stop_report.source_stamp=time_msg(clock.wall-1.)
    elif case=='old_sdk':
        measured(node,clock);node.stop_report.sdk_arm_generation=0
    elif case=='pre_stop_pose':
        measured(node,clock);node.stop_report.source_stamp=time_msg(clock.wall+.01)
    else:
        measured(node,clock)
        node.snapshot['source_snapshot']=node.snapshot['source_snapshot'].with_goal(has_goal_yaw=True,goal_yaw=1.)
    node.follow_tick()
    if case in ('outside','yaw'):
        assert node.follow['cancel']['reason']=='post_stop_goal_mismatch'
    else:
        assert node.follow['cancel'] is None


def window_stop_fixture(monkeypatch):
    node,clock=fixture(monkeypatch)
    stop=deepcopy(node.stop_report)
    stop.version.anchor_id='initial-anchor';stop.version.anchor_revision=1
    stop.version.reference_generation=1
    permit=deepcopy(node.execution_permit);permit.sequence+=1
    permit.version.anchor_id='corrected-anchor';permit.version.anchor_revision=3
    permit.version.reference_generation=9
    node.on_execution_permit(permit)
    node.stop_report=None;stop.sequence+=1
    return node,clock,stop


def test_measured_stop_from_grant_binding_covers_later_committed_anchor(monkeypatch):
    node,clock,stop=window_stop_fixture(monkeypatch)
    node.on_stop_report(stop);measured(node,clock)
    assert node.stop_report is not None
    assert node.stop_report.version != node.execution_permit.version
    node.follow_tick()
    assert node.follow['cancel']['reason']=='goal_reached'


@pytest.mark.parametrize('field',['schema_version','session_id','task_id','route_id','route_hash',
    'map_version_id','localization_epoch','localization_seed_id','execution_id',
    'control_epoch','sdk_session','sdk_arm_generation'])
def test_later_anchor_does_not_relax_task_or_execution_stop_ownership(monkeypatch,field):
    node,clock,stop=window_stop_fixture(monkeypatch)
    target=stop.version if hasattr(stop.version,field) else stop
    old=getattr(target,field);setattr(target,field,old+1 if isinstance(old,int) else old+'-foreign')
    node.on_stop_report(stop);measured(node,clock)
    assert node.stop_report is None
    node.follow_tick()
    assert node.follow['cancel'] is None


@pytest.mark.parametrize('case',['stale','not_measured','nonzero_not_blocked','before_stop_body'])
def test_later_anchor_keeps_source_time_and_real_stop_proofs(monkeypatch,case):
    node,clock,stop=window_stop_fixture(monkeypatch)
    if case=='stale':stop.source_stamp=time_msg(clock.wall-1.)
    elif case=='not_measured':stop.measured_stop_confirmed=False
    elif case=='nonzero_not_blocked':stop.nonzero_blocked=False
    else:stop.source_stamp=time_msg(clock.wall+.01)
    node.on_stop_report(stop);measured(node,clock)
    node.follow_tick()
    assert node.follow['cancel'] is None
