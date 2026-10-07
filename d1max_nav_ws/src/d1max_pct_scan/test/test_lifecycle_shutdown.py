from copy import deepcopy
import json
import sys
from types import SimpleNamespace
import pytest
from d1max_pct_scan import lifecycle_shutdown as module
from d1max_pct_scan.lifecycle_shutdown import OriginalClockSource, drained_status


def status():
    return dict(schema=2, session_id='s', stamp=10.2, lifecycle_active=False,
                lifecycle_draining=False, lifecycle_quarantined=False,
                physical_stop_confirmed=False)


def check(value):
    return drained_status(value, session_id='s', requested_at=10., now=10.3)


def test_software_retirement_does_not_require_or_invent_physical_stop():
    value = status()
    assert check(value)
    assert value['physical_stop_confirmed'] is False


@pytest.mark.parametrize('key,value', [('schema',1), ('session_id','old'),
    ('stamp',9.9), ('stamp',10.9), ('stamp',float('nan')),
    ('lifecycle_active',True), ('lifecycle_draining',True),
    ('lifecycle_quarantined',True)])
def test_stale_foreign_active_or_quarantined_is_not_drained(key, value):
    data = deepcopy(status())
    data[key] = value
    assert not check(data)


def fake_shutdown_ros(monkeypatch, *, initial_source=0., steps=(), service_ready=True,
                      response_ready=True, response_accepted=True):
    """Run the real shutdown loop with fake imports and independent clocks."""
    clock=SimpleNamespace(mono=0.,source=initial_source)
    events=SimpleNamespace(node_options=None,requests=[],observations=[],spins=0,
        context_shutdown=False,node_destroyed=False,executor_shutdown=False,subscriptions={})
    callbacks={}

    class FakeContext:
        live=False
        def ok(self): return self.live
        def shutdown(self): self.live=False;events.context_shutdown=True

    class FakeParameter:
        def __init__(self,name,*,value): self.name,self.value=name,value

    class FakeFuture:
        def done(self): return response_ready
        def result(self): return SimpleNamespace(schema_version=2,accepted=response_accepted)

    class FakeClient:
        def service_is_ready(self): return service_ready
        def call_async(self,request):
            events.requests.append(dict(source=clock.source,mono=clock.mono,
                schema=request.schema_version,reason=request.reason,session_id=request.session_id))
            return FakeFuture()

    class FakeNode:
        def get_clock(self):
            return SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=round(clock.source*1e9)))
        def create_subscription(self,message_type,topic,observe,qos):
            callbacks[topic]=observe
            events.subscriptions[topic]=qos
            return SimpleNamespace()
        def create_client(self,service_type,topic): return FakeClient()
        def destroy_node(self): events.node_destroyed=True

    class FakeExecutor:
        def __init__(self,*,context): pass
        def add_node(self,node): pass
        def spin_once(self,*,timeout_sec):
            clock.mono+=timeout_sec
            step=steps[events.spins] if events.spins<len(steps) else {}
            events.spins+=1
            if 'source' in step:
                clock.source=step['source']
                if '/clock' in callbacks:
                    ns=round(clock.source*1e9)
                    callbacks['/clock'](SimpleNamespace(clock=SimpleNamespace(
                        sec=ns//1000000000,nanosec=ns%1000000000)))
            if 'clock_ns' in step and '/clock' in callbacks:
                ns=step['clock_ns']
                callbacks['/clock'](SimpleNamespace(clock=SimpleNamespace(
                    sec=ns//1000000000,nanosec=ns%1000000000)))
            if 'status_stamp' in step:
                value=status();value.update(session_id='s',stamp=step['status_stamp'])
                callbacks['/d1max/live_planning/bt/status'](SimpleNamespace(data=json.dumps(value)))
        def shutdown(self,*,timeout_sec): events.executor_shutdown=True

    def init(*,context,**kwargs): context.live=True
    def create_node(name,**kwargs): events.node_options=kwargs;return FakeNode()
    fake_rclpy=SimpleNamespace(init=init,create_node=create_node,Parameter=FakeParameter)
    imports={'rclpy':fake_rclpy,'rclpy.context':SimpleNamespace(Context=FakeContext),
        'rclpy.parameter':SimpleNamespace(Parameter=FakeParameter),
        'rclpy.executors':SimpleNamespace(SingleThreadedExecutor=FakeExecutor),
        'rclpy.qos':SimpleNamespace(QoSProfile=lambda **kwargs:kwargs,
            DurabilityPolicy=SimpleNamespace(TRANSIENT_LOCAL='durable')),
        'std_msgs.msg':SimpleNamespace(String=SimpleNamespace),
        'rosgraph_msgs.msg':SimpleNamespace(Clock=SimpleNamespace),
        'd1max_navigation_bt_interfaces.srv':SimpleNamespace(
            PrepareTransition=SimpleNamespace(Request=SimpleNamespace))}
    for name,value in imports.items(): monkeypatch.setitem(sys.modules,name,value)
    monkeypatch.setattr(module,'time',SimpleNamespace(monotonic=lambda:clock.mono))
    def observe_gate(value,**kwargs):
        accepted=drained_status(value,**kwargs)
        events.observations.append(dict(stamp=value['stamp'],accepted=accepted,**kwargs))
        return accepted
    monkeypatch.setattr(module,'drained_status',observe_gate)
    return clock,events


def assert_fake_shutdown_cleaned_up(events):
    assert events.context_shutdown and events.node_destroyed and events.executor_shutdown


def test_shutdown_defaults_to_explicit_system_clock_and_ignores_global_ros_arguments(monkeypatch):
    _,events=fake_shutdown_ros(monkeypatch,initial_source=20.,steps=(
        {'source':20.1,'status_stamp':20.1},{'source':20.2,'status_stamp':20.2}))
    result=module.drain_task_owner('s')
    assert result['request_accepted'] and result['software_retired']
    assert result['request_sent'] and result['timeout_stage'] is None
    assert result['requested_source_ns']==20_000_000_000 and result['requested_at']==20.
    assert not result['physical_stop_confirmed']
    assert events.node_options['use_global_arguments'] is False
    parameters=events.node_options['parameter_overrides']
    assert [(p.name,p.value) for p in parameters]==[('use_sim_time',False)]
    assert type(parameters[0].value) is bool
    assert events.requests[0]['source']==20.
    assert events.requests[0]['schema']==2 and events.requests[0]['session_id']=='s'
    assert '/clock' not in events.subscriptions
    assert_fake_shutdown_cleaned_up(events)


def test_simulated_shutdown_waits_for_clock_then_fences_old_inactive_status(monkeypatch):
    _,events=fake_shutdown_ros(monkeypatch,steps=(
        {'source':0.,'status_stamp':0.},
        {'source':20.,'status_stamp':0.},
        {'source':20.1,'status_stamp':19.9},
        {'source':20.2,'status_stamp':19.9},
        {'source':20.3,'status_stamp':20.3}))
    result=module.drain_task_owner('s',use_sim_time=True)
    assert result['request_accepted'] and result['software_retired']
    assert not result['physical_stop_confirmed']
    assert [(p.name,p.value) for p in events.node_options['parameter_overrides']]==[('use_sim_time',True)]
    assert events.node_options['use_global_arguments'] is False
    assert len(events.requests)==1
    assert events.requests[0]['source']==20. and events.requests[0]['mono']==pytest.approx(.1)
    # The 19.9 s status is within the .6 s source freshness window, but older
    # than the barrier captured immediately before the service call at 20 s.
    assert [(o['stamp'],o['accepted']) for o in events.observations]==[(19.9,False),(20.3,True)]
    assert all(o['requested_at']==20. for o in events.observations)
    assert result['elapsed_s']==pytest.approx(.25)
    assert_fake_shutdown_cleaned_up(events)


def test_simulated_shutdown_clock_stays_zero_consumes_original_wall_budget(monkeypatch):
    clock,events=fake_shutdown_ros(monkeypatch,steps=({'source':0.,'status_stamp':0.},)*140)
    result=module.drain_task_owner('s',use_sim_time=True)
    assert not result['request_accepted'] and not result['software_retired']
    assert not result['physical_stop_confirmed'] and not events.requests
    assert not result['request_sent'] and result['timeout_stage']=='clock'
    assert result['requested_at'] is None and result['requested_source_ns'] is None
    assert not events.observations
    assert result['reason']=='task_owner_drain_timeout'
    assert clock.source==0.
    assert 6.<=result['elapsed_s']<6.051
    assert_fake_shutdown_cleaned_up(events)


def test_late_joined_original_clock_drains_while_builtin_ros_clock_stays_zero(monkeypatch):
    ns=1_791_377_450_939_382_334
    clock,events=fake_shutdown_ros(monkeypatch,steps=(
        {'clock_ns':ns,'status_stamp':0.},
        {'status_stamp':ns*1e-9},
        {'status_stamp':ns*1e-9}))
    result=module.drain_task_owner('s',use_sim_time=True)
    assert clock.source==0. and events.requests[0]['source']==0.
    assert events.subscriptions['/clock']=={'depth':1,'durability':'durable'}
    assert result['request_sent'] and result['request_accepted'] and result['software_retired']
    assert result['requested_source_ns']==ns
    assert result['requested_at']==ns*1e-9 and not result['physical_stop_confirmed']
    assert result['elapsed_s']==pytest.approx(.15)
    assert_fake_shutdown_cleaned_up(events)


def test_simulated_regression_after_request_cannot_use_retained_clock_or_old_status(monkeypatch):
    _,events=fake_shutdown_ros(monkeypatch,steps=(
        {'clock_ns':20_000_000_000},
        {'status_stamp':20.},
        {'status_stamp':19.9},
        {'status_stamp':20.3},
        {'clock_ns':19_000_000_000,'status_stamp':20.}))
    result=module.drain_task_owner('s',use_sim_time=True)
    assert result['request_sent'] and result['request_accepted']
    assert not result['software_retired'] and not result['physical_stop_confirmed']
    assert result['reason']=='task_owner_drain_clock_regressed'
    assert result['timeout_stage']=='clock_regressed'
    assert result['requested_source_ns']==20_000_000_000
    assert [(o['stamp'],o['accepted']) for o in events.observations]==[(19.9,False),(20.3,False)]
    assert_fake_shutdown_cleaned_up(events)


def test_simulated_regression_before_request_prevents_sending(monkeypatch):
    _,events=fake_shutdown_ros(monkeypatch,service_ready=False,steps=(
        {'clock_ns':20_000_000_000},{'clock_ns':19_000_000_000}))
    result=module.drain_task_owner('s',use_sim_time=True)
    assert not result['request_sent'] and not result['request_accepted'] and not events.requests
    assert result['timeout_stage']=='clock_regressed' and not result['software_retired']
    assert_fake_shutdown_cleaned_up(events)


@pytest.mark.parametrize('ready,response,stage,sent,accepted',[
    (False,True,'service',False,False),
    (True,False,'response',True,False),
    (True,True,'retirement',True,True)])
def test_timeout_stage_preserves_original_six_second_wall_budget(monkeypatch,ready,response,stage,sent,accepted):
    _,events=fake_shutdown_ros(monkeypatch,initial_source=20.,service_ready=ready,response_ready=response)
    result=module.drain_task_owner('s')
    assert result['timeout_stage']==stage and result['reason']=='task_owner_drain_timeout'
    assert result['request_sent'] is sent and result['request_accepted'] is accepted
    assert not result['software_retired'] and 6.<=result['elapsed_s']<6.051
    assert_fake_shutdown_cleaned_up(events)


@pytest.mark.parametrize('sec,nanosec',[(0,0),(-1,0),(2147483648,0),(True,0),(1.,0),
    (1,-1),(1,1000000000),(1,False),(1,1.)])
def test_original_clock_rejects_invalid_stamp_without_inventing_source(sec,nanosec):
    original=OriginalClockSource()
    assert not original.observe(SimpleNamespace(clock=SimpleNamespace(sec=sec,nanosec=nanosec)))
    assert original.nanoseconds==0


def test_original_clock_retains_exact_integers_but_regression_cannot_recover():
    original=OriginalClockSource()
    def sample(ns):
        return SimpleNamespace(clock=SimpleNamespace(sec=ns//1000000000,nanosec=ns%1000000000))
    ns=1_791_377_450_939_382_334
    assert original.observe(sample(ns)) and original.nanoseconds==ns
    assert original.observe(sample(ns))
    assert not original.observe(sample(ns-1)) and original.fault=='clock_regressed'
    assert not original.observe(sample(ns+1)) and original.nanoseconds==ns


@pytest.mark.parametrize('session_id,budget',[('',6.),('s',0.),('s',10.01)])
def test_shutdown_contract_rejects_empty_identity_or_unbounded_wall_budget(session_id,budget):
    with pytest.raises(ValueError,match='invalid_shutdown_contract'):
        module.drain_task_owner(session_id,budget_s=budget)
