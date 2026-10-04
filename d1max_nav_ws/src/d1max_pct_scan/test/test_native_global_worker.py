"""Pure offline persistent-worker protocol; no ROS node or robot connection."""
import multiprocessing
import threading
import time
import json
from types import SimpleNamespace

import pytest

from d1max_pct_scan.native_global_worker import serve_requests
from d1max_pct_scan.live_global_planner import LiveGlobalPlanner
from d1max_pct_scan.live_goal_pause import RetainedGlobalComputation
from d1max_pct_scan.live_global_contract import RequestEvidence, GlobalPlanError
from d1max_pct_scan.live_global_worker_cleanup import RetiredChildren
from test_live_goal_pause import IDENTITY, INTENT, _localizer, _navigation


class FakePlanner:
    def warmup_resources(self):
        return {'native_map_cache':{'entries':3},'scope':'resources_only'}
    def plan(self, start, goal, start_layer, goal_layer):
        if goal == 'fail':
            raise ValueError('native failed search')
        return dict(path=[start,goal], layer_ids=[start_layer,goal_layer],
                    source_tomogram_sha256='map-a', execution_authorized=False,
                    native_map_cache={'entries':1}, debug_huge_map='not transferred')


def request(generation, goal):
    return dict(kind='plan', generation=generation, start_xyz=[0,0,0],
                goal_xyz=goal, start_layer=0, goal_layer=0)


def receive_result(connection):
    packets = []
    while True:
        assert connection.poll(2.), 'worker did not reply within offline test deadline'
        value = connection.recv()
        packets.append(value)
        if value['kind'] != 'progress':
            return packets


def test_many_goals_share_one_instance_but_never_a_stale_result():
    parent, child = multiprocessing.Pipe()
    built = []
    def build():
        built.append(1)
        return FakePlanner()
    thread = threading.Thread(target=serve_requests, args=(child,build), daemon=True)
    thread.start()
    try:
        for generation in range(1,21):
            goal = [generation,0,0]
            parent.send(request(generation, goal))
            packets = receive_result(parent)
            result = packets[-1]
            assert result['kind'] == 'planned' and result['generation'] == generation
            assert result['worker_reused'] == (generation > 1)
            assert result['result']['path'][-1] == goal
            assert 'debug_huge_map' not in result['result']
            assert packets[0]['phase'] == ('initializing_map' if generation == 1
                                          else 'native_search_and_validation')
        assert len(built) == 1
        parent.send({'kind':'shutdown'})
        thread.join(timeout=2.)
        assert not thread.is_alive()
    finally:
        parent.close()


def test_startup_warmup_has_no_goal_generation_or_route_and_first_plan_reuses():
    parent,child=multiprocessing.Pipe()
    built=[]
    def build():
        built.append(1);return FakePlanner()
    thread=threading.Thread(target=serve_requests,args=(child,build),daemon=True)
    thread.start()
    try:
        parent.send({'kind':'warmup','warmup_id':17})
        assert parent.poll(2.)
        assert parent.recv()==dict(kind='warmup_progress',warmup_id=17,
                                  phase='initializing_native_resources')
        assert parent.poll(2.)
        warmed=parent.recv()
        assert warmed['kind']=='warmed' and warmed['warmup_id']==17
        assert 'generation' not in warmed and 'result' not in warmed
        assert warmed['metrics']['native_map_cache']['entries']==3
        parent.send(request(1,[3,0,0]))
        result=receive_result(parent)[-1]
        assert result['kind']=='planned' and result['generation']==1
        assert result['worker_reused'] is True and len(built)==1
        parent.send({'kind':'shutdown'});thread.join(timeout=2.)
        assert not thread.is_alive()
    finally:
        parent.close()


def test_repeated_warmup_is_not_unbounded_rebuild():
    parent,child=multiprocessing.Pipe()
    thread=threading.Thread(target=serve_requests,args=(child,FakePlanner),daemon=True)
    thread.start()
    try:
        parent.send({'kind':'warmup','warmup_id':1})
        for _ in range(2):
            assert parent.poll(2.);parent.recv()
        parent.send({'kind':'warmup','warmup_id':2})
        assert parent.poll(2.)
        assert parent.recv()['kind']=='warmup_failed'
        thread.join(timeout=2.);assert not thread.is_alive()
    finally:
        parent.close()


def warmup_owner(monkeypatch):
    now=[10.]
    monkeypatch.setattr('d1max_pct_scan.live_global_planner.time.monotonic',lambda:now[0])
    events=[]
    obj=SimpleNamespace(child=None,pipe=None,retired_children=SimpleNamespace(pending=[]),
        warmup={'phase':'not_started','warmup_id':0},p={'warmup_timeout_s':3.},
        current=None,generation=91)
    def spawn(owner):
        events.append('spawn')
        owner.child=SimpleNamespace(is_alive=lambda:True)
        owner.pipe=SimpleNamespace(send=lambda packet:events.append(packet),poll=lambda _:False)
    monkeypatch.setattr(LiveGlobalPlanner,'_spawn_native_worker',spawn)
    def stop():
        events.append('stop');obj.child=obj.pipe=None
    obj.stop_child=stop
    return obj,now,events


def test_warmup_timeout_latches_without_tick_respawn_or_goal_changes(monkeypatch):
    obj,now,events=warmup_owner(monkeypatch)
    assert LiveGlobalPlanner._start_warmup(obj)
    assert obj.generation==91 and events[1]==dict(kind='warmup',warmup_id=1)
    assert not LiveGlobalPlanner._start_warmup(obj)
    now[0]=13.
    LiveGlobalPlanner._poll_warmup(obj)
    assert obj.warmup['phase']=='failed' and events.count('stop')==1
    for _ in range(20):
        LiveGlobalPlanner._poll_warmup(obj)
        assert not LiveGlobalPlanner._start_warmup(obj)
    assert events.count('spawn')==1 and obj.generation==91


def test_exited_worker_final_decode_remains_bounded_by_original_warmup_deadline(monkeypatch):
    obj,now,events=warmup_owner(monkeypatch)
    LiveGlobalPlanner._start_warmup(obj)
    obj.child.is_alive=lambda:False
    obj.pipe.receive_finished=False
    LiveGlobalPlanner._poll_warmup(obj)
    assert obj.warmup['phase']=='running'
    now[0]=13.
    LiveGlobalPlanner._poll_warmup(obj)
    assert obj.warmup['phase']=='failed' and events.count('stop')==1


def test_owner_tick_does_not_retire_warmup_mid_decode_and_retires_failed_idle_channel(monkeypatch):
    obj,now,events=warmup_owner(monkeypatch)
    LiveGlobalPlanner._start_warmup(obj)
    obj.retired_children.reap=lambda:None
    obj._revoke_if_context_lost=lambda:None
    obj.pending_goal=None
    obj.static_validator=SimpleNamespace(poll=lambda:None)
    obj.last_status_at=now[0]
    obj.child.is_alive=lambda:False
    obj.pipe.receive_finished=False
    LiveGlobalPlanner.tick(obj)
    assert obj.child is not None and obj.warmup['phase']=='running'
    obj.warmup['phase']='ready'
    obj.child.is_alive=lambda:True
    obj.pipe.failed=True
    obj.worker_generation=None
    assert not LiveGlobalPlanner.idle_worker_available(obj)
    LiveGlobalPlanner.tick(obj)
    assert obj.child is None and events.count('stop')==1


@pytest.mark.parametrize('token,kind',[(2,'warmed'),(1,'planned'),(1,'warmup_failed')])
def test_warmup_wrong_token_route_packet_or_failure_cannot_be_ready(monkeypatch,token,kind):
    obj,_,events=warmup_owner(monkeypatch);LiveGlobalPlanner._start_warmup(obj)
    obj.pipe=SimpleNamespace(poll=lambda _:True,
        recv=lambda:dict(kind=kind,warmup_id=token,metrics={}))
    LiveGlobalPlanner._poll_warmup(obj)
    assert obj.warmup['phase']=='failed' and obj.child is None and events[-1]=='stop'


@pytest.mark.parametrize('second', [request(1,[2,0,0]),request(2,'fail')])
def test_stale_generation_or_failed_search_closes_worker(second):
    parent, child = multiprocessing.Pipe()
    thread = threading.Thread(target=serve_requests, args=(child,FakePlanner), daemon=True)
    thread.start()
    try:
        parent.send(request(1,[1,0,0]))
        assert receive_result(parent)[-1]['kind'] == 'planned'
        parent.send(second)
        assert receive_result(parent)[-1]['kind'] == 'failed'
        thread.join(timeout=2.)
        assert not thread.is_alive()
    finally:
        parent.close()


def lifecycle_fake(*, busy=False):
    actions = []
    pause = RetainedGlobalComputation()
    pause.install(INTENT)
    obj = SimpleNamespace(pause=pause, generation=5, worker_generation=5 if busy else None,
        child=SimpleNamespace(is_alive=lambda:True), pipe=object(),
        static_validator=SimpleNamespace(invalidate=lambda:None),
        current=object() if busy else None,
        stop_child=lambda:actions.append('terminate_own_child'),
        publish_empty=lambda reason:actions.append('empty'),
        clear_visual_path=lambda:actions.append('clear_visual'))
    obj.idle_worker_available = lambda:LiveGlobalPlanner.idle_worker_available(obj)
    return obj, actions


def test_new_goal_reuses_idle_worker_and_cancels_busy_worker():
    idle, idle_actions = lifecycle_fake()
    LiveGlobalPlanner.revoke(idle, 'new_goal', preserve_idle_worker=True)
    assert idle_actions == ['empty','clear_visual']
    busy, busy_actions = lifecycle_fake(busy=True)
    LiveGlobalPlanner.revoke(busy, 'new_goal', preserve_idle_worker=True)
    assert busy_actions == ['terminate_own_child','empty','clear_visual']
    assert busy.current is None and busy.pending_goal is None


@pytest.mark.parametrize('reason',['explicit_user_cancel','hard_fault','map_changed','shutdown'])
def test_explicit_cancel_or_hard_change_destroys_even_idle_worker(reason):
    obj, actions = lifecycle_fake()
    LiveGlobalPlanner.revoke(obj,reason)
    assert actions == ['terminate_own_child','empty','clear_visual']
    assert obj.pause.intent is None


def test_old_generation_or_non_owned_reply_cannot_commit():
    obj = SimpleNamespace(current=RequestEvidence(8,4,'seed-a',100.,(0,0,.5),10.),
        worker_generation=8,
        revoke=lambda reason:pytest.fail('stale result touched current worker'))
    LiveGlobalPlanner._finish(obj,dict(kind='planned',generation=7,result={}))
    assert obj.current.generation == 8
    obj.worker_generation = 9
    LiveGlobalPlanner._finish(obj,dict(kind='planned',generation=8,result={}))
    assert obj.current.generation == 8


def test_slow_ui_valid_false_does_not_block_fresh_source_pose(monkeypatch):
    monkeypatch.setattr('d1max_pct_scan.live_global_planner.time.monotonic',lambda:10.)
    pose = dict(schema=1,epoch=4,seed_id='seed-a',frame_id='d1max_loc_map',
        body_frame='d1max_loc_base_link',valid=True,pose_valid=True,reset_pending=False,
        fault=None,motion_control_enabled=False,pose_timeout_sec=.08,
        received_at_unix=100.,output_stamp_sec=100.)
    obj = SimpleNamespace(now_s=lambda:100.03,pose_received=10.,body_received=10.,
        body=[0,0,.5],body_stamp=100.,body_context=IDENTITY,p={'freshness_s':.5},pose_status=pose,
        navigation=dict(valid=False),confirmed_identity=lambda:IDENTITY,
        bridge=SimpleNamespace(source_frame='d1max_loc_map'))
    assert LiveGlobalPlanner.planning_context(obj) == (4,'seed-a')
    pose['output_stamp_sec'] = 99.9
    with pytest.raises(GlobalPlanError,match='continuous_pose_invalid'):
        LiveGlobalPlanner.planning_context(obj)


def test_next_goal_uses_idle_child_without_spawning_another():
    obj, _ = lifecycle_fake()
    sent = []
    obj.pipe = SimpleNamespace(send=sent.append)
    obj.pending_goal = ({'xyz':[0,0,0],'layer_id':0},{'xyz':[2,0,0],'layer_id':0},6)
    obj.current = RequestEvidence(6,4,'seed-a',100.,(0,0,.5),10.)
    obj.retired_children = SimpleNamespace(reap=lambda:None,pending=[])
    obj.goal_deadline_monotonic = float('inf')
    obj.goal_started_monotonic = 10.
    obj.tomogram = SimpleNamespace(sha256='hash-a')
    obj.localizer, obj.navigation = _localizer(), _navigation()
    obj.revoke = lambda reason:pytest.fail(reason)
    obj.mp = SimpleNamespace(Process=lambda **kw:pytest.fail('unnecessary process spawn'))
    LiveGlobalPlanner._maybe_launch_pending(obj)
    assert len(sent) == 1 and sent[0]['generation'] == 6
    assert obj.worker_generation == 6 and obj.pending_goal is None


def test_pending_goal_waits_for_retired_child_without_overlap():
    obj = SimpleNamespace(pending_goal=object(),
        retired_children=SimpleNamespace(reap=lambda:None,pending=[object()]))
    LiveGlobalPlanner._maybe_launch_pending(obj)
    assert obj.state == 'waiting_for_native_worker_cleanup'


def test_success_retains_idle_worker_but_error_does_not():
    obj, actions = lifecycle_fake(busy=True)
    obj.current = RequestEvidence(5,4,'seed-a',100.,(0,0,.5),10.)
    obj.localizer,obj.navigation = _localizer(),_navigation()
    obj.goal_deadline_monotonic = float('inf')
    obj.goal_started_monotonic = 10.
    jobs = []
    obj.static_validator = SimpleNamespace(submit=lambda generation,call:jobs.append((generation,call)))
    obj.tomogram = SimpleNamespace(sha256='hash-a')
    obj.bridge = object()
    obj.source_route_builder = object()
    obj.pose_status = _navigation()
    obj._commit_cached_result = lambda:actions.append('commit')
    obj.revoke = lambda reason:actions.append('revoke')
    LiveGlobalPlanner._finish(obj,dict(kind='planned',generation=5,
                                      result={'native_map_cache':{'entries':1}}))
    assert obj.current is not None and obj.worker_generation is None
    assert jobs[0][0] == 5 and actions == []
    LiveGlobalPlanner._finish_validation(obj,(5,{'source_tomogram_sha256':'hash-a'},None,.14))
    assert obj.current is None
    assert actions == ['commit']
    assert obj.idle_worker_available()
    obj.current = RequestEvidence(6,4,'seed-a',100.,(0,0,.5),10.)
    obj.worker_generation = 6
    LiveGlobalPlanner._finish(obj,dict(kind='failed',generation=6,error='native failure'))
    assert actions[-1] == 'revoke'


class SlowPlanner:
    def plan(self,*args):
        time.sleep(10.)
        return {}


def test_busy_real_process_is_terminated_and_reaped_without_waiting_for_plan():
    context = multiprocessing.get_context('spawn')
    parent, child = context.Pipe()
    process = context.Process(target=serve_requests,args=(child,SlowPlanner),daemon=True)
    process.start()
    child.close()
    retired = RetiredChildren(terminate_grace_s=.1)
    try:
        parent.send(request(1,[1,0,0]))
        assert parent.poll(3.)
        assert parent.recv()['kind'] == 'progress'
        began = time.monotonic()
        retired.retire(process,parent)
        assert time.monotonic()-began < .2
        retired.drain_on_shutdown(timeout_s=1.)
        assert not process.is_alive() and not retired.pending
        assert time.monotonic()-began < 2.
    finally:
        if process.is_alive():
            process.kill()
        process.join(timeout=1.)
        parent.close()


def test_pose_monotonic_subscription_age_uses_advertised_ttl(monkeypatch):
    monkeypatch.setattr('d1max_pct_scan.live_global_planner.time.monotonic',lambda:10.09)
    pose = dict(schema=1,epoch=4,seed_id='seed-a',frame_id='d1max_loc_map',
        body_frame='d1max_loc_base_link',valid=True,pose_valid=True,reset_pending=False,
        fault=None,motion_control_enabled=False,pose_timeout_sec=.08,
        received_at_unix=100.,output_stamp_sec=100.)
    obj = SimpleNamespace(now_s=lambda:100.02,pose_received=10.,body_received=10.08,
        body=[0,0,.5],body_stamp=100.,body_context=IDENTITY,p={'freshness_s':.5},pose_status=pose,
        navigation={},confirmed_identity=lambda:IDENTITY,bridge=SimpleNamespace(source_frame='d1max_loc_map'))
    with pytest.raises(GlobalPlanError,match='continuous_pose_invalid_or_stale'):
        LiveGlobalPlanner.planning_context(obj)


@pytest.mark.parametrize('bad', [{'received_at_unix':1e10}, {'pose_timeout_sec':10.},
                                {'output_stamp_sec':float('nan')}, {'schema':True}])
def test_bad_pose_packet_withdraws_authority_without_poisoning_watermark(bad):
    valid = dict(schema=1,epoch=4,seed_id='seed-a',frame_id='d1max_loc_map',
        body_frame='d1max_loc_base_link',valid=True,pose_valid=True,reset_pending=False,
        fault=None,motion_control_enabled=False,pose_timeout_sec=.08,
        received_at_unix=100.,output_stamp_sec=100.)
    obj = SimpleNamespace(now_s=lambda:100.02,p={'freshness_s':.5},pose_status={},
        bridge=SimpleNamespace(source_frame='d1max_loc_map'),
        pose_watermark=-float('inf'),_revoke_if_context_lost=lambda:None)
    LiveGlobalPlanner.on_pose_status(obj,SimpleNamespace(data=json.dumps(valid)))
    assert obj.pose_watermark == 100.
    LiveGlobalPlanner.on_pose_status(obj,SimpleNamespace(data=json.dumps({**valid,**bad})))
    assert obj.pose_status == {} and obj.pose_watermark == 100.
    LiveGlobalPlanner.on_pose_status(obj,SimpleNamespace(data=json.dumps(
        {**valid,'received_at_unix':100.01,'output_stamp_sec':100.01})))
    assert obj.pose_status['valid'] is True and obj.pose_watermark == 100.01


@pytest.mark.parametrize('source,stamp_key,method',[
    ('navigation','received_at_unix',LiveGlobalPlanner.on_navigation),
    ('localizer','wall_time',LiveGlobalPlanner.on_localizer)])
@pytest.mark.parametrize('bad_stamp',[1e10,float('nan'),True,99.])
def test_slow_status_bad_time_does_not_poison_independent_watermark(source,stamp_key,method,bad_stamp):
    obj = SimpleNamespace(now_s=lambda:100.02,p={'freshness_s':.5},navigation={},localizer={},
        nav_watermark=-float('inf'),localizer_watermark=-float('inf'),
        _revoke_if_context_lost=lambda:None)
    field = 'nav_watermark' if source == 'navigation' else 'localizer_watermark'
    method(obj,SimpleNamespace(data=json.dumps({stamp_key:100.})))
    assert getattr(obj,field) == 100.
    method(obj,SimpleNamespace(data=json.dumps({stamp_key:bad_stamp})))
    assert getattr(obj,source) == {} and getattr(obj,field) == 100.
    method(obj,SimpleNamespace(data=json.dumps({stamp_key:100.01})))
    assert getattr(obj,source)[stamp_key] == 100.01


def test_explicit_slow_reset_revokes_even_before_next_compact_pose_arrives():
    obj = SimpleNamespace(now_s=lambda:100.,navigation={'reset_pending':True})
    with pytest.raises(GlobalPlanError,match='explicit_fault_or_reset'):
        LiveGlobalPlanner.planning_context(obj)
