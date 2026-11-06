"""No ROS initialization: exact production RViz callback/transaction bodies."""
from copy import deepcopy
from types import SimpleNamespace
from unittest.mock import Mock
import pytest
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseWithCovarianceStamped
from d1max_navigation_bt_interfaces.msg import InitialPoseOutcome
from d1max_navigation_bt_interfaces.srv import PrepareInitialPose
from d1max_pct_scan.initial_pose_client import InitialPoseClient, TRANSACTION
from d1max_pct_scan.live_view import LiveView


def request():
    r=PrepareInitialPose.Request(schema_version=1,session_id='session',request_id='intent',
        source_map_sha256='a'*64)
    r.body_pose=PoseWithCovarianceStamped();r.body_pose.header.frame_id='d1max_loc_map'
    r.body_pose.header.stamp=Time(sec=100);r.body_pose.pose.pose.orientation.w=1.
    r.body_pose.pose.pose.position.z=.5
    return r


def prepared():
    client=InitialPoseClient();r=request()
    client.begin(r,dict(id='intent',session_id='session',created_at=100.,z=.5),10.)
    return client,r


def reply(**changes):
    defaults=dict(schema_version=1,accepted=True,ready=False,application_authorized=False,reason='waiting')
    defaults.update(changes)
    return PrepareInitialPose.Response(**defaults)


def test_prepare_does_not_publish_seed_even_if_retired():
    c,r=prepared();assert c.receive(r,reply(ready=True),10.1) is None
    assert c.pending is not None and c.operation==r.COMMIT


def test_exact_commit_releases_once_with_original_intent_stamp():
    c,r=prepared();c.receive(r,reply(ready=True),10.1);commit=c.request(10.2)
    seed=c.receive(commit,reply(ready=True,application_authorized=True),10.3)
    assert seed['initial_pose_transaction']==TRANSACTION
    assert seed['intent_source_stamp_ns']=='100000000000' and seed['created_at']==100.
    assert seed['source_map_sha256']=='a'*64
    assert c.receive(commit,reply(ready=True,application_authorized=True),10.4) is None


@pytest.mark.parametrize('change',['identity','pose','operation'])
def test_foreign_or_late_reply_cannot_submit(change):
    c,r=prepared();c.receive(r,reply(ready=True),10.1);commit=c.request(10.2)
    if change=='identity':commit.request_id='other'
    if change=='pose':commit.body_pose.pose.pose.position.x=.001
    if change=='operation':commit.operation=commit.PREPARE
    assert c.receive(commit,reply(ready=True,application_authorized=True),10.3) is None
    assert c.pending is not None


def test_expired_draft_cannot_apply_late_grant():
    c,r=prepared();c.receive(r,reply(ready=True),10.1);commit=c.request(10.2)
    assert c.receive(commit,reply(ready=True,application_authorized=True),20.) is None
    assert c.pending is None


def newer(r, identifier='other'):
    other=deepcopy(r);other.request_id=identifier
    other.body_pose.header.stamp.nanosec+=1
    return other


def test_latest_intent_replaces_one_pending_even_at_identical_coordinates():
    c,r=prepared()
    for number in range(20):
        r=newer(r,'intent-'+str(number))
        c.begin(r,dict(id=r.request_id,session_id='session'),10.1)
        assert c.pending[0].request_id==r.request_id
        assert c.pending[0].body_pose.pose==request().body_pose.pose
    assert c.operation==r.PREPARE


@pytest.mark.parametrize('invalid',['clock','binding','source','id'])
def test_invalid_replacement_preserves_the_existing_pending(invalid):
    c,r=prepared();other=newer(r);stamp=10.1
    command=dict(id='other',session_id='session')
    if invalid=='clock':stamp=float('nan')
    if invalid=='binding':command['id']='wrong'
    if invalid=='source':other.body_pose.header.stamp=r.body_pose.header.stamp
    if invalid=='id':other.request_id=command['id']='intent'
    with pytest.raises(ValueError):c.begin(other,command,stamp)
    assert c.pending[0]==r


@pytest.mark.parametrize('operation',[0,1])
@pytest.mark.parametrize('accepted',[False,True])
def test_superseded_reply_and_error_cannot_clear_fail_or_expire_latest(operation,accepted):
    c,r=prepared()
    if operation==r.COMMIT:c.receive(r,reply(ready=True),10.1)
    old=c.request(10.2);latest=newer(r)
    c.begin(latest,dict(id='other',session_id='session'),11.)
    before=deepcopy(c.pending)
    assert c.receive(old,reply(accepted=accepted,ready=True,
        application_authorized=operation==r.COMMIT,reason='old_rejection'),100.) is None
    c.fail('old_transport_failure',request=old)
    assert c.pending==before and c.phase=='waiting_retirement'
    assert c.reason=='等待旧任务退役'


def harness(tmp_path,monkeypatch):
    from d1max_pct_scan import live_view
    monkeypatch.setattr(live_view.time,'monotonic',lambda:10.)
    monkeypatch.setattr(live_view.time,'time',lambda:123.)
    n=SimpleNamespace(single_floor=True,session=dict(id='session',version_id='map',initial_pose_transaction=TRANSACTION,
        map_pcd='/map.pcd',input_hashes={'/map.pcd':'a'*64},body_height=.5),
        directory=tmp_path,last_state={},state_at=10.,initial_floor='floor1',
        initial_ground_bridge=Mock(),initial_pose_client=Mock(),initial_pose_transaction=InitialPoseClient(),
        initial_pose_future=None,initial_pose_rpc_at=0.,initial_pose_request=None,
        cancel_pub=Mock(),get_logger=lambda:Mock(),get_clock=lambda:SimpleNamespace(
            now=lambda:SimpleNamespace(nanoseconds=100000000000)))
    monkeypatch.setattr(live_view,'floor_initial_body_z',lambda *a:.5)
    n.initial_pose_client.call_async.side_effect=lambda _:Mock(**{'done.return_value':False})
    return n


def test_actual_rviz_valid_draft_only_enters_owner_handshake(tmp_path,monkeypatch):
    n=harness(tmp_path,monkeypatch);r=request();LiveView.initial(n,r.body_pose)
    n.cancel_pub.publish.assert_not_called()
    assert not (tmp_path/'initial_pose.json').exists()
    assert n.initial_pose_transaction.pending is not None
    assert n.initial_pose_transaction.pending[0].body_pose.header.stamp==r.body_pose.header.stamp


@pytest.mark.parametrize('operation',[0,1])
def test_actual_new_input_detaches_old_rpc_and_sends_latest_without_timeout(tmp_path,monkeypatch,operation):
    n=harness(tmp_path,monkeypatch);r=request();LiveView.initial(n,r.body_pose)
    LiveView.poll_initial_pose(n)
    if operation==r.COMMIT:
        n.initial_pose_future.done.return_value=True
        n.initial_pose_future.result.return_value=reply(ready=True)
        LiveView.poll_initial_pose(n)
    old_future,old_request=n.initial_pose_future,n.initial_pose_request
    old_future.done.return_value=False
    latest=newer(r)
    LiveView.initial(n,latest.body_pose)
    new_id=n.initial_pose_transaction.pending[0].request_id
    assert new_id!=old_request.request_id
    assert n.initial_pose_transaction.pending[0].body_pose.pose==old_request.body_pose.pose
    assert n.initial_pose_future is None
    n.initial_pose_client.remove_pending_request.assert_called_with(old_future)
    calls=n.initial_pose_client.call_async.call_count
    LiveView.poll_initial_pose(n)
    assert n.initial_pose_client.call_async.call_count==calls+1
    assert n.initial_pose_request.request_id==new_id
    assert n.initial_pose_request.operation==r.PREPARE
    assert not (tmp_path/'initial_pose.json').exists()


def finish_rpc(n,response):
    n.initial_pose_future.done.return_value=True
    n.initial_pose_future.result.return_value=response
    LiveView.poll_initial_pose(n)


def test_actual_replacement_writes_only_latest_commit_with_original_new_source(tmp_path,monkeypatch):
    import json
    n=harness(tmp_path,monkeypatch);r=request();LiveView.initial(n,r.body_pose)
    LiveView.poll_initial_pose(n);finish_rpc(n,reply(ready=True))
    abandoned=n.initial_pose_future;old_id=n.initial_pose_request.request_id
    LiveView.initial(n,newer(r).body_pose)
    abandoned.done.return_value=True
    abandoned.result.return_value=reply(ready=True,application_authorized=True)
    LiveView.poll_initial_pose(n);new_id=n.initial_pose_request.request_id
    finish_rpc(n,reply(ready=True))
    assert not (tmp_path/'initial_pose.json').exists()
    finish_rpc(n,reply(ready=True,application_authorized=True))
    command=json.loads((tmp_path/'initial_pose.json').read_text())
    assert command['id']==new_id and new_id!=old_id
    assert command['intent_source_stamp_ns']=='100000000001'
    assert n.initial_pose_transaction.submitted and n.initial_pose_transaction.pending is None
    abandoned.result.assert_not_called()
    n.cancel_pub.publish.assert_not_called()


@pytest.mark.parametrize('fails',[False,True])
def test_actual_old_result_reentrant_new_input_cannot_write_or_fail_latest(tmp_path,monkeypatch,fails):
    n=harness(tmp_path,monkeypatch);r=request();LiveView.initial(n,r.body_pose)
    LiveView.poll_initial_pose(n)
    n.initial_pose_future.done.return_value=True
    n.initial_pose_future.result.return_value=reply(ready=True)
    LiveView.poll_initial_pose(n)
    old=n.initial_pose_future;old.done.return_value=True
    def result():
        LiveView.initial(n,newer(r).body_pose)
        if fails:raise RuntimeError('late old RPC failure')
        return reply(ready=True,application_authorized=True)
    old.result.side_effect=result
    # Fresh transport futures, rather than reusing Mock's single return object.
    n.initial_pose_client.call_async.side_effect=lambda _:Mock()
    LiveView.poll_initial_pose(n)
    assert n.initial_pose_transaction.pending is not None
    assert n.initial_pose_transaction.phase=='waiting_retirement'
    assert n.initial_pose_request.operation==r.PREPARE
    assert n.initial_pose_request.body_pose.header.stamp.nanosec==1
    assert not (tmp_path/'initial_pose.json').exists()


def test_invalid_new_input_does_not_detach_valid_current_rpc(tmp_path,monkeypatch):
    n=harness(tmp_path,monkeypatch);r=request();LiveView.initial(n,r.body_pose)
    LiveView.poll_initial_pose(n);old=n.initial_pose_future
    before=deepcopy(n.initial_pose_transaction.pending)
    bad=newer(r).body_pose;bad.pose.pose.orientation.w=0.
    LiveView.initial(n,bad)
    assert n.initial_pose_future is old and n.initial_pose_transaction.pending==before
    n.initial_pose_client.remove_pending_request.assert_not_called()
    assert n.initial_pose_transaction.phase=='rejected'


def test_service_disappearance_expires_independent_feedback_not_generic_goal_reason(tmp_path,monkeypatch):
    from d1max_pct_scan import live_view
    n=harness(tmp_path,monkeypatch);n.reason='目标已提交，等待规划'
    n.initial_pose_client.service_is_ready.return_value=False
    LiveView.initial(n,request().body_pose)
    LiveView.poll_initial_pose(n)
    assert n.initial_pose_transaction.phase=='waiting_service'
    monkeypatch.setattr(live_view.time,'monotonic',lambda:20.)
    LiveView.poll_initial_pose(n)
    assert n.initial_pose_transaction.pending is None
    assert n.initial_pose_transaction.feedback()['phase']=='timeout'
    assert '过期' in n.initial_pose_transaction.feedback()['reason']
    assert n.reason=='目标已提交，等待规划'


def test_rpc_timeout_can_immediately_retry_with_fresh_input(tmp_path,monkeypatch):
    from d1max_pct_scan import live_view
    n=harness(tmp_path,monkeypatch);r=request();LiveView.initial(n,r.body_pose)
    LiveView.poll_initial_pose(n);old=n.initial_pose_future
    monkeypatch.setattr(live_view.time,'monotonic',lambda:12.1)
    LiveView.poll_initial_pose(n)
    assert n.initial_pose_transaction.phase=='timeout' and n.initial_pose_future is None
    n.initial_pose_client.remove_pending_request.assert_called_with(old)
    LiveView.initial(n,newer(r).body_pose);LiveView.poll_initial_pose(n)
    assert n.initial_pose_request.operation==r.PREPARE
    assert n.initial_pose_transaction.phase=='waiting_retirement'


def outcome(r,**changes):
    defaults=dict(schema_version=1,session_id=r.session_id,request_id=r.request_id,
        map_version_id='map',intent_source_stamp=r.body_pose.header.stamp,
        source_stamp=Time(sec=100,nanosec=100000000),localization_epoch=2,
        localization_seed_id='new-seed',accepted=True,applied=True,
        reason='initial_pose_seed_installed_not_localization_ready')
    defaults.update(changes)
    return InitialPoseOutcome(**defaults)


def submitted():
    c,r=prepared();c.receive(r,reply(ready=True),10.1);commit=c.request(10.2)
    c.receive(commit,reply(ready=True,application_authorized=True),10.3)
    c.mark_submitted(commit)
    return c,r


def test_feedback_only_completes_for_exact_outcome_and_new_fresh_ready_identity():
    c,r=submitted()
    c.observe_ready(('session',1,'old-seed'),ready=True,source_ns=100200000000)
    assert c.phase=='awaiting_localization'
    assert c.observe_outcome(outcome(r),map_version_id='map',now_ns=100200000000)
    for identity,ready,stamp in [(('session',1,'old-seed'),True,100200000000),
                                  (('session',2,'new-seed'),False,100200000000),
                                  (('session',2,'new-seed'),True,100000000000)]:
        c.observe_ready(identity,ready=ready,source_ns=stamp)
        assert c.phase=='awaiting_localization'
    c.observe_ready(('session',2,'new-seed'),ready=True,source_ns=100200000000)
    assert c.feedback() is None
    assert not c.observe_outcome(outcome(r,source_stamp=Time(sec=100,nanosec=200000000)),
        map_version_id='map',now_ns=100200000000)
    assert c.feedback() is None


def test_late_exact_application_fact_keeps_original_time_and_needs_fresh_ready_identity():
    c,r=submitted();message=outcome(r);original=deepcopy(message)
    assert c.observe_outcome(message,map_version_id='map',now_ns=200000000000)
    assert message==original and c.outcome_source_ns==100100000000
    # A durable application acknowledgement is not a current pose lease.
    c.observe_ready(('session',2,'new-seed'),ready=False,source_ns=100200000000)
    c.observe_ready(('session',1,'old-seed'),ready=True,source_ns=200000000000)
    assert c.phase=='awaiting_localization'
    assert not c.observe_outcome(outcome(r,request_id='old-request'),
        map_version_id='map',now_ns=200000000000)
    assert c.phase=='awaiting_localization'
    c.observe_ready(('session',2,'new-seed'),ready=True,source_ns=200000000000)
    assert c.feedback() is None and c.outcome_source_ns==100100000000


@pytest.mark.parametrize('changes',[
    dict(schema_version=2),dict(session_id='other'),dict(request_id='old'),
    dict(map_version_id='other'),dict(intent_source_stamp=Time(sec=99)),
    dict(source_stamp=Time(sec=99)),dict(source_stamp=Time(sec=101)),
    dict(localization_epoch=0),dict(localization_seed_id='')])
def test_foreign_stale_future_or_invalid_outcome_cannot_acknowledge_current_seed(changes):
    c,r=submitted()
    assert not c.observe_outcome(outcome(r,**changes),map_version_id='map',now_ns=100700000000)
    assert c.applied_identity is None and c.phase=='awaiting_localization'


def test_old_outcome_after_replacement_is_ignored_and_exact_rejection_is_visible():
    c,r=submitted();latest=newer(r)
    c.begin(latest,dict(id='other',session_id='session'),11.)
    assert not c.observe_outcome(outcome(r),map_version_id='map',now_ns=100200000000)
    assert c.pending[0]==latest
    c.receive(latest,reply(ready=True),11.1);commit=c.request(11.2)
    c.receive(commit,reply(ready=True,application_authorized=True),11.3);c.mark_submitted(commit)
    assert c.observe_outcome(outcome(latest,accepted=False,applied=False,reason='sensor_not_ready'),
        map_version_id='map',now_ns=100200000000)
    assert c.feedback()['phase']=='rejected' and 'sensor_not_ready' in c.feedback()['reason']


def test_actual_tick_shows_independent_feedback_then_matching_typed_completion(tmp_path,monkeypatch):
    import json
    from d1max_pct_scan.typed_view_state import TypedDisplayContext
    n=harness(tmp_path,monkeypatch)
    n.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(
        nanoseconds=100200000000,to_msg=lambda:Time(sec=100,nanosec=200000000)))
    n.frame='d1max_loc_map';n.freeze=None;n.last_seed_id=None
    n.global_state,n.scan_state,n.bt_state={},{},{}
    n.global_at=n.scan_at=n.bt_at=0.
    n.reason,n.reason_scope='目标操作失败，与定位无关','input'
    n.expire_requests=Mock();n.marker=Mock();n.publish_goal_editor=Mock()
    n.last_snapshot_at=10.
    n.typed_view=SimpleNamespace(state=SimpleNamespace(source_ns=100200000000),
        display_context=lambda **_:TypedDisplayContext(('session',1,'old-seed'),True))
    n.diagnostics_pub=Mock()
    def row():
        LiveView.tick(n)
        return json.loads(n.diagnostics_pub.publish.call_args.args[0].data)['stages']['localization']
    LiveView.initial(n,request().body_pose)
    assert row()['label']=='初值待退役'
    assert n.reason not in row()['detail']
    LiveView.poll_initial_pose(n);finish_rpc(n,reply(ready=True))
    assert row()['label']=='初值提交中'
    committed=deepcopy(n.initial_pose_request)
    finish_rpc(n,reply(ready=True,application_authorized=True))
    assert row()['label']=='初值已提交 · 等待定位'
    # A successful prior pose must not acknowledge the newer request.
    LiveView.on_initial_pose_outcome(n,outcome(committed))
    assert row()['label']=='初值已提交 · 等待定位'
    n.typed_view.display_context=lambda **_:TypedDisplayContext(('session',2,'new-seed'),True)
    assert row()['label']=='已定位'
    n.cancel_pub.publish.assert_not_called()


def test_actual_service_rejection_feedback_is_reportable_and_preserves_goal_notice(tmp_path,monkeypatch):
    n=harness(tmp_path,monkeypatch);n.reason='目标已提交，等待规划'
    LiveView.initial(n,request().body_pose);LiveView.poll_initial_pose(n)
    finish_rpc(n,reply(accepted=False,reason='initial_pose_retirement_pending'))
    assert n.initial_pose_transaction.phase=='waiting_retirement'
    finish_rpc(n,reply(accepted=False,reason='navigation_lifecycle_not_active'))
    assert n.initial_pose_transaction.feedback()==dict(session_id='session',phase='rejected',
        reason='navigation_lifecycle_not_active')
    assert n.reason=='目标已提交，等待规划'


@pytest.mark.parametrize('bad',['frame','orientation','covariance','stamp','version'])
def test_actual_rviz_invalid_draft_never_cancels_or_writes(tmp_path,monkeypatch,bad):
    n=harness(tmp_path,monkeypatch);r=request();p=r.body_pose
    if bad=='frame':p.header.frame_id='other'
    if bad=='orientation':p.pose.pose.orientation.w=0.
    if bad=='covariance':p.pose.covariance[7]=-1.
    if bad=='stamp':p.header.stamp.sec=95
    if bad=='version':n.session.pop('initial_pose_transaction')
    LiveView.initial(n,p)
    n.cancel_pub.publish.assert_not_called();n.initial_pose_client.call_async.assert_not_called()
    assert not (tmp_path/'initial_pose.json').exists() and n.initial_pose_transaction.pending is None


def test_actual_rviz_mailbox_waits_matching_commit_not_prepare(tmp_path,monkeypatch):
    n=harness(tmp_path,monkeypatch);r=request();LiveView.initial(n,r.body_pose)
    LiveView.poll_initial_pose(n)
    prepare=n.initial_pose_request
    n.initial_pose_future=Mock();n.initial_pose_future.done.return_value=True
    n.initial_pose_future.result.return_value=reply(ready=True)
    LiveView.poll_initial_pose(n)
    assert not (tmp_path/'initial_pose.json').exists()
    assert n.initial_pose_request.operation==r.COMMIT
    n.initial_pose_future=Mock();n.initial_pose_future.done.return_value=True
    n.initial_pose_future.result.return_value=reply(ready=True,application_authorized=True)
    LiveView.poll_initial_pose(n)
    import json
    command=json.loads((tmp_path/'initial_pose.json').read_text())
    assert command['created_at']==123. and command['intent_source_stamp_ns']=='100000000000'
    assert prepare.body_pose.header.stamp.sec==100
    n.cancel_pub.publish.assert_not_called()
