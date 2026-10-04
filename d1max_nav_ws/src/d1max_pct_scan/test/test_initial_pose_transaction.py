"""No ROS initialization: exact production RViz callback/transaction bodies."""
from copy import deepcopy
from types import SimpleNamespace
from unittest.mock import Mock
import pytest
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseWithCovarianceStamped
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


def test_second_draft_does_not_supersede_prepared_intent():
    c,r=prepared();other=deepcopy(r);other.request_id='other'
    with pytest.raises(ValueError):c.begin(other,dict(id='other',session_id='session'),10.1)
    assert c.pending[0].request_id=='intent'


def harness(tmp_path,monkeypatch):
    from d1max_pct_scan import live_view
    monkeypatch.setattr(live_view.time,'monotonic',lambda:10.)
    monkeypatch.setattr(live_view.time,'time',lambda:123.)
    n=SimpleNamespace(single_floor=True,session=dict(id='session',initial_pose_transaction=TRANSACTION,
        map_pcd='/map.pcd',input_hashes={'/map.pcd':'a'*64},body_height=.5),
        directory=tmp_path,last_state={},state_at=10.,initial_floor='floor1',
        initial_ground_bridge=Mock(),initial_pose_client=Mock(),initial_pose_transaction=InitialPoseClient(),
        initial_pose_future=None,initial_pose_rpc_at=0.,initial_pose_request=None,
        cancel_pub=Mock(),get_logger=lambda:Mock(),get_clock=lambda:SimpleNamespace(
            now=lambda:SimpleNamespace(nanoseconds=100000000000)))
    monkeypatch.setattr(live_view,'floor_initial_body_z',lambda *a:.5)
    return n


def test_actual_rviz_valid_draft_only_enters_owner_handshake(tmp_path,monkeypatch):
    n=harness(tmp_path,monkeypatch);r=request();LiveView.initial(n,r.body_pose)
    n.cancel_pub.publish.assert_not_called()
    assert not (tmp_path/'initial_pose.json').exists()
    assert n.initial_pose_transaction.pending is not None
    assert n.initial_pose_transaction.pending[0].body_pose.header.stamp==r.body_pose.header.stamp


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
