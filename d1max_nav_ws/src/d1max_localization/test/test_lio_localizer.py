import json
import queue
import time
from collections import deque
from unittest.mock import Mock
import pytest
import rclpy
from std_msgs.msg import String
from d1max_localization.lio_localizer import LioLocalizer, navigation_tracking_state
from d1max_localization.lio_fusion import LioMapState
from d1max_localization.math_utils import Pose3
from d1max_localization.startup_relocalization import StartupRelocalization
from d1max_localization.global_registration import RegistrationConfig
from d1max_localization.global_lio_localizer import GlobalLioLocalizer


@pytest.fixture
def node(tmp_path):
    # Exercise callbacks without constructing ROS endpoints or any robot connection.
    n=LioLocalizer.__new__(LioLocalizer)
    n.core=LioMapState();n.now_s=Mock(return_value=100.)
    n.get_clock=Mock();n.get_clock.return_value.now.return_value=rclpy.time.Time(seconds=100.)
    n.p={'odom_frame':'odom','tracking_frame':'tracking','map_frame':'map','trajectory_rate_hz':5.,
         'tracking_offset_body':[0.,0.,0.],'sdk_to_tracking_yaw':0.,'body_frame':'body'}
    n.navigation_pose={};n.navigation_pose_at=0.;n.navigation_pose_received=-float('inf')
    n.frontend={};n.frontend_at=0.;n.robot_at=100.;n.head_direction=1
    n.path=deque();n.pose_times=deque();n.active_seed='old';n.confirmed_seed='old';n.verified_confirmations=3
    n.last_output=0.;n.last_path=0.;n.last_command=None;n.command_result=None;n.last_error=''
    n.directory=tmp_path;n.session={'id':'test'};n.epoch_changed_at=0.
    for name in ('path_pub','local_pub','prediction_pub','tf','global_pub','pose_pub','icp_pub','seed_pub'):
        setattr(n,name,Mock())
    return n


def sample(n,epoch=1,valid=True,t=100.,**changes):
    n.now_s.return_value=t
    v={'schema':1,'epoch':epoch,'valid':valid,'fault':False,'reason':'tracking',
       'received_at_unix':t,'stamp_ns':str(round(t*1e9)),'frame':'odom','child_frame':'tracking',
       'position':[0.,0.,0.],'orientation':[0.,0.,0.,1.],'linear':[0.,0.,0.],'angular':[0.,0.,0.],
       'pose_covariance':[.1 if i%7==0 else 0. for i in range(36)],'twist_covariance':[0.]*36}
    v.update(changes);m=String();m.data=json.dumps(v);n.on_local_sample(m)


def status(n,epoch,**changes):
    v={'backend':'faster_lio','epoch':epoch,'ready':True,'fault':False}
    v.update(changes);m=String();m.data=json.dumps(v);n.on_frontend(m)


def test_reset_clears_anchor_path_seed_and_blocks_old_results(node):
    sample(node);status(node,1)
    node.core.seed(Pose3((5.,0.,0.),(0.,0.,0.,1.)),100.)
    sample(node,t=100.1);assert node.core.accept(100.1,Pose3((5.,0.,0.),(0.,0.,0.,1.)),100.1)
    sample(node,t=100.2);assert node.pose_pub.publish.call_count==1
    sample(node,valid=False,t=100.21,fault=True,reason='imu_gap')
    assert not node.seed_ready() and not node.core.tracking(100.21)
    sample(node,epoch=2,valid=False,t=100.22,reason='imu_reinitializing')
    assert node.active_seed is None and node.confirmed_seed is None and not node.path
    assert node.core.anchor is None and node.core.fault is None
    sample(node,epoch=1,t=100.23)
    assert not node.core.local and node.core.local_epoch==2
    sample(node,epoch=2,t=100.3);status(node,2)
    assert node.seed_ready() and not node.core.tracking(100.3)
    assert node.pose_pub.publish.call_count==1
    assert not node.path_pub.publish.call_args.args[0].poses


def test_delayed_status_cannot_poison_or_authorize_current_epoch(node):
    sample(node,epoch=2);status(node,1,fault=True)
    assert not node.front_ready() and node.core.fault is None
    status(node,2);assert node.front_ready()
    status(node,1,fault=True);assert node.front_ready()


def test_fatal_fault_not_cleared_by_same_epoch_samples(node):
    sample(node,valid=False,fault=True,reason='imu_clock_reset')
    sample(node,t=100.1);status(node,1)
    assert node.core.fault=='imu_clock_reset' and not node.seed_ready()
    node.local_pub.publish.assert_not_called()


@pytest.mark.parametrize('changes',[{'epoch':True},{'epoch':-1},{'position':[float('nan'),0.,0.]},
    {'pose_covariance':[0.]},{'frame':'wrong'},{'received_at_unix':90.},{'orientation':[0.,0.,0.,0.]}])
def test_invalid_sample_cannot_change_epoch_or_publish(node,changes):
    sample(node,**changes)
    assert node.core.local_epoch==0
    node.local_pub.publish.assert_not_called()


def test_initial_pose_queued_before_reset_is_rejected(node):
    command={'id':'before-reset','session_id':'test','created_at':time.time()-.1,'reference':'body','x':0.,'y':0.,'z':0.,'yaw':0.}
    (node.directory/'initial_pose.json').write_text(json.dumps(command))
    node.epoch_changed_at=time.time();node.read_command()
    assert not node.command_result['accepted']
    assert '轮次' in node.command_result['message']
    node.seed_pub.publish.assert_not_called()


def test_short_output_wait_is_not_a_filter_reinitialization():
    for reason in ('waiting_local', 'aligned_history_stale', 'waiting_aligned_history', 'filter_stale'):
        assert navigation_tracking_state(dict(valid=False, fault='', state=reason), 's') == (False, 'output_waiting')
    assert navigation_tracking_state(dict(valid=False, state='initializing_filter'), 's') == (False, 'filter_initializing')
    assert navigation_tracking_state(dict(valid=False, reset_pending=True), 's') == (False, 'filter_initializing')
    assert navigation_tracking_state(dict(valid=False, fault='jump'), 's') == (False, 'navigation_fault')
    assert navigation_tracking_state(dict(valid=True, seed_id='s'), 's') == (True, 'tracking')
    assert navigation_tracking_state(dict(valid=True, seed_id='old'), 's') == (False, 'output_waiting')


def test_continuous_pose_uses_verified_navigation_not_intermittent_raw_posterior(node):
    sample(node);status(node,1)
    node.p['navigation_output_enabled']=True
    node.active_seed=node.confirmed_seed='seed';node.verified_confirmations=3
    value=dict(schema=1,epoch=1,seed_id='seed',frame_id='map',body_frame='body',
               received_at_unix=100.,output_stamp_sec=99.98,pose_timeout_sec=.08,
               pose_valid=True,valid=True,fault='',reset_pending=False,quality='tracking')
    node.on_navigation_pose(String(data=json.dumps(value)))
    node.core.stream_valid=False  # newest raw LIO status pending; predictor remains fresh
    assert node.continuous_pose(100.) == (True,'tracking','fresh_continuous_pose')
    node.core.fault='clock_reset'
    assert node.continuous_pose(100.) == (False,'fault','local_fault')
    node.core.fault=None
    node.confirmed_seed='old'
    assert not node.continuous_pose(100.)[0]


def test_delayed_or_future_pose_status_cannot_refresh_receipt(node):
    sample(node)
    good=dict(schema=1,epoch=1,received_at_unix=100.)
    node.on_navigation_pose(String(data=json.dumps(good)))
    receipt=node.navigation_pose_received
    for stamp in (99.9,100.,100.02):
        node.on_navigation_pose(String(data=json.dumps(dict(good,received_at_unix=stamp))))
        assert node.navigation_pose_received == receipt
    node.on_navigation_pose(String(data=json.dumps(dict(good,epoch=0,received_at_unix=100.001))))
    assert node.navigation_pose_received == receipt


def test_status_keeps_matching_and_continuous_pose_and_motion_facts_separate(node):
    # Regression of the 23:11 real-session snapshot: a genuine, fresh 49 Hz
    # navigation stream must not become "lost" because raw LIO/matcher health
    # is between posteriors. Keep legacy localized false, not fabricated true.
    sample(node);status(node,1)
    node.p.update(navigation_output_enabled=True,extrinsics_verified=False,
                  time_alignment_verified=False)
    node.session['version_id']='map-v1'
    node.active_seed=node.confirmed_seed='seed';node.verified_confirmations=3
    node.core.seed_time=90.;node.core.last_correction=99.58
    node.core.tracking=Mock(return_value=False)
    node.core.begin_recovery=Mock(return_value=None)
    node.read_command=Mock();node.publish_alignment=Mock();node.seed_ready=Mock(return_value=True)
    node.imu_at=node.cloud_at=node.sdk_at=node.robot_at=100.
    node.speed_report={};node.speed_report_at=0.
    node.navigation=dict(schema=1,epoch=1,seed_id='seed',valid=True,navigation_ready=False,
                         fault='',global_observed_hz=49.,local_fresh=True)
    node.navigation_at=100.
    node.matcher=node.input_clock={}
    node.status_pub=Mock();node.saved=queue.Queue(maxsize=1)
    pose=dict(schema=1,epoch=1,seed_id='seed',frame_id='map',body_frame='body',
              received_at_unix=100.,output_stamp_sec=99.972,pose_timeout_sec=.08,
              pose_valid=True,valid=True,fault='',reset_pending=False,quality='coasting')
    node.on_navigation_pose(String(data=json.dumps(pose)))
    node.tick()
    result=json.loads(node.status_pub.publish.call_args.args[0].data)
    assert result['localized'] is result['map_localized'] is False
    assert result['continuous_pose_valid'] is result['global_ekf_fresh'] is True
    assert result['state']=='tracking_degraded' and result['matching_state']=='lost'
    assert result['continuous_pose_state']=='coasting'
    assert result['navigation_ready'] is False
    assert result['output_observed_hz']==49.
    assert result['navigation']['valid'] is True
    # An actual source expiration must still pause output status even if the
    # old full navigation diagnostic still says valid.
    node.now_s.return_value=100.061
    node.tick()
    expired=json.loads(node.status_pub.publish.call_args.args[0].data)
    assert expired['continuous_pose_valid'] is expired['global_ekf_fresh'] is False
    assert expired['continuous_pose_reason']=='pose_output_source_stale'


def startup_node(node):
    node.__class__=GlobalLioLocalizer
    node.startup=StartupRelocalization('test')
    node.registration_config=RegistrationConfig()
    node.relocalization_worker=Mock(state='ready')
    node.relocalization_scan=None;node.relocalization_index={}
    node.active_seed=node.confirmed_seed=None;node.verified_confirmations=0
    node.startup.map_sha256='a'*64
    for i in range(9):sample(node,t=99.2+i*.1)
    status(node,1)
    assert node.startup.capture(99.95,100.)
    return node


def test_global_result_uses_single_seed_owner_not_navigation_output_or_tf(node):
    n=startup_node(node)
    transform=[[1,0,0,3],[0,1,0,2],[0,0,1,0],[0,0,0,1]]
    n.relocalization_worker.poll.return_value=dict(kind='result',accepted=True,
        request_id=n.startup.request['request_id'],map_sha256='a'*64,candidate=dict(transform=transform))
    n.poll_relocalization(100.)
    assert n.core.seed_time==100. and n.startup.state=='confirming'
    assert n.active_seed=='100000000000' and n.confirmed_seed is None
    assert n.seed_pub.publish.call_count==1
    assert n.seed_pub.publish.call_args.args[0].pose.pose.position.x==3.
    n.global_pub.publish.assert_not_called();n.tf.sendTransform.assert_not_called()
    assert n.relocalization_worker is None


def test_manual_seed_cancels_worker_and_cannot_be_overwritten(node):
    n=startup_node(node);worker=n.relocalization_worker
    command=dict(id='manual',session_id='test',created_at=time.time(),reference='tracking',x=5.,y=2.,z=0.,yaw=0.)
    (n.directory/'initial_pose.json').write_text(json.dumps(command))
    n.read_command()
    worker.close.assert_called_once()
    assert n.command_result['accepted'] and n.startup.state=='confirming'
    assert n.startup.reason.startswith('manual_seed') and n.startup.request is None


def test_global_failure_stays_manual_without_looping_worker(node):
    n=startup_node(node);worker=n.relocalization_worker
    worker.poll.return_value=dict(kind='result',accepted=False,reason='ambiguous_place_or_floor',
        request_id=n.startup.request['request_id'],map_sha256='a'*64)
    n.poll_relocalization(100.)
    assert n.startup.state=='failed' and n.relocalization_worker is None
    worker.close.assert_called_once();n.seed_pub.publish.assert_not_called()
    n.poll_relocalization(100.)
    assert n.relocalization_worker is None


def test_index_lifecycle_reports_have_original_map_identity_and_fenced_sequence(node, monkeypatch):
    n=startup_node(node)
    n.read_command=Mock();n.poll_relocalization=Mock()
    n.startup.verification=Mock()
    n.startup_status_sequence=0
    n.relocalization_map_sha256='b'*64
    n.relocalization_index={};n.relocalization_scan_status={}
    n.relocalization_status_pub=Mock()
    n.core.tracking=Mock(return_value=False)
    monkeypatch.setattr(LioLocalizer,'tick',lambda self:None)
    # ROS clock pause cannot prevent real index-progress messages from being
    # ordered; the sequence is lifecycle evidence, never a pose/source stamp.
    n.tick();n.tick()
    packets=[json.loads(call.args[0].data) for call in n.relocalization_status_pub.publish.call_args_list]
    assert [p['status_sequence'] for p in packets]==[1,2]
    assert [p['received_at_unix'] for p in packets]==[100.,100.]
    assert all(p['requested_map_sha256']=='b'*64 for p in packets)
    assert not any('local_odometry' in p or 'source_stamp' in p for p in packets)
