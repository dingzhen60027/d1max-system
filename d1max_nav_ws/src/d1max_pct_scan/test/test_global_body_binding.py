from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Time
from rclpy.serialization import serialize_message,deserialize_message
from nav_msgs.msg import Path

from d1max_pct_scan.live_global_planner import LiveGlobalPlanner,prepare_validated_path
from test_live_goal_pause import IDENTITY


def body(stamp,x):
    result = Odometry()
    result.header.frame_id = 'd1max_loc_map'
    result.child_frame_id = 'd1max_loc_base_link'
    result.header.stamp = Time(sec=int(stamp),nanosec=round((stamp-int(stamp))*1e9))
    result.pose.pose.position.x = float(x)
    result.pose.pose.position.z = .5
    result.pose.pose.orientation.w = 1.
    return result


def node(monkeypatch):
    clock = [100.04]
    identity = [IDENTITY]
    monkeypatch.setattr('d1max_pct_scan.live_global_planner.time.monotonic',lambda:clock[0])
    obj = SimpleNamespace(pause=SimpleNamespace(intent=None),bridge=SimpleNamespace(source_frame='d1max_loc_map'),
        p={'freshness_s':.5},now_s=lambda:clock[0],confirmed_identity=lambda:identity[0],
        sensor_identity=IDENTITY,body_barrier=100.,last_body_stamp=100.,body=None,body_context=None,
        pose_received=100.02,pose_status=dict(schema=1,epoch=4,seed_id='seed-a',
            frame_id='d1max_loc_map',body_frame='d1max_loc_base_link',valid=True,pose_valid=True,
            reset_pending=False,fault=None,motion_control_enabled=False,pose_timeout_sec=.08,
            received_at_unix=100.02,output_stamp_sec=100.02),_revoke_if_context_lost=lambda:None)
    return obj,clock,identity


def test_old_body_packet_cannot_replace_new_sample_or_renew_subscription(monkeypatch):
    obj,clock,_ = node(monkeypatch)
    LiveGlobalPlanner.on_body(obj,body(100.03,1.))
    accepted = obj.body_received
    clock[0] = 100.05
    LiveGlobalPlanner.on_body(obj,body(100.01,99.))
    assert obj.body[0] == 1. and obj.body_stamp == 100.03
    assert obj.body_received == accepted and obj.body_context == IDENTITY


def test_new_epoch_raises_barrier_and_rejects_queued_pre_reset_body(monkeypatch):
    obj,clock,identity = node(monkeypatch)
    LiveGlobalPlanner.on_body(obj,body(100.03,1.))
    identity[0] = replace(IDENTITY,epoch=5,confirmed_seed='seed-b')
    obj.pose_status.update(epoch=5,seed_id='seed-b',received_at_unix=100.04,output_stamp_sec=100.04)
    LiveGlobalPlanner.on_body(obj,body(100.035,99.))
    assert obj.body is None and obj.body_barrier == 100.04
    clock[0] = 100.06
    LiveGlobalPlanner.on_body(obj,body(100.039,99.))
    assert obj.body is None
    LiveGlobalPlanner.on_body(obj,body(100.05,2.))
    assert obj.body[0] == 2. and obj.body_context == identity[0]


def test_old_epoch_pose_lease_cannot_authorize_new_epoch_body(monkeypatch):
    obj,clock,identity = node(monkeypatch)
    identity[0] = replace(IDENTITY,epoch=5,confirmed_seed='seed-b')
    obj.sensor_identity = identity[0]
    LiveGlobalPlanner.on_body(obj,body(100.03,1.))
    assert obj.body is None


def test_prebuilt_message_headers_share_fresh_commit_stamp(monkeypatch):
    points = np.array([[0.,0.,-.5],[1.,0.,-.5]])
    monkeypatch.setattr('d1max_pct_scan.live_global_planner.validate_static_route',
        lambda *args, **kwargs:dict(xyz=points,diagnostics={},source_tomogram_sha256='hash'))
    checked = prepare_validated_path({},None,SimpleNamespace(source_frame='d1max_loc_map'))
    message = checked['path_message']
    assert all(pose.header is message.header for pose in message.poses)
    message.header.stamp = Time(sec=101,nanosec=5)
    decoded = deserialize_message(serialize_message(message),Path)
    assert all(pose.header.stamp == message.header.stamp for pose in decoded.poses)
    assert decoded.poses[1].pose.position.x == 1.
