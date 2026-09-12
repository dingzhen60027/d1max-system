"""Real ROS message objects; synthetic trajectories, no robot or SDK."""
from math import sin,cos,pi
import pytest
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from d1max_localization.icp_fusion_bridge import IcpFusionBridge
from d1max_localization.math_utils import Pose3,yaw_from_quaternion

class RecordingPublisher:
    def __init__(self):self.messages=[]
    def publish(self,message):self.messages.append(message)

@pytest.fixture
def node(tmp_path,monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR',str(tmp_path));rclpy.init()
    node=IcpFusionBridge()
    node.global_set_pose_pub=RecordingPublisher()
    node.icp_prediction_pub=RecordingPublisher()
    node.icp_pose_pub=RecordingPublisher()
    try:yield node
    finally:node.destroy_node();rclpy.shutdown()

def local(node,t,x=0.,yaw=0.):
    m=Odometry();m.header.frame_id='odom';m.child_frame_id='d1max_loc_tracking'
    m.header.stamp=rclpy.time.Time(nanoseconds=t).to_msg()
    m.pose.pose.position.x=x;m.pose.pose.orientation.z=sin(yaw/2);m.pose.pose.orientation.w=cos(yaw/2)
    node.on_local_odometry(m)

def seed(node,t):
    node.on_initial_pose(node.tracking_pose_message(Pose3((4.,-2.,0.),(0.,0.,0.,1.))))
    node.initialized_stamp_ns=t-1

def raw(node,t,pose=None):
    node.on_raw_icp(node.tracking_pose_message(pose or node.reference_pose,t))

def test_global_ekf_reset_is_deferred_until_verified_time_paired_alignment(node):
    t=node.get_clock().now().nanoseconds
    seed(node,t)
    assert not node.global_set_pose_pub.messages and not node.fusion_gate.alignment_locked
    local(node,t)
    raw(node,t)
    assert node.fusion_gate.alignment_locked
    reset=node.global_set_pose_pub.messages[0]
    assert node.message_stamp_ns(reset)==t
    assert reset.pose.pose.position.x==4.

def test_missing_local_pair_cannot_lock_or_replace_anchor(node):
    t=node.get_clock().now().nanoseconds
    seed(node,t);raw(node,t)
    assert not node.fusion_gate.alignment_locked
    assert node.reference_local_pose is None
    assert not node.global_set_pose_pub.messages

def test_delayed_reset_is_motion_transported_and_not_restamped(node):
    t=node.get_clock().now().nanoseconds-200_000_000
    seed(node,t)
    # Delayed ICP at t+10 ms; local trajectory brackets it at 20 ms intervals.
    for i in range(11):local(node,t+i*20_000_000,x=i*.02)
    raw(node,t+10_000_000)
    reset=node.global_set_pose_pub.messages[0]
    assert node.message_stamp_ns(reset)==t+200_000_000
    assert reset.pose.pose.position.x==pytest.approx(4.19)
    assert node.reference_local_pose.position[0]==pytest.approx(.01)
    assert node.message_stamp_ns(node.icp_pose_pub.messages[0])==t+10_000_000

@pytest.mark.parametrize('yaw',[pi/9,pi/3,17*pi/18,-3*pi/4])
def test_prediction_follows_arbitrary_local_turn_not_global_drift(node,yaw):
    t=node.get_clock().now().nanoseconds-100_000_000
    seed(node,t);local(node,t);raw(node,t)
    local(node,t+100_000_000,yaw=yaw)
    prediction=node.active_prediction()
    assert yaw_from_quaternion(prediction.orientation)==pytest.approx(yaw)
    node.publish_prediction()
    assert node.message_stamp_ns(node.icp_prediction_pub.messages[-1])==t+100_000_000
    node.publish_prediction()
    assert len(node.icp_prediction_pub.messages)==1
    raw(node,t+100_000_000,prediction)
    assert node.accepted==2

def test_missing_motion_does_not_publish_fresh_looking_frozen_prediction(node):
    t=node.get_clock().now().nanoseconds
    seed(node,t);local(node,t);raw(node,t)
    node.reference_local_pose=None
    node.publish_prediction()
    assert not node.icp_prediction_pub.messages
    assert node.active_prediction() is None

def test_rejected_unpaired_measurement_keeps_both_reference_halves(node):
    t=node.get_clock().now().nanoseconds
    seed(node,t);local(node,t);raw(node,t)
    before=(node.reference_pose,node.reference_local_pose)
    raw(node,t+1_000_000,Pose3((4.1,-2.,0.),(0.,0.,0.,1.)))
    assert (node.reference_pose,node.reference_local_pose)==before
    assert node.accepted==1

def test_quality_covariance_is_not_replaced_by_smaller_fixed_noise(node):
    t=node.get_clock().now().nanoseconds
    seed(node,t);local(node,t)
    m=node.tracking_pose_message(node.reference_pose,t)
    for i in (0,7,14,21,28,35):m.pose.covariance[i]=.8
    node.on_raw_icp(m)
    assert node.icp_pose_pub.messages[0].pose.covariance[0]==.8

def test_unstamped_or_reversed_local_odometry_never_gets_receipt_stamp(node):
    t=node.get_clock().now().nanoseconds
    local(node,0)
    assert not node.local_pose_buffer
    local(node,t);local(node,t-1)
    assert len(node.local_pose_buffer)==1

def test_large_gap_is_not_hidden_by_nearest_odometry_sample(node):
    t=node.get_clock().now().nanoseconds-300_000_000
    local(node,t);local(node,t+300_000_000)
    assert node.local_pose_at(t+100_000_000) is None
