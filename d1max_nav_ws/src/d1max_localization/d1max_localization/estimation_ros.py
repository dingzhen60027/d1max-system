"""Small ROS-message adapters; numerical estimation modules do not import ROS."""

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from .math_utils import Pose3, normalize_quaternion
from .estimation.contracts import vector


def stamp_time(seconds):
    return rclpy.time.Time(nanoseconds=round(seconds * 1e9)).to_msg()


def seconds(message):
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


def read_pose(pose):
    p = pose.position
    q = pose.orientation
    return Pose3(
        vector((p.x, p.y, p.z)), normalize_quaternion(vector((q.x, q.y, q.z, q.w), 4, 2.0))
    )


def fill_pose(message, pose):
    message.position.x, message.position.y, message.position.z = pose.position
    message.orientation.x, message.orientation.y, message.orientation.z, message.orientation.w = (
        pose.orientation
    )


def pose_message(frame, pose, stamp, cov=None):
    m = PoseStamped() if cov is None else PoseWithCovarianceStamped()
    m.header.frame_id = frame
    m.header.stamp = stamp_time(stamp)
    fill_pose(m.pose if cov is None else m.pose.pose, pose)
    if cov is not None:
        m.pose.covariance = cov
    return m


def transform(parent, child, pose, stamp):
    m = TransformStamped()
    m.header.frame_id = parent
    m.child_frame_id = child
    m.header.stamp = stamp_time(stamp)
    m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = pose.position
    (
        m.transform.rotation.x,
        m.transform.rotation.y,
        m.transform.rotation.z,
        m.transform.rotation.w,
    ) = pose.orientation
    return m


def odometry(frame, child, pose, stamp, linear, angular, pose_cov, twist_cov):
    m = Odometry()
    m.header.frame_id = frame
    m.child_frame_id = child
    m.header.stamp = stamp_time(stamp)
    fill_pose(m.pose.pose, pose)
    m.pose.covariance = pose_cov
    m.twist.covariance = twist_cov
    m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z = linear
    m.twist.twist.angular.x, m.twist.twist.angular.y, m.twist.twist.angular.z = angular
    return m
