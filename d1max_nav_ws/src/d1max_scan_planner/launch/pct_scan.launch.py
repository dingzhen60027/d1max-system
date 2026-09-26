"""Real SCAN local planner for an externally managed PCT navigation session.

No router, simulator, motion controller, SDK session, or frame aliases are created.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    value = lambda key: LaunchConfiguration(key).perform(context)
    session_id = value("session_id")
    if not session_id or any(c not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-" for c in session_id):
        raise ValueError("PCT + SCAN requires an explicit safe session_id")
    namespace = value("namespace") or f"/d1max/pct_scan/s_{session_id}/scan"
    config = os.path.join(get_package_share_directory("d1max_scan_planner"),
                          "config", "d1max_scan_planner.yaml")
    return [Node(
        package="scan_planner", executable="scan_planner_node",
        namespace=namespace, name="scan_planner_node", output="screen",
        parameters=[config, {
            "use_sim_time": value("use_sim_time").lower() == "true",
            "fsm.navi_mode": 3,
            "fsm.require_tagged_reference": True,
            "fsm.navigation_session_id": session_id,
            "fsm.strict_input_frames": True,
            "fsm.odom_twist_in_body_frame": True,
            "fsm.odom_timeout": 0.5,
            "fsm.max_replan_interval": 1.0,
            "fsm.reference_path_guidance": True,
            "fsm.reference_path_z_offset": float(value("reference_z_offset")),
            "fsm.reference_start_tolerance": 1.0,
            "grid_map.frame_id": value("frame_id"),
            "grid_map.sliding_map_frame_id": f"d1max_pct_scan_{session_id}_local",
            "grid_map.strict_input_frames": True,
            "grid_map.maximum_cloud_pose_dt": 0.25,
            "grid_map.cloud_is_world": True,
            "grid_map.need_extrinsic": False,
            "manager.max_vel": 0.30,
            "optimization.max_vel": 0.30,
            "optimization.lambda_reference": 20.0,
        }],
        remappings=[
            ("body_pose", value("odom_topic")),
            ("sensor_pose", value("sensor_pose_topic")),
            ("cloud", value("cloud_topic")),
        ],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("session_id"),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("frame_id", default_value="d1max_loc_map"),
        DeclareLaunchArgument("odom_topic", default_value="/d1max/localization/odometry/global"),
        DeclareLaunchArgument("sensor_pose_topic", default_value="/d1max/pct_scan/sensor_pose_map"),
        DeclareLaunchArgument("cloud_topic", default_value="/d1max/pct_scan/cloud_map"),
        DeclareLaunchArgument("reference_z_offset", default_value="0.55"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        OpaqueFunction(function=_setup),
    ])
