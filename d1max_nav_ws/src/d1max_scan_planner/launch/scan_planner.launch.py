import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _setup(context):
    backend = LaunchConfiguration("backend").perform(context)
    profiles = {
        "faster_lio": ("/Odometry", "/cloud_registered"),
        "fastlio2": (
            "/d1max/fastlio2/odometry",
            "/d1max/fastlio2/world_cloud",
        ),
    }
    if backend not in profiles:
        raise RuntimeError("backend must be 'faster_lio' or 'fastlio2'")

    default_odom, default_cloud = profiles[backend]
    odom_override = LaunchConfiguration("odom_topic").perform(context)
    cloud_override = LaunchConfiguration("cloud_topic").perform(context)
    odom_topic = odom_override or default_odom
    cloud_topic = cloud_override or default_cloud

    share = get_package_share_directory("d1max_scan_planner")
    config = os.path.join(share, "config", "d1max_scan_planner.yaml")
    rviz_config = os.path.join(share, "rviz", "d1max_scan_planner.rviz")

    planner = Node(
        package="scan_planner",
        executable="scan_planner_node",
        namespace="d1max/scan",
        name="scan_planner_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            config,
            {
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                ),
                "fsm.navi_mode": ParameterValue(
                    LaunchConfiguration("navi_mode"), value_type=int
                ),
                "grid_map.frame_id": LaunchConfiguration("frame_id"),
            },
        ],
        remappings=[
            ("body_pose", odom_topic),
            ("sensor_pose", odom_topic),
            ("cloud", cloud_topic),
            ("move_base_simple/goal", LaunchConfiguration("goal_topic")),
            ("initial_path", LaunchConfiguration("initial_path_topic")),
            ("planning/go2_execution_frozen", "/d1max/scan/execution_frozen"),
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="d1max_scan_rviz",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        parameters=[
            {
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                )
            }
        ],
    )
    return [planner, rviz]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("backend", default_value="faster_lio"),
            DeclareLaunchArgument("odom_topic", default_value=""),
            DeclareLaunchArgument("cloud_topic", default_value=""),
            DeclareLaunchArgument("frame_id", default_value="map"),
            DeclareLaunchArgument("navi_mode", default_value="1"),
            DeclareLaunchArgument("goal_topic", default_value="/goal_pose"),
            DeclareLaunchArgument(
                "initial_path_topic", default_value="/d1max/scan/initial_path"
            ),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            OpaqueFunction(function=_setup),
        ]
    )
