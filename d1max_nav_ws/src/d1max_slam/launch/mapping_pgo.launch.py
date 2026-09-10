from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    slam_share = Path(get_package_share_directory("d1max_slam"))
    use_rviz = LaunchConfiguration("use_rviz")
    output_dir = LaunchConfiguration("output_dir")

    frontend = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(slam_share / "launch" / "mapping.launch.py")),
        launch_arguments={
            "backend": "faster_lio",
            "use_rviz": "false",
            "output_dir": output_dir,
        }.items(),
    )

    pgo = Node(
        package="sc_pgo",
        executable="alaserPGO",
        name="d1max_sc_pgo",
        output="screen",
        parameters=[
            str(slam_share / "config" / "sc_pgo_d1max.yaml"),
            {"save_directory": PathJoinSubstitution([output_dir, "sc_pgo"])},
        ],
        remappings=[
            ("/aft_mapped_to_init", "/d1max/faster_lio/odometry"),
            ("/velodyne_cloud_registered_local", "/d1max/faster_lio/cloud_registered_body"),
            ("/aft_pgo_odom", "/d1max/pgo/odometry"),
            ("/aft_pgo_path", "/d1max/pgo/path"),
            ("/aft_pgo_map", "/d1max/pgo/map"),
            ("/loop_scan_local", "/d1max/pgo/loop_scan"),
            ("/loop_submap_local", "/d1max/pgo/loop_submap"),
            ("/pgo_loop_count", "/d1max/pgo/loop_count"),
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="d1max_pgo_rviz",
        output="screen",
        arguments=["-d", str(slam_share / "rviz" / "d1max_pgo.rviz")],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "output_dir", default_value="/home/dndx/d1max_nav_ws/maps"
            ),
            GroupAction(actions=[frontend], scoped=True),
            pgo,
            rviz,
        ]
    )
