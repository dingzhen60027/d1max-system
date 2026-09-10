from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
        DeclareLaunchArgument('pcd'),
        DeclareLaunchArgument('path'),
        DeclareLaunchArgument('frame', default_value='map'),
        Node(package='d1max_pct_planner', executable='pct_visualize', output='screen',
             arguments=['--pcd', LaunchConfiguration('pcd'),
                        '--path', LaunchConfiguration('path'),
                        '--frame', LaunchConfiguration('frame')]),
        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', PathJoinSubstitution([
                 FindPackageShare('d1max_pct_planner'), 'rviz', 'pct.rviz'])]),
    ])


from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
