from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    robot_ip = LaunchConfiguration("robot_ip")
    robot_port = LaunchConfiguration("robot_port")
    return LaunchDescription([
        DeclareLaunchArgument("robot_ip", default_value="192.168.168.168"),
        DeclareLaunchArgument("robot_port", default_value="8081"),
        ExecuteProcess(
            cmd=["ros2", "run", "rmw_zenoh_cpp", "rmw_zenohd"],
            name="d1max_zenoh_router",
            output="screen",
        ),
        TimerAction(
            period=1.0,
            actions=[
                Node(
                    package="d1max_sdk_bridge",
                    executable="sdk_state_bridge",
                    name="d1max_sdk_bridge",
                    output="screen",
                    emulate_tty=True,
                    parameters=[{
                        "robot_ip": robot_ip,
                        "robot_port": ParameterValue(robot_port, value_type=int),
                    }],
                ),
                Node(
                    package="d1max_sdk_gui",
                    executable="d1max_sdk_gui",
                    name="d1max_sdk_gui",
                    output="screen",
                ),
            ],
        ),
    ])
