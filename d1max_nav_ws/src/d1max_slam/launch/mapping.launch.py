from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    backend = LaunchConfiguration("backend").perform(context)
    use_rviz = LaunchConfiguration("use_rviz").perform(context).lower() in (
        "1", "true", "yes", "on"
    )
    output_dir = LaunchConfiguration("output_dir").perform(context)
    lidar_mode = LaunchConfiguration("lidar_mode").perform(context)
    if backend not in ("faster_lio", "fastlio2"):
        raise RuntimeError("backend must be 'faster_lio' or 'fastlio2'")
    if lidar_mode not in ("dual", "front", "rear"):
        raise RuntimeError("lidar_mode must be 'dual', 'front', or 'rear'")

    package_share = Path(get_package_share_directory("d1max_slam"))
    actions = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="d1max_lidar_level_calibration",
            output="screen",
            arguments=[
                "--x", "0.0",
                "--y", "0.0",
                "--z", "0.0",
                # Kept as an identity link so existing frame names remain
                # stable. Airy point-to-IMU calibration is applied below.
                "--qx", "0.0",
                "--qy", "0.0",
                "--qz", "0.0",
                "--qw", "1.0",
                "--frame-id", "d1max_lidar",
                "--child-frame-id", "d1max_lidar_uncalibrated",
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="d1max_airy_to_ros",
            output="screen",
            arguments=[
                "--x", "0.0",
                "--y", "0.0",
                "--z", "0.0",
                # Airy publishes LiDAR points and IMU samples with the same
                # frame_id even though their physical frames differ. This is
                # the normalized LiDAR-point -> IMU/ROS rotation derived from
                # the factory DIFOP calibration. Replace it with this unit's
                # DIFOP values when the robot firmware exposes device_info_.
                "--qx", "-0.499867275",
                "--qy", "0.503186620",
                "--qz", "0.497953310",
                "--qw", "0.498977388",
                "--frame-id", "d1max_lidar_uncalibrated",
                "--child-frame-id", "rslidar_head",
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="d1max_lidar_extrinsic",
            output="screen",
            arguments=[
                # Active D1 Max firmware /localization rear-to-front calibration:
                # p_front = diag(1, -1, -1) * p_rear + [0, 0, -0.7323].
                "--x", "0.0",
                "--y", "0.0",
                "--z", "-0.7323",
                "--qx", "1.0",
                "--qy", "0.0",
                "--qz", "0.0",
                "--qw", "0.0",
                "--frame-id", "rslidar_head",
                "--child-frame-id", "rslidar_tail",
            ],
        ),
        Node(
            package="d1max_slam",
            executable="dual_lidar_adapter",
            name="dual_lidar_adapter",
            output="screen",
            parameters=[
                str(package_share / "config" / "dual_lidar.yaml"),
                {"lidar_mode": lidar_mode},
            ],
        )
    ]

    if backend == "faster_lio":
        registered_topic = "/d1max/faster_lio/cloud_registered"
        trajectory_file = str(Path(output_dir) / "faster_lio_trajectory.txt")
        actions.append(
            Node(
                package="faster_lio",
                executable="run_mapping_online",
                name="laserMapping",
                output="screen",
                arguments=[f"--traj_log_file={trajectory_file}"],
                parameters=[
                    str(package_share / "config" / "faster_lio_airy96.yaml"),
                    {
                        "common.lid_topic": "/d1max/slam/points",
                        "common.imu_topic": "/d1max/slam/imu",
                        "common.time_sync_en": False,
                        "preprocess.lidar_type": 2,
                        "preprocess.scan_line": 192 if lidar_mode == "dual" else 96,
                        "preprocess.time_scale": 1.0,
                        "mapping.extrinsic_est_en": False,
                        "mapping.extrinsic_T": [0.0, 0.0, 0.0],
                        "mapping.extrinsic_R": [
                            1.0, 0.0, 0.0,
                            0.0, 1.0, 0.0,
                            0.0, 0.0, 1.0,
                        ],
                        "pcd_save.pcd_save_en": False,
                    },
                ],
                remappings=[
                    ("cloud_registered", registered_topic),
                    ("cloud_registered_body", "/d1max/faster_lio/cloud_registered_body"),
                    ("cloud_registered_effect_world", "/d1max/faster_lio/effect_world"),
                    ("Odometry", "/d1max/faster_lio/odometry"),
                    ("path", "/d1max/faster_lio/path"),
                ],
            )
        )
        rviz_config = package_share / "rviz" / "faster_lio.rviz"
    else:
        registered_topic = "/d1max/fastlio2/world_cloud"
        fastlio2_config = package_share / "config" / "fastlio2_airy96.yaml"
        actions.append(
            Node(
                package="fastlio2",
                executable="lio_node",
                name="lio_node",
                output="screen",
                parameters=[{"config_path": str(fastlio2_config)}],
                remappings=[
                    ("body_cloud", "/d1max/fastlio2/body_cloud"),
                    ("world_cloud", registered_topic),
                    ("lio_path", "/d1max/fastlio2/path"),
                    ("lio_odom", "/d1max/fastlio2/odometry"),
                ],
            )
        )
        rviz_config = package_share / "rviz" / "fastlio2.rviz"

    actions.append(
        Node(
            package="d1max_slam",
            executable="map_capture_node",
            namespace="d1max/slam",
            name="map_capture",
            output="screen",
            parameters=[
                {
                    "input_topic": registered_topic,
                    "output_directory": output_dir,
                    "voxel_size": 0.08,
                    "map_topic": "/d1max/slam/map_cloud",
                    "publish_period_sec": 1.0,
                    "auto_save_on_shutdown": True,
                }
            ],
        )
    )
    if use_rviz:
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="d1max_slam_rviz",
                output="screen",
                arguments=["-d", str(rviz_config)],
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "backend",
                default_value="faster_lio",
                description="SLAM backend: faster_lio or fastlio2",
            ),
            DeclareLaunchArgument(
                "use_rviz", default_value="true", description="Start RViz2"
            ),
            DeclareLaunchArgument(
                "lidar_mode",
                default_value="dual",
                description="LiDAR input mode: dual, front, or rear",
            ),
            DeclareLaunchArgument(
                "output_dir",
                default_value="/tmp/d1max_maps",
                description="Directory used by the map capture service",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
