"""Isolated central-IMU frontend experiment; no PGO or robot driver."""
import math
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def rotation_quaternion(flat):
    """Quaternion xyzw for the row-major child-to-parent rotation."""
    r = [[float(flat[3 * i + j]) for j in range(3)] for i in range(3)]
    trace = sum(r[i][i] for i in range(3))
    if trace > 0:
        s = 2 * math.sqrt(trace + 1)
        return [(r[2][1] - r[1][2]) / s, (r[0][2] - r[2][0]) / s,
                (r[1][0] - r[0][1]) / s, s / 4]
    i = max(range(3), key=lambda k: r[k][k])
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2 * math.sqrt(1 + r[i][i] - r[j][j] - r[k][k])
    q = [0., 0., 0., (r[k][j] - r[j][k]) / s]
    q[i], q[j], q[k] = s / 4, (r[j][i] + r[i][j]) / s, (r[k][i] + r[i][k]) / s
    return q


def validated_extrinsics(calibration):
    """Read explicit experimental transforms; never repair an invalid matrix."""
    def rigid(value, label):
        rotation = [float(x) for x in value['rotation']]
        translation = [float(x) for x in value['translation']]
        if len(rotation) != 9 or len(translation) != 3:
            raise ValueError(f'{label} requires rotation[9] and translation[3]')
        if not all(math.isfinite(x) for x in rotation + translation):
            raise ValueError(f'{label} contains nonfinite values')
        rows = [rotation[i:i + 3] for i in (0, 3, 6)]
        ortho = max(abs(sum(rows[i][k] * rows[j][k] for k in range(3)) - (i == j))
                    for i in range(3) for j in range(3))
        a, b, c, d, e, f, g, h, i = rotation
        determinant = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)
        if ortho > 1e-5 or abs(determinant - 1.) > 1e-5:
            raise ValueError(f'{label} is not a valid rotation: orthogonality={ortho}, det={determinant}')
        return rotation, translation

    front = rigid(calibration['lio_extrinsic'], 'lio_extrinsic')
    rear = calibration.get('lidar_extrinsics', {}).get('rear_to_front', {
        'rotation': [1., 0., 0., 0., -1., 0., 0., 0., -1.],
        'translation': [0., 0., -.7323],
        'parent_frame': 'rslidar_head', 'child_frame': 'rslidar_tail',
    })
    if rear.get('parent_frame') != 'rslidar_head' or rear.get('child_frame') != 'rslidar_tail':
        raise ValueError('rear_to_front must map rslidar_tail points into rslidar_head')
    return front, rigid(rear, 'rear_to_front')


def static_tf(name, parent, child, translation, quaternion):
    args = []
    for key, value in zip(('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'),
                          [*translation, *quaternion]):
        args.extend(['--' + key, str(value)])
    args.extend(['--frame-id', parent, '--child-frame-id', child])
    return Node(package='tf2_ros', executable='static_transform_publisher', name=name,
                arguments=args, output='screen', parameters=[{'use_sim_time': False}])


def setup(context):
    output = Path(LaunchConfiguration('output_dir').perform(context))
    frontend = LaunchConfiguration('frontend_config').perform(context)
    adapter = LaunchConfiguration('adapter_config').perform(context)
    calibration_path = LaunchConfiguration('calibration_config').perform(context)
    calibration = yaml.safe_load(Path(calibration_path).read_text())
    cloud_topic = LaunchConfiguration('cloud_topic').perform(context)
    (rotation, translation), (rear_rotation, rear_translation) = validated_extrinsics(calibration)
    body_frame = calibration['frame_id']
    return [
        static_tf('central_experiment_lidar_level', 'd1max_lidar', 'd1max_lidar_uncalibrated',
                  [0., 0., 0.], [0., 0., 0., 1.]),
        static_tf('central_experiment_airy_to_ros', 'd1max_lidar_uncalibrated', 'rslidar_head',
                  [0., 0., 0.], [-0.499867275, 0.503186620, 0.497953310, 0.498977388]),
        static_tf('central_experiment_lidar_extrinsic', 'rslidar_head', 'rslidar_tail',
                  rear_translation, rotation_quaternion(rear_rotation)),
        static_tf('central_experiment_imu_to_lidar', body_frame, 'd1max_lidar',
                  translation, rotation_quaternion(rotation)),
        Node(package='d1max_slam', executable='dual_lidar_adapter', name='dual_lidar_adapter',
             output='screen', parameters=[adapter, {
                 'use_sim_time': False, 'lidar_mode': 'dual',
                 'input_imu_topic': '/central_experiment/unused_front_imu',
                 'output_imu_topic': '/central_experiment/unused_imu',
             }]),
        Node(package='faster_lio', executable='run_mapping_online', name='laserMapping',
             output='screen', parameters=[frontend, {
                 'use_sim_time': False,
                 'common.lid_topic': cloud_topic,
                 'common.imu_topic': calibration['output_topic'],
                 'common.time_sync_en': False,
                 'preprocess.lidar_type': 2, 'preprocess.scan_line': 192,
                 'preprocess.time_scale': 1.0,
                 'mapping.extrinsic_est_en': False,
                 'mapping.extrinsic_R': rotation, 'mapping.extrinsic_T': translation,
                 'publish.world_frame': 'camera_init', 'publish.body_frame': body_frame,
                 'pcd_save.pcd_save_en': False,
             }], remappings=[
                 ('cloud_registered', '/d1max/faster_lio/cloud_registered'),
                 ('cloud_registered_body', '/d1max/faster_lio/cloud_registered_body'),
                 ('cloud_registered_effect_world', '/d1max/faster_lio/effect_world'),
                 ('Odometry', '/d1max/faster_lio/odometry'),
                 ('path', '/d1max/faster_lio/path'),
             ]),
        Node(package='d1max_slam', executable='map_capture_node', namespace='d1max/slam',
             name='map_capture', output='screen', parameters=[{
                 'use_sim_time': False,
                 'input_topic': '/d1max/faster_lio/cloud_registered',
                 'output_directory': str(output), 'voxel_size': 0.08,
                 'map_topic': '/d1max/slam/map_cloud', 'publish_period_sec': 1.0,
                 # The experiment supervisor explicitly saves final/partial
                 # maps. Avoid a second identical PCD during node shutdown.
                 'auto_save_on_shutdown': False,
             }]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('output_dir'), DeclareLaunchArgument('frontend_config'),
        DeclareLaunchArgument('adapter_config'), DeclareLaunchArgument('calibration_config'),
        DeclareLaunchArgument('cloud_topic', default_value='/d1max/slam/points'),
        OpaqueFunction(function=setup),
    ])
