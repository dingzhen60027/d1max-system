# d1max_slam

D1 Max dual Airy96 lidar integration for ROS 2 Humble. This package only reads
sensor data. It contains no motion-control publisher or SDK control call.

## Data flow

- `/front_lidar` (`rslidar_head`) and `/rear_lidar` (`rslidar_tail`) are paired.
- The robot TF transforms rear points into `rslidar_head`.
- `/d1max/slam/points` exposes `x y z intensity ring time` for Faster-LIO.
- `/d1max/slam/livox_points` exposes the same scan as `CustomMsg` for FastLIO2.
- `/front_lidar/imu` is republished as `/d1max/slam/imu`; acceleration is
  multiplied by `9.80665` to convert the observed `g` units to `m/s^2`.
- Rear rings are shifted from `0..95` to `96..191`.

The default backend is `faster_lio`. The FastLIO2 path is retained as an
experimental comparison because its upstream package only consumes Livox
`CustomMsg`, so the adapter must synthesize that message.

## Map output

The map capture node voxelizes registered world points continuously. Calling
`/d1max/slam/save` writes a timestamped binary PCD and updates `scans.pcd`.
