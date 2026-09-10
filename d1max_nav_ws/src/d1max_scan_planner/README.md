# D1 Max SCAN-Planner integration

This package connects the D1 Max LIO output to SCAN-Planner's rolling 3D local
map and B-spline planner. It intentionally launches no controller and publishes
no `/cmd_vel` command.

## Inputs

- `body_pose` and `sensor_pose`: selected LIO odometry
- `cloud`: current scan transformed into the `map` frame
- `/goal_pose`: interactive PoseStamped goal for `navi_mode:=1`
- `/d1max/scan/initial_path`: external 3D nav_msgs/Path for `navi_mode:=3`

## Outputs

- `/d1max/scan/planning/bspline`: optimized local trajectory
- `/d1max/scan/grid_map/occupancy`: rolling 3D occupancy cloud
- `/d1max/scan/grid_map/occupancy_inflate`: collision-inflated cloud
- `/d1max/scan/optimal_list`: RViz trajectory marker

## Run

Start one LIO backend first, then run one of:

```bash
/home/dndx/d1max_nav_ws/start_scan_planner.sh faster_lio
/home/dndx/d1max_nav_ws/start_scan_planner.sh fastlio2
```

For non-default Faster-LIO topic remaps:

```bash
ros2 launch d1max_scan_planner scan_planner.launch.py \
  backend:=faster_lio odom_topic:=/your/odom cloud_topic:=/your/world_cloud
```

For a multi-floor global planner path:

```bash
ros2 launch d1max_scan_planner scan_planner.launch.py \
  backend:=fastlio2 navi_mode:=3 use_rviz:=true
```
