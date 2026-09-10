# D1 Max navigation workspace

This workspace starts with dual-96-line lidar SLAM and is structured so later
localization, Nav2, elevators and cross-floor map management can be added as
separate packages.

## Packages

- `d1max_slam`: D1 Max sensor adapters, backend launch, RViz and PCD saving.
- `faster_lio`: migrated from `/home/dndx/go2_nav`; default SLAM backend.
- `fastlio2`: migrated from `/home/dndx/go2_nav`; optional backend.
- `livox_ros_driver2`: migrated message/driver dependency required by FastLIO2.

Generated data directories (`Log`, `PCD`, `result`, `.git`) from the source
Faster-LIO tree were intentionally not migrated.

## Build

```bash
cd /home/dndx/d1max_nav_ws
./build.sh
```

## Start mapping

Make sure the existing Zenoh connection to the NX is running, then start the
default backend:

```bash
./start_slam.sh faster_lio true
```

Keep the robot stationary for IMU initialization, then walk it slowly through
the mapping area. To compare the second migrated algorithm:

```bash
./start_slam.sh fastlio2 true
```

Do not run both backends simultaneously. Their odometry frames and compute load
would interfere with diagnosis.

## Save map

```bash
./save_map.sh
```

Maps are written to `maps/d1max_map_YYYYMMDD_HHMMSS.pcd`; `maps/scans.pcd`
points to the latest saved map. A graceful Ctrl+C also triggers a save.

## Initial assumptions

- Both Airy96 clouds use the observed schema: `x/y/z/intensity` FLOAT32,
  `ring` UINT16 and `timestamp` FLOAT64.
- Existing robot TF provides `rslidar_tail` to `rslidar_head` calibration.
- Point timestamps are absolute seconds (the adapter also detects epoch
  milliseconds, microseconds and nanoseconds).
- The front lidar IMU acceleration is expressed in `g`; the adapter converts it.
- This phase produces 3D PCD maps. Nav2 occupancy maps and cross-floor topology
  belong to the next phase after LIO quality is confirmed.
