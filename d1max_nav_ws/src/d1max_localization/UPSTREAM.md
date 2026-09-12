# Source provenance

- 2026-09-12 default backend: the existing local Faster-LIO ROS 2 port now has an opt-in bounded localization runtime (IMU reception separate from the mapping worker, frame/time/health checks, bounded queues and map, configurable stationary initialization). Upstream algorithm: gaoxiang12/faster-lio; existing user modifications for mapping were retained. This is not an unmodified upstream binary.
- New default coordinator `lio_fusion.py` / `lio_localizer.py` combines a scan-time LIO pose with verified full-SE(3) map alignment. MC velocity is telemetry only. Added generation-bound recovery, registration observability checks, synthetic moving scans and an isolated read-only bag probe.

- `fused_icp_matcher.cpp`, rotation deskewer / acquisition gate, `icp_fusion_bridge.py`, math / motion helpers and their original tests were adapted from the user's local `/home/dndx/go2_nav/src/go2_localization` working tree, inspected 2026-09-10. That package declares MIT in package.xml; its maintainer metadata has been preserved. No separate LICENSE file was present in that source package.
- `dual_lidar_adapter.cpp` was adapted from the user's local `d1max_nav_ws/src/d1max_slam/src/dual_lidar_adapter.cpp`. Static calibration matches the existing D1 mapping pipeline, replacing public TF lookup with explicit private transforms. No proprietary SDK implementation is copied into this package.
- New D1 additions: sensor admission / gyro-bias supervisor, SDK velocity frame conversion, verified-only global TF / pose publication, dual-EKF configuration, session launch, offline tests and Web lifecycle integration.
- Runtime dependencies retain their own licenses: robot_localization, fast_gicp, ROS / PCL, Livox message definitions. Livox messages are an internal compatibility format; no claim that the D1 lidar is a Livox device.

Vendor SDK headers / binaries remain separately supplied in the existing D1 SDK bridge workspace, not redistributed here.
