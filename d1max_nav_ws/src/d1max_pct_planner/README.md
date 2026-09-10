# D1 Max PCT Planner

This package keeps upstream PCT Planner isolated in `pct_planner_vendor` and adds:

- CPU tomography for systems where the NVIDIA driver/CUDA is unavailable.
- Offline global path generation through the upstream C++ planner.
- ROS 2 map/path publication and RViz2 visualization.

The upstream project is GPLv2. This integration does not replace localization,
local obstacle avoidance, gait control, or footstep planning.

