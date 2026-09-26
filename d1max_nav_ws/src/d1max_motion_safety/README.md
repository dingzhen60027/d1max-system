# Continuous Collision Monitor (Humble)

Executable: `d1max_motion_safety/continuous_collision_monitor`. It keeps the native
node name `collision_monitor`, native lifecycle and existing Collision Monitor
parameters/remappings. Run it with a single-threaded executor; launch and lifecycle
activation are configured by the navigation package, not this adapter.

Native Humble Collision Monitor suppresses continued zero output after
`stop_pub_timeout`, including when fresh zero or collision-stop inputs continue
arriving. This makes a downstream command freshness gate unable to distinguish
an intentionally stopped, healthy pipeline from a silent failed pipeline.
The relevant upstream implementation is
[CollisionMonitor::publishVelocity, lines 149–158](https://github.com/ros-navigation/navigation2/blob/humble/nav2_collision_monitor/src/collision_monitor_node.cpp#L149-L158).

The adapter first completes native `on_configure`, then replaces only its depth-1
input subscription callback. For each received command it sets `stop_stamp_` to
the current native clock and invokes native `cmdVelInCallback`. Native finite-value
validation, `process_active_`, sources, polygons and output publication still decide
the result. See the upstream
[input callback and process path](https://github.com/ros-navigation/navigation2/blob/humble/nav2_collision_monitor/src/collision_monitor_node.cpp#L139).

There is no output timer, cached-command replay, direct velocity publication or
watchdog relaxation. No input means no output; inactive nodes and invalid input
still produce no output. Resetting the timestamp does not make stale sensors fresh
or prove obstacle coverage. Keep independent source/TF health checks and the
downstream real-message timeout. If native processing itself takes longer than
`stop_pub_timeout`, the native suppression condition can still apply.

This package is tied to the protected API in locally installed Nav2 Humble 1.1.20.
Recheck the native contract and tests on any Nav2 update. It neither calls the robot
SDK nor changes posture, gait, mode, control ownership or emergency-stop state.

## Offline validation

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select d1max_motion_safety --cmake-args -DBUILD_TESTING=ON -DRMW_IMPLEMENTATION=rmw_zenoh_cpp
colcon test --packages-select d1max_motion_safety --event-handlers console_direct+
colcon test-result --test-result-base build/d1max_motion_safety --verbose
```

Tests link the real installed `collision_monitor_core`, use same-process command
and output callbacks, and supply obstacle points from memory. They compare native
zero suppression with adapter behavior, exercise the native STOP polygon, and check
inactive/deactivated, invalid-input and no-input behavior. No robot or production
navigation process is started.

The test wrapper retains `rmw_zenoh_cpp`, domain 219, and starts an owned private
Zenoh router on a unique loopback port. Its router has no uplink, multicast or
gossip; test sessions connect only to that private endpoint. The test binary refuses
direct execution without the wrapper's isolation markers. Input/output transport
uses intra-process communication. This is an isolated local ROS test graph, not the
production graph. The private router is cleaned up after success or failure. No
communication middleware, production environment or service configuration is changed.
