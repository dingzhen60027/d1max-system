# D1 Max SDK to ROS 2 bridge

## Protected Foxglove console adapter (2026-09-09, v0.5)

`sdk_console_bridge` is the new console-specific controller; do not run it beside
`sdk_telemetry_bridge` or the original controller. It connects locked and never
automatically claims control, moves, stands or recovers an e-stop. Configuration
and startup are in `../foxglove_d1max/config/control.yaml` and
`scripts/start_live_controls.sh` (relative to the ROS workspace root).

Its pure `src/console_control_core.hpp` requires a command ACK and two new matching
RobotStates before advancing. Motion status 1 is a transition, not a completed
stand. Ownership must be explicitly acquired by this process; the observed SDK
control-source enum alone is insufficient. Velocity requires confirmed ownership,
GENERAL + low-speed preparation, fresh safe state and a live UI lease.

Only private stamped/session-scoped velocity and lease messages are accepted;
there is no generic `/cmd_vel` subscription. UI lease loss cancels queued steps;
reconnection and e-stop recovery never resume movement automatically. An already
authorized release/stop or e-stop may finish after the UI locks. State-machine
cancellation cannot undo a posture command already sent to the robot.

`src/test_console_control.cpp` tests the core without ROS/SDK. Optional CMake flag
`-DD1MAX_BUILD_CONSOLE_MOCK=ON` builds a non-installed, SDK-library-free mock target;
the integration harness requires isolated ROS Domain 91 and local-only Zenoh.
Production and mock binaries are separate. This is not a certified safety system;
robot-side network-loss behavior and real motion remain unverified. The September
9 deployment was paused before process replacement because the wired NIC was DOWN.

The original bridge description below is historical and does not describe the new
console adapter's rules. In particular, do not use its older interpretation of
STAND_UP as a stable completion state for the console.

## Receive-only viewer adapter (2026-09-09)

For Foxglove monitoring use `sdk_telemetry_bridge`, **not** the control bridge
described below. It only registers the SDK data callback and connects to receive
automatic RobotState/fault reports. It has no ROS command services/subscriptions,
never calls movement, emergency-stop, TakeControl or ReleaseControl, and only
disconnects its observer on shutdown. rclcpp still has an internal
`/parameter_events` subscription; parameter services are disabled.

Run through `../foxglove_d1max/scripts/start_sdk_telemetry.sh` from the workspace
root, or use the viewer's `start_live_view.sh`. The wrapper refuses to launch when
an existing `/d1max_sdk_bridge/robot_state` publisher is discovered.
The observer publishes the existing `robot_state`, `faults`,
`connection_state_text`, and `behavior_state` topics. Readiness and fault-latch
fields are null, since no control state machine runs in this observer.

The original guarded controller implementation below has not been enabled or
modified by this viewer integration.

This package is the single boundary between the proprietary D1 Max high-level
SDK and ROS 2. Joint-state support is not used because the current robot
firmware does not provide that stream.

ROS services submit behavior goals to a guarded state machine. The state
machine sends one SDK transition command and then waits for the 1 Hz
RobotState feedback to confirm completion before sending the next command.

## Safety model

- Startup connects passively and never takes control or sends motion.
- SDK Guide V0.0.9 transition rules are enforced.
- SDK callbacks only mean command received; completion uses RobotState.
- E-stop, stale state, lost control, timeout, and Error/Fatal faults cancel
  transitions and block velocity.
- cmd_vel reaches Move only in confirmed GENERAL mode while SDK owns control.
- A watchdog sends one zero velocity when cmd_vel becomes stale.
- Shutdown stops when legal and releases SDK control.

## State transitions

- LIE_DOWN to STAND uses StandUp.
- LIE_DOWN to CRAWL uses Crawl.
- CRAWL to STAND uses StandUp.
- GENERAL to CRAWL uses Crawl.
- Upright states enter GENERAL, IN_PLACE, or STAIR with SetMode.
- Stable motion modes enter LIE_DOWN with LieDown.
- LOCKED is never automatically unlocked by navigation. The operator must call
  unlock_to_stand, which sends StandUp and waits for upright RobotState.
- Error/Fatal fault latches cannot be cleared through ROS. Resolve the physical
  fault and restart the bridge so an old fault cannot be acknowledged blindly.

prepare_navigation executes and confirms each required step:

    TAKE_CONTROL -> STAND -> GENERAL_MODE -> SET_SPEED

## ROS interfaces

Topics:

- /d1max_sdk_bridge/robot_state
- /d1max_sdk_bridge/faults
- /d1max_sdk_bridge/behavior_state
- /d1max_sdk_bridge/transition_event
- /d1max_sdk_bridge/ready_for_navigation
- /d1max_sdk_bridge/connection_state
- /d1max_sdk_bridge/connection_state_text
- /cmd_vel

Services:

- /d1max_sdk_bridge/take_control
- /d1max_sdk_bridge/release_control
- /d1max_sdk_bridge/stand
- /d1max_sdk_bridge/lie_down
- /d1max_sdk_bridge/crawl
- /d1max_sdk_bridge/general_mode
- /d1max_sdk_bridge/in_place_mode
- /d1max_sdk_bridge/stair_mode
- /d1max_sdk_bridge/lock
- /d1max_sdk_bridge/unlock_to_stand
- /d1max_sdk_bridge/prepare_navigation
- /d1max_sdk_bridge/halt
- /d1max_sdk_bridge/soft_estop

Twist is converted to SDK Move as:

    linear.y  -> left_right
    linear.x  -> forward_back
    angular.z -> yaw

## Build and run

    source /opt/ros/humble/setup.bash
    cd d1max_ros2/sdk_bridge_ws
    colcon build --symlink-install
    source install/setup.bash
    ros2 launch d1max_sdk_bridge sdk_state_bridge.launch.py

Prepare navigation:

    ros2 service call /d1max_sdk_bridge/prepare_navigation std_srvs/srv/Trigger '{}'
    ros2 topic echo /d1max_sdk_bridge/behavior_state
    ros2 topic echo /d1max_sdk_bridge/ready_for_navigation

Wired SDK endpoint is 192.168.168.168:8081. Wireless is
192.168.234.1:8081.
