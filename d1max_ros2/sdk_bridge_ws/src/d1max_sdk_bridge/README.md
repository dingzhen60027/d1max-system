# D1 Max SDK to ROS 2 bridge

## Current production entry (2026-09-12): OnMcData velocity ONLY

The active Foxglove/Web path uses **sdk_monitor_bridge**, not the historical
controllers below. It never moves, changes posture/mode or releases an e-stop.
Following explicit operator authorization, the managed configuration now enables
`TakeControl` **only after `OnControlAvailable`**, as described below.
The robot can also assign a new connection as master when
no master exists; omitting `TakeControl` does not guarantee observer-only ownership.
Its sole robot safety service is the explicit one-way soft
e-stop. After the first RobotState on each SDK connection plus one second, it requests
`SetMcConfig(true,0,write_handler)`: telemetry configuration only. The second argument
is timeout in milliseconds, NOT frequency. `config/monitor.yaml` sets the ready delay,
a three-second retry interval and at most three requests per burst, followed by
15 / 30 / 60-second capped cooldowns. Exhaustion does not permanently disable
MC reporting requests. A fresh stream stops requests even if its ACK was lost (reported as
`streaming_unconfirmed`, never fabricated as an ACK). Two seconds of continuous
healthy-rate data reset the burst budget. A stream outage is reported immediately
after 0.3 s; re-enabling waits at least 1 s. Only this idempotent telemetry request can retry;
e-stop / robot control retry rules are unchanged. Replay prevents telemetry requests.
`IDataCallback::OnMcData` only copies a fixed-size MotionData packet and receipt times
to a bounded 128-slot queue. A separate worker validates and publishes ROS JSON on
`/d1max_sdk_bridge/velocity`. The source is `sdk_mc`, frame is `sdk_body`;
`v_body[3]` is m/s, `omega_body[3]` is rad/s. Compatibility fields `forward_speed`,
`lateral_speed`, `yaw_speed` are exactly v_body[0], v_body[1], omega_body[2], so existing
Foxglove plots need no layout change. The active bridge never enables OnSpeedData and
omits RobotState velocity fields entirely. Localization also rejects non-MC sources,
even if the legacy fallback parameter is true. RobotState still supplies heading,
batteries and safety state. World position/quaternion are NOT published as map pose/TF.

The supplied 0.1.0 headers and SDK Markdown document fixed 50 Hz; the older PDF's
100 Hz description conflicts with this version. Always report observed frequency,
not the documented value as a measurement. The September 12 authorized reconnect
validated this bridge at approximately 50 Hz with 1,100 received samples; this is
bounded live reception evidence, not a long-duration or moving-localization test.

`source_timestamp_ns` preserves the original uint64 as a decimal string. Its epoch
is undocumented. `received_at_unix` is host callback receipt time; `stamp_unix` is
the first host receipt plus source timestamp deltas. `clock_mode` is
`source_delta_host_anchor`, `clock_approximate` is true. This preserves source intervals
under batching but is NOT hardware synchronization and does not calibrate one-way
network delay or the separate LiDAR clock. Zero/repeated/backwards source stamps,
nonfinite velocities, queue age >= 0.3 s and clock disagreement >= 0.3 s are rejected.
Clock resets require a new telemetry generation (connection or confirmed ownership
handoff); there is no silent rebase within one generation.
Overflow drops oldest packets with a counter; disconnect clears source timing, and
shutdown joins the worker, disables only its requested MC stream with a bounded send
timeout, then disconnects. No old speed is timer-republished or replaced with zero.
Existing `sdk_commands_sent` in the safety status is the e-stop request
counter; it does not count telemetry configuration or ownership requests.

`/d1max_sdk_bridge/speed_report_status` separates write completion, configuration ACK,
sample count, freshness and observed Hz. Frequency uses a two-second monotonic callback
window, requires at least one second of samples, and becomes zero on stale input.
`streaming` requires an on-ACK and observed rate within 80–120% of the documented
50 Hz reference, not just an ACK. `expected_hz` is diagnostic, never a rate request.
`source_hz`, invalid/timestamp/stale and queue-drop counters remain separate.
Missing ACK, missing data, mismatched rate, retry cooldown and disconnection remain
explicit states. A new connection clears measurements / ACK / retry budget; late write
callbacks cannot alter a later connection or attempt. No second SDK socket is opened.
`attempts` is the current burst count, `total_attempts` is the connection-wide request
ID, and `next_retry_sec` is a monotonic-clock countdown (-1 when not retrying).
Initial connection uses the documented async API. The bridge retries every 5 s only
in `DISCONNECTED`; the SDK retains ownership of connecting, handshaking and automatic
reconnection. It never disconnects a healthy SDK session to repair MC reporting.
An exclusive `/run/user/<uid>/d1max-sdk-monitor.lock` also protects direct executable
launches, in addition to the wrapper's legacy-publisher discovery check. Do not remove
the lock file while running; the OS releases its lock when the owning process exits.

Regression targets: CTest `test_mc_report`, `test_monitor_estop`, `test_sdk_session`, `test_sdk_ownership`, localization pytest,
and Foxglove parser/layout tests. Local regression checks passed. Initial September 12
reconnects while APP held the master session did NOT recover MC. After confirmed APP
release and explicit operator authorization, a new connection became master and
MC recovered: one enable request, on-ACK, fresh samples, observed 50.005–50.016 Hz,
zero timestamp rejections. Do not confuse these live results with offline tests.

### Authorized APP -> SDK handoff (2026-09-12; offline validation)

`config/monitor.yaml` enables `auto_take_control_on_available: true` at the user's
request. The executable default remains false. No additional service or motion
controller is exposed. The installed SDK documents that SDK cannot preempt APP.

- When APP owns the session, wait; missing MC or `control_source=SDK` does NOT
  authorize TakeControl. Initial connection does not unconditionally take control.
- Copy `OnControlAvailable` to a bounded event queue. The executor waits 0.5 s,
  requires fresh RobotState, then sends **one** async `TakeControl(0, handler)`.
- Write success is not ownership. Require successful `OnTakeControlAck`, then
  **two newer RobotStates** reporting SDK. The latter enum alone cannot identify
  this client as owner. Confirmed handoff starts a new MC generation, requests
  `SetMcConfig(true,0)`, and independently checks actual callbacks / ACK / Hz.
- `OnControlLost` cancels pending work and relinquishes the local ownership claim.
  A later APP release can trigger a new handoff after a previously confirmed cycle.
  No Move, posture, mode change, e-stop release or automatic motion resume is added.
- Requests do not retry: ACK timeout (3 s), state-confirmation timeout (4 s), write
  error or callback overflow remain explicit. Ambiguous attempts block further
  TakeControl on that connection because the vendor ACK has no request identifier.
  Reconnect only after operator review; never reconnect the whole stack in a loop.
- Replay / disconnection suppress handoff; shutdown joins the worker and closes the
  existing SDK socket. No secondary SDK connection or ReleaseControl request is added.

`speed_report_status.ownership` and `/d1max/monitor/status.ownership` expose the
handoff state, ACK, confirmations, request count and error. Web shows waiting for
APP, taking control, verifying, rejected or uncertain; real MC data remains the
only source of frequency. Existing Foxglove plots and layout stay unchanged.
Callbacks only copy bounded events; SDK sends / JSON / ROS run outside callbacks.

Compiled and covered by offline core, safety-boundary, health and UI tests.
Robot is intentionally disconnected; repeated real APP release / reclaim cycles
and this firmware's TakeControl behavior still need live acceptance. If no release
notification arrives, stay waiting rather than infer ownership or cycle sockets.

### MC session-permission incident: diagnosed and recovered on 2026-09-12

See the [Chinese incident record and safe recovery procedure](docs/incidents/2026-09-12-mc-session-permission.md).
APP disconnect alone did not promote the existing SDK slave session. The authorized
reconnect after APP release resolved this occurrence; the firmware restriction remains.

The robot's `robot_remote` log records our SDK connection
`192.168.168.10:36540` as `Master session already exists ... as slave`.
RobotState identified the control source as APP. A bounded local wire trace confirmed
that the vendor `SetMcConfig(true)` sent message type 1008 with `sensor:30`,
`enable:true`, `freq:0`; RobotState arrived, but no MC packets or configuration ACK did.
`freq:0` is the vendor encoding for this boolean API, not a requested zero-Hz rate.

Read-only inspection of the installed server established why: type 1008 (0x3f0)
falls through `ClientChannel::Step1` to `IsMaster`; a non-master returns false,
and `ProcessMessage` skips `Step2`, which handles sensor configuration. This is a
restriction of the inspected robot server, not proof of a universal SDK requirement.
Registering `IDataCallback` correctly or retrying configuration cannot bypass it.

Evidence: robot log `/root/.ros/log/robot_remote_2269_1771865610129.log`;
server `/opt/robot/install/robot_remote/lib/robot_remote/robot_remote`, SHA-256
`bd7c490b40bfc04c6f9ed84614f10230c718dbbaf7770f2dd2f0f5dd30ef45c3`;
`Step1` dispatch 0x7d214–0x7d258, master gate 0x7db90–0x7dbcc;
`ProcessMessage` gate 0x7cc88–0x7ccbc. Robot log timestamps are February 24 while
the host date is September 12. Clock skew was observed, but was not established as
the cause of this failure; no clock, chrony or robot-service changes were made.

The bridge must NOT call `TakeControl` outside the explicitly authorized handoff
policy above, disconnect the APP, spoof a master, or substitute RobotState/OnSpeedData.
A master-session test needs explicit operator
approval and safe local supervision. Keeping APP control while enabling MC from a
slave requires vendor confirmation/support for the installed firmware. Do not start
localization or claim MC ready until actual fresh callback data has been verified.

Normative SDK sources are the supplied RobotSDK-0.1.0-charging_v2-x86_64 package:
`docs/zh/sdk_callback_cn.md`, `docs/zh/sdk_client_api_cn.md` (SetMcConfig),
`include/robot_sdk/sdk_type.hpp` (MotionData) and `example/data.cpp`.
Read these before changing SDK usage; callbacks must only copy/check data, not perform
ROS/network publishing, JSON serialization, file I/O or heavy computation.

Foxglove's existing velocity plots now parse only /d1max_sdk_bridge/velocity;
battery gauges retain their separate RobotState source. This MC revision does not
create/edit a native layout or change panel splits, cameras or trajectory width.

Do not run a second SDK bridge for localization. The new d1max_localization
package subscribes to the existing SDK JSON and lidar IMU; it has no SDK command
client. Historical controller sources remain for reference and are not activated.

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
