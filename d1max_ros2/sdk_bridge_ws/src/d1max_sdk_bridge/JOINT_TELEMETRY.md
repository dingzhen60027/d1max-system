# Optional read-only joint telemetry

`sdk_monitor_bridge` declares `joint_state_enabled` with default `false`. This
change does not modify the deployed monitor parameter file, start another SDK
session, or change the existing MC reporting / control-ownership / motion logic.
When explicitly enabled, the existing session calls only
`SetJointStateConfig(true, 0, ...)` for this feature. It makes at most three
requests per observed connection, 3 seconds apart, after a 0.5 second connection
settle period. It never reconnects or takes control to recover this stream.
Shutdown closes only a joint stream requested in the current connection.

## ROS interface

Both topics use `std_msgs/msg/String` containing JSON:

- `/d1max_sdk_bridge/joint_states`: one message per accepted SDK callback, with
  `names`, `positions`, optional `velocities` and `efforts` (empty when absent),
  `session`, `generation`, monotonic `sequence`, and `received_at_unix`.
- `/d1max_sdk_bridge/joint_state_status`: diagnostics, configuration ACK/write
  state, retry counts, invalid/stale/dropped counts, and `stream_fresh`.

Every measurement explicitly declares `clock_mode: "receipt_only"`,
`source_timestamp_available: false`, `source_timestamp_ns: null`,
`units_verified: false`, and `urdf_mapping_verified: false`. No source stamp or
`sensor_msgs/JointState` is fabricated. A status heartbeat never republishes or
updates a measurement's receipt time. The 0.25 second receipt TTL bounds local
queue age and consumer freshness; it **cannot bound acquisition age or network
delay**, since the vendor data type provides neither a timestamp nor sequence.
Do not use this stream alone to remove perceived obstacles as robot self-returns.
Joint order, units, zero/sign convention and pose-time alignment need independent
verification before any URDF collision-mask use.

The official SDK bundle's `sdk_callback.hpp` exposes `OnJointStateData` and
`OnJointStateConfig`; `sdk_client.hpp` exposes `SetJointStateConfig(bool, ...)`.
`sdk_type.hpp::JointStateData` contains only four vectors and supplies no
acquisition timestamp or units. The bundled Chinese callback/client/type Markdown
documents omit these newer joint declarations; they do require lightweight data
callbacks and distinguish command write completion from the control ACK.
The official bundle's `example/data.cpp:136-141` does document the convention
through its output labels: positions in **rad**, velocities in **rad/s**, efforts
in **Nm**. `units_verified: false` means that actual received values and their
URDF zero/sign conventions have not been validated on this robot; it does not
mean the bundle has no unit convention. `CHANGELOG.md:6-10` lists joint reporting
as new in SDK v0.1.0. `example/data.cpp:218-224` enables/disables it with
`SetJointStateConfig(bool)`; the example's connection/configuration flow has no
explicit `TakeControl` prerequisite. These are client API facts, not proof that
every compatible firmware exposes the new stream. The README compatibility table
does not specify per-feature minimum firmware versions.

## Bounds and validation

The callback validates/copies at most 64 joints, each name at most 64 ASCII
alphanumeric / underscore / slash / hyphen characters. Names must be nonempty and
unique. Positions must match names exactly; velocity/effort must be empty or the
same length. All supplied numbers must be finite. Original names, values and
ordering are retained without invented remapping, unit conversion or joint-limit
clamping. Invalid packets are discarded in full. A latest-only mailbox bounds
queued memory; JSON and ROS publication run on a separate joint worker, not inside
the SDK callback or the existing MC worker. Its failure disables only joint
reporting. Local stale, future-receipt and out-of-order packets are rejected.
Disconnect/replay clears admission evidence; old write callbacks cannot satisfy a
new generation. A positive configuration ACK is diagnostic, not a measurement.
The SDK's bool-only ACK has no request identifier either: its receipt barrier
rejects locally queued pre-request ACKs but cannot identify an old ACK delivered
late by the SDK. It never authorizes geometry use or substitutes for fresh data.

`test_joint_report` is an SDK-free, ROS-free regression executable covering these
contracts. Build/test does not connect to a robot. Enabling this startup-only
parameter in an existing process requires the separately authorized monitor
restart; this implementation does not perform that restart.
