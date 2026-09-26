# Per-sensor ray metadata, version 1

This is an **opt-in acquisition-metadata branch**, not an obstacle map, a
deskewer, or a planner input. It is disabled by default and no localization or
planning launch file enables or consumes it. It does not resolve the currently
observed ground-voxel/body-envelope overlap or unobserved near-body volume.

## Scope and source contract

`dual_lidar_adapter` subscribes to the existing configured front/rear
`sensor_msgs/PointCloud2` sources. Each source message is handled independently:
there is no front/rear pairing wait, ring folding, voxelization or LIO 1:4 point
subsampling in this branch. Rear-only and front-fallback operation cannot lose
the source identity. Input message memory is read-only.

The branch uses exactly the existing configured transforms:

```
T_target_front = front_transform
T_target_rear  = front_transform * rear_to_front_transform
p_target(t)   = T_target_sensor * p_sensor(t)
o_target(t)   = T_target_sensor * [0, 0, 0]
```

Both endpoint and origin are expressed in the **instantaneous rigid target
frame at the point's acquisition time**. The output is not deskewed to the
header stamp, scan end, or map frame. The origin is not the robot body origin.
These are the active software calibration values, **not a claim that physical
factory calibration or point/IMU synchronization has been verified**.

The accepted input schema matches the current Airy adapter: `x/y/z/intensity`
FLOAT32, `ring` UINT16, `timestamp` FLOAT64, little-endian. Organized row padding
is honored. Input frame must match the specific source callback's configured
frame. Duplicate/missing/wrong-width required fields, non-rigid calibration,
oversized messages and invalid clocks fail closed. Malformed points are omitted
with no invented replacement. Original source indices remain available.

The existing sensor-native `min_range` and `max_range` also apply here. In the
current localization profile these are 0.5 m and 60 m. Thus this branch does not
claim new observation inside the existing blind range. Unknown cells stay
unknown. There is no self mask, ground removal or occupancy update.

## Wire schema

Topic: `/d1max/localization/perception/rays_raw` (configurable)

Message: `sensor_msgs/msg/PointCloud2`, height 1, little-endian, 64 bytes/point.
QoS: best-effort, volatile, depth 1. Each message contains exactly one sensor.
`header.frame_id` is the configured `target_frame`; `header.stamp` is the earliest
retained acquisition time after the shared clock offset. Points retain original
input order; they need not be in time order.

| Field | Offset / type | Meaning |
| --- | --- | --- |
| x, y, z | 0, 4, 8 / FLOAT32 | Endpoint in instantaneous target frame, metres |
| intensity | 12 / FLOAT32 | Unmodified source intensity |
| origin_x, origin_y, origin_z | 16, 20, 24 / FLOAT32 | This sensor's origin in that same frame, metres |
| sensor_id | 28 / UINT16 | 0 = configured front source; 1 = configured rear source |
| ring | 30 / UINT16 | Original source ring; no +96 offset or modulo 4 |
| offset_time | 32 / UINT32 | Acquisition time minus earliest retained source time, nanoseconds |
| source_index | 36 / UINT32 | Original row-major index before rejected/range-filtered points |
| timestamp | 40 / FLOAT64 | Point acquisition time in the shared ROS/host epoch, seconds |
| source_timestamp | 48 / FLOAT64 | Decoded acquisition time in the original sensor epoch, seconds |
| raw_timestamp | 56 / FLOAT64 | Exact original FLOAT64 timestamp value; original units unchanged |

`timestamp = source_timestamp + InputClock.offset`. The offset is the **same
InputClock instance** that serves the adapter's IMU and LIO publications; this
branch never observes/fits a separate clock and never stamps with receipt time.
The existing absolute-seconds/ms/us/ns or configured relative-time decoder is
shared, unchanged. Float64 epoch precision limits absolute timestamps to roughly
sub-microsecond precision; `offset_time` avoids a second epoch subtraction by
consumers but does not create accuracy absent from the source.

The current LIO time gates are retained: valid duration 1–150 ms, newest point
age no more than 500 ms and no more than 100 ms in the future. A missing, expired
or faulted shared clock prevents publication. Clock and age are rechecked after
conversion. A downstream collision consumer must impose its own stricter
freshness/session/TF coverage gates, never accept this 500 ms upper bound as
motion authorization.

## Enablement and resource bounds

Parameters on `dual_lidar_adapter`:

```
perception_rays.enabled = false
perception_rays.output_topic = /d1max/localization/perception/rays_raw
perception_rays.max_input_points = 250000
```

`config/perception_rays.disabled.yaml` is an example overlay, deliberately not
loaded anywhere. For a future isolated offline run, layer a reviewed copy after
the normal calibration configuration and explicitly set `enabled: true`. Do
not run a second live adapter or replace the primary configuration with this
overlay. Current launch/startup behavior remains unchanged.

Disabled: no additional publisher or worker thread, parsing, packing or pending
cloud retention. The original LIO callbacks only encounter a false branch.

Enabled: one worker, one pending shared cloud per sensor and at most one cloud
being converted. New arrivals replace only their source's pending message;
round-robin service prevents front traffic starving the rear. Oversize input
is rejected before queuing; the point cap and 64 MiB input-data cap bound
conversion memory. The worker publishes best-effort and does not block LIO
with conversion/network work. Drops mean missing observations, never free space.
The existing input-clock diagnostic adds counters for publications, rejection,
overwritten metadata and unavailable clock. No new robot communication is made.

## Projection and downstream integration boundary

An independent, default-off projector is now implemented in
`d1max_pct_scan.perception_ray_projector`; see that package's
`PERCEPTION_RAY_PROJECTOR.md`. The raw branch itself remains unchanged and does
not become a collision authority. The boundary requirements are:

1. The separate projector subscribes to this versioned schema plus a time-indexed
   local odometry/pose history. For each acquisition time, transform **both**
   endpoint and origin with the same `T_map_target(t)`. Require pose coverage,
   stable context/generation and calibrated extrinsics. Do not substitute one
   scan-end pose/origin for the two spinning sensors during motion.
2. Preserve sensor ID, timestamp and origin in the projected output. If mapping
   correction changes, drop/reproject pending raw rays coherently; do not mix
   different map corrections in one local occupancy update.
3. Add a dedicated native grid-map ray input; the existing XYZ cloud plus a
   single sensor odometry origin cannot represent these rays. Traverse only
   genuinely measured origin-to-endpoint segments, with observed-space tracking
   and the existing bounded memory/query budgets. Missing rays and stale/invalid
   metadata must not clear unknown or occupied space.
4. Ground support and body/leg returns require a separate, calibrated evidence
   policy. A PCT support height alone is not proof that an entire voxel is free;
   a static URDF box without verified geometry/joint pose is not a valid
   near-body self mask. Neither is implemented or silently enabled here.

## Offline validation

`test_perception_rays` is a pure C++ gtest executable: it never initializes a ROS
context, opens a socket, accesses an SDK or publishes a message. It covers
extrinsic composition and both origins, source identity without pairing,
organized layout/schema packing, original indices, acquisition order, absolute
and relative times, shared-clock offset/expiry, range/freshness rejection,
malformed/oversized/nonfinite inputs, and bounded fair queue/shutdown semantics.

Build only `dual_lidar_adapter` / `test_perception_rays` and run the latter
directly when validating this branch. Do not invoke unrelated launch tests on
the live ROS domain. Passing these tests validates metadata conversion and its
resource contract, not motion deskew, free-space correctness, physical
calibration or live robot navigation.
