# Context-tagged dual-sensor ray input

This opt-in path is separate from SCAN's legacy `cloud` / `sensor_pose` input.
`grid_map.use_projected_rays` defaults to `false`; old configurations keep their
existing subscriptions and unknown-space policy. New mode requires lidar input,
map-frame geometry, no further extrinsic transform, localization context, and
`require_observed_free=true`. This is not motion acceptance.

## Contract

Subscribe to `projected_rays` (`d1max_planning_interfaces/ProjectedRays`). Each
packet contains **one physical sensor acquisition**. IDs are fixed to front `0`
and rear `1`; `grid_map.expected_ray_sensor_ids` must remain `[0,1]` for this D1 Max
version. Both endpoints and individual origins are already in `map`.

The native reader checks the 64-byte little-endian schema, bounded row/data sizes,
finite XYZ/origin, a consistent sensor ID, and mandatory field types/offsets.
Original header time is the earliest acquisition, not packet reception. End time
must follow it by no more than 250 ms; acquisition and alignment times must be
fresh and after the current context barrier. Session, epoch, seed, context
sequence, and barrier must match **exactly**. Per-source timestamp/projection
sequence replays are rejected.

Each source owns one latest pending packet (maximum 100,000 rays / 8 MiB per
packet), so front arrivals cannot overwrite pending rear input. Timer integration
checks both source age (maximum 500 ms) and monotonic queue wait (maximum 250 ms).
At most two acquisitions and 16 million voxel visits are integrated per timer
iteration. Exhausting traversal invalidates that iteration's planning support.

Each ray stops at its actual measured hit. Clipping to range/map/window makes an
artificial endpoint a free observation, never a surface. Only cells visited by
real rays gain free evidence. Real endpoints receive one hit vote per cell;
free traversal receives one miss vote per cell. A hit wins a same-frame tie.
The representative origin is used **only** to centre the rolling ROI; it never
replaces per-point origins. No robot volume, ground plane or start footprint is
cleared. Unobserved space remains unknown.

## Freshness and diagnostics

Only completed integration updates a source's support time. Public
`latestCloudStamp()` is the minimum of the two source stamps, or zero while either
source is missing/stale, including monotonic receipt expiry even if source time
pauses. Freshness is rechecked after ray integration. Localization context reset
clears all pending packets and integration leases; coordinate-identity changes
also clear actual map evidence using the existing reset mechanism.

`grid_map/projected_rays_status` is a reliable `std_msgs/String` JSON status.
Changed completed-source stamps, validity and context are published immediately
at the end of the occupancy callback (bounded by its 20 Hz timer). Unchanged
state has a 5 Hz heartbeat. This avoids adding up to 200 ms of notification delay
to a 500 ms source lease; heartbeat messages still carry the original source
stamp and cannot renew sensor evidence. Fields are `schema=1`, `session_id`, `epoch`, `seed_id`, `sequence`,
`barrier_ns`, `received_at_unix`, `valid`, `reason`, `source_stamp_ns`,
`integrated_counts`, and per-source `sources` counters/stamps. The wall timestamp
is **status emission time**, never a sensor timestamp or an integration lease.
Only `source_stamp_ns` represents currently valid, both-source integrated support.

Memory-only regression target: `test_projected_rays` (no ROS initialization,
discovery, robot connection, motion command, or replacement of unknown by free).
