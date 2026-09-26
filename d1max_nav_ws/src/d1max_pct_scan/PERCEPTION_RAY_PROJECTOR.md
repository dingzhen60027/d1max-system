# Per-sensor acquisition-time projection

This optional module is a separate perception process. It does not change LIO,
publish motion commands, acquire SDK control, remove ground/self returns, or
make unknown space free. Startup remains owned by the live-session supervisor.

## Inputs and one coherent transform

- `/d1max/localization/perception/rays_raw`: version-1 64-byte ray metadata.
- `/d1max/localization/odometry/local`: **odom → body** continuous local motion.
- `/d1max/localization/pose`: **map → tracking**, not the body pose.
- `/tf_static`: the existing direct **body → tracking** and **tracking → lidar**
  edges. Missing edges are not replaced by guessed identity transforms.
- Localization/navigation/continuous-pose status plus the native map-context
  request and its exact acknowledgement. Session, epoch, seed, sequence and
  source-time reset barrier must agree.

For a scan, use an exact-source-stamp pair of local and global tracking poses
at or just after the scan end (maximum 100 ms by default):

```
T_odom_tracking(t) = T_odom_body(t) * T_body_tracking
T_map_odom(ref)    = T_map_tracking(ref) * inverse(T_odom_tracking(ref))
T_map_lidar(t)     = T_map_odom(ref) * T_odom_tracking(t) * T_tracking_lidar
```

The **same `T_map_odom(ref)` is used for the whole scan**. Interpolate only the
local pose history at each point's measured acquisition time, using linear XYZ
and shortest-arc quaternion SLERP. Apply the resulting `T_map_lidar(t)` to both
the endpoint and that point's front/rear sensor origin. Do not raycast every
point from a single robot origin or mix different map corrections within one
scan. Neither nearest-pose substitution nor extrapolation is permitted.

## Output contract

Topic `/d1max/live_planning/rays_map`, type
`d1max_planning_interfaces/msg/ProjectedRays`, sensor-data QoS.

The envelope contains:

- `session_id`, `epoch`, `seed_id`, `context_sequence`, `barrier_ns`: exact
  acknowledged native map identity, independent of user-goal generations.
- `projection_sequence`: increases only when a new frame is projected.
- `acquisition_end`: latest original point acquisition time.
- `alignment_stamp`: the exact-source-stamp pose pair used for that scan.
- `rays`: PointCloud2 in the map frame, with the original **earliest** point
  time as header stamp. All 64-byte metadata fields remain intact; only XYZ and
  origin XYZ change. `sensor_id=0` is front, `1` rear. Native integration must
  not transform the packet again or collapse its per-point origins.

Source-age checks use the earliest point conservatively; `acquisition_end`,
processing time and status heartbeat never renew that lease. PointCloud2
`offset_time` remains nanoseconds relative to the source/header scan start.

`/d1max/live_planning/ray_projector_status` reports context readiness, pose and
alignment buffer sizes, per-source true publication stamps, bounded counters,
processing time and failure reason. `valid` additionally requires a recent
published observation from every configured source; it is not motion permission
or proof that every queried voxel is observed free.

## Enablement

Source default is disabled. The supported entry point is:

```
python -m d1max_pct_scan.perception_ray_projector --session <owned-session-directory>
```

It starts only when that session explicitly selects
`perception_backend: per_sensor_rays`, its **session copy** of localization YAML
enables `dual_lidar_adapter`'s `perception_rays.enabled`, and the continuous
navigation output is enabled. It reads frame names and source topic from that
same profile. An absent session/backend does not create a ROS context. The main
localization YAML need not be changed. Middleware remains Zenoh.

Optional `perception_projector` session settings correspond exactly to the pure
`Limits` dataclass. All are bounded; defaults include 2 s/256 pose samples,
250,000 points per input, maximum bracketing pose gap 60 ms, source TTL 500 ms,
maximum scan span 150 ms, and alignment wait 100 ms. A same-context map-correction
step exceeding 0.20 m or 0.10 rad, or any static-extrinsic change, latches a
projection fault until a new acknowledged context. This does not erase or
secretly repair the native occupancy map.

Each sensor has one latest pending cloud. Repeated/old clouds cannot renew the
250 ms receipt deadline. Context change immediately clears pending clouds and
pose/alignment history, and reapplies the known static edges in the new context.
A single geometry worker operates on a private immutable-value history snapshot;
there is at most one in-flight job and one pending cloud per source, without an
executor backlog. The main ROS thread continues receiving fresh pose and status
callbacks. Before committing a completed result it rechecks the context/token,
original receipt deadline, earliest source time and the unchanged 80 ms pose
lease. Reset or invalidation prevents an already-running job from publishing.
A large frame may be dropped when stale; overload is not hidden by restamping.

## Validation and limits

Pure core and real-callback/fake-ROS tests exercise 53 cases without ROS init,
SDK, sockets or robot control: rotating/translating scans, both real sensor
origins, body/tracking lever arm, one alignment per scan, pose callback ordering,
exact context ACK, reset/expiry, malformed schema, source provenance, missing
pose/static coverage, bounded queues/history, private worker snapshots,
non-blocking status callbacks and rejection of obsolete worker results.

Interpolation and transform composition are batched over exact unique point
timestamps, then gathered for every original point. This does not round time or
downsample rays. There are no per-point TF queries. The 09-26 recorded-data
file-only replay converted 151 raw scans and projected/integrated 141 with both
sensors. Ten were rejected for startup context or missing/bracketed pose
coverage. Successful scans retained approximately 58k/60k points with about
8.7k distinct times; pure projection measured median 8.67 ms, p95 10.09 ms and
maximum 14.05 ms on this development machine. This excludes ROS serialization
and native insertion and is not NUC hardware acceptance. The point cap is a
memory bound, not a throughput guarantee.

The same replay still found the near-body collision query blocked in all 149
checks, while its 2 m forward queries were free. Successful two-source metadata
integration is therefore not evidence that the original near-body ground and
unknown-space blockage has been solved.

The file-only `build/d1max_localization/perception_rays_offline` utility converts
recorded raw PointCloud2 CDRs using the **same C++ converter as the live adapter**.
It takes a JSONL request manifest and emits JSONL results plus new CDR files;
it never calls `rclcpp::init`. A request requires input/output CDR paths,
sensor ID and source/target frames, effective translation/quaternion, the recorded
shared clock offset and the recorded processing time. Original input files are
not modified and existing output files are refused.

Correct origin/time metadata and motion projection do not by themselves solve
ground-voxel/body-envelope overlap, near-body occlusion, physical calibration,
or dynamic leg returns. Those require separately validated evidence policies;
this module does not introduce an unverified self mask or unknown-space bypass.
