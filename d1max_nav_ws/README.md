# D1 Max navigation workspace

BehaviorTree.CPP owns the navigation task: localization, 3D goal, fixed global
route, continuous local reference, candidate planning/validation, tracking,
safety and the sole SDK writer. PCT and SCAN are the current replaceable backends.
The architecture targets indoor/outdoor, multi-floor navigation. Single-floor
runs are only the current test scope, not a separate architecture. Roots come
from the environment (`deploy/d1max.env.example`), floors from map manifests;
each release/profile still limits execution to its actually verified coverage.

## Development mainline

As confirmed by the user on 2026-10-03, the existing BehaviorTree.CPP navigation
framework is the **only development mainline**. Its session module is
`d1max_pct_scan.navigation_session`, with `tools/navigation_entry.sh` as the
public version-pinned entry. They expose the same existing implementation and
loader; `single_floor_session` / `single_floor_entry.sh` remain legacy names
for callers and sealed releases, not another navigation stack. Extend this same task, reference, planning, tracking and
SDK pipeline for single-floor, multi-floor, indoor and outdoor navigation;
do not introduce another navigator, task owner or competing motion pipeline.
Planner implementations remain selectable through configuration and contracts.

Operational budgets have one source:
`src/d1max_pct_scan/config/navigation_runtime.yaml`. Session preparation derives
the BT, adapters and functional monitor parameters from it and verifies their
agreement before startup. Short input/heartbeat gaps pause the existing task;
fresh evidence resumes it within the shared recovery budget. A rejected local
candidate cannot erase a still-safe accepted trajectory. Temporary waits retain
the committed route; true collision, loss of control, emergency stop and
unconfirmed retirement still deny motion. Internal interface/build identities
prevent stale results and support rollback; they are not separate user workflows.

Work continues in the current checkout; no new Git branch or parallel navigation
project is created without an explicit user request. Isolated builds, regression
fixtures, release bundles and rollback snapshots are validation/version artifacts
of this same mainline, not alternative architectures. See [AGENTS.md](AGENTS.md)
for the project working rules and [current status](docs/status/PROJECT_TAKEOVER_STATUS_20260928.md)
for the dated implementation and acceptance boundaries.

The default entry now selects the complete mainline runtime at
`experiments/navigation_runtime` (2026-10-08), with its integrity manifest pinned
in `deploy/single_floor_release.json`. Localization, interfaces, BT, planners,
tracker, SDK adapter and RViz load from that same bundle. Historical bundles are
rollback artifacts, not additional workflows. Selecting these bytes does not
grant SDK motion authority or claim physically measured performance.

2026-10-14: the selected default map restores the **2026-10-02 multi-floor global
planning result** (`global_20261002_170003_637005`). The same entry now exposes
both floors and their staircase connection, with original-map 3D goals and
floor-aware initial poses. Only deployment paths changed; the PCT geometry and
cost arrays were preserved bit for bit. Map selection does not select or disable
the task graph: the existing complete navigation BT, reference manager, SCAN,
tracker and SDK configuration are retained. The conditioned map is still a
planning input, not a control TF or a physical acceptance record. Route geometry
and movement authorization remain separate from module startup; planning success
is not navigation arrival. The original-coordinate floor map is retained unchanged.

The 2026-10-04 performance round reduces native snapshot buffers, removes hot-loop
heap allocation and isolates complete-frame worker IPC and periodic SDK publishing.
Same-input measurements and the bounded-memory tradeoffs are recorded in the
[navigation module performance section](src/d1max_pct_scan/README.md#性能整改2026-10-04未部署)
and `experiments/navigation_performance_20261004/`. Those source changes are
included in the current mainline bundle. Physical acceptance flags are unchanged.
Core microbenchmarks are not an end-to-end 20 ms guarantee.

## Layout

```
d1max_nav_ws/
├── src/                  ROS 2 packages (colcon)
│   ├── d1max_*           our packages: localization, pct_planner, pct_scan,
│   │                     scan_planner, trajectory_tracker, navigation(_bt), ...
│   ├── faster_lio fastlio2 livox_ros_driver2 sc_pgo    migrated SLAM stack
│   └── pct_planner_vendor scan_planner_vendor          upstream planners
├── scripts/              mapping and auxiliary/legacy operator tools
│   ├── d1max_env.sh      shared roots; every script sources it
│   ├── build/            build_slam, build_pct_planner, build_slam_pgo
│   ├── mapping/          start_slam, start_slam_pgo, save_map, record_slam_bag, ...
│   ├── planning/         legacy previews and diagnostic launchers
│   └── navigation/       legacy navigation/Nav2 launchers
├── tools/                release entry and offline engineering tools
│   ├── navigation_entry.sh    public entry used by the Web
│   ├── single_floor_entry.sh  compatible version-pinned loader
│   ├── release/          seal / capture / evidence for releases
│   ├── map/              map building and point-cloud processing
│   ├── pointcloud_preprocessing/   importable preprocessing package
│   ├── diagnostics/      audits and probes
│   ├── benchmark/        benchmarks and plots
│   └── validation/       isolated graph runs, smoke tests, replays
├── docs/
│   ├── status/           current project status (read first)
│   ├── design/           contracts and architecture
│   ├── reports/          dated implementation and audit reports
│   └── archive/          superseded handovers and notes
├── deploy/               environment template and pinned official release
├── maps/ bags/ experiments/   data (git-ignored)
└── build/ install/ log/       colcon output (git-ignored)
```

Root compatibility symlinks have been removed; use the paths above.

## Build

```bash
scripts/build/build_slam.sh        # livox driver, fastlio2, faster_lio, d1max_slam
```

## Mapping

Make sure the Zenoh connection to the NX is up, then:

```bash
scripts/mapping/start_slam.sh faster_lio true
scripts/mapping/save_map.sh
```

Keep the robot stationary for IMU initialization, then walk it slowly. Do not
run two SLAM backends at once. Maps go to `maps/d1max_map_YYYYMMDD_HHMMSS.pcd`;
`maps/scans.pcd` points to the latest. A graceful Ctrl+C also saves.

## Navigation

The official navigation session API is `d1max_pct_scan.navigation_session`
through `tools/navigation_entry.sh`. Single-floor is a test/profile scope only.
These names do not claim accepted indoor/outdoor or stair motion capability;
the current execution profile remains floor-segment only. The public entry
delegates to the same loader and preserves the exact module recorded by each
sealed release, rather than rewriting old manifests or creating a second graph.
`deploy/single_floor_release.json` selects one hash-pinned release; it does not
grant control or mark physical acceptance. `D1MAX_RELEASE` is an explicit
override for another sealed bundle, never an automatic newest-version lookup.

```bash
tools/navigation_entry.sh --check-release   # bytes/import/RMW loading only; no ROS node or SDK connection
```

The same graph has two purposes: `planning_only` (no safety/SDK motion pipeline)
and `execution` (typed BT authorization, safety gate and existing SDK writer).
For either, prepare a fresh session, seal it, then run it. A live execution
session binds the current SDK session and the shared runtime motion profile;
it does not require a physical calibration report. Startup never
connects/reconnects SDK, sets posture, or executes a goal. Task ownership,
source freshness, emergency/control-loss stopping and software limits remain.

`scripts/planning/start_live_planning_view.sh`, the old motion coordinator and
the old Nav2 launcher are retained as legacy workflows, not official motion
entry points. The Web navigation launcher uses `/api/navigation-session`; the
old `/api/single-floor` URL is an alias of the same runtime/lock, not a second service. Its private
activation binds the release and its runtime profile; the current SDK session
is bound when starting. Selecting a release alone is not a task execution grant.

The Web's `/#/planning` page now has one mainline lifecycle entry, not a
preview/navigation selector. Its built static assets must be rebuilt alongside
the Web source. It reports selected/configured/running release identities and
opens one RViz window for the admitted session. Global/local are in-window
display/camera layouts; switching retains the same viewer process and does not
restart localization, the BT or planning. A namespace-wide viewer lock also
prevents separate CLI invocations from creating a second window. Closing RViz
does not cancel the task. Historical two-view units are cleanup-only.
The historical `/api/live-planning/start` endpoint is retired (410); it is not
a fallback when release validation fails. The navigation graph binds the
existing SDK session at startup; it never opens a second SDK connection.

The current Web configuration uses one `launch_policy: navigation`, not a fixed
preview/execution mode. The sealed `runtime_motion_profile.json` supplies the
same motion parameters to SCAN, the tracker, BT and the sole SDK writer. An
offline physical calibration report is no longer a startup requirement.
Runtime validity and physical acceptance are separate: vendor nominal speed
scales and configured planning/stopping margins are not measured guarantees.
The SDK's two published speed tables conflict; the default uses the larger
nominal scale per axis (1/1/2 for forward/lateral/yaw) and 0.3 m/s / 0.5 rad/s
software caps. Zero-command ACK and fresh stationary MC observations remain
required for a measured-stop report. Emergency stop, control loss, source age,
command expiry, route identity and collision behavior are not bypassed.
Legacy physical-record overrides remain compatible, but are not the default.
The formal tracking chain uses typed committed geometry and measured progress,
not the historical Go2 freeze Bool. A freshly re-certified actual-state entry
is ordered by task/generation/curve identity, not the old solve start clock.
Startup does not issue a goal, take control, or automatically execute a route;
the BT still owns the goal-bound execution confirmation in RViz. A running
session retains its admitted capability until stopped, irrespective of changes
to the next launch configuration.

Package READMEs under `src/` describe each stage; `docs/status/` has the
current state and open items.

Real robot bring-up order and the read-only readiness check:
`docs/design/REAL_ROBOT_BRINGUP.md` and
`python3 tools/diagnostics/robot_preflight.py` (uses the same default selector).
Before starting anything, `python3 tools/diagnostics/robot_preflight.py --files-only`
checks the sealed release and the actual Web/SDK launch bindings without reading
service state or contacting the robot. An optional `D1MAX_RELEASE` override is
explicit; it must still agree with the other launchers. A file-check pass is not
a navigation or physical-motion acceptance result.

The SDK monitor default is `@mainline`, resolved by this same selector. Both its
ELF and `monitor.yaml` come from that sealed bundle, not the mutable SDK workspace.
The Web activation is a separate private local configuration. On 2026-10-08 it
is bound to this default release for `planning_only`, because there is no valid
bound physical acceptance record. The interface reports this purpose instead
of presenting a planning preview as an executable robot task.

## Sensor assumptions

- Both Airy96 clouds use `x/y/z/intensity` FLOAT32, `ring` UINT16 and
  `timestamp` FLOAT64; timestamps are absolute seconds (the adapter also
  detects epoch ms/µs/ns).
- Robot TF provides the `rslidar_tail` → `rslidar_head` calibration.
- Front lidar IMU acceleration is in `g`; the adapter converts it.
