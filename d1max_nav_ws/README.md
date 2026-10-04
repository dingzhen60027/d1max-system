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

Work continues in the current checkout; no new Git branch or parallel navigation
project is created without an explicit user request. Isolated builds, regression
fixtures, release bundles and rollback snapshots are validation/version artifacts
of this same mainline, not alternative architectures. See [AGENTS.md](AGENTS.md)
for the project working rules and [current status](docs/status/PROJECT_TAKEOVER_STATUS_20260928.md)
for the dated implementation and acceptance boundaries.

Choosing the development mainline does not switch the production release or
grant SDK motion authority. The default release selector remains unchanged;
deployment and physical acceptance are separate steps.

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
session also requires the current SDK session identity and a verified physical
record. Startup never connects/reconnects SDK, sets posture, or executes a goal.
The 0.3 m/s limit and all existing acceptance checks remain unchanged.

`scripts/planning/start_live_planning_view.sh`, the old motion coordinator and
the old Nav2 launcher are retained as legacy workflows, not official motion
entry points. The Web navigation launcher uses `/api/navigation-session`; the
old `/api/single-floor` URL is an alias of the same runtime/lock, not a second service. Its private
activation must bind this release, the current SDK session and, for execution,
the physical record. Selecting a release is not activating an unavailable record.

The Web's `/#/planning` page now has one mainline lifecycle entry, not a
preview/navigation selector. Its built static assets must be rebuilt alongside
the Web source. It reports selected/configured/running release identities and
opens one RViz window for the admitted session. Global/local are in-window
display/camera layouts; switching retains the same viewer process and does not
restart localization, the BT or planning. A namespace-wide viewer lock also
prevents separate CLI invocations from creating a second window. Closing RViz
does not cancel the task. Historical two-view units are cleanup-only.
The historical `/api/live-planning/start` endpoint is retired (410); it is not
a fallback when release validation fails. For `planning_only`, explicit
`bind_current_on_start` may capture the existing SDK data session at startup;
`execution` retains its fixed session and physical-record requirements.

Package READMEs under `src/` describe each stage; `docs/status/` has the
current state and open items.

Real robot bring-up order and the read-only readiness check:
`docs/design/REAL_ROBOT_BRINGUP.md` and
`D1MAX_RELEASE=<release> python3 tools/diagnostics/robot_preflight.py`.

## Sensor assumptions

- Both Airy96 clouds use `x/y/z/intensity` FLOAT32, `ring` UINT16 and
  `timestamp` FLOAT64; timestamps are absolute seconds (the adapter also
  detects epoch ms/µs/ns).
- Robot TF provides the `rslidar_tail` → `rslidar_head` calibration.
- Front lidar IMU acceleration is in `g`; the adapter converts it.
