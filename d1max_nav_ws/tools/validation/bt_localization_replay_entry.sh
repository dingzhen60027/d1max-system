#!/usr/bin/env bash
# Replay the selected version-consistent bundle. Never enters the SDK path.
set -eo pipefail
if [[ "${BASH_SOURCE[0]}" == *"/source_snapshot/"* ]] && [ -z "${D1MAX_NAV_ROOT:-}" ]; then
  echo 'frozen_replay_requires_explicit_D1MAX_NAV_ROOT_actual_workspace' >&2
  exit 2
fi
REPLAY_SCRIPT_DIR=$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")
REPLAY_WS=${D1MAX_NAV_ROOT:-$(cd -- "$REPLAY_SCRIPT_DIR/../.." && pwd)}
export D1MAX_NAV_ROOT="$REPLAY_WS"
REPLAY_PREFLIGHT="$REPLAY_SCRIPT_DIR/../release/single_floor_entry_preflight.py"
if [ -z "${D1MAX_RELEASE:-}" ]; then
  D1MAX_RELEASE=$(/usr/bin/python3 "$REPLAY_PREFLIGHT" select --nav-root "$REPLAY_WS")
fi
export D1MAX_RELEASE
REPLAY_TOOLS=$(/usr/bin/python3 "$REPLAY_PREFLIGHT" tools-root --release "$D1MAX_RELEASE" --nav-root "$REPLAY_WS")
if [ "$(readlink -f -- "${BASH_SOURCE[0]}")" != "$REPLAY_TOOLS/validation/bt_localization_replay_entry.sh" ]; then
  exec bash "$REPLAY_TOOLS/validation/bt_localization_replay_entry.sh" "$@"
fi
REPLAY_PREFLIGHT="$REPLAY_TOOLS/release/single_floor_entry_preflight.py"
export PYTHONPATH=
REPLAY_RMW="${D1MAX_RMW_PREFIX:-/home/dndx/智元四足机器人D1 Max二次开发文档资料包v0.1.0/d1max_ros2/local/opt/ros/humble}"
export D1MAX_RMW_PREFIX="$REPLAY_RMW"
/usr/bin/python3 "$REPLAY_PREFLIGHT" startup-check --release "$D1MAX_RELEASE"
source /opt/ros/humble/setup.bash
export AMENT_PREFIX_PATH="$REPLAY_RMW:$AMENT_PREFIX_PATH"
export LD_LIBRARY_PATH="$REPLAY_RMW/lib:$REPLAY_RMW/opt/zenoh_cpp_vendor/lib:$LD_LIBRARY_PATH"
export PATH="$REPLAY_RMW/bin:$REPLAY_RMW/lib/rmw_zenoh_cpp:$PATH"
source "$REPLAY_WS/install/local_setup.bash"
for component in interfaces native tracker bt sdk rviz application; do
  if [ "$component" = application ]; then
    REPLAY_LOCALIZATION_DEPENDENCIES=$(/usr/bin/python3 "$REPLAY_PREFLIGHT" localization-dependencies --release "$D1MAX_RELEASE")
    if [ -n "$REPLAY_LOCALIZATION_DEPENDENCIES" ]; then
      source "$REPLAY_LOCALIZATION_DEPENDENCIES/local_setup.bash"
    fi
  fi
  source "$D1MAX_RELEASE/$component/install/local_setup.bash"
done
# A source checkout or a separate development prefix must not override the
# localization selected with the other components of this bundle.
export PYTHONPATH="$(dirname -- "$REPLAY_TOOLS"):$PYTHONPATH"
/usr/bin/python3 "$REPLAY_PREFLIGHT" verify --release "$D1MAX_RELEASE"
/usr/bin/python3 -c 'import json,os; from pathlib import Path; p=Path(os.environ["D1MAX_RELEASE"]); d=json.loads((p/"release.json").read_text()); assert d.get("local_state_contract")=="continuous_odom_v1", "replay_requires_the_continuous_local_state_bundle"'
exec /usr/bin/python3 "$REPLAY_TOOLS/validation/run_bt_localization_replay.py" "$@"
