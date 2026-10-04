#!/usr/bin/env bash
# Official, version-pinned single_floor_session entry. Never connects the SDK.
# Roots come from the environment (see deploy/d1max.env.example); the release
# names its own sealed manifest in <release>/release.json.
set -eo pipefail
if [[ "${BASH_SOURCE[0]}" == *"/source_snapshot/"* ]] && [ -z "${D1MAX_NAV_ROOT:-}" ]; then
  echo 'frozen_entry_requires_explicit_D1MAX_NAV_ROOT_actual_workspace' >&2
  exit 2
fi
: "${D1MAX_NAV_ROOT:=$(cd -- "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")/.." && pwd)}"
: "${D1MAX_APP_ROOT:=${D1MAX_VENDOR_ROOT:-$HOME/智元四足机器人D1 Max二次开发文档资料包v0.1.0}/d1max_ros2}"
# An explicit release overrides the checked-in selection (e.g. an isolated
# regression bundle). Never choose the newest directory by timestamp.
ENTRY_SCRIPT_DIR=$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")
ENTRY_PREFLIGHT="$ENTRY_SCRIPT_DIR/release/single_floor_entry_preflight.py"
if [ -z "${D1MAX_RELEASE:-}" ]; then
  D1MAX_RELEASE=$(/usr/bin/python3 "$ENTRY_PREFLIGHT" select --nav-root "$D1MAX_NAV_ROOT")
fi
export D1MAX_NAV_ROOT D1MAX_APP_ROOT D1MAX_RELEASE
ENTRY_TOOLS=$(/usr/bin/python3 "$ENTRY_PREFLIGHT" tools-root --release "$D1MAX_RELEASE" --nav-root "$D1MAX_NAV_ROOT")
if [ "$(readlink -f -- "${BASH_SOURCE[0]}")" != "$ENTRY_TOOLS/single_floor_entry.sh" ]; then
  exec bash "$ENTRY_TOOLS/single_floor_entry.sh" "$@"
fi
ENTRY_PREFLIGHT="$ENTRY_TOOLS/release/single_floor_entry_preflight.py"
export PYTHONPATH=
# rmw_zenoh_cpp is installed in the application's local ROS prefix, not /opt.
# Do not source d1max_ros2_env.sh: that script overwrites the isolated domain
# and router configuration. Load libraries only, then select transport below.
: "${D1MAX_RMW_PREFIX:=$D1MAX_APP_ROOT/local/opt/ros/humble}"
export D1MAX_RMW_PREFIX
# Pure-stdlib inventory check runs before ANY ROS/base/overlay setup code.
# A new candidate must already be sealed; this is not a malicious-code sandbox.
/usr/bin/python3 "$ENTRY_PREFLIGHT" startup-check --release "$D1MAX_RELEASE"
source /opt/ros/humble/setup.bash
if [ ! -f "$D1MAX_RMW_PREFIX/lib/librmw_zenoh_cpp.so" ]; then
  echo "zenoh_runtime_missing: $D1MAX_RMW_PREFIX/lib/librmw_zenoh_cpp.so" >&2
  exit 2
fi
export AMENT_PREFIX_PATH="$D1MAX_RMW_PREFIX${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
export CMAKE_PREFIX_PATH="$D1MAX_RMW_PREFIX${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="$D1MAX_RMW_PREFIX/lib:$D1MAX_RMW_PREFIX/opt/zenoh_cpp_vendor/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export PYTHONPATH="$D1MAX_RMW_PREFIX/lib/python3.10/site-packages${PYTHONPATH:+:$PYTHONPATH}"
export PATH="$D1MAX_RMW_PREFIX/bin:$D1MAX_RMW_PREFIX/lib/rmw_zenoh_cpp:$PATH"
source "$D1MAX_NAV_ROOT/install/local_setup.bash"
for component in interfaces native tracker bt sdk rviz application; do
  if [ "$component" = application ]; then
    ENTRY_LOCALIZATION_DEPENDENCIES=$(/usr/bin/python3 "$ENTRY_PREFLIGHT" localization-dependencies --release "$D1MAX_RELEASE")
    if [ -n "$ENTRY_LOCALIZATION_DEPENDENCIES" ]; then
      source "$ENTRY_LOCALIZATION_DEPENDENCIES/local_setup.bash"
    fi
  fi
  source "$D1MAX_RELEASE/$component/install/local_setup.bash"
done
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
if [ "${D1MAX_NAV_TRANSPORT:-}" != isolated_mock ]; then
  export ROS_DOMAIN_ID="${D1MAX_ROS_DOMAIN_ID:-24}"
  export D1MAX_NAV_TRANSPORT=live
  export ZENOH_SESSION_CONFIG_URI="$D1MAX_APP_ROOT/foxglove_d1max/config/zenoh-live.json5"
  unset ZENOH_CONFIG_OVERRIDE ZENOH_SESSION_CONFIG ZENOH_ROUTER_CONFIG D1MAX_OFFLINE_ZENOH_TEST
fi
/usr/bin/python3 "$ENTRY_PREFLIGHT" verify --release "$D1MAX_RELEASE"
if [ "${1:-}" = --check-release ]; then
  exit 0
fi
exec /usr/bin/python3 -m d1max_pct_scan.single_floor_session "$@"
