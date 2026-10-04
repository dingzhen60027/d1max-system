#!/usr/bin/env bash
set -euo pipefail
D1MAX_MONITOR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${D1MAX_MONITOR_DIR}/../d1max_ros2_env.sh"
export ZENOH_SESSION_CONFIG_URI="${D1MAX_MONITOR_DIR}/config/zenoh-live.json5"
set +u
source "${D1MAX_MONITOR_DIR}/../sdk_bridge_ws/install/setup.bash"
set -u
# Opt-in sealed navigation release (schema-3 execution channel): the managed
# unit carries no per-release setting, so a one-line file names the release.
# Absent file and variable keep the workspace monitor unchanged.
D1MAX_MONITOR_RELEASE_FILE="${D1MAX_MONITOR_DIR}/config/monitor-release"
if [[ -z "${D1MAX_MONITOR_RELEASE:-}" && -f "${D1MAX_MONITOR_RELEASE_FILE}" ]]; then
  D1MAX_MONITOR_RELEASE="$(head -n 1 "${D1MAX_MONITOR_RELEASE_FILE}")"
fi
if [[ -n "${D1MAX_MONITOR_RELEASE:-}" ]]; then
  export D1MAX_MONITOR_RELEASE
  set +u
  for D1MAX_RELEASE_COMPONENT in interfaces sdk; do
    source "${D1MAX_MONITOR_RELEASE}/${D1MAX_RELEASE_COMPONENT}/install/local_setup.bash"
  done
  set -u
fi
# The helper holds a process-lifetime lock, retains the ROS duplicate check and
# consumes motion opt-in only inside the existing managed service invocation.
exec python3 "${D1MAX_MONITOR_DIR}/scripts/sdk_motion_startup.py" _launch
