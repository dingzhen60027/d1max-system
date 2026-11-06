#!/usr/bin/env bash
set -euo pipefail
D1MAX_MONITOR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# Resolve the shared default selector and verify the sealed SDK before sourcing
# any ROS or SDK overlay. This read-only action cannot connect to the robot.
D1MAX_MONITOR_RELEASE="$(/usr/bin/python3 "${D1MAX_MONITOR_DIR}/scripts/sdk_motion_startup.py" resolve-release)"
export D1MAX_MONITOR_RELEASE
source "${D1MAX_MONITOR_DIR}/../d1max_ros2_env.sh"
export ZENOH_SESSION_CONFIG_URI="${D1MAX_MONITOR_DIR}/config/zenoh-live.json5"
# The default is the unique navigation release. Explicit absolute-path
# overrides still select a sealed release, never the old workspace ELF.
if [[ -n "${D1MAX_MONITOR_RELEASE:-}" ]]; then
  export D1MAX_MONITOR_RELEASE
  set +u
  for D1MAX_RELEASE_COMPONENT in interfaces sdk; do
    source "${D1MAX_MONITOR_RELEASE}/${D1MAX_RELEASE_COMPONENT}/install/local_setup.bash"
  done
  set -u
fi
# The helper holds a process-lifetime lock, retains the ROS duplicate check and
# consumes legacy opt-in only inside the existing managed service invocation.
# The selected mainline starts task_follower_v1 only; it does not take control
# until the BT's one task grant, and never starts a second SDK writer.
exec python3 "${D1MAX_MONITOR_DIR}/scripts/sdk_motion_startup.py" _launch
