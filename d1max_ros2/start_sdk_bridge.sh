#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BRIDGE_SETUP="${SCRIPT_DIR}/sdk_bridge_ws/install/setup.bash"

source "${SCRIPT_DIR}/d1max_ros2_env.sh"

if [[ ! -f "${BRIDGE_SETUP}" ]]; then
  echo "桥接包尚未构建，请先运行："
  echo "  cd ${SCRIPT_DIR}/sdk_bridge_ws"
  echo "  colcon build --symlink-install"
  exit 1
fi

case "$-" in
  *u*) RESTORE_NOUNSET=1; set +u ;;
  *) RESTORE_NOUNSET=0 ;;
esac
source "${BRIDGE_SETUP}"
if [[ "${RESTORE_NOUNSET}" -eq 1 ]]; then
  set -u
fi
unset RESTORE_NOUNSET

exec ros2 launch d1max_sdk_bridge sdk_state_bridge.launch.py "$@"
