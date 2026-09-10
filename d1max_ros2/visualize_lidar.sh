#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
URDF_PREFIX="${SCRIPT_DIR}/urdf_ws/install/max_description"
ASCII_URDF_PREFIX="/tmp/d1max_max_description"

source "${SCRIPT_DIR}/d1max_ros2_env.sh"

case "$-" in
  *u*) RESTORE_NOUNSET=1; set +u ;;
  *) RESTORE_NOUNSET=0 ;;
esac
source "${SCRIPT_DIR}/urdf_ws/install/setup.bash"
if [[ "${RESTORE_NOUNSET}" -eq 1 ]]; then
  set -u
fi
unset RESTORE_NOUNSET

# RViz resource_retriever cannot reliably parse file URLs containing spaces or
# non-ASCII characters. Keep an ASCII alias as the first ament package prefix.
ln -sfn "${URDF_PREFIX}" "${ASCII_URDF_PREFIX}"
export AMENT_PREFIX_PATH="${ASCII_URDF_PREFIX}:${AMENT_PREFIX_PATH}"
export CMAKE_PREFIX_PATH="${ASCII_URDF_PREFIX}:${CMAKE_PREFIX_PATH}"

if ! (exec 3<>/dev/tcp/127.0.0.1/7447) 2>/dev/null; then
  "${SCRIPT_DIR}/start.sh" >/dev/null
else
  exec 3>&-
fi

exec ros2 launch max_description d1max_lidar_rviz.launch.py
