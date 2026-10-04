#!/usr/bin/env bash
set -eo pipefail

source "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")/../d1max_env.sh"
WS_DIR="$D1MAX_NAV_ROOT"
ROBOT_ENV="$D1MAX_ROS2_ENV"
BACKEND="${1:-faster_lio}"
USE_RVIZ="${2:-true}"

case "$BACKEND" in
  faster_lio|fastlio2) ;;
  *)
    echo "Usage: $0 [faster_lio|fastlio2] [true|false] [launch arguments...]" >&2
    exit 2
    ;;
esac
shift || true
shift || true

if pgrep -af '[s]can_planner_node' >/dev/null; then
  echo "SCAN-Planner is already running; stop that launch before starting another." >&2
  pgrep -af '[s]can_planner_node' >&2 || true
  exit 1
fi

source /opt/ros/humble/setup.bash
if [[ -f "$ROBOT_ENV" ]]; then
  source "$ROBOT_ENV"
fi
if [[ ! -f "$WS_DIR/install/setup.bash" ]]; then
  echo "Workspace is not built: $WS_DIR/install/setup.bash is missing." >&2
  exit 1
fi
source "$WS_DIR/install/setup.bash"

exec ros2 launch d1max_scan_planner scan_planner.launch.py \
  backend:="$BACKEND" use_rviz:="$USE_RVIZ" "$@"
