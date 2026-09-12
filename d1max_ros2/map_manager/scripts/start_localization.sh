#!/usr/bin/env bash
set -euo pipefail
D1MAX_LOCALIZATION_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${D1MAX_LOCALIZATION_SCRIPT_DIR}/../../d1max_ros2_env.sh"
set +u
source /home/dndx/d1max_nav_ws/install/setup.bash
set -u
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=24
export ZENOH_SESSION_CONFIG_URI="${D1MAX_LOCALIZATION_SCRIPT_DIR}/../../foxglove_d1max/config/zenoh-live.json5"
if [[ "$#" != 3 || ! -f "$1" || ! -d "$2" || ! -f "$3" ]]; then
  echo '需要配置文件、会话目录及定位 PCD。' >&2
  exit 2
fi
exec ros2 launch d1max_localization localization.launch.py "config:=$1" "session_dir:=$2" "map_pcd:=$3"
