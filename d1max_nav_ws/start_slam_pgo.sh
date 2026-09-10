#!/usr/bin/env bash
set -eo pipefail

WS=/home/dndx/d1max_nav_ws
ROBOT_ENV="/home/dndx/智元四足机器人D1 Max二次开发文档资料包v0.1.0/d1max_ros2/d1max_ros2_env.sh"
GTSAM_PREFIX=/home/dndx/.local/ros-humble-gtsam/opt/ros/humble
USE_RVIZ="${1:-true}"
OUTPUT_DIR="${2:-$WS/maps}"
RUN_ID="${D1MAX_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
RUN_DIR="$OUTPUT_DIR/runs/$RUN_ID"

stop_previous_mapping() {
  pkill -TERM -f "ros2 launch d1max_slam mapping_pgo.launch.py" 2>/dev/null || true
  pkill -TERM -f "ros2 launch d1max_slam mapping.launch.py" 2>/dev/null || true
  for process in alaserPGO run_mapping_online dual_lidar_adapter map_capture_node rviz2; do
    pkill -TERM -x "$process" 2>/dev/null || true
  done
  pkill -TERM -f "__node:=d1max_lidar_level_calibration" 2>/dev/null || true
  pkill -TERM -f "__node:=d1max_airy_to_ros" 2>/dev/null || true
  pkill -TERM -f "__node:=d1max_lidar_extrinsic" 2>/dev/null || true
  sleep 2
  for process in alaserPGO run_mapping_online dual_lidar_adapter map_capture_node rviz2; do
    pkill -KILL -x "$process" 2>/dev/null || true
  done
}

ensure_zenoh_router() {
  if pgrep -x rmw_zenohd >/dev/null; then
    return
  fi
  nohup ros2 run rmw_zenoh_cpp rmw_zenohd \
    >/tmp/d1max_zenoh_router.log 2>&1 &
  sleep 2
  if ! pgrep -x rmw_zenohd >/dev/null; then
    echo "Failed to start rmw_zenohd; see /tmp/d1max_zenoh_router.log" >&2
    exit 1
  fi
}

stop_previous_mapping
source "$ROBOT_ENV"
ensure_zenoh_router
export CMAKE_PREFIX_PATH="$GTSAM_PREFIX:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$GTSAM_PREFIX/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
source "$WS/install/setup.bash"

mkdir -p "$RUN_DIR/sc_pgo"
ln -sfn "$RUN_DIR" "$OUTPUT_DIR/latest"
printf 'SLAM output: %s\n' "$RUN_DIR"
exec ros2 launch d1max_slam mapping_pgo.launch.py \
  use_rviz:="$USE_RVIZ" output_dir:="$RUN_DIR"
