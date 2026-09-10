#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_ENV="/home/dndx/智元四足机器人D1 Max二次开发文档资料包v0.1.0/d1max_ros2/d1max_ros2_env.sh"
BAG_PATH="${1:-latest}"
LIDAR_MODE="${2:-front}"
USE_RVIZ="${3:-true}"
PLAYBACK_RATE="${D1MAX_BAG_RATE:-1.0}"
READ_AHEAD_QUEUE_SIZE="${D1MAX_BAG_READ_AHEAD_QUEUE_SIZE:-5000}"
BAG_DOMAIN_ID="${D1MAX_BAG_DOMAIN_ID:-42}"
BAG_RMW="${D1MAX_BAG_RMW:-rmw_fastrtps_cpp}"
SAVE_SETTLE_SEC="${D1MAX_SAVE_SETTLE_SEC:-5}"
SAVE_SERVICE="/d1max/slam/save"

if [[ "$LIDAR_MODE" != "dual" && "$LIDAR_MODE" != "front" && "$LIDAR_MODE" != "rear" ]]; then
  echo "Usage: $0 [bag_path|latest] [dual|front|rear] [true|false]" >&2
  exit 2
fi
if [[ "$USE_RVIZ" != "true" && "$USE_RVIZ" != "false" ]]; then
  echo "use_rviz must be true or false" >&2
  exit 2
fi
if [[ ! "$READ_AHEAD_QUEUE_SIZE" =~ ^[1-9][0-9]*$ ]]; then
  echo "D1MAX_BAG_READ_AHEAD_QUEUE_SIZE must be a positive integer" >&2
  exit 2
fi

if [[ "$BAG_PATH" == "latest" ]]; then
  BAG_PATH="$(
    find "$ROOT/bags" -mindepth 2 -maxdepth 2 -name metadata.yaml \
      -printf '%T@ %h\n' | sort -nr | head -n 1 | cut -d' ' -f2-
  )"
fi
if [[ -z "$BAG_PATH" || ! -f "$BAG_PATH/metadata.yaml" ]]; then
  echo "Bag not found or not finalized: $BAG_PATH" >&2
  exit 1
fi
if [[ ! -f "$ROOT/install/setup.bash" ]]; then
  echo "Workspace is not built: $ROOT/install/setup.bash" >&2
  exit 1
fi
if pgrep -x run_mapping_online >/dev/null 2>&1; then
  echo "Faster-LIO is running; refusing to start FAST-LIO2." >&2
  exit 3
fi

set +u
source "$ROBOT_ENV"
export ROS_DOMAIN_ID="$BAG_DOMAIN_ID"
export RMW_IMPLEMENTATION="$BAG_RMW"
source "$ROOT/install/setup.bash"
set -u

ROUTER_PID=""
LAUNCH_PID=""
PLAY_PID=""
MAPPING_READY=false
SAVE_DONE=false
MAP_FILE=""

save_map() {
  local timeout_sec="${1:-60}"
  local result points bytes

  if [[ "$MAPPING_READY" != "true" || -z "$LAUNCH_PID" ]] || ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "Cannot save: the mapping launch is not running." >&2
    return 1
  fi

  echo "Requesting an explicit map save through $SAVE_SERVICE ..."
  if ! result="$(timeout "${timeout_sec}s" ros2 service call \
      "$SAVE_SERVICE" std_srvs/srv/Trigger '{}' 2>&1)"; then
    echo "$result" >&2
    echo "Map save service failed or timed out." >&2
    return 1
  fi
  echo "$result"
  if ! grep -Eq 'success=(True|true)|success: true' <<<"$result"; then
    echo "Map save service returned failure." >&2
    return 1
  fi

  MAP_FILE="$(
    find "$OUTPUT_DIR" -maxdepth 1 -type f -name 'd1max_map_*.pcd' \
      -printf '%T@ %p\n' | sort -nr | head -n 1 | cut -d' ' -f2-
  )"
  if [[ -z "$MAP_FILE" || ! -s "$MAP_FILE" ]]; then
    echo "Map service reported success, but no non-empty PCD exists in $OUTPUT_DIR" >&2
    return 1
  fi

  points="$(LC_ALL=C sed -n 's/^POINTS //p;/^DATA /q' "$MAP_FILE" | head -n 1)"
  bytes="$(stat -c '%s' "$MAP_FILE")"
  if [[ ! "$points" =~ ^[1-9][0-9]*$ ]]; then
    echo "Invalid PCD header in $MAP_FILE (POINTS=$points)" >&2
    return 1
  fi

  SAVE_DONE=true
  printf 'MAP_SAVED=%s\nMAP_POINTS=%s\nMAP_BYTES=%s\n' "$MAP_FILE" "$points" "$bytes"
}

cleanup() {
  trap - INT TERM EXIT
  if [[ -n "$PLAY_PID" ]] && kill -0 "$PLAY_PID" 2>/dev/null; then
    kill -INT "$PLAY_PID" 2>/dev/null || true
    wait "$PLAY_PID" 2>/dev/null || true
  fi
  if [[ "$SAVE_DONE" != "true" && "$MAPPING_READY" == "true" ]]; then
    save_map 30 || true
  fi
  if [[ -n "$LAUNCH_PID" ]] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    kill -INT "$LAUNCH_PID" 2>/dev/null || true
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi
  if [[ -n "$ROUTER_PID" ]] && kill -0 "$ROUTER_PID" 2>/dev/null; then
    kill -INT "$ROUTER_PID" 2>/dev/null || true
    wait "$ROUTER_PID" 2>/dev/null || true
  fi
}
trap cleanup INT TERM EXIT

if [[ "$RMW_IMPLEMENTATION" == "rmw_zenoh_cpp" ]] && \
   ! (exec 3<>/dev/tcp/127.0.0.1/7447) 2>/dev/null; then
  ros2 run rmw_zenoh_cpp rmw_zenohd >"/tmp/d1max_bag_zenoh_${$}.log" 2>&1 &
  ROUTER_PID=$!
  for _ in {1..30}; do
    (exec 3<>/dev/tcp/127.0.0.1/7447) 2>/dev/null && break
    kill -0 "$ROUTER_PID" 2>/dev/null || {
      echo "Failed to start the bag Zenoh router." >&2
      exit 1
    }
    sleep 0.2
  done
fi

STAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT_DIR="$ROOT/maps/fastlio2_${LIDAR_MODE}_bag_${STAMP}"
mkdir -p "$OUTPUT_DIR"

echo "Bag:        $BAG_PATH"
echo "LiDAR:      $LIDAR_MODE"
echo "IMU:        /front_lidar/imu"
echo "RMW:        $RMW_IMPLEMENTATION"
echo "ROS domain: $ROS_DOMAIN_ID"
echo "Read ahead: $READ_AHEAD_QUEUE_SIZE messages"
echo "Map output: $OUTPUT_DIR"

ros2 launch d1max_slam mapping.launch.py \
  backend:=fastlio2 lidar_mode:="$LIDAR_MODE" use_rviz:="$USE_RVIZ" \
  output_dir:="$OUTPUT_DIR" &
LAUNCH_PID=$!
sleep 5
if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
  echo "FAST-LIO2 launch exited during startup." >&2
  wait "$LAUNCH_PID"
  exit 1
fi
MAPPING_READY=true

PLAY_TOPICS=(/front_lidar/imu)
case "$LIDAR_MODE" in
  dual) PLAY_TOPICS+=(/front_lidar /rear_lidar) ;;
  front) PLAY_TOPICS+=(/front_lidar) ;;
  rear) PLAY_TOPICS+=(/rear_lidar) ;;
esac

ros2 bag play "$BAG_PATH" --rate "$PLAYBACK_RATE" \
  --read-ahead-queue-size "$READ_AHEAD_QUEUE_SIZE" \
  --disable-keyboard-controls --topics "${PLAY_TOPICS[@]}" &
PLAY_PID=$!
set +e
wait "$PLAY_PID"
PLAY_STATUS=$?
set -e
PLAY_PID=""
if [[ "$PLAY_STATUS" -ne 0 ]]; then
  echo "Bag playback failed with status $PLAY_STATUS." >&2
  exit "$PLAY_STATUS"
fi

echo "Bag playback finished; waiting ${SAVE_SETTLE_SEC}s for final registered clouds."
sleep "$SAVE_SETTLE_SEC"
save_map 60

if [[ "$USE_RVIZ" == "true" ]]; then
  echo "PCD is already saved. Inspect RViz, then press Ctrl+C to stop."
  wait "$LAUNCH_PID" || true
else
  cleanup
  trap - INT TERM EXIT
fi
