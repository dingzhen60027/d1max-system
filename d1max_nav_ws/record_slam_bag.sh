#!/usr/bin/env bash
set -eo pipefail

# Record the raw D1 Max sensor data required to rerun the complete SLAM input
# pipeline. Processed LIO topics are intentionally not required: the raw bag can
# be replayed through dual_lidar_adapter and either LIO backend.

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_ENV="/home/dndx/智元四足机器人D1 Max二次开发文档资料包v0.1.0/d1max_ros2/d1max_ros2_env.sh"
OUTPUT_ROOT="${D1MAX_BAG_DIR:-${WS_DIR}/bags}"
LABEL="${1:-slam_raw}"
WAIT_SECONDS="${D1MAX_BAG_WAIT_SECONDS:-30}"
MIN_FREE_GIB="${D1MAX_BAG_MIN_FREE_GIB:-5}"

# Keep the output directory name shell-safe.
LABEL="${LABEL//[^a-zA-Z0-9_-]/_}"
STAMP="$(date +%Y%m%d_%H%M%S)"
BAG_PATH="${OUTPUT_ROOT}/${LABEL}_${STAMP}"
INFO_PATH="${BAG_PATH}.info.txt"

REQUIRED_TOPICS=(
  /front_lidar
  /rear_lidar
  /front_lidar/imu
  /rear_lidar/imu
)

OPTIONAL_TOPICS=(
  /tf
  /tf_static
)

ALL_TOPICS=("${REQUIRED_TOPICS[@]}" "${OPTIONAL_TOPICS[@]}")

source /opt/ros/humble/setup.bash
if [[ ! -f "$ROBOT_ENV" ]]; then
  echo "ERROR: robot ROS environment not found: $ROBOT_ENV" >&2
  exit 1
fi
source "$ROBOT_ENV"

mkdir -p "$OUTPUT_ROOT"

if pgrep -af '[r]os2 bag record' >/dev/null; then
  echo "ERROR: another ros2 bag recorder is already running:" >&2
  pgrep -af '[r]os2 bag record' >&2 || true
  exit 1
fi

AVAILABLE_KIB="$(df -Pk "$OUTPUT_ROOT" | awk 'NR == 2 {print $4}')"
AVAILABLE_GIB=$((AVAILABLE_KIB / 1024 / 1024))
if (( AVAILABLE_GIB < MIN_FREE_GIB )); then
  echo "ERROR: only ${AVAILABLE_GIB} GiB is free; at least ${MIN_FREE_GIB} GiB is required." >&2
  exit 1
fi

ESTIMATED_MINUTES=$((AVAILABLE_GIB * 1024 / 42 / 60))
echo "Output: $BAG_PATH"
echo "Free disk: ${AVAILABLE_GIB} GiB (roughly ${ESTIMATED_MINUTES} minutes at 42 MiB/s)"
echo "Waiting for all required robot sensor publishers..."

wait_for_publisher() {
  local topic="$1"
  local deadline=$((SECONDS + WAIT_SECONDS))
  local count="0"

  while (( SECONDS < deadline )); do
    count="$(ros2 topic info "$topic" 2>/dev/null | awk '/^Publisher count:/ {print $3}')"
    if [[ "$count" =~ ^[1-9][0-9]*$ ]]; then
      printf '  %-24s publisher_count=%s\n' "$topic" "$count"
      return 0
    fi
    sleep 1
  done

  echo "ERROR: no publisher found for required topic $topic after ${WAIT_SECONDS}s." >&2
  return 1
}

for topic in "${REQUIRED_TOPICS[@]}"; do
  wait_for_publisher "$topic"
done

echo "Probing one real message from every required topic..."
declare -a PROBE_PIDS=()
declare -a PROBE_TOPICS=()
for topic in "${REQUIRED_TOPICS[@]}"; do
  timeout -s INT -k 2 12 \
    ros2 topic echo "$topic" --once --no-arr --qos-reliability best_effort \
    >/dev/null 2>&1 &
  PROBE_PIDS+=("$!")
  PROBE_TOPICS+=("$topic")
done

PROBE_FAILED=0
for index in "${!PROBE_PIDS[@]}"; do
  if wait "${PROBE_PIDS[$index]}"; then
    printf '  %-24s message_received=yes\n' "${PROBE_TOPICS[$index]}"
  else
    echo "ERROR: topic ${PROBE_TOPICS[$index]} has a publisher but no message was received." >&2
    PROBE_FAILED=1
  fi
done

if (( PROBE_FAILED != 0 )); then
  echo "Recording aborted: the bag would not contain a complete SLAM input." >&2
  exit 1
fi

QOS_FILE="$(mktemp /tmp/d1max-slam-bag-qos.XXXXXX.yaml)"
trap 'rm -f "$QOS_FILE"' EXIT
cat > "$QOS_FILE" <<'QOS'
/front_lidar:
  history: keep_last
  depth: 10
  reliability: reliable
  durability: volatile
/rear_lidar:
  history: keep_last
  depth: 10
  reliability: reliable
  durability: volatile
/front_lidar/imu:
  history: keep_last
  depth: 1000
  reliability: reliable
  durability: volatile
/rear_lidar/imu:
  history: keep_last
  depth: 1000
  reliability: reliable
  durability: volatile
/tf:
  history: keep_last
  depth: 100
  reliability: reliable
  durability: volatile
/tf_static:
  history: keep_last
  depth: 1
  reliability: reliable
  durability: transient_local
QOS

{
  echo "D1 Max raw SLAM bag"
  echo "started_at=$(date --iso-8601=seconds)"
  echo "hostname=$(hostname)"
  echo "output=$BAG_PATH"
  echo "storage=sqlite3"
  echo "compression=none"
  echo "split_size_bytes=17179869184"
  echo "max_cache_size_bytes=536870912"
  echo "free_disk_gib=$AVAILABLE_GIB"
  echo "required_topics=${REQUIRED_TOPICS[*]}"
  echo "optional_topics=${OPTIONAL_TOPICS[*]}"
  echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-unset}"
  echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset}"
  echo "warning=robot sensor time and PC wall time must be reconciled before mixing PC-stamped TF"
} > "$INFO_PATH"

echo
echo "All four raw SLAM streams are live. Recording now."
echo "Press Ctrl+C once to stop cleanly. Do not power off during bag finalization."
echo

STATUS=0
ros2 bag record \
  --storage sqlite3 \
  --output "$BAG_PATH" \
  --max-bag-size 17179869184 \
  --max-cache-size 536870912 \
  --compression-mode none \
  --qos-profile-overrides-path "$QOS_FILE" \
  --include-unpublished-topics \
  "${ALL_TOPICS[@]}" || STATUS=$?

{
  echo "stopped_at=$(date --iso-8601=seconds)"
  echo "recorder_exit_status=$STATUS"
  echo
  ros2 bag info "$BAG_PATH" 2>&1 || true
} >> "$INFO_PATH"

echo
echo "Bag:  $BAG_PATH"
echo "Info: $INFO_PATH"

if (( STATUS != 0 && STATUS != 130 )); then
  echo "ERROR: rosbag recorder exited with status $STATUS. Inspect the info file." >&2
  exit "$STATUS"
fi

