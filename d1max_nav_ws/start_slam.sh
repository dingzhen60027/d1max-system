#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BACKEND="${1:-faster_lio}"
USE_RVIZ="${2:-true}"
ROBOT_ENV="${D1MAX_ROS2_ENV:-/home/dndx/智元四足机器人D1 Max二次开发文档资料包v0.1.0/d1max_ros2/d1max_ros2_env.sh}"

if [[ ! -f "$ROOT/install/setup.bash" ]]; then
  echo "Workspace is not built. Run: $ROOT/build.sh" >&2
  exit 1
fi
if [[ ! -f "$ROBOT_ENV" ]]; then
  echo "Robot ROS environment not found: $ROBOT_ENV" >&2
  exit 1
fi
if [[ "$BACKEND" != "faster_lio" && "$BACKEND" != "fastlio2" ]]; then
  echo "Usage: $0 [faster_lio|fastlio2] [true|false]" >&2
  exit 2
fi

find_d1max_slam_pids() {
  local mode="${1:-all}"
  local proc pid executable arg index is_launch is_project is_static is_rviz
  local -a argv

  for proc in /proc/[0-9]*; do
    pid="${proc##*/}"
    [[ "$pid" == "$$" || "$pid" == "$PPID" ]] && continue
    argv=()
    mapfile -d '' -t argv 2>/dev/null < "$proc/cmdline" || true
    ((${#argv[@]})) || continue

    is_launch=false
    for ((index = 0; index + 3 < ${#argv[@]}; ++index)); do
      if [[ "${argv[index]##*/}" == "ros2" &&
            "${argv[index + 1]}" == "launch" &&
            "${argv[index + 2]}" == "d1max_slam" &&
            "${argv[index + 3]}" == "mapping.launch.py" ]]; then
        is_launch=true
        break
      fi
    done
    if [[ "$is_launch" == true ]]; then
      printf '%s\n' "$pid"
      continue
    fi
    [[ "$mode" == "launch" ]] && continue

    is_project=false
    for arg in "${argv[@]}"; do
      if [[ "$arg" == "$ROOT/install/"* ]]; then
        executable="${arg##*/}"
        case "$executable" in
          dual_lidar_adapter|map_capture_node|run_mapping_online|lio_node)
            is_project=true
            ;;
        esac
      fi
    done

    is_static=false
    is_rviz=false
    executable="${argv[0]##*/}"
    if [[ "$executable" == "static_transform_publisher" ]]; then
      for arg in "${argv[@]}"; do
        case "$arg" in
          __node:=d1max_lidar_level_calibration|__node:=d1max_airy_to_ros|__node:=d1max_lidar_extrinsic)
            is_static=true
            ;;
        esac
      done
    elif [[ "$executable" == "rviz2" ]]; then
      for arg in "${argv[@]}"; do
        [[ "$arg" == "$ROOT/"* ]] && is_rviz=true
      done
    fi

    if [[ "$is_project" == true || "$is_static" == true || "$is_rviz" == true ]]; then
      printf '%s\n' "$pid"
    fi
  done
}

wait_for_old_slam() {
  local timeout_sec="$1"
  local deadline=$((SECONDS + timeout_sec))
  local -a remaining
  while ((SECONDS < deadline)); do
    mapfile -t remaining < <(find_d1max_slam_pids all)
    ((${#remaining[@]} == 0)) && return 0
    sleep 0.2
  done
  return 1
}

cleanup_previous_slam() {
  local -a launch_pids remaining
  mapfile -t launch_pids < <(find_d1max_slam_pids launch)
  if ((${#launch_pids[@]})); then
    echo "Stopping previous D1 Max SLAM launch: ${launch_pids[*]}"
    kill -INT "${launch_pids[@]}" 2>/dev/null || true
    wait_for_old_slam 12 || true
  fi

  mapfile -t remaining < <(find_d1max_slam_pids all)
  if ((${#remaining[@]})); then
    echo "Terminating stale D1 Max SLAM processes: ${remaining[*]}"
    kill -TERM "${remaining[@]}" 2>/dev/null || true
    wait_for_old_slam 3 || true
  fi

  mapfile -t remaining < <(find_d1max_slam_pids all)
  if ((${#remaining[@]})); then
    echo "Killing unresponsive D1 Max SLAM processes: ${remaining[*]}"
    kill -KILL "${remaining[@]}" 2>/dev/null || true
    wait_for_old_slam 1 || true
  fi

  mapfile -t remaining < <(find_d1max_slam_pids all)
  if ((${#remaining[@]})); then
    echo "Unable to clean previous D1 Max SLAM processes: ${remaining[*]}" >&2
    return 1
  fi
}

cleanup_previous_slam

mkdir -p "$ROOT/maps"
source "$ROBOT_ENV"
source "$ROOT/install/setup.bash"
exec ros2 launch d1max_slam mapping.launch.py \
  backend:="$BACKEND" use_rviz:="$USE_RVIZ" output_dir:="$ROOT/maps"
