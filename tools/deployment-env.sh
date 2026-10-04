#!/usr/bin/env bash
# Source this file. It sets paths only; never loads ROS, starts a service or SDK.
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  printf '%s\n' 'Use: source tools/deployment-env.sh' >&2
  exit 2
fi
D1MAX_DEPLOYMENT_REPO="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
export D1MAX_PROJECT_ROOT="${D1MAX_PROJECT_ROOT:-$D1MAX_DEPLOYMENT_REPO}"
export D1MAX_NAV_ROOT="${D1MAX_NAV_ROOT:-$D1MAX_DEPLOYMENT_REPO/d1max_nav_ws}"
export D1MAX_APP_ROOT="${D1MAX_APP_ROOT:-$D1MAX_DEPLOYMENT_REPO/d1max_ros2}"
export D1MAX_MAPS_ROOT="${D1MAX_MAPS_ROOT:-$D1MAX_NAV_ROOT/maps}"
export PCT_PLANNER_ROOT="${PCT_PLANNER_ROOT:-$D1MAX_NAV_ROOT/src/pct_planner_vendor}"
export D1MAX_ROS2_ENV="${D1MAX_ROS2_ENV:-$D1MAX_APP_ROOT/d1max_ros2_env.sh}"
export D1MAX_RMW_PREFIX="${D1MAX_RMW_PREFIX:-$D1MAX_APP_ROOT/local/opt/ros/humble}"
# No D1MAX_RELEASE default: the existing selector must fail if its pinned
# package is absent. Do not silently choose the newest build directory.
unset D1MAX_DEPLOYMENT_REPO
