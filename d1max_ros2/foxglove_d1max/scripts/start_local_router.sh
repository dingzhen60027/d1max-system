#!/usr/bin/env bash
set -euo pipefail
D1MAX_PANEL_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${D1MAX_PANEL_DIR}/../d1max_ros2_env.sh"
export ZENOH_ROUTER_CONFIG_URI="${D1MAX_PANEL_DIR}/config/zenoh-router-local.json5"
exec ros2 run rmw_zenoh_cpp rmw_zenohd
