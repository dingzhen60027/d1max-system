#!/usr/bin/env bash
set -euo pipefail
D1MAX_PANEL_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${D1MAX_PANEL_DIR}/../d1max_ros2_env.sh"
export ZENOH_SESSION_CONFIG_URI="${D1MAX_PANEL_DIR}/config/zenoh-local.json5"
# Dedicated port: does not replace the user's existing playback bridge.
# Replay viewer is entirely read-only. No controls or estop requests from a bag.
exec ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
  address:=127.0.0.1 port:=8767 num_threads:=4 max_qos_depth:=100 \
  use_sim_time:=false send_buffer_limit:=50000000 \
  capabilities:='[connectionGraph]' \
  client_topic_whitelist:="['a^']" service_whitelist:="['a^']" param_whitelist:="['a^']"
