#!/usr/bin/env bash
set -euo pipefail
D1MAX_PANEL_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${D1MAX_PANEL_DIR}/../d1max_ros2_env.sh"
export ZENOH_SESSION_CONFIG_URI="${D1MAX_PANEL_DIR}/config/zenoh-live.json5"
# Dedicated LIVE viewer: no client publishing, services or parameter writes.
# Large original meshes use the dedicated local HTTP server, not this bridge.
exec ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
  address:=127.0.0.1 port:=8769 num_threads:=4 min_qos_depth:=1 max_qos_depth:=2 \
  send_buffer_limit:=30000000 use_sim_time:=false \
  capabilities:='[connectionGraph]' \
  client_topic_whitelist:="['a^']" service_whitelist:="['a^']" param_whitelist:="['a^']" \
  topic_whitelist:="['^/(front|rear)_lidar(/imu)?$','^/(front|rear)_camera/image_compressed$','^/tf(_static)?$','^/joint_states$','^/d1max(_sdk_bridge|/console)/.*$','^/odom/current_pose$']" \
  asset_uri_allowlist:="['a^']" \
  sysinfo:=false publish_client_count:=false
