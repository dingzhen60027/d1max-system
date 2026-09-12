#!/usr/bin/env bash
set -euo pipefail
D1MAX_MONITOR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${D1MAX_MONITOR_DIR}/../d1max_ros2_env.sh"
set +u
source /home/dndx/d1max_nav_ws/install/setup.bash
set -u
export ZENOH_SESSION_CONFIG_URI="${D1MAX_MONITOR_DIR}/config/zenoh-live.json5"
# Client publishing and parameter writes are absent. Only estop is callable.
exec ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
  address:=127.0.0.1 port:=8769 num_threads:=4 min_qos_depth:=1 max_qos_depth:=2 \
  send_buffer_limit:=60000000 use_sim_time:=false \
  capabilities:='[services,connectionGraph]' \
  client_topic_whitelist:="['a^']" param_whitelist:="['a^']" asset_uri_allowlist:="['a^']" \
  service_whitelist:="['^/d1max/monitor/s_[a-f0-9]{16}/soft_estop$']" \
  topic_whitelist:="['^/(front|rear)_lidar(/imu)?$','^/(front|rear)_camera/image_compressed$','^/tf(_static)?$','^/joint_states$','^/d1max_sdk_bridge/(robot_state|behavior_state|connection_state_text|transition_event|faults|velocity|speed_report_status)$','^/d1max/monitor/status$','^/d1max/maps/.*$','^/d1max/localization/.*$','^/odom/current_pose$']" \
  sysinfo:=false publish_client_count:=false
