#!/usr/bin/env bash
set -eo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
unset LD_LIBRARY_PATH PYTHONPATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
source "/home/dndx/智元四足机器人D1 Max二次开发文档资料包v0.1.0/d1max_ros2/d1max_ros2_env.sh"
source "$task_root/ws/install/local_setup.bash"
export RMW_IMPLEMENTATION=rmw_zenoh_cpp ROS_DOMAIN_ID=220
export ZENOH_SESSION_CONFIG_URI="$task_root/zenoh-session.json5"
export ZENOH_ROUTER_CONFIG_URI="$task_root/zenoh-router.json5"
export OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=1
exec /usr/bin/python3 "$task_root/run.py" "$@"
