#!/usr/bin/env bash
set -eo pipefail
source '/home/dndx/智元四足机器人D1 Max二次开发文档资料包v0.1.0/d1max_ros2/d1max_ros2_env.sh'
source /home/dndx/d1max_nav_ws/install/setup.bash
exec python3 /home/dndx/d1max_nav_ws/src/d1max_pct_scan/d1max_pct_scan/session.py "$@"
