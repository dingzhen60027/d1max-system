# Shared filesystem roots for shell entry points; source, do not execute.
# Every value may be overridden from the environment or an EnvironmentFile.
# Python code resolves the same names in d1max_pct_planner/paths.py.
: "${D1MAX_NAV_ROOT:=$(cd -- "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")/.." && pwd)}"
: "${D1MAX_VENDOR_ROOT:=$HOME/智元四足机器人D1 Max二次开发文档资料包v0.1.0}"
: "${D1MAX_APP_ROOT:=$D1MAX_VENDOR_ROOT/d1max_ros2}"
: "${D1MAX_ROS2_ENV:=$D1MAX_APP_ROOT/d1max_ros2_env.sh}"
: "${D1MAX_GTSAM_PREFIX:=$HOME/.local/ros-humble-gtsam/opt/ros/humble}"
: "${LIVOX_SDK2_PREFIX:=$HOME/go2_nav/thirdparty/livox-sdk2-install}"
export D1MAX_NAV_ROOT D1MAX_VENDOR_ROOT D1MAX_APP_ROOT D1MAX_ROS2_ENV D1MAX_GTSAM_PREFIX LIVOX_SDK2_PREFIX
