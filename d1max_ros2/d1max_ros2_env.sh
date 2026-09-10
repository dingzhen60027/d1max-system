#!/usr/bin/env bash

D1MAX_ROS2_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOCAL_PREFIX="${D1MAX_ROS2_DIR}/local/opt/ros/humble"

case "$-" in
  *u*) D1MAX_RESTORE_NOUNSET=1; set +u ;;
  *) D1MAX_RESTORE_NOUNSET=0 ;;
esac
source /opt/ros/humble/setup.bash
if [[ "${D1MAX_RESTORE_NOUNSET}" -eq 1 ]]; then
  set -u
fi
unset D1MAX_RESTORE_NOUNSET

if [[ -d "${LOCAL_PREFIX}" ]]; then
  export AMENT_PREFIX_PATH="${LOCAL_PREFIX}${AMENT_PREFIX_PATH:+:${AMENT_PREFIX_PATH}}"
  export CMAKE_PREFIX_PATH="${LOCAL_PREFIX}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
  export LD_LIBRARY_PATH="${LOCAL_PREFIX}/lib:${LOCAL_PREFIX}/opt/zenoh_cpp_vendor/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
  export PYTHONPATH="${LOCAL_PREFIX}/lib/python3.10/site-packages${PYTHONPATH:+:${PYTHONPATH}}"
  export PATH="${LOCAL_PREFIX}/bin:${LOCAL_PREFIX}/lib/rmw_zenoh_cpp:${PATH}"
fi

export ROS_DOMAIN_ID=24
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_ROUTER_CONFIG_URI="${D1MAX_ROS2_DIR}/config/d1max_router.json5"

unset ROS_LOCALHOST_ONLY
