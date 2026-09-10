#!/usr/bin/env bash
set -euo pipefail

if [[ "${EUID}" -eq 0 ]]; then
  echo "请以普通用户运行本脚本，不要在命令前加 sudo。"
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENV_FILE="${SCRIPT_DIR}/d1max_ros2_env.sh"
LOCAL_ROOT="${SCRIPT_DIR}/local"
LOCAL_PREFIX="${LOCAL_ROOT}/opt/ros/humble"
NETWORK_DEVICE="enx6c1ff7bc241e"
ROBOT_IP="192.168.168.100"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "错误：未找到 ROS 2 Humble：/opt/ros/humble/setup.bash"
  exit 1
fi

echo "[1/3] 准备 ROS 2 Zenoh RMW..."
if [[ ! -x "${LOCAL_PREFIX}/lib/rmw_zenoh_cpp/rmw_zenohd" ]]; then
  DOWNLOAD_DIR="$(mktemp -d /tmp/d1max-ros2-install.XXXXXX)"
  trap 'rm -rf "${DOWNLOAD_DIR}"' EXIT
  (
    cd "${DOWNLOAD_DIR}"
    apt download ros-humble-rmw-zenoh-cpp ros-humble-zenoh-cpp-vendor
    for DEB_FILE in ./*.deb; do
      dpkg-deb -x "${DEB_FILE}" "${LOCAL_ROOT}"
    done
  )
fi

echo "[2/3] 检查机器狗有线网络..."
if ! ip -4 -brief address show dev "${NETWORK_DEVICE}" 2>/dev/null | grep -q '192\.168\.168\.10/24'; then
  echo "错误：${NETWORK_DEVICE} 未配置为 192.168.168.10/24。"
  echo "请检查 D1max 有线连接，或执行："
  echo "  nmcli connection modify D1max ipv4.method manual ipv4.addresses 192.168.168.10/24 ipv4.gateway '' ipv4.dns ''"
  echo "  nmcli connection up D1max"
  exit 1
fi

echo "[3/3] 写入 ROS 2 用户环境..."
SOURCE_LINE="source \"${ENV_FILE}\""
if ! grep -Fqx "${SOURCE_LINE}" "${HOME}/.bashrc"; then
  printf '\n# D1 Max ROS 2 communication\n%s\n' "${SOURCE_LINE}" >> "${HOME}/.bashrc"
fi

echo
echo "配置完成。请运行："
echo "  ${SCRIPT_DIR}/start.sh"
echo
if ping -c 1 -W 1 "${ROBOT_IP}" >/dev/null 2>&1; then
  echo "机器狗 ${ROBOT_IP} 网络可达。"
else
  echo "提示：机器狗 ${ROBOT_IP} 当前未响应 ping，请确认机器狗已开机并接好网线。"
fi
