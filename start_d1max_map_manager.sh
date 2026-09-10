#!/usr/bin/env bash
# D1 Max 3D-only map workspace.

set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
APP_ROOT="$PROJECT_ROOT/d1max_ros2/map_manager"
PORT="${D1MAX_MAP_MANAGER_PORT:-8766}"
LOCAL_PYTHON="$APP_ROOT/.venv/bin/python"
GO2_PYTHON="/home/dndx/go2_nav/build/env/map_manager_venv/bin/python"

if [[ -n "${D1MAX_MAP_MANAGER_PYTHON:-}" ]]; then
  PYTHON_BIN="$D1MAX_MAP_MANAGER_PYTHON"
elif [[ -x "$LOCAL_PYTHON" ]]; then
  PYTHON_BIN="$LOCAL_PYTHON"
elif [[ -x "$GO2_PYTHON" ]]; then
  PYTHON_BIN="$GO2_PYTHON"
else
  echo "ERROR: 找不到带 FastAPI/Uvicorn 的 Python 环境。" >&2
  echo "请按 d1max_ros2/map_manager/README.md 创建 .venv。" >&2
  exit 1
fi

if [[ ! -d "${D1MAX_MAPS_ROOT:-/home/dndx/d1max_nav_ws/maps}" ]]; then
  echo "ERROR: D1 Max maps 目录不存在: ${D1MAX_MAPS_ROOT:-/home/dndx/d1max_nav_ws/maps}" >&2
  exit 1
fi

export D1MAX_PROJECT_ROOT="$PROJECT_ROOT"
export D1MAX_NAV_ROOT="${D1MAX_NAV_ROOT:-/home/dndx/d1max_nav_ws}"
export D1MAX_MAPS_ROOT="${D1MAX_MAPS_ROOT:-$D1MAX_NAV_ROOT/maps}"

echo "=========================================="
echo "  D1 Max 3D Map Workspace"
echo "  http://127.0.0.1:${PORT}"
echo "  Maps: $D1MAX_MAPS_ROOT"
echo "  ROS:  rmw_zenoh_cpp / Domain 24"
echo "=========================================="

exec "$PYTHON_BIN" -m uvicorn backend.app:app \
  --app-dir "$APP_ROOT" \
  --host 127.0.0.1 \
  --port "$PORT"
