#!/usr/bin/env bash
# Legacy name now starts the monitor-only bridge, never the control gateway.
set -euo pipefail
D1MAX_MONITOR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec bash "${D1MAX_MONITOR_DIR}/scripts/start_monitor_bridge.sh" "$@"
