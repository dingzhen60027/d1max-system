#!/usr/bin/env bash
set -euo pipefail
D1MAX_MONITOR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec bash "${D1MAX_MONITOR_DIR}/scripts/start_live_monitor.sh" "$@"
