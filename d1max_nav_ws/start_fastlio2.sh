#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
USE_RVIZ="${1:-true}"

# Never stop or replace a running Faster-LIO session from this isolated entry
# point. The operator must stop it explicitly before comparing backends.
if pgrep -x run_mapping_online >/dev/null 2>&1; then
  echo "Faster-LIO is running; refusing to start FAST-LIO2." >&2
  echo "Stop Faster-LIO explicitly, then run this command again." >&2
  exit 3
fi

exec "$ROOT/start_slam.sh" fastlio2 "$USE_RVIZ"
