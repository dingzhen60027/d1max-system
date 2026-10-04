#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")/../d1max_env.sh"
source "$D1MAX_ROS2_ENV"
source "$D1MAX_NAV_ROOT/install/setup.bash"
exec python3 "$D1MAX_NAV_ROOT/src/d1max_pct_scan/d1max_pct_scan/session.py" "$@"
