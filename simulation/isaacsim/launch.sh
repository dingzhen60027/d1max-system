#!/usr/bin/env bash
set -eo pipefail
D1MAX_SIM_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
source "$D1MAX_SIM_DIR/env.sh"
exec /usr/bin/python3 "$D1MAX_SIM_DIR/run.py" "$@"
