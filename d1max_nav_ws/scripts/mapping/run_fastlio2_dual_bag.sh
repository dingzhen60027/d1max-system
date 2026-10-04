#!/usr/bin/env bash
set -euo pipefail

HERE="$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")"
exec "$HERE/run_fastlio2_single_bag.sh" "${1:-latest}" dual "${2:-true}"
