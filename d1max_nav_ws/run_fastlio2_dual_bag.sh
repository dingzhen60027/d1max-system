#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "$ROOT/run_fastlio2_single_bag.sh" "${1:-latest}" dual "${2:-true}"
