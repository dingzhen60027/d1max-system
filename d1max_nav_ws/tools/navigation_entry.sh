#!/usr/bin/env bash
# Public entry for the unique indoor/outdoor, multi-floor navigation mainline.
# Keep the existing pinned loader: release identity, startup verification,
# transport and capability scope are not changed by this compatibility name.
set -eo pipefail
NAVIGATION_ENTRY_DIR=$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")
exec bash "$NAVIGATION_ENTRY_DIR/single_floor_entry.sh" "$@"
