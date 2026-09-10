#!/usr/bin/env bash
set -eo pipefail

WS=/home/dndx/d1max_nav_ws
PCD="${1:-$WS/maps/runs/20260824_175852/sc_pgo/optimized_map.pcd}"
OUT="${2:-$WS/maps/pct_d1max}"
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
mkdir -p "$OUT"
ros2 run d1max_pct_planner pct_build_tomogram \
  --pcd "$PCD" --config "$WS/src/d1max_pct_planner/config/d1max_map_pct.yaml" \
  --output "$OUT/d1max_map.pickle" --preview "$OUT/d1max_tomogram_preview.ply"
