#!/usr/bin/env bash
set -eo pipefail

WS=/home/dndx/d1max_nav_ws
VENDOR="$WS/src/pct_planner_vendor"
OUT="$WS/maps/pct_trial_building"
CONFIG="$WS/src/d1max_pct_planner/config/d1max_pct.yaml"
PCD="$VENDOR/rsc/pcd/building2_9.pcd"

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
export PCT_PLANNER_ROOT="$VENDOR"
export LD_LIBRARY_PATH="$VENDOR/planner/lib/3rdparty/gtsam-4.1.1/install/lib:$VENDOR/planner/lib/build/src/common/smoothing:${LD_LIBRARY_PATH:-}"

mkdir -p "$OUT"
if [[ ! -f "$PCD" ]]; then
  unzip -o "$VENDOR/rsc/pcd/pcd_files.zip" building2_9.pcd -d "$VENDOR/rsc/pcd"
fi

ros2 run d1max_pct_planner pct_build_tomogram \
  --pcd "$PCD" --config "$CONFIG" \
  --output "$OUT/building2_9.pickle" --preview "$OUT/tomogram_preview.ply"
ros2 run d1max_pct_planner pct_plan_offline \
  --tomogram "$OUT/building2_9.pickle" --vendor-root "$VENDOR" \
  --start 5.0 5.0 --goal -6.0 -1.0 --output "$OUT/path.csv"

echo "Trial complete: $OUT"
echo "Visualize: ros2 launch d1max_pct_planner pct_visualize.launch.py pcd:=$PCD path:=$OUT/path.csv"
