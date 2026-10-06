#!/usr/bin/env bash
# Rebuild this checkout in the selected simulation workspace, without activation.
set -eo pipefail
D1MAX_SIM_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
D1MAX_SIM_REPO=$(cd -- "$D1MAX_SIM_DIR/../.." && pwd -P)
export D1MAX_SIM_BUILD="${D1MAX_SIM_BUILD:-$(dirname -- "$D1MAX_SIM_REPO")/d1max-build-isaac}"
export D1MAX_SIM_DEPS="${D1MAX_SIM_DEPS:-$(dirname -- "$D1MAX_SIM_REPO")/d1max-deps}"
JOBS=${D1MAX_SIM_JOBS:-2}
export CC="${D1MAX_SIM_CC:-/usr/bin/gcc-11}" CXX="${D1MAX_SIM_CXX:-/usr/bin/g++-11}"
if [[ ! -f "$D1MAX_SIM_DEPS/setup.bash" ]]; then
  echo "Prepare the documented dependency overlay first: $D1MAX_SIM_DEPS/setup.bash" >&2
  exit 2
fi
# Clean inherited Conda/Python/native paths before PCT configuration or pip.
source "$D1MAX_SIM_DIR/env.sh"
D1MAX_SIM_BOOST_LIBRARY_DIR="/usr/lib/$("$CC" -print-multiarch)"
if [[ ! -d "$D1MAX_SIM_BUILD/pct_vendor" ]]; then
  bash "$D1MAX_SIM_REPO/tools/build-source.sh" --scope pct \
    --output "$D1MAX_SIM_BUILD" --jobs "$JOBS" \
    --c-compiler "$CC" --cxx-compiler "$CXX" --apply
fi
if [[ ! -x "$D1MAX_SIM_BUILD/ros-python/bin/python" ]]; then
  /usr/bin/python3 -m venv --system-site-packages "$D1MAX_SIM_BUILD/ros-python"
fi
"$D1MAX_SIM_BUILD/ros-python/bin/python" -m pip install \
  'numpy==1.26.4' 'scipy==1.11.4' 'open3d==0.19.0'
source "$D1MAX_SIM_DIR/env.sh"
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
export CMAKE_BUILD_PARALLEL_LEVEL="$JOBS" MAKEFLAGS="-j$JOBS"
mkdir -p "$D1MAX_SIM_BUILD/nav-source/src" "$D1MAX_SIM_BUILD/nav-source/tools"
rsync -rlt --safe-links --exclude=pct_planner_vendor/ --exclude=__pycache__/ \
  "$D1MAX_SIM_REPO/d1max_nav_ws/src/" "$D1MAX_SIM_BUILD/nav-source/src/"
rsync -rlt --exclude=__pycache__/ "$D1MAX_SIM_REPO/d1max_nav_ws/tools/pointcloud_preprocessing/" \
  "$D1MAX_SIM_BUILD/nav-source/tools/pointcloud_preprocessing/"
colcon --log-base "$D1MAX_SIM_BUILD/nav/log" build \
  --base-paths "$D1MAX_SIM_BUILD/nav-source/src" \
  --build-base "$D1MAX_SIM_BUILD/nav/build" --install-base "$D1MAX_SIM_BUILD/nav/install" \
  --executor sequential --cmake-clean-cache --packages-select scan_planner_msgs d1max_planning_interfaces d1max_navigation_bt_interfaces \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    -DCMAKE_C_COMPILER="$CC" -DCMAKE_CXX_COMPILER="$CXX" \
    -DPython3_EXECUTABLE=/usr/bin/python3
source "$D1MAX_SIM_BUILD/nav/install/local_setup.bash"
# Keep the localization closure in its own install, with GCC 11 for oneTBB.
colcon --log-base "$D1MAX_SIM_BUILD/localization/log" build \
  --base-paths "$D1MAX_SIM_BUILD/nav-source/src" \
  --build-base "$D1MAX_SIM_BUILD/localization/build" \
  --install-base "$D1MAX_SIM_BUILD/localization/install" \
  --executor sequential --cmake-clean-cache --packages-select livox_ros_driver2 faster_lio d1max_localization \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    -DCMAKE_C_COMPILER="$CC" -DCMAKE_CXX_COMPILER="$CXX" \
    -DPython3_EXECUTABLE=/usr/bin/python3 -DLIVOX_SDK2_PREFIX="$LIVOX_SDK2_PREFIX" \
    -DBOOST_LIBRARYDIR="$D1MAX_SIM_BOOST_LIBRARY_DIR"
source "$D1MAX_SIM_BUILD/localization/install/local_setup.bash"
colcon --log-base "$D1MAX_SIM_BUILD/nav/log" build \
  --base-paths "$D1MAX_SIM_BUILD/nav-source/src" \
  --build-base "$D1MAX_SIM_BUILD/nav/build" --install-base "$D1MAX_SIM_BUILD/nav/install" \
  --executor sequential --cmake-clean-cache --packages-up-to d1max_pct_scan \
  --packages-skip d1max_localization faster_lio livox_ros_driver2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    -DCMAKE_C_COMPILER="$CC" -DCMAKE_CXX_COMPILER="$CXX" \
    -DPython3_EXECUTABLE=/usr/bin/python3 -DLIVOX_SDK2_PREFIX="$LIVOX_SDK2_PREFIX" \
    -DBOOST_LIBRARYDIR="$D1MAX_SIM_BOOST_LIBRARY_DIR"
source "$D1MAX_SIM_BUILD/nav/install/local_setup.bash"
mkdir -p "$D1MAX_SIM_BUILD/sdk-source/src"
rsync -rlt --exclude=vendor/ --exclude=__pycache__/ \
  "$D1MAX_SIM_REPO/d1max_ros2/sdk_bridge_ws/src/d1max_sdk_bridge/" \
  "$D1MAX_SIM_BUILD/sdk-source/src/d1max_sdk_bridge/"
colcon --log-base "$D1MAX_SIM_BUILD/sdk/log" build \
  --base-paths "$D1MAX_SIM_BUILD/sdk-source/src" \
  --build-base "$D1MAX_SIM_BUILD/sdk/build" --install-base "$D1MAX_SIM_BUILD/sdk/install" \
  --executor sequential --cmake-clean-cache --packages-select d1max_sdk_bridge \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
    -DCMAKE_C_COMPILER="$CC" -DCMAKE_CXX_COMPILER="$CXX" \
    -DPython3_EXECUTABLE=/usr/bin/python3 -DD1MAX_BUILD_ISOLATED_EXECUTION_ONLY=ON
ctest --test-dir "$D1MAX_SIM_BUILD/sdk/build/d1max_sdk_bridge" --output-on-failure
source "$D1MAX_SIM_DIR/env.sh"
if [[ -d "$D1MAX_SIM_BUILD/isaac-candidate" ]]; then
  echo "Build complete. Existing candidate preserved; choose a new output for build_candidate.py."
else
  export D1MAX_QUADRUPED_ASSET_ROOT="${D1MAX_QUADRUPED_ASSET_ROOT:-$D1MAX_SIM_BUILD/quadruped-assets-spot}"
  /usr/bin/python3 "$D1MAX_SIM_DIR/download_quadruped_assets.py" --output "$D1MAX_QUADRUPED_ASSET_ROOT"
  /usr/bin/python3 "$D1MAX_SIM_DIR/build_candidate.py" \
    --output "$D1MAX_SIM_BUILD/isaac-candidate" \
    --select-local "$D1MAX_SIM_BUILD/isaac_fixture.json" \
    --nav-install "$D1MAX_SIM_BUILD/nav/install" \
    --sdk-install "$D1MAX_SIM_BUILD/sdk/install" \
    --localization-install "$D1MAX_SIM_BUILD/localization/install" \
    --pct-vendor "$D1MAX_SIM_BUILD/pct_vendor" \
    --scene-config "$D1MAX_SIM_DIR/assets/large_quadruped_scene.json"
fi
