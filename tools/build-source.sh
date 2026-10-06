#!/usr/bin/env bash
# Rebuild this source snapshot in a fresh, external directory. No runtime launch.
set -euo pipefail

usage() {
  printf '%s\n' \
    'Usage: bash tools/build-source.sh --output NEW_PATH [options] [--apply]' \
    'Default is a read-only command plan. --apply performs the selected builds.' \
    '  --scope nav|sdk|pct|web|all   default: nav (includes native PCT)' \
    '  --nav-root PATH             default: <publication>/d1max_nav_ws' \
    '  --app-root PATH             default: <publication>/d1max_ros2' \
    '  --pct-root PATH             default: <nav-root>/src/pct_planner_vendor' \
    '  --ros-prefix PATH           default: /opt/ros/humble' \
    '  --rmw-prefix PATH           explicit Zenoh prefix; or D1MAX_RMW_PREFIX' \
    '  --dependency-prefix PATH    additional Humble overlay; repeatable' \
    '  --livox-prefix PATH         Livox-SDK2 install; or LIVOX_SDK2_PREFIX' \
    '  --sdk-vendor-root PATH      or D1MAX_SDK_VENDOR_ROOT; default: <app>/sdk_bridge_ws/src/d1max_sdk_bridge/vendor/robot_sdk' \
    '  --jobs N                    default: 2; sequential ROS packages' \
    '  --python PATH               default: /usr/bin/python3 (must be Python 3.10)' \
    '  --c-compiler PATH           default: /usr/bin/gcc' \
    '  --cxx-compiler PATH         default: /usr/bin/g++' \
    'Fresh output is required even after a failed build; use another directory.' \
    'No apt/rosdep install, services, ROS nodes, SDK connection or release sealing.' \
    'Web/all use npm ci --ignore-scripts from the copied lockfile, then npm run build.'
}

fail() { printf 'ERROR: %s\n' "$*" >&2; exit 2; }
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
repo_root=$(cd -- "$script_dir/.." && pwd -P)
nav_root=${D1MAX_NAV_ROOT:-$repo_root/d1max_nav_ws}
app_root=${D1MAX_APP_ROOT:-$repo_root/d1max_ros2}
pct_root=${PCT_PLANNER_ROOT:-}
ros_prefix=/opt/ros/humble
rmw_prefix=${D1MAX_RMW_PREFIX:-}
livox_prefix=${LIVOX_SDK2_PREFIX:-}
sdk_vendor_root=${D1MAX_SDK_VENDOR_ROOT:-}
python_bin=/usr/bin/python3
c_compiler=/usr/bin/gcc
cxx_compiler=/usr/bin/g++
output=''
scope=nav
jobs=2
apply=false
dependency_prefixes=()
while (($#)); do
  case "$1" in
    --apply) apply=true; shift ;;
    --help|-h) usage; exit 0 ;;
    --output|--scope|--nav-root|--app-root|--pct-root|--ros-prefix|--rmw-prefix|--dependency-prefix|--livox-prefix|--sdk-vendor-root|--jobs|--python|--c-compiler|--cxx-compiler)
      (($# >= 2)) || fail "Missing value for $1"
      case "$1" in
        --output) output=$2 ;; --scope) scope=$2 ;;
        --nav-root) nav_root=$2 ;; --app-root) app_root=$2 ;;
        --pct-root) pct_root=$2 ;; --ros-prefix) ros_prefix=$2 ;;
        --rmw-prefix) rmw_prefix=$2 ;; --livox-prefix) livox_prefix=$2 ;;
        --sdk-vendor-root) sdk_vendor_root=$2 ;; --jobs) jobs=$2 ;;
        --python) python_bin=$2 ;; --dependency-prefix) dependency_prefixes+=("$2") ;;
        --c-compiler) c_compiler=$2 ;; --cxx-compiler) cxx_compiler=$2 ;;
      esac
      shift 2 ;;
    *) fail "Unknown option: $1" ;;
  esac
done
[[ -n "$output" ]] || fail '--output is required'
[[ "$jobs" =~ ^[1-9][0-9]*$ ]] || fail '--jobs must be a positive integer'
want_nav=false; want_sdk=false; want_pct=false; want_web=false
case "$scope" in
  nav) want_nav=true; want_pct=true ;;
  sdk) want_sdk=true ;;
  pct) want_pct=true ;;
  web) want_web=true ;;
  all) want_nav=true; want_sdk=true; want_pct=true; want_web=true ;;
  *) fail '--scope must be nav, sdk, pct, web or all' ;;
esac
want_ros=false
if "$want_nav" || "$want_sdk"; then want_ros=true; fi
command -v realpath >/dev/null || fail 'realpath is required'
[[ -e "$output" || -L "$output" ]] && fail "Output already exists: $output"
output=$(realpath -m -- "$output")
[[ "$output" != / ]] || fail 'Output must be a new, external directory'
nav_root=$(realpath -m -- "$nav_root")
app_root=$(realpath -m -- "$app_root")
pct_root=$(realpath -m -- "${pct_root:-$nav_root/src/pct_planner_vendor}")
ros_prefix=$(realpath -m -- "$ros_prefix")
sdk_vendor_root=$(realpath -m -- "${sdk_vendor_root:-$app_root/sdk_bridge_ws/src/d1max_sdk_bridge/vendor/robot_sdk}")
if [[ -n "$rmw_prefix" ]]; then rmw_prefix=$(realpath -m -- "$rmw_prefix"); fi
if [[ -n "$livox_prefix" ]]; then livox_prefix=$(realpath -m -- "$livox_prefix"); fi
for i in "${!dependency_prefixes[@]}"; do
  dependency_prefixes[$i]=$(realpath -m -- "${dependency_prefixes[$i]}")
done
# Resolve symlinks before checking ancestry, including a nonexistent leaf.
protected_roots=("$repo_root" "$nav_root" "$app_root" "$pct_root" "$ros_prefix" "$sdk_vendor_root")
[[ -z "$rmw_prefix" ]] || protected_roots+=("$rmw_prefix")
[[ -z "$livox_prefix" ]] || protected_roots+=("$livox_prefix")
protected_roots+=("${dependency_prefixes[@]}")
for protected in "${protected_roots[@]}"; do
  [[ "$output" != "$protected" && "$output" != "$protected/"* && "$protected" != "$output/"* ]] || \
    fail "Output must be outside source/dependency trees: $protected"
done
[[ ! -e "$output" && ! -L "$output" ]] || fail "Output already exists: $output"

missing=()
need_command() { command -v "$1" >/dev/null || missing+=("command: $1"); }
need_file() { [[ -f "$1" ]] || missing+=("file: $1"); }
need_dir() { [[ -d "$1" ]] || missing+=("directory: $1"); }
host_arch=$(uname -m)
case "$host_arch" in
  x86_64) host_machine='Advanced Micro Devices X86-64' ;;
  aarch64) host_machine='AArch64' ;;
  *) host_machine='unsupported' ;;
esac
need_host_elf() {
  local header
  if [[ -f "$2" ]] && command -v readelf >/dev/null; then
    header=$(LC_ALL=C readelf -h -- "$2" 2>/dev/null) || header=''
    [[ "$host_machine" != unsupported && "$header" == *"$host_machine"* ]] || \
      missing+=("$1 ELF architecture differs from supported host: $host_arch")
  fi
}
need_command rsync
if "$want_ros" || "$want_pct"; then
  need_command cmake
  need_command gcc; need_command g++; need_command make
  [[ -x "$python_bin" ]] || missing+=("Python executable: $python_bin")
  [[ -x "$c_compiler" ]] || missing+=("C compiler executable: $c_compiler")
  [[ -x "$cxx_compiler" ]] || missing+=("C++ compiler executable: $cxx_compiler")
fi
if "$want_ros"; then
  need_command readelf
  need_file "$ros_prefix/setup.bash"
  need_command colcon
  colcon_bin=$(command -v colcon || printf colcon)
  if [[ -z "$rmw_prefix" ]]; then
    missing+=('explicit --rmw-prefix (or D1MAX_RMW_PREFIX)')
  else
    need_file "$rmw_prefix/lib/librmw_zenoh_cpp.so"
    need_host_elf Zenoh "$rmw_prefix/lib/librmw_zenoh_cpp.so"
    need_dir "$rmw_prefix/opt/zenoh_cpp_vendor/lib"
    need_dir "$rmw_prefix/share/rmw_zenoh_cpp"
  fi
  for prefix in "${dependency_prefixes[@]}"; do need_file "$prefix/local_setup.bash"; done
  need_file "$nav_root/src/d1max_planning_interfaces/package.xml"
  need_file "$nav_root/src/scan_planner_vendor/scan_planner_msgs/package.xml"
fi
if "$want_nav"; then
  need_file "$nav_root/src/d1max_pct_scan/package.xml"
  need_file "$nav_root/src/d1max_navigation_bt/package.xml"
  if [[ -z "$livox_prefix" ]]; then
    missing+=('explicit --livox-prefix (or LIVOX_SDK2_PREFIX)')
  else
    need_file "$livox_prefix/include/livox_lidar_api.h"
    need_file "$livox_prefix/include/livox_lidar_def.h"
    need_file "$livox_prefix/lib/liblivox_lidar_sdk_shared.so"
    need_host_elf Livox "$livox_prefix/lib/liblivox_lidar_sdk_shared.so"
  fi
fi
if "$want_pct"; then
  need_file "$pct_root/planner/lib/CMakeLists.txt"
  need_file "$pct_root/planner/lib/3rdparty/gtsam-4.1.1/CMakeLists.txt"
  need_file "$pct_root/planner/lib/3rdparty/osqp/CMakeLists.txt"
  need_file "$pct_root/planner/lib/3rdparty/pybind11/include/pybind11/pybind11.h"
  need_file "$pct_root/planner/lib/3rdparty/osqp/lin_sys/direct/qdldl/qdldl_sources/include/qdldl.h"
  need_file "$pct_root/planner/lib/3rdparty/osqp/lin_sys/direct/qdldl/qdldl_sources/src/qdldl.c"
fi
if "$want_sdk"; then
  need_command readelf
  need_file "$app_root/sdk_bridge_ws/src/d1max_sdk_bridge/CMakeLists.txt"
  sdk_arch=$host_arch
  sdk_library="$sdk_vendor_root/lib/$sdk_arch/librobot_sdk.so"
  need_file "$sdk_vendor_root/include/robot_sdk/sdk_client.hpp"
  need_file "$sdk_library"
  if [[ -f "$sdk_library" ]]; then
    sdk_resolved=$(realpath -e -- "$sdk_library")
    [[ "$sdk_resolved" == "$sdk_vendor_root/"* ]] || missing+=('Robot SDK library escapes the supplied SDK tree')
    need_host_elf SDK "$sdk_library"
  fi
fi
if "$want_web"; then
  need_command node; need_command npm
  need_file "$app_root/map_manager/frontend/package.json"
  need_file "$app_root/map_manager/frontend/package-lock.json"
  node_bin=$(command -v node || true)
  npm_bin=$(command -v npm || true)
fi

printf 'Mode: %s; scope: %s; jobs: %s\nOutput: %s\n' "$([[ "$apply" == true ]] && printf build || printf dry-run)" "$scope" "$jobs" "$output"
printf '%s\n' 'Outputs are source-build artifacts, not a sealed or activated navigation release.'
if ((${#missing[@]})); then
  printf 'Missing prerequisites:\n' >&2
  printf '  %s\n' "${missing[@]}" >&2
  if "$apply"; then fail 'Prerequisites missing; no output directory was created'; fi
fi

run() {
  printf '+'; printf ' %q' "$@"; printf '\n'
  if "$apply"; then "$@"; fi
}
copy_source() {
  run rsync -rlt --safe-links --exclude=.git --exclude=.gitmodules \
    --exclude=build/ --exclude=install/ --exclude=log/ --exclude=Log/ \
    --exclude=node_modules/ --exclude=dist/ --exclude=__pycache__/ \
    --exclude=.pytest_cache/ --exclude=.venv/ --exclude=venv/ \
    --exclude='*.so' --exclude='*.so.*' --exclude='*.a' --exclude='*.o' \
    --exclude='*.pyc' --exclude='*.pcd' --exclude='*.bag' --exclude='*.db3' \
    "$@"
}
clean_environment() {
  # Ignore inherited ROS/Conda and old source-overlay paths. No production
  # router config is sourced; these commands never create ROS nodes.
  local name
  for name in "${!ROS_@}" "${!ZENOH_@}" "${!AMENT_@}" "${!COLCON_@}"; do
    [[ -z "$name" ]] || unset "$name"
  done
  unset PYTHONPATH PYTHONHOME LD_PRELOAD LD_LIBRARY_PATH CMAKE_PREFIX_PATH \
    VIRTUAL_ENV CONDA_PREFIX CONDA_DEFAULT_ENV RMW_IMPLEMENTATION
  export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
  # Preserve an explicitly provisioned Node runtime (e.g. nvm) for all/web.
  if "$want_web" && [[ -n "$node_bin" && -n "$npm_bin" ]]; then
    export PATH="$(dirname -- "$node_bin"):$(dirname -- "$npm_bin"):$PATH"
  fi
  export CC="$c_compiler" CXX="$cxx_compiler"
  export CMAKE_BUILD_PARALLEL_LEVEL="$jobs" MAKEFLAGS="-j$jobs"
}
prepare_ros_environment() {
  printf '+ source %q (clean build environment)\n' "$ros_prefix/setup.bash"
  printf '+ add explicit rmw_zenoh_cpp prefix %q (libraries and package paths only)\n' "$rmw_prefix"
  if "$apply"; then
    clean_environment
    set +u
    source "$ros_prefix/setup.bash"
    set -u
    [[ "${ROS_DISTRO:-}" == humble ]] || fail 'ROS prefix must select Humble'
    for prefix in "${dependency_prefixes[@]}"; do
      set +u
      source "$prefix/local_setup.bash"
      set -u
      [[ "${ROS_DISTRO:-}" == humble ]] || fail 'Dependency overlay changed ROS distribution'
    done
    for name in "${!ROS_@}" "${!ZENOH_@}"; do
      case "$name" in ROS_DISTRO|ROS_VERSION|ROS_PYTHON_VERSION|'') ;; *) unset "$name" ;; esac
    done
    export AMENT_PREFIX_PATH="$rmw_prefix${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
    export CMAKE_PREFIX_PATH="$rmw_prefix${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
    export LD_LIBRARY_PATH="$rmw_prefix/lib:$rmw_prefix/opt/zenoh_cpp_vendor/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    export PYTHONPATH="$rmw_prefix/lib/python3.10/site-packages${PYTHONPATH:+:$PYTHONPATH}"
    export PATH="$rmw_prefix/bin:$rmw_prefix/lib/rmw_zenoh_cpp:$PATH"
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp
    export LIVOX_SDK2_PREFIX="$livox_prefix"
  fi
  for prefix in "${dependency_prefixes[@]}"; do printf '+ source Humble overlay %q\n' "$prefix/local_setup.bash"; done
}

# ABI check happens before the first write and never imports ROS/native modules.
if "$apply" && { "$want_ros" || "$want_pct"; }; then
  [[ "$("$python_bin" -I -c 'import sys; print("%d.%d" % sys.version_info[:2])')" == 3.10 ]] || \
    fail 'Humble/PCT builds require Python 3.10; no output directory was created'
fi
if "$apply" && "$want_web"; then
  node_version=$("$node_bin" -p 'process.versions.node')
  IFS=. read -r node_major node_minor _ <<< "$node_version"
  [[ "$node_major" =~ ^[0-9]+$ && "$node_minor" =~ ^[0-9]+$ ]] || fail 'Cannot determine Node version'
  if ! ((node_major == 20 && node_minor >= 19 || node_major == 22 && node_minor >= 12 || node_major > 22)); then
    fail 'The locked Vite build needs Node 20.19+ or 22.12+; no output directory was created'
  fi
fi
if [[ ! -d "$(dirname -- "$output")" ]]; then run mkdir -p -- "$(dirname -- "$output")"; fi
run mkdir -- "$output"
if "$want_pct"; then
  pct_out="$output/pct_vendor"
  run mkdir -- "$pct_out"
  copy_source "$pct_root/" "$pct_out/"
  if "$apply"; then clean_environment; fi
  gtsam="$pct_out/planner/lib/3rdparty/gtsam-4.1.1"
  osqp="$pct_out/planner/lib/3rdparty/osqp"
  native="$pct_out/planner/lib"
  run cmake -S "$gtsam" -B "$gtsam/build" -DCMAKE_INSTALL_PREFIX="$gtsam/install" \
    -DCMAKE_BUILD_TYPE=Release -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_TESTS=OFF \
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_BUILD_UNSTABLE=OFF -DGTSAM_BUILD_PYTHON=OFF
  run cmake --build "$gtsam/build" --parallel "$jobs"
  run cmake --install "$gtsam/build"
  run cmake -S "$osqp" -B "$osqp/build" -DCMAKE_INSTALL_PREFIX="$osqp/install" \
    -DCMAKE_BUILD_TYPE=Release -DUNITTESTS=OFF
  run cmake --build "$osqp/build" --parallel "$jobs"
  run cmake --install "$osqp/build"
  run cmake -S "$native" -B "$native/build" -DCMAKE_BUILD_TYPE=Release \
    -DPYTHON_EXECUTABLE="$python_bin" -DPython_EXECUTABLE="$python_bin" -DPython3_EXECUTABLE="$python_bin"
  run cmake --build "$native/build" --parallel "$jobs"
  run find "$native/build/src" -type f -name '*.so' -exec cp -t "$native" -- '{}' +
  printf 'PCT runtime root: %s (CMake build/link evidence is retained)\n' "$pct_out"
fi
if "$want_ros"; then
  nav_source="$output/nav-source"
  run mkdir -- "$nav_source"
  if "$want_nav"; then
    copy_source --exclude=/pct_planner_vendor/ "$nav_root/src/" "$nav_source/src/"
    # setup.py installs these audited helpers from ../../tools. Preserve that
    # source-relative layout in the isolated copy as well.
    copy_source "$nav_root/tools/pointcloud_preprocessing/" "$nav_source/tools/pointcloud_preprocessing/"
    nav_targets=(d1max_pct_scan)
  else
    # The SDK needs typed interfaces, not a second navigator or a SLAM build.
    run mkdir -- "$nav_source/src"
    copy_source "$nav_root/src/scan_planner_vendor/scan_planner_msgs/" "$nav_source/src/scan_planner_msgs/"
    copy_source "$nav_root/src/d1max_planning_interfaces/" "$nav_source/src/d1max_planning_interfaces/"
    nav_targets=(d1max_planning_interfaces)
  fi
  prepare_ros_environment
  run "$colcon_bin" --log-base "$output/nav/log" build --base-paths "$nav_source/src" \
    --build-base "$output/nav/build" --install-base "$output/nav/install" \
    --executor sequential --packages-up-to "${nav_targets[@]}" \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
      -DPython3_EXECUTABLE="$python_bin" -DLIVOX_SDK2_PREFIX="$livox_prefix"
fi
if "$want_sdk"; then
  sdk_source="$output/sdk-source/src/d1max_sdk_bridge"
  run mkdir -p -- "$sdk_source/vendor/robot_sdk"
  copy_source --exclude=/vendor/ "$app_root/sdk_bridge_ws/src/d1max_sdk_bridge/" "$sdk_source/"
  # SDK binaries are the explicitly supplied manufacturer dependency. Preserve
  # relative library symlinks; the preflight checked the selected host ELF.
  run rsync -rlt --safe-links "$sdk_vendor_root/" "$sdk_source/vendor/robot_sdk/"
  printf '+ source %q\n' "$output/nav/install/local_setup.bash"
  if "$apply"; then
    set +u
    source "$output/nav/install/local_setup.bash"
    set -u
  fi
  run "$colcon_bin" --log-base "$output/sdk/log" build --base-paths "$output/sdk-source/src" \
    --build-base "$output/sdk/build" --install-base "$output/sdk/install" \
    --executor sequential --packages-up-to d1max_sdk_bridge \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DPython3_EXECUTABLE="$python_bin"
fi
if "$want_web"; then
  web_out="$output/web"
  run mkdir -- "$web_out"
  copy_source "$app_root/map_manager/frontend/" "$web_out/"
  run npm --prefix "$web_out" ci --ignore-scripts --no-audit --no-fund
  run npm --prefix "$web_out" run build
fi
if "$apply"; then
  printf '%s\n' 'Source builds completed. Output was not sealed, deployed, selected or activated.'
else
  printf '%s\n' 'Dry run complete; no files were created and no setup hooks/builds were executed.'
  if ((${#missing[@]})); then exit 1; fi
fi
