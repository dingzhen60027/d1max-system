#!/usr/bin/env bash
# Source in a fresh terminal; this selects only the local simulation build.
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo 'Use: source simulation/isaacsim/env.sh' >&2
  exit 2
fi
D1MAX_SIM_REPO=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)
export D1MAX_SIM_BUILD="${D1MAX_SIM_BUILD:-$(dirname -- "$D1MAX_SIM_REPO")/d1max-build-isaac}"
export D1MAX_SIM_DEPS="${D1MAX_SIM_DEPS:-$(dirname -- "$D1MAX_SIM_REPO")/d1max-deps}"
export ISAAC_SIM_ROOT="${ISAAC_SIM_ROOT:-/home/eric/isaacsim}"
# Select the Humble fixture independently of an active Conda/other ROS shell.
# Its own documented dependency overlay adds the required scientific libraries.
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
unset CONDA_PREFIX CONDA_DEFAULT_ENV VIRTUAL_ENV PYTHONHOME PYTHONEXE
unset PYTHONPATH LD_LIBRARY_PATH LD_PRELOAD AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
unset CPATH CPLUS_INCLUDE_PATH LIBRARY_PATH PKG_CONFIG_PATH
source "$D1MAX_SIM_REPO/tools/deployment-env.sh"
if [[ ! -f "$D1MAX_SIM_DEPS/setup.bash" ]]; then
  echo "Missing dependency environment: $D1MAX_SIM_DEPS/setup.bash" >&2
  return 2
fi
source "$D1MAX_SIM_DEPS/setup.bash"
for D1MAX_SIM_OVERLAY in "$D1MAX_SIM_BUILD/nav/install" "$D1MAX_SIM_BUILD/localization/install" "$D1MAX_SIM_BUILD/sdk/install"; do
  [[ -f "$D1MAX_SIM_OVERLAY/local_setup.bash" ]] && source "$D1MAX_SIM_OVERLAY/local_setup.bash"
done
export PCT_PLANNER_ROOT="$D1MAX_SIM_BUILD/pct_vendor"
export PYTHONPATH="$D1MAX_SIM_BUILD/ros-python/lib/python3.10/site-packages${PYTHONPATH:+:$PYTHONPATH}"
# Wheel-owned Fortran/BLAS dependencies use hashed SONAMEs. The original
# runtime verifier also checks each transitive ELF independently of its parent
# module's RPATH, so expose the actual selected wheel sidecar directories.
D1MAX_SIM_SCI_LIBS=$(/usr/bin/python3 - <<'PY'
import importlib.util
from pathlib import Path
paths = []
for name in ('numpy', 'scipy', 'open3d'):
    spec = importlib.util.find_spec(name)
    if spec and spec.origin:
        root = Path(spec.origin).resolve().parent
        sidecar = root.parent / (name + '.libs')
        if sidecar.is_dir():
            paths.append(str(sidecar))
        if name == 'open3d' and any(root.glob('*.so*')):
            paths.append(str(root))
print(':'.join(dict.fromkeys(paths)))
PY
)
if [[ -n "$D1MAX_SIM_SCI_LIBS" ]]; then
  export LD_LIBRARY_PATH="$D1MAX_SIM_SCI_LIBS${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=2 MKL_NUM_THREADS=1
unset D1MAX_SIM_OVERLAY D1MAX_SIM_REPO D1MAX_SIM_SCI_LIBS
