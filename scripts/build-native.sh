#!/usr/bin/env bash
# Build from source on Linux/ROS 2 Jazzy. No dependency installation or sudo.
set -eo pipefail

p9_repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
p9_build_jobs="${BUILD_JOBS:-2}"
if [[ ! "$p9_build_jobs" =~ ^([1-9]|1[0-6])$ ]]; then
  echo 'BUILD_JOBS must be an integer from 1 to 16; default is 2.' >&2
  exit 2
fi
if [[ "$#" -ne 0 ]]; then
  echo 'Usage: BUILD_JOBS=2 scripts/build-native.sh' >&2
  exit 2
fi
if [[ ! -r /opt/ros/jazzy/setup.bash ]]; then
  echo 'ROS 2 Jazzy is required at /opt/ros/jazzy; see docs/native.md.' >&2
  exit 2
fi

source /opt/ros/jazzy/setup.bash
if ! command -v colcon >/dev/null 2>&1; then
  echo 'colcon is unavailable. Ask for dependencies to be preinstalled; see docs/native.md.' >&2
  exit 2
fi

# The historical Point-LIO source opens ROOT_DIR/Log even when detailed logging
# is disabled. These are per-checkout directories, never a shared old workspace.
mkdir -p "$p9_repo_root/src/point_lio_unilidar/Log" "$p9_repo_root/src/point_lio_unilidar/PCD"
cd -- "$p9_repo_root"
export P9_HOME="$p9_repo_root"
MAKEFLAGS="-j${p9_build_jobs}" CMAKE_BUILD_PARALLEL_LEVEL="$p9_build_jobs" \
  colcon build --executor sequential --event-handlers console_direct+ \
    --packages-select point_lio_unilidar go2_p9 \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

sha256sum "$p9_repo_root/install/point_lio_unilidar/lib/point_lio_unilidar/pointlio_mapping" > "$p9_repo_root/provenance/binary.sha256"
echo "Native build complete. Run: $p9_repo_root/scripts/p9 diagnose"
