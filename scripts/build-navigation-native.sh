#!/usr/bin/env bash
# Separate opt-in native build of the navigation release; old build-native.sh is unchanged.
set -eo pipefail
p9_nav_repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
p9_nav_jobs="${BUILD_JOBS:-2}"
if [[ "$#" -ne 0 || ! "$p9_nav_jobs" =~ ^([1-9]|1[0-6])$ ]]; then
  echo 'Usage: BUILD_JOBS=2 scripts/build-navigation-native.sh (jobs 1..16)' >&2
  exit 2
fi
source /opt/ros/jazzy/setup.bash
command -v colcon >/dev/null
mkdir -p "$p9_nav_repo/src/point_lio_unilidar/Log" "$p9_nav_repo/src/point_lio_unilidar/PCD"
cd -- "$p9_nav_repo"
export P9_HOME="$p9_nav_repo"
MAKEFLAGS="-j${p9_nav_jobs}" CMAKE_BUILD_PARALLEL_LEVEL="$p9_nav_jobs" \
  colcon build --executor sequential --event-handlers console_direct+ \
    --packages-select point_lio_unilidar go2_p9 visibility_graph_msg terrain_analysis \
      terrain_analysis_ext local_planner far_planner go2_p9_navigation \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
find "$p9_nav_repo/install" -type f -executable -path '*/lib/*' -print0 \
  | sort -z | xargs -0 -r sha256sum > "$p9_nav_repo/provenance/navigation-binaries.sha256"
echo "Navigation build complete: $p9_nav_repo/scripts/p9nav --help"
