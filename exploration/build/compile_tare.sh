#!/usr/bin/env bash
set -eo pipefail
p9_jobs=${BUILD_JOBS:-2}
[[ "$p9_jobs" =~ ^[1-9][0-9]*$ ]] || exit 2
source /opt/ros/jazzy/setup.bash
source /opt/p9_ws/install/setup.bash
export LD_LIBRARY_PATH="/tare_ws/src/tare_planner/or-tools/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
cd /tare_ws
MAKEFLAGS="-j${p9_jobs}" CMAKE_BUILD_PARALLEL_LEVEL="$p9_jobs" \
  colcon build --executor sequential --event-handlers console_direct+ \
  --packages-select tare_planner --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  2>&1 | tee /opt/p9-tare-provenance/tare-build.log
source install/local_setup.bash
ldd install/tare_planner/lib/tare_planner/tare_planner_node > /opt/p9-tare-provenance/tare-linkage.txt
! grep -F 'not found' /opt/p9-tare-provenance/tare-linkage.txt
readelf -h install/tare_planner/lib/tare_planner/tare_planner_node > /opt/p9-tare-provenance/tare-elf.txt
sha256sum install/tare_planner/lib/tare_planner/tare_planner_node > /opt/p9-tare-provenance/tare-binary.sha256
# Exercise the exact upstream TSP wrapper against a known small square tour; this
# catches architecture/runtime/ABI failures that a successful linker cannot detect.
c++ -std=c++17 -O2 -I/tare_ws/src/tare_planner/include \
  -I/tare_ws/src/tare_planner/or-tools/include \
  /opt/p9-tare-build/tare_tsp_smoke.cpp \
  /tare_ws/src/tare_planner/src/tsp_solver/tsp_solver.cpp \
  -L/tare_ws/src/tare_planner/or-tools/lib -Wl,-rpath,/tare_ws/src/tare_planner/or-tools/lib \
  -lortools -o /opt/p9-tare-provenance/tare-tsp-smoke
/opt/p9-tare-provenance/tare-tsp-smoke | tee /opt/p9-tare-provenance/tare-tsp-smoke.json
ros2 pkg executables tare_planner > /opt/p9-tare-provenance/ros-executables.txt
dpkg-query -W > /opt/p9-tare-provenance/packages.txt
