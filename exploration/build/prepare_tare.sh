#!/usr/bin/env bash
set -euo pipefail
cd /opt/p9-tare-build-inputs
sha256sum -c SHA256SUMS
p9_arch=$(dpkg --print-architecture)
case "$p9_arch" in
  arm64) p9_archive=or-tools_arm64_debian-11_cpp_v9.8.3296.tar.gz; p9_machine=AArch64 ;;
  amd64) p9_archive=or-tools_amd64_ubuntu-22.04_cpp_v9.8.3296.tar.gz; p9_machine='Advanced Micro Devices X86-64' ;;
  *) echo "Unsupported target architecture: $p9_arch" >&2; exit 2 ;;
esac
mkdir -p /tare_ws /opt/p9-tare-provenance
tar -xzf tare-b6760adb4280b379567c76f4e684ce628e28ef71.tar.gz --strip-components=1 -C /tare_ws
cd /tare_ws
git apply --check /opt/p9-tare-build-inputs/P9_TARE.patch
git apply /opt/p9-tare-build-inputs/P9_TARE.patch
# Original repository bundles an x86-64 OR-Tools binary. Replace the complete SDK
# with Google's matching-architecture v9.8.3296 SDK, preserving TARE solver code.
mv src/tare_planner/or-tools /opt/p9-tare-build-inputs/upstream-ortools-amd64
mkdir src/tare_planner/or-tools
tar -xzf "/opt/p9-tare-build-inputs/$p9_archive" --strip-components=1 -C src/tare_planner/or-tools
readelf -h src/tare_planner/or-tools/lib/libortools.so.9.8.3296 > /opt/p9-tare-provenance/ortools-elf.txt
grep -F "$p9_machine" /opt/p9-tare-provenance/ortools-elf.txt
export LD_LIBRARY_PATH="/tare_ws/src/tare_planner/or-tools/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
ldd src/tare_planner/or-tools/lib/libortools.so.9.8.3296 > /opt/p9-tare-provenance/ortools-linkage.txt
! grep -F 'not found' /opt/p9-tare-provenance/ortools-linkage.txt
printf '%s\n' "$p9_arch" > /opt/p9-tare-provenance/architecture.txt
printf '%s\n' "$p9_archive" > /opt/p9-tare-provenance/ortools-archive.txt
printf '%s\n' 'b6760adb4280b379567c76f4e684ce628e28ef71' > /opt/p9-tare-provenance/tare-commit.txt
cp /opt/p9-tare-build-inputs/SHA256SUMS /opt/p9-tare-provenance/input-SHA256SUMS
cp /opt/p9-tare-build-inputs/P9_TARE.patch /opt/p9-tare-provenance/
# Original TSP source and build rules are intentionally unchanged by our platform port.
sha256sum src/tare_planner/CMakeLists.txt src/tare_planner/src/tsp_solver/tsp_solver.cpp \
  > /opt/p9-tare-provenance/tsp-source.sha256
