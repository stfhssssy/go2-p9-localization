#!/usr/bin/env bash
set -eo pipefail
p9_fix_root=/opt/p9-tare-frontier-fix
mkdir -p /opt/p9-tare-provenance/frontier-fix
cp /opt/p9-tare-provenance/tare-build.log /opt/p9-tare-provenance/frontier-fix/before-build.log
cd /tare_ws
patch --batch --forward -p1 < "$p9_fix_root/TARE_FRONTIER_PARAMETER.patch"
sha256sum "$p9_fix_root/TARE_FRONTIER_PARAMETER.patch" > /opt/p9-tare-provenance/frontier-fix/patch.sha256
# The existing source/build tree is retained so only modified translation units rebuild.
BUILD_JOBS="${FRONTIER_BUILD_JOBS:-4}" bash /opt/p9-tare-build/compile_tare.sh
cp /opt/p9-tare-provenance/tare-build.log /opt/p9-tare-provenance/frontier-fix/after-build.log
