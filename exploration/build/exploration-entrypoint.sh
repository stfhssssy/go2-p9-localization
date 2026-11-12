#!/usr/bin/env bash
set -eo pipefail
exec /usr/local/bin/p9-entrypoint bash -c '
  set -eo pipefail
  source /opt/p9_control_ws/install/local_setup.bash
  source /tare_ws/install/local_setup.bash
  export LD_LIBRARY_PATH="/tare_ws/src/tare_planner/or-tools/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  exec "$@"
' p9explore-entrypoint "$@"
