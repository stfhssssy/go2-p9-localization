#!/usr/bin/env bash
# Entrypoint for the isolated TARE / Gazebo experiment only. Never a real robot.
set -eo pipefail
export ROS_DOMAIN_ID=94
export GZ_PARTITION=ssy-p9-nav-20261110
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export SAR_ROBOT_IFACE=lo
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces></General></Domain></CycloneDDS>'
if [[ ! -f /tare_ws/install/local_setup.bash ]]; then
  echo 'TARE overlay missing: /tare_ws/install/local_setup.bash' >&2
  exit 2
fi
if [[ ! -f /tare_ws/src/tare_planner/or-tools/lib/libortools.so.9 ]]; then
  echo 'TARE bundled OR-Tools runtime missing: libortools.so.9' >&2
  exit 2
fi
if [[ $# -eq 0 ]]; then
  echo 'Usage: inside.sh COMMAND [ARG ...]' >&2
  exit 2
fi
# docker exec does not inherit environment changes from the container entrypoint.
# Reapply it, then the existing verified P9/Gazebo overlay ordering, and finally
# source only the TARE overlay (do not replay its build-time parent setup chain).
exec /usr/local/bin/p9-entrypoint /opt/p9/simulation/inside.sh \
  bash -c 'set -eo pipefail; source /tare_ws/install/local_setup.bash; export LD_LIBRARY_PATH="/tare_ws/src/tare_planner/or-tools/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"; exec "$@"' \
  p9-exploration "$@"
