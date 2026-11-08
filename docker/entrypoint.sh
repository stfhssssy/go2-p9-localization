#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
if [ -f /opt/unitree_ros_ws/install/setup.bash ]; then
  source /opt/unitree_ros_ws/install/setup.bash
fi
source /opt/p9_ws/install/setup.bash
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
if [ -z "${CYCLONEDDS_URI:-}" ]; then
  p9_iface="${SAR_ROBOT_IFACE:-lo}"
  if [[ ! "$p9_iface" =~ ^[a-zA-Z0-9_.:-]+$ ]]; then
    echo 'Invalid SAR_ROBOT_IFACE' >&2; exit 2
  fi
  export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${p9_iface}\"/></Interfaces></General></Domain></CycloneDDS>"
fi
case "${1:-diagnose}" in
  live|replay|diagnose|save|version) set -- p9 "$@" ;;
esac
if [ "$#" -eq 0 ]; then set -- p9 diagnose; fi
exec "$@"
