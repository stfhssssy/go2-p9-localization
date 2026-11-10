#!/usr/bin/env bash
# Run only inside the isolated Gazebo navigation container.
set -eo pipefail
export ROS_DOMAIN_ID=94
export GZ_PARTITION=ssy-p9-nav-20261110
export P9_HOME=/opt/p9
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash

# The Docker entrypoint may already have sourced the new overlay. Ament's
# prepend_unique hooks leave an existing entry where it is: sourcing /ws then
# sourcing the overlay again can therefore keep OLD /ws packages in front.
# Remove only our overlay entries first, preserving all unrelated underlays.
# local_setup then adds this overlay once at the front without replaying its
# build-time parent chain. prepare_runtime.py independently checks resolution.
p9_nav_overlay=/opt/p9_ws/install
for p9_nav_var in AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH \
    PYTHONPATH PATH LD_LIBRARY_PATH PKG_CONFIG_PATH ROS_PACKAGE_PATH \
    GZ_SIM_RESOURCE_PATH GZ_SIM_SYSTEM_PLUGIN_PATH GAZEBO_MODEL_PATH \
    GAZEBO_PLUGIN_PATH IGN_GAZEBO_RESOURCE_PATH IGN_GAZEBO_SYSTEM_PLUGIN_PATH; do
  p9_nav_remaining="${!p9_nav_var}"
  if [[ -z "$p9_nav_remaining" ]]; then
    continue
  fi
  p9_nav_rebuilt=''
  p9_nav_separator=''
  while :; do
    p9_nav_item="${p9_nav_remaining%%:*}"
    if [[ "$p9_nav_item" != "$p9_nav_overlay" && "$p9_nav_item" != "$p9_nav_overlay/"* ]]; then
      p9_nav_rebuilt+="$p9_nav_separator$p9_nav_item"
      p9_nav_separator=:
    fi
    if [[ "$p9_nav_remaining" != *:* ]]; then
      break
    fi
    p9_nav_remaining="${p9_nav_remaining#*:}"
  done
  printf -v "$p9_nav_var" '%s' "$p9_nav_rebuilt"
  export "$p9_nav_var"
done
source "$p9_nav_overlay/local_setup.bash"
unset p9_nav_overlay p9_nav_var p9_nav_remaining p9_nav_rebuilt p9_nav_separator p9_nav_item
exec "$@"
