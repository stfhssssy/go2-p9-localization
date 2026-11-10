#!/usr/bin/env bash
# Run only inside the isolated Gazebo navigation container.
set -eo pipefail
export ROS_DOMAIN_ID=94
export GZ_PARTITION=ssy-p9-nav-20261110
export P9_HOME=/opt/p9
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
source /opt/p9_ws/install/setup.bash
exec "$@"
