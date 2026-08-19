#!/usr/bin/env bash
set -e
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source /opt/prism-ros2/setup.bash
export AMENT_PREFIX_PATH="/opt/prism-ros2${AMENT_PREFIX_PATH:+:${AMENT_PREFIX_PATH}}"
export LD_LIBRARY_PATH="/opt/prism-sdk/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
exec "$@"
