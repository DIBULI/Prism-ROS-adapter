#!/usr/bin/env bash
set -e
source /opt/ros/noetic/setup.bash
source /opt/prism-ros1/setup.bash
export LD_LIBRARY_PATH="/opt/prism-sdk/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
exec "$@"
