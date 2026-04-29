#!/usr/bin/env bash
set -e

source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"

if [ -f "${ROS_WS:-/workspace}/install/setup.bash" ]; then
  source "${ROS_WS:-/workspace}/install/setup.bash"
fi

mkdir -p "${ROS_LOG_DIR:-/tmp/ros-log}"

exec "$@"
