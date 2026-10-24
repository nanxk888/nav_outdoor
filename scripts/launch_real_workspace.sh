#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
WORKSPACE_DIR="$PWD"
source /opt/ros/humble/setup.bash
if [ ! -r install/setup.bash ]; then
  echo 'Build the workspace first: bash scripts/build_real_outdoor.sh' >&2
  exit 1
fi
source install/setup.bash
# Explicit per-process environment; never change a user's shell or other running nodes.
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset CYCLONEDDS_URI FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE ROS_DISCOVERY_SERVER
unset NXK_CONTROL_CONFIG NXK_WEB_TOKEN
exec ros2 launch d1h_nav2 real_workspace.launch.py workspace:="$WORKSPACE_DIR" "$@"
