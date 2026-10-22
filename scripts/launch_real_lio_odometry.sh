#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u
exec ros2 launch d1h_fast_lio navigation_lio.launch.py "$@"
