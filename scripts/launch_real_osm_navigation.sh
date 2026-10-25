#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u
if [ "$#" -ne 1 ]; then
  echo 'Usage: bash scripts/launch_real_osm_navigation.sh /absolute/path/to/osm_profile.json' >&2
  exit 1
fi
exec ros2 launch d1h_nav2 real_osm_navigation.launch.py profile:="$1"
