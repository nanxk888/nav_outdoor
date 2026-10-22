#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
source install/local_setup.bash
# All ROS fixtures are isolated from the live sensor / control domains.
export ROS_DOMAIN_ID=94
python3 scripts/test_gnss_quality.py
python3 scripts/test_osm_roads.py
python3 scripts/test_obstacle_memory.py
python3 scripts/test_station_keep.py
python3 scripts/test_stable_planner.py
python3 scripts/test_path_handover.py
python3 scripts/test_osm_road_ros.py
python3 scripts/test_real_gnss_ros.py
bash scripts/check_navigation_lio.sh
