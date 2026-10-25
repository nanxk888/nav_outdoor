#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
source install/local_setup.bash
# All ROS fixtures are isolated from the live sensor / control domains.
export ROS_DOMAIN_ID=94
python3 scripts/test_robot_wire.py
python3 scripts/test_robot_policy.py
python3 scripts/test_d1_launcher.py
python3 scripts/test_workspace_stop.py
python3 scripts/test_robot_udp_pair.py
if ros2 pkg prefix d1h_robot_bridge >/dev/null 2>&1; then
  python3 scripts/test_fsm_bridge_ros.py
else
  echo 'SKIP: D1 FSM ROS integration test requires compiled d1h_robot_bridge.'
fi
python3 scripts/test_navigation_control.py
python3 scripts/test_navigation_web.py
if command -v node >/dev/null 2>&1; then
  node scripts/test_web_viewport.js
  node scripts/test_web_interactions.js
  node scripts/test_web_control.js
else
  echo 'SKIP: frontend JavaScript unit tests require Node.js (web serving does not).'
fi
python3 scripts/test_gnss_driver_guard.py
python3 scripts/test_gnss_quality.py
python3 scripts/test_real_ground.py
python3 scripts/test_real_cloud_ros.py
python3 scripts/test_osm_roads.py
python3 scripts/test_obstacle_memory.py
python3 scripts/test_station_keep.py
python3 scripts/test_stable_planner.py
python3 scripts/test_path_handover.py
python3 scripts/test_osm_road_ros.py
python3 scripts/test_real_gnss_ros.py
bash scripts/check_navigation_lio.sh
