#!/usr/bin/env bash
# Copy this file AND d1_receiver_launcher.py beside robot_udp_receiver.py on D1.
set -eo pipefail
RECEIVER_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source /opt/d1_ros2/setup.bash
source /opt/d1_ros2/namespace.sh
source "$RECEIVER_DIR/bridge_ws/install/local_setup.bash"
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
exec python3 "$RECEIVER_DIR/d1_receiver_launcher.py" "$@"
