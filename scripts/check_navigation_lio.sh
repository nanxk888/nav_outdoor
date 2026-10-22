#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=93
if [ "${1:-}" != "--stream-only" ]; then
  ctest --test-dir build/d1h_fast_lio --output-on-failure
fi
python3 scripts/test_navigation_lio_stream.py
