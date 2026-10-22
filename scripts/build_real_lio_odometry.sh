#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
# Dependencies must already be installed/built. No old fast_livo overlay needed.
if [ -r install/local_setup.bash ]; then source install/local_setup.bash; fi
export CMAKE_BUILD_PARALLEL_LEVEL=1
export MAKEFLAGS=-j1
colcon build --symlink-install --packages-select d1h_fast_lio --executor sequential
