#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
# First source the verified Livox/vikit underlay in the calling terminal.
if [ -r install/local_setup.bash ]; then source install/local_setup.bash; fi
export CMAKE_BUILD_PARALLEL_LEVEL=1
export MAKEFLAGS=-j1
colcon build --symlink-install --executor sequential \
  --packages-select d1h_fast_lio d1h_global_localization d1h_nav2
