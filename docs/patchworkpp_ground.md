# Official Patchwork++ real navigation ground backend

Default `real_cloud_filter` now calls the upstream C++ implementation through its
official Python binding. It does not use the old `real_ground.py` RANSAC function.
Upstream source: https://github.com/url-kaist/patchwork-plusplus
Pinned commit: `3e6903a1d5537a4cc2ace897b0bbb98a92d6014c` (BSD-2-Clause).
No Open3D, neural model, CUDA or Autoware installation is required.

## Orin build / launch

Stop the existing workspace, update the checkout using your normal transfer
procedure, then run (first build requires GitHub/Python package index access):

```bash
cd /workspace/nxk
sudo apt-get install -y python3-venv python3-dev build-essential cmake libeigen3-dev git
source /opt/ros/humble/setup.bash
source /workspace/ws_livox/install/setup.bash
source /workspace/fast-livo2-deep-robotics/install/local_setup.bash
bash scripts/build_real_outdoor.sh
source install/local_setup.bash
bash scripts/launch_real_workspace.sh \
  sensor_height_m:=0.46 \
  gps_cyclonedds_xml:=/workspace/D1h/d1h-autonomy/.local/robot/perception/cyclonedds.xml \
  with_gps_driver:=false with_nav:=true \
  control_config:=/root/.config/nxk/control.json
```

Build invokes `install_patchworkpp.sh`. Build tools live in `.local/patchworkpp-build`,
the binding in `.local/patchworkpp`; system ROS Python packages are not upgraded.
The unified launcher adds only the private binding directory to PYTHONPATH.
Direct `ros2 run` requires explicitly exporting that PYTHONPATH as well.
Missing backend reports NOT READY; it never silently falls back to RANSAC.

## Coordinate/parameter contract

- Input is the existing EXACT acquisition-stamp matched, gravity-aligned native
  `lio_odom` deskewed cloud. Translate by the IMU position; do NOT rotate the
  tilted IMU attitude into the cloud a second time.
- `sensor_height_m` is the measured INITIAL IMU height. 0.46m is not correct if
  startup is already in another stance. Initialize a floor-z anchor, track it
  with nearby accepted ground, and normalize virtual z for the library's fixed
  height prior. Subsequent IMU z changes do not imply the floor moved.
- XYZ contains no valid intensity: RNR is OFF. RVPF and TGR are ON.
- `patchwork_th_dist_m=0.02`, `patchwork_th_seeds_m=0.04`; vertical fitting
  distances 0.05/0.10m. Do not use upstream vehicle defaults of 0.125m for
  small robot obstacles without testing.
- `patchwork_min_range_m=0.3`, `patchwork_max_range_m=6.0`; preserve the existing
  0.6m sensor-distance self exclusion, nav range 5m and obstacle heights 3..90cm.
- Zones use sectors [16,24,32,32], rings [2,2,2,2], minimum 6 points per patch,
  10 low representatives, 3 iterations, uprightness cos(20 degrees).
- At least `patchwork_min_ground_points=20` accepted ground points within 2.5m
  and +/-0.15m of the prior floor are required to confirm the height anchor.
  There is no whole-scene 45% planar inlier ratio. This is still a locally
  planar/limited-slope trial configuration, NOT a terrain or pit detector.
- Unknown points can conservatively mark obstacles; only classified returns
  can generate clearing rays. Rays still obey existing memory occlusion rules.
- LIO discontinuity resets the engine and requires measured-height initialization
  again. Topic frames/stamps and nav/clear names remain unchanged for LIO/GPS.

`/navigation/cloud_filter/status` reports backend `official_patchworkpp`, actual
parameters, classification counts, height and processing_ms. Input/height/ground
validation failures still invalidate output; no fabricated free space is emitted.

## Local tests (synthetic, no physical robot)

```bash
python3 scripts/test_patchwork_ground.py
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1 python3 scripts/test_patchwork_ground_ros.py
```

Validate real MID-360 recordings at low speed before treating the adapter as
qualified: sparse ground, chair legs, 3..5cm obstacles, cluttered turns, slopes,
changing stance, glass/reflections, false-negative obstacles and processing tail
latency on Orin. Synthetic results do not certify real tracking or segmentation.
