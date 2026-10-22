# Retained FAST-LIVO2 algorithm / navigation-only fork

This independent package snapshots `src/fast_livo2_ros2/{src,include}` in this
workspace. It does not edit or replace the existing `fast_livo` package or its
simulation/mapping launch files. It is NOT a byte-identical copy of the remote
Orin `/workspace/fast-livo2-deep-robotics` checkout, which is not locally accessible.
Both use FAST-LIVO2, but exact Orin source parity must be checked before deployment.

Original FAST-LIVO2 developer: Chunran Zheng, HKU MARS.
Upstream: https://github.com/hku-mars/FAST-LIVO2
GPLv2 LICENSE and per-file notices retained. The inherited package.xml BSD label
is corrected to GPL-2.0-only in this derivative; bundled third-party headers keep
their original notices. Vikit/Sophus/PCL dependencies remain external.

Changes: unique package/executable, strict LIO-only profile; no image input,
disk logging or persistent map export; disable world-cloud/path/MAVROS output;
gravity-aligned local basis with covariance transformation; state measurement
timestamp; IMU-child-frame velocity/gyro and nominal estimator covariance;
bounded input queues and fail-stop checks. No base_link claim, TF, GPS fusion,
automatic respawn or actuator output. Registration still uses the upstream
IMU propagation, point-to-plane update and sliding voxel map, not a new estimator.

Analytic stationary input exposed an inherited degenerate-plane eigenvalue-index
crash. The fork uses the equivalent symmetric eigensolver with sorted indices and
rejects point/line clusters whose plane normal is undefined. The original package
is not patched. Tracking output requires at least 30 matched point-plane features.

Nominal covariance depends on upstream state/noise assumptions. It does not
include full mounting/timing/calibration uncertainty and is not measured accuracy.
The copied optional visual implementation remains compiled as a dependency, but
the executable refuses visual activation. Reducing build dependencies is future work.
