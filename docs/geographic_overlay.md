# OSM / GPS / LIO unified display

This is explicit geographic registration for visualization, **not a GPS/LIO EKF**,
not a navigation TF publisher and not a way to bypass navigation safety checks.
No new ROS publishers, robot commands or localization initialization calls are made.

1. Outdoors, wait for valid GPS and fresh LIO/filtered cloud data; stop the robot.
2. Open Parameters and status -> initial direction / geographic alignment.
3. Enter the robot nose bearing: true north 0 degrees, east 90, clockwise positive.
4. Enter measured IMU +X bearing offset clockwise from the nose, and the GPS antenna
   position relative to IMU in IMU X/Y/Z axes, metres. Zero defaults mean approximate
   co-location/alignment, not verified extrinsics. Realsense is not involved.
5. Click the stationary alignment button. Geographic view becomes the main view:
   green = raw GPS antenna, cyan = geographically registered IMU/LIO position and
   track, orange = filtered obstacle points. Antenna and IMU dots need not coincide.
6. Pan/zoom normally, use geographic follow to re-enable following after panning.
   Clear alignment or refresh to discard it. Each browser tab has its own alignment.

Latest source timestamps must differ by <=0.3 s and inputs must be fresh. The click
asserts the robot is physically stationary: this is not an automatic standstill detector
or timestamp interpolation. Do not initialize while moving or during posture changes.
Computer and Orin wall clocks must agree for the source-age check.

The transform uses measured initial LIO translation/yaw, explicit geographic bearing,
and the current full quaternion for antenna lever-arm compensation. World-frame
deskewed points use the SAME rigid transform as the trajectory, never one pose applied
again to points already in LIO world. Existing LIO internal LiDAR/IMU extrinsics must
also be correct. The planar display assumes the native LIO world is gravity aligned.

OSM uses Web Mercator. Physical ENU metre displacements are divided by cos(anchor
latitude) before drawing in Mercator coordinates. At campus scale this makes cloud,
trajectory, roads and the latitude-corrected scale bar consistent. It is a local
tangent approximation, not surveyed geodesy or long-distance map reconstruction.
Do not rescale point clouds manually to make buildings fit.

Raw GPS remains independent. Later GPS updates do not directly shift an already
registered cloud; GPS loss leaves relative LIO dead reckoning explicitly labelled.
This prevents GPS jitter moving the scene but does not correct accumulated drift.
Initial GPS error, manual bearing error, LIO drift, extrinsic error and OSM geometry
error all remain. GPS covariance is displayed as an upstream-estimated axis standard
deviation when available, not guaranteed accuracy. No RTK quality is inferred.

Alignment is discarded on disconnection, stale input, frame/epoch change or a detected
LIO reset. The backend starts a new epoch on nonincreasing timestamps, >1-second input
gaps, frame changes or large translation discontinuities. This is conservative but
cannot prove detection of every small reset; manually clear/re-align after ANY known
LIO restart. No invalid or expired cloud is kept as a live obstacle overlay.

Deployment: pull, restart the web/workspace process, hard-refresh the browser. No C++
changes or extra Python dependencies for this display feature. Normal workspace build
is still required when other installed launch/package changes have been pulled.

Tests: `node scripts/test_geo_alignment.js`, `node scripts/test_web_interactions.js`,
`python3 scripts/test_navigation_web.py`. Math/HTTP/fake-DOM tests do not replace
outdoor distance, heading, reset and visual acceptance tests.
