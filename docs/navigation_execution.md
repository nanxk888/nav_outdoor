# GPS / LIO navigation execution

For the separate GPS-free short-range full-path mode, see
[lio_local_navigation.md](lio_local_navigation.md). It does not need this geographic
registration or the Nav2 launch. Partial/frontier exploration remains preview-only.

Robot forward is the red emergency-stop-button side. The operator confirmed Livox
IMU +X was initially assumed forward, but the operator's front-chair check confirmed
the arrow was reversed. Robot forward now uses projected IMU -X (180 degree horizontal
mounting correction), consistently in browser arrows, geographic registration and
navigation_base odometry/TF/twist. Raw LIO and sensor clouds are NOT rotated or negated.
Restart the workspace and redo GPS/manual-bearing registration after updating.
The tilted IMU is NOT treated as a level vendor body frame.

## Startup and operation

Build and use the existing unified launch on Orin:

```bash
cd /workspace/nxk
bash scripts/build_real_outdoor.sh
bash scripts/launch_real_workspace.sh sensor_height_m:=0.46 control_config:=/root/.config/nxk/control.json
```

The D1 filter/UDP receiver remains externally managed. Livox is now guarded by
the unified launcher (see `livox_unified_startup.md`). Do not
launch duplicate instances. The height is the actual INITIAL IMU height, not the
height after standing. No SDK mode is required to start perception or the web UI.

Nav2 is now selected by default, but waits for a fresh GPS/LIO initial registration.
An omitted `nav_profile` uses the project's OSM file and example datum; an explicit
profile can override geometry/datum. `with_nav:=false` disables Nav2 startup.

1. Keep the robot stationary, enter its actual compass heading (north 0, east 90),
   and click **GPS + direction alignment / navigation registration** in the web UI.
2. Wait for the live reference status to report registered and Nav2 ACTIVE. HTTP
   202 only means queued, not accepted. Invalid/stale/mismatched samples are rejected.
3. Manually select loco. Select a destination on the road network and click start
   navigation. The coordinator attaches a new receiver session; it does not stand
   the robot or select loco automatically.
4. A blue line on OSM is the actual Nav2 planning output (not a promise the route
   remains traversable). Stop/cancel terminates the current software session.

## Coordinates and command path

`navigation_reference` derives a virtual yaw-only `navigation_base` below the IMU,
`/odometry/local`, and timestamped TF from native LIO. It retains the full tilted
orientation for the IMU/sensor transform. This is a planar projected reference, NOT
a calibrated vendor `base_link`. GPS/IMU co-location and IMU +X/body-forward are
operator-supplied approximations, with the 180 degree field correction above taking
precedence over the former +X assumption. The circular 0.6 m footprint is a provisional
conservative planning model, not a measured robot outline.

GPS plus manual heading initializes `map -> odom`. This is **initial registration,
not continuous GNSS/LIO fusion**. GPS loss after registration permits LIO dead
reckoning, with accumulating drift. Starting without GPS does not locate an arbitrary
geographic goal; the native LIO memory tool remains a candidate planner only.

OSM A* -> stable global route -> rolling 5 m local Smac plan with filtered live
clouds -> regulated pure pursuit -> velocity smoother ->
`/real_navigation/cmd_vel_candidate` -> web coordinator -> authenticated UDP -> D1
receiver -> vendor UserCommand. Limits: forward 0..0.3 m/s, yaw +/-0.5 rad/s;
desired cruise 0.2 m/s. No automatic reverse or mode switching.

The live reference requires fresh LIO, accepted ground filtering and fresh observed
memory. Source expiry/discontinuity invalidates GPS registration; re-register after
recovery. Lost SDK, receiver, browser heartbeat, command stream or live reference
stops the navigation session; it does not automatically resume. Software stop is
not a hardware emergency stop.

Road IDs/approval booleans are not mandatory. Off-road start access is bounded to
20 m and still requires observed-free evidence across the whole connector, now
including the configured corridor clearance. Unknown/blocked space is not silently
opened. Automatic frontier exploration and frontier-to-road handoff are NOT wired
to actuation. Off-road destinations are not yet supported by this road planner.

## Verification scope

`test_navigation_reference.py` and `test_navigation_reference_ros.py` cover tilted
heading, source TF/odometry, GPS/manual-heading anchoring and expiry invalidation.
`test_real_navigation_execution_ros.py` runs real Nav2 against synthetic sensors and
an authenticated loopback UDP receiver, in isolated domain 94. No real robot is
connected. These checks do not validate hardware tracking, OSM accuracy, terrain
traversability or outdoor collision avoidance.
