# Orin unified startup

From `/workspace/nxk`, after sourcing ROS, Livox and verified dependency underlays:

```bash
bash scripts/build_real_outdoor.sh
bash scripts/launch_real_workspace.sh \
  sensor_height_m:=0.46 \
  control_config:=/root/.config/nxk/control.json
```

Ground filtering is now enabled by default. The height is the **actual initial IMU height**,
not a permanent height assumption: 0.46 m is only appropriate for the measured low stance.
Override it when initializing in a different stance. Disable with `with_nav_cloud:=false`.
Livox hardware driver remains externally managed; do not start a duplicate driver.

To include the existing Nav2/OSM planning launch in this same command:

```bash
bash scripts/launch_real_workspace.sh \
  sensor_height_m:=0.46 \
  control_config:=/root/.config/nxk/control.json \
  with_nav:=true \
  nav_profile:=/root/.config/nxk/reviewed_osm_profile.json
```

Use an actually reviewed profile with surveyed datum, width, approved way IDs and an
existing absolute Orin OSM path. The repository example deliberately fails validation.
No profile is generated or marked approved automatically. Nav2 remains opt-in until
localization, body extrinsics, TF and safety inputs have been integrated and verified.
This launch does not implement these missing prerequisites or authorize movement.

Each owned process emits one `[STARTED]` record. A separate 45-second observer emits
one `[READY]` or `[NOT READY]` record per selected subsystem: LIO tracking, GPS receipt,
GNSS gate receipt, both filtered cloud streams plus accepted ground, HTTP response,
and Nav2 lifecycle ACTIVE. GPS receipt may still report status -1 (no satellite fix).
Nav2 ACTIVE is not proof of functional navigation, TF validity or obstacle safety.
Timeouts do not weaken guards or automatically restart processes. Later runtime
failures must still be observed through existing status channels and error logs.
The observer exits after its startup audit; this is not a continuous safety monitor.

Do not run a separate Nav2 launch alongside the unified one. An owned main process
exiting emits a `[PROCESS EXIT]` error with name and exit code, but does not stop other
modules or restart the failed one. Ctrl+C still stops all processes owned by this launch.
Motion safety/watchdog checks remain in effect: keeping visualization alive does not
allow navigation with failed localization or perception. Configuration errors before
process spawning still prevent startup. Stopping the workspace is not a hardware emergency stop.
