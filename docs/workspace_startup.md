# Orin unified startup

Current policy: no road approval fields or mandatory approved-way list are required.
`survey_confirmed` is ignored; configure the actual OSM file, datum and corridor width.
See [observed_navigation.md](observed_navigation.md) for remaining execution-chain
dependencies. Older references below to a "reviewed profile" describe geometry setup,
not a required approval flag.

## Stop / duplicate launches / LIO diagnostics

```bash
cd /workspace/nxk
bash scripts/stop_real_workspace.sh --dry-run  # inspect exact PID targets
bash scripts/stop_real_workspace.sh            # SIGINT, then bounded TERM/KILL fallback
```

This targets the current checkout's unified launch, known module executables/scripts,
GPS bridge configuration and their current descendants. It does not broadly pkill
ROS/Python, stop the D1 receiver, or stop an externally owned Livox/GPS driver.
An owned GPS child is stopped. Already-orphaned Nav2 executables installed outside
this checkout cannot always be attributed safely and are not killed by name.
Run with the same OS user as bringup. No files are deleted. Stop cancels running
software control; it is not a hardware emergency stop. Keep physical supervision.

The normal unified entry holds `.local/real_workspace.lock` to reject duplicate
invocations for this checkout. Direct `ros2 launch` or standalone LIO launches bypass
that shell lock: do not mix those entry points. A lock file remaining on disk is
normal and is not evidence of a held lock; do not delete it to bypass a live process.

LIO validation errors now include source/previous timestamps, sample age, allowed
age, queue sizes and individual flags (1 means triggered). Output validation also
reports `gravity_not_aligned` and `invalid_covariance`. This is error-only detail,
not per-frame logging. Existing validity thresholds and fail-stop behavior are unchanged.
Rebuild C++ on Orin before relying on the new error messages.

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

LIO observation memory and candidate-only planning now also start by default
(`with_memory:=false` disables them). See [observed_navigation.md](observed_navigation.md).

Nav2/OSM planning is now selected by default and waits for GPS/manual-heading
registration. See [navigation_execution.md](navigation_execution.md). To override
the automatically selected project OSM/datum profile:

```bash
bash scripts/launch_real_workspace.sh \
  sensor_height_m:=0.46 \
  control_config:=/root/.config/nxk/control.json \
  with_nav:=true \
  nav_profile:=/root/.config/nxk/osm_profile.json
```

Configure an existing absolute Orin OSM path, numeric datum and road corridor width.
No survey/approval booleans or mandatory road-ID list are required. Supported roads
are extracted automatically when no list is provided. The new sensor-derived
virtual navigation reference supplies timestamped transforms and live readiness;
loading a profile alone does not register the robot on the geographic map.

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
