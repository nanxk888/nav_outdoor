# Unified Livox + LIO startup

`launch_real_workspace.sh` now enables `with_livox_driver:=true` by default.
It does not install an operating-system boot service. Web, GPS bridge, memory,
reference and other modules start independently; sensor failures do not close them.

The `livox_lio_guard` process checks the configured host IP, carrier, source route
and radar ping. It never edits network addresses/routes. It checks actual Livox
binary processes (not the `ros2 run` wrapper). An existing driver must use domain 0,
localhost-only 0, Cyclone DDS, the same XML URI and expected MID360 config path.
Conflicts/multiple drivers are reported, not killed or automatically replaced.

When absent, the guard starts an owned driver with CustomMsg, one shared topic,
real sensor source, 10Hz points, livox_frame and the configured MID360 JSON.
It sources the driver setup in a child shell and explicitly sets DDS afterward.
Both streams must have fresh, increasing source stamps for at least one second
before LIO starts. Default wait is 30 seconds; cached data cannot satisfy readiness.

```bash
bash scripts/launch_real_workspace.sh \
  sensor_height_m:=0.46 \
  gps_cyclonedds_xml:=/workspace/D1h/d1h-autonomy/.local/robot/perception/cyclonedds.xml \
  with_gps_driver:=false \
  with_nav:=false \
  control_config:=/root/.config/nxk/control.json
```

Optional arguments:

- `with_livox_driver:=false`: never start a driver; still validate an external
  driver and wait for live sensors before LIO.
- `livox_setup:=/absolute/install/setup.bash`
- `livox_config:=/absolute/MID360_config.json`
- `livox_wait_sec:=30` (5..120 seconds)
- `with_lio:=false`: manage/validate the driver without starting LIO.

Default setup and JSON are under
`/workspace/D1h/d1h-autonomy/.local/sensor-driver/install`.
For outdoor Nav2 omit `with_nav:=false`; GPS/bearing registration remains required.

Ctrl+C cleans up only the driver/LIO owned by this guard. The scoped stop script
also includes guard descendants, never unrelated external drivers. If LIO exits,
the driver remains running; no LIO auto-restart occurs. Driver exit or readiness
failure is logged, while other independent modules stay up. Fix the cause, stop
the old workspace and explicitly launch again. Robot motion does not auto-resume.

Offline tests cover process classification, DDS conflict, network failure and
sensor timing. Actual Orin USB recovery, DDS and process cleanup still require
machine-side validation; no real hardware was operated during development.
