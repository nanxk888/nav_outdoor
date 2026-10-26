# Observed-space memory and road access

## Current policy: no manual road approval

`survey_confirmed` is no longer checked. An absent/empty `approved_way_ids` now
automatically selects supported ground-level OSM roads; a nonempty list is only an
optional user route filter. Private/restricted/unsupported bridge/tunnel ways remain
excluded because they are not represented by this planar graph. `road_width_m`
(default 3 m) is a configurable geometric corridor, not a surveyed-width claim.
The current example profile is usable after setting its real file path/datum/width;
there is no approval checkbox to turn on. Old approval wording below is historical
and does not impose a gate. Robot connection config likewise no longer requires
`hardware_authorized`, `interfaces_verified` or `survey_confirmed` flags.

This does NOT create missing localization/controller nodes. The current automatic
control chain still requires a real producer for `/real_navigation/safety_status`,
the actual body/map transforms and running Nav2. No such producer is implemented
in this repository yet. Removing approvals must not be mistaken for completing
that execution chain or for replacing missing measurements with `true`.

## What is implemented, and what is NOT

The unified Orin launch now enables `lio_memory` (`with_memory:=false` disables it).
It consumes exact-matched native-LIO odometry and ground-filtered nav/clear clouds,
without requiring GPS, `map` TF or simulation assets. It outputs observation memory
and **candidate paths**, NOT velocity, robot modes, TF or a navigation-safe flag.
The web has separate memory-candidate buttons; existing robot navigation remains
guarded. This is not yet unattended explore-to-goal navigation or a qualified terrain map.

## Memory, in both display modes

- Sparse 0.25 m grid: occupied=100, observed planar visibility=0, absent/expired=unknown.
- Each observation earns its own 300 s (5 min) TTL, refreshed by new observations,
  at most 60,000 retained cells. No scan logs. Capacity limits may evict older cells sooner.
- At most 2,000 deduplicated rays per processed scan; no ray beyond a current obstacle.
  No-return areas are NOT marked free. New positive obstacle evidence wins over clearing.
- An 80 m rolling OccupancyGrid is emitted on `/navigation/memory/grid` in native LIO
  frame. Its timestamp is the actual last matched scan, not a fabricated fresh observation.
- Bounded sparse display cells, status, goals and routes use `/navigation/memory/status`.
  `/navigation/memory/path_candidate` is a preview, never connected to UDP velocity.
- LIO gaps >1 s, backwards stamps, frame changes or large pose jumps clear map and goal.
  Missing/stale input invalidates candidate output. Small silent resets still need supervision.

The sensor ray origin uses IMU position as the requested near-co-location approximation.
Zero GPS/IMU mounting offset is now the UI default and no extrinsic entry is required.
This does not calibrate unknown LiDAR/IMU internal transforms or certify body clearance.
Planar visibility cannot establish absence of holes, stairs, overhang collisions or
traversable ground. Native-LIO planning does not pretend the IMU is base_link.

## Candidate planning without GPS

In Parameters -> memory candidate planning, click “no GPS: select LIO relative goal”,
then click the local LIO view. The backend checks frame/epoch/freshness and bounds the
goal to 100 m. Eight-connected A* plans ONLY on remembered free cells, using a 0.5 m
circular footprint and rejecting diagonal corner cutting. Unknown is blocked.

If the target is not yet reachable in known space, planning can return a goal-directed
frontier standoff whose entire tested footprint is still known free. It is explicitly
`partial`, not a complete route. Search is bounded to 20k expansions / ~0.2 s. If no
route/frontier qualifies, it reports failure and does not release a stale path.

Memory updates while the operator manually moves the robot; candidates replan as new
observations arrive. **Automatic driving to frontiers and then continuing exploration
is not connected.** That requires the body-frame controller/TF, terrain/safety validity,
progress/timeout and failure handling to be qualified, not bypassed.

## Geographic display and OSM goals

After explicit GPS/heading registration, exactly the same LIO memory and candidates
are transformed to the OSM display. The “OSM target -> memory candidate” button maps
the selected geographic target back into this native-LIO memory. It does NOT run road
routing or replace the separate Nav2 goal action. Without an earlier valid registration,
an arbitrary geographic goal cannot be located in a newly started GPS-free LIO frame.

## Off-road start -> nearest approved road

The real OSM RoadGraph now supports a bounded start connector, keeping the TRUE start
pose in the path, not snapping the reported robot position. The goal still must lie
in an approved road corridor. Campus boundary, approved ways and directionality remain.

The ROS road planner defaults `offroad_start_limit_m` to 20 m (0 disables, maximum 30).
An existing private profile with an explicit older value overrides this default; update
that field to 20.0 yourself if appropriate. No private profile is overwritten automatically.
It selects the nearest segment, and requires fresh memory evidence covering a 0.5 m
footprint around the connector. Unknown, blocked, stale or missing TF rejects access;
it does not assume that all non-road space is open. A blocked nearest connector currently
does not search alternative detours: it safely fails for operator review.

Only the verified narrow connector is temporarily opened in the road prior; all other
off-road area stays blocked. Access is withdrawn when the connector evidence becomes
stale/blocked. Existing global obstacle memory still participates in road routing.

Actual road execution now uses sensor-derived `map -> odom -> navigation_base`
and native-LIO transforms after explicit GPS/manual-heading registration. See
[navigation_execution.md](navigation_execution.md) for operation and limitations.
Browser-only alignment is not a substitute for the live ROS registration status.

## Reset and staged exploration are different from a full connector

The browser also clears its displayed historical LIO trajectory and local preview
target when the backend memory epoch changes (confirmed reset, including from
another tab). Clearing is not treated as successful merely because HTTP returned
202. New trajectory samples accumulate without joining the old trace. The LIO
estimator origin, geographic alignment and initial-heading-up view are unchanged.

The web “reset LIO observation memory and candidate goal” button clears memory and
the candidate goal, advances the memory epoch, and discards buffered old scans.
It does NOT restart/reset the LIO estimator or change the LIO coordinate origin.
It does not drive/stop the robot or clear separate legacy Nav2 costmaps/overlays.
New scans immediately begin accumulating again.
Actual LIO resets/gaps still automatically clear memory as before.

A 20 m road-access allowance is NOT proof of a free 20 m connector. The road planner
still requires evidence for the entire connector before releasing a complete road
route. With only the first 5 m observed, the memory planner may provide a PARTIAL
frontier candidate within that observed area (with footprint/edge margin), then
replan as new scans arrive. Automatic driving between those frontiers and automatic
handoff to road navigation remain unconnected. A 5 m sensor range does NOT imply
every cell in that range is free or that terrain is traversable.

## Deploy and verify

Pull and stop the old workspace, source ROS/Livox/verified dependencies, run
`bash scripts/build_real_outdoor.sh`, then use the original unified launch. Hard-refresh
the browser. Check `[READY] memory` and `/navigation/memory/status`. No D1 changes.

Tests: `python3 scripts/test_observed_memory.py`, `python3 scripts/test_osm_roads.py`,
and (only an isolated domain) `ROS_DOMAIN_ID=94 python3 scripts/test_lio_memory_ros.py`.
These are algorithm/ROS fixture checks, NOT outdoor autonomous-navigation acceptance.
