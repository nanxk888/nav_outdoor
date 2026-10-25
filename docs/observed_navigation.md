# Observed-space memory and road access

## What is implemented, and what is NOT

The unified Orin launch now enables `lio_memory` (`with_memory:=false` disables it).
It consumes exact-matched native-LIO odometry and ground-filtered nav/clear clouds,
without requiring GPS, `map` TF or simulation assets. It outputs observation memory
and **candidate paths**, NOT velocity, robot modes, TF or a navigation-safe flag.
The web has separate memory-candidate buttons; existing robot navigation remains
guarded. This is not yet unattended explore-to-goal navigation or a qualified terrain map.

## Memory, in both display modes

- Sparse 0.25 m grid: occupied=100, observed planar visibility=0, absent/expired=unknown.
- Each observation earns its own 120 s TTL, at most 60,000 retained cells. No scan logs.
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

The ROS road planner defaults `offroad_start_limit_m` to 10 m (0 disables, maximum 30).
It selects the nearest segment, and requires fresh memory evidence covering a 0.5 m
footprint around the connector. Unknown, blocked, stale or missing TF rejects access;
it does not assume that all non-road space is open. A blocked nearest connector currently
does not search alternative detours: it safely fails for operator review.

Only the verified narrow connector is temporarily opened in the road prior; all other
off-road area stays blocked. Access is withdrawn when the connector evidence becomes
stale/blocked. Existing global obstacle memory still participates in road routing.

Actual road navigation still needs reviewed profile, valid `map -> odom -> base_link`
and native-LIO-to-map transforms, local planner/controller, correct footprint and the
existing safety status producer. Browser-only alignment is NOT this TF. This change
does not complete those remaining dependencies or enable navigation automatically.

## Deploy and verify

Pull and stop the old workspace, source ROS/Livox/verified dependencies, run
`bash scripts/build_real_outdoor.sh`, then use the original unified launch. Hard-refresh
the browser. Check `[READY] memory` and `/navigation/memory/status`. No D1 changes.

Tests: `python3 scripts/test_observed_memory.py`, `python3 scripts/test_osm_roads.py`,
and (only an isolated domain) `ROS_DOMAIN_ID=94 python3 scripts/test_lio_memory_ros.py`.
These are algorithm/ROS fixture checks, NOT outdoor autonomous-navigation acceptance.
