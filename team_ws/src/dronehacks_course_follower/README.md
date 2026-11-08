# dronehacks_course_follower

Flies the whole course from the map file: plans a route, flies it with smooth led pose setpoints
and a look-ahead heading, and lands on the pad by a downward twist so PX4 disarms itself.

Why it exists: in the organisers' lab simulator (2026-11-07) localization held (PX4 within
2-6 cm of truth), but their example mission's step setpoints overshot legs by 38-60 cm, swung the
heading ±40° and turned in place facing a wall until cuVSLAM failed. This follower is the one
flown in Michael Bar On's PX4 SITL against the organisers' unmodified bridge and interface, where
it reached the pad in about 27 s (his repo's `sim/README.md`, "The event's own path", 2026-10-28).

## Against the other flight packages (2026-11-07)

All four publish only `/skybeta/setpoint/*` through the kit's interface and never arm or change
mode.

| Package, branch | Flies | Lands | Has run |
|---|---|---|---|
| this one, `navigation/course-follower` | the whole course, planned from the map at start | itself, by twist | this package, launched as the stack launches it, in PX4 SITL with cuVSLAM in the loop through the organisers' 0.3.3 interface, from the ground to disarm (Known risks) |
| `dronehacks_autonomy`, `navigation/team-ws-autonomy` | the whole course; the same design (A*, inflation, arcs, speed profile) | itself | a stand-in PX4 and stand-in ROS |
| `dronehacks_traj`, `navigation/vo-traj-follower` | the whole course with `planner/`'s Follower | holds over the pad for the pilot by default | a stand-in PX4 with drone 1's parameters |
| `dronehacks_manual_maze_tunnel_chicane_gateb_landing`, `main` | fixed waypoints as step poses, through Gate B | the pilot (decision #25) | — |

Step poses are what overshot in the organisers' lab (docs/flight-findings.md). Decision #25 kept
landing with the pilot pending an organiser-supported procedure; the organisers announced on
2026-11-07 that drones have since taken off, flown and landed autonomously. Pick one package for
`COMPETITOR_PKG` by the same bench check and a recorded short hover on the drone.

## What it does

| Phase | Sends | Ends when |
|---|---|---|
| wait | pose over the drone at the map's flight height, heading not commanded | `/skybeta/state` reports Offboard; the drone must be within 0.5 m of the map's launch mark, or the run is refused |
| climb | the same pose, held over the start | odometry within 0.1 m of the flight height |
| fly | the led pose sliding along the rounded route, heading on the look-ahead point | the setpoint reaches the route's end and the drone is within 0.15 m of it, or 2 s later |
| descend | twist: 0.5 m/s down, steered back over the pad at 1/s, capped 0.3 m/s; 1.0 m/s down once on the floor (odometry height steady 0.3 s within 0.3 m of its height at Offboard) | PX4 disarms (`armed` false); with `return_home`, PX4 reports `landed` |
| return_rest | twist 0.15 m/s down, pressing on the pad | `return_rest_s` later; a disarm here ends the run, landed on the pad |
| return_climb | pose straight over the pad at `return_height_m` | odometry within 0.15 m of it, or settled above 90 % of it |
| return_fly | pose sliding straight to over the launch mark at `return_speed_mps`, heading held | over the mark, as the route's end |
| return_descend | the descent's twist, aimed at the launch mark | PX4 disarms |

The pilot taking the drone back (Offboard ends) stops it at once; it sends nothing until a disarm.
PX4's offset from the odometry stays frozen from the pad descent on.

The firmer press on the floor is for PX4's land detector (v1.16.1 `MulticopterLandDetector`):
ground contact needs the thrust setpoint under its low-throttle threshold, and on the floor the
vertical velocity loop gets there only as fast as the setpoint's error and its integrator push it.
At 0.3 m/s skybeta-11 sat on the floor about 4 s before `has_low_throttle` (`035510`); a 1 m/s
error times `MPC_Z_VEL_P_ACC` (4) asks for 4 m/s² down at once. No PX4 parameter is changed.

- **Planner** (`planner.py`): A* on a 5 cm grid, lethal within 0.32 m of a wall (prop reach
  0.28 m + 0.04 m), Nav2-style inflation cost beyond, boulders widened by the rangefinder's
  footprint. Goals in the map's own `order`. The window is flown only when `allow_window` and the
  map says it is passable (the opening spans the flight height with 0.25 m either side and is
  wider than the lethal ring twice); otherwise its opening is walled across and A* finds Gate B.
  A flown window is one straight leg on its normal through the opening's middle, from up to
  0.7 m before to up to 0.7 m after (shortened a cell at a time where a lethal ring covers the
  point), so the 0.4 m corner arcs end before the frame. Why: in 39 SITL flights (2026-11-07) the
  window was the closest pass in 21; the estimate was within a few cm of truth there, but PX4 cut
  the turn into the opening, about 8.7 cm toward the frame. The route is planned once, before
  take-off, so no in-flight re-plan can shorten that lead-in. The ideal dry run takes about 31 s
  with the lead-in, against 27 s without it.
  A* reaches a leg's fixed end through whatever cell the obstacles leave, often one last cell
  beside it: two tight turns the follower rounds with arcs of half that 5 cm leg, flown almost at
  rest. So a corner closer than `MIN_LEG_M` to a leg's end is folded into it, the end sliding along
  the leg's line to where the leg before meets it (never more than a cell toward the opening), when
  the new lines are as clear as the old. Measured offline 2026-11-07 on the course with boulders,
  approach from the east: `python -m dronehacks_course_follower.dry_run --speed 1.0` lands in
  22.8 s against 25.4 s, slowest point within 1.2 m of the window 0.61 m/s against 0.16 m/s,
  window crossing and nearest-wall clearance unchanged; without the boulders the route was already
  one turn and is unchanged.
- **Follower** (`follower.py`): corners as arcs from 0.4 m before, a speed profile within PX4's
  v1.16 default `MPC_ACC_HOR` 3, `MPC_JERK_AUTO` 4 and turn acceleration, an eased jerk-limited
  pace that slows when the drone strays toward a wall, falls behind or its heading lags, heading
  on the point 0.8 m ahead at up to 45°/s, and the pose led by v / 0.95 + a / (0.95 x 1.8), the
  heading by yaw rate / 2.8 (PX4 v1.16 defaults of `MPC_XY_P`, `MPC_XY_VEL_P_ACC`, `MC_YAW_P`).
- **Frames** (`frames.py`): the mission works in the course frame; the node applies
  contracts/frames.md's measured transform `p_odom = o_odom + R(yaw)(p_course - o_course)`, with
  `o_course` the map's launch mark and the team's existing `course_to_odom_yaw/x/y` parameters.
  The stack starts team code with no launch arguments, so without them the node reads the
  transform itself, as the team's `measure_frame` does: the mean odometry of the drone resting on
  the launch mark facing the launch heading over the last 2 s, frozen at arming. A drone that moved
  over 2 cm or turned over 1° in that window is refused: nothing is sent until a disarm, so the
  interface never enters Offboard. A cuVSLAM restart moves the odom origin, so the transform is
  re-read at every stack start and never stored.

Left out on purpose (rejected in SITL, or not needed for a first flight): racing line, heading
plans, turn slow-down, cross-track push, window servo, live re-planning, and the return to the
launch mark (it needs a re-arm the interface cannot give).

## The map

**`maps/course_map.json` is the course as changed on 2026-11-07 evening** [measured by tape, by the
team]: the organisers replaced the maze's three dividers (M1-M3) with one diagonal wall and
put a wall 24 in (0.61 m) inside the north net, where skybeta-11's first follower run hit it on
2026-11-08 at the position its log gives [measured by tape]. The map takes that wall along the
whole north side (assumed: measured only at the hit, near the diagonal's top end). The diagonal runs from the barrier's north
face, 1.25 m east of where the barrier meets the west net, to 1.40 m short of the original north
net at the old M2 divider's east-west position: (-1.798, 0.508) to (-0.813, 1.648), drawn
0.10 m thick (assumed). The gap from its top end to the north wall was taped at about 3 ft
on 2026-11-08, wider than this map's, so the map errs narrow there. The team's SolidWorks export
(`navigation/new-course-layout`, `course_v3_layout.yaml`) draws the wall longer, to (-0.693, 1.811),
which would leave under 0.6 m against the north wall: the taped gap rules it out, so it is not in the map.
**The window and the flight height are tape-measured (2026-11-08):** the opening runs 36 in to
61 in above the floor (0.914-1.549 m), not the published 12-48 in, so the map's `height_m` is its
middle, 1.23 m above the floor, flown with `odom_origin_height_m` 0.10 (the Pixhawk's rest
height). The tunnel is open 0.03-1.85 m, about 1.39 m wide at that height; the walls are 1.73 m.
Everything else is the published v3 map below, unchanged. The route goes
from the launch mark along the diagonal's north-west side, around its top end, to the tunnel.
The rock east of the window is placed from the team's photos and description on 2026-11-08
(estimated, not taped), against the south net. The rock south of the tunnel sits about 4 in off the
east net, so the rangefinder's cone over the tunnel's axis misses it; it is not in the map, which
lets the tunnel's run-out reach its full length on the axis. The rock west of
the window sits about a foot from the south net, and the pad's centre 36 in from the west net and
48 in from the south net, about a metre south of the published pad (both taped roughly by the team).
Stop-and-go's legs keep off the rocks' rangefinder rings without the corner-cutting margin, since
it holds at every corner, and it drops a corner crowding the hold before or after it.
The shortest path ran the leg after the diagonal-wall gap and the turning hold above the tunnel's
north mouth along the north wall, and skybeta-11 hit that wall there on 2026-11-08. The map's first
goal, `corridor`, is a point midway between Barrier 3 and the north wall that the route must pass,
and the tunnel's `lead_m` `[south, north]` puts the north hold midway between the mouth and the wall.
The diagonal-wall gap itself stays the tightest place on the route; taping its narrowest width
settles how tight.
The boulders are still avoided; whether to plan over them (smoother, faster, but under the
rangefinder) is being checked.

The published v3 map it was edited from is written by `sim/generate.py` in Michael Bar On's DroneDepthCamTesting
repo (`write_course_map`, published variant, commit `52c8c27`), from the organisers' published
course OBJ (`dronehacks-course-v3-obj.zip`, SHA-256
`52498970486fdb72ff6460d9b6297fdc128eabc089c7a25a7df6cf445990167c`), cut at 0.8 m. It was used
instead of `contracts/course_v3_layout.yaml` because the contract schema carries the tunnel only
as an axis: the tube's side walls are not in it (the manual navigator rebuilds them in
`_tunnel_side_walls`), and the planner needs them as walls. The OBJ cut has them.

Format (JSON, course frame: x east, y north, metres, origin at the floor centre):
`height_m` (flight height above the floor), `launch` (`[east, north, heading_deg]`, heading
counter-clockwise from east), `segments` (`[x1, y1, x2, y2]` wall cuts), `bounds`
(`[west, east, south, north]`, the net), `boulders` (`[east, north, radius]`), `goals` (by name:
`at`; the tunnel's two mouths `ends`, and optionally `lead_m`, its run-out past each end, which
overrides `tunnel_lead_m`; the window's `along`, `size_m` `[width, height]` and
`span_m` `[bottom, top]`), and `order` (the goal names in the order a run passes them). A
surveyed course goes in a new file passed as `course_map:=`; nothing in the code knows this course.

## Launch

```bash
colcon build --packages-select dronehacks_course_follower && source install/setup.bash
ros2 launch dronehacks_course_follower competitor.launch.py     # as the stack starts it
```

Set `COMPETITOR_PKG=dronehacks_course_follower` and `COMPETITOR_LAUNCH=competitor.launch.py` in
`/etc/skybeta/drone.env` for the stack to start it; it runs `ros2 launch` with no arguments
(`run_competitor.sh`), so every default below is what flies. Restart the stack with the drone on the
launch mark, then before arming set it **exactly on the mark, facing the launch heading, and leave it
still for 2 s**: the log says "course placed from the rest on the mark" at arming. The follower cannot
tell where the mark is, so a misplaced drone flies the whole route shifted by the same error.

| Argument | Default | |
|---|---|---|
| `course_map` | packaged | a surveyed map |
| `course_to_odom_yaw`, `_x`, `_y` | NaN | a measured transform, all three or none; none reads it from the rest on the mark at arming |
| `odom_origin_height_m` | 0.10 | the odom origin's height above the floor: skybeta-11's Pixhawk sits about 4 in (0.10 m) up at rest, measured by the team on 2026-11-08; 0 flies odom z = map height |
| `speed_mps` | NaN | a fixed top speed; NaN flies the speed ladder's current step (below) |
| `speed_ladder` | stop:0.3,0.5,0.7,0.85,1.0,1.0+return | rising steps: stop-and-go, smooth top speeds (1.0 is the kit's `MPC_XY_VEL_MAX`), the return on top (below) |
| `route_mode` | empty | `stop` or `smooth` forces the mode; empty takes the ladder's step |
| `stop_speed_mps` | 0.3 | stop-and-go's leg speed when `route_mode:=stop` forces it |
| `stop_turn_rate_dps` | 30 | stop-and-go's in-place yaw at each hold |
| `stop_arrive_m`, `stop_settle_s`, `stop_hold_timeout_s` | 0.15, 1.0, 15 | a hold waits until the drone is this near its waypoint and steady this long, or goes on after the timeout |
| `ladder_file` | /root/team_ws/course_follower_ladder.json | the ladder's step across runs (host: `~/team_ws/`) |
| `rate_hz` | 50 | follower publish rate |
| `pose_lead` | false | true leads the pose by velocity and acceleration: faster, but only safe with the interface at 50 Hz (Known risks) |
| `allow_window` | true | false always takes Gate B |
| `land` | true | false holds over the pad for the pilot |
| `tunnel_lead_m` | 0.75 | > 0 flies the tunnel as one straight leg from that far outside its mouths; it touched more in SITL, but skybeta-11 hit the wall west of the south mouth turning nearer it on 2026-11-08, so the real course wins |
| `heading_mode` | fixed | `fixed` commands the heading at Offboard from the climb to the landing, never yawing; `ahead` faces the point 0.8 m ahead on the route (below) |
| `turn_rate_dps` | 45.0 | cap on the commanded heading's turn rate (`ahead`); PX4's `MPC_YAWRAUTO_MAX` default |
| `yaw_comp` | false | true offsets the pose against the airframe's push PX4 relearns after each yaw (below); not yet flown |
| `yaw_comp_tau_s` | 4.5 | PX4's velocity integrator lag as modelled, `MPC_XY_VEL_P_ACC` / `MPC_XY_VEL_I_ACC` at stock values |
| `yaw_comp_gain` | 1.71 | PX4's pose-to-acceleration gain, `MPC_XY_P` x `MPC_XY_VEL_P_ACC` at stock values |
| `yaw_comp_max_m` | 0.20 | cap on the compensator's setpoint offset |
| `yaw_comp_push_gain` | 1.25 | the push per unit of hover accelerometer reading, fitted on the corners of two flights |
| `setpoint_rate_hz` | 10 | the interface's own rate, the kit default |
| `record` | true | record the run (below); false keeps the follower up for another attempt after a disarm |
| `return_home` | empty | `true`/`false` forces the return to the launch mark after the pad landing (below); empty takes the ladder's step, on only at its top; not yet flown on the drone |
| `return_height_m` | 2.3 | the return's height above the floor, over the 1.727 m walls; must stay under 3.0 m above the odom origin |
| `return_rest_s` | 2.0 | the rest on the pad after PX4 reports landed; over 3.0 is refused |
| `return_speed_mps` | 0.5 | the return's straight slide to over the launch mark |

### The return (rule 7.10)

After landing on the pad the drone may take off by itself, fly over the barrier into the start
area and land and disarm within 2 ft (0.61 m) of the launch mark, all within 3 min, for a bonus; a
miss loses only the bonus. Rule 7.7 lets it fly above the walls (1.727 m). With `return_home:=true`
the descent onto the pad goes on until `/skybeta/state` reports `landed`, then presses down for
`return_rest_s` (2 s) so the landing is plain on the video, climbs straight up over the pad to
`return_height_m` (2.3 m), slides straight to over the launch mark at `return_speed_mps`
(0.5 m/s) with the heading held, and lands there as on the pad. **COM_DISARM_LAND must be 5 s**
(raised on skybeta-11 with the organisers' permission): at 2 s PX4 disarms during the rest and the
run ends on the pad, which still counts. PX4's offset from the odometry is frozen from the pad
descent on, so the rangefinder's floor error on the pad and its steps over the wall tops never
reach a setpoint. A lost odometry on the return holds 3 s, then goes straight down. A run that
landed on the pad climbs the speed ladder even if the return fails.

Flown once in Michael Bar On's PX4 v1.16.1 SITL (2026-11-07, the older v3 course map, organisers'
0.3.3 interface at 10 Hz, `COM_DISARM_LAND` 5 set in SITL, `return_home:=true`, ladder speed
0.5 m/s), scored from PX4's ground truth: it lifted off the pad 2.95 s after PX4 reported landed (no
disarm on the pad), peaked 2.18 m up with 37 cm to the nearest wall, the slide began on the settle
rule at 2.13 m by the tracker, PX4's height estimate ran 0.15-0.27 m above truth with no step over
the wall tops (largest per-sample change 3 mm), and it disarmed 0.11 m from the launch mark 126 s
after arming (the return itself 31 s). Not yet flown on the drone.

```bash
ros2 launch dronehacks_course_follower competitor.launch.py return_home:=true
ros2 run dronehacks_course_follower dry_run --return-home      # reports the landing's distance from the mark
```

### Heading mode: does yawing drift the drone?

In the square test (`flights/20261108-013018-light-square`) the drone drifted 10-20 cm off each
corner after an in-place yaw and took about 10 s to crawl back; a hold without a yaw stayed within
about 5 cm. So the default, `heading_mode:=fixed`, flies the route without yawing at all, the
camera facing the launch heading throughout: the push PX4's integrator has learnt stays pointing
the right way, and no hold waits on a turn. Facing north, the camera looks back out of the tunnel's
lit north mouth while the drone flies south through it. `heading_mode:=ahead` turns to face along
the route, slower with `turn_rate_dps:=20.0`.

### The tracker jumping: straight down

cuVSLAM has jumped metres at once on skybeta-11 (during a manual landing and on the ground,
`flights/2026-11-08-slot.md`). In any Offboard phase, a pose more than 0.3 m from the last one and
faster than 3 m/s ends the run: the mission descends straight down where it is, steering on
nothing, until PX4 lands and disarms, and logs `the tracker jumped ... m at once`.

### Push compensation: cancel the drift and still turn

The drift is a push fixed to the airframe (about 0.10 m/s² forward-left of the nose in
`20261108-013018`, 0.25 in `20261108-003042`). PX4's velocity integrator cancels it in world axes,
so after a yaw its correction points the old way until it relearns it. With `yaw_comp:=true`
(`yaw_comp.py`) the climb hovers over the start until 2.5 s of steady hover (under 0.2 m/s, turning
under 5°/s; at most 10 s of waiting) give the push from PX4's accelerometer
(`/fmu/out/sensor_combined`, FRD: forward = ax, left = -ay, times `yaw_comp_push_gain`). It then
models the integrator as a lag of R(heading) d over `yaw_comp_tau_s` and adds
-(R d - L) / `yaw_comp_gain`, capped at `yaw_comp_max_m`, to every pose setpoint in odom before
PX4's offset; steady flight along the route refines the push over 30 s. The log names the push
once and the offset each second, and `/course_follower/yaw_comp` carries both (JSON) for the bag.
Without `px4_msgs` or accelerometer data it stays off and the climb does not wait. The defaults
are fitted on replays of those two flights at stock PX4 gains, not yet flown.

```bash
ros2 launch dronehacks_course_follower competitor.launch.py yaw_comp:=true
```

### Power up on the launch mark: PX4 and the tracker must share a frame

The kit's interface hands PX4 our odom setpoint unchanged (ENU to NED). PX4 takes x, y and yaw from
vision but its height from the rangefinder (`EKF2_EV_CTRL` 9, `EKF2_HGT_REF` 2), with its origin
where the flight controller powered up; odom's is where the stack started. On skybeta-11
(2026-11-07, `flights/20261108-003042-light-square` on `core/flight-iteration`) PX4 read -0.72 m
on the floor against odom's 0.01 m and the square test flew to 1.33 m for 0.74 m. **Power-cycle the
drone, FC and Jetson, sitting on the launch mark.** What remains is handled as the square test on
this branch does: every pose setpoint carries PX4's offset from the odometry (PX4's position from
`/skybeta/state` `local_enu`), measured at rest and followed, smoothed over 1 s, in flight, since the
rangefinder reads wrong on the floor and true in the air. Over 0.3 m at rest it streams nothing, so
the kit never takes over: the log says `PX4 and cuVSLAM disagree by 0.73 m: not streaming, so no
Offboard. Power-cycle ...`, then `PX4 offset from cuVSLAM 0.16 m: streaming`.

### The speed ladder: the team chooses each run's step

With no `speed_mps` given, a recorded run flies the ladder's current step, read from `ladder_file`
at launch. **The step only moves by hand**: a run records its outcome (landed on the pad or not)
and leaves the step alone, so the team decides between runs whether to repeat it, test something
else at the same speed, or go up. The default ladder is `stop:0.3,0.5,0.7,0.85,1.0,1.0+return`:
step 0 is stop-and-go at 0.3 m/s (below), then the smooth follower at each top speed, and at the
top the same 1.0 m/s with the return to the launch mark. An entry is a speed (smooth),
`stop:<m/s>` (stop-and-go), either with `+return`. `route_mode:=stop|smooth` and
`return_home:=true|false` override the step. The log names it at launch (`stop-and-go at 0.30
m/s ... (ladder step 1 of 6)`) and at the end. The file, `~/team_ws/course_follower_ladder.json`
on the host, keeps every run; set its `step` (0 is the first) to choose the next run, or delete
it to go back to stop-and-go. **A file left from an older ladder points at the wrong step: delete
it before the first run.**
The ladder cannot see wall touches: watch each run, and step back by hand after one.

### Stop-and-go: the safe bottom step

Flown as the square test flew its box on skybeta-11: the planner's waypoints (its A* route after
its own shortcutting, not the rounded arcs) as straight legs. A pose slides along each leg at
`stop_speed_mps` (0.3) with the heading held, never more than 0.3 m ahead of the drone; at every
waypoint a hold first yaws in place to face the next leg at `stop_turn_rate_dps` (30°/s; none with
`heading_mode:=fixed`), then waits until the drone is within `stop_arrive_m` (0.15 m), within 10°
of the leg's heading and steady for `stop_settle_s` (1 s), so the 10-20 cm drift a yaw leaves dies
out before the leg. A hold that never settles goes on after `stop_hold_timeout_s` (15 s) and logs
it. Every leg is checked clear of the lethal ring (prop reach 0.28 m + 0.04 m, boulders' footprints
included) cell to cell, as the A* shortcut checks its own; a leg that is not gets the A* path's
corners in between. The window's lead-in point is a waypoint, so the drone holds there facing
through the window before crossing. yaw_comp, PX4's offset, the placement on the mark, the landing
and the return work as in the smooth mode.

Dry runs (ideal drone: no climb, arrives at once, so a hold is its yaw at 30°/s plus 1 s), from
Offboard to the final disarm, against the 3-minute limit:

| Map | Course and pad landing | With the return (COM_DISARM_LAND 5 s) |
|---|---|---|
| packaged | 80 s | 95 s |
| older v3 (SITL) | 113 s | 128 s |

A real hold also waits for the drift to settle, a few seconds after each turn, so a real run is
longer; the return at the bottom step is off by default (the ladder adds it only at its top).
Flown once in Michael Bar On's PX4 v1.16.1 SITL (2026-11-07, the older v3 map, 17.1 m of legs,
`route_mode:=stop`, `COM_DISARM_LAND` 5 s), scored from PX4's ground truth: no wall touch, nearest
wall 38 cm (at take-off; 39.8 cm along the route), no hold timed out, 3 cm from the pad's centre,
136 s from arming to the pad and 151 s to the disarm, 1.33 times the ideal dry run. At that ratio
the v3 course plus the return (31 s in SITL) would pass 3 minutes; the packaged map's shorter
course (80 s ideal) would not.

```bash
ros2 launch dronehacks_course_follower competitor.launch.py route_mode:=stop
ros2 run dronehacks_course_follower dry_run --route-mode stop --return-home
```

### Every run is recorded

With `record:=true` the launch does what the square test's does: it writes
`/root/team_ws/flights/<UTC time>-light-follower/` (the host's `~/team_ws/flights`) with
`meta/manifest.txt`, the follower writes the transform it flew as `meta/params/course_follower.yaml`
(where the flight report reads it), and the launch records a light
rosbag2 bag (`-s mcap -a`, excluding images, depth, point clouds and the tracker's visualisation,
as `dronehacks_telemetry`'s light profile does) from launch, so the pre-arm start-up and the
Offboard handover are in it. Once a climb has started and PX4 has disarmed, the follower logs
"flight over and disarmed", exits 5 s later, and the launch sends the recorder SIGINT, which
closes the bag cleanly. One flight per launch: restart the stack on the mark to fly again, which
the odom origin needs anyway. Arming on the ground or a refused route does not end it.

After the flight, fetch the ULog into the folder's `px4/` and run
`python3 -m dronehacks_telemetry.analyze <folder> --course-map contracts/course_v3_layout.yaml`
(branch `telemetry/flight-recorder`). The bag is lost only if the stack is killed first; then
`ros2 bag reindex <folder>/bag -s mcap` in the container repairs it. Checked in a ROS 2 Humble
container with a stand-in interface and a scripted arm, Offboard and disarm (2026-11-07): the bag
closed with its metadata and the analyzer read the folder. Not yet run in the skybeta image,
which needs `rosbag2_storage_mcap`.

Plan and print without ROS or a drone:

```bash
ros2 run dronehacks_course_follower dry_run            # or: python3 -m dronehacks_course_follower.dry_run
ros2 run dronehacks_course_follower dry_run --no-window --steps 40
ros2 run dronehacks_course_follower dry_run --heading-mode fixed
```

## Known risks

- **A wrong accelerometer bias from boot pushes the drone at its first turn.** In SITL
  (2026-11-07), PX4's estimator learned a horizontal accelerometer bias of up to 0.4 m/s² in the
  first seconds after boot with the drone level and still. Take-off then slid 0.3-0.47 m, and the
  route's first turn, the first maze corner, let the estimator correct it, pushing the drone about
  0.2 m into the wall end. Every first-corner touch below came from this, not from the follower:
  with `ekf2` restarted before arming, plain poses cleared the corner in 4 of 4 (nearest
  38.6-41.1 cm) and slid 5 cm or less on take-off. Turning over the start first moved the push
  onto the launch mark instead, so it is not in the package. Whether the real drone does this is
  not known. **Before arming, check the horizontal accelerometer bias** (`estimator_sensor_bias` in
  QGroundControl's MAVLink inspector); above about 0.1 m/s², power-cycle the drone still and level
  on the mark.

- **Led poses through a 10 Hz interface, so plain poses by default.** The interface resends the
  latest pose at `setpoint_rate_hz`, 10 Hz by default, so each led pose is held 100 ms. This
  package itself, flown in Michael Bar On's PX4 SITL through the organisers' unmodified 0.3.3
  interface at 10 Hz from the ground to disarm with no launch arguments (2026-11-07, 4 flights
  each): led poses touched a wall at the first maze corner in 2 of 4 (nearest 19-21 cm against the
  28 cm prop reach) with route times 37-41 s; plain poses touched there in 2 of 6 (nearest
  19-41 cm), route times 58-62 s, half the yaw error. With the interface at 50 Hz, led poses
  flew 27-32 s cleanly in earlier SITL runs: ask the organisers whether 50 Hz is allowed.
- **A dark tunnel without odometry stops the setpoint.** Odometry older than 0.5 s pauses the
  pace, so the setpoint holds; in SITL with the tunnel blinded and no tracker output, PX4 then lost
  its position and landed in the tunnel. This bites only if the kit's cuVSLAM stops publishing in
  the dark; it kept publishing on a blank view in the organisers' lab.
- **The climb gate.** PX4 holds the height by its own estimate, which in SITL sat 0.1-0.17 m off the
  tracker's z, so the climb also ends once the tracker's height has held still for 1.5 s above half
  the flight height; before, one SITL flight hovered on the mark for minutes.
- **PX4 gains.** The lead and speed profile assume PX4 v1.16 defaults; check them on the drone.
- **The twist landing is not organiser-confirmed.** The interface does not floor twists (it
  floors poses at 0.3 m). PX4 detects ground contact only while the setpoint descends at
  1.1 x `LNDMC_Z_VEL_MAX` (0.275 m/s at 0.25), then disarms after `COM_DISARM_LAND`.
- **Heights.** Odom z is height above where cuVSLAM started; set `odom_origin_height_m` once measured.
  At its default 0 the flight height is 0.8 m above the odom origin: in SITL the body flew about
  1.0 m above the floor. The measured window spans 0.914-1.549 m.
- **Placement on the mark.** Read from the rest, the transform is only as good as where the drone
  sits and where its nose points: 5 cm off the mark is 5 cm off every wall, and 2° off the heading
  is 7 cm at 2 m. Tape the mark's outline and a heading line on the floor.

## Bench check, props off

Launch with `record:=false` for steps 3-8: with recording on, the follower exits after the first
disarm that follows a climb (step 7).

1. Props off, drone on the launch mark, stack restarted there. `ros2 run dronehacks_course_follower dry_run`:
   it prints the waypoints, "window flown" (or closed) and lands in the ideal run.
2. Read, do not write, PX4's `MPC_XY_P` (0.95), `MPC_XY_VEL_P_ACC` (1.8), `MC_YAW_P` (2.8),
   `MPC_ACC_HOR` (3), `MPC_JERK_AUTO` (4), `MPC_XY_VEL_MAX` (1), `LNDMC_Z_VEL_MAX` (0.25),
   `COM_DISARM_LAND` (2). A different value means the lead or landing needs re-deriving.
3. Launch with no arguments: the log names the map, the waypoint count, window flown or closed,
   the flight height, and "placed from the rest on the launch mark at arming".
4. With the drone still on the mark, `ros2 topic hz /skybeta/setpoint/pose`: about 50 Hz.
   `ros2 topic echo --once /skybeta/setpoint/pose`: x, y on the drone's odometry, z the flight
   height, orientation all zero. Pick the drone up: the stream stops while it moves and resumes
   2 s after it is set down still. Arm while carrying it: "the course cannot be placed", no
   Offboard. Disarm, set it on the mark facing the launch heading, wait 2 s, arm: "course placed".
5. Pilot arms in Altitude and switches to Position: `/skybeta/state` shows `offboard_active` true
   and the log says "climbing". Lift the drone by hand to the flight height: "at height: flying".
6. Carry it slowly along the first corridor, turning it to face where the pose's orientation
   points (the pace waits while the drone's heading trails by 30° or more): the pose slides ahead
   of the drone along the route in the right physical direction, and the orientation turns toward
   the next corridor before the corner. Hold the drone still: the pose stops about 1 m ahead.
   A wrong direction means the transform is wrong: stop.
7. Flip the mode switch: the log says "Offboard ended" and the pose stops. Disarm: "waiting".
8. Arm again 1 m from where it was placed and switch to Position: "refusing the route" (the
   transform stays placed after a disarm while the stack runs).
9. First props-on flight with `land:=true`: a low hover over a mat, then the descent. Go if PX4
   disarms within 6 s of touchdown with no bounce, three times out of three.

## Tests

No ROS needed:

```bash
python3 -m pytest team_ws/src/dronehacks_course_follower/tests -q
```

They pin the planner on the packaged map and on that map turned a quarter turn, the follower's
led pose, pace limits, heading rate and fixed heading, the course-to-odom transform, the mission's phases and the
landing trigger, the push compensator's signs, step, decay and cap (and that it cancels the drift
in PX4's stock loops), a dry run to a landing, and a guard that no course coordinate or wall name
appears in the package's code.
