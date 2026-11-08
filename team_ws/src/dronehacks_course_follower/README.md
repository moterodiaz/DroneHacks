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
| descend | twist: 0.5 m/s down, steered back over the pad at 1/s, capped 0.3 m/s | PX4 disarms (`armed` false) |

The pilot taking the drone back (Offboard ends) stops it at once; it sends nothing until a disarm.

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

`maps/course_map.json` is written by `sim/generate.py` in Michael Bar On's DroneDepthCamTesting
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
`at`; the tunnel's two mouths `ends`; the window's `along`, `size_m` `[width, height]` and
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
| `odom_origin_height_m` | 0.0 | the odom origin's height above the floor; 0 flies odom z = map height, as the manual navigator's `hover_altitude_m` does |
| `speed_mps` | NaN | a fixed top speed; NaN flies the speed ladder's current step (below) |
| `speed_ladder` | 0.5,0.7,0.85,1.0 | rising top speeds, m/s; 1.0 is the kit's `MPC_XY_VEL_MAX` |
| `ladder_file` | /root/team_ws/course_follower_ladder.json | the ladder's step across runs (host: `~/team_ws/`) |
| `rate_hz` | 50 | follower publish rate |
| `pose_lead` | false | true leads the pose by velocity and acceleration: faster, but only safe with the interface at 50 Hz (Known risks) |
| `allow_window` | true | false always takes Gate B |
| `land` | true | false holds over the pad for the pilot |
| `tunnel_lead_m` | 0.0 | > 0 flies the tunnel as one straight leg from that far outside its mouths; it touched more in SITL, so it is off |
| `setpoint_rate_hz` | 10 | the interface's own rate, the kit default |
| `record` | true | record the run (below); false keeps the follower up for another attempt after a disarm |

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

### The speed ladder: each clean run flies the next one faster

With no `speed_mps` given, a recorded run flies the ladder's current step, read from `ladder_file`
at launch; when it ends, a landing on the pad (PX4 disarmed in the descent) moves the next run up a
step, anything else keeps the speed. The log names it at launch (`top speed 0.70 m/s (ladder step 2
of 4)`) and at the end (`speed ladder: landed on the pad, next run 0.85 m/s`). The file keeps every
run. To repeat or go back a speed, edit its `step` (0 is the first) on the host,
`~/team_ws/course_follower_ladder.json`; delete it to start again at the slowest. The ladder cannot
see wall touches: watch each run, and step back by hand after one.

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
```

## Known risks

- **Led poses through a 10 Hz interface, so plain poses by default.** The interface resends the
  latest pose at `setpoint_rate_hz`, 10 Hz by default, so each led pose is held 100 ms. This
  package itself, flown in Michael Bar On's PX4 SITL through the organisers' unmodified 0.3.3
  interface at 10 Hz from the ground to disarm with no launch arguments (2026-11-07, 4 flights
  each): led poses touched a wall at the first maze corner in 2 of 4 (nearest 19-21 cm against the
  28 cm prop reach) with route times 37-41 s; plain poses touched there in 2 of 6 (nearest
  19-41 cm), route times 58-62 s, half the yaw error. The first maze corner is the course's
  weak spot in every mode. With the interface at 50 Hz, led poses
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
  1.0 m above the floor. The window's top bar is at 1.22 m.
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
led pose, pace limits and heading rate, the course-to-odom transform, the mission's phases and the
landing trigger, a dry run to a landing, and a guard that no course coordinate or wall name
appears in the package's code.
