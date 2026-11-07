# lidar-vo-fusion

This method flies the DroneHacks course on drone 1. It's this team's own version, separate from
Mateo's (the PR #5 contracts and his field guide).

- **The route:** planned from the organisers' STEP model through the ground-truth checkpoints.
- **The correction:** the MS200K 360° lidar corrects cuVSLAM's drift **in the flight commands**.
- **PX4's estimate is left alone.** PX4 flies on raw cuVSLAM through the organisers' bridge, and
  team code can't sit in front of it (`docs/kit-interfaces.md`). Our nodes work out how far that
  estimate has drifted from the truth, and shift every setpoint by the inverse.

It is the ROS 2 package **`dronehacks_lidar`**, with `depthcam/` as the package root. Deployment is
in `docs/deploy.md`.

| Node (`ros2 run dronehacks_lidar …`) | Does | Lidar |
|---|---|---|
| `vo_guard` | withholds cuVSLAM poses that can't be trusted (stamp off the clock, tracker failed, jump, spin, gyro disagreement, settling) | no, runs in every run |
| `lidar_localizer` | matches each MS200K revolution to the course map → body pose + covariance | only in allowed runs |
| `pose_fusion` | estimates the drift `C` (truth ≈ `C ∘ odom`), publishes the true pose, `~/ready` and `~/aligned`, sends `C⁻¹ ∘ setpoint` to the kit | uses `lidar_localizer` when allowed |
| `follower` | flies the route: checks the start, takes off, follows CP1 → CP4, holds over the pad, lands; holds still while `~/aligned` is false | no |

`launch/lidar_nav.launch.py` starts the kit's `flight_interface` exactly as the kit's example
does, then these four.

**Status:**
- **Tested:** unit tests on Windows pass, with synthetic data and closed loops on the real
  routes.
- **Not run:** the ROS-level tests (`tests/test_ros_nodes.py`), because this laptop has no
  ROS 2.
- **Not flown.** See "Before it flies".

## The course: map, checkpoints, routes

- **Map:** the organisers' STEP model. `dronehacks-course-v3.step` at the repo root is
  byte-identical to `course/maps/course_v3.step`. `course/12_extract_course_map.py` turns it into
  `course/maps/course_v3_map.json`, and re-extracting gives the same map.
- **Checkpoints:** the ground truth's (`DRONEHACKS.md` §4, `course/maps/course_v3_rules.json`):
  CP1 the end of the first corridor, CP2 Gate A, CP3 the tunnel's south end, CP4 the landing area
  through the window or Gate B, CP5 land and disarm on the pad.
- **Routes:** `follower.make_route` runs the ported planner (`course/planner/`). It grows every
  obstacle by a 0.40 × 0.50 m drone plus 0.05 m margin at a fixed heading (nose north), searches,
  straightens and smooths. It writes `routes/course_v3_window.json` (15.5 m) and
  `routes/course_v3_gate_b.json` (16.2 m), format `dronehacks-lidar-route/1`. Each route records
  its map's and rules' hashes; `tests/test_follower.py` checks them and replans the window route
  to confirm it comes out the same.

## Lidar policy and route, per run

`lidar_nav.launch.py` requires `lidar_policy` and has no default for it. `route` defaults to
`window`. The stack's `competitor` window launches with no arguments, and the kit passes only
`COMPETITOR_PKG` / `COMPETITOR_LAUNCH` into the container. So the choice is the launch file's
name: `lidar_nav_{allowed,forbidden}[_gate_b].launch.py`.

| `lidar_policy` | lidar_localizer | pose_fusion |
|---|---|---|
| `forbidden` | not started | lidar never subscribed; Guard poses and follower setpoints republished unchanged; always ready |
| `allowed` | reads the lidar, publishes poses | true pose and setpoints corrected by `C`; ready once the correction has settled |

## follower

The details are in `follower/mission.py`.

1. **Wait, and open the door for Offboard:** the kit asks PX4 for Offboard once per arming, at the
   pilot's switch to Position, and only if a team setpoint is under 1 s old
   (`auto_offboard.py`). So until `/skybeta/state` says `offboard_active`, the follower streams
   the takeoff pose (its current x and y, the cruise height, its current heading), **but only
   while the checks below pass.**
   - **Checks pass:** `/follower/status` says "on the start, sending the takeoff pose". The
     pilot's switch to Position starts Offboard, and the drone takes off.
   - **Checks fail:** nothing is sent, and the status says why. The kit refuses Offboard, the
     vehicle stays on the ground under the pilot, and PX4 disarms it 10 s after arming
     (`COM_DISARM_PRFLT`).
2. **The pre-takeoff checks:**
   - the true pose must be within 0.25 m of the route's start, at the planned heading within
     10°;
   - `/pose_fusion/ready` must be true.

   cuVSLAM's origin is wherever the stack started, so a drone carried after power-on fails the
   first check. The route is converted through the **measured** course→odom transform
   (contracts/frames.md "Course → odom"; `odom_yaw_deg` / `odom_origin_x` / `odom_origin_y` are
   required parameters with no defaults — see `docs/deploy.md`, "Every run"), and these checks
   cross-check that the measured transform and the physical placement agree. The checks run
   again once Offboard is active, for up to 20 s. They fail there only if
   something changed in the second before the switch. If they are still failing at 20 s, the
   follower blocks and sends nothing: the interface holds the last takeoff pose, and the pilot
   takes over.
3. **Take off,** to `cruise_z_m − rest_height_m` (0.73 m), at the planned heading.
4. **Follow:**
   - the heading is held the whole way, because the planner's clearances are for that one
     heading;
   - the carrot is the furthest path point (up to 0.5 m ahead) whose straight line from the drone
     keeps every path point in between within `max_cut_m` (1.5 cm), so tight corners slow it down;
   - progress only moves forward, and each gate crossed is recorded.
5. **Land:** it holds over the pad with a pose setpoint for the pilot (issue #25 point 4: `land`
   is `false` by default, so the pilot takes over the landing; a velocity-only descent is not a
   contracted landing API, per `contracts/px4.md`). With `land: true` it instead holds over the
   pad for 2 s, then descends with a velocity setpoint (the kit only floors *pose* setpoints).
   PX4 disarms 2 s after touchdown (`COM_DISARM_LAND`), which ends Offboard.
6. **Stop:** when Offboard ends, for any reason, it stops for good. A pose older than 0.5 s
   (a cuVSLAM gap past the timeout) makes it hold still with a zero velocity setpoint in flight
   (or descend straight down while landing); before Offboard it still sends nothing.

Not done: the optional return flight, and the planner's speed profile (PX4 sets the pace).

## vo_guard, lidar_localizer, pose_fusion

- **`vo_guard`** (`pose_fusion/guard.py`): withholds a pose whose header stamp is more than
  0.5 s behind its receive time or 0.05 s ahead of it (another clock, or stale: the gyro rule
  needs both on one clock), when the tracker reports failure, for a jump over 10 m/s, a spin over
  720°/s, a disagreement with PX4's gyro over 3° in 0.5 s, and during 0.2 s of settling after any
  break. `~/tracking` says, frame by frame, whether it passed the pose, so the follower can hold
  from the first withheld frame. It reads **no camera images** (dropped): a
  blank view the tracker doesn't report is caught only through the gyro. Passed poses keep
  cuVSLAM's frame, with fixed variances.
- **`lidar_localizer`** (`pose_fusion/localizer.py`): the bench-tested `course/lidar_localize.py`
  matcher, deskewed and tilt-corrected with PX4's attitude. It reports covariance as
  `cov_scale × σ² × (JᵀWJ)⁻¹`, holds the first pose until 3 trusted matches, and uses the
  past-test mount (yaw offset 87.4°, clockwise, scan plane 0.035 m).
- **`pose_fusion`** (`pose_fusion/fusion.py`, `setpoints.py`):
  - **The estimate:** the planar `C` from gated lidar fixes, compared at their own timestamps.
  - **Slew limits:** `C_applied` moves at most 0.10 m/s and 3°/s.
  - **Re-acquire:** after 5 consistent gated fixes.
  - **Guard breaks:** `C` is kept.
  - **Commands:** pose x/y go through `C⁻¹` (an all-zero quaternion stays all-zero); twist x/y
    are rotated.
  - **Refresh:** the follower's last pose is re-sent at 10 Hz while `C` moves, but only while the
    follower is still streaming (its last pose under 0.5 s old). A re-sent pose from a silent
    follower would look live to the kit's Offboard rule.
  - **Ready:** `~/ready` is true when at least 10 fixes have fused, the last within 0.5 s, and
    the applied correction is within 2 cm / 0.5° of the estimate.
  - **Frame breaks:** `~/aligned` turns false at every VO gap (cuVSLAM may have re-initialised,
    so its frame may have jumped) and after 2 gated fixes in a row. It is true again once a fix
    passes the gate and `C_applied` is within 5 cm / 2° of `C`. While the follower holds still
    with a zero velocity setpoint, which doesn't depend on `C`, pose_fusion applies the re-learnt
    `C` at once.
- **`follower`, holding still:** in flight it holds with a zero velocity setpoint while the pose
  can't be flown on. That is from the first frame the Guard withholds (`~/tracking`), while no
  fresh pose arrives (none for 0.15 s, or the newest captured over 0.5 s ago), and while
  `~/aligned` is false. A held *pose* setpoint would be the wrong thing: PX4 gets cuVSLAM's poses
  without our Guard's settling, so its frame can jump while our chain is still blind. If the hold
  lasts 5 s, it lands straight down.
- **Rules they keep:** no node writes to `/fmu/in/*`, sets a PX4 parameter, arms, disarms,
  changes mode, or restarts the organisers' processes. Acro is never used (AGENTS.md §3.6).

## Results (synthetic, Windows, no ROS)

| Test | Result |
|---|---|
| the shipped **window** route flown in closed loop: PX4-like vehicle (τ 0.4 s, ≤ 1.0 m/s like `MPC_XY_VEL_MAX`), whole mission | CP1 → CP4 in order, then hold, land, disarm, stop; **worst 14.9 mm off the planned path** (margin 50 mm); 33.7 s along the route, mean 0.46 m/s; lands within 5 cm of the pad centre |
| the shipped **Gate B** route, same | worst 14.8 mm off the path; 41.0 s |
| drift correction on the commands: odom drifts 0.43 m / 6.9° over 60 s | true position off the target: 0.208 m uncorrected → 0.003 m corrected |
| Offboard, with the kit's rule modelled (a setpoint under 1 s old at the switch, one attempt per arming) | checks pass: the takeoff pose streams before Offboard and is the first command after it (no jump). Carried 0.6 m off the mark, or turned 30°: nothing sent, Offboard refused, the drone stays on the ground. Lidar not settled at the switch: refused; a switch once ready flies. Moved in the second before the switch: blocked. The mission without this fix never gets Offboard (8 of these tests fail on it) |
| refresh from a silent follower | never: only while its last pose is under 0.5 s old |
| re-acquire after a cuVSLAM frame jump of 1.0 m / 20° | no setpoint jump; within 3 cm / 1° after 30 s |
| tracking lost 0.5 s mid-route, then a frame jump of 0.6 m / -0.4 m / 15° that PX4 sees at once and our chain 0.2 s later, flown through the whole chain (`tests/test_frame_break.py`) | holds still from the first withheld frame (0.14 s later on the quiet pose stream alone, before the jump); PX4's jump moves the drone 0.8 mm; carries on 1.3 s after the break began: **worst 12.2 mm off the path** (10.2 mm with no break). Without the realign hold: 203 mm. With the lidar gone after the break: holds 5 s, then lands straight down |
| localiser on the real map, synthetic MS200K | 0 lost; 1.3 mm RMS (idealised) |

The vehicle model ignores PX4's real tracking error, vibration and wind. The real numbers come
from the bench: lidar against taped marks read 5-12 cm (`course/docs/bench-results.md`).

## Before it flies

1. **Run the ROS tests** (`tests/test_ros_nodes.py`) on a ROS 2 Humble host with `px4_msgs`
   built, or on the drone after `team_build.sh` (`docs/deploy.md`, "Run the ROS tests"). They
   drive all four nodes under `rclpy`, with a pseudo-terminal as the MS200K's port. Nothing here
   has run under ROS yet. A user-space install on this laptop was blocked as running downloaded
   code.
2. **On drone 1:**
   - confirm `/dev/ms200` in the container, the past-test mount values, and `rest_height_m`
     (0.07 m);
   - the kit's container pins numpy below 2. No numpy-2-only call was found, but nothing has run
     on numpy 1.x;
   - calibrate `cov_scale` on the bench.
3. **The props-off check, the carry test, a hover, then the first leg,** with the pilot ready
   (`docs/deploy.md`).
4. **Margins:** the tightest spot leaves +3 mm beyond the planner's 5 cm margin. The follower
   adds at most 1.5 cm by design, and PX4's real tracking error comes on top. Check the first
   flights' logs against the route before going faster.
5. **The tunnel:** with images dropped, a dark tunnel that cuVSLAM doesn't report as failed is
   caught only by the gyro rule. The lidar (when allowed) keeps correcting there.

## Provenance

- **DroneDepthCamTesting `course-planner-lidar` @ `14c28ca`.** 60 files were ported byte-for-byte:
  the course map and its extractor, the planner, the MS200K reader, the matcher, the SITL tools
  and the docs. Code that commands PX4 directly was left out. One change was made:
  `lidar_localize.match` returns its information matrix.
- **DroneDepthCamTesting `host-agnostic-cuvslam-setup` @ `ad28e40`:** the Guard's thresholds and
  settle time.
- **The drone snapshot** (`drone-snapshot/`): the kit's interfaces, `docs/kit-interfaces.md`. The
  launch file copies `competitor.launch.py`'s `flight_interface` block, and the topic resolver
  follows its `topics.py` rule.

## Files

| Path | Role |
|---|---|
| `../../package.xml`, `setup.py`, `setup.cfg`, `resource/` | the ROS 2 package `dronehacks_lidar` |
| `../../launch/lidar_nav.launch.py`, `lidar_nav_{allowed,forbidden}[_gate_b].launch.py` | flight interface + the four nodes; the per-run variants |
| `../../routes/course_v3_{window,gate_b}.json` | the routes, planned from the STEP map through the checkpoints |
| `../../deploy/install_on_drone.sh` | install on drone 1 (refuses any other host) |
| `../../dronehacks_lidar/*_node.py`, `px4_topics.py`, `launch_variants.py` | the ROS 2 nodes and helpers |
| `../../follower/route.py`, `make_route.py`, `mission.py` | the route format, the route maker, the mission (ROS-free) |
| `../../pose_fusion/` | Guard, command correction, drift filter, localiser, map facts, frames, policy (ROS-free) |
| `../../course/` | ported: map, extractor, planner, matcher, lidar reader, docs |
| `config/pose_fusion.example.yaml` | parameters for all four nodes (installed with the package) |
| `docs/deploy.md`, `docs/kit-interfaces.md` | deploying on drone 1; the organisers' interfaces |
| `../../tests/` | `test_follower.py` (routes, closed loop, mission), `test_vo_guard.py`, `test_setpoint_correction.py`, `test_pose_fusion.py`, `test_lidar_localizer.py`; `test_ros_nodes.py` (needs ROS) |
