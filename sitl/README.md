# DroneHacks SITL: drone 1 in software

A software-in-the-loop test environment for **our drone (drone 1, `skybeta-01`)** flying the
competition course. It runs the team's autonomy **unmodified**, together with the organizers' kit
code and parameters that are preloaded on the drone, against models of everything we cannot run on
a laptop: PX4 v1.16, the airframe, cuVSLAM, the rangefinder, the MS200K lidar and the safety pilot.
The world is the STEP model, `dronehacks-course-v3.step`.

```
py -m dronesim gui              # the graphical test environment (or double-click run_gui.cmd)
py -m dronesim list             # autonomy versions, followers, options, PX4 parameter profiles
py -m dronesim run --version traj --follower path_lead --option route=window --out runs/x
py -m dronesim run --version lidar_forbidden --option dropout=hold --speed-cap 1.5 --out runs/y
py -m dronesim run --version vo_nolidar --track-speed 0.6 --descent-speed 0.3 --out runs/z
py -m dronesim run --version traj --option route=window --option rocks=fly_over --out runs/w
```

Run these from `sitl/`. Several `run` commands may run at once: each simulation builds its own
tree (removed when it ends), and the shared caches are written atomically. You need Python 3.10+ with `numpy` and `pyyaml` (`requirements.txt`).
Tkinter comes with the standard Windows installer. No ROS, Docker, WSL or GPU is needed. The first
start builds the course caches from the STEP file, which takes about 75 s once (the 2026-11-07 export
has about a quarter more triangles than the earlier one).

---

## What is real and what is modelled

| Layer | In the SITL | Source |
|---|---|---|
| Team autonomy | **Real code, unmodified**: the version's packages, archived from a git ref with `git archive`, built like `colcon`, launched with `ros2 launch $COMPETITOR_PKG $COMPETITOR_LAUNCH` and no arguments, as `run_competitor.sh` does | `versions.yaml` |
| Kit `skybeta_flight_interface`, `px4_odom_bridge`, `competitor_example` and their launch files | **Real code, unmodified** | `drone-snapshot/home/dronehacks/skybeta/src` |
| ROS 2 Humble: rclpy, launch, launch_ros, ament_index, tf2_ros, DDS | **Emulated** (`dronesim/rosfake`). Message classes are generated from the real `.msg` files, with rosidl's type assertions. See the details below | `rosfake/interfaces/SOURCES.txt` |
| PX4 v1.16 | **Modelled.** The control libraries are ported: PositionControl, Takeoff, AttitudeControl, RateControl, ControlAllocation. Commander, the failsafes, the land detector, the flight modes and EKF2 are behavioural models, all driven by the organizers' parameter files | `px4/params` in the snapshot, `contracts/px4.md` |
| uXRCE-DDS client and agent | Modelled: `/fmu/out/*` at the `dds_topics.yaml` rates and QoS, `_v1` names, time sync, UART delay | |
| cuVSLAM (stereo VO) | Modelled. 30 Hz with measured latency, startup-relative odom frame, drift, tracking from what the camera sees in the mesh (the dark tunnel), pose jumps on re-acquisition | organizer measurements in the kit README / CLAUDE.md |
| VL53L1X rangefinder | Modelled: one beam raycast against the mesh, 5 Hz, 0–4 m | `03a_height_reference.params` |
| MS200K lidar | **Byte-exact serial stream** on a virtual `/dev/ms200`: 47-byte frames, CRC-8, and each point raycast at its own time. The team's own driver parses it | the team's `course/ms200.py` |
| Airframe | 6-DOF rigid body. Rotor geometry comes from `CA_ROTOR*`; motors, drag, ground effect, battery and the landing gear are modelled, and props and body collide with the mesh | `config` defaults in `vehicle/physics.py` |
| Course | **The STEP file**: 31 solids tessellated from the B-rep, with names, so walls, the window, the tube, domed boulders, nets and the pad are all real geometry (the 2026-11-07 layout, below) | `dronehacks-course-v3.step` |
| Safety pilot | The organizers' procedure, scripted: arm on Altitude, then Position 2 s later, then hands off. With the return enabled, it re-arms on the pad 3 s after the disarm the same way. Every RC channel can also be operated from the GUI | kit README "How your drone is tested" |
| Referee | Rulebook §4 against ground truth: whole-drone checkpoints in order, +5 s per touch (the body, the props or the landing gear), the over-wall rule, end of run, window bonus, the optional return (rule 7.10) | `course/course_v3_rules.json` (team ground truth, blob `3675a35`, updated here for the 2026-11-07 layout: blob `827e75b`) |

### The course (2026-11-07 layout)

`dronehacks-course-v3.step` is now the team's SolidWorks 2023 export (SHA-256 `2194664e…b876`,
kept byte-exact by `.gitattributes`). It replaced the 2026-11-06 Onshape export
(`b0e52800…4500`). What changed:

* **Removed** (hidden in the assembly, so not exported): the maze dividers M1, M2 and M3 and the
  net roof. The net's side panels are still 3.048 m tall, and the top of the net box is the
  course's height limit: climbing above it ends the run as leaving the course.
* **Added**: two walls, the two bodies of one part (`Part67 / Boss-Extrude1[1]` and `[2]`), 68 in
  tall: a 1.8 × 0.35 m block against the west and north nets, and a 0.10 m diagonal wall at
  42.9° from east whose south end meets Barrier 1's north face. The narrowest new gap is about
  1.0 m, between the two.
* **Checkpoints**: CP1 (end of the first corridor) is gone. CP2–CP5 keep their ids, names, lines
  and points. CP2 Gate A is now an explicit segment where it was (M3's centreline from its old
  north end to the north net), because its southern wall is gone.
* Every other part is unchanged (bounding boxes within 0.1 mm).

How the SITL reads it: instance names come from the PRODUCT, minus SolidWorks' `.step` file
extension and `^<assembly>` virtual-component suffix, so the names are the same as before; a
multi-body part's solids are `<part> / <body>`. A solid whose name gives no kind (here `Part67`)
is a wall if it stands on the floor and reaches the wall top, so touching it costs 5 s. SolidWorks
writes the boulder domes as two B-spline halves whose edges meet at the apex (Onshape used a
VERTEX_LOOP there); the tessellator now handles edge loops through a surface pole.

### One test session, as the simulation runs it

```
 0.0 s  PX4 boots: EKF2 on mag heading, rangefinder height; the stack container starts
 1.5 s  MicroXRCEAgent session up; /fmu/* appear                (agent window)
 2.0 s  px4_odom_bridge (kit launch file)                       (bridge window)
 2.5 s  cuVSLAM starts; odom = base_link here, x = the nose     (slam window)
 4.0 s  ros2 launch $COMPETITOR_PKG $COMPETITOR_LAUNCH          (competitor window)
15.0 s  the pilot arms on Altitude, throttle low
17.0 s  the pilot switches to Position; flight_interface asks for Offboard (auto_offboard.py rules)
```

The environment the competitor window sees is the `drone.env` whitelist (`ROS_DOMAIN_ID=1` and so
on), so `DRONEHACKS_*` variables do not arrive, just as on the drone.

---

## The GUI

* **Left: setup.** The column scrolls (mouse wheel or its scroll bar); the speed, **Start**,
  Pause, Stop and Save stay pinned at its bottom. The wheel never changes a dropdown's value.
  * **Autonomy version.** Every entry in `versions.yaml`, with its git ref and the
    `COMPETITOR_PKG`/`COMPETITOR_LAUNCH` it would use.
  * **Follower algorithm.** Followers marked *sim injection* patch the mission module inside
    the simulated process only; the drone would run the shipped one. Parameter followers are
    real parameters of the version.
  * **Version options**: route, landing, return flight, what to do on a tracking dropout (fly
    through by dead reckoning, the shipped behaviour, or hold still, the old one), and **Rocks**
    (plan around the boulders, or fly over them; the carrot builds have it for the window route).
  * **PX4 parameter profile.**
  * **Scenario**:
    * the **speed cap**, PX4's `MPC_XY_VEL_MAX`. It starts at the selected profile's value (1.0 m/s
      on `drone1-fleet`) and follows the profile until you type your own. A version's own speed
      limit follows it too (`dronehacks_traj`'s `v_max_mps`, through `$v_max`). The kit's
      flight interface still caps twist setpoints at its `max_speed_mps` (1.5 m/s);
    * the **track speed**: how fast the autonomy itself flies the route, separate from PX4's cap.
      Blank keeps the version's own behaviour. traj (`track_speed_mps`): the trajectory is planned
      at that speed and FOLLOW flies it by velocity instead of reference poses. The carrot builds
      (`track_speed_mps`): the carrot's lookahead is regulated to hold that ground speed. Corners
      still slow both, and PX4's cap still applies;
    * the **landing descent** (`descent_mps`; blank: the version's 0.3 m/s). Below 0.275 m/s PX4's
      land detector can't use "commanded down, not moving" (1.1 x `LNDMC_Z_VEL_MAX`), so it waits
      for low throttle and disarms far later (33 s at 0.2 m/s, against 3 s at 0.3);
    * seed and duration;
    * start pose and calibration error (what the operator "measured");
    * tunnel lighting;
    * cuVSLAM drift, as a multiplier. At 1 the median error is 2–7 cm (worst 0.1–0.2 m),
      depending on the seed; from 3 up it starts costing wall touches (finding 11);
    * cuVSLAM latency. It changes little until about 0.18 s, then PX4 drops every vision sample
      as too old (finding 11). The run notes warn when it is set that high;
    * `EKF2_EV_DELAY` and other PX4 overrides;
    * wind;
    * pilot timing.
  * **Speed** (in the pinned footer): 0.5x, real time, 2x, 4x or as fast as the machine allows.
* **Centre: map.**
  * The STEP geometry and the checkpoint gates.
  * The true drone (props green when armed), where EKF2 thinks it is, where cuVSLAM says it is,
    the active setpoint, lidar returns and the flown trail.
  * Below the map: height (true, EKF2, cuVSLAM, setpoint) and speed / estimate-error charts.
* **Right: status.**
  * PX4 nav state, user-intended mode, failsafe, takeoff state, EKF2 validity and dead
    reckoning, vision fusion, height source, cuVSLAM tracking and feature score, setpoint age,
    battery, lidar.
  * Vision fusion shows what EKF2 did with each cuVSLAM sample so far: fused, rejected by the
    gate, too old, or a reset.
  * The referee's checkpoint table, showing both the centre crossing and the official
    whole-drone crossing.
  * **Safety pilot** controls: arm switch, mode switch Manual/Altitude/Position, throttle, take
    over, kill switch, RC link.
  * **Faults**: cuVSLAM lost, lidar off, gust, kill a node's process.
* **Bottom tabs.**
  * **Logs**: every node, in ROS log format.
  * **Referee events.**
  * **Topics**: a live graph with publisher and subscriber counts and rates; select one to echo.
  * **PX4 messages**: the MAVLink console view.
  * **Run info**: provenance, the config written into the installed package, the values used,
    and the PX4 parameters with the file each came from.
* **The result** (when the run ends or you press Stop): the referee's score, the return's
  outcome if one was flown, and cuVSLAM's error against the truth (median and maximum), its
  tracking losses and PX4's vision-sample counts.
* **Stop** ends the flight but keeps the run, so **Save run** still works. Save writes
  `result.json`, `log.txt` and `trajectory.csv`. Start a new run, or close the window, to
  discard it.

---

## Autonomy versions and followers

| id | what | source |
|---|---|---|
| `traj` | dronehacks_traj: minimum-time trajectory + Follower, no lidar (PR #26 + the dropout coast) | `navigation/vo-traj-follower` (local, ahead of PR #26) |
| `lidar_allowed` / `lidar_forbidden` | dronehacks_lidar: vo_guard → pose_fusion (MS200K) → carrot follower (PR #27 + the dropout coast) | `localization/lidar-nav` (local, ahead of PR #27) |
| `return_flight` | the lidar build with the rule 7.10 return leg and the dropout coast | `localization/return-flight` (local) |
| `vo_nolidar` | dronehacks_vo: vo_guard + carrot follower, with the dropout coast | `core/vo-nav-no-lidar` (local) |
| `manual_full` | hard-coded full-course waypoints | `origin/core/manual-waypoint-nav` |
| `manual_square` | dronehacks_nav square mission | `origin/core/manual-waypoint-nav` |
| `kit_square` | the organizers' example: hover, then a 5 m square | drone snapshot |

**Followers.**

* `traj`:
  * `plain`: reference poses, the shipped default;
  * `formula_lead` and `path_lead`: the planner's lead modes, as sim injections;
  * `waypoints`: the bench's kit-style WaypointFollower over the route waypoints, as a sim
    injection.
* The lidar family:
  * the shipped carrot (0.5 m lookahead);
  * a 1.0 m lookahead (a real parameter).

**Rocks** (`traj`, `lidar_allowed`, `lidar_forbidden`, `vo_nolidar`): `avoid` is the shipped
planning; `fly_over` drops the boulders (0.53 m tall, under the 0.8 m flight band). traj plans
with `avoid_boulders: false`, on any route. The carrot builds fly
`routes/course_v3_window_over_rocks.json` (`make_route --over-rocks`) through the
`*_over_rocks.launch.py` variants, so only with the window route.

**Track speed and landing descent** map, per version, to the node parameter in the `tunables`
section of `versions.yaml`; versions without one (the manual and kit missions) ignore them.

**Tracking dropout** (an option of `traj`, the lidar family and `vo_nolidar`): `coast`, the shipped
behaviour, flies on along the route by dead reckoning on PX4's velocity estimate, then re-anchors
the frame when cuVSLAM resumes; `hold` sets `coast_max_s` to 0, the behaviour before the fix (hold
still, which in the dark tube ends in PX4's position-loss failsafe).

**Where the code comes from.** A version's `git` ref is used as named if this repository has it
(a worktree's local branch), else as `origin/<ref>` (a fresh clone). As of 2026-11-07:
`navigation/vo-traj-follower` is on origin with the coast, velocity follow and 2 m/s fixes;
`localization/lidar-nav` on origin is PR #27's merged head (`be7262c`), without the coast,
track-speed and over-rocks commits, which are local, on `141f49e`; `core/vo-nav-no-lidar` and
`localization/return-flight` are local only. A version
whose ref can't be found says so when you start it.

**To add a version**, add an entry to `versions.yaml`:

* a git ref or a directory;
* the package directories;
* the launch file;
* any installed config the operator would edit;
* options and followers.

The header of that file explains each field. The measured values (`$rest_height_m`,
`$traj_yaw`, `$odom_yaw_deg`, ...) are computed from the true start pose plus the scenario's
calibration error.

---

## PX4 parameter profiles

| profile | what |
|---|---|
| `drone1-fleet` (default) | The fleet guide's load order (`00_baseline`, 02, 03a, 03b, 04–09) plus drone 1's own `SN0001-fixes`, with the organisers' failsafe values from `contracts/px4.md` on top (`MPC_XY_VEL_MAX` 1.0, `NAV_RCL_ACT` 6, `COM_OF_LOSS_T` 1 s, `COM_FAIL_ACT_T` 5 s ...) |
| `drone1-fleet-files-only` | The same without the contract overlay (`MPC_XY_VEL_MAX` 12, `NAV_RCL_ACT` 2) |
| `drone1-as-dumped` | Drone 1's 2026-10-15 dump, exactly as recorded |
| `drone1-dump-plus-fixes` | The dump, its fixes file and the staged files |
| `drone1-team-allowlist` | `drone1-fleet` plus the only values `contracts/px4.md` lets the team write (`EKF2_EVA_NOISE` 0.01, `EKF2_EV_DELAY` 0) |
| `drone16-vio` | The organizers' VIO reference airframe |

The GUI's Run info tab shows, for the parameters that matter, which file each value came from.

---

## Fidelity notes

**ROS 2.** These Humble behaviours are reproduced because team code can depend on them:

* **QoS matching.** A RELIABLE reader never hears a BEST_EFFORT writer; TRANSIENT_LOCAL is
  replayed to late joiners. Incompatible pairs log rclpy's "incompatible QoS" warning.
* **Discovery delay** (0.25 s).
* **Message copies.** Every reader gets its own copy.
* **Static parameter typing.** An int override for a float raises `InvalidParameterTypeException`,
  and `[]` infers as BYTE_ARRAY.
* **Time.** Mixing clock types raises.
* **launch_ros** YAML-coerces string parameters, so `"3.0"` becomes a float.
* **Callbacks** run only while an executor spins, and an exception escapes `spin()`.
* **Processes.**
  * Each launched node runs its real `main()` on a thread, on simulated time.
  * `time.sleep`, `time.monotonic` and `time.time`, threads the node starts and pyserial reads
    are all simulated, so runs are deterministic for a given seed.
  * Process exit codes and launch's `process has died` lines are reproduced.

The kit's own `skybeta_flight_interface` and `px4_odom_bridge` unit tests (149) pass on this
runtime.

**PX4.**

* **Takeoff.** Offboard's `want_takeoff` rule, the `MPC_TKO_RAMP_T` ramp starting at −g/`MPC_Z_VEL_P_ACC`,
  and `MPC_TILTMAX_LND` until flight.
* **Commander.**
  * The arm switch acts on its edges only, and the mode switch only when it moves.
  * Stick override per `COM_RC_OVERRIDE`.
  * Offboard needs `offboard_control_mode` within `COM_OF_LOSS_T`, and a valid position for
    position control.
  * Offboard loss applies `COM_OBL_RC_ACT` and changes the *user's* mode, so Offboard never
    returns.
  * Position loss falls back to Altitude (`COM_POSCTL_NAVL` 0).
  * RC loss applies `NAV_RCL_ACT`, after the `COM_FAIL_ACT_T` hold.
  * Geofence (`GF_MAX_VER_DIST` 3 m on drone 1).
* **Auto disarm**: `COM_DISARM_PRFLT` and `COM_DISARM_LAND`, and the kill switch with
  `COM_KILL_DISARM`.
* **Land detector**: three-stage hysteresis. While flying, ground contact makes `mc_pos_control`
  cut the thrust (its `flying_but_ground_contact` branch: empty setpoint, high downward
  acceleration, integrators reset), so it reaches landed within `LNDMC_TRIG_TIME` of
  touchdown, and `COM_DISARM_LAND` disarms 2 s later.
* **Pilot modes**: low throttle descends at `MPC_LAND_SPEED` near the ground.
* **Acro** is refused and flagged (AGENTS.md §3.6).
* **EKF2.**
  * Delayed fusion of vision at `timestamp_sample − EKF2_EV_DELAY`, the 5σ gate, resets on the
    bridge's `reset_counter`, and the heading reset to vision yaw. This is why PX4's local frame
    is cuVSLAM's frame.
  * Accelerometer-bias states, and dead reckoning with `xy_valid` false after
    `EKF2_NOAID_TOUT`.
  * Rangefinder height in conditional mode, with the consistency gate and a baro fallback;
    `EKF2_MIN_RNG` on the ground. The local z origin is the floor.
  * When the rangefinder has given no valid reading for 0.5 s (out of range above 4 m, or
    blocked), the barometer becomes the height reference until range is good again.

**Airframe and sensors.**

* Wind drag and GUI gusts are forces on the body, so the IMU's specific force includes them, as
  a real accelerometer's does.
* After a lidar outage (the GUI fault) the MS200 resumes with fresh frames; nothing from the
  outage is replayed.

**Referee.**

* A touch is any contact of the body, a prop or the landing gear with a wall, the tube, its
  supports or a boulder. Contacts with one object within 0.5 s count as one touch.
* **Leaving the course** is leaving the nets' bounding box (5 cm margin). The 2026-11-07 export
  has no roof, so nothing physically stops a climb; above the panels' top (3.048 m) the run
  ends.
* The window bonus (5 points) is for any finished window-route run, even one with a checkpoint
  taken over a wall.
* **The optional return (rule 7.10).** After the outbound pad disarm, if the version's return
  option is on, the scripted pilot re-arms on the pad. The arm switch goes off, then on 0.5 s
  later (on Altitude, throttle low), and Position follows as at the start. The run ends when
  the return does. It succeeds if the drone disarms within 2 ft (0.61 m) of the launch mark's
  centre inside the same 3-minute limit; run points are then multiplied by 1.25. A crash,
  leaving the course, the time limit or a disarm further away fails it, and that loses only the
  bonus. Walls may be overflown on the way back.

**cuVSLAM.**

* Odometry is stamped with the capture time and published 62 ± 10 ms later, in order.
* The odom frame is base_link at startup.
* Drift: scale error, heading drift that grows with distance, and random walk (z worst).
* Tracking comes from 35 camera rays against the mesh.
  * The inside of the tube is dark. With the `dark` setting, openings seen from inside are
    blown out.
  * The net is see-through to the lit hall.
  * Fast rotation blurs.
* After losing tracking, the pose resumes where it froze.

**Assumptions to verify on the drone**, all exposed as scenario or config values:

* airframe mass 1.35 kg, inertia, motor curve, time constants and drag;
* IMU rest height 0.07 m;
* how dark the tube really is;
* the cuVSLAM drift magnitudes;
* that course north is magnetic north.

---

## What the SITL has shown so far

Each of these comes from running the real code here. They are worth checking on the drone.

Findings 1–12 were measured on the **2026-11-06 layout** (Onshape export: maze dividers M1–M3, a
net roof, five checkpoints). The 2026-11-07 layout changes the course north of Barrier 1, so
route results there need re-running once the autonomy branches have new routes; findings that
no longer apply are marked *superseded* rather than removed.

1. **Drone 1's 2026-10-15 parameter dump has `COM_FLTMODE6 = 0`.** The switch position the pilot
   calls "Position" is Manual there, so the arm-then-Position procedure never starts Offboard. The
   fleet baseline and drone 16 have 2. Check the live value before a test.
2. **`dronehacks_traj` with its shipped follower flies at about 0.3 m/s**, not the planned
   1.0 m/s. The lag governor holds the reference back because PX4's position loop lags by
   v/`MPC_XY_P`. This matches the planner bench's "plain poses" result. With `path_lead` it takes
   about half the time.
3. *Superseded by the 2026-11-07 layout: CP1 is removed.* **CP1 is easy to miss.** The rules
   file's CP1 line was at north 2.584 m. The traj route turns
   east with its centre just past it but its tail short of it, so the *whole drone* never crosses.
   Every later checkpoint then does not count (rule 7.6). The lidar routes go far enough north.
4. **The dark tunnel defeated every plan, the lidar ones too; flying through fixes it.**
   * cuVSLAM drops out in the tube.
   * PX4 only ever sees raw cuVSLAM, so the team's lidar fusion never reaches EKF2.
   * PX4 dead-reckons for `EKF2_NOAID_TOUT` (5 s), then leaves Offboard for Altitude, and the
     low throttle brings the drone down in the tube (the organizers' guide warns of exactly
     this). EKF2's estimate is still within about 0.2 m when it gives up.
   * The followers *held still* while tracking was false (traj stopped on its last pose), which
     guaranteed that timeout.
   * **Fix (2026-11-07, all four autonomy branches, local commits):** in FOLLOW the follower now
     coasts on along the route by velocity, dead-reckoning on PX4's own velocity estimate, for up
     to `coast_max_s` (4 s). cuVSLAM then resumes 1.3–3.3 s after the drop, inside the 5 s.
     cuVSLAM resumes from where it froze, so its frame has jumped back by the coast. Without
     the lidar, the follower re-anchors its frame on the dead reckoning. With the lidar,
     pose_fusion re-learns the jump.
   * Dark-tunnel results with the fix (seed 1): no position-loss failsafe in any version.
     * `vo_nolidar` finishes all five checkpoints with no touches, in 105.3 s (104.9 s lit).
     * `lidar_allowed`, `lidar_forbidden` and `return_flight` reach CP4 with no touches, then
       hold over the pad (landing is off by default).
     * traj gets through the tube but still scores nothing, because of finding 3 (superseded:
       CP1 is gone in the 2026-11-07 layout).
     * After the tube, the path stays within 1 cm (`lidar_forbidden`, median) to 7 cm
       (`vo_nolidar`, median; 12.5 cm at most) of the same version's lit-tunnel path.
     * The `dropout: hold` option reproduces the old failure.
   * A lit tunnel (scenario option) never drops out.
5. **`manual_full` assumes odom +x = course east.** cuVSLAM's +x is the nose, which faces north on
   the launch mark, so the drone flies west into the net and later flips.
6. **The kit's 5 m square** flies into the north net at about 2 m. In this course, that is the
   point of trying it here first. In the 2026-11-07 layout it touches the new
   `Part67 / Boss-Extrude1[1]` block first (seed 1, 30.0 s).
7. **Landing.** The followers descend at a constant `descent_mps` (0.3 m/s, PX4's own crawl
   speed), about 2.3 s from the cruise height to the pad. The SITL had kept the drone armed on
   the pad for 21 s after touchdown, because its PX4 model lacked the ground-contact thrust cut.
   With the cut it disarms 3.0 s after touchdown. A descent below 0.275 m/s takes far longer to
   disarm on real PX4 too (33 s at 0.2 m/s).
8. **Track speed (2026-11-07).** The carrot followers flew slowly, median about 0.2 m/s: any
   cross-track offset above `max_cut_m` (1.5 cm) collapsed the carrot to the next path point.
   With `track_speed_mps` set, the drone's own offset is allowed on top of `max_cut_m`. In the
   lit tunnel, `vo_nolidar` then finishes in 74 s at 0.3 m/s, 48 s at 0.6 m/s and 47 s at
   0.9 m/s, against 87 s by default, with no touches. `lidar_allowed` at 0.6 m/s reaches CP4 at
   36 s instead of 71 s.

   traj's reference poses are held near 0.3 m/s by its governor: PX4 trails a moving pose by
   v/`MPC_XY_P`. Only its dropout coast, which flies the reference velocity, was faster, so a
   higher plan speed sped up the tunnel alone. Relaxing the governor just let PX4 cut corners
   by that lag until the cross-track abort stopped the drone. With a track speed, FOLLOW now
   flies the trajectory by twist: the reference velocity 0.55 s ahead (PX4's velocity-loop
   lag), plus a pull onto the reference. Twice a second it also sends a hold pose under the
   drone, so stopped twists fall back to holding there. Window route, dark tunnel, takeoff to
   pad: 59 s by default, 42 s at 0.5 m/s, 32 s at 0.8 m/s and 30 s at 1.0 m/s, with no touches
   (path lead at 0.8 m/s: 35 s).
9. **Flying over the rocks.** The carrot planner's window route never needed to cross a boulder:
   the over-rocks route is within 4 cm of it. traj's window route is clearance-weighted, and
   without the boulders it flies straight over Boulder 3, reaching the pad 1.4 s sooner with no
   touches. Over the rock PX4's rangefinder height dipped from 0.73 to 0.60 m. Its range
   consistency check held the true height at 0.71–0.72 m, so the drone did not climb toward the
   window's 1.22 m top edge.
10. **Height references differ.** PX4's z is floor-referenced (rangefinder) while odom z is
   start-referenced, so pose setpoints fly about 7 cm lower than they read in odom.
11. **cuVSLAM drift and latency barely matter, until latency crosses a cliff.**
   Both looked like they did nothing because, at their defaults, they don't. The runs below are
   traj, window route, lit tunnel, track speed 0.8 m/s, seeds 1–3. The error is each cuVSLAM
   pose against the truth at its capture time.
   * **Drift ×1** (default): no touches. Median error 2–7 cm, worst 0.12–0.19 m. Drift is a
     random walk, mostly in heading, so one setting gives quite different runs: a different
     random sequence at ×1 reached 0.34 m.
   * **Drift ×3 and ×5**: 1–2 wall touches per run (the C2a chicane wall, the maze dividers of
     the old layout),
     worst error 0.33–0.45 m. All still finish.
   * **Drift ×10**: seed 1 hit the tube and crashed. Seed 2 never left TAKEOFF: cuVSLAM's height
     read 0.50 m while PX4 held 0.73 m on the rangefinder, and traj waits for the *odom* height
     to reach its takeoff height.
   * **Latency up to 0.14 s**: EKF2 still fuses almost every sample. At 0.14 s seeds 1 and 2
     touched once or twice (the follower acts on an older pose); seed 3 did not.
   * **0.16–0.17 s**: 19–53 % of samples arrive too old for EKF2; the runs still finish.
   * **0.20 s and up**: every sample is too old. EKF2 never has a position, so Offboard never
     starts and the drone stays on the ground. The cliff is `EKF2_DELAY_MAX` −
     `EKF2_EV_DELAY` + 50 ms = 0.18 s on drone 1's parameters, so the real latency (62 ms) has
     margin.

   The result now shows cuVSLAM's error and tracking losses and EKF2's vision-sample counts, and
   the run notes warn when the latency is past the cliff.
12. **The optional return works in the sim.** `return_flight` with `return: on`, lit tunnel,
   seed 1 (simulation times):
   * The outbound run finished all five checkpoints, and PX4 disarmed on the pad at 103.0 s.
   * The pilot re-armed at 106.5 s; the drone took off again at 110.9 s and flew back.
   * It disarmed 0.005 m from the launch mark's centre at 136.1 s: 116.9 s after the clock
     started, inside the 3 minutes. Run points x 1.25.
   * With `return: off` the drone stays on the pad.
   * *Not reproduced on the 2026-11-07 layout* until the lidar build's own map is updated:
     `return_flight` (at `959e234`) arms but never takes off, because `pose_fusion` never reports
     ready against the old `course_v3_map.json` it ships (the same build takes off on the old
     STEP model).

---

## Tests

```
cd sitl
py -B -m unittest tests.test_rosfake tests.test_models tests.test_step_mesh tests.test_course_layout tests.test_cache tests.test_catalog tests.test_review_fixes   # ~1 min
py -B -m unittest tests.test_flight -v                                            # ~5 min: real flights
```

**What they cover.**

* `test_step_mesh`:
  * the course model against reference numbers from two independent extractions, and the
    two new walls;
  * watertightness of all 31 solids;
  * the window opening and the tube;
  * instance names from products (SolidWorks' `.step` extension and `^<assembly>` suffix, the
    `<part> / <body>` names of a multi-body part), on synthetic assemblies and the course;
  * edge loops through a surface pole: the course boulders, a split hemisphere and a sphere of
    two lunes;
  * raycast and closest-point correctness and timing, with the open roof.
* `test_course_layout`: the geometry wall fallback (synthetic and the two new walls), the net
  box as the height limit without a roof, and the rules file (no CP1, CP2 a segment, every name
  resolving).
* `test_rosfake`:
  * rosidl message semantics;
  * parameter typing;
  * timers and executors;
  * QoS incompatibility;
  * time-type rules;
  * simulated `sleep`, including a callback that sleeps;
  * launch coercion and errors.
* `test_models`:
  * control allocation and PositionControl;
  * the takeoff ramp;
  * parameter profiles (including the `COM_FLTMODE6` finding);
  * the MS200 frame codec;
  * the referee's gates from the STEP model and its whole-drone crossing rule (on CP2).
* `test_catalog`: the rocks option's launch variants and requirements, and the tunables.
* `test_cache`:
  * workspace cleanup measures ages on the real clock (parallel runs used to delete each
    other's live build trees, because the simulated clock is a day ahead);
  * six parallel extractions of one commit, and an interrupted extraction being replaced;
  * two package selections of one commit getting their own trees, and a published tree never
    being moved aside;
  * unsafe archive members (`..`, absolute paths, links) refused when Python's tar filter is
    missing;
  * the speed cap overriding `MPC_XY_VEL_MAX`.
* `test_review_fixes`: the barometer taking over after a range loss, an external force felt by
  the IMU, the lidar resuming fresh after an outage, a landing-gear touch on a boulder, and the
  window bonus with an over-wall checkpoint.
* `test_flight`:
  * the organizer procedure on the kit example;
  * the traj stack flying to the tunnel exit;
  * traj landing on the pad by the window route, with PX4 disarming within 4 s of touchdown;
  * the optional return succeeding (skipped without the local `localization/return-flight`);
  * a stopped GUI run still saving its results.

**The kit's own tests on this runtime** need pytest. Start a Python that has pytest; at its top
call `dronesim.rosfake.activate()` and build the kit packages with
`dronesim.rosfake.packages.Workspace`, then run `pytest` on
`drone-snapshot/.../skybeta_flight_interface/test`. 149 pass.

---

## Limitations

* **PX4 is a model, not the firmware.** The control loops follow the v1.16 sources, but EKF2,
  commander and the navigator are behavioural. Use the real firmware (Gazebo or SIH) to validate
  anything this disagrees with.
* **No camera images are rendered.** cuVSLAM is a visibility and drift model. Code that consumes
  `/oak/*/image_*` gets `camera_info` only.
* **Python only.** C++ nodes and composable containers are not supported.
* **Node "processes" share one Python interpreter.** Module-level globals are shared between
  processes that import the same module.
* **Callbacks take zero simulated time.** A long planning computation freezes simulated time
  instead of delaying the stack.
* **Contacts.** The nets are rigid. Contact is penalty-based; a crash ends the run but the tumble
  is not modelled in detail. There is no roof solid (2026-11-07 export); the referee, not the
  physics, enforces the height limit.
* **Python version.** The local interpreter (3.13) differs from the Jetson (3.10).

## Code map

```
sitl/
  versions.yaml          autonomy versions, options, followers
  course/                the team's ground-truth checkpoint rules (vendored)
  dronesim/
    sim.py               the engine: timeline, physics/PX4 loop, faults, telemetry, results
    rosfake/             ROS 2 Humble on simulated time (runtime, rclpy, launch, msg generation, packages)
    px4/                 params, control (ported), estimator, commander, land detector, modes, uxrce bridge
    vehicle/             6-DOF physics, contacts
    sensors/             cuVSLAM, MS200K serial device, OAK IMU / camera TF
    world/               STEP parser + B-rep tessellation, mesh queries, the course
    scoring/referee.py   rulebook section 4 on ground truth
    autonomy/            catalog (git archive -> workspace), follower injection hooks
    gui/                 Tk app, map, worker process
  tests/
```
