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
start builds the course caches from the STEP file, which takes about 40 s once.

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
| Course | **The STEP file**: 33 solids tessellated from the B-rep, with names, so walls, the window, the tube, domed boulders, nets and the pad are all real geometry | `dronehacks-course-v3.step` |
| Safety pilot | The organizers' procedure, scripted: arm on Altitude, then Position 2 s later, then hands off. Every RC channel can also be operated from the GUI | kit README "How your drone is tested" |
| Referee | Rulebook §4 against ground truth: whole-drone checkpoints in order, +5 s per touch, the over-wall rule, end of run, window bonus | `course/course_v3_rules.json` (team ground truth, blob `3675a35`) |

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
      Blank keeps the version's own behaviour. traj: its planner's `v_max_mps`. The carrot
      builds: `track_speed_mps`, which regulates the carrot's lookahead to hold that ground speed
      (corners still slow it, and PX4's cap still applies);
    * the **landing descent** (`descent_mps`; blank: the version's 0.3 m/s). Below 0.275 m/s PX4's
      land detector can't use "commanded down, not moving" (1.1 x `LNDMC_Z_VEL_MAX`), so it waits
      for low throttle and disarms far later (33 s at 0.2 m/s, against 3 s at 0.3);
    * seed and duration;
    * start pose and calibration error (what the operator "measured");
    * tunnel lighting, cuVSLAM drift and latency;
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
* **Save run** writes `result.json`, `log.txt` and `trajectory.csv`.

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

1. **Drone 1's 2026-10-15 parameter dump has `COM_FLTMODE6 = 0`.** The switch position the pilot
   calls "Position" is Manual there, so the arm-then-Position procedure never starts Offboard. The
   fleet baseline and drone 16 have 2. Check the live value before a test.
2. **`dronehacks_traj` with its shipped follower flies at about 0.3 m/s**, not the planned
   1.0 m/s. The lag governor holds the reference back because PX4's position loop lags by
   v/`MPC_XY_P`. This matches the planner bench's "plain poses" result. With `path_lead` it takes
   about half the time.
3. **CP1 is easy to miss.** The rules file's CP1 line is at north 2.584 m. The traj route turns
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
     * traj gets through the tube but still scores nothing, because of finding 3.
     * After the tube, the path stays within 1 cm (`lidar_forbidden`, median) to 7 cm
       (`vo_nolidar`, median; 12.5 cm at most) of the same version's lit-tunnel path.
     * The `dropout: hold` option reproduces the old failure.
   * A lit tunnel (scenario option) never drops out.
5. **`manual_full` assumes odom +x = course east.** cuVSLAM's +x is the nose, which faces north on
   the launch mark, so the drone flies west into the net and later flips.
6. **The kit's 5 m square** flies into the north net at about 2 m. In this course, that is the
   point of trying it here first.
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
9. **Flying over the rocks.** The carrot planner's window route never needed to cross a boulder:
   the over-rocks route is within 4 cm of it. traj's window route is clearance-weighted, and
   without the boulders it flies straight over Boulder 3, reaching the pad 1.4 s sooner with no
   touches. Over the rock PX4's rangefinder height dipped from 0.73 to 0.60 m. Its range
   consistency check held the true height at 0.71–0.72 m, so the drone did not climb toward the
   window's 1.22 m top edge.
10. **Height references differ.** PX4's z is floor-referenced (rangefinder) while odom z is
   start-referenced, so pose setpoints fly about 7 cm lower than they read in odom.

---

## Tests

```
cd sitl
py -B -m unittest tests.test_rosfake tests.test_models tests.test_step_mesh tests.test_cache tests.test_catalog   # ~9 s
py -B -m unittest tests.test_flight -v                                            # ~2 min: real flights
```

**What they cover.**

* `test_step_mesh`:
  * the course model against reference numbers from two independent extractions;
  * watertightness of all 33 solids;
  * the window opening and the tube;
  * raycast and closest-point correctness and timing.
* `test_rosfake`:
  * rosidl message semantics;
  * parameter typing;
  * timers and executors;
  * QoS incompatibility;
  * time-type rules;
  * simulated `sleep`;
  * launch coercion and errors.
* `test_models`:
  * control allocation and PositionControl;
  * the takeoff ramp;
  * parameter profiles (including the `COM_FLTMODE6` finding);
  * the MS200 frame codec;
  * the referee's gates from the STEP model and its whole-drone crossing rule.
* `test_catalog`: the rocks option's launch variants and requirements, and the tunables.
* `test_cache`:
  * workspace cleanup measures ages on the real clock (parallel runs used to delete each
    other's live build trees, because the simulated clock is a day ahead);
  * six parallel extractions of one commit, and an interrupted extraction being replaced;
  * the speed cap overriding `MPC_XY_VEL_MAX`.
* `test_flight`:
  * the organizer procedure on the kit example;
  * the traj stack flying to the tunnel exit;
  * `vo_nolidar` landing on the pad, with PX4 disarming within 4 s of touchdown.

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
  is not modelled in detail.
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
