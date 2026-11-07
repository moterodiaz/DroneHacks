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
```

Run these from `sitl/`. You need Python 3.10+ with `numpy` and `pyyaml` (`requirements.txt`).
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

* **Left: setup.**
  * **Autonomy version.** Every entry in `versions.yaml`, with its git ref and the
    `COMPETITOR_PKG`/`COMPETITOR_LAUNCH` it would use.
  * **Follower algorithm.** Followers marked *sim injection* patch the mission module inside
    the simulated process only; the drone would run the shipped one. Parameter followers are
    real parameters of the version.
  * **Version options**: route, landing, return flight.
  * **PX4 parameter profile.**
  * **Scenario**:
    * seed and duration;
    * start pose and calibration error (what the operator "measured");
    * tunnel lighting, cuVSLAM drift and latency;
    * `EKF2_EV_DELAY` and other PX4 overrides;
    * wind;
    * pilot timing.
  * **Speed**: 0.5x, real time, 2x, 4x or as fast as the machine allows.
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
| `traj` | dronehacks_traj: minimum-time trajectory + Follower, no lidar (PR #26) | `origin/navigation/vo-traj-follower` |
| `lidar_allowed` / `lidar_forbidden` | dronehacks_lidar: vo_guard → pose_fusion (MS200K) → carrot follower (PR #27) | `origin/localization/lidar-nav` |
| `return_flight` | the lidar build with the rule 7.10 return leg | `localization/return-flight` (local) |
| `vo_nolidar` | dronehacks_vo: vo_guard + carrot follower | `core/vo-nav-no-lidar` (local) |
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
* **Land detector**: three-stage hysteresis.
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
4. **The dark tunnel defeats every VIO-only plan, and the lidar plans too.**
   * cuVSLAM drops out in the tube.
   * PX4 only ever sees raw cuVSLAM, so the team's lidar fusion never reaches EKF2.
   * PX4 dead-reckons for `EKF2_NOAID_TOUT` (5 s), then leaves Offboard for Altitude, and the
     low throttle brings the drone down in the tube.
   * The lidar and vo followers deliberately *hold still* while tracking is false, which
     guarantees that timeout.
   * With a lit tunnel (scenario option), the same stacks pass.
5. **`manual_full` assumes odom +x = course east.** cuVSLAM's +x is the nose, which faces north on
   the launch mark, so the drone flies west into the net and later flips.
6. **The kit's 5 m square** flies into the north net at about 2 m. In this course, that is the
   point of trying it here first.
7. **Height references differ.** PX4's z is floor-referenced (rangefinder) while odom z is
   start-referenced, so pose setpoints fly about 7 cm lower than they read in odom.

---

## Tests

```
cd sitl
py -B -m unittest tests.test_rosfake tests.test_models tests.test_step_mesh      # ~5 s
py -B -m unittest tests.test_flight -v                                            # ~1 min: real flights
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
* `test_flight`:
  * the organizer procedure on the kit example;
  * the traj stack flying to the tunnel exit.

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
