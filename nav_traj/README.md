# nav_traj: the no-lidar flight stack, flown by the trajectory Follower

ROS 2 package `dronehacks_traj`. It plans from the course map, then flies the
whole mission (take off, follow the route, hold over the pad, and land with
`land: true` -- the default holds over the pad for the pilot) using the
trajectory Follower in `planner/` (package `dronehacks_planner`). It uses no
lidar: PX4 flies on the kit's cuVSLAM through the organisers' bridge.

**Status: ready to install, not flown.**
* **The map.** It ships with one generated from the organisers' STEP model
  (`maps/course_v3_step.yaml`; see `maps/README.md`). That map isn't surveyed.
* **What has run:**
  * the whole mission, arming to disarm, against a stand-in PX4 with drone 1's
    parameters;
  * the real node and launch files, against stand-in ROS modules.
* **What hasn't run:** real ROS 2. `nav_traj/scripts/install_on_drone.sh`
  builds the packages on the drone and dry-runs there.

This is one of three variants:

| Variant | Branch, package | Pose | Follower in FOLLOW |
|---|---|---|---|
| Lidar | `core/port-depth-camera-testing`, `dronehacks_lidar` | Guard + lidar drift correction (`pose_fusion`) | carrot on the planner's polyline |
| No lidar | `core/vo-nav-no-lidar`, `dronehacks_vo` | Guard | carrot on the polyline |
| No lidar, trajectory Follower (this one) | `navigation/vo-traj-follower`, `dronehacks_traj` | raw cuVSLAM (the default; the Guard is opt-in) | `dronehacks_planner` Follower: smoothed path, speed profile, reference poses |

This variant and `dronehacks_vo` can run the same Guard; only the follower
differs. In the planner benchmark, which flies both followers in the same
simulator (planner/README.md, "Two followers"), this Follower flew the course
in 23.5 s against the carrot's 54.8 s. Both had no wall touches.

```
cuVSLAM /visual_slam/tracking/odometry ──► (opt-in vo_guard, dronehacks_vo) ──► /vo_guard/odometry ──┐
/skybeta/state (JSON) ──────────────────────────────────────────────────────────────────────┤
/fmu/out/vehicle_land_detected, /fmu/out/vehicle_attitude (read only) ──────────────────────┤
                                                                                             ▼
                                                      mission ──► /skybeta/setpoint/{pose,twist}
                                                                   ──► the kit's flight_interface ──► PX4
```

## Install on drone 1

```sh
# on the drone's host shell, in a checkout of this branch
V_MAX=1.0 REST_HEIGHT=<measured> ODOM_YAW=<measured> ODOM_X=<measured> ODOM_Y=<measured> \
  nav_traj/scripts/install_on_drone.sh    # GUARD_SRC=<dronehacks_vo dir> optional
```

The script:
1. copies `planner/` and `nav_traj/` into `~/team_ws/src`, and writes `V_MAX`
   (the live `MPC_XY_VEL_MAX`), `REST_HEIGHT` and the measured course->odom
   transform (`ODOM_YAW`, `ODOM_X`, `ODOM_Y`) into the installed config. If
   one isn't given, it keeps the value the last install wrote;
2. builds them in the running `skybeta-stack` container;
3. checks the container can import them;
4. dry-runs the whole mission on the map the competitor window will fly;
5. prints the `drone.env` lines to set.

It changes nothing outside `~/team_ws/src`. The full checklist is in
`docs/deploy.md`.

## The mission

| State | Sends | Leaves when |
|---|---|---|
| WAIT | nothing until the prechecks pass, then the takeoff pose (here, at flight height); zero twists if a precheck fails after it was streamed | Offboard starts |
| TAKEOFF | the takeoff pose | within 0.10 m of flight height |
| FOLLOW | the Follower's reference poses (course frame converted to `odom`) | the Follower is done over the pad |
| HOLD | the pad pose | `land: true`: 2 s within 0.10 m of it, then LAND. Default `land: false`: it holds over the pad until the pilot takes the vehicle (px4.md) |
| LAND | twist: down 0.3 m/s, sideways toward the pad | PX4 reports landed (only reached with `land: true`) |
| LANDED | twist: down 0.09 m/s | PX4 disarms (`COM_DISARM_LAND`), which ends Offboard |
| STOPPED | nothing, for good | — |
| GROUNDED | zero twists | (Offboard started with a stale pose or failed precheck) PX4 disarms or the pilot takes over |
| HELD | a hold at the current pose | (more than 0.35 m off the path) the pilot takes over |

The prechecks gate everything. The kit grants Offboard only if a team
setpoint is under 1 s old when the pilot switches to Position
(`auto_offboard.py`). So the mission offers the takeoff pose only while all
of these hold:
- the pose is fresh;
- the drone is within 0.15 m of where the tracker started;
- its heading is within 5° of the launch heading;
- PX4's heading is within 2° of vision's (field guide, build step 2; set
  `px4_heading_gate_deg: 0.0` to turn it off).

Otherwise, before any takeoff pose was streamed, it sends nothing, Offboard
is refused, and PX4 disarms after `COM_DISARM_PRFLT` (10 s). After the
takeoff pose has been streamed, a failed precheck replaces it with a
zero-velocity twist: the kit's pose setpoint never expires and Offboard is
still granted up to 1 s after the last team setpoint, so merely stopping
would leave the drone free to climb on the stored takeoff pose. Kept from
the lidar stream's mission: it never
re-requests Offboard, never arms, disarms or changes mode, and stops for good
when Offboard ends. With a stale pose in the air it sends nothing, so the
interface holds the last pose. That pose is a led pose the Follower has
checked can be reached in a straight line.

Frames (`frames.py`): `odom` is cuVSLAM's frame, and its placement on the
course is **measured** (`contracts/frames.md`):
`p_odom = o_odom + R_odom_course * p_course`. The node takes that transform
as three required parameters -- `course_to_odom_yaw` (the rotation of
`R_odom_course`) and `course_to_odom_x`/`course_to_odom_y` (`o_odom`) -- with
no defaults: it refuses to start and publishes nothing without them, and it
never derives them from the map's launch heading (a measured identity is
passed explicitly). The prechecks still compare the drone's heading with the
map's launch heading. Setpoint height is
`slice_height − rest_height_m` (the rest height is measured too).

## The map

The launch picks the map in this order: `map_file:=`, then
`~/team_ws/course_map.yaml` if present (a surveyed map, when there is one),
then the packaged `maps/course_v3_step.yaml`. The launch log names the map
it used.

The packaged map is the **2026-11-07 layout**: `dronehacks-course-v3.step`
(SHA-256 `2194664e…`; maze dividers M1–M3 gone, a north-west block and a
diagonal wall added, checkpoint 1 removed) through the team's STEP extraction
and `dronehacks_planner.convert`. `maps/README.md` has the details;
`test_maps.py` regenerates it byte for byte. Three definitions in it:
* **CP2 Gate A** is a `gate` goal, `cp2_gate_a`: the old line from M3's north
  end to the north net, unchanged, now that no wall bounds it. The route flies
  straight through it, east, so the whole drone crosses the line.
* **Gate B** follows `contracts/course.md`: "1.13 m gap, C2b to A1", wall end
  to wall end. The lidar stream's rules file puts Gate B between C2b and
  Barrier 2 instead. The contract wins (AGENTS.md §3.1). This still needs
  resolving with an organiser. The **default route goes through Gate B**
  (planner issue #25: window goals are opt-in, `PlannerConfig.allow_windows`;
  the window route, 18.1 s planned at 1 m/s, only plans when the window is
  allowed or preferred, against Gate B's 19.7 s).
* **The tunnel** is sliced at its narrowest within the drone's 0.7–0.9 m band
  (1.409 m inside).

## Dry run (offline, the whole mission)

```sh
cd nav_traj
PYTHONPATH="../planner:." python -m dronehacks_traj.dry_run maps/course_v3_step.yaml --rest-height 0.07
# on the drone, after the build:
ros2 run dronehacks_traj dry_run <map> --rest-height 0.07 --v-max <MPC_XY_VEL_MAX>
```

On the STEP map at 1.0 m/s (the stand-in PX4, not a flight), through Gate B:

| Event | Time |
|---|---|
| Offboard | 2.0 s |
| Lift-off | 2.3 s |
| Touchdown | 63.1 s |
| PX4 disarms | 65.1 s |

The route is tunnel > gate_b > pad, 17.7 m, planned at 24.42 s (the window
route is opt-in, `allow_windows`). The flight takes longer than planned: the
Follower slows for clearance. The run had 0 wall touches and came no closer
than 0.36 m to a wall. It went through both openings whole-drone and landed
on the pad. Planning on start-up took 0.9 s on a laptop.

## Tests

```sh
cd nav_traj && python -m unittest discover -s test     # or: colcon test --packages-select dronehacks_traj
```

The 50 tests cover (49 run without ROS 2; 1 needs rclpy and is skipped
without it):
- frames (the contract's worked example, the measured transform and the
  model's) and the mission's states;
- whole flights in the stand-in: nominal; the default holding over the pad
  for the pilot (`land: false`); a failed heading gate; a pilot takeover; an
  odometry dropout; the provisional and the STEP map;
- the packaged map regenerating from its sources, and its contract
  dimensions;
- the parameter file against the node's declarations and types, and the
  decided defaults in it;
- **the real node and launch files on stand-in ROS modules**
  (`test/fake_ros.py`): wiring, prechecks, takeoff, a stale state,
  configuration errors (including the node failing closed without the
  measured course->odom transform), map fallback, and Guard selection.

One test needs real `rclpy` and is skipped without it. The stand-ins check
our code paths, not ROS itself.

## Files

| Path | What |
|---|---|
| `dronehacks_traj/mission.py` | the state machine (ROS-free) |
| `dronehacks_traj/frames.py` | course ↔ odom, quaternions, PX4 heading |
| `dronehacks_traj/factory.py` | map in, plan (`dronehacks_planner.pipeline.fastest`), mission out; flight checks |
| `dronehacks_traj/vehicle_sim.py` | the whole flight offline: kit interface, Offboard rule, stand-in PX4 (drone 1's parameters) |
| `dronehacks_traj/dry_run.py` | the dry-run command |
| `dronehacks_traj/mission_node.py` | the ROS 2 node |
| `launch/traj_nav.launch.py` | `flight_interface` (exactly as the kit's), `vo_guard`, `mission` |
| `launch/traj_nav_competitor.launch.py`, `..._raw.launch.py` | for `COMPETITOR_LAUNCH`: Guard if installed / never |
| `launch/traj_nav_competitor_window_max.launch.py` | for `COMPETITOR_LAUNCH`: the window with no speed cap of our own on the taped map `maps/course_arena_2026-11-08.yaml`, over the boulders, landing on the pad (`docs/deploy.md` section 4) |
| `config/flight_line_window_max.yaml` | that line's overrides: window, boulders ignored, rolling take-off, early descent, land, every speed at PX4's 12 m/s |
| `launch/traj_nav_competitor_window_0p5mps.launch.py` | for `COMPETITOR_LAUNCH`: the same line slowed to 0.5 m/s for test flights |
| `config/flight_line_window_0p5mps.yaml` | the max line's overrides with every speed at 0.5 m/s |
| `config/traj_nav.example.yaml` | the mission node's parameters |
| `maps/` | the STEP-derived map, its sources and provenance |
| `scripts/install_on_drone.sh` | copy, build, check, dry-run on the drone |
| `docs/deploy.md` | the full procedure for drone 1 |

## Before it flies

* **Run it on real ROS 2:** `install_on_drone.sh` on the drone, then the
  props-off checks in `docs/deploy.md` §6.
* **Read `MPC_XY_VEL_MAX` off the drone** and set `v_max_mps`. It's 1.0 in the
  SN0001 file and 12 in the baseline file; the speed profile plans to it.
* **Measure `rest_height_m` on drone 1**: tape-measure the body centre's
  height above the floor. `frames.md`'s 0.07 m is the FlyX lab stand-in's, not
  necessarily this kit's.
* **Measure the course->odom transform** at each tracking initialisation
  (`contracts/frames.md`): the node refuses to start without
  `course_to_odom_yaw`, `course_to_odom_x` and `course_to_odom_y`, and never
  assumes an identity rotation from the launch heading.
* **The map isn't surveyed.** If the course is laid off its model by more
  than about 5 cm, expect touches (`course.md`, "Layout tolerance").
  Tape-measure a few wall ends against `maps/course_v3_step.yaml`, the
  window first.
* **The Guard is off by default** (`guard_package: none`): the mission reads
  cuVSLAM directly, the contracted path (contracts/ros-graph.md). With
  `guard_package:=auto`, `dronehacks_vo`'s Guard still runs if that package
  is installed; the launch log says which.
* **The heading gate** assumes PX4's heading and vision's agree without an
  offset. If the team sets the interface's `odom_to_ned_yaw_offset_rad`,
  allow for it.
* **The optional return** over the barrier is not built.

Credits: the state machine is ported from the lidar stream's
`follower/mission.py` (commit 6dc1263); the Follower is `planner/`'s; the
map's STEP extraction is the lidar stream's (commit 324ff23).
