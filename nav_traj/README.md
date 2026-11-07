# nav_traj: the no-lidar flight stack, flown by the trajectory Follower

ROS 2 package `dronehacks_traj`. It plans from the course map, then flies the
whole mission (take off, follow the route, hold over the pad, land) using the
trajectory Follower in `planner/` (package `dronehacks_planner`). It uses no
lidar: PX4 flies on the kit's cuVSLAM through the organisers' bridge.

**Status: not flown, and the ROS layer has not run.** There was no ROS 2 here
to run it, so `mission_node.py` and the launch files are untested beyond
static checks. The ROS-free mission has flown, arming to disarm, against a
stand-in PX4. It is one of three variants:

| Variant | Branch, package | Pose | Follower in FOLLOW |
|---|---|---|---|
| Lidar | `core/port-depth-camera-testing`, `dronehacks_lidar` | Guard + lidar drift correction (`pose_fusion`) | carrot on the planner's polyline |
| No lidar | `core/vo-nav-no-lidar`, `dronehacks_vo` | Guard | carrot on the polyline |
| **No lidar, trajectory Follower (this one)** | `core/vo-nav-traj-follower`, `dronehacks_traj` | Guard (`dronehacks_vo`'s), or raw cuVSLAM | `dronehacks_planner` Follower: smoothed path, speed profile, lead poses |

This variant and `dronehacks_vo` run the same Guard; only the follower
differs. In the planner benchmark, which flies both followers in the same
simulator (planner/README.md, "Two followers"), this Follower flew the course
in 23.5 s against the carrot's 54.8 s. Both had no wall touches.

```
cuVSLAM /visual_slam/tracking/odometry ──► vo_guard (dronehacks_vo) ──► /vo_guard/odometry ──┐
/skybeta/state (JSON) ──────────────────────────────────────────────────────────────────────┤
/fmu/out/vehicle_land_detected, /fmu/out/vehicle_attitude (read only) ──────────────────────┤
                                                                                             ▼
                                                      mission ──► /skybeta/setpoint/{pose,twist}
                                                                   ──► the kit's flight_interface ──► PX4
```

## The mission

| State | Sends | Leaves when |
|---|---|---|
| WAIT | nothing until the prechecks pass, then the takeoff pose (here, at flight height) | Offboard starts |
| TAKEOFF | the takeoff pose | within 0.10 m of flight height |
| FOLLOW | the Follower's led poses (course frame converted to `odom`) | the Follower is done over the pad |
| HOLD | the pad pose | 2 s within 0.10 m of it |
| LAND | twist: down 0.3 m/s, sideways toward the pad | PX4 reports landed |
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

Otherwise it sends nothing, Offboard is refused, and PX4 disarms after
`COM_DISARM_PRFLT` (10 s). Kept from the lidar stream's mission: it never
re-requests Offboard, never arms, disarms or changes mode, and stops for good
when Offboard ends. With a stale pose in the air it sends nothing, so the
interface holds the last pose. That pose is a led pose the Follower has
checked can be reached in a straight line.

Frames (`frames.py`): `odom` is cuVSLAM's frame, which is the contract's
tracker frame (`contracts/frames.md`). Its +x is the map's launch heading (north
on the published course), +y is to the left, and +z is up from the body at
rest. A course point maps to `x = Δnorth, y = −Δeast` when the launch heading
is north. That matches the contract's worked example, which
`test_contract_worked_example` checks. Setpoint height is
`slice_height − rest_height_m`.

## Dry run (offline, the whole mission)

```sh
cd nav_traj
PYTHONPATH="../planner:." python -m dronehacks_traj.dry_run ../planner/test/fixtures/published_v3_fieldguide.yaml --rest-height 0.07
# on the drone, after the build:
ros2 run dronehacks_traj dry_run /root/team_ws/course_map.yaml --rest-height 0.07 --v-max <MPC_XY_VEL_MAX>
```

On the provisional map at 1.0 m/s (the stand-in PX4, not a flight):

| Event | Time |
|---|---|
| Offboard | 2.0 s |
| Lift-off | 2.3 s |
| Touchdown | 32.6 s |
| PX4 disarms | 34.6 s |

The run had 0 wall touches and came no closer than 0.35 m to a wall. It went
through both openings whole-drone and landed on the pad. It passed through
every state from WAIT to STOPPED. Planning on start-up took 1.1 s on a laptop.

## Tests

```sh
cd nav_traj && python -m unittest discover -s test     # or: colcon test --packages-select dronehacks_traj
```

The 26 tests cover:
- frames, including the contract's worked example;
- every precheck, GROUNDED, HELD, and a stale pose in the air;
- whole flights in the stand-in: nominal; a failed heading gate (Offboard
  refused, preflight disarm); a pilot takeover; an odometry dropout; the
  published map;
- the parameter file against the node's declarations and types;
- that the launch files start no lidar node and the kit's interface unchanged.

The ROS message conversions run only where `rclpy` exists (skipped here).

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
| `launch/traj_nav_competitor.launch.py`, `..._raw.launch.py` | for `COMPETITOR_LAUNCH`: with and without the Guard |
| `config/traj_nav.example.yaml` | the mission node's parameters |
| `docs/deploy.md` | putting it on drone 1 |

## Not done

* **The ROS layer has not run**: the node, the launch files, `colcon build`
  and `colcon test`. Do it on the drone or a ROS 2 Humble machine
  (docs/deploy.md) before any flight.
* **The map** is the planner's provisional fixture. The surveyed map must
  replace it (planner/README.md).
* **The Guard** comes from `dronehacks_vo`, which is not merged and is still
  changing on its branch. Its `vo_guard` parameters `odom_in` and
  `output_topic` are what this launch sets.
* **`v_max_mps`** must equal the drone's live `MPC_XY_VEL_MAX`: 1.0 in the
  SN0001 file, 12 in the baseline file.
* **The heading gate** compares PX4's NED heading, converted by
  `yaw_ENU = 90° − yaw_NED`, with vision's. If the team sets the interface's
  `odom_to_ned_yaw_offset_rad`, the gate must allow for it.
* **The optional return** over the barrier is not built.

Credits: the state machine is ported from the lidar stream's
`follower/mission.py` (commit 6dc1263); the Follower is `planner/`'s.
