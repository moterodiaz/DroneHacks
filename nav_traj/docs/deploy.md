# Deploying `dronehacks_traj` on drone 1

This variant has no lidar, and the trajectory Follower flies it. Paths are on
the drone's host shell unless marked *(container)*. `~/team_ws` on the host is
`/root/team_ws` in the containers (competitor_guide.md §5). Nothing here
touches the organisers' PX4 parameters (rule 8.5).

## 1. Install, build and dry-run

```sh
# on the drone (host shell)
git clone https://github.com/moterodiaz/DroneHacks.git && cd DroneHacks
git checkout navigation/vo-traj-follower     # or stay on main once it has merged
nav_traj/scripts/install_on_drone.sh
# with the Guard from the lidar-free build (branch core/vo-nav-no-lidar):
GUARD_SRC=<that checkout>/depthcam nav_traj/scripts/install_on_drone.sh
```

The script does all of this:
1. Copies `planner/` to `~/team_ws/src/dronehacks_planner` and `nav_traj/` to
   `~/team_ws/src/dronehacks_traj`, plus `dronehacks_vo` if `GUARD_SRC` is set.
2. Builds them in the running `skybeta-stack` container with `team_build.sh`.
3. Checks that `numpy`, `yaml` and both packages import there. Both libraries
   ship with the image, so no team image is needed.
4. Dry-runs the whole mission on the map the competitor window will fly. It
   must end with `dry run: PASS`.

Don't copy `drone-snapshot/` into the workspace: its skybeta packages would
shadow the image's.

## 2. The map

The competitor window flies `~/team_ws/course_map.yaml` if it exists.
Otherwise it flies the map generated from the STEP model, installed with the
package (`share/dronehacks_traj/maps/course_v3_step.yaml`). The launch log says
which. The STEP map isn't surveyed, so before relying on it:

* **Tape-measure** the window's edges, the maze wall ends and Gate B against
  `nav_traj/maps/course_v3_step.yaml`. Coordinates are course `(east, north)`
  metres, origin at the floor centre. Off by more than 5 cm means touches
  (`course.md`).
* **If the course is off,** write a corrected map in the same format to
  `~/team_ws/course_map.yaml`, then rerun the script: its dry run then uses it.

## 3. Parameters

Set the measured values when you run the script, e.g.
`V_MAX=1.0 REST_HEIGHT=<measured> ODOM_YAW=<measured> ODOM_X=<measured> ODOM_Y=<measured> nav_traj/scripts/install_on_drone.sh`.
It writes them into the installed `config/traj_nav.example.yaml` and dry-runs with the same
values. A rerun without them keeps what the last run wrote. Don't edit the copy in
`~/team_ws/src`: every run replaces it. Change other parameters in the
checkout's `nav_traj/config/traj_nav.example.yaml` before running the script.

* `v_max_mps`: the live `MPC_XY_VEL_MAX`, from QGroundControl > Parameters.
  The SN0001 file says 1.0, the baseline file 12.
* `rest_height_m`: the body centre above the floor when landed. Tape-measure it
  on drone 1 (`frames.md`'s 0.07 m is the lab stand-in's, not this kit's).
* `course_to_odom_yaw`, `course_to_odom_x`, `course_to_odom_y`: the **measured**
  course->odom transform (`contracts/frames.md`): the rotation of `R_odom_course`
  in radians, then the translation `o_odom` in metres. Measure it at each
  tracking initialisation; do not assume an identity rotation from the launch
  heading. The node refuses to start and publishes nothing without them.
* `px4_heading_gate_deg`: 2.0. Leave it on unless the first bench test shows
  PX4's and vision's headings differ by a constant (competitor_guide.md §1).
  In that case, set the interface's offset first.

## 4. Launch

In `/etc/skybeta/drone.env` (sudo):

```sh
COMPETITOR_PKG=dronehacks_traj
COMPETITOR_LAUNCH=traj_nav_competitor.launch.py        # raw cuVSLAM, the default (no Guard)
# COMPETITOR_LAUNCH=traj_nav_competitor_raw.launch.py  # also raw cuVSLAM
# COMPETITOR_LAUNCH=traj_nav_competitor_window_2mps.launch.py  # the window at 2 m/s (below)
# COMPETITOR_LAUNCH=traj_nav_competitor_window_0p5mps.launch.py  # the same, slowed to 0.5 m/s
```

**The window-at-2-m/s flight line** (`traj_nav_competitor_window_2mps.launch.py`) is the
default launch plus `config/flight_line_window_2mps.yaml` over the mission's parameters: CP4
through the window, track speed, the plan's speed limit and the twist cap all 2.0 m/s, and the
kit interface's documented `max_speed_mps` launch argument 2.0 (default 1.5).
**It flies the taped map**, `maps/course_arena_2026-11-08.yaml` (the course as the team measured it
on 2026-11-08, `maps/README.md`), unless `~/team_ws/course_map.yaml` or `$DRONEHACKS_MAP` is
there. It flies at the window's taped middle, 1.23 m, and straight over the boulders
(`avoid_boulders: false`). **The taped pad is about 1 m south of the STEP's.** Before flying it:
* **Remove any older `~/team_ws/course_map.yaml`**: it would win over the taped map.
* **PX4 still caps the speed.** `contracts/px4.md` records the organisers' `MPC_XY_VEL_MAX` as
  1.0 m/s and forbids writing it. Read the live value; at 1.0 the drone flies at 1.0.
* **The window is not cleared** by `contracts/course.md` (no reviewed clearance budget). The
  planner's window pass keeps 0.137 m over its inscribed radius on the taped map.
* On the taped map no plan verifies at 2.0 m/s (along-path jerk at a turn), so the node plans
  at 1.8 m/s and logs why. The speed profile peaks near 1.4 m/s either way: the unvalidated
  cornering limit (`a_lat_mps2` 1.0) binds before the speed caps. SITL results: `sitl/README.md`.

**The slow test line** (`traj_nav_competitor_window_0p5mps.launch.py`) is the same line with
`config/flight_line_window_0p5mps.yaml`, on the same taped map. That file differs only in its speeds: all four are
0.5 m/s, below PX4's 1.0 cap, so the pilot has time to take over before a touch. The tube takes
longer at this speed. In SITL, cuVSLAM was lost for 3.1-3.2 s, within the 4 s `coast_max_s`.
If the real tube stays dark for more than 4 s, the drone stops inside it, and PX4 drops to
Altitude 5 s after the loss (`EKF2_NOAID_TOUT`). Be ready to take over at the tube.

The kit passes only its own drone.env keys into the container, so don't put
`DRONEHACKS_*` settings there; they never arrive. Restart the stack on the
ground, with the drone still on the launch mark facing north:
`docker rm -f skybeta-stack`. By hand instead, after stopping the
`competitor` window (Ctrl-C there):

```sh
ros2 launch dronehacks_traj traj_nav.launch.py                    # (container) the packaged map
ros2 launch dronehacks_traj traj_nav.launch.py map_file:=/root/team_ws/course_map.yaml guard_package:=none
```

## 5. What the log should say

```
dronehacks_traj: map .../course_v3_step.yaml
dronehacks_traj: Guard dronehacks_vo/vo_guard -> /vo_guard/odometry      (only with guard_package:=auto or a named package; the default runs no Guard)
[mission]: map ...: route tunnel > gate_b > pad, 17.70 m, planned 24.42 s at v_max 1.0 m/s ...
```

## 6. Before the first flight

1. **Props off.** Watch `ros2 topic echo /mission/status`. Its `reason` must
   reach `ready: offering the takeoff pose`. If it says why not, fix that.
2. **Carry test** (contracts/frames.md, "Acceptance check"). Carry the drone 1 m
   along the launch heading: `/visual_slam/tracking/odometry` x must grow.
   Then check, props off, that a pose setpoint moves `/fmu/in/trajectory_setpoint`
   the same way.
3. **Prechecks.** Turn the drone 10°, or move it 0.3 m: the status must show
   the reason, and `ros2 topic hz /skybeta/setpoint/pose` must read nothing.
   Then arm and switch to Position: the flight interface's log must say
   `staying out of Offboard: no team setpoint received`.
4. **Record a bag of every flight** (`host/run.sh bag`).
5. **First flights**: tethered and low. Watch `/mission/status`: `lag_m`,
   `cross_track_m`, `lead_scale`. The safety pilot keeps the throttle up for
   takeover.
