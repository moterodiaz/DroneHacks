# Deploying `dronehacks_traj` on drone 1

This variant has no lidar, and the trajectory Follower flies it. Paths are on
the drone's host shell unless marked *(container)*. `~/team_ws` on the host is
`/root/team_ws` in the containers (competitor_guide.md §5). Nothing here
touches the organisers' PX4 parameters (rule 8.5).

## 1. Copy the packages into the team workspace

```sh
# on the drone (host shell), from a checkout of this repo on branch core/vo-nav-traj-follower
cp -r planner  ~/team_ws/src/dronehacks_planner
cp -r nav_traj ~/team_ws/src/dronehacks_traj
# For the Guard: the lidar-free build's package (branch core/vo-nav-no-lidar):
cp -r <that checkout>/depthcam ~/team_ws/src/dronehacks_vo
```

Copy only these directories. In particular, don't copy `drone-snapshot/`: its
skybeta packages would shadow the image's (`team_build.sh` warns about
shadowed packages). Without `dronehacks_vo`, use
`traj_nav_competitor_raw.launch.py` in step 5.

## 2. Build

```sh
docker exec -it skybeta-stack bash                       # (container)
team_build.sh --packages-select dronehacks_planner dronehacks_traj
python3 -c "import numpy, yaml; print('ok')"             # both ship with the image; no team image needed
```

The stack's `competitor` window also builds `~/team_ws` when it starts.

## 3. The map and the dry run

Put the **surveyed** course map, in contracts/course.md format, at
`~/team_ws/course_map.yaml`. The competitor launch always reads that path: the
kit passes no `DRONEHACKS_*` keys from drone.env into the container. Then, in
the container:

```sh
ros2 run dronehacks_traj dry_run /root/team_ws/course_map.yaml --rest-height 0.07 --v-max <live MPC_XY_VEL_MAX>
```

It must print `dry run: PASS`. It plans exactly as the node will, and flies the
whole mission against the stand-in PX4.

## 4. Parameters

Edit `config/traj_nav.example.yaml` in `~/team_ws/src/dronehacks_traj`, then
rebuild:

* `v_max_mps`: the live `MPC_XY_VEL_MAX`, from QGroundControl > Parameters.
  The SN0001 file says 1.0, the baseline file 12.
* `rest_height_m`: the body centre above the floor when landed. Tape-measure it.
* `px4_heading_gate_deg`: 2.0. Leave it on unless the first bench test shows
  PX4's and vision's headings differ by a constant (competitor_guide.md §1).
  In that case, set the interface's offset first.

## 5. Launch

In `/etc/skybeta/drone.env`:

```sh
COMPETITOR_PKG=dronehacks_traj
COMPETITOR_LAUNCH=traj_nav_competitor.launch.py        # or traj_nav_competitor_raw.launch.py: no Guard
```

Then restart the stack on the ground, with the drone still on the launch
mark: `docker rm -f skybeta-stack`. By hand instead, after stopping the
`competitor` window (Ctrl-C there):

```sh
ros2 launch dronehacks_traj traj_nav.launch.py map_file:=/root/team_ws/course_map.yaml   # (container)
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
