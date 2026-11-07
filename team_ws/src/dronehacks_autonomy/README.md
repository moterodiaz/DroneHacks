# dronehacks_autonomy

Map-driven flight for the Sky Beta. It reads a course map, plans a route with A*, smooths it,
profiles the speed and flies it through the kit's flight interface. **A new course is a new map
file; no code changes.**

**Status: not flown.** Everything below has only run offline, against a stand-in PX4 and stand-in
ROS modules. It has never been built with colcon or run under real ROS 2. Every timing and
clearance is a model output.

## How it works

```
course map (contracts/course.md)
  -> contract_map.py        validate; add the tunnel's walls; keep a blocked window's wall solid
  -> planning/route.py      A* on a 5 cm grid with Nav2's inflation cost, goals in map order
  -> planning/smoothing.py  corner arcs, re-verified against the walls
  -> planning/speed.py      speed profile from PX4's limits, re-checked after it is built
  -> planning/follower.py   reference pose along the trajectory, with a progress governor
  -> mission.py             wait, take off, follow, hold, land; fails safe to a hold
  -> mission_node.py        /skybeta/setpoint/{pose,twist} to the kit's flight_interface
```

It subscribes to cuVSLAM odometry, `/skybeta/state` and read-only PX4 attitude and land-detected
topics. It publishes only `/skybeta/setpoint/*`. It never arms, disarms, changes mode or writes
`/fmu/in/*`.

No route logic names a course feature. The route through the landing area on the published
layout comes from the window's wall being solid, which leaves one gap for A* to find.

## Run it offline

From this directory, with numpy, PyYAML and pytest installed:

```sh
python3 -m pytest test -q
python3 -m dronehacks_autonomy.dry_run maps/course_v3_layout.yaml --rest-height 0.07
```

## Put it on drone 1

These steps are for a person: they touch the drone and end in a flight.

1. Copy this folder to `~/team_ws/src/dronehacks_autonomy` on drone 1. Run `hostname` first; it
   must print `skybeta-01`.
2. In the container: `team_build.sh --packages-select dronehacks_autonomy`.
3. Set `COMPETITOR_PKG=dronehacks_autonomy` in `/etc/skybeta/drone.env`.
4. Create `~/team_ws/autonomy_params.yaml`, with values read off this drone:

   ```yaml
   mission:
     ros__parameters:
       rest_height_m: 0.07    # tape-measure this drone; this number is the lab stand-in's
       v_max_mps: 1.0         # the live MPC_XY_VEL_MAX, from QGroundControl
   ```

5. If the course was surveyed, put that map at `~/team_ws/course_map.yaml`.
6. After every power-up, with the drone still on the launch mark facing the launch heading:
   `ros2 run dronehacks_autonomy measure_frame`, then restart the `competitor` window.

Until step 6 has run, the mission refuses to start. It publishes nothing, the kit refuses
Offboard and the pilot keeps the vehicle.

## Check with the props off

These can't be tested offline.

- **The carry test** (`contracts/frames.md`): carry the drone known distances and compare with the
  odometry. `measure_frame` trusts the placement and does not replace this.
- **The heading gate**: `ros2 topic echo /mission/status` shows why it isn't offering takeoff. If
  PX4's heading differs from vision's by a constant, the 2 degree gate will never open. Measure
  the offset (`competitor_guide.md` section 1) before changing `px4_heading_gate_deg`.
- **The setpoint cadence**: `contracts/ros-graph.md` leaves it unvalidated. Check `rate_hz`
  against the interface's timeout behaviour.

## Known limits

- **Landing is off by default** (`land: false`). The drone holds over the pad for the pilot, and
  any pilot input other than abort ends a scored run, so this can't finish one. `contracts/px4.md`
  blocks the velocity descent until the organisers confirm a procedure. Once they have, set
  `land: true` in `autonomy_params.yaml`.
- **The window is off by default.** It needs `use_window: true` and a reviewed `clearance_radius`
  in the map.
- **The map is trusted.** Nothing senses walls in flight, so a course laid differently from its
  map needs a re-surveyed map. In the field guide's simulator, 10 cm off touched on 5 of 5 runs.
- **The tunnel has no vision-loss handling beyond a hold.** A stale pose stops new setpoints, and
  the interface holds the last one. If tracking is lost PX4 leaves Offboard and it does not return.

## Credits

The planner and follower come from `planner/` and the mission from `nav_traj/` on branch
`navigation/path-planner`, both in this repo. Built on the organisers' skybeta starter project,
NVIDIA cuVSLAM and Isaac ROS, Luxonis depthai-ros, PX4, ROS 2, NumPy and PyYAML. The inflation cost
follows Nav2's formula.
