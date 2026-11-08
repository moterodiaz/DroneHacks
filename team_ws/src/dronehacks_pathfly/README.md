# dronehacks_pathfly

This ROS 2 package flies the path from
[jsun4231/path-planning](https://github.com/jsun4231/path-planning) (see
[data/SOURCE.md](data/SOURCE.md)). It talks to the drone through the skybeta
flight interface only.

**To fly it, follow [BRINGUP.md](BRINGUP.md):** loading, arena setup for least
drift, ground checks with `pathfly_check`, staged test flights
(`stage1`-`stage3.launch.py`) and the full run.

| # | Phase | What happens |
|---|---|---|
| 1 | `TAKEOFF` | Lifts straight up to **0.2 m** over the START marker. |
| 2 | `PATH` | Follows the planned path. It climbs **0.2 → 1.25 m during the first 1.5 m** of the path, so it is moving and climbing at the same time. |
| 3 | `PATH_END`, `LAND_1` | Settles over the path's end (the GOAL marker) and **touches down**. |
| 4 | `GROUND_1`, `CLIMB_2` | After 0.5 s on the ground, climbs straight up to **2.0 m**. |
| 5 | `FORWARD` | Flies **straight back over the START marker** at 2.0 m: 3.26 m, **without yawing**. |
| 6 | `LAND_2`, `DONE` | **Touches down on the START marker** and keeps pressing down until PX4 disarms. On drone 11, `COM_DISARM_LAND` = 5 s. |

**Nose direction at lift-off: east**, along the map's +x axis (to the right in
`path.png`, the CAD X direction). That is `start_yaw_deg: 0.0`.

**East is required for this path.** The path-planning repo (e0f22d8) computed
its wall margins for the 0.50 × 0.40 m drone held facing east the whole way.
Those margins are as low as 8 cm.

The node can still fly at another heading if `start_yaw_deg` says so: it
rotates the path to match, and the return leg still ends over START. But the
planner's margins then no longer hold, and the node logs a WARNING at
start-up.

**The drone never yaws (default, `hold_heading: true`).** The nose, and so the
camera, faces east for the whole flight. The return leg goes 95.6° to the left
of the nose: sideways, slightly backwards.

`hold_heading: false` makes the drone turn 95.6° counter-clockwise to face
START before the return leg. That happens at 2 m, above the walls, but it
also triggers the warning, so leave it off for this path.

Every setpoint is a velocity (`/skybeta/setpoint/twist`), never a pose. The
flight interface floors every pose at `min_altitude_m` (0.3 m), so a pose
cannot hold 0.2 m and cannot land. Twists are not floored. The interface still
caps their speed at 1.5 m/s; this node itself stays under 1.0 m/s.

On the path the speed is 0.25–0.6 m/s, lower where the planner's wall margin
is thin. The flight from arming to final disarm takes about 60 s.

## Load it onto the drone

1. Copy the package to drone 11. Run `hostname` first: it must say
   `skybeta-11`. Never log in to any other drone.

   ```bash
   scp -r team_ws/src/dronehacks_pathfly <user>@<skybeta-11>:~/team_ws/src/
   ```

2. Point the stack at it in `/etc/skybeta/drone.env` on the drone, then
   restart the stack. Restart only on the ground, with the drone still.

   ```bash
   COMPETITOR_PKG=dronehacks_pathfly
   COMPETITOR_LAUNCH=pathfly.launch.py
   ```

   ```bash
   docker rm -f skybeta-stack      # it comes back in ~10 s
   ```

   The stack's `competitor` window builds the whole of `~/team_ws/src`, then
   launches `flight_interface` and this node. Any other package in
   `~/team_ws/src` must also build.

3. Check the log:

   ```bash
   docker exec -it skybeta-stack tmux attach -t skybeta
   ```

   In the `competitor` window you should see `path .../path_dense.csv` and a
   summary line, for example:

   ```text
   path 12.27 m, 124 points, start (0.580, 4.596), end (0.900, 1.350); ...
   NOSE AT LIFT-OFF: 0.0 deg CCW from arena +x (east, along arena +x).
   Last leg: goal -> take-off point, 3.26 m at 95.6 deg = 95.6 deg counter-clockwise of the nose
   ```

## Before every flight

Run it on the bench with the **props off** before the first real flight, and
again after any change. Then fly it low and tethered.

1. **Place the drone centred on the START marker with its nose pointing east,**
   along the map's +x axis: to the right in `path.png`, parallel to the
   arena's top wall, towards the far end of the top corridor.
   - "East" here means the map's +x, not compass east. The drone has no
     compass.
   - The default config expects exactly this. If you place it another way,
     set `start_yaw_deg` in [config/pathfly.yaml](config/pathfly.yaml) to
     match. The node rotates the whole path by this angle; the return leg
     still ends over START.
   - A few degrees of error rotate the whole path about START: 3° moves the
     goal end about 0.17 m sideways, which matters in the narrow corridor.
     Line the drone up carefully, for example against the top wall.
2. **The return leg crosses the walls at 2.0 m.** The straight line from the
   GOAL back to START passes over the wall block on the left of the map.
   Check that nothing taller than about 1.7 m stands on that line.
3. Keep the drone still while the stack starts. The node anchors the path to
   the odometry pose at the moment Offboard begins, so it does not matter
   where the drone was when the stack powered up.
4. Safety pilot procedure:
   1. Arm with the mode switch on **Altitude**, throttle low.
   2. Switch to **Position**.
   3. Offboard starts and the node takes off.

   To take over at any time, raise the throttle towards centre or flip the
   mode switch. The node then stops for good: it does not re-enter Offboard
   until the vehicle is disarmed and armed again.

## What it does when something goes wrong

| Situation | Behaviour |
|---|---|
| No fresh odometry before take-off | Publishes nothing, so the interface refuses Offboard. |
| Offboard already active when the node starts (node restarted in flight) | Does not fly. |
| Offboard lost (pilot took over, failsafe) | `ABORTED`: publishes nothing from then on. |
| Odometry stale for more than 0.3 s | Zero-velocity twist; the mission pauses. |
| Odometry stale for more than 2 s | `HOLD`. |
| More than 0.5 m off the path or the forward leg | `HOLD`: hovers in place. The pilot lands. |
| Higher than 2.5 m above the start | `HOLD` at 2.2 m. |
| Drone lags the reference by 8–18 cm | The reference slows, then waits for it. |
| Touchdown | Detected from PX4's land detector (`landed` in `/skybeta/state`), or from the drone not descending below 0.3 m while commanded down, or from a timeout. |

## Offline dry run and tests

These need no ROS. Run them from this directory:

```bash
python3 -m dronehacks_pathfly.sim                         # timeline + final position
python3 -m dronehacks_pathfly.sim --start-yaw-deg 90 --noise 0.02 --csv run.csv
python3 -m dronehacks_pathfly.sim --set hold_heading=false   # with the 66° turn
python3 -m unittest discover -s test                      # 34 tests
```

The simulator runs the real mission code against a simple PX4 model:
- velocity lag;
- twist expiry;
- auto-Offboard on arming;
- the land detector;
- `COM_DISARM_PRFLT` and `COM_DISARM_LAND`.

Wall clearance is estimated from the planner margins in the path CSV, not from
the arena map. Results:
- **No noise:** the drone stays within 29 mm of the path. The worst
  estimated clearance is 65 mm, under the north wall.
- **2 cm odometry noise plus 5 cm/s of drift PX4 does not see:** within
  42 mm. The worst estimated clearance is 52–64 mm, under the north wall.
- **10 cm/s of unseen drift** is too much: in the 8 cm corridor under the
  north wall the estimated clearance drops to −20 mm, which is a wall touch.

The simulator is not the airframe. Its numbers say nothing about VIO quality
on the real course.

## Known limits

- **Two stretches are tight.** The planner leaves 79–82 mm of margin over
  about 0.8 m under the north wall and at the goal. That is measured for the
  real 0.50 × 0.40 m footprint, **nose held east**. VIO error and any heading
  error eat into it directly. The node warns at start-up if it is configured
  for any other heading or to yaw.
- **Check the north wall's position.** The path assumes the north wall is
  0.54 m inside the outer boundary (y = 5.556). If the taped wall is 0.61 m in
  (y = 5.486), the drone's top edge passes it with about 1 cm, not 8 cm.
- **The 0.2 m lift-off is close to the floor.** It may be at or below the
  minimum range of the downward rangefinder. Watch the first lift-off: if
  height there is unreliable, raise `liftoff_height_m`.
- **The return leg is flown at 2.0 m over the walls.** That gives about
  0.35 m above walls 1.65 m tall, if the arena's are, less the drone's own
  height. The leg has no obstacle checking. The camera also sees the wall top
  close below it.
- **Velocity commands are not rotated by `odom_to_ned_yaw_offset_rad`.** A
  constant yaw mismatch between the vision frame and PX4's shows up as a
  sideways error, which the position loop corrects. Fly a hover first.
- **The interface has no land command.** Touchdown is a slow commanded
  descent: 0.4 m/s, then 0.2 m/s below 0.4 m. The second take-off must start
  within `COM_DISARM_LAND` (5 s); the node waits only 0.5 s.

## Parameters

All parameters are in [config/pathfly.yaml](config/pathfly.yaml). The full
list, with defaults, is the `Config` dataclass in
[dronehacks_pathfly/mission.py](dronehacks_pathfly/mission.py). Write every
number as a float (`2.0`, not `2`); `hold_heading` is `true` or `false`.

To use another parameter file:

```bash
ros2 launch dronehacks_pathfly pathfly.launch.py params_file:=/root/team_ws/my.yaml
```
