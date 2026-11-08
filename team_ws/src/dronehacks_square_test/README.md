# dronehacks_square_test

A cage test for the autonomy chain on cuVSLAM alone: take off, fly a small box with slow turns,
land, touch-and-go, land and disarm. Same kit contract as `competitor_example` (pose and twist
setpoints to `skybeta_flight_interface`; nothing to `/fmu/in/*`, no PX4 parameters, never arms
or changes mode). About 57 s.

| # | Step | During the hold | Hold |
|---|---|---|---|
| 1 | Climb to 0.73 m (the course follower's setpoint: cruise 0.80 m - rest 0.07 m) | | 3 s |
| 2 | Forward 0.5 m | turn +90 deg (left) at 20 deg/s | 3 s |
| 3 | Left 0.5 m | turn +90 deg | 3 s |
| 4 | Back 0.5 m | turn +90 deg | 3 s |
| 5 | Right 0.5 m, over the start | turn +90 deg, back to the take-off heading | 3 s |
| 6 | Land as the course follower does: 0.5 m/s down (PX4 needs over 0.275 m/s to see ground contact), pulling back over the start | | |
| 7 | Idle on the ground 1 s after PX4 reports landed (under `COM_DISARM_LAND`, 2 s) | | |
| 8 | Take off to 0.73 m | | 3 s |
| 9 | Land the same way; PX4 disarms | | |

Forward/left are the body axes at power-on. Moves never yaw; a hold's 3 s start once the drone
is within 10 cm horizontally, 15 cm vertically and 5 deg. Pilot to Altitude: stops for good. Pose
stale: holds still, then straight down after 3 s. A step over 20 s: lands where it is. Status on
`/square_test/status`.

## Power up on the spot (why the first flights never moved sideways)

The kit's interface hands PX4 our odom setpoint unchanged (ENU to NED), so PX4's local frame and
cuVSLAM's odom must start in the same place. PX4 takes only x, y and yaw from vision
(`EKF2_EV_CTRL` 9) and its height from the rangefinder (`EKF2_HGT_REF` 2), so its height origin is
where the flight controller powered up; odom's origin is where the stack started. In skybeta-11's
last flight (`flights/20261108-003042-light-square`, recovered from the cut bag) PX4 read -0.72 m
on the floor while odom read 0.01 m: it flew to 1.33 m by odom for a 0.74 m setpoint, so the first
hold's height check never passed and the 20 s timeout landed it. In `20261108-002220` odom read
-0.76 m on the floor (the stack had started higher). **Power-cycle the drone, FC and Jetson,
sitting on its take-off spot**, and leave it there until the stack is up.

The node checks it: before Offboard it compares PX4's `/fmu/out/vehicle_local_position` (read only)
with odom, and while they differ by over `frames_limit_m` (0.15 m) it streams nothing, so the kit
never takes over. The log says `PX4 height and cuVSLAM disagree by 0.73 m: not streaming, so no
Offboard. Power-cycle the drone ...`, then `PX4 and cuVSLAM agree: streaming` once fixed.

## On the drone

Copy this folder to `~/team_ws/src/`, then point the stack at it (`sudo`, drone on its take-off
spot, still):

```bash
sudo cp /etc/skybeta/drone.env /etc/skybeta/drone.env.bak && sudo sed -i \
  -e 's/^COMPETITOR_PKG=.*/COMPETITOR_PKG=dronehacks_square_test/' \
  -e 's/^COMPETITOR_LAUNCH=.*/COMPETITOR_LAUNCH=square_test.launch.py/' /etc/skybeta/drone.env \
  && grep ^COMPETITOR /etc/skybeta/drone.env && docker rm -f skybeta-stack
```

The stack rebuilds `~/team_ws` and comes back on its own. Arm in Altitude, switch to Position
within 10 s.

## Recording it

Automatic, from arming only. The node starts a `dronehacks_telemetry` light bag (its exclude regex) at
the first arming into `~/team_ws/flights/<UTC>-light-square/` (`bag/`, `meta/manifest.txt`, an empty
`px4/`); nothing is recorded while the drone sits powered and disarmed. When the mission is over and
PX4 has disarmed, the node closes the bag with SIGINT 5 s later and exits.
Power cut before that: `ros2 bag reindex <run>/bag -s mcap` in the
container. `record:=false` turns it off. The folder is root-owned (written in the container):
readable for `scp`, deletable only from inside the container.

Then fetch PX4's `.ulg` with QGroundControl into the run's `px4/` folder and run
`python3 -m dronehacks_telemetry.analyze <run>` on a laptop; the report's per-leg tracking
treats each waypoint as a leg, so each side, climb and landing gets its own row.

## Tests

```bash
python3 -m pytest team_ws/src/dronehacks_square_test/test -q
```

A crude drone model (pose tracking at 0.6 m/s, the kit's 0.3 m floor, PX4's auto-disarm after
landing, yaw at 60 deg/s): the full sequence, the turns and their rate, a ground wait longer
than `COM_DISARM_LAND`, pilot takeover, a stale pose, flying without a heading.

Not checked: any real flight. Built and its tests run in skybeta-11's stack on 2026-11-07.
