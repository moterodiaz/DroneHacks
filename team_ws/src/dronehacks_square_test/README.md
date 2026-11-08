# dronehacks_square_test

A cage test for the autonomy chain on cuVSLAM alone: take off, fly a small box with slow turns,
land, rest on the ground, take off, fly a circle, land and disarm. Same kit contract as `competitor_example` (pose and twist
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
| 7 | Rest on the ground 2 s after PX4 reports landed, motors idling (`COM_DISARM_LAND` is 5 s on skybeta-11; at 2 s the run ends here) | | |
| 8 | Take off to 0.73 m, then a 1 m circle inside the box and back over the start | | 3 s |
| 9 | Land the same way; PX4 disarms | | |

Forward/left are the body axes at power-on. Moves never yaw; a hold's 3 s start once the drone
is within 20 cm horizontally, 15 cm vertically and 10 deg. Pilot to Altitude: stops for good. Pose
stale: holds still, then straight down after 3 s. A step over 30 s: lands where it is. Status on
`/square_test/status`.

## Heading mode (does yawing cause the corner drift?)

In `flights/20261108-013018-light-square` the drone drifted 10-20 cm off each corner after its
in-place yaw and took about 10 s to crawl back; a hold without a yaw stayed within about 5 cm.
`heading_mode` tests that without touching PX4:

| `heading_mode` | Flies |
|---|---|
| `turn` (default) | the table above: +90 deg at each corner (a "lap turn") |
| `fixed` | the same box and circle never yawing: the take-off heading held throughout (a "lap fixed") |
| `both` | a lap fixed, then a lap turn from over the start, then steps 6-9: the A/B on one battery and one bias |

```bash
ros2 launch dronehacks_square_test square_test.launch.py heading_mode:=both
```

The stack launches team code with no arguments, so to fly it from the stack set the default in the
drone's copy before restarting the stack:
`sed -i "s/('heading_mode', 'turn')/('heading_mode', 'both')/" ~/team_ws/src/dronehacks_square_test/launch/square_test.launch.py`.
The status names the lap (`FLY: step 4/28: hold, lap fixed`), so the bag shows which is which.

## Push compensation (cancel the drift and still turn)

The drift is a push fixed to the airframe (about 0.10 m/s² forward-left of the nose in
`20261108-013018`, 0.25 in `20261108-003042`) that PX4's velocity integrator cancels in world
axes, so after a yaw its correction points the old way until it relearns it. `yaw_comp`
(`yaw_comp.py`, the same module as the course follower's) measures the push from PX4's
accelerometer (`/fmu/out/sensor_combined`, FRD: forward = ax, left = -ay, times
`yaw_comp_push_gain`) over 2.5 s of the first hold (it waits for it; under 0.2 m/s and 5°/s), models
the integrator as a lag of R(heading) d over `yaw_comp_tau_s`, and adds -(R d - L) / `yaw_comp_gain`,
capped at `yaw_comp_max_m`, to every pose setpoint before PX4's offset. Off by default, not yet flown.

| Argument | Default | |
|---|---|---|
| `yaw_comp` | false | `true` compensates throughout; `both` flies the turning box twice, a "lap comp off" then a "lap comp on", compensated from there on (needs `heading_mode` turn) |
| `yaw_comp_tau_s` | 4.5 | PX4's integrator lag, `MPC_XY_VEL_P_ACC` / `MPC_XY_VEL_I_ACC` at stock values |
| `yaw_comp_gain` | 1.71 | PX4's pose-to-acceleration gain, `MPC_XY_P` x `MPC_XY_VEL_P_ACC` at stock values |
| `yaw_comp_max_m` | 0.20 | cap on the offset |
| `yaw_comp_push_gain` | 1.25 | the push per unit of hover accelerometer reading, fitted on two flights' corners |

```bash
ros2 launch dronehacks_square_test square_test.launch.py yaw_comp:=both
```

The log names the push once and the offset each second; `/square_test/yaw_comp` carries both
(JSON) for the bag. Without `px4_msgs` or accelerometer data it never offsets and no hold waits.

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

The mission handles what remains: every setpoint carries the PX4-minus-camera offset (PX4's
position from `/skybeta/state` `local_enu`), taken at Offboard and followed, smoothed over 1 s,
through the flight, since the rangefinder reads wrong on the floor and right in the air. Its checks
stay in the camera frame. Over `max_offset_m` (0.3 m) on the ground it streams nothing, so the kit
never takes over; the status says `PX4 and camera disagree by 0.73 m: power-cycle on the spot`.

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
landing, yaw at 60 deg/s): the full sequence, the turns and their rate, a ground rest longer
than `COM_DISARM_LAND`, a disarm mid-flight, pilot takeover, a stale pose, flying without a
heading, each heading mode, and the push compensation's A/B, cap on the pose and off without IMU.

Not checked: any real flight. Built and its tests run in skybeta-11's stack on 2026-11-07.
