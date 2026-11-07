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
| 6 | Land as the course follower does: 0.3 m/s down, pulling back over the start | | |
| 7 | Idle on the ground 1 s after PX4 reports landed (under `COM_DISARM_LAND`, 2 s) | | |
| 8 | Take off to 0.73 m | | 3 s |
| 9 | Land the same way; PX4 disarms | | |

Forward/left are the body axes at power-on. Moves never yaw; a hold's 3 s start once the drone
is within 10 cm and 5 deg. Pilot to Altitude: stops for good. Pose stale: holds still, then
straight down after 3 s. A step over 20 s: lands where it is. Status on `/square_test/status`.

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

Use the flight recorder in this branch (`dronehacks_telemetry`, see its README). Its report's
per-leg tracking treats each waypoint as a leg, so each side, climb and landing gets its own
row. On the Jetson host, with `REC` pointing at `dronehacks_telemetry/scripts/record_flight.sh`:

```bash
bash $REC start --label square1     # before arming
bash $REC stop                      # after landing and disarm
```

then fetch PX4's `.ulg` with QGroundControl into the run's `px4/` folder and run
`python3 -m dronehacks_telemetry.analyze <run>` on a laptop.

## Tests

```bash
python3 -m pytest team_ws/src/dronehacks_square_test/test -q
```

A crude drone model (pose tracking at 0.6 m/s, the kit's 0.3 m floor, PX4's auto-disarm after
landing, yaw at 60 deg/s): the full sequence, the turns and their rate, a ground wait longer
than `COM_DISARM_LAND`, pilot takeover, a stale pose, flying without a heading.

Not checked: any real flight. Built and its tests run in skybeta-11's stack on 2026-11-07.
