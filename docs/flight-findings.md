# Flight findings: what the simulators and the organisers' lab showed

What any flight package on the Sky Beta has to get right, collected from Michael Bar On's
DroneDepthCamTesting repo (branch `host-agnostic-cuvslam-setup`: `docs/dronehacks-2026.md`,
`docs/day-one-checklist.md`, `sim/README.md`), where each finding is recorded with its command,
host and output. **[M]** measured, **[I]** inferred. "SITL" is that repo's PX4 v1.16.1 SITL with
Gazebo, a simulated OAK-D and cuVSLAM in the loop, flown through the organisers' unmodified bridge.
The section "On skybeta-11" is from our own drone's square-test flights; the rest is not yet
checked on a real drone.

## On skybeta-11 (2026-11-07) [M unless marked]

Read from the light bags in `flights/` (odometry, `/skybeta/state`, `/fmu/in/trajectory_setpoint`,
`/fmu/out/*`); no PX4 ULog yet. Analysis scripts in Michael Bar On's session scratchpad (`drift/`,
`flights11/`).

- **PX4's height has its own origin.** PX4 takes height from the rangefinder with its origin
  where the FC powered up; odom's is where the stack started. `003042` read PX4 −0.72 m on the
  floor against odom 0.01 m and flew to 1.33 m for 0.74 m. Power-cycling on the mark plus the
  PX4-offset correction fixed it: `013018` held 0.71-0.75 m for 0.73 m, with the offset 0.13 m on
  the floor and 0.20-0.23 m in the air.
- **The drone drifts after every yaw in place, not on arrival.** In `013018` PX4's estimate and
  odom agree (median 1.3 cm, 95th percentile 3.3 cm; heading 0.6°) and PX4 receives the right
  target, so it is control error. Moves close to under 3 cm within about 1 s; a hold without a yaw
  stayed within 7 cm. After each 90° yaw at 20°/s the error peaked at 10-18 cm, 1-3 s after the
  yaw ended, always to the left of the new nose, and took up to 8 s to come back under 5 cm.
- **Why: a push fixed to the airframe.** The acceleration PX4's tilt does not account for is
  about 0.10 m/s² (about 0.6° of tilt), forward-left of the nose; it is steadier in the body frame
  than the world frame, and rotating it by each yaw predicts every corner's drift direction within
  9-18°. PX4's integrator holds the correction in world axes, so a yaw leaves it pointing the wrong
  way until relearned. In `003042` the push was 0.25 m/s². [I] A thrust axis tilted about 1° from
  the IMU, or an unlearned accelerometer bias; the ULog's `estimator_sensor_bias` tells them apart.
- **What it means for the route [I].** A heading change Δψ leaves PX4 short by 2 sin(Δψ/2) times
  the push: 10-18 cm per 90° at 0.10 m/s², 25-35 cm at 0.25, against about ±0.18 m of margin in
  the maze. Fixes without touching PX4: hold a fixed heading (`heading_mode:=fixed` in both
  packages), or yaw slowly on straights, in smaller steps, with a settle after. The square test's
  `heading_mode:=both` flies one lap fixed and one turning, an A/B in one flight.
- **Touch-and-go:** PX4 disarmed 1.9 s after touchdown, 0.8 s into the climb out, and Offboard
  needs the pilot after a disarm. A return flight must climb within about 1 s of touchdown.
- **Landing:** 0.3 m/s showed no ground contact for 1.3 s after touchdown (`003042`); both packages
  descend at 0.5 m/s, and `013018` disarmed on the ground. Odom moved about 15 cm sideways at
  touchdown; tape-measure the landing spot to tell a slide from a tracker jump.

## Before arming (2026-11-07)

- **Offboard needs skybeta 0.3.3 or its hotfix [M, upstream].** The 0.3.2 interface sent
  `DO_SET_MODE` to system 1, but each fleet FC's `MAV_SYS_ID` is its drone number, so PX4 dropped
  it and no drone but drone 1 entered Offboard. Fixed upstream in 0.3.3 (`target_system` 0 = the
  FC's own). The organisers' announcement listed drone 11, ours. Check: `cat ~/skybeta/VERSION`;
  after arm-then-Position, `docker logs skybeta-stack 2>&1 | grep -i offboard` must not say
  "did not enter Offboard within 2 s".
- **Arm in Altitude, then switch to Position** (organisers' announcement). The interface then
  requests Offboard once; arming already in Position gives no autonomy.
- **Restart the stack with the drone on the launch mark** (`docker rm -f skybeta-stack`):
  cuVSLAM's odom origin is wherever the stack started, not the mark.
- **Motor order and direction, props off**, on every drone swapped in. Our first drone flipped
  on take-off from a firmware flash that overwrote the motor configuration; the organisers
  reconfigure it once on request.
- **`COMPETITOR_PKG`** in `/etc/skybeta/drone.env` is what flies on arming. Drone 1 shipped
  `competitor_example`, a 5 m square.

## The kit's interface [M, read from drone 1's snapshot]

- It resends the latest pose at `setpoint_rate_hz`, 10 Hz by default. A follower publishing
  faster has its later poses held 100 ms each. In SITL, our led poses through a 10 Hz interface
  flew four of six runs cleanly and lost yaw control at a corner in one; at 50 Hz all were clean.
  Fallback: plain (unled) poses, slower. Worth asking the organisers whether 50 Hz is allowed.
- Pose setpoints are clamped to 0.3-3.0 m above the odom origin; twists are clamped only in speed.
  So landing has to be a downward twist; a pose cannot reach the floor.
- An all-zero quaternion means "yaw not commanded".
- `COM_RC_OVERRIDE` 3: raising the throttle stick past 30 % takes the drone back from Offboard.

## Localization held; the reference mission's flying did not (organisers' lab, 2026-11-07) [M]

In the organisers' FlyX lab (their simulator, ROS 2 Jazzy, cuVSLAM about 13.6 Hz):
- PX4's estimate stayed within 2-6 cm of truth and 1° of heading.
- Their reference mission's step setpoints overshot each leg by 38-60 cm and swung the heading
  ±40°.
- Both flights then turned in place facing a wall: under 1 % texture in view, cuVSLAM failed,
  PX4's position went invalid and the landing failsafe ended the run at legs 3-4.

**[I] For any follower:** smooth, rate-limited setpoints, not steps; and never turn in place
facing a wall. Turn while moving, looking along the route.

## Take-off [M, SITL, 39 flights]

- In 20 of 39 runs the drone came within its 28 cm prop reach of a wall during PX4's take-off,
  nearest 4.5 cm. It slides 25-45 cm along the floor at near-hover thrust before lifting, then
  swings back once airborne. PX4's estimate followed the truth, so this is motion, not
  estimation error. The organisers' notes on the real Sky Beta report 1-2 s from spool-up to
  lift-off and about 0.35 m/s of drift in Altitude mode.
- The launch mark is 40.6 cm from the nearest wall. The most open point within 0.2 m of it is
  15 cm north (53.3 cm clear). Whether that still counts as "on the mark" is an organiser question.
- **[I]** Leave the floor fast, from where there is most room.

## Landing [M, SITL]

- PX4 counts ground contact only while the setpoint descends at 1.1 × `LNDMC_Z_VEL_MAX` or more:
  0.275 m/s at the fleet's 0.25 (v1.16.1 `MulticopterLandDetector`). A 0.3 m/s twist clears it
  narrowly; 0.5 m/s clears it with margin.
- The time goes to detection and the disarm, not the descent: about 3 s to `landed`, then
  `COM_DISARM_LAND`'s 2 s.

## The course [M, SITL]

- **The window** was the closest pass in 21 of 39 runs: PX4 cut the turn into the opening by
  about 8.7 cm toward the frame. Our planner flies it as one straight leg on its normal from up
  to 0.7 m before to 0.7 m after, so corner arcs end before the frame.
- **The tunnel's side walls** are not in `contracts/course_v3_layout.yaml` (the tunnel is an
  axis there); a planner needs them as walls.
- **Losing tracking while moving is what crashes [M, SITL, 2026-10-26].** Blinding the camera
  for 5-8 s mid-leg overshot a corner by about 4 m in the worst run; a pose that jumps back when
  vision returns does the same. Hovering blind for 6 s only grew the error from 2 to 5 cm.
- **The rangefinder** was rejected by EKF2 after a slow window crossing in one reversed-course
  SITL run. Check `estimator_status` in the first real flight's log.
- **Generality:** the same planner, with no route file, finished 8 of 9 SITL flights on course
  layouts it had never seen.

## What to record on every flight

`team_ws/src/dronehacks_telemetry` (branch `telemetry/flight-recorder`): a bag from arming to
disarm plus the PX4 ULog, read by `python3 -m dronehacks_telemetry.analyze <folder>`. It reports
vision fusion and delay, setpoint tracking, vibration, the take-off slide and the landing time,
the numbers the findings above are about.
