# The kit's interfaces, as preloaded on the drone

This page records what this method relies on from the organisers' software on the drone's
Jetson. The source is the preloaded snapshot, committed extracted at the repo root as
[`drone-snapshot/`](../../../../drone-snapshot/) (from `drone-snapshot.zip`, SHA-256 `f44b5788…175e`,
captured 2026-11-06; AGENTS.md §1). It holds skybeta `VERSION` 0.3.2 under
`drone-snapshot/home/dronehacks/skybeta/`; the paths below are relative to that. Re-read it if the
organisers ship a new version.

## Topics team code may use

From `docs/competitor_guide.md` §1, "The contract":

| Topic | Type | Team code | Frame |
|---|---|---|---|
| `/visual_slam/tracking/odometry` | `nav_msgs/Odometry` | subscribes | ENU in `odom`; FLU body twist |
| `/skybeta/setpoint/pose` | `geometry_msgs/PoseStamped` | publishes | ENU in `odom` |
| `/skybeta/setpoint/twist` | `geometry_msgs/TwistStamped` | publishes | ENU world velocity |
| `/skybeta/state` | `std_msgs/String` (JSON) | subscribes | — |

"Do not publish to `/fmu/in/*` yourself." Reading `/fmu/out/*` is how the guide itself checks the
yaw (`ros2 topic echo /fmu/out/vehicle_attitude --qos-reliability best_effort`).

## The `odom` frame

From `docs/competitor_guide.md` §1, "Frames — read this twice":

- **Labelled ENU, but not compass-referenced:** `odom` is called ENU, yet "East" and "North"
  mean "whatever direction the vehicle's x and y axes pointed when the SLAM node started".
- **Origin:** the startup pose.
- **On the launch mark, facing north:** x is north and y is west. That is the tracker frame
  `pose_fusion` works in (contracts/frames.md, "Origin").
- **The flight interface's conversion:** `(x_ned, y_ned, z_ned) = (y_enu, x_enu, -z_enu)` and
  `yaw_ned = 90° - yaw_enu`. PX4's local frame is EKF2's, so its heading is arbitrary.
  **Open question for the frames contract:** contracts/frames.md describes course → PX4 NED/FRD
  setpoints as "north → PX4 +x (forward)". The kit's interface takes ENU `odom` setpoints and
  permutes x/y, so team code never sends FRD setpoints. Nothing in this method depends on that
  mapping, but the Follower will.

## The vision bridge

From `src/oakd_vslam_bringup/oakd_vslam_bringup/px4_odom_bridge.py` and
`container_scripts/run_bridge.sh`:

- **Topics:** `input_topic` (default `/visual_slam/tracking/odometry`) → `output_topic` (default
  `/fmu/in/vehicle_visual_odometry`). The PX4 side uses BEST_EFFORT / VOLATILE / KEEP_LAST.
- **Frames:** ENU/FLU becomes NED/FRD. `pose_frame` defaults to FRD (arbitrary heading), so EKF2
  works out the rotation between the vision frame and its own.
- **Variances:** the position variance is permuted x/y; the orientation and velocity variances
  pass through. `min_position_variance` defaults to 0.
- **Resets:** `reset_counter` increments on each `/visual_slam/status` non-tracking → tracking
  transition.
- **Launch:** `run_bridge.sh` starts it with **all defaults**, on the skybeta underlay only
  ("Flight-critical path ... never the team overlay"). No setting changes the bridge's input.
  Putting anything in front of it would mean changing the organisers' stack start-up.
- **What this method does instead:** the bridge and PX4's estimate are left alone. The drift
  correction goes into the setpoints sent to `/skybeta/setpoint/*` (README, "Where it sits"),
  which is the interface the guide gives team code.

## PX4 settings that matter here

From `px4/params/03a_height_reference.params` and `03b_ekf2_vision.params`:

| Parameter | Value | Effect here |
|---|---|---|
| `EKF2_EV_CTRL` | 9 | vision gives horizontal position and yaw only |
| `EKF2_EV_NOISE_MD` | 0 | EKF2 uses the variances in the message. That is why `pose_fusion` forwards the Guard's variances unchanged |
| `EKF2_EVP_NOISE` / `EKF2_EVA_NOISE` | 0.10 / 0.05 | |
| `EKF2_EVP_GATE` | 5 (baseline) | a correction jump would be gated; `pose_fusion` ramps corrections in instead |
| `EKF2_EV_DELAY` | 70 (bench), 60 (drone 16 file) | contracts/px4.md sets 0, the team's own measurement. Not this method's call |
| `EKF2_HGT_REF` / `EKF2_BARO_CTRL` / `EKF2_RNG_CTRL` | 2 / 1 / 1 | rangefinder height, barometer backup, range fused only below 1 m/s and 3 m. The localiser's body height (rest height + PX4's climb) inherits this |

The old `course/docs/hardware-setup.md` recommends `EKF2_HGT_REF 0` (barometer). The organisers'
value above wins (contracts/px4.md), and that guide's PX4 parameter table does not apply to the
issued drone.

## Running team code on the drone

From `docs/competitor_guide.md` §5 and `host/run.sh`:

- **Where code lives:** team packages go in `~/team_ws/src`, bind-mounted at `/root/team_ws`.
  `team_build.sh` (colcon, `--symlink-install`) builds them as an overlay. Code anywhere else in a
  container is lost when it stops.
- **Extra dependencies:** they go in a team image (`host/run.sh build --init`, then edit
  `~/team_ws/Dockerfile`). Never `pip install` in a running container. The localiser needs
  `pyserial`.
- **Devices:** the stack container runs `--privileged` with `/dev/bus/usb` mounted, so a USB
  serial lidar should be visible inside it. That is not verified on the drone yet.
- **Launch:** `COMPETITOR_PKG` / `COMPETITOR_LAUNCH` in `/etc/skybeta/drone.env` choose the team
  launch file. It must start `flight_interface` with `auto_offboard_on_arm:=true`.
- **Restarts:** restarting the stack re-zeroes the VIO origin, so do it only on the ground.
- **`drone.env`:** keys `DRONE_ID`, `TEAM`, `ROS_DOMAIN_ID`, `SKYBETA_DDS_SCOPE`,
  `CAMERA_X/Y/Z/ROLL/PITCH/YAW` (the organisers' measured camera mount), `TEAM_WS`,
  `COMPETITOR_PKG`, `COMPETITOR_LAUNCH`. No secrets. Its values are not copied here.
- **No lidar:** the snapshot contains no lidar driver or launch file. The MS200K reader is ours
  (`course/ms200.py`).
