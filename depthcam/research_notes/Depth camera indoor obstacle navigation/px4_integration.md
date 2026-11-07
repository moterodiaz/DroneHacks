# PX4 integration for companion-computer VIO, depth-camera obstacle data and Offboard control (Pixhawk 6C, indoor, GPS-denied)

Scope: PX4-side interfaces, parameters, message rates and safety mechanisms for (a) external VIO (cuVSLAM on a Jetson Orin Nano, from an OAK-D S2) into EKF2, (b) obstacle data into PX4 Collision Prevention, and (c) Offboard control from Python. Research date: 2026-10-25. Where the docs and the code disagree, the PX4 v1.17 source (branch `release/1.17`, docs commit of 2026-08-06) was read directly and is cited. "R117" links below point to `https://github.com/PX4/PX4-Autopilot/blob/release/1.17/...`.

## 1. Which PX4 release to target (October 2026) and what changed for these features

### Takeaway
PX4 **v1.17.0** is the current stable release (published 2026-05-13). v1.18 is in beta/RC: v1.18.0-rc1 came out 2026-09-10. The feature changes that matter came mostly in v1.16 (Aug 2025). v1.16 removed the PX4-side avoidance interface, moved Collision Prevention to acceleration-based Position mode, and added ROS 2 message versioning. Many older tutorials use parameters that no longer exist: `COM_OBS_AVOID`, `MAV_ODOM_LP`, `COM_POS_FS_DELAY`, `MPC_COL_PREV_D`, and the simple/smooth `MPC_POS_MODE` values.

### Cited Findings
- GitHub release list: v1.17.0 stable, published 2026-05-13. v1.16.2 2026-04-22, v1.16.1 2026-01-21, v1.16.0 2025-08-06. Pre-releases: v1.18.0-alpha1 2026-05-13, beta1 2026-07-08, beta2 2026-08-09, rc1 2026-09-10. Only one v1.17 tag (v1.17.0) exists, so there is no v1.17.x patch yet. — [PX4 releases (GitHub)](https://github.com/PX4/PX4-Autopilot/releases)
- The docs release index marks v1.17 "Stable" and v1.18 "Beta". The v1.18 notes say "PX4 v1.18 is in beta testing." — [PX4 releases index](https://docs.px4.io/main/en/releases/); [v1.18 release notes](https://docs.px4.io/main/en/releases/1.18.html)
- **v1.16 changes relevant here:**
  - Collision prevention now works in acceleration-based Position mode (`MPC_POS_MODE`) (PR #23507, merged 2024-11-21).
  - Optical flow is enabled by default in EKF2 (PR #23436). New `SENS_FLOW_SCALE` (PR #23936).
  - `COM_POS_FS_DELAY` removed: the position-loss failsafe now triggers 1 s after position loss (PR #24063).
  - Optional uORB message version field (PR #23850) and an experimental ROS 2 message translation node (PR #24113).
  - Gazebo Harmonic replaces Garden (PR #23603). New x500_lidar_2d, x500_lidar_front and x500_lidar_down models.
  - VOXL 2 boards explicitly set `EKF2_EV_CTRL=15` (PR #24648).
  - Sources: [v1.16 release notes](https://docs.px4.io/main/en/releases/1.16.html); [PR #23507](https://github.com/PX4/PX4-Autopilot/pull/23507)
- PR #24172, "Removed Functionality: Remove collision avoidance interface on PX4 side", was merged 2025-02-18 (after v1.15, before v1.16.0). — [PR #24172](https://github.com/PX4/PX4-Autopilot/pull/24172)
- **v1.17 changes relevant here:**
  - "Offboard to Position (hold) mode change is no longer entered without RC" (PR #26391).
  - The pilot can take over from a degraded failsafe state (PR #26269).
  - `MAV_PROTO_VER` defaults to MAVLink 2 (PR #25583).
  - EKF2 range fusion skips unhealthy samples (#25634) and rejects samples during `bad_acc_vertical` (#25636).
  - `EKF2_RNG_A_IGATE` removed (#25137). `EKF2_MIN_RNG` default lowered from 0.1 m to 0.01 m (#25574).
  - `MPC_HOLD_DZ` renamed to `MAN_DEADZONE`, and `MPC_*_MAN_EXPO` removed (#25435). `COM_RC_ARM_HYST` removed (#25999).
  - Renames: `manual_lockdown` → `kill`, `FORCE_FAILSAFE` → `TERMINATION`.
  - New `UXRCE_DDS_NS_IDX` (#25444).
  - New Altitude Cruise mode. Gazebo Jetty support, with Harmonic still the supported, CI-tested version.
  - The notes list no Collision Prevention or external-vision changes.
  - Source: [v1.17 release notes](https://docs.px4.io/main/en/releases/1.17.html)
- **v1.18 (beta) changes relevant here:**
  - Fusion sources can be toggled at runtime (`VEHICLE_CMD_ESTIMATOR_SENSOR_ENABLE`). The new `EKF2_SENS_EN` replaces `EKF2_EN_BOOT`. New `ESTIMATOR_SENSOR_FUSION_STATUS` stream (PR #26739).
  - `COM_RC_OVERRIDE` and `COM_RC_STICK_OV` replaced by `MAN_OVERRIDE_SPD`, and override "now applies uniformly across auto and offboard modes" (PR #27041).
  - Deprecated MAVLink request commands removed; use `MAV_CMD_REQUEST_MESSAGE` (PRs #27251, #27252).
  - Nine commander parameters removed, including `COM_TAKEOFF_ACT`.
  - Source: [v1.18 release notes](https://docs.px4.io/main/en/releases/1.18.html)
- v1.15 changes: an Error-State Kalman Filter derivation (PR #22262), `MAV_CMD_EXTERNAL_POSITION_ESTIMATE`, Auxiliary Global Position fusion, the experimental PX4 ROS 2 Interface Library and Dynamic Modes API (PR #20707), `UXRCE_DDS_SYNCT` (PR #21757), and Gazebo replacing jMAVSim as the default simulator. — [v1.15 release notes](https://docs.px4.io/main/en/releases/1.15.html)

### Inferences
- Use v1.17.0 for the competition. A move to v1.18 RCs mid-integration would rename the override parameters, change commander parameters, and could change behaviour under you.
- If you use ROS 2, build the `release/1.17` px4_msgs branch so the message definitions match the firmware.
- Any guide written for v1.14 or earlier (including most ModalAI forum posts and the PX4-Avoidance tutorials) uses removed parameters. Check every parameter name against the v1.17 parameter reference.

### Gaps
- No v1.18.0 final date is known. No v1.17.1 patch exists as of this date.
- The v1.17 release notes text may be incomplete: a small final part of the page (about 0.9 KB) was not read.

## 2. Feeding cuVSLAM VIO into EKF2: message choice, frames, parameters, rates

### Takeaway
The best path is MAVLink **ODOMETRY** (#331) with:
- `frame_id = MAV_FRAME_LOCAL_FRD`
- `child_frame_id = MAV_FRAME_BODY_FRD`
- `estimator_type = MAV_ESTIMATOR_TYPE_VIO` (or VISION)
- the full pose and velocity covariances

The ROS 2 equivalent is `/fmu/in/vehicle_visual_odometry` (px4_msgs `VehicleOdometry`) via uXRCE-DDS. Stream at 30–50 Hz. Set `EKF2_EV_CTRL=15`, `EKF2_HGT_REF=3` (Vision), `EKF2_GPS_CTRL=0` and `EKF2_MAG_TYPE=5`, plus `EKF2_EV_POS_X/Y/Z` and `EKF2_EV_DELAY`. With no magnetometer and no GPS, EKF2 needs EV yaw fusion (bit 3) as its heading reference. **`VISION_POSITION_ESTIMATE` cannot carry velocity or quality.**

### Cited Findings
- **Message mapping (docs):**
  - `VISION_POSITION_ESTIMATE` → `vehicle_visual_odometry`.
  - `ODOMETRY` with `frame_id = MAV_FRAME_LOCAL_FRD` → `vehicle_visual_odometry`.
  - `ATT_POS_MOCAP` and `ODOMETRY` with `MAV_FRAME_MOCAP_NED` → `vehicle_mocap_odometry`.
  - **EKF2 subscribes only to `vehicle_visual_odometry`.** Only ODOMETRY carries linear velocity.
  - `GLOBAL_VISION_POSITION_ESTIMATE`, `VISION_SPEED_ESTIMATE` and `VICON_POSITION_ESTIMATE` are not supported.
  - Source: [External Position Estimation](https://docs.px4.io/main/en/ros/external_position_estimation.html)
- Rate (docs): the messages "should be streamed at between 30Hz (if containing covariances) and 50 Hz". If the rate is too low, EKF2 will not fuse them. — [External Position Estimation](https://docs.px4.io/main/en/ros/external_position_estimation.html)
- **What v1.17 `handle_message_odometry` actually does:**
  - **`frame_id` (position):** accepts `MAV_FRAME_LOCAL_NED`, `LOCAL_ENU`, `LOCAL_FRD` and `LOCAL_FLU`, converting ENU and FLU to NED and FRD. Any other value means no position is set.
  - **`child_frame_id` (velocity):** accepts `LOCAL_NED/ENU/FRD/FLU` and `MAV_FRAME_BODY_FRD`. The deprecated `BODY_NED` and `BODY_OFFSET_NED` are mapped to body FRD.
  - **Covariances:** copied only if `estimator_type != MAV_ESTIMATOR_TYPE_NAIVE`. Position variance comes from `pose_covariance` indices 0, 6 and 11, orientation variance from 15, 18 and 20, and velocity variance from `velocity_covariance` indices 0, 6 and 11.
  - **`reset_counter` and `quality`:** passed through unchanged.
  - **Routing is by `estimator_type`:** UNKNOWN, NAIVE, VISION and VIO go to visual odometry. MOCAP goes to mocap odometry, which EKF2 does not fuse. GPS, GPS_INS, LIDAR and AUTOPILOT are rejected with a critical message.
  - **Timestamps:** `timestamp_sample = _mavlink_timesync.sync_stamp(time_usec)`.
  - Source: [R117 mavlink_receiver.cpp](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/modules/mavlink/mavlink_receiver.cpp)
- **Doc/code conflict:** the docs say `ODOMETRY` with `MAV_FRAME_MOCAP_NED` routes to mocap odometry. The v1.17 code routes by `estimator_type`, and `MOCAP_NED` is not an accepted position frame. — [docs](https://docs.px4.io/main/en/ros/external_position_estimation.html) vs [R117 mavlink_receiver.cpp](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/modules/mavlink/mavlink_receiver.cpp)
- `VISION_POSITION_ESTIMATE` handling: `pose_frame` is always NED, the quaternion is built from roll/pitch/yaw, and the covariance mapping is the same as for ODOMETRY. `reset_counter` is copied, but `quality` is not set and there is no velocity. — [R117 mavlink_receiver.cpp](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/modules/mavlink/mavlink_receiver.cpp)
- **MAVLink ODOMETRY fields:**
  - `time_usec`, `frame_id`, `child_frame_id`, x/y/z (m), and `q` as w,x,y,z ("1 0 0 0 is the null-rotation").
  - vx/vy/vz (m/s) and roll/pitch/yaw speeds (rad/s).
  - `pose_covariance` and `velocity_covariance`: float[21], the row-major upper-right triangle of a 6×6 matrix. NaN in the first element means unknown.
  - `reset_counter` (uint8), "incremented when the estimate resets in any of the dimensions", for example on a SLAM loop closure.
  - `estimator_type` (`MAV_ESTIMATOR_TYPE`).
  - `quality` (int8, %): -1 = failed, 0 = unknown/invalid, 1 = worst, 100 = best.
  - Source: [MAVLink common: ODOMETRY](https://mavlink.io/en/messages/common.html#ODOMETRY)
- `VISION_POSITION_ESTIMATE` (#102) has usec, x/y/z, roll/pitch/yaw, plus the float[21] covariance and `reset_counter` as MAVLink 2 extensions. — [MAVLink common: VISION_POSITION_ESTIMATE](https://mavlink.io/en/messages/common.html#VISION_POSITION_ESTIMATE)
- **ROS 2 / uORB `VehicleOdometry` (v1.17, `MESSAGE_VERSION = 0`):**
  - `timestamp` and `timestamp_sample`.
  - `pose_frame`: 1 = NED, 2 = FRD ("constant arbitrary heading offset from True North").
  - `position` (m) and `q` (Hamiltonian w,x,y,z, rotation from body to world).
  - `velocity_frame`: 1 = NED, 2 = FRD, 3 = BODY_FRD.
  - `velocity`, `angular_velocity`, and `position_variance`, `orientation_variance`, `velocity_variance` (3 elements each).
  - `reset_counter`, and `quality` (the comment says "Unused").
  - Source: [R117 msg/versioned/VehicleOdometry.msg](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/msg/versioned/VehicleOdometry.msg)
- The default v1.17 `dds_topics.yaml` bridges the following topics:
  - Inputs: `/fmu/in/vehicle_visual_odometry`, `/fmu/in/vehicle_mocap_odometry`, `/fmu/in/obstacle_distance`, `/fmu/in/distance_sensor`, `/fmu/in/sensor_optical_flow`, `/fmu/in/trajectory_setpoint`, `/fmu/in/offboard_control_mode`, `/fmu/in/vehicle_command`, `/fmu/in/goto_setpoint`.
  - Outputs: `/fmu/out/vehicle_odometry`, `vehicle_local_position`, `vehicle_status`, `estimator_status_flags`, `failsafe_flags`, `collision_constraints`.
  - Source: [R117 dds_topics.yaml](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/modules/uxrce_dds_client/dds_topics.yaml)
- **How EKF2 ingests vision samples (v1.17):**
  - Only NED and FRD pose frames are accepted. Velocity frames NED, FRD and BODY_FRD are accepted.
  - In noise mode 0, each variance is floored with `fmaxf(EKF2_EV*_NOISE², reported variance)`.
  - The quaternion must be finite and close to unit norm.
  - `ev_data.time_us = ev_odom.timestamp_sample`, with the comment "use timestamp from external computer, clocks are synchronized when using MAVROS".
  - Source: [R117 EKF2.cpp](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/modules/ekf2/EKF2.cpp)
- **External vision parameters (main/v1.17 YAML):**
  - `EKF2_EV_CTRL`: bitmask, default 0, max 15. Bit 0 horizontal position, bit 1 vertical position, bit 2 3D velocity, bit 3 yaw.
  - `EKF2_EV_NOISE_MD`: 0 = use the EV-reported variance with the parameters as a lower bound (default), 1 = use the parameters.
  - `EKF2_EV_DELAY`: 0 ms by default, range 0–300, reboot required.
  - `EKF2_EV_QMIN`: 0–100, default 0.
  - `EKF2_EVP_NOISE` 0.1 m (min 0.01), `EKF2_EVV_NOISE` 0.1 m/s, `EKF2_EVA_NOISE` 0.1 rad (min 0.05).
  - `EKF2_EVP_GATE` 5 SD, `EKF2_EVV_GATE` 3 SD.
  - `EKF2_EV_POS_X/Y/Z`: VI sensor focal point in the body frame (m).
  - Source: [params_external_vision.yaml](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/ekf2/params_external_vision.yaml)
- `EKF2_HGT_REF`: 0 baro, 1 GPS (default), 2 range, 3 vision. Reboot required. Range and vision "should only be used when for operation over a flat surface as the local NED origin will move up and down with ground level." — [R117 ekf2 module.yaml](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/modules/ekf2/module.yaml)
- `EKF2_MAG_TYPE`: 0 Automatic (default), 1 Magnetic heading, 5 None, 6 Init. Reboot required. With None, "the estimator will use other sources of heading: GPS heading or external vision." — [R117 params_magnetometer.yaml](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/modules/ekf2/params_magnetometer.yaml); [EKF2 tuning guide](https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf.html)
- `EKF2_GPS_CTRL` is a bitmask (0 lon/lat, 1 altitude, 2 3D velocity, 3 dual-antenna heading), default 7. — [R117 params_gnss.yaml](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/modules/ekf2/params_gnss.yaml)
- Docs: "if yaw data is used (bit 3) the heading is with respect to the external vision frame; otherwise the heading is relative to North." — [EKF2 tuning guide](https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf.html)
- **EV position frame handling (v1.17 code):**
  - LOCAL_FRAME_NED EV position is used only once the EKF yaw is aligned (`yaw_align`).
  - For LOCAL_FRAME_FRD, if EV yaw is being fused, the EKF uses the EV frame directly.
  - If EV yaw is not fused, the EKF rotates EV position by a filtered EV-to-EKF rotation (`_ev_q_error_filt`) and inflates the horizontal variance by the EV orientation variance.
  - Source: [R117 ev_pos_control.cpp](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/modules/ekf2/EKF/aid_sources/external_vision/ev_pos_control.cpp)
- The VIO guide's "Check/Verify" section relies on `MAV_ODOM_LP`, but a note says "The MAV_ODOM_LP parameter mentioned below was removed in PX4 v1.14. This section needs to be updated." Other checks in the guide:
  - The quaternion should be near (1,0,0,0) when the vehicle is aligned with the EV frame.
  - z should decrease when lifted, x increase when moved forward, and y increase when moved right.
  - Velocities must be in body FRD.
  - "Toilet-bowling" means the camera-to-body transform is wrong.
  - Source: [R117 visual_inertial_odometry.md](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/docs/en/computer_vision/visual_inertial_odometry.md)
- The VIO guide also suggests a `HEARTBEAT` from component id 197 (`MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY`). In v1.17, PX4 only records that heartbeat's time; no use of it in a health check was found. — [R117 VIO doc](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/docs/en/computer_vision/visual_inertial_odometry.md); [R117 mavlink_receiver.cpp](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/modules/mavlink/mavlink_receiver.cpp)
- Auto modes (Mission, Return, Land, Hold, Orbit) "require a global position". A local-only system can send `SET_GPS_GLOBAL_ORIGIN` to set the EKF origin. PX4 v1.17 handles that message and forwards it to EKF2 as `VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN`. — [External Position Estimation](https://docs.px4.io/main/en/ros/external_position_estimation.html); [R117 EKF2.cpp](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/modules/ekf2/EKF2.cpp)
- **ModalAI's v1.18 "indoor_vio" preset (a vendor reference for VIO-only indoor flight):**
  - `EKF2_MAG_TYPE 5`, `EKF2_BARO_CTRL 1`, `EKF2_EV_CTRL 15`, `EKF2_OF_CTRL 0`.
  - `EKF2_RNG_CTRL 1` with `EKF2_RNG_DELAY 50` ms ("we send this over mavlink and PX4 ignores our correct timestamp").
  - `EKF2_HGT_REF 3`.
  - `EKF2_GPS_CTRL 0` ("has to be set after EKF2_HGT_REF. Otherwise PX4 will automatically reset EKF2_GPS_CTRL to 7").
  - `EKF2_EV_NOISE_MD 0`, EVP/EVV/EVA noise 0.1, `EKF2_EVP_GATE 5`, `EKF2_EVV_GATE 3`.
  - `EKF2_EV_DELAY 0` ("VOXL will do proper timesyncing").
  - `COM_ARM_WO_GPS 1`.
  - `EKF2_EV_QMIN` set positive, with the comment that quality 1 means QVIO is temporarily dead-reckoning and -1 means failure.
  - Source: [voxl-px4-params indoor_vio.params (v1.18)](https://gitlab.com/voxl-public/voxl-sdk/utilities/voxl-px4-params/-/blob/master/params/v1.18/EKF2_helpers/indoor_vio.params)
- **MAVSDK `Mocap.set_odometry` gotcha (MAVSDK main):**
  - It sends ODOMETRY with `frame_id` set to `MAV_FRAME_LOCAL_FRD` if `LocalFrd` is chosen, otherwise `MAV_FRAME_RESERVED_14` (MOCAP_NED).
  - `child_frame_id` is always `MAV_FRAME_BODY_FRD`.
  - If `estimator_type` is left Unknown it defaults to `MAV_ESTIMATOR_TYPE_MOCAP` "for backward compatibility".
  - Source: [MAVSDK mocap_impl.cpp](https://github.com/mavlink/MAVSDK/blob/main/cpp/src/mavsdk/plugins/mocap/mocap_impl.cpp)

### Inferences
- **The MAVSDK default is a silent failure.** If you use MAVSDK's Mocap plugin with default settings, PX4 v1.17 routes your VIO to `vehicle_mocap_odometry`, which EKF2 never fuses. Picking the MOCAP_NED frame would also drop the position, because PX4 does not accept frame 14 for position. Always set `frame_id = LocalFrd` and `estimator_type = Vio` (or Vision). Check after the first connection that `estimator_status_flags` / `vehicle_local_position` show EV fusion. If your installed MAVSDK version has no `estimator_type` field in the Odometry struct, build ODOMETRY yourself, for example with pymavlink.
- **Recommended EV configuration for this platform:**
  - `EKF2_EV_CTRL = 15`
  - `EKF2_HGT_REF = 3` (or 2 if a downward rangefinder over a flat floor is added; see section 4)
  - `EKF2_GPS_CTRL = 0`, set after HGT_REF
  - `EKF2_MAG_TYPE = 5` (indoor magnetic fields near steel and motor currents are unreliable)
  - `EKF2_BARO_CTRL = 1` as a secondary height source
  - `EKF2_EV_NOISE_MD = 0` with floors around `EKF2_EVP_NOISE` 0.05–0.1 m
  - `EKF2_EV_POS_X/Y/Z` = the OAK-D left-camera optical centre in body FRD (cuVSLAM's pose origin)
  - Reboot after changing them.
- **Frames:**
  - cuVSLAM and ROS conventions are typically FLU body / ENU-like world. PX4 accepts `LOCAL_FLU`/`LOCAL_ENU` for position, but only FRD/NED/body-FRD for velocity. The simplest robust approach is to convert everything in Python to world FRD (x forward at start, z down) and body FRD before sending.
  - Rotate the camera-optical-frame pose into the body frame using the OAK-D mount extrinsics.
  - Send body-frame velocity with `child_frame_id = BODY_FRD`.
- **Rate and dropouts:** the 30 Hz cuVSLAM output is at the low end of the documented 30–50 Hz range. Do not decimate it further. If the guard skips frames, keep gaps well under 200 ms (see section 3).
- **Covariance:** pass cuVSLAM's covariance through, but scale it so the innovation test ratios in the log sit comfortably below 1. VIO covariances are often optimistic, and the noise floors protect you from that.

### Gaps
- No PX4 documentation explains whether `VehicleOdometry.timestamp_sample` sent over uXRCE-DDS must already be in PX4's time base. Agent–client time sync exists (`UXRCE_DDS_SYNCT`), but the exact timestamp translation in the client was not verified.
- No PX4 source states how realistic cuVSLAM's covariance is. That needs log-based validation.
- The integer values of `MAV_ESTIMATOR_TYPE_VIO` and `MAV_FRAME_LOCAL_FRD` were not re-verified here. Use the named constants from pymavlink/MAVSDK.

## 3. VIO resets, quality, dropouts, timestamps and EV-delay tuning

### Takeaway
EKF2 v1.17 watches `reset_counter`. When it changes (with sufficient quality), EKF2 snaps its horizontal position to the new EV value instead of fighting a jump. It uses `quality` against `EKF2_EV_QMIN` to gate both starting and continuing fusion. It declares "vision data stopped" after 400 ms without samples.

Timestamps only work as intended if MAVLink TIMESYNC has converged. Otherwise PX4 stamps EV with its arrival time, and `EKF2_EV_DELAY` must absorb the whole camera → VIO → link latency.

### Cited Findings
- **Reset and quality logic (`ev_control.cpp`, v1.17):**
  - `ev_reset = (ev_sample.reset_counter != _ev_sample_prev.reset_counter)`.
  - `quality_sufficient = (EKF2_EV_QMIN <= 0) || (quality >= EKF2_EV_QMIN)`.
  - Starting conditions also need consecutive samples less than `EV_MAX_INTERVAL` apart.
  - If no sample arrives for `2 * EV_MAX_INTERVAL`, EV fusion stops with "vision data stopped".
  - The EV-to-EKF rotation filter is re-initialised on an EV reset.
  - Source: [R117 ev_control.cpp](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/modules/ekf2/EKF/aid_sources/external_vision/ev_control.cpp)
- `EV_MAX_INTERVAL = 200e3` µs ("Maximum allowable time interval between external vision system measurements"). `no_aid_timeout_max = 1'000'000` µs. — [R117 EKF/common.h](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/modules/ekf2/EKF/common.h)
- **Behaviour on an EV reset (no GNSS):**
  - With sufficient quality, EKF2 logs "reset to EV" and calls `resetHorizontalPositionTo(measurement)`.
  - Without sufficient quality: "EV has reset, but quality isn't sufficient — we have no choice but to stop EV and try to resume once quality is acceptable."
  - If fusion fails for more than 1 s, EKF2 resets to EV (if resets remain and quality is sufficient), otherwise it stops EV fusion.
  - EV velocity and EV height have equivalent reset paths.
  - Sources: [R117 ev_pos_control.cpp](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/modules/ekf2/EKF/aid_sources/external_vision/ev_pos_control.cpp); [R117 ev_vel_control.cpp](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/modules/ekf2/EKF/aid_sources/external_vision/ev_vel_control.cpp); [R117 ev_height_control.cpp](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/modules/ekf2/EKF/aid_sources/external_vision/ev_height_control.cpp)
- `EKF2_EV_QMIN`: "EV is only started and fused when its quality metric exceeds this threshold". Optional, default 0. — [params_external_vision.yaml](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/ekf2/params_external_vision.yaml)
- **Doc/code inconsistency:** the `VehicleOdometry.msg` comment calls `quality` "Unused", but EKF2 copies `ev_odom.quality` and gates fusion on it. — [R117 VehicleOdometry.msg](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/msg/versioned/VehicleOdometry.msg); [R117 EKF2.cpp](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/modules/ekf2/EKF2.cpp)
- **`EKF2_EV_DELAY` (docs):**
  - It is "the difference between the vision system timestamp and the 'actual' capture time that would have been recorded by the IMU clock".
  - It can be 0 with correct timestamping and timesync, but usually needs empirical tuning.
  - Estimate it from the offset between IMU and EV data in logs, then fine-tune for the lowest innovations during dynamic manoeuvres.
  - Enable `SDLOG_PROFILE` bit 7 ("Computer Vision and Avoidance") to log EV data.
  - Sources: [R117 VIO doc](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/docs/en/computer_vision/visual_inertial_odometry.md); [External Position Estimation](https://docs.px4.io/main/en/ros/external_position_estimation.html)
- `EKF2_DELAY_MAX`: default 200 ms, "should be at least as large as the largest EKF2_XXX_DELAY parameter", reboot required. — [R117 ekf2 module.yaml](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/modules/ekf2/module.yaml)
- **PX4 MAVLink timesync:**
  - `sync_stamp()` returns `usec + offset` only once synchronisation has converged; otherwise it returns `hrt_absolute_time()`, i.e. the arrival time.
  - Convergence requires `CONVERGENCE_WINDOW = 500` samples. Samples with RTT above `MAX_RTT_SAMPLE = 10 ms` are not used.
  - Sources: [R117 Timesync.cpp](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/lib/timesync/Timesync.cpp); [R117 Timesync.hpp](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/lib/timesync/Timesync.hpp)
- In "Onboard" MAVLink mode, PX4 streams `TIMESYNC` at 10 Hz. — [R117 mavlink_main.cpp](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/modules/mavlink/mavlink_main.cpp)
- MAVSDK's core implements TIMESYNC: it replies to `tc1 == 0` requests and computes the autopilot clock offset, which it uses to stamp outgoing messages such as odometry. — [MAVSDK timesync.cpp](https://github.com/mavlink/MAVSDK/blob/main/cpp/src/mavsdk/core/timesync.cpp); [MAVSDK mocap_impl.cpp](https://github.com/mavlink/MAVSDK/blob/main/cpp/src/mavsdk/plugins/mocap/mocap_impl.cpp)
- For DISTANCE_SENSOR over MAVLink, PX4 sets the timestamp to its own receive time ("not trusted" sender time). ModalAI compensates with `EKF2_RNG_DELAY 50`. — [R117 mavlink_receiver.cpp](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/modules/mavlink/mavlink_receiver.cpp); [voxl indoor_vio.params](https://gitlab.com/voxl-public/voxl-sdk/utilities/voxl-px4-params/-/blob/master/params/v1.18/EKF2_helpers/indoor_vio.params)

### Inferences
- **Map the cuVSLAM guard onto PX4 semantics:**
  - Increment `reset_counter` (mod 256) only when cuVSLAM re-initialises or relocalises, or when the guard re-seeds the pose discontinuously. Never increment it for an ordinary skipped frame.
  - Set `quality` to 100 (or scale it with the tracking score) when tracking well, a low value while coasting on the gyro, and -1 when lost.
  - Set `EKF2_EV_QMIN` just above the coasting value. Coasted samples then stop EV fusion, and EKF2 dead-reckons on its own IMU instead of fusing extrapolated poses as if they were measurements.
  - If you prefer to keep fusing while coasting, inflate the covariance instead.
- **Dropout budget:** PX4 tolerates gaps of up to about 200 ms between samples for start-up conditions, and stops EV after 400 ms of silence. After that, local position stays valid until `EKF2_NOAID_TOUT` (default 5 s), but drift grows quadratically with accelerometer bias. Treat any gap over about 0.5 s as an emergency for the mission logic (hold or land), not something the EKF will ride through gracefully.
- **Timestamps:**
  - Use the OAK-D capture timestamp (DepthAI device-to-host synced), converted to the companion's monotonic clock, as `time_usec`.
  - With MAVSDK, timesync is handled for you.
  - With raw pymavlink, you must answer PX4's TIMESYNC requests (reply with `tc1` = your time in ns, `ts1` echoed) at low RTT. Otherwise PX4 uses arrival time, and `EKF2_EV_DELAY` must be set to the full capture-to-arrival latency (probably tens of ms; measure it).
  - At 10 Hz, 500 samples take about 50 s, so expect timesync to converge roughly a minute after the link comes up. Wait before arming.
- **Tuning `EKF2_EV_DELAY`:** log with `SDLOG_PROFILE` bit 7, cross-correlate EV velocity with IMU-integrated or EKF velocity during sharp roll/pitch moves (props on, tethered or hovering), then step `EKF2_EV_DELAY` in 5–10 ms increments to minimise `estimator_aid_src_ev_pos` / `ev_vel` innovations. Raise `EKF2_DELAY_MAX` if the delay exceeds 200 ms.

### Gaps
- No published end-to-end latency figures for PyCuVSLAM on an Orin Nano with an OAK-D S2 were found. It must be measured.
- The PX4 docs do not describe recommended `quality` semantics for VIO producers beyond the MAVLink definition.

## 4. Should a downward rangefinder or optical flow be added, and how does EKF2 fuse multiple sources?

### Takeaway
Yes. A downward rangefinder is cheap insurance for height, and is required for optical flow. Optical flow adds an independent velocity aid that keeps PX4's local position valid if VIO drops briefly. EKF2 fuses every enabled source simultaneously, each with its own innovation gate. Height is the exception: it converges to the one `EKF2_HGT_REF` source while the others are bias-corrected towards it.

### Cited Findings
- Optical flow is used only if "Valid range finder data is available", `EKF2_OF_CTRL` is set, and flow quality exceeds `EKF2_OF_QMIN`. — [R117 EKF2 tuning guide](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/docs/en/advanced_config/tuning_the_ecl_ekf.md)
- `EKF2_OF_CTRL` defaults to 1 (enabled) from v1.16 (PR #23436). `EKF2_OF_DELAY` defaults to 20 ms. — [R117 params_optical_flow.yaml](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/modules/ekf2/params_optical_flow.yaml); [v1.16 notes](https://docs.px4.io/main/en/releases/1.16.html)
- `EKF2_RNG_CTRL` modes:
  - 0 disabled.
  - 1 conditional (default): used only below `EKF2_RNG_A_VMAX` speed and `EKF2_RNG_A_HMAX` height, e.g. takeoff and landing.
  - 2 always enabled, intended for near-flat surfaces such as indoors.
  - `EKF2_RNG_DELAY` defaults to 5 ms.
  - The parameter text recommends `MPC_ALT_MODE` for terrain following rather than range height fusion.
  - Source: [R117 params_range_finder.yaml](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/modules/ekf2/params_range_finder.yaml)
- Typical height configurations (docs):
  - Indoor, non-flat terrain: GPS_CTRL 0, BARO_CTRL 1, RNG_CTRL 1, HGT_REF 2.
  - Indoor, flat terrain: RNG_CTRL 2, HGT_REF 2.
  - External vision: HGT_REF 3, other sources "as required".
  - "Over the long term the height estimate follows the 'reference source'."
  - Source: [R117 EKF2 tuning guide](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/docs/en/advanced_config/tuning_the_ecl_ekf.md)
- If rangefinder data stays inconsistent for 5 s and range is the active height source, the estimator resets height to it. If other height sources are active, the rangefinder is declared faulty. There is also a kinematic obstruction check. — [R117 EKF2 tuning guide](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/docs/en/advanced_config/tuning_the_ecl_ekf.md)
- The position-loss trigger is "a timeout since the last sensor data was fused that provides direct speed or horizontal position measurements". The sensors in that category are "GNSS, optical flow, airspeed, VIO, auxiliary global position". — [R117 safety.md](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/docs/en/config/safety.md)
- `EKF2_NOAID_TOUT` defaults to 5,000,000 µs (range 0.5–10 s): the "maximum inertial dead-reckoning time" before the horizontal nav solution is reported invalid. — [R117 ekf2 module.yaml](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/modules/ekf2/module.yaml)
- Collision Prevention ignores distance sensors with `ROTATION_DOWNWARD_FACING` or `ROTATION_UPWARD_FACING`, so a downward rangefinder does not interfere with it. — [R117 CollisionPrevention.cpp](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/lib/collision_prevention/CollisionPrevention.cpp)
- A comment in the arming check says a missing GNSS should "only warn and don't block arming because there could still be a valid position estimate from another source e.g. optical flow, VIO". — [R117 estimatorCheck.cpp](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/modules/commander/HealthAndArmingChecks/checks/estimatorCheck.cpp)
- v1.18 (beta) lets fusion sources be toggled at runtime (`VEHICLE_CMD_ESTIMATOR_SENSOR_ENABLE`, `EKF2_SENS_EN`). — [v1.18 notes](https://docs.px4.io/main/en/releases/1.18.html)
- The Gazebo model list includes a flow-equipped x500 airframe (`4021_gz_x500_flow`). — [R117 init.d-posix airframes](https://github.com/PX4/PX4-Autopilot/tree/release/1.17/ROMFS/px4fmu_common/init.d-posix/airframes)

### Inferences
- **Recommended additions:**
  - A downward 1D lidar (e.g. a small ToF or LiDAR on I2C/UART) as `EKF2_RNG_CTRL=1` (conditional) for takeoff and landing, or `=2` with `EKF2_HGT_REF=2` if the course floor is flat. Obstacles on the floor would make range-referenced height jump.
  - Optionally an optical-flow sensor (PMW3901/PAW3902 class) with `EKF2_OF_CTRL=1`, which then gives EKF2 a second velocity aid.
  - Because flow and VIO are fused together, a VIO dropout no longer starts the 5 s dead-reckoning clock. That is the main benefit.
- **If you do not fit a flow sensor, set `EKF2_OF_CTRL=0` explicitly**, because it defaults to enabled since v1.16. The same applies to the PX4 default `EKF2_RNG_CTRL=1` if no rangefinder is fitted.
- **Risks of fusing VIO and flow together:**
  - Inconsistent scale or frames cause innovation rejections and EKF resets.
  - Low-texture or netted floors give poor flow quality.
  - Tune the `EKF2_OF_QMIN` and `EKF2_EVV_NOISE` floors.
  - With `EKF2_MAG_TYPE=5`, heading comes only from EV yaw. While VIO is down, yaw is gyro-integrated and flow still helps velocity, but heading drift is unbounded until VIO returns.

### Gaps
- No PX4 documentation gives explicit arbitration rules between EV and flow beyond per-source gating. Behaviour under disagreement must be tested in logs or simulation (the gz_x500_flow and x500_vision models).

## 5. PX4 Collision Prevention: how it works, parameters, message semantics, and which modes it supports

### Takeaway
Collision Prevention (CP) is a velocity/acceleration limiter that **only acts on stick-generated acceleration setpoints**. That means Position mode with `MPC_POS_MODE=4` (Acceleration based), Position Slow mode, and nudging during Auto landing. It does not modify Offboard setpoints or Mission waypoints in v1.17.

CP fuses `OBSTACLE_DISTANCE` (and non-vertical `DISTANCE_SENSOR`) data into a 72-bin, 5°-per-bin body-frame map. It limits speed towards each bin so the vehicle can stop at `max(CP_DIST, min_distance)`. It blocks motion into bins with no data unless `CP_GO_NO_DATA=1`.

### Cited Findings
- **Docs, purpose and modes:**
  - CP slows and stops a multicopter before an obstacle.
  - "It can be enabled for multicopter vehicles in Position mode (with MPC_POS_MODE set to Acceleration based)."
  - It accepts data from a companion computer, MAVLink rangefinders, FC-attached rangefinders, or a combination.
  - It "may restrict vehicle maximum speed if the sensor range isn't large enough".
  - QGC notifies the pilot when CP is active.
  - Source: [Collision Prevention](https://docs.px4.io/main/en/computer_vision/collision_prevention.html); [R117 collision_prevention.md](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/docs/en/computer_vision/collision_prevention.md)
- **Code: where CP runs (v1.17):**
  - `_collision_prevention.modifySetpoint(_acceleration_setpoint, _velocity_setpoint)` is called only in `StickAccelerationXY.cpp` (when `is_active()`, i.e. `CP_DIST > 0`).
  - `StickAccelerationXY` is used by `FlightTaskManualAcceleration` (Position mode), `FlightTaskManualAccelerationSlow` (Position Slow) and `FlightTaskAuto`. In Auto it is used only to generate nudging setpoints around the land position.
  - `FlightTaskManualPosition` still declares a CP member, but that task is the legacy path.
  - Sources: [R117 StickAccelerationXY.cpp](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/modules/flight_mode_manager/tasks/Utility/StickAccelerationXY.cpp); [R117 FlightTaskAuto.cpp](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/modules/flight_mode_manager/tasks/Auto/FlightTaskAuto.cpp)
- `modifySetpoint` only intervenes when the stick is deflected (`setpoint_length > 0.001`). It shapes the stick-derived acceleration setpoint, using `CP_DELAY`, `MPC_JERK_MAX`, `MPC_ACC_HOR`, `MPC_XY_VEL_P_ACC`, `MPC_XY_P` and `MPC_VEL_MANUAL`. — [R117 CollisionPrevention.cpp](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/lib/collision_prevention/CollisionPrevention.cpp); [R117 CollisionPrevention.hpp](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/lib/collision_prevention/CollisionPrevention.hpp)
- A PX4 forum user asking about CP in Offboard concluded from the docs that CP "only works in position mode". No contrary answer was found. — [PX4 discuss: collision prevention whilst in offboard](https://discuss.px4.io/t/using-collision-prevention-whilst-in-offboard-mode/20983)
- **Parameters (v1.17 defaults):**
  - `CP_DIST`: -1.0 m (disabled; range -1 to 15). The minimum distance, "measured to the sensors, not the vehicle's outer edge".
  - `CP_DELAY`: 0.4 s (0–1). The docs say tracking delay is typically 0.1–0.5 s and external vision may add about 0.2 s.
  - `CP_GUIDE_ANG`: 30° (0–90). 0 disables guidance; 30° is a "good starting balance".
  - `CP_GO_NO_DATA`: 0.
  - `MPC_POS_MODE`: 4 (Acceleration based) by default. The only other value is 0 (Direct velocity).
  - Sources: [R117 collisionprevention_params.c](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/lib/collision_prevention/collisionprevention_params.c); [Collision Prevention docs](https://docs.px4.io/main/en/computer_vision/collision_prevention.html); [R117 multicopter_position_mode_params.c](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/modules/mc_pos_control/multicopter_position_mode_params.c)
- **Internal map and timeouts:**
  - `BIN_COUNT` = 72 (the size of `obstacle_distance_s::distances`) and `BIN_SIZE = 360/72 = 5°`.
  - Per-bin data older than `RANGE_STREAM_TIMEOUT_US = 500 ms` reverts to `UINT16_MAX` (no data).
  - With no obstacle data, the stick acceleration is zeroed and "No obstacle data, not moving..." is printed.
  - If the data is older than `TIMEOUT_HOLD_US = 5 s`, CP sends `VEHICLE_CMD_DO_SET_MODE` to Auto Loiter (Hold).
  - The docs describe the same: "No range data received, no movement allowed" after 0.5 s, then HOLD after 5 s. To recover, set `CP_DIST` negative or switch to Altitude or Stabilized.
  - Sources: [R117 CollisionPrevention.cpp](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/lib/collision_prevention/CollisionPrevention.cpp); [Collision Prevention docs](https://docs.px4.io/main/en/computer_vision/collision_prevention.html)
- **Velocity constraint per bin:**
  - Only bins with `min_distance < d < UINT16_MAX` are considered.
  - `stop_distance = d − min_dist_to_keep − v∥·(CP_DELAY + data_age)`.
  - `max_vel = computeMaxSpeedFromDistance(MPC_JERK_MAX, MPC_ACC_HOR, stop_distance, 0)`.
  - `min_dist_to_keep = max(min_distance_from_message/100, CP_DIST)`.
  - Source: [R117 CollisionPrevention.cpp](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/lib/collision_prevention/CollisionPrevention.cpp)
- `_checkSetpointDirectionFeasability()` rejects a setpoint whose bin has `UINT16_MAX` unless `CP_GO_NO_DATA=1`. Even with `CP_GO_NO_DATA=1`, the setpoint is still rejected if that bin is inside a sensor's field of view (`_data_fov`). — [R117 CollisionPrevention.cpp](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/lib/collision_prevention/CollisionPrevention.cpp)
- **Frames accepted by CP:**
  - `MAV_FRAME_GLOBAL` (0) or `MAV_FRAME_LOCAL_NED` (1): north-aligned, rotated by the vehicle yaw.
  - `MAV_FRAME_BODY_FRD` (12): front-aligned.
  - Anything else logs "Obstacle message received in unsupported frame".
  - A code comment warns: "TODO this gives false output if the offset is not a multiple of the resolution."
  - Bin lower bound = `bin*width + angle_offset − width/2`.
  - Sources: [R117 CollisionPrevention.cpp](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/lib/collision_prevention/CollisionPrevention.cpp); [R117 ObstacleMath.cpp](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/lib/collision_prevention/ObstacleMath.cpp); [R117 ObstacleDistance.msg](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/msg/ObstacleDistance.msg)
- **MAVLink `OBSTACLE_DISTANCE` (#330):**
  - `time_usec`, `sensor_type` (`MAV_DISTANCE_SENSOR`).
  - `distances` uint16[72] in **cm**: "0 means touching the sensor, max_distance+1 means no obstacle, UINT16_MAX unknown/unused". Index 0 is "north + angle_offset", or forward in the body frame.
  - `increment` (uint8, deg, clockwise; ignored if `increment_f` is non-zero).
  - `min_distance` and `max_distance` (cm).
  - Extensions: `increment_f` (float deg), `angle_offset` (float deg; 0 = forward, positive = clockwise) and `frame` (defaults to north-aligned global; body-mounted sensors should use `MAV_FRAME_BODY_FRD`).
  - Source: [MAVLink common: OBSTACLE_DISTANCE](https://mavlink.io/en/messages/common.html#OBSTACLE_DISTANCE)
- **PX4 handling of OBSTACLE_DISTANCE (v1.17):**
  - The uORB timestamp is PX4's receive time (`hrt_absolute_time()`); the sender `time_usec` is ignored.
  - `distances`, `min_distance`, `max_distance`, `angle_offset` and `frame` are copied.
  - `increment` uses `increment_f` if it is greater than 0, else the integer `increment`.
  - `frame` is not validated at receive time.
  - **PX4 has no handler for `OBSTACLE_DISTANCE_3D`** (it is not in the `handle_message` switch).
  - Source: [R117 mavlink_receiver.cpp](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/modules/mavlink/mavlink_receiver.cpp)
- **ROS 2 alternative:** publish px4_msgs `ObstacleDistance` on `/fmu/in/obstacle_distance` with `frame`, `sensor_type`, `distances[72]` (cm), `increment` (float32 deg), `min_distance`/`max_distance` (cm) and `angle_offset`. `obstacle_distance_fused` is the internal map. — [R117 ObstacleDistance.msg](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/msg/ObstacleDistance.msg); [R117 dds_topics.yaml](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/modules/uxrce_dds_client/dds_topics.yaml)
- **Companion status in the docs:**
  - "The companion implementation/setup is currently untested (the original companion project was unmaintained and has been archived)."
  - The required rate depends on speed: "Initial testing … used a vehicle moving at 4 m/s with OBSTACLE_DISTANCE messages being emitted at 10Hz."
  - For debugging, add `/fmu/out/obstacle_distance_fused` to `dds_topics.yaml` and use the PlotJuggler Lua script.
  - Simulation: `make px4_sitl gz_x500_lidar_2d`.
  - Source: [R117 collision_prevention.md](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/docs/en/computer_vision/collision_prevention.md)
- **Supported rangefinders:**
  - LightWare SF45 rotating lidar from v1.14 (320°). It no longer auto-starts on TELEM2 from v1.16.
  - Sony AS-DT1, publishing about a 35° forward FOV in 5° bins.
  - Other rangefinders need `h_fov` set in their driver.
  - Sources: [Collision Prevention docs](https://docs.px4.io/main/en/computer_vision/collision_prevention.html); [v1.16 notes](https://docs.px4.io/main/en/releases/1.16.html)
- ModalAI forum posts from 2023 (PX4 v1.12-era) say CP needed `COM_OBS_AVOID` and simple/smooth `MPC_POS_MODE`. That conflicts with current PX4, where `COM_OBS_AVOID` no longer exists in v1.17 and CP requires `MPC_POS_MODE=4`. — [ModalAI forum thread](https://forum.modalai.com/topic/2834/avoidance-collision-prevention-disabled?page=3); [R117 commander_params.c](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/modules/commander/commander_params.c) (no `COM_OBS_AVOID`)

### Inferences
- **For an Offboard-driven autonomous run, PX4 CP will not protect you.** The companion planner must implement its own stopping and avoidance logic. Clamp velocity commands using the same braking model (section 12). CP is still valuable as a safety net for manual Position-mode flights and pilot takeover: if the safety pilot flips to Position mode near the netting, CP limits speed towards walls the camera can see.
- **Encoding a ~70–80° forward HFOV for PX4 CP.** Use `frame = MAV_FRAME_BODY_FRD` and `increment_f = 5.0` (matching PX4's 5° bins, avoiding the offset/resolution TODO bug). Fill about 15 sectors from `angle_offset = −35°` (centred on bins at −35…+35°), and set every other element to `UINT16_MAX`. A finer `increment_f` (e.g. 1°) is legal MAVLink, but PX4 resamples it into 5° bins anyway.
- **Distance encoding:** per sector, use the minimum valid depth (cm) within a horizontal band of the depth image (vehicle-height band, pitch-compensated). Use `max_distance + 1` for "no obstacle within range", and `UINT16_MAX` for invalid or unknown pixels. Keep `min_distance` small and realistic (OAK-D S2 minimum depth, around 20–35 cm). A large `min_distance` silently raises the keep-out distance and excludes close bins from the velocity constraint.
- **Rate:** stream at ≥10 Hz, preferably 15–20 Hz. ArduPilot recommends ≥10 Hz and ModalAI defaults to 20 Hz. Any gap over 500 ms blanks the bins and halts CP-governed motion.

### Gaps
- No source confirmed whether any PX4 release before v1.16 applied CP to Offboard velocity setpoints. Historical behaviour was not checked beyond v1.17.
- No official PX4 example companion code for OBSTACLE_DISTANCE from a depth camera exists today. The original one, PX4-Avoidance's local_planner, is archived.

## 6. Single forward camera: limitations and existing companion examples that publish obstacle data

### Takeaway
With only a forward ~70–80° camera and the default `CP_GO_NO_DATA=0`, PX4 CP blocks every Position-mode stick command into the about 290° of bins with no data. That means no sideways, backwards or diagonal-out-of-FOV movement. Setting `CP_GO_NO_DATA=1` allows those moves but leaves them unprotected. The usual depth-to-OBSTACLE_DISTANCE scripts (ArduPilot `d4xx_to_mavlink.py`, ModalAI `voxl-vision-hub` VOA) take a horizontal band of the depth image, compensate for pitch, and send 72 entries at 10–20 Hz.

### Cited Findings
- Docs: "Motion is blocked in directions with no sensor data", so without rear data the vehicle cannot fly backwards. `CP_GO_NO_DATA=1` allows movement into uncovered directions. If one of several sensors fails, its coverage is treated as uncovered. — [Collision Prevention docs](https://docs.px4.io/main/en/computer_vision/collision_prevention.html)
- Code: with `CP_GO_NO_DATA=1`, motion is still blocked in bins inside a sensor FOV that currently lack data (`_data_fov`). — [R117 CollisionPrevention.cpp](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/lib/collision_prevention/CollisionPrevention.cpp)
- **ArduPilot `d4xx_to_mavlink.py` (RealSense, non-ROS Python):**
  - Sends `OBSTACLE_DISTANCE`, "up to 72 distances at once".
  - Divides the horizontal FOV into 72 evenly spaced rays along one horizontal "obstacle line" (`obstacle_line_height_ratio`).
  - Takes the minimum over `obstacle_line_thickness_pixel`.
  - Applies RealSense post-processing filters and pitch compensation from MAVLink `ATTITUDE`.
  - Sends at ≥10 Hz (about 15 Hz expected) with a forward-facing camera only.
  - ArduPilot side: `PRX1_TYPE=2`, `AVOID_ENABLE=7`, `AVOID_MARGIN=1.5`, `AVOID_BEHAVE` 0 = slide / 1 = stop.
  - Sources: [ArduPilot RealSense depth camera](https://ardupilot.org/copter/docs/common-realsense-depth-camera.html); [d4xx_to_mavlink.py](https://github.com/thien94/vision_to_mavros/blob/master/scripts/d4xx_to_mavlink.py)
- **ModalAI VOA (`voxl-vision-hub`):**
  - Converts the stereo DFS point cloud to `OBSTACLE_DISTANCE` for PX4 and ArduPilot at `voa_send_rate_hz` (default 20).
  - Uses VIO history to motion-compensate depth, or PX4 attitude if VIO is off.
  - Suggested indoor test parameters: `CP_DIST 1.5`, `CP_DELAY 0` (it does its own delay compensation; ">0 causes twitchy and unpredictable behavior"), `CP_GUIDE_ANG 0`, `CP_GO_NO_DATA 1`.
  - Source: [ModalAI VOA docs](https://docs.modalai.com/voa/)
- A ModalAI forum user with accurate distances and `CP_DIST 1.5` reported the drone did not stop. The moderator pointed to `MPC_POS_MODE` per the PX4 docs. — [ModalAI forum: collision prevention](https://forum.modalai.com/topic/2671/collision-prevention)
- The original PX4 companion pipeline (PX4-Avoidance) is archived on GitHub; its last push was 2024-04-10. — [PX4/PX4-Avoidance](https://github.com/PX4/PX4-Avoidance)

### Inferences
- **For a forward-only camera:**
  - (a) Fly the autonomous course in Offboard, with the companion planner treating unseen directions as unknown. Only command motion within, or close to, the camera FOV, and yaw to look before translating sideways.
  - (b) For manual Position-mode safety, set `CP_GO_NO_DATA=1`. Otherwise the pilot cannot back away from the net. Accept that rear and side motion are unprotected.
  - (c) Keep `CP_GUIDE_ANG` modest (0–30°). Guidance can slide the vehicle into unseen sectors.
- **Netting is a hard target for stereo depth.** It is thin and low-texture, and it shows the background through it. Treat the course boundary as a geofence-like virtual wall in the companion map as well (see the separate netting notes in this folder).
- If you send the ModalAI-style `CP_DELAY=0`, timestamps must truly be compensated. PX4 stamps `OBSTACLE_DISTANCE` with its receive time, so camera-to-PX4 latency is otherwise invisible to CP. Use `CP_DELAY` ≈ depth pipeline latency + link latency (about 0.1–0.2 s) unless you motion-compensate on the companion.

### Gaps
- No maintained, PX4-specific OAK-D → OBSTACLE_DISTANCE script was found. The Luxonis/DepthAI examples found were not PX4-specific.

## 7. Offboard control: stream rate, setpoints, arming and mode switching, loss failsafes

### Takeaway
PX4 needs a ≥2 Hz proof-of-life stream (MAVLink setpoints, or the ROS 2 `OffboardControlMode` heartbeat). The stream must run for more than 1 s before you switch to Offboard or arm in it. If it stops for `COM_OF_LOSS_T` (default 1.0 s), PX4 runs `COM_OBL_RC_ACT` (default 0 = Position mode). If RC is also lost, it falls back to `NAV_RCL_ACT` (default 2 = Return), which is a poor choice indoors without GPS.

Use `SET_POSITION_TARGET_LOCAL_NED` in `MAV_FRAME_LOCAL_NED` or `MAV_FRAME_BODY_NED`, or `TrajectorySetpoint` (NED) with NaN-masking.

### Cited Findings
- **Proof of life and arming (docs):**
  - "PX4 requires that the external controller provides a continuous 2Hz 'proof of life' signal". PX4 "enables offboard control only after receiving the signal for more than a second".
  - The stream must be running "before arming in offboard mode or switching to offboard mode when flying".
  - RC is disabled except for mode changes. `COM_RC_IN_MODE=4` allows flying with no manual controller.
  - If the rate falls below 2 Hz, PX4 leaves Offboard after `COM_OF_LOSS_T`.
  - Source: [R117 offboard.md](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/docs/en/flight_modes/offboard.md)
- **Commander parameters (v1.17):**
  - `COM_OF_LOSS_T` default **1.0 s** (0–60).
  - `COM_OBL_RC_ACT` default **0**: 0 Position, 1 Altitude, 2 Stabilized, 3 Return, 4 Land, 5 Hold, 6 Terminate, 7 Disarm.
  - `NAV_RCL_ACT` default **2** (Return): 1 Hold, 2 Return, 3 Land, 5 Terminate, 6 Disarm.
  - `COM_RC_LOSS_T` 0.5 s.
  - `COM_RC_IN_MODE` default **3** ("RC or MAVLink keep first"); 4 = disable manual control.
  - `COM_FAIL_ACT_T` 5 s (Hold before the failsafe action).
  - `COM_RC_OVERRIDE` default 1 (renamed or replaced in v1.18).
  - Source: [R117 commander_params.c](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/modules/commander/commander_params.c)
- Failsafe code: on `offboard_control_signal_lost` the action comes from `COM_OBL_RC_ACT`. For the Position/Altitude/Stabilized fallbacks, "Check if RC is available, if not use the mode specified in NAV_RCL_ACT". Position falls back to Altitude, then Stabilized, if the mode cannot run. — [R117 failsafe.cpp](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/modules/commander/failsafe/failsafe.cpp)
- `COM_RCL_EXCEPT` bit 2 ignores RC loss while in Offboard. — [R117 offboard.md](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/docs/en/flight_modes/offboard.md)
- v1.17: "Offboard to Position (hold) mode change is no longer entered without RC" (PR #26391). — [v1.17 notes](https://docs.px4.io/main/en/releases/1.17.html)
- **MAVLink copter setpoints:**
  - `SET_POSITION_TARGET_LOCAL_NED` supports position, velocity, acceleration, or position with velocity/acceleration feed-forward.
  - Frames are `MAV_FRAME_LOCAL_NED` and `MAV_FRAME_BODY_NED`; body NED supports only velocity and acceleration.
  - `SET_POSITION_TARGET_GLOBAL_INT` and `SET_ATTITUDE_TARGET` are also accepted. An attitude message with zero or non-finite q is ignored and does not count as proof of life.
  - Source: [Offboard docs](https://docs.px4.io/main/en/flight_modes/offboard.html)
- **PX4 v1.17 handler for `SET_POSITION_TARGET_LOCAL_NED`:**
  - It only runs if `MAV_FWDEXTSP` is enabled (default 1) and the target system/component matches (or is 0).
  - LOCAL_NED fields are used directly, with type_mask "ignore" bits mapped to NaN.
  - BODY_NED: velocity is rotated by yaw only, acceleration by the full attitude, and position is set to NaN.
  - **Any other frame (including LOCAL_OFFSET_NED / BODY_OFFSET_NED) is rejected.**
  - FORCE_SET with acceleration is rejected.
  - The trajectory setpoint is published only if `vehicle_status.accepts_offboard_setpoints`.
  - Sources: [R117 mavlink_receiver.cpp](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/modules/mavlink/mavlink_receiver.cpp); [R117 mavlink_params.c](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/modules/mavlink/mavlink_params.c)
- **ROS 2:**
  - `OffboardControlMode` has the fields position, velocity, acceleration, attitude, body_rate, thrust_and_torque and direct_actuator. The first true field selects which estimate is required and which controllers are disabled.
  - `TrajectorySetpoint` (NED; position, velocity, acceleration, jerk (logging only), yaw, yawspeed): a position setpoint has non-NaN position (velocity and acceleration act as feed-forward); a velocity setpoint has NaN position; an acceleration setpoint has NaN position and velocity.
  - "only the content of the TrajectorySetpoint messages determines which of the three controllers shall run".
  - Sources: [R117 TrajectorySetpoint.msg](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/msg/versioned/TrajectorySetpoint.msg); [R117 OffboardControlMode.msg](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/msg/OffboardControlMode.msg); [Offboard docs](https://docs.px4.io/main/en/flight_modes/offboard.html)
- ROS 2 warning (main docs): "PX4 has no means of filtering and distinguishing ROS 2 messages from internal messages, in any mode". You must "Publish PX4 setpoint messages ONLY in Offboard mode", stream `OffboardControlMode` as the keep-alive, and "Stream the setpoints it wants: unlike with MAVLink, PX4 won't trigger a failsafe if setpoints aren't sent regularly". The PX4 ROS 2 Interface Library is "a safer alternative". — [Offboard docs (main)](https://docs.px4.io/main/en/flight_modes/offboard.html)
- ROS 2 frames: ROS nodes must convert ENU/FLU to NED/FRD for all `TrajectorySetpoint` fields. — [R117 ROS 2 user guide](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/docs/en/ros2/user_guide.md)
- Mode numbers for `MAV_CMD_DO_SET_MODE` (param1 = 1 custom-mode flag, param2 = main mode): MANUAL 1, ALTCTL 2, POSCTL 3, AUTO 4, ACRO 5, **OFFBOARD 6**, STABILIZED 7, ..., TERMINATION 10, ALTITUDE_CRUISE 11 (v1.17). — [R117 px4_custom_mode.h](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/modules/commander/px4_custom_mode.h)
- MAVSDK Offboard plugin:
  - "Client code must specify a setpoint before starting Offboard mode".
  - The plugin automatically resends setpoints at 20 Hz.
  - `stop()` switches to Hold.
  - Setpoint calls do not return errors, so monitor the flight mode, because another component can switch the vehicle out of Offboard.
  - Source: [MAVSDK Offboard guide](https://mavsdk.mavlink.io/main/en/cpp/guide/offboard.html)

### Inferences
- **Indoor failsafe settings:**
  - `COM_OF_LOSS_T` 0.5–1.0 s.
  - `COM_OBL_RC_ACT = 5` (Hold) if the position estimate is VIO-backed and trusted, or `4` (Land). Do not use Return without a GPS home.
  - `NAV_RCL_ACT = 3` (Land) so the no-RC fallback is not Return.
  - Fly with a real RC transmitter and a safety pilot (`COM_RC_IN_MODE` 0 or 3). Do not use `COM_RC_IN_MODE=4` indoors.
- **Python state machine pattern (MAVLink):**
  1. CONNECT: heartbeat, timesync converged, EV fusing (`vehicle_local_position` xy_valid, low EKF innovations).
  2. PREARM: start the setpoint stream at 20–50 Hz holding the current position, and wait more than 1 s.
  3. ARM (`MAV_CMD_COMPONENT_ARM_DISARM`).
  4. OFFBOARD (`MAV_CMD_DO_SET_MODE` 1/6).
  5. TAKEOFF: position setpoint z = −h.
  6. MISSION: velocity or position setpoints from the planner, with a speed clamp from the stopping model.
  7. On any watchdog trip (VIO quality < threshold, depth stale > 0.3 s, EKF flags bad, mode ≠ Offboard): command a zero-velocity hold, and land if it persists.
  8. LAND (`MAV_CMD_NAV_LAND`, or a descending velocity setpoint) → DISARM.
  - Keep the setpoint publisher on its own thread or async task, independent of the planner, so planner stalls do not stop the stream. Also make the planner's own heartbeat gate the setpoint content (hold if the planner is stale).
- **Prefer velocity, or position + velocity-feed-forward, setpoints in local NED**, with yaw specified. `MPC_XY_VEL_MAX` still clamps horizontal velocity in the position controller (section 10).

### Gaps
- No official PX4 example of a full Python (pymavlink) Offboard state machine was found. The pattern above is synthesised from the docs and code.

## 8. MAVSDK-Python vs pymavlink vs ROS 2 (px4_msgs / uXRCE-DDS) for a team with days to integrate

### Takeaway
For a Python team with days, **MAVSDK** is the fastest route to safe Offboard control. It handles setpoint resend, timesync and mode/arm commands. Note that MAVSDK v4 (Sept 2026) changed the Python package. Use **pymavlink** (or MAVSDK's raw MAVLink passthrough) for the two custom streams where MAVSDK's helpers are risky or missing: ODOMETRY with the correct estimator type, and OBSTACLE_DISTANCE. ROS 2 / uXRCE-DDS gives the cleanest typed interface but has more set-up cost and version-matching pitfalls.

### Cited Findings
- **MAVSDK v4.0.0 (published 2026-09-22) shipped "New Python bindings".**
  - From v4, the PyPI name `mavsdk` "refers instead to a native binding that calls the MAVSDK C++ library directly". There is no gRPC and no `mavsdk_server`, and plugins are constructed, e.g. `Action(drone).arm()`.
  - `mavsdk.asyncio` offers an asyncio API, and the base package a synchronous, callback-based one.
  - The old gRPC wrapper is now `mavsdk-grpc` (import `mavsdk_grpc`) and "continues to be developed".
  - `mavsdk<4` "receives no further updates".
  - Sources: [MAVSDK v4.0.0 release](https://github.com/mavlink/MAVSDK/releases/tag/v4.0.0); [MAVSDK v4 Python migration](https://mavsdk.mavlink.io/v4/en/python/migration.html)
- On PyPI, `mavsdk` is at 4.0.6 and requires Python ≥3.8. The MAVSDK-Python repo's latest release is 4.0.5 (2026-10-07). — [PyPI mavsdk](https://pypi.org/project/mavsdk/); [MAVSDK-Python releases](https://github.com/mavlink/MAVSDK-Python/releases)
- The MAVSDK Offboard plugin resends at 20 Hz and requires an initial setpoint before `start()`. — [MAVSDK Offboard guide](https://mavsdk.mavlink.io/main/en/cpp/guide/offboard.html)
- The MAVSDK Mocap plugin defaults ODOMETRY `estimator_type` to MOCAP, which PX4 routes away from EKF2 (section 2). — [MAVSDK mocap_impl.cpp](https://github.com/mavlink/MAVSDK/blob/main/cpp/src/mavsdk/plugins/mocap/mocap_impl.cpp); [R117 mavlink_receiver.cpp](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/modules/mavlink/mavlink_receiver.cpp)
- **ROS 2 setup:**
  - From v1.14, ROS 2 uses uXRCE-DDS. The companion runs `MicroXRCEAgent` (e.g. `serial --dev <port> -b 921600`).
  - PX4 sets `UXRCE_DDS_CFG` to the port (e.g. TELEM2) and the `SER_TEL2_BAUD` rate. MAVLink must first be disabled on that port (`MAV_1_CONFIG=0`).
  - ROS 2 apps must be built against the same px4_msgs definitions as the firmware (px4_msgs branches per release). From v1.16, the message translation node allows mismatched versions, using `_v<N>` topic suffixes.
  - Sources: [R117 uxrce_dds.md](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/docs/en/middleware/uxrce_dds.md); [R117 ROS 2 user guide](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/docs/en/ros2/user_guide.md); [R117 msg translation node](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/docs/en/ros2/px4_ros2_msg_translation_node.md)
- "ROS 2 code that subscribes to topics published by PX4 must specify a appropriate (compatible) QoS setting in order to listen to topics." — [R117 ROS 2 user guide](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/docs/en/ros2/user_guide.md)
- The PX4 ROS 2 Interface Library (C++) allows "complete replacement of PX4 flight modes" and is PX4's recommended replacement for the removed avoidance interface. — [Path planning interface (removed)](https://docs.px4.io/main/en/computer_vision/path_planning_interface.html)

### Inferences
- **Recommendation for days-scale integration:** one Python process (or two, given the repo's one-process-per-camera rule) using one MAVLink connection on TELEM2 at 921600 baud.
  - Use MAVSDK (v4 `mavsdk.asyncio`, or `mavsdk-grpc` if existing code depends on the 3.x API) for connect, telemetry, arm, mode and the Offboard stream.
  - Use pymavlink, or MAVSDK's MavlinkDirect/passthrough plugin (stabilised in v4 per the release notes), to send ODOMETRY with `estimator_type=VIO` and OBSTACLE_DISTANCE.
  - If you skip MAVSDK entirely, pymavlink is fine but you must implement: TIMESYNC replies, 20+ Hz setpoint resend, COMMAND_ACK handling for arm and mode, heartbeats (send your own `HEARTBEAT` with `MAV_TYPE_ONBOARD_CONTROLLER`), and mode monitoring.
- **MAVSDK v4 is new (released about 1 month before the competition).** Pin an exact version and test it on the Jetson (aarch64 wheel availability was not verified). Falling back to `mavsdk-grpc` or `mavsdk<4` is reasonable, but `mavsdk<4` gets no updates.
- **ROS 2 makes sense if the team already runs ROS 2 Humble on the Jetson** (e.g. Isaac ROS cuVSLAM). Otherwise it adds DDS agent set-up, QoS, px4_msgs version pinning, and the "no failsafe if setpoints stop" hazard.
- **Message bandwidth on 921600 baud** (about 92 KB/s) is ample:
  - ODOMETRY is about 230 bytes, so 30 Hz is about 7 KB/s.
  - OBSTACLE_DISTANCE is about 170 bytes, so 20 Hz is about 3.5 KB/s.
  - Setpoints at 50 Hz are about 3 KB/s.
  - The default Onboard stream set from PX4 (ATTITUDE 100 Hz, HIGHRES_IMU 50 Hz, ...) is the bigger consumer. Use `MAV_1_RATE` or stream-rate requests if the link saturates.

### Gaps
- Not verified: whether MAVSDK v4 Python wheels exist for aarch64 / JetPack 6 Python, and whether the v4 Mocap Odometry struct exposes `estimator_type` in Python.

## 9. Status of PX4-Avoidance (local_planner, global_planner, safe_landing_planner) and `COM_OBS_AVOID`

### Takeaway
The PX4-Avoidance repository (local_planner, global_planner, safe_landing_planner) is **archived**. The PX4-side "Path Planning Interface" (`COM_OBS_AVOID`, mission obstacle avoidance, Safe Landing) was removed by PR #24172 (merged 2025-02-18), so v1.15 is the last release that contains it. The replacement is the PX4 ROS 2 Interface Library (custom modes) or plain Offboard control.

### Cited Findings
- Docs badge "Removed PX4 v1.15": the Path Planning Interface and its features (obstacle avoidance in missions, Safe Landing) "are no longer supported or maintained" and "should not be used in any PX4 version". The code was "abandoned due to architectural constraints". It is replaced by "more generic approaches" such as the PX4 ROS 2 Interface Library, which "allows complete replacement of PX4 flight modes". — [Path Planning Interface (removed)](https://docs.px4.io/main/en/computer_vision/path_planning_interface.html)
- PR #24172, "Remove collision avoidance interface on PX4 side", was merged 2025-02-18. Per a summary of the PX4 sync and the PR, it removed the FlightTask avoidance library, the avoidance health check and `COM_OBS_AVOID`. — [PR #24172](https://github.com/PX4/PX4-Autopilot/pull/24172); [PX4 Sync/Q&A Jan 15 2025](https://discuss.px4.io/t/px4-sync-q-a-jan-15-2025/43274)
- `COM_OBS_AVOID` does not appear anywhere in the v1.17 commander parameters or the v1.17 sources searched. — [R117 commander_params.c](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/modules/commander/commander_params.c)
- v1.15 docs: obstacle avoidance was enabled with `COM_OBS_AVOID=1`. — [v1.15 obstacle avoidance docs](https://docs.px4.io/v1.15/en/computer_vision/obstacle_avoidance.html)
- PX4/PX4-Avoidance: `archived: true`, last push 2024-04-10. — [PX4/PX4-Avoidance](https://github.com/PX4/PX4-Avoidance)
- A forum user asked how to implement an onboard workflow after v1.16 dropped the trajectory waypoint messages. No documented answer was found. — [PX4 discuss: onboard workflow with v1.16](https://discuss.px4.io/t/how-to-implement-onboard-workflow-with-px4-version-1-16-0/48311)
- The CP docs say the companion path is "untested" because "the original companion project was unmaintained and has been archived". — [R117 collision_prevention.md](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/docs/en/computer_vision/collision_prevention.md)

### Inferences
- Do not plan around PX4 Mission mode with companion avoidance. Write the planner on the companion and command PX4 in Offboard, which is also simpler without global position. If a "PX4-native mode" feel is wanted, the C++ PX4 ROS 2 Interface Library can register a custom mode, but that is more work than days allow.

### Gaps
- There is a small labelling inconsistency: the docs say "Removed PX4 v1.15", while the code removal landed after v1.15.0 (Feb 2025). In practice v1.15 still contains the code and v1.16 does not.

## 10. Indoor safety: geofence without GPS, kill switch, velocity limits, position-loss failsafe, VIO stalls

### Takeaway
PX4's geofence works on **global (WGS84) position relative to Home**, not on the local frame. Indoors it only works if you set an EKF global origin (`SET_GPS_GLOBAL_ORIGIN`) so that global position and Home exist. Rely instead on the companion's own local-frame virtual walls, the RC kill switch (`COM_KILL_DISARM` 5 s), and conservative `MPC_XY_VEL_MAX`. Position loss drops a multicopter to Altitude mode, or Stabilized if there is no height. VIO stalls longer than about 0.4 s stop EV fusion. The EKF then declares horizontal position invalid after `EKF2_NOAID_TOUT` (5 s), or earlier via `COM_POS_FS_EPH` (5 m).

### Cited Findings
- **Geofence failsafe parameters:**
  - The fence is "a cylinder centered on the home position".
  - `GF_ACTION` (None/Warning/Hold/Return/Terminate/Land; default 2 = Hold).
  - `GF_MAX_HOR_DIST` and `GF_MAX_VER_DIST` (0 disables).
  - `GF_SOURCE` (0 = global position estimate, 1 = GPS).
  - `GF_PREDICT` (experimental, default 0).
  - Terminate also requires `CBRK_FLIGHTTERM`.
  - Sources: [R117 safety.md](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/docs/en/config/safety.md); [R117 geofence_params.c](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/modules/navigator/geofence_params.c)
- Geofence code computes horizontal distance with `get_distance_to_point_global_wgs84` against the Home lat/lon, only if `home_global_position_valid()`. Polygon fences are checked against global position and must contain Home. — [R117 geofence.cpp](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/modules/navigator/geofence.cpp); [R117 geofence.md](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/docs/en/flying/geofence.md)
- Auto modes need a global position; local-only systems can send `SET_GPS_GLOBAL_ORIGIN`. — [External Position Estimation](https://docs.px4.io/main/en/ros/external_position_estimation.html)
- **Kill switch:**
  - It "immediately stops all motor outputs".
  - By default motors restart if the switch is reverted within 5 s (`COM_KILL_DISARM`); after that the vehicle disarms.
  - There is also an irreversible kill gesture.
  - In v1.17, `manual_lockdown` was renamed `kill`.
  - Sources: [R117 safety.md](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/docs/en/config/safety.md); [R117 commander_params.c](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/modules/commander/commander_params.c); [v1.17 notes](https://docs.px4.io/main/en/releases/1.17.html)
- **Velocity and acceleration limit defaults (v1.17):**
  - `MPC_XY_VEL_MAX` 12 m/s, `MPC_XY_CRUISE` 5 m/s, `MPC_VEL_MANUAL` 10 m/s.
  - `MPC_ACC_HOR` 3 m/s², `MPC_ACC_HOR_MAX` 5 m/s².
  - `MPC_JERK_MAX` 8 m/s³, `MPC_JERK_AUTO` 4 m/s³.
  - `MPC_Z_VEL_MAX_UP` 3 m/s, `MPC_Z_VEL_MAX_DN` 1.5 m/s.
  - `MPC_TILTMAX_AIR` 45°, `MPC_XY_ERR_MAX` 2 m.
  - Sources: [R117 multicopter_position_control_limits_params.c](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/modules/mc_pos_control/multicopter_position_control_limits_params.c); [R117 multicopter_autonomous_params.c](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/modules/mc_pos_control/multicopter_autonomous_params.c); [R117 multicopter_position_mode_params.c](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/modules/mc_pos_control/multicopter_position_mode_params.c)
- The position controller limits horizontal velocity to `min(MPC_XY_VEL_MAX, vehicle_local_position.vxy_max)` via `setVelocityLimits()`, constraining the combined position-loop and feed-forward velocity setpoint. `MPC_XY_CRUISE` and `MPC_VEL_MANUAL` are clamped to `MPC_XY_VEL_MAX`. — [R117 MulticopterPositionControl.cpp](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/modules/mc_pos_control/MulticopterPositionControl.cpp); [R117 PositionControl.cpp](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/modules/mc_pos_control/PositionControl/PositionControl.cpp)
- **Position-loss failsafe:**
  - It is triggered by a timeout since the last velocity- or position-aiding fusion (`EKF2_NOAID_TOUT`) or by EPH > `COM_POS_FS_EPH` (default 5 m; multicopters only). `COM_VEL_FS_EVH` is 1 m/s.
  - The action for multicopters is "switch to Altitude mode if a height estimate is available, otherwise Stabilized".
  - The failsafe triggers 1 s after loss (v1.16 removed `COM_POS_FS_DELAY`).
  - `COM_POS_LOW_EPH` (default -1, disabled) can trigger `COM_POS_LOW_ACT` on low accuracy.
  - Sources: [R117 safety.md](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/docs/en/config/safety.md); [R117 commander_params.c](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/modules/commander/commander_params.c); [v1.16 notes](https://docs.px4.io/main/en/releases/1.16.html)
- `COM_ARM_WO_GPS` defaults to 1 ("Warning only"). ModalAI also sets it for indoor VIO. — [R117 commander_params.c](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/modules/commander/commander_params.c); [voxl indoor_vio.params](https://gitlab.com/voxl-public/voxl-sdk/utilities/voxl-px4-params/-/blob/master/params/v1.18/EKF2_helpers/indoor_vio.params)
- v1.18 (beta) adds `FD_ALT_LOSS` flight termination when the vehicle descends too far below its setpoint. — [v1.18 notes](https://docs.px4.io/main/en/releases/1.18.html)
- A data-link-loss failsafe (`NAV_DLL_ACT`) and a manual-control-loss failsafe (`NAV_RCL_ACT`) exist. — [R117 safety.md](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/docs/en/config/safety.md)

### Inferences
- **Indoor parameter set:**
  - `MPC_XY_VEL_MAX` about 2 m/s (hard ceiling, applies in Offboard too).
  - `MPC_VEL_MANUAL` about 1.5 m/s and `MPC_XY_CRUISE` about 1.5 m/s.
  - `MPC_Z_VEL_MAX_UP` about 1 m/s and `MPC_Z_VEL_MAX_DN` about 0.7 m/s.
  - Optionally a lower `MPC_TILTMAX_AIR` (25–30°) for a 7-inch quad indoors.
  - `NAV_RCL_ACT=3` (Land) and `COM_OBL_RC_ACT=5`/`4` (see section 7).
  - Map an RC kill switch and an RC Position/Altitude switch for the safety pilot.
- **Geofence indoors:**
  - Option 1: send `SET_GPS_GLOBAL_ORIGIN` (any fixed lat/lon/alt) after EV fusion starts, so Home is set at arming and `GF_MAX_HOR_DIST`/`GF_MAX_VER_DIST` act as a cylinder around the takeoff point (e.g. 1 m inside the nets, `GF_ACTION` = Land).
  - This depends on VIO drift (2–3 % drift is about 0.5 m over a 20 m path) and must be validated in simulation.
  - Option 2: implement the box fence on the companion in the VIO frame.
  - Do both if time permits.
- **What happens when VIO stalls:**
  - Under 0.2 s: fine.
  - 0.2–0.4 s: EV still considered, but the next sample may restart fusion.
  - Over 0.4 s: "vision data stopped".
  - EKF dead-reckons on IMU until `EKF2_NOAID_TOUT` (5 s), or until EPH exceeds 5 m. Then local position becomes invalid, Offboard and Position cannot run, and the vehicle falls to Altitude mode (drifting horizontally) or Stabilized.
  - Lower `EKF2_NOAID_TOUT` (e.g. 1–2 s) so the failsafe fires before large silent drift. Make the companion watchdog command hold or land as soon as VIO quality drops, without waiting for PX4.
  - With `EKF2_MAG_TYPE=5`, a re-acquired VIO with a `reset_counter` change is needed for a clean re-alignment.

### Gaps
- Indoor use of PX4's geofence with a synthetic global origin is not documented or tested by PX4. Behaviour under EKF origin or position resets is unknown.

## 11. Simulation and bench-test path (SITL with Gazebo, HITL, props-off)

### Takeaway
PX4 v1.17 SITL with Gazebo Harmonic provides `gz_x500_depth` (a forward OAK-D-style depth camera), `gz_x500_vision` (Gazebo ground-truth odometry injected as `vehicle_visual_odometry`), `gz_x500_lidar_2d/front/down` and `gz_x500_flow`. That is enough to test the obstacle pipeline, the EV parameter set and the Offboard state machine before flight. HITL is community-supported and only works with jMAVSim or Gazebo Classic. SIH (simulation on the FC itself) is the easiest way to test on the real Pixhawk 6C with props off.

### Cited Findings
- **Gazebo vehicles:**
  - `make px4_sitl gz_x500_depth`: "forward-facing depth camera attached, modelled on the OAK-D".
  - `make px4_sitl gz_x500_vision`: X500 with visual odometry.
  - `gz_x500_lidar_2d` (Hokuyo UTM-30LX), `gz_x500_lidar_front`, `gz_x500_lidar_down` (LW20/C), `gz_x500_mono_cam`, `gz_x500_gimbal`.
  - Source: [R117 Gazebo vehicles](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/docs/en/sim_gazebo_gz/vehicles.md)
- Airframes `4002_gz_x500_depth`, `4005_gz_x500_vision` and `4021_gz_x500_flow` exist in v1.17 SITL. — [R117 init.d-posix airframes](https://github.com/PX4/PX4-Autopilot/tree/release/1.17/ROMFS/px4fmu_common/init.d-posix/airframes)
- `GZBridge` subscribes to `/model/<name>/odometry_with_covariance` and converts ENU/FLU to NED/FRD. It copies covariances and publishes to `vehicle_visual_odometry` (reset_counter is not set). — [R117 GZBridge.cpp](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/modules/simulation/gz_bridge/GZBridge.cpp)
- CP simulation: `make px4_sitl gz_x500_lidar_2d`, then add obstacles to the world. — [R117 collision_prevention.md](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/docs/en/computer_vision/collision_prevention.md)
- **Other v1.16–v1.17 Gazebo features:**
  - Gazebo Harmonic is the supported version.
  - Faster-than-real-time simulation (v1.16).
  - `PX4_GZ_MODEL_POSE` and selectively disabled bridge sensors (v1.17).
  - Sources: [v1.16 notes](https://docs.px4.io/main/en/releases/1.16.html); [v1.17 notes](https://docs.px4.io/main/en/releases/1.17.html)
- HITL "is community supported and maintained". PX4 supports HITL for multicopters "using jMAVSim or Gazebo Classic". — [R117 hitl.md](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/docs/en/simulation/hitl.md)
- **SIH:**
  - Runs controller, estimator and simulator on the flight controller. It supports quadrotors (since v1.9) and is compatible with all boards except FMUv2.
  - Also available as SITL from v1.14.
  - SIH is community supported.
  - Source: [R117 SIH](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/docs/en/sim_sih/index.md)
- ROS 2 example offboard workflow with SITL: `MicroXRCEAgent udp4 -p 8888` plus px4_ros_com examples. — [R117 ROS 2 user guide](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/docs/en/ros2/user_guide.md)

### Inferences
- **Recommended test ladder:**
  1. **SITL + gz_x500_vision**: verify the full EV parameter set (`EKF2_GPS_CTRL=0`, `MAG_TYPE=5`, `EV_CTRL=15`, `HGT_REF=3`) and failsafe behaviour by killing the odometry bridge. Also verify the Python Offboard state machine (MAVSDK on UDP 14540).
  2. **SITL + gz_x500_depth**: bridge the Gazebo depth image to the companion code (via `gz` transport or `ros_gz_bridge`), run the real OBSTACLE_DISTANCE generator and the planner against an obstacle world (boxes, poles, thin "net" panels), and check `obstacle_distance_fused` and `collision_constraints`.
  3. **Replace Gazebo odometry with cuVSLAM on recorded OAK-D datasets** (the repo already has datasets) to check frame conventions offline. Compare against `vehicle_odometry` back-streamed in SITL.
  4. **Bench, props off, real hardware:** Jetson ↔ Pixhawk 6C on TELEM2. Carry the vehicle by hand with VIO streaming. Confirm in the QGC MAVLink Inspector or logs that `LOCAL_POSITION_NED` follows (x forward, y right, z down), EV fusion flags are set, timesync has converged, and failsafes fire when the VIO process is killed. Optionally use SIH on the 6C to exercise Offboard arming and mode logic with simulated physics.
  5. **Tethered or low hover in Position mode** with CP enabled, then Offboard at walking speed.
- The camera thermal limit (5–10 min) should shape the bench tests. Run short, watchdog-guarded sessions.

### Gaps
- No PX4-maintained example bridging `gz_x500_depth` images into an OBSTACLE_DISTANCE generator was found.
- The gz depth camera's FOV and range parameters were not inspected (they live in the PX4-gazebo-models repo).

## 12. Latency budget and indoor speed limits from stopping distance

### Takeaway
With PX4's own braking model (jerk 8 m/s³, accel 3 m/s²) plus 0.4 s of sensing/control delay, stopping takes about 1.3 m at 1 m/s, 2.1 m at 1.5 m/s and 3.0 m at 2 m/s, plus `CP_DIST`. Stereo depth of thin obstacles and netting becomes unreliable well before 5 m. Fly the indoor course at about 1–1.5 m/s, with `CP_DIST` (or the companion keep-out) around 0.7–1.0 m.

### Cited Findings
- PX4 braking model: `computeMaxSpeedFromDistance(jerk, accel, d, v_end)` with `b = 4·a²/j`, `c = −2·a·d − v_end²`, `v_max = 0.5·(−b + sqrt(b² − 4c))`. — [R117 TrajMath.hpp](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/lib/mathlib/math/TrajMath.hpp)
- CP adds `v∥·(CP_DELAY + data_age)` to the required distance and keeps `max(CP_DIST, min_distance)`. — [R117 CollisionPrevention.cpp](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/lib/collision_prevention/CollisionPrevention.cpp)
- Docs: `CP_DELAY` covers "sensor and velocity-setpoint tracking delay"; tracking is typically 0.1–0.5 s, and external vision may add up to about 0.2 s. PX4's own testing used 4 m/s with 10 Hz OBSTACLE_DISTANCE. — [Collision Prevention docs](https://docs.px4.io/main/en/computer_vision/collision_prevention.html)
- Defaults `MPC_ACC_HOR` 3 m/s², `MPC_JERK_MAX` 8 m/s³, `CP_DELAY` 0.4 s. — [R117 params](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/modules/mc_pos_control/multicopter_autonomous_params.c); [R117 collisionprevention_params.c](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/lib/collision_prevention/collisionprevention_params.c)
- Anecdotal OAK-D stereo latency from the Luxonis forum: about 24 ms at 400P with LR-check and subpixel at 80 FPS, and about 26 ms at 75 FPS. Latency grows with on-device NN or filtering, and with bandwidth limits. There is no official Luxonis spec. — [Luxonis forum 957](https://discuss.luxonis.com/d/957/16); [Luxonis forum 6963](https://discuss.luxonis.com/d/6963-higher-stereo-latency-on-depthai-v3)
- EV recommended at 30–50 Hz; OBSTACLE_DISTANCE at ≥10 Hz (ArduPilot) and 20 Hz (ModalAI default). — [External Position Estimation](https://docs.px4.io/main/en/ros/external_position_estimation.html); [ArduPilot RealSense](https://ardupilot.org/copter/docs/common-realsense-depth-camera.html); [ModalAI VOA](https://docs.modalai.com/voa/)

### Inferences
- **Computed with PX4's formula** (a = 3 m/s², j = 8 m/s³), as braking distance + delay distance at 0.4 s, before adding `CP_DIST`:

  | Speed | Braking | Delay (0.4 s) | Total |
  |---|---|---|---|
  | 0.5 m/s | 0.42 m | 0.20 m | 0.62 m |
  | 1.0 m/s | 0.92 m | 0.40 m | 1.32 m |
  | 1.5 m/s | 1.50 m | 0.60 m | 2.10 m |
  | 2.0 m/s | 2.17 m | 0.80 m | 2.97 m |
  | 3.0 m/s | 3.75 m | 1.20 m | 4.95 m |

  The 4·a²/j term makes PX4's model conservative. Raising `MPC_JERK_MAX` (e.g. 15–20) shortens it, at the cost of jerkier flight.
- **Latency budget to measure:**
  - OAK-D capture → host: about 25–40 ms for depth at 30 fps.
  - cuVSLAM: one frame plus processing, about 20–40 ms.
  - Depth-to-sectors: under 10 ms.
  - MAVLink serial at 921600: about 2–3 ms per message.
  - PX4 control and tracking: about 0.1–0.3 s.
  - The 0.4 s default `CP_DELAY` is a reasonable conservative start. Set the companion planner's look-ahead the same way.
- **Speed choice:** a 7-inch quad in a netted indoor course with obstacles of various sizes should run at about 1.0–1.5 m/s (stopping inside about 2 m) with a 0.7–1.0 m keep-out. Raise speed only after logs show that EKF innovations and depth latency stay low. Give `MPC_XY_VEL_MAX` some headroom over the planner's commanded maximum, so the position controller can recover position error without saturating.

### Gaps
- No measured end-to-end latency for this exact stack (OAK-D S2 → PyCuVSLAM on Orin Nano → Pixhawk 6C) was found. It must be logged (OAK-D device timestamps vs PX4 `timestamp_sample`).
- No published stereo-depth reliability figures for netting at a given range were found.

## 13. Recommended architecture and hardware link (synthesis)

### Takeaway
On a Pixhawk 6C (no Ethernet), connect the Jetson to **TELEM2** (UART5, `/dev/ttyS3`, full flow control), which defaults to MAVLink "Onboard" at 921600 baud. Run one Python MAVLink process that:
- streams ODOMETRY at 30 Hz (`LOCAL_FRD` / `BODY_FRD` / `VIO`, with covariance, `reset_counter` and quality);
- streams OBSTACLE_DISTANCE at 15–20 Hz (`BODY_FRD`, 5° sectors over the camera FOV);
- runs the Offboard state machine with velocity or position setpoints at 20–50 Hz.

PX4 runs EKF2 on EV (plus an optional rangefinder and flow) and enforces velocity limits and failsafes. Collision Prevention is a manual-mode safety net only.

### Cited Findings
- Pixhawk 6C UART map: USART1 `/dev/ttyS0` GPS1, USART2 `/dev/ttyS1` TELEM3, UART5 `/dev/ttyS3` **TELEM2**, UART7 `/dev/ttyS5` TELEM1, UART8 `/dev/ttyS6` GPS2. "TELEM2 – Full flow control". The page lists no Ethernet port for the 6C. — [R117 pixhawk6c.md](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/docs/en/flight_controller/pixhawk6c.md)
- TELEM2 defaults: `MAV_1_CONFIG = TELEM 2`, `MAV_1_MODE = Onboard`, `MAV_1_RATE = 0` (half maximum), `MAV_1_FORWARD = Disabled`, `SER_TEL2_BAUD = 921600`. — [R117 mavlink_peripherals.md](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/docs/en/peripherals/mavlink_peripherals.md)
- "Ethernet is the recommended connection, if supported by your flight controller". "PX4 expects companion computers to connect via TELEM2 for offboard control". An FTDI USB-serial adapter from TELEM2 to the companion's USB is "a safe and easy" option, with udev symlinks recommended. — [R117 pixhawk_companion.md](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/docs/en/companion_computer/pixhawk_companion.md)
- To use uXRCE-DDS on TELEM2 instead, set `MAV_1_CONFIG=0`, then `UXRCE_DDS_CFG=TELEM2`. — [R117 uxrce_dds.md](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/docs/en/middleware/uxrce_dds.md)
- The Onboard MAVLink mode streams TIMESYNC 10 Hz, HIGHRES_IMU 50 Hz, LOCAL_POSITION_NED 30 Hz, ATTITUDE 100 Hz, DISTANCE_SENSOR 10 Hz and others. — [R117 mavlink_main.cpp](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/modules/mavlink/mavlink_main.cpp)
- v1.16 added a parameter to always start the MAVLink stream via USB (PR #22234). — [v1.16 notes](https://docs.px4.io/main/en/releases/1.16.html)

### Inferences
- **Data flow:**
  - OAK-D S2 → (stereo + IMU) → PyCuVSLAM (WSL2 now, the Jetson later) → guard → pose/vel/cov.
  - Pose/vel/cov feeds MAVLink ODOMETRY at 30 Hz with `reset_counter` and quality.
  - Depth feeds the sector extractor → OBSTACLE_DISTANCE at 15–20 Hz, and the companion local map/planner.
  - The planner feeds the Offboard setpoint stream at 20–50 Hz.
  - PX4 sends back `LOCAL_POSITION_NED`/`ODOMETRY`, `ATTITUDE`, `HEARTBEAT` (mode) and `STATUSTEXT` for the watchdogs.
- **Link choice:** Jetson UART (40-pin header) direct to TELEM2 with a common ground, or a USB-FTDI adapter. Use 921600 baud with hardware flow control if wired. A Pixhawk USB-to-Jetson link also works for MAVLink, but USB disconnects and brown-outs make it less robust in flight.
- **Use ROS 2 only if** the team wants Isaac ROS cuVSLAM or nvblox on the Jetson. Then put uXRCE-DDS on TELEM2 (or TELEM3), and keep a MAVLink instance on another port for QGC/MAVSDK.
- **Pre-flight checklist (PX4 side):**
  - EKF2 parameters as in section 2, plus a reboot.
  - `COM_ARM_WO_GPS=1`.
  - Failsafes as in sections 7 and 10.
  - `SDLOG_PROFILE` with bit 7 set.
  - `MPC_POS_MODE=4` and `CP_DIST` > 0 (e.g. 0.8 m) for the manual safety net.
  - `CP_GO_NO_DATA=1` if a forward-only camera must allow backing away.
  - `MPC_XY_VEL_MAX` about 2 m/s.
  - RC kill switch mapped.
  - Verify timesync convergence and EV fusion before arming.

### Gaps
- Whether the Jetson Orin Nano's UART reliably sustains 921600 baud with flow control to the 6C was not verified (check the JetPack UART device, e.g. `/dev/ttyTHS*`, and the carrier-board pinout).
- PX4 has a Holybro Pixhawk Jetson Baseboard page (`companion_computer/holybro_pixhawk_jetson_baseboard.md`), but it was not read. Whether it applies to a standalone 6C wired to an Orin Nano dev kit is unverified.
