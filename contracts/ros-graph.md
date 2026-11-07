# ROS graph

Reasoning and background: `docs/field-guide.html` §architecture (the four
lanes) and §kit. This file is the normative statement; where they disagree,
this file wins (AGENTS.md §3.1).

## Published kit path and planned team nodes

| node | owner | subscribes | publishes | msg type | rate | QoS |
|---|---|---|---|---|---|---|
| `depthai-ros` | kit (Luxonis driver) | — | stereo image pair, IMU | `sensor_msgs/Image`, `sensor_msgs/Imu` | 1024×640 @ 30 fps image, 200 Hz IMU | default |
| cuVSLAM (`visual_slam`) | library (Isaac ROS, NVIDIA) | stereo image pair, IMU | pose | `nav_msgs/Odometry` (pose+covariance) | ~30 Hz, stereo-only mode | default |
| `px4_odom_bridge.py` | kit | `/visual_slam/tracking/odometry`, tracking status | `VehicleOdometry` (world-fixed arbitrary-heading FRD) | `px4_msgs/VehicleOdometry` | passthrough | best-effort out |
| **Planner** | **ours** | `contracts/course.md` map file | route | internal | once per run / re-plan | — |
| **Follower** (planned) | **ours** | Planner's route, odometry, kit state | `/skybeta/setpoint/pose` or `/skybeta/setpoint/twist` | `geometry_msgs/PoseStamped` / `geometry_msgs/TwistStamped`, world ENU | see below | match kit |
| `skybeta_flight_interface` | kit | competitor pose/twist | PX4 setpoints and heartbeat (kit alone writes `/fmu/in/*`) | clamped 1.5 m/s twist and 0.3–3 m altitude by default; no horizontal geofence | 10 Hz default | kit-owned |

## Guard proposal (blocked, not in the shipped data path)

The organizer bridge consumes cuVSLAM directly. The field guide's upstream
Guard is a simulator proposal, not a deployed node or permission to remap,
replace, or publish competing vision. It remains blocked pending organizer
permission, a reviewed contract change and kit validation.

The proposed texture <1%, failed-tracking, 10 m/s and 720°/s gates, gyro
cross-check, fixed variances, re-anchor and restart-after-30-frames policy
are simulator observations/design inputs, not a flight-safety bound.
These permissive motion gates do not protect the course's ±0.18 m error
budget. Re-anchoring to a drifting EKF estimate cannot guarantee continuity
or clearance. Tunnel traversal during vision loss remains blocked until a
measured loss-duration/drift bound fits the clearance budget, with a
pilot-approved abort procedure. Do not infer that withholding or resetting
the tracker makes a run safe.

## Planner (ours) — map-only goals

Consumes `contracts/course.md`'s map-file format, not typed coordinates.
A* on a 5 cm grid, Nav2 inflation cost, impassable within 0.32 m of a wall.
Goals, in order, come from the map: tunnel middle, window, pad. This is
rule 9.5's generality requirement; see AGENTS.md §3.7.

## Follower (planned) — supplied interface only

The previous fixed-gain, 50 Hz led-pose design is dropped, as recorded in
the changelog. The interface accepts both pose and twist, but no competitor
acceleration input. Do not modify it or require an increased output rate.
Its published defaults are `setpoint_rate_hz=10.0` (10 Hz) and
`setpoint_timeout_s=1.0` s. Publish fresh inputs with margin inside that timeout;
the organizer startup procedure requires setpoints before arming, at least
once per second. Merely matching the timeout boundary is not a robust
ongoing cadence. Verify the actual launch configuration and timeout behavior
with props off before choosing the follower cadence. PX4's documented
1.0 m/s cap still applies; the interface's 1.5 m/s twist clamp does not
override it.

Sources: read-only `drone-snapshot/home/dronehacks/skybeta/README.md` and
`src/skybeta_flight_interface/skybeta_flight_interface/flight_interface.py`
under that snapshot. Use `contracts/frames.md` for measured frame alignment.

## QoS / DDS traps (normative, not advisory)

- `/fmu/out/*` topics are **best-effort**. A default (reliable) subscriber
  silently receives nothing — set the matching QoS explicitly. Source:
  `organisers`.
- Fast DDS's default 512 KB shared-memory segment drops 1 MB best-effort
  images. Size the segment or downsample before publishing. Source:
  `measured`.
