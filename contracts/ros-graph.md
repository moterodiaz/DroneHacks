# ROS graph

Reasoning and background: `docs/field-guide.html` §architecture (the four
lanes) and §kit. This file is the normative statement; where they disagree,
this file wins (AGENTS.md §3.1).

## Nodes

| node | owner | subscribes | publishes | msg type | rate | QoS |
|---|---|---|---|---|---|---|
| `depthai-ros` | kit (Luxonis driver) | — | stereo image pair, IMU | `sensor_msgs/Image`, `sensor_msgs/Imu` | 1024×640 @ 30 fps image, 200 Hz IMU | default |
| cuVSLAM (`visual_slam`) | library (Isaac ROS, NVIDIA) | stereo image pair, IMU | pose | `nav_msgs/Odometry` (pose+covariance) | ~30 Hz, stereo-only mode | default |
| **Guard** | **ours** | cuVSLAM pose | filtered pose | `nav_msgs/Odometry` | passthrough, with withholds | default |
| `px4_odom_bridge.py` | kit | Guard's filtered pose (ENU) | `VehicleOdometry` (FRD) | `px4_msgs/VehicleOdometry` | passthrough | best-effort out |
| **Planner** | **ours** | `contracts/course.md` map file | route | internal | once per run / re-plan | — |
| **Follower** | **ours** | Planner's route, PX4 estimate | led pose setpoints | ENU pose/twist | 50 Hz | — |
| `skybeta_flight_interface` | kit | Follower's setpoints | PX4 setpoints (never `/fmu/in/*` directly) | ENU pose/twist, clamped 1.5 m/s and 0.3–3 m | ≤50 Hz | — |

## Guard (ours) — withhold conditions

The guard sits between cuVSLAM and `px4_odom_bridge.py`. It withholds a frame
when any of:
- image texture under 1%,
- cuVSLAM reports tracking failed,
- the implied motion exceeds 10 m/s or 720°/s,
- the implied rotation disagrees with the gyro.

On vision return after a withhold, the guard continues from PX4's own
estimate — no jump. It writes fixed, honest variances on every pose it
passes (cuVSLAM's own covariance is not a real uncertainty and must not be
forwarded as one). After 30 consecutively withheld frames, restart the
tracker. Source: `measured` (x500 stand-in sim).

## Planner (ours) — map-only goals

Consumes `contracts/course.md`'s map-file format, not typed coordinates.
A* on a 5 cm grid, Nav2 inflation cost, impassable within 0.32 m of a wall.
Goals, in order, come from the map: tunnel middle, window, pad. This is
rule 9.5's generality requirement; see AGENTS.md §3.7.

## Follower (ours) — lead formula

Route in, led pose setpoints out at 50 Hz:

```
led_pose = pose + v / 0.95 + a / (0.95 × 1.8)
```

This compensates for `skybeta_flight_interface`'s pose-only API lagging the
true trajectory. Source: `measured`.

## QoS / DDS traps (normative, not advisory)

- `/fmu/out/*` topics are **best-effort**. A default (reliable) subscriber
  silently receives nothing — set the matching QoS explicitly. Source:
  `organisers`.
- Fast DDS's default 512 KB shared-memory segment drops 1 MB best-effort
  images. Size the segment or downsample before publishing. Source:
  `measured`.
