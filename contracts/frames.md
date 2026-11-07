# Frames

Reasoning and background: `docs/field-guide.html` §architecture, "Frames, before
anything flies". This file is the normative statement; where they disagree,
this file wins (AGENTS.md §3.1).

## Frame table

| frame | convention | axes | source |
|---|---|---|---|
| Course | surveyed world | east, north, up; floor-centred origin | `contracts/course.md` |
| cuVSLAM `odom` | local world, ENU-labelled | startup-relative, not compass-aligned | `organisers` |
| ROS `base_link` | body FLU | forward, left, up; Pixhawk primary IMU origin | `organisers` |
| PX4 local | world NED | north, east, down; EKF2's own origin/yaw | `organisers` |
| PX4 body | body FRD | forward, right, down | `organisers` |

World ENU → NED maps `(x, y, z)` to `(y, x, -z)`. Body FLU → FRD maps it
to `(x, -y, -z)`. These are different proper 3-D rotations, not a reflection
or an interchangeable "forward/north" frame. ROS messages use ROS timestamps;
PX4 messages use microseconds, with time-domain handling owned by the kit.

## Course → odom (normative)

The launch mark is `(−2.515, 0.914)` in course `(east, north)` metres,
facing north. This physical heading is not sufficient to establish the
vision axes: stereo-only cuVSLAM starts at the rig's startup pose, without
compass or gravity alignment. Arming does not define or reset that origin.

Measure a rigid course-to-odom transform at each tracking initialization:

    p_odom = o_odom + R_odom_course * (p_course - o_course)

Here `o_course` is the surveyed position of the tracked IMU at the reference
pose, `o_odom` is its simultaneous odometry reading, and `R_odom_course`
rotates course-world vectors into odom-world vectors. Use level setup and
known independent east/north/up displacements to check orientation, scale
and height. Do not assume an identity rotation from the launch heading.
Course heading is clockwise from north; transform its direction by the
same rotation before extracting odom yaw (about +z, from +x toward +y).
If leveling/alignment cannot be established, flight remains blocked.

The field guide's `body_resting_height = 0.07 m` was measured on the lab
stand-in. It is not automatically this kit's IMU height above the floor.
Only with that height and a level, aligned startup origin verified does
`0.8 - 0.07 = 0.73 m` give the desired upward displacement.

### Conditional worked example

For the published pad `(−2.042, −0.792)` and launch mark:

    Δeast  = −2.042 − (−2.515) = +0.473 m
    Δnorth = −0.792 − 0.914   = −1.706 m

Only if `R_odom_course` is identity, `o_odom` is zero, and the measured
reference height is `0.07 m`, the odom target at `0.8 m` above the floor is
`(+0.473, −1.706, +0.73) m`. The plain ENU → NED conversion of this vector
is `(−1.706, +0.473, −0.73) m`, for bench comparison only. Neither example
establishes actual odom/PX4 origin or heading agreement.

## The competitor output (normative)

Publish odom-frame ENU `geometry_msgs/PoseStamped` position/yaw on
`/skybeta/setpoint/pose`, or world-frame ENU `geometry_msgs/TwistStamped`
velocity/yaw-rate on `/skybeta/setpoint/twist`. Team code never publishes to
`/fmu/in/*` or modifies `skybeta_flight_interface`.

The kit performs ENU → NED internally. That axis conversion alone does not
align odom with EKF2. The bridge declares vision `POSE_FRAME_FRD` (arbitrary
world heading, not a body-relative position), allowing EKF2 to estimate a
rotation. The interface defaults to `anchor_setpoints=False` and
`odom_to_ned_yaw_offset_rad=0.0`: it does not automatically apply that EKF
rotation. Its optional anchoring subtracts an odom reference, rotates the
converted displacement about NED down by the configured residual yaw,
and adds a simultaneous PX4 reference. Measure and verify the residual yaw
and origins, using only organizer-permitted configuration; do not patch the
interface or assume zero offset is valid. Block flight if the supplied
configuration cannot represent the measured transform.

Source: read-only `drone-snapshot/home/dronehacks/skybeta/README.md` §Frames,
`src/oakd_vslam_bringup/oakd_vslam_bringup/px4_odom_bridge.py`, and
`src/skybeta_flight_interface/skybeta_flight_interface/{frames,flight_interface}.py`
under that snapshot.

## Acceptance check (required before flight, not performed by this PR)

With props off, record the reference and hand-carry the level drone through
known independent course east/north/up displacements. Compare predicted
odom position and yaw against `/visual_slam/tracking/odometry`, and the
interface's converted target against simultaneous PX4 local position.
Repeat from a different startup heading and after a tracker restart;
invalidate alignment after any reset. Require agreement within the 5 cm
course survey tolerance (`contracts/course.md`), not just matching vector
magnitudes. Do not arm or fly on an unverified transform.
