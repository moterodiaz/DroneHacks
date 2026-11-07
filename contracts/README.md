# Contracts

Contracts are authoritative. If code disagrees with what's documented here, the
code is wrong. To change a contract: open a PR, add a `contracts/CHANGELOG.md`
entry, and ping the affected area owners (see docs/BOARD.md) — then change the
code in the same PR or a follow-up that links back to it.

Repository contract, cross-area and guard decisions may be autonomous: record
the rationale, alternatives and acceptance checks, notify affected owners, and
obtain independent exact-head review before guarded merging. No per-PR human
approval is required for these repository decisions. Spending, live flight,
deployment and access changes remain manual. Missing physical measurements,
organizer permissions and safety evidence remain blockers, not assumptions.

`docs/field-guide.html` is background and reasoning for Stanford DroneHacks —
schedule, scoring, rulebook digest, lessons learned. It is not authoritative.
The numbers code must agree on are extracted here: `contracts/frames.md`,
`contracts/course.md`, `contracts/px4.md`, `contracts/ros-graph.md`.

## Organizer source evidence

The organizer snapshot is **not included in this PR branch**. References to
`drone-snapshot/home/dronehacks/skybeta/` in these contracts name the archived
copy at repository commit `1aebf63daf9a0c6d9471df44138d1ac637d2da96`, not a
local dependency or permission to modify the organizer stack. Review it through
the immutable links below; no snapshot copy or hardware access is needed.

| source | Git blob ID | evidence used |
|---|---|---|
| [Organizer README](https://github.com/moterodiaz/DroneHacks/blob/1aebf63daf9a0c6d9471df44138d1ac637d2da96/drone-snapshot/home/dronehacks/skybeta/README.md) | `ea279b0dccca687abb5f01a82e82f29af8d8567a` | no interface edits/raw PX4 publication; pilot arming/Offboard sequence; startup-relative frames |
| [Odometry bridge](https://github.com/moterodiaz/DroneHacks/blob/1aebf63daf9a0c6d9471df44138d1ac637d2da96/drone-snapshot/home/dronehacks/skybeta/src/oakd_vslam_bringup/oakd_vslam_bringup/px4_odom_bridge.py) | `427844419267c95d2b47adc741c60c95f2bd61e4` | direct cuVSLAM input; world-fixed `POSE_FRAME_FRD` default |
| [Frame conversion source](https://github.com/moterodiaz/DroneHacks/blob/1aebf63daf9a0c6d9471df44138d1ac637d2da96/drone-snapshot/home/dronehacks/skybeta/src/skybeta_flight_interface/skybeta_flight_interface/frames.py) | `b2b1f4558e64195bb77bf3662cbe6b6e374ad5ff` | separate world/body conversions and optional origin/yaw alignment |
| [Flight interface source](https://github.com/moterodiaz/DroneHacks/blob/1aebf63daf9a0c6d9471df44138d1ac637d2da96/drone-snapshot/home/dronehacks/skybeta/src/skybeta_flight_interface/skybeta_flight_interface/flight_interface.py) | `e8340178b0438e4700c74e44cb2ce3f1e32a472f` | 10 Hz default, pose/twist inputs, 1 s timeout, altitude/speed clamps and anchoring defaults |

These paths/blob IDs were checked against that commit and the GitHub contents
API. They make the published-source claims reproducible; they do not establish
the configuration of the running kit, flight safety or a live organizer approval.
The README's fleet firmware reference and the older simulator version pin differ;
verify the actual issued kit before relying on either.
