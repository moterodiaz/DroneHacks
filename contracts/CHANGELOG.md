# Contracts changelog

Newest entries first. One entry per contract-affecting change, linking the PR.

## 2026-11-06 — ENU competitor output, course goal schema, Q1/Q2 answered
Fixes PR #5's two open review findings: `contracts/frames.md` now states the
ENU output path to `/skybeta/setpoint/pose` (the NED/FRD transform is
log/bench-only — the kit's `skybeta_flight_interface` does that conversion
internally); `contracts/course.md`'s goal schema now defines `params` for
`tunnel`, `window`, and `launch`, plus the reject-if-missing rule.

### Findings
- 2026-11-06 — Q1 and Q2 (below) are answered by the kit's own published
  docs, mirrored read-only into `drone-snapshot/home/dronehacks/skybeta/`
  (`README.md`, `CLAUDE.md`) — not a live organiser answer, but the
  organisers' own source of truth. **Q1:** the kit's `px4_odom_bridge.py`
  consumes `/visual_slam/tracking/odometry` directly; there is no guard
  node upstream of it, and team code never touches `/fmu/in/*`. **Q2:**
  no — `skybeta_flight_interface` must not be modified (stated explicitly
  in `skybeta/README.md`). The 50 Hz led-pose design is dropped; team code
  publishes ENU setpoints to `/skybeta/setpoint/pose`/`/skybeta/setpoint/twist`
  at whatever rate `skybeta_flight_interface` expects (see
  `contracts/ros-graph.md`).

## 2026-11-06 — first four contracts, extracted from the field guide
Added `contracts/frames.md`, `contracts/course.md`, `contracts/px4.md`,
`contracts/ros-graph.md`. Extracted from `docs/field-guide.html` (the team's
background/reasoning doc for Stanford DroneHacks, committed the same PR).
Notified: all area owners (none claimed yet — see docs/BOARD.md).

### Findings
Dated, `measured` on the x500 stand-in sim unless noted. A finding is a
contract too — once recorded here, code may rely on it.

- 2026-11-06 — cuVSLAM answers confidently on a blank view (hundreds of
  features, "standing still") instead of reporting loss. Withholding blank
  frames alone still produced a 2.2 m jump on return (149 cm rms); adding
  re-anchoring onto PX4's estimate brought it to 24.5 cm. See
  `contracts/ros-graph.md` guard section.
- 2026-11-06 — stereo-only cuVSLAM beats its inertial mode in flight:
  inertial mode's in-flight IMU init blocks tracking (one break per run);
  stereo-only ran 5 of 5 clean. PX4's EKF2 already does the inertial fusion.
- 2026-11-06 — latency is the dominant cost, and the organisers'
  `EKF2_EV_DELAY 70` double-counts it (their bridge already stamps at
  capture). With 50 ms added latency, `70` touched a wall on 4 of 5 runs;
  keep `0` and confirm from the first hover's log. See `contracts/px4.md`.
- 2026-11-06 — corners set the time, not top speed: 1.0, 1.5 and 2.0 m/s all
  took 27–31 s launch to pad; 1.0 m/s was cleanest (0 of 5 touched). Fly at
  the kit's 1.0 m/s cap.

### Still open
The field guide's day-one questions for an organiser, verbatim. Each answer
lands here as a new dated finding when it arrives.

1. Who publishes vision to PX4: their bridge fed by our guard, or our own
   bridge?
2. May we fork the flight interface (the two-line fix) and raise its
   `setpoint_rate_hz` to 50? Or use the starter's advanced track, which takes
   velocity and acceleration directly?
3. Who arms and takes off for a scored run, and can our code re-arm for the
   return?
4. Can the camera's stereo depth run beside cuVSLAM in their `depthai-ros`
   launch?
5. What speed limit is set? At 1.0 m/s it costs us nothing.
6. Firmware: is it v1.16.1 (`ver all`)?
7. May we tape-measure the course as laid and plan on that map (rule 6.4)?
8. The tunnel: roughly how dark inside, and what is its floor? It sets the
   exposure cap and the guard's threshold.

## 2026-11-06 — contracts layer established
No contracts exist yet. This file and `contracts/README.md` establish the rule
(contracts beat code; CI's `contracts-guard` enforces this file gets touched
whenever `contracts/` does) ahead of the first real contract.
