# Frames

Reasoning and background: `docs/field-guide.html` §architecture, "Frames, before
anything flies". This file is the normative statement; where they disagree,
this file wins (AGENTS.md §3.1).

## Frame table

| frame | convention | x | y | z | timestamp | source |
|---|---|---|---|---|---|---|
| PX4 | NED / FRD | north / forward | east / right | down | microseconds | `organisers` |
| ROS | ENU / FLU | east / forward | north / left | up | ROS clock | `organisers` |

A 90° mixup between these two is the failure mode this file exists to kill.

## Origin

The tracker's frame starts at the drone's body resting on the launch mark
`(−2.515, 0.914)` (course `(east, north)`, metres), facing north up the first
corridor. Source: `organisers` (launch mark), `surveyed`/`measured` once the
team places and checks it.

## The transform (normative)

Course point `(east, north)` → PX4 NED/FRD setpoint:

1. Subtract the launch mark: `(east − (−2.515), north − 0.914)`.
2. Rotate −90° so north becomes forward. Output frame is PX4 NED/FRD (x
   forward, y right, z down). Explicit axis map with signs:
   `world north → PX4 +x (forward)`, `world east → PX4 +y (right)`.
3. `body_resting_height = 0.07 m` (tape-measured, FlyX lab). Source:
   `measured`. `z = 0.8 m − 0.07 m = 0.73 m (= HEIGHT_M)`. PX4 NED
   z-positive is down, so the down setpoint is `−HEIGHT_M = −0.73 m`.

**Worked example — landing pad `(−2.042, −0.792)`:**

Step 1 — subtract launch mark `(−2.515, 0.914)`:

    Δeast  = −2.042 − (−2.515) = +0.473 m
    Δnorth = −0.792 − 0.914   = −1.706 m

Step 2 — axis map (drone faces north; PX4 NED/FRD output):

    forward (PX4 +x) = Δnorth = −1.706 m
    right   (PX4 +y) = Δeast  = +0.473 m

Step 3 — height (body_resting_height = 0.07 m; desired floor clearance = 0.8 m):

    down    (PX4 +z) = −HEIGHT_M = −0.73 m

PX4 NED/FRD setpoint: (−1.706, +0.473, −0.73) m.
Verify via the bench carry test in the Acceptance check section below.

Any code claiming to implement this transform must cite this section. A
different rotation, a different sign, or a different height source is a
contract break, not a bug fix — open a PR against this file first.

## Acceptance check

The bench carry test: hold the drone level at the launch mark, facing north,
and walk it to a known course point by hand. The computed setpoint must match
that point within the course's survey tolerance (`contracts/course.md`). Any
code that publishes vision or setpoints must pass this check before flying.
