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

Course point `(east, north)` → PX4 setpoint:

1. Subtract the launch mark: `(east − (−2.515), north − 0.914)`.
2. Rotate −90° so north becomes forward.
3. `z = 0.8 m − body_resting_height`. `body_resting_height = HEIGHT_M = 0.73`
   in the FlyX lab (tape-measured). Source: `measured`.

Any code claiming to implement this transform must cite this section. A
different rotation, a different sign, or a different height source is a
contract break, not a bug fix — open a PR against this file first.

## Acceptance check

The bench carry test: hold the drone level at the launch mark, facing north,
and walk it to a known course point by hand. The computed setpoint must match
that point within the course's survey tolerance (`contracts/course.md`). Any
code that publishes vision or setpoints must pass this check before flying.
