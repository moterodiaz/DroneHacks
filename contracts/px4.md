# PX4

Reasoning and background: `docs/field-guide.html` §kit and §build. This file
is the normative statement; where they disagree, this file wins
(AGENTS.md §3.1). See also AGENTS.md §3.8.

## Writable parameter allowlist

Rule 8.5: never touch the organisers' PX4 safety settings. Everything not
listed here — failsafes, kill switch, geofence, battery limits — is theirs
and frozen. Save their `.params` baseline (QGroundControl › Parameters ›
Tools › Save to file) before any change, into `contracts/px4-baselines/`.
skybeta-11's, saved 2026-11-07 before `COM_DISARM_LAND` went to 5, is
[`skybeta-11-2026-11-07.params`](px4-baselines/skybeta-11-2026-11-07.params).

| parameter | value | why | source |
|---|---|---|---|
| `EKF2_EVA_NOISE` | `0.01` | PX4 1.16 uses this noise unsquared, weighting vision yaw 5× weaker than 1.18 | `measured` |
| `EKF2_EV_DELAY` | `0` | Simulator-only candidate (vision innovations at 0 against 70 in sim); capture timestamps do not eliminate pipeline latency. Measure kit latency and confirm in an approved first-hover log before relying on this value | `measured` |
| `EKF2_EV_QMIN` | `0` (keep, do not raise) | Their bridge sends quality 0; a higher QMIN refuses every sample | `measured` |
| `COM_DISARM_LAND` | `5` (skybeta-11 only; organisers' default `2`) | Auto-disarm 5 s after PX4 reports landed, so a touch-and-go can rest on the ground and lift off before the motors stop: lift-off took over 0.8 s in flight 20261108-013018, which a 2 s window left no room for. Set by hand in QGroundControl after saving their baseline, never by code | `organisers` (permission reported by the team, 2026-11-07) |
| `COM_FLTMODE1` | `100` (skybeta-11 only; organisers' value `0` = Manual) | Mode-switch position 1 (channel 6, `RC_MAP_FLTMODE` 6), which selected Manual, selects "DH Bailout" (External Mode 1 of the DH Bailout firmware below) instead. Positions 4 and 6 (Altitude, Position) and the kill and arm switches stay as issued, and no slot may hold Acro (§3.6). Requires the DH Bailout firmware; without it, value 100 selects nothing. Set by hand in QGroundControl after saving their baseline, never by code. **Blocked until the DH Bailout firmware exception below is cleared** | `team` (asked for by @jsun4231, 2026-11-07); organiser permission pending |

Nothing else under `EKF2_*`, `COM_*`, `NAV_*`, `MIS_*`, or `MPC_*` may be
written by our code. `COM_DISARM_LAND` above is a hand-set, organiser-approved
exception for skybeta-11; it does not open the rest of `COM_*`. Nor does
`COM_FLTMODE1`. With the DH Bailout firmware, PX4's commander itself writes
`COM_MODE0_HASH` the first time the mode registers, to pin its External Mode
index. That is PX4's own bookkeeping, not a write by our code. A parameter
diff after flashing shows it. The failsafe table below documents their values for
reference — do not set them.

## Scoped exception: the DH Bailout firmware (blocked)

`firmware/bailout` builds PX4 for skybeta-11's Pixhawk 6C as the organisers'
fleet firmware `v1.16.2-1-g7a39c59` plus one commit. That commit adds the
`dh_bailout` module and its start line, built with the organisers' toolchain.
The module registers one flight mode, "DH Bailout", through commander's external-mode interface; nothing runs on
the Jetson. Selected by the safety pilot, it brakes to rest, settles, then
descends until PX4's land detector and `COM_DISARM_LAND` disarm.
- It writes no parameter and sends no vehicle command.
- It cannot arm (`mode_req_prevent_arming`).
- It changes no failsafe, kill switch, geofence or battery behaviour.
- It is a pilot-initiated land abort (rules 5.1–5.3, 7.11), not the autonomous
  landing that stays blocked below.

Flashing it replaces the pinned firmware, so it is blocked until all of these hold:

1. The organisers have given permission for this firmware and for `COM_FLTMODE1`,
   recorded here with who granted it and when.
2. `firmware/bailout/sitl/sitl_bailout_test.py` passes at the exact commit to be
   flashed, with and without `--fi`.
3. A props-off bench check after flashing passes:
   - QGroundControl lists DH Bailout;
   - the switch selects it;
   - arming is refused in it;
   - switching back works;
   - `firmware/bailout/tools/fc_params_compare.py` against
     `px4-baselines/skybeta-11-2026-11-07.params` differs only in `COM_DISARM_LAND`,
     `COM_FLTMODE1`, `COM_MODE0_HASH` and the flight counters.
4. A human flashes it, props off, with QGroundControl's custom-firmware option. To go
   back, flash the organisers' vendored
   `px4_fmu-v6c_default-v1.16.2-1-g7a39c59.px4` the same way.

Source: `firmware/bailout/README.md`; PX4 v1.16.2 `src/modules/commander/ModeManagement.cpp`.

## Failsafe behaviour (reference only — organisers' values, do not write)

Values as set on our drone, skybeta-11, read from its FC over USB on 2026-11-07
(they match [`skybeta-11-2026-11-07.params`](px4-baselines/skybeta-11-2026-11-07.params)).
Until that day the team flew drone 1, whose 2026-10-15 dump (`SN0001`) had
`NAV_RCL_ACT` `6` and `MPC_XY_VEL_MAX` `1.0`. Those values are not ours any more.

| trigger | parameter | behaviour | source |
|---|---|---|---|
| Setpoints stop | `COM_OF_LOSS_T` | `1 s`; Position mode holds where it is, on vision | `organisers` |
| Vision lost too | `COM_POSCTL_NAVL` | `0`; falls to Altitude mode, holds height, drifts sideways | `organisers` |
| Transmitter link lost | `NAV_RCL_ACT` | `2`; requests Return. Return needs a global position, which this indoor drone never has, so PX4 falls back to Land, or to Descend without a local position | `organisers` |
| Failsafe triggers | `COM_FAIL_ACT_T` | `5 s`; holds before acting | `organisers` |
| Takeoff mode | `MIS_TAKEOFF_ALT` | `2.5 m`; climbs over the walls — our start must command its own height | `organisers` |
| Kill switch | `COM_KILL_DISARM` | `5 s`; motors stop at once, reversible within window | `organisers` |
| Speed cap | `MPC_XY_VEL_MAX` | `12` m/s, so the FC itself does not cap speed indoors; our code must | `organisers` |
| Estimator fusion | `EKF2_EV_CTRL` | `9`; vision gives horizontal position and yaw | `organisers` |
| Height reference | `EKF2_HGT_REF` | `2`; rangefinder gives height | `organisers` |

## Version and transport pins

| item | value | source |
|---|---|---|
| PX4 firmware | `v1.16.2-1-g7a39c59`, git `7a39c5954139c917d21e0c5cdf6cc10449b1c011`, built Oct 20 2026 23:49:18. This is the organisers' v1.16.2 plus the NuttX TX-DMA fix, vendored in `drone-snapshot/home/dronehacks/skybeta/px4/firmware/`. skybeta-11's `ver all`, 2026-11-07; drone 1 ran `v1.16.1` | `organisers` |
| ROS 2 | Humble | `organisers` |
| `px4_msgs` | the kit's copy for v1.16, never ours — some 1.16 topics carry `_v1` | `organisers` |
| DDS domain | `ROS_DOMAIN_ID` must equal PX4's `UXRCE_DDS_DOM_ID`. On skybeta-11 both are `11` (the FC's value, 2026-11-07; convention: drone number). Drone 1's were `1` | `organisers` |

## Prohibition

Do not publish to `/fmu/in/*` directly. All setpoints go through
`skybeta_flight_interface`. It has no arm path. With
`auto_offboard_on_arm:=true`, it enters Offboard once after the safety
pilot arms on the ground in Altitude and switches to Position, subject
to the kit's readiness checks. Pilot takeover prevents automatic re-entry
until disarm/rearm. The default altitude floor is 0.3 m; pose/twist input
is not a supported land/disarm/rearm API. Landing and autonomous return
after disarm remain blocked pending an organizer-supported procedure, except
for the scoped landing exception below.
Do not bypass the clamp, publish raw PX4 commands, or patch the interface.
Source: `drone-snapshot/home/dronehacks/skybeta/README.md` §Flight sequence
and the supplied `flight_interface.py`.

## Scoped exception: autonomous landing on the pad

The team (@jsun4231, 2026-11-08) lets `dronehacks_traj`'s window flight lines land on the pad by
themselves: `config/flight_line_window_max.yaml` and `config/flight_line_window_0p5mps.yaml` set
`land: true`. Nothing else may land autonomously. The default traj launches keep `land: false`,
which holds over the pad for the pilot.

- How it lands: a downward twist through the kit's interface, `/skybeta/setpoint/twist`, with a
  small horizontal pull back over the pad. It continues until PX4's land detector reports landed
  and `COM_DISARM_LAND` disarms. It never arms, disarms or changes mode, and it writes no
  parameter. This is the method `dronehacks_course_follower` uses on skybeta-11
  (`core/flight-iteration`).
- Evidence:
  - Flight `20261108-013018` on skybeta-11 disarmed on the ground after such a descent
    (`docs/flight-findings.md` on `core/flight-iteration`, measured).
  - The organisers announced on 2026-11-07 that drones have since taken off, flown and landed
    autonomously. This is reported by the team, in `dronehacks_course_follower/README.md` on
    `core/flight-iteration`.
- Still blocked: no land, disarm or rearm API, no autonomous return after disarm, and no other
  package or launch.
- **Record the organisers' confirmation here, with who gave it and when, before the first flight**:
  until then this exception rests on their announcement as the team reported it.

Source: `team` (@jsun4231, 2026-11-08); organisers' support reported by the team.
