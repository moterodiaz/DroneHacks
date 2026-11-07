# PX4

Reasoning and background: `docs/field-guide.html` §kit and §build. This file
is the normative statement; where they disagree, this file wins
(AGENTS.md §3.1). See also AGENTS.md §3.6.

## Writable parameter allowlist

Rule 8.5: never touch the organisers' PX4 safety settings. Everything not
listed here — failsafes, kill switch, geofence, battery limits — is theirs
and frozen. Save their `.params` baseline (QGroundControl › Parameters ›
Tools › Save to file) before any change.

| parameter | value | why | source |
|---|---|---|---|
| `EKF2_EVA_NOISE` | `0.01` | PX4 1.16 uses this noise unsquared, weighting vision yaw 5× weaker than 1.18 | `measured` |
| `EKF2_EV_DELAY` | `0` | Simulator-only result (vision innovations at 0 against their 70 in sim); their bridge already stamps at capture, so their `70` double-counts latency — kit and first-hover confirmation still pending | `measured` |
| `EKF2_EV_QMIN` | `0` (keep, do not raise) | Their bridge sends quality 0; a higher QMIN refuses every sample | `measured` |

Nothing else under `EKF2_*`, `COM_*`, `NAV_*`, `MIS_*`, or `MPC_*` may be
written by our code. The failsafe table below documents their values for
reference — do not set them.

## Failsafe behaviour (reference only — organisers' values, do not write)

| trigger | parameter | behaviour | source |
|---|---|---|---|
| Setpoints stop | `COM_OF_LOSS_T` | `1 s`; Position mode holds where it is, on vision | `organisers` |
| Vision lost too | `COM_POSCTL_NAVL` | `0`; falls to Altitude mode, holds height, drifts sideways | `organisers` |
| Transmitter link lost | `NAV_RCL_ACT` | `6`; **disarms and falls**, even in Offboard | `organisers` |
| Failsafe triggers | `COM_FAIL_ACT_T` | `5 s`; holds before acting | `organisers` |
| Takeoff mode | `MIS_TAKEOFF_ALT` | `2.5 m`; climbs over the walls — our start must command its own height | `organisers` |
| Kill switch | `COM_KILL_DISARM` | `5 s`; motors stop at once, reversible within window | `organisers` |
| Speed cap | `MPC_XY_VEL_MAX` | `1.0` m/s | `organisers` |
| Estimator fusion | `EKF2_EV_CTRL` | `9`; vision gives horizontal position and yaw | `organisers` |
| Height reference | `EKF2_HGT_REF` | `2`; rangefinder gives height | `organisers` |

## Version and transport pins

| item | value | source |
|---|---|---|
| PX4 firmware | `v1.16.1` (`ver all` on the MAVLink console) | `organisers` |
| ROS 2 | Humble | `organisers` |
| `px4_msgs` | the kit's copy for v1.16, never ours — some 1.16 topics carry `_v1` | `organisers` |
| DDS domain | `ROS_DOMAIN_ID` must equal PX4's `UXRCE_DDS_DOM_ID` (their file sets `1`) | `organisers` |

## Prohibition

Do not publish to `/fmu/in/*` directly. All setpoints go through
`skybeta_flight_interface`, which never arms or changes mode on its own
(their guide, quoted at `docs/field-guide.html` §architecture).
