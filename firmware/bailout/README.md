# DH Bailout: a bail-out flight mode inside the PX4 firmware

A flight mode for skybeta-11's Pixhawk 6C that the safety pilot selects with the RC
mode switch while autonomy is flying. It stops the drone before it hits something:

1. **Brake.** Zero velocity along the direction of travel, as a jerk-limited stop.
   The defaults are 3 m/s² (about 17° of tilt) building at 20 m/s³. A climb or sink
   is stopped at the same time.
2. **Settle.** Hold the stop point until every axis is slower than 0.15 m/s for 0.3 s
   (at most 1.5 s).
3. **Descend.** Hold x/y and heading, and descend at 0.7 m/s. Below 0.8 m on the
   rangefinder, or with no rangefinder, it slows to 0.35 m/s. PX4's land detector sees
   the touchdown and PX4 disarms `COM_DISARM_LAND` (5 s on drone 11) later.

It runs entirely on the flight controller. There is no ROS and nothing on the Jetson:
if the Jetson, the stack or the DDS link dies, the mode still works.

> **Status: built and tested, NOT flashed and NOT approved to fly.** Flashing is blocked
> by `contracts/px4.md` ("Scoped exception: the DH Bailout firmware"). The blockers are
> organiser permission, SITL at the flashed commit, and a props-off bench check, and a
> human must do the flash. See [Before it flies](#before-it-flies).

## How it is built into PX4

`src/modules/dh_bailout` is a PX4 module, compiled into the firmware and started at
boot from `boards/px4/fmu-v6c/init/rc.board_extras`. It registers with commander
through commander's **external-mode interface**: `register_ext_component_request`,
then arming-check replies and `config_control_setpoints`. That is the same interface
a ROS 2 external mode uses, but here it is a uORB-only module on the FC. Commander
then treats it like any external mode:

- it is **External Mode 1** (`nav_state` 23), named "DH Bailout" in QGroundControl
  (MAVLink `AVAILABLE_MODES`);
- `COM_FLTMODEx = 100` puts it on a mode-switch slot;
- `mode_req_prevent_arming`: **it cannot be armed in**;
- if it ever stopped answering, commander's own fallback (Land, then Descend) takes over.

This needs **no change to commander, failsafes, the mode list, MAVLink or `px4_msgs`**.
A new built-in mode would have meant editing about 12 switch statements across
commander, failsafe, MAVLink, OSD and telemetry, and a message the Jetson's `px4_msgs`
also reads.

**The kit's flight_interface keeps publishing `/fmu/in/trajectory_setpoint` at 10 Hz
in every PX4 mode** (`flight_interface.py` `tick()`), and PX4 v1.16 uses whichever
sample came last. While autonomy is still flying, that sample is its forward velocity
or next waypoint. The module subscribes to the topic and re-publishes its own setpoint
the moment a foreign sample appears. On the FC that gap is microseconds, not the
milliseconds a ROS node would leave. `dh_bailout status` counts the overrides.

### What it changes on the FC

| what | change | by whom |
|---|---|---|
| firmware | organisers' `v1.16.2-1-g7a39c59` + one commit → `v1.16.2-2-g5cc214d` | a human, QGC custom firmware |
| `COM_MODE0_HASH` | `0` → the hash of "DH Bailout", on the first boot | PX4's commander, so the External Mode index stays fixed |
| `COM_FLTMODE1` | `0` (Manual) → `100` (DH Bailout) | a human, by hand in QGC |
| anything else | nothing. The parameters live in the FC's EEPROM (`/fs/mtd_params`), which flashing does not erase. Check with `tools/fc_params_compare.py` | |

The module adds **no parameters**: its tunables are compiled in (`BailoutParams` in
`bailout_logic.hpp`). This keeps the parameter set identical apart from the two rows above.

### The mode switch on drone 11

Channel 6 (`RC_MAP_FLTMODE` 6). Drone 11 currently has slot 1 = Manual (0), 2/3/5 unassigned
(-1), 4 = Altitude (1), 6 = Position (2), so only slot 1 selects Manual. With
`COM_FLTMODE1 = 100`, the position that selected Manual selects DH Bailout. Altitude and
Position stay where they were, and so do the kill switch (ch 5) and arm switch (ch 7).

What the pilot loses: **there is no Manual (stabilised) position on the switch any more.**
If the estimate fails, PX4 still falls back by itself: Position → Altitude → stabilised.

## Flying with it

- **Engage:** flip the mode switch to the old Manual position. PX4 acts when the slot
  *changes*, so selecting it while already there does nothing.
- **Leave:** switch to Altitude or Position. The sticks do **not** take over in DH Bailout,
  because PX4's stick override (`COM_RC_OVERRIDE`) applies only to auto modes and
  Offboard. The kill switch always works.
- **Arming** is refused while the switch is on DH Bailout. Arm in Altitude, as now.
- **After it lands**, PX4 disarms 5 s later (`COM_DISARM_LAND`). Autonomy does not
  resume: flight_interface waits for a new disarm and arm.
- **Scoring:** a bail-out is a "land" abort (rules 5.1–5.3, 7.11).

## Limits

- **It lands where it stops.** That can be on an obstacle; it does not search for a clear spot.
- **No horizontal estimate, no brake.** After a long VIO dropout PX4 has no horizontal
  position or velocity. The mode then flies level and descends, drifting at whatever
  speed it had, as PX4's Descend does. While dead reckoning still has a velocity
  (`EKF2_NOAID_TOUT`), it brakes.
- **The braked speed is capped** at 2 m/s horizontally and 1 m/s vertically, so a
  glitched velocity cannot become metres of commanded flight. A faster real drone is
  still braked, by the velocity loop. Drone 11's FC itself caps nothing indoors
  (`MPC_XY_VEL_MAX` 12).
- **Estimator resets** (VIO re-anchoring, height, heading) move the reference with the
  drone. After a gap of more than 0.5 s in the position estimate, it holds where the
  drone is rather than flying back.

## Files

| path | what |
|---|---|
| `src/modules/dh_bailout/` | the PX4 module: `DhBailout.cpp/.hpp` (PX4 glue), `bailout_logic.hpp` and `brake_profile.hpp` (pure logic, no STL, also built by the host test) |
| `boards/px4/fmu-v6c/init/rc.board_extras` | `dh_bailout start` at boot |
| `src/modules/dh_fi_emulator/` | **SITL test only**: emulates flight_interface's 10 Hz stream. Its Kconfig `depends on PLATFORM_POSIX`, so it cannot be built into the Pixhawk image, and `build.sh` checks the image for it |
| `build.sh` | adds the module as one commit on the organisers' prepared PX4 tree and builds |
| `test/test_bailout_logic.cpp` | host unit and closed-loop tests of the logic |
| `sitl/sitl_bailout_test.py` | PX4 SITL (SIH) test of the whole mode |
| `tools/fc_params_compare.py` | read-only diff of the FC's parameters against a backup |

## Building

1. Prepare the organisers' source exactly as they do:
   `PREPARE_ONLY=1 drone-snapshot/home/dronehacks/skybeta/px4/firmware/txdma-fix/build_fmu-v6c_v1.16.2-txdma.sh <dir>`
   This gives `<dir>/PX4-Autopilot` at `7a39c595` (`v1.16.2-1-g7a39c59`), the commit drone 11's FC runs.
2. `firmware/bailout/build.sh <dir>/PX4-Autopilot px4_fmu-v6c_default [px4_sitl_default]`
   adds one commit (fixed author and date, so it is always `5cc214d` for this source) and builds.
   Use the organisers' toolchain (their `BUILD_NOTES.md`):
   - GNU Arm Embedded 9-2020-q2-update;
   - cmake 3.16.3 and ninja 1.10.0;
   - a minimal PATH.

   With a different compiler, every function in the image is compiled differently,
   not just this module.

### The image (2026-11-07)

`px4_fmu-v6c_default-v1.16.2-2-g5cc214d-dh-bailout.px4`:
- sha256 `8756dc4c602da057faa8b3d5494b9dc54d6172d72a63243b2faa438973c5eb78`;
- built with the organisers' toolchain;
- **1,965,372 of 1,966,080 bytes: 708 bytes free.** The organisers' image leaves 6,196.

The module is about 5.5 KB. Getting there took several cuts:
- no `ModuleBase`;
- no tunable-limit table in the NuttX build;
- no QGC text messages (QGC already announces the mode by name);
- short strings.

The first, unslimmed version overflowed the flash by 1,432 bytes with gcc 9.3.1.
gcc 10.3.1 makes the whole image about 14 KB smaller, but it would recompile every
module differently from the fleet's.

Compared with the organisers' `v1.16.2-1-g7a39c59.px4`:
- `parameter_xml` (the parameter definitions) and `airframe_xml` are **identical**, and so
  are the `.px4` keys;
- the only readable strings that differ are the version and hash, the build date, and the
  module's own strings ("DH Bailout", `dh_bailout start`, `rc.board_extras`, its status
  line).

The build fetches Micro-CDR v2.0.1 from GitHub (Micro-XRCE-DDS-Client's superbuild). It
was built offline here by pointing git's `url.<local>.insteadOf` at a local clone,
through `GIT_CONFIG_*` environment variables. The SITL build also wants PX4-OpticalFlow,
for the Gazebo plugins. The headless SIH test does not need it: `bin/px4` links before
that step.

## Tests

Logic, on the host (from `firmware/bailout`):

```bash
g++ -std=c++17 -Wall -Wextra -Werror -Isrc/modules/dh_bailout test/test_bailout_logic.cpp && ./a.out
```

This covers the stop profile, parameter clamping, setpoint well-formedness (PX4's
`PositionControl::_inputValid()` rules), estimator resets, stale and partial estimates,
and closed-loop flights against a PX4-like vehicle with drone 11's gains (tilt cap 45°,
5 s disarm).

PX4 SITL, with the SIH quad, headless:

```bash
python3 sitl/sitl_bailout_test.py --build <PX4>/build/px4_sitl_default [--speed 1] [--fi] [--target land]
```

`--fi` adds the flight_interface stream. `--target land` flies the same test with PX4's
stock Land instead, as a control. The stop is judged on SIH's **ground truth**
(`vehicle_local_position_groundtruth`), because SIH's GPS and baro estimator is far
noisier than drone 11's VIO and rangefinder. `sitl/compare_runs.py` tabulates all runs.

### SITL results (2026-11-07, SIH quad, Offboard at ~1 m/s, then a switch)

These checks pass in every run:
- it registers as External Mode 1, and `AVAILABLE_MODES` names it "DH Bailout";
- arming is refused in it;
- it stays in the mode until disarmed;
- its setpoint freezes at the braked stop point;
- the descent is 0.7 m/s, then 0.35 m/s below 0.8 m.

Ground truth, until the true horizontal speed first fell below 0.3 m/s. DH Bailout rows
are the final build (`5cc214d`); stock Land is unchanged by the module:

| mode | flight_interface stream | under 0.3 m/s | peak forward travel | touchdown, land detector, disarm |
|---|---|---|---|---|
| DH Bailout (3 runs) | no | **0.51–0.66 s, 0.37–0.39 m** | 0.44–0.48 m | 4.2–4.4 s; landed and disarmed in 3 of 3 |
| stock Land (2 runs) | no | 0.79–0.84 s, 0.54–0.59 m | 0.72–0.84 m | 4.4–4.8 s; landed and disarmed |
| DH Bailout (2 runs) | **yes** (1 m/s forward, 10 Hz) | 0.92–0.99 s, 0.38–0.39 m | 0.39–0.41 m | 3.8–4.0 s; land detector at 16 s and 28 s, disarm 5 s later |
| stock Land (1 run) | **yes** | 2.28 s, 1.22 m | 1.40 m | 4.3 s; **never detected landing, never disarmed** |

`sitl_bailout_test.py` passes every check in all five DH Bailout runs except one. In that
run the land detector fired at 27.6 s, so the disarm fell after the test's 30 s window.

What this shows:
- **DH Bailout stops in about half the time and distance of stock Land.**
- **With flight_interface streaming, stock Land is badly hurt:** the 10 Hz forward-velocity
  samples keep pushing it, and their `vz = 0` resets the land detector's "commanded
  descent" check every time. DH Bailout overrode every foreign sample (204–290 per run)
  and still braked in the same distance, landed and disarmed.
- In SITL some samples still got through, because PX4's work queues run without
  real-time priorities on Linux unless run as root. On the FC (NuttX), our re-publish
  runs on the high-priority `nav_and_controllers` queue right after the DDS client's
  publish, so the window is much smaller. It is not zero, so landing detection can
  still be slower while flight_interface runs. The clean fix is on the organisers'
  side: flight_interface should publish only in Offboard.
- **SIH's estimator, not the mode, causes the rest:**
  - after stopping, both modes wander 0.2–0.8 m;
  - in two DH Bailout runs, height drifted up to 0.3 m while held;
  - the land detector is slow in both modes.

  Both modes do the first and last, and the logs show our setpoint holding still while
  the estimate moves. Drone 11 estimates from VIO and the rangefinder, so expect less.
  SITL is evidence of behaviour, not of drone 11's numbers.

## Before it flies

`contracts/px4.md` blocks flashing until all of these hold:

1. The organisers' permission for this firmware and for `COM_FLTMODE1`, recorded in the
   contract (who and when), as was done for `COM_DISARM_LAND`.
2. The SITL test passes at the exact commit to be flashed.
3. Props off, after flashing:
   - QGC lists DH Bailout;
   - the switch selects it;
   - arming is refused in it;
   - switching back works;
   - `python tools/fc_params_compare.py COM3 contracts/px4-baselines/skybeta-11-2026-11-07.params --expect COM_DISARM_LAND=5.0 --expect COM_FLTMODE1=100 --expect COM_MODE0_HASH`
     reports no unexpected difference.
4. A human flashes it: QGC → Vehicle Setup → Firmware → Advanced → custom firmware file.
   To go back, flash `drone-snapshot/home/dronehacks/skybeta/px4/firmware/px4_fmu-v6c_default-v1.16.2-1-g7a39c59.px4`
   the same way.

Then a low, tethered hover: bail out from a hover, then from slow flight.
