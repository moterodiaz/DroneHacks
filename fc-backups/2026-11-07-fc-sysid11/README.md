# skybeta-11's flight controller, 2026-11-07: before and after the DH Bailout flash

On 2026-11-07, skybeta-11's Pixhawk 6C was connected by USB (COM3, VID:PID 3185:0038)
to the team laptop. This folder holds the two firmware images and two read-only
captures of the FC:

* **before** (this folder): the organisers' firmware `v1.16.2-1-g7a39c59` and all
  parameters, captured before any change. Nothing was written and the FC was not
  rebooted.
* **after** ([`post-flash/`](post-flash/)): a team member flashed the DH Bailout
  firmware `v1.16.2-2-g5cc214d` (source: `firmware/bailout`, PR #46), and the same
  capture was repeated. It includes a parameter compare against the "before" dump.

To go back, flash `px4_fmu-v6c_default-v1.16.2-1-g7a39c59.px4` with QGC's
custom-firmware option. The parameters are on the FC's EEPROM and survive a flash.

**Which drone this is.** The FC says it is drone 11:

* its MAVLink system ID is 11;
* `MAV_SYS_ID` is 11;
* `UXRCE_DDS_DOM_ID` is 11.

The organizers' convention is `MAV_SYS_ID` = `UXRCE_DDS_DOM_ID` = drone number. The
team confirmed the same day that it flies drone 11 (AGENTS.md §1, PR #43).

## Files

| file | what it is |
|---|---|
| `fc-sysid11-params-2026-11-07.params` | All 1108 parameters, in QGC format (QGC › Parameters › Tools › Load from file reads it). Floats are printed as the shortest decimal that round-trips the float32. |
| `fc-sysid11-params-2026-11-07.json` | The same parameters, each with its raw 32-bit word, MAVLink type and index, so a restore can be checked bit for bit. Also includes PX4's `_HASH_CHECK` entry. |
| `fc-sysid11-identity-2026-11-07.json` | The HEARTBEAT and `AUTOPILOT_VERSION` fields: firmware/OS versions, board, vendor/product, and the MCU UID `3833496850778372145`. |
| `fc-sysid11-shell-output-2026-11-07.txt` | Output of `ver all`, `param status`, the SD-card listing, `mtd status`, `df` and `free` over the MAVLink shell. |
| `px4_fmu-v6c_default-v1.16.2-1-g7a39c59.px4` (+ `.sha256`) | The firmware this FC runs: the organizers' patched v1.16.2 + NuttX TX-DMA fix. The same bytes as `drone-snapshot/home/dronehacks/skybeta/px4/firmware/`. |

The parameters were read twice in full over MAVLink. The two passes agree bit for bit.

## After the flash (`post-flash/`)

| file | what it is |
|---|---|
| `px4_fmu-v6c_default-v1.16.2-2-g5cc214d-dh-bailout.px4` (+ `.sha256`) | The firmware flashed: the organisers' `v1.16.2-1-g7a39c59` plus one commit adding the `dh_bailout` module. It was built with their toolchain (gcc 9.3.1), and its `parameter_xml` is identical to theirs. sha256 `8756dc4c…eb78` |
| `fc-sysid11-identity-postflash-2026-11-07.json`, `fc-sysid11-shell-output-postflash-2026-11-07.txt` | `AUTOPILOT_VERSION` and `ver all`: PX4 git hash `5cc214d5f7d76e28a04a9fd8a7ae4b7a5d51d5a3`, built `Nov  7 2026 22:15:23`, GCC 9.3.1 20200408. This matches the image above |
| `fc-bailout-status-postflash-2026-11-07.txt` | `dh_bailout status` and `commander status`: registered as External Mode 1 (`nav_state` 23, "DH Bailout") and answering arming checks |
| `fc-sysid11-params-postflash-2026-11-07.params` / `.json` | All 1116 parameters after the flash, two identical passes |
| `param-compare-vs-preflash-backup.txt` | `firmware/bailout/tools/fc_params_compare.py` against the "before" `.json` |

What the compare shows:

* **Expected, from the firmware.**
  * `COM_MODE0_HASH` = -1819584171, the hash of "DH Bailout". PX4's commander writes it
    the first time the mode registers, so the External Mode index stays fixed.
  * `COM_MODE1..7_HASH` = 0. These are listed now because commander looks them up once
    an external mode registers. That is why the count went from 1108 to 1116.
* **A flight counter:** `COM_FLIGHT_UUID` 40 → 42, from two arming cycles since the
  "before" capture.
* **Six accelerometer offsets** (`CAL_ACC0/1_X/Y/ZOFF`) changed by up to 0.074 m/s².
  The flash did not do this. PX4 v1.16 saves the accelerometer bias EKF2 learned in
  flight into these offsets at disarm (`src/modules/sensors/vehicle_imu/VehicleIMU.cpp`).
  That is enabled on this FC (`SENS_IMU_AUTOCAL` 1, `EKF2_IMU_CTRL` 7).
  * Only offsets changed, never the scale factors, which fits learned bias rather than
    a recalibration.
  * It fits the two arming cycles above.
  * They are the FC's own refined calibration, and they were left as they are.
* **Nothing else differs.**
* `COM_FLTMODE1` is still `0` (Manual). DH Bailout is not on the mode switch yet.

## Before the flash: how we know the firmware file is the one the FC ran

PX4 cannot read its program flash back out over USB, so the image here is the
organizers' vendored file. The FC matches it in every field that can differ
between their builds:

| | FC (`ver all`, `AUTOPILOT_VERSION`) | vendored `.px4` |
|---|---|---|
| PX4 git hash | `7a39c5954139c917d21e0c5cdf6cc10449b1c011` | `7a39c5954139c917d21e0c5cdf6cc10449b1c011` |
| NuttX git hash | `c21890c6c91d09a648308644094bd86817231682` | `c21890c6…` (BUILD_NOTES.md) |
| build date/time | `Oct 20 2026 23:49:18` | the embedded strings `Oct 20 2026` and `23:49:18`; `build_time` 2026-10-21 06:49:33 UTC |
| toolchain | GNU GCC 9.3.1 20200408 | 9.3.1 20200408 |
| board | PX4_FMU_V6C, `board_version` 0x38 | `board_id` 56 (0x38) |

The organizers' notes say rebuilds of this source differ only in the embedded
date and time. So the same hash plus the same build second means the same build.

A stricter check, comparing the bootloader's flash CRC with this image, was not
run, because it reboots the FC.

## Not captured

* `/fs/mtd_caldata`, `/fs/mtd_mft_rev` and `/fs/mtd_id` on the FC's EEPROM. Parameters
  live in `/fs/mtd_params`, which is captured above through the parameter protocol.
* The SD card's files: `dataman`, `parameters_backup.bson`, `param_import_fail.*` and
  the `log/` directory. Only the listing was captured. Flashing does not erase the SD card.
