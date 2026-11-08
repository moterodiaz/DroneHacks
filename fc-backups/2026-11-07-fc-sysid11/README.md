# FC backup, 2026-11-07: the Pixhawk 6C that reports sysid 11

On 2026-11-07, a Pixhawk 6C was connected by USB (COM3, VID:PID 3185:0038) to the
team laptop. This folder holds a read-only capture of it, taken before any change.
Nothing was written to the FC, and it was not rebooted.

**Which drone this is.** The FC says it is drone 11:

* its MAVLink system ID is 11;
* `MAV_SYS_ID` is 11;
* `UXRCE_DDS_DOM_ID` is 11.

The organizers' convention is `MAV_SYS_ID` = `UXRCE_DDS_DOM_ID` = drone number.
Our drone is drone 1 (AGENTS.md §1), so treat this as drone 11's configuration
until someone confirms which airframe it came from.

## Files

| file | what it is |
|---|---|
| `fc-sysid11-params-2026-11-07.params` | All 1108 parameters, in QGC format (QGC › Parameters › Tools › Load from file reads it). Floats are printed as the shortest decimal that round-trips the float32. |
| `fc-sysid11-params-2026-11-07.json` | The same parameters, each with its raw 32-bit word, MAVLink type and index, so a restore can be checked bit for bit. Also includes PX4's `_HASH_CHECK` entry. |
| `fc-sysid11-identity-2026-11-07.json` | The HEARTBEAT and `AUTOPILOT_VERSION` fields: firmware/OS versions, board, vendor/product, and the MCU UID `3833496850778372145`. |
| `fc-sysid11-shell-output-2026-11-07.txt` | Output of `ver all`, `param status`, the SD-card listing, `mtd status`, `df` and `free` over the MAVLink shell. |
| `px4_fmu-v6c_default-v1.16.2-1-g7a39c59.px4` (+ `.sha256`) | The firmware this FC runs: the organizers' patched v1.16.2 + NuttX TX-DMA fix. The same bytes as `drone-snapshot/home/dronehacks/skybeta/px4/firmware/`. |

The parameters were read twice in full over MAVLink. The two passes agree bit for bit.

## How we know the firmware file is the one on the FC

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
