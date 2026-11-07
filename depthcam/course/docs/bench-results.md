# Bench results: Jetson + Pixhawk 6C + OAK-D S2 + MS200K (2026-10-31 to 2026-11-04)

What happened when [hardware-setup.md](hardware-setup.md)'s bench tests ran on the real parts, without
a frame or motors: what had to be fixed to get there, every test's numbers, the causes found and fixed
in the code, and what is still open.  Newest state first; the details follow.

## Where things stand

| | State |
|---|---|
| Lidar -> Jetson -> PX4 position | **Works on the bench.** PX4 agrees with the lidar to 0.6 cm rms (2.1 cm max) walking a 1 m square; it locks on 0.5 s after the bridge starts |
| Lidar -> PX4 timing | ODOMETRY age p99 38 ms live (was 98 ms; EKF2_DELAY_MAX is 100); 8-iteration cap bounds a match at ~50 ms |
| Tracker safety | A wrong fit is now "lost" (not sent to PX4) and a start off the mark is acquired; it sent a false lock for a minute before the fix |
| Lidar localisation vs taped marks | 5-12 cm, limited by placing a hand-held rig on a mark (S read 10.8 cm apart on two visits); not yet measured to the 3 cm target |
| cuVSLAM, rig still | 100 % tracked in both modes (after the warm-up IMU fix) |
| cuVSLAM, carried | Tracks walking where the camera sees texture; fails on blank walls (any visual tracker would); `vio` resets after camera frame gaps |
| cuVSLAM live `vio` | Aborts its process (a cuVSLAM 17.0.0 background-thread bug); the live view uses `stereo` and restarts cuVSLAM if it dies |
| OAK-D frame drops | Only when the rig is carried (never on the table, under load, with the cable flexed, or panned); after securing the cable, none while walking, but still in the first 5 s of one run |
| Not run | Test 7 (pull the lidar's USB), Test 9 (arm + Offboard), anything in flight |

## Next steps

1. **OAK-D IMU calibration** (datasets/fit_imu_calibration.py, 60 s still on each of the 6 faces): the
   accelerometer reads gravity as ~9.1 m/s^2 and the noise densities in sensor.yaml are NVIDIA's
   RealSense placeholders.  `vio` should then ride through short frame gaps instead of resetting.
2. **Find the remaining early frame drops**: check whether they follow handling at the start of a run.
3. **A gap-free walking recording, both cuVSLAM modes offline**: the drift figures so far include
   position lost across resets.
4. **Test 5 to 3 cm**: the rig on a fixed-height stand set on each mark, or a plumb line, instead of
   hand-held overhead.
5. **Test 8 again** with the tracker guards, and the bridge recording its raw lidar stream so a
   failure can be replayed (19 does not record yet).
6. Tests 7 and 9, then the drone: stage 2 of [hardware-setup.md](hardware-setup.md).

## Jetson setup: what the guide did not say

JetPack 6.2.1 (L4T R36.4.7), Orin Nano devkit, MAXN_SUPER (`sudo nvpmodel -m 2`; the default 25 W mode
caps the CPU at 1344 MHz and pushed the bridge's p99 from 27 to 36 ms under cuVSLAM load).

- **No CH340 driver in JetPack 6** (`CONFIG_USB_SERIAL_CH341 is not set`): the MS200K's adapter enumerates
  (1a86:7523) but no /dev/ttyUSB appears.  Build the module against the installed headers:
  ```
  mkdir ~/ch341-driver && cd ~/ch341-driver
  curl -fsSL -o ch341.c "https://git.kernel.org/pub/scm/linux/kernel/git/stable/linux.git/plain/drivers/usb/serial/ch341.c?h=v5.15.148"
  printf 'obj-m := ch341.o\n' > Makefile && make -C /lib/modules/$(uname -r)/build M=$PWD modules
  sudo install -D -m 644 ch341.ko /lib/modules/$(uname -r)/updates/ch341.ko && sudo depmod -a && sudo modprobe ch341
  ```
  Rebuild after any kernel update.  udev name: `SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="ms200", MODE="0666"`.
- **The devkit's USB-C port is device-mode only** (the l4tbr0 network to a PC): a Pixhawk there never
  enumerates.  Use the USB-A ports.
- **Check cables carry data**: the Pixhawk's and the lidar's original cables were power-only (the
  devices powered up, the kernel logged nothing).
- **Keep the lidar and the camera off USB 2 splitters.**  The OAK-D on the Huasheng splitter ran at
  USB 2 (`usb=HIGH`); the splitter later failed (one port passed power but no data, and the lidar's
  CH340 froze with `-110` / `-71` errors).  Both now sit on direct ports; the Pixhawk, which only needs
  power there, can use the splitter.
- **jtop**: pip no longer runs jetson-stats' post-install step, so `jtop.service` never appears;
  `sudo jtop --install-service` creates it.
- **mavlink-router**: built from source (meson, ninja), config as in hardware-setup.md section 4 with
  `Device = /dev/ttyTHS1` (TELEM2 on header pins 8/10 is ttyTHS1 on this board; heartbeats verified at
  921600 with 0 bad frames).
- ModemManager disabled, brltty removed, user in `dialout`.

## Pixhawk 6C

- Came with PX4 v1.16.0; flashed **v1.17.0** from the Jetson over USB (the release's
  `px4_fmu-v6c_default.px4`, sha256 checked, with that tag's `Tools/px_uploader.py`); calibration kept.
- Parameters set through the PX4 console over MAVLink SERIAL_CONTROL (`param set ...`, as in
  hardware-setup.md 4.4), not PARAM_SET (pymavlink mis-encodes integer parameters).
  `EKF2_DELAY_MAX` 60 reads back as 100: PX4 raises it.
- **Accelerometer recalibrated**: the old calibration measured gravity as 10.70 m/s^2 (9 % off) and
  EKF2's tilt was 15-20 deg off the accelerometer's.  After QGC's accel + gyro calibration: 9.807 m/s^2.
- **Level Horizon** on the level rig (`commander calibrate level`): SENS_BOARD_X/Y_OFF +0.14 / -2.28 deg,
  replacing +14.02 / +3.58 (a level calibration done with the board held tilted) and before that
  -6.43 / -3.81.  Reads roll +0.2, pitch -0.2 deg after a reboot.  Redo it whenever the Pixhawk moves.

## Lidar yaw offset

Measured by fitting lines to the walls in a 5 s recording with the arrow square to a wall about 1 m
ahead: the front wall's perpendicular gives the offset, a side wall 90 deg from it checks the sweep
direction.  Far more precise than a finger in front of the sensor (that read 99 deg).  It moved
whenever the rig was rearranged: 89.5 -> 81.5 (the lidar turned on its standoff) -> **87.4** (current;
no `--counter-clockwise`).  Fix the lidar so it cannot turn: every degree is ~1.7 cm of error per metre.

## Tests

| Test | Result |
|---|---|
| 1 Pixhawk <-> Jetson | Pass: heartbeats through mavlink-router on 14540 and 14541 |
| 2 Lidar alone | Pass: 10.0 Hz, 447 points/turn, 99 % returns, 0 CRC errors, USB jitter p95 1.9 ms |
| 3 Camera alone | Pass: USB 3, 99.9 % tracked (`stereo`), 9.5 ms per frame |
| 4 Room map | Built by walking instead of measuring (21_room_map.py, below) |
| 5 Lidar position | 0 of 607 revolutions lost; marks 5.3-11.8 cm from 1 m (hand placement; see above); heading steady at ~92.5 deg |
| 6 Lidar -> PX4 | Pass: PX4 vs lidar 0.6 cm rms / 2.1 cm max; age p50 28 ms (p99 then 98 ms, fixed below) |
| 7 Losing the lidar | With synthetic scans: PX4 stops fusing ~1 s and marks xy invalid ~5 s after the bridge stops; USB pull not yet done |
| 8 Everything at once | Failed: the tracker false lock (fixed below) and cuVSLAM resets while carried (below) |

Test 6's timing part also ran with synthetic scans (`course/bench/bridge_static_sim.py`): age p99
23 ms alone, 25.8 ms with cuVSLAM `vio` running (after the fixes).

## Mapping a room by walking it (course/21_room_map.py)

The tape-measure map (20_bench_map.py) needs a bare rectangular room; 21_room_map.py records a walk
(lidar + PX4 attitude) and builds the map by 2D lidar SLAM (course/room_mapper.py).  In simulation every
face lands within 1.7-5.3 mm.  Lessons from three real rooms:

- **Map and test at the same height.**  A square test held at a different height from the mapping walk
  matched nowhere in the map (best 9.7 cm rms anywhere) and lost 59 % of revolutions.  Overhead with
  arms locked (H 2.5 m, scan plane 2.535 m) is repeatable, stays above the person and above doorways.
- **Hold still for 5 s at the start**: the first revolution is the map's origin (the tool warns if the
  rig moved > 1 cm in the first second).
- **Build without --closed unless the walk ended exactly on the start pose**: ending 14 cm / 3.6 deg off
  by hand, --closed bent walls by up to 3.5 cm; the open build checked at 0.21 cm against the walk.
- Give the real ceiling height (`--ceiling`); keep everyone else out of the room.

## Lidar -> PX4 timing (fixed)

Test 6's age p99 was 98 ms, max 104 ms: about 1 revolution in 20 took ~70 ms longer.
`course/bench/bridge_step_timing.py` put all of it in the matcher (87-92 ms; the PX4 queries, sending,
GC and the OS were all small).  Gauss-Newton was bouncing between two fits 0.3-0.7 mm apart (each step
undoing the last) until its 15-iteration cap.  Fixes: stop at the midpoint of such a 2-cycle (commit
8a8e67f), and cap the bridge at 8 iterations (3c1e67d; bench recordings never needed more than 7).
Live: p99 99 -> 38 ms, revolutions over 60 ms 27 -> 1 of ~700; poses unchanged (<= 0.38 mm).

## Tracker false lock (fixed)

In Test 8 every match was rejected from the start although it fit (3 cm, 444 of 445 beams): the
tracker's guess drifted with PX4's velocity, meaningless before PX4 fuses the lidar (~0.1 m/s), until
good matches were "too far" from it.  Once the rig moved, it accepted a wrong fit near the drifted
guess (24 cm rms, 14-17 % of the beams) and sent it to PX4 for a minute, because it only checked how
far a match moved.  `LIVE_GUARDS` (commit 628837e), used by the bridge and 18 localize: a fit over
10 cm or under 50 % of the beams is lost; a good fit up to 1 m / 45 deg is accepted until the first
trusted match and after 5 lost in a row; the guess holds while lost; the bridge waits for 3 good
matches after a re-acquisition and logs each match's own result and rejection reason.

## cuVSLAM

- **Live `vio` diverged at start-up** (37 % tracked): the 1 s IMU backlog registered after the 60
  warm-up frames.  Window set to 0 (commit 324196a): 100 % tracked, 4 mm drift in 30 s, rig still.
- **Blank walls**: walking the square overhead, the camera looked at a large white wall; 24 % of the
  frames had < 50 trackable corners and every reset fell in that stretch (`stereo` 100 % but 0.74 m
  end offset on a 4 m square; `vio` 92 %).  Not cuVSLAM's fault; any visual tracker needs texture.
- **Table loop at chest height** (texture everywhere): both modes track walking; both reset at one
  corner where a 58 deg/s turn met a 433 ms camera gap.
- **OAK-D frame drops**: lost at the camera (its sequence counter skips; the recorder's queue and the
  IMU were fine; chip 40 C).  Never on the table: idle, under bridge + `vio` load, with the cable
  flexed, or panned (0 lost frames each).  Only when carried.  After securing the cable: no gaps while
  walking two loops, 4 extra gaps in the first 5 s, and `vio` reset 6 times right after them (it does
  not yet ride through gaps: see the IMU calibration above).  The kernel's "Disable of device-initiated
  U1/U2 failed" appears only when the camera is closed, not during streaming.

## cuVSLAM aborts in the live view (2026-11-06)

"Camera stops returning new frames" in 22_live_localization.py was the cuVSLAM process dying: a background
thread in libcuvslam.so throws `std::out_of_range` (`_Map_base::at`), nothing catches it, and the process
aborts (SIGABRT; the message is in the run's console.log, not vio.log).  It happens only with live `vio`
(bundle adjustment on cuVSLAM's background thread, async_sba); no offline replay ever crashed.
Rig still, 145 s runs, the full live view (no-map lidar -> PX4, cuVSLAM, window):

| cuVSLAM | Aborted | Tracked | ms/frame mean / max | Camera gaps |
|---|---|---|---|---|
| `vio`, background bundle adjustment | 2 of 4 (34 s, 50 s) | 99.7 % | ~17 / ~780 | 2-3 |
| `vio`, bundle adjustment in the tracking thread | 0 of 3 | 99.9-100 % | ~29 / 830-920 | 25-34, up to 1.2 s |
| `stereo`, background bundle adjustment | 0 of 3 | 100 % | 12-14 / 46-70 | 1, at start-up |

The live view now defaults to `stereo`, and restarts a dead cuVSLAM (live_view.VioSupervisor) and places it
again at the lidar pose.  Worth reporting to NVIDIA (cuVSLAM 17.0.0, Jetson, Inertial mode, async_sba).
Separately, the OAK-D's firmware crashed once while being closed after a run (depthai "Device ... has
crashed", error 9001, a crash dump in ~/.cache/depthai/crashdumps); it recovers on the next start.

## Tools added for these tests (course/bench/)

| Tool | Use |
|---|---|
| `bridge_static_sim.py` | the bridge against real PX4 with a fixed synthetic rig: timing and EKF2 fusion with no room |
| `bridge_step_timing.py` | the live bridge with every step of every revolution timed (steps.csv, stalls.csv) |
| `square_stops.py` | Test 5 from a recording: localise it, find the stops, compare each with S |
