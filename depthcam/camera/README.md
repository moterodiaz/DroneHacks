# Camera: OAK-D S2 with DepthAI v3

The ladder of small self-checking scripts (`00` to `06`: device info, RGB, stereo depth, point cloud,
IMU, record, replay) for the OAK-D S2 on DepthAI **v3** (`depthai==3.10.0`), plus `common.py` (device
open / retry / exit-code rules shared by every script in the repo) and the camera reference docs in
[docs/](docs/). Everything here runs on Windows and is meant to run unchanged on the Jetson. Setup is
in the [root README](../README.md).

## Common flags

All seven scripts share these (from `camera/common.py`):

| Flag | Meaning |
|---|---|
| `--device-id ID` | OAK device id to open (default: the first device found); an explicit id never retries, it fails at once with one stderr line. Only `06_replay.py --mode holistic` uses it in the replay script; `--mode video` never opens a camera and ignores it. |
| `--duration SECONDS` | Stop after this many seconds (default 0 = run until `q` / Esc / Ctrl-C). `05_record.py` also accepts `--seconds` as an alias. |
| `--headless` | No OpenCV windows or key handling; print one status line per second to stdout instead. `00` never opens a window (flag accepted, changes nothing); `04` only suppresses the `--plot` window. |

**No camera:** every script that needs the device exits with status **2** and one final human-readable line on stderr, never a traceback. An explicit `--device-id` never retries: it fails at once (~1-2 s) with exactly one line whether a camera is plugged in (`Device X not found (unplugged, or in use by another process). Found: OAKDS2EXAMPLEID000.`) or not (`No OAK device found. Check the USB-C cable and that no other program has the camera open.`). Only a run **without** `--device-id` retries: when nothing enumerates (unplugged, held by another process, or still rebooting after the previous script) the open is attempted three times, with a `Retrying in 3 s (1/3)...` / `(2/3)...` notice on stderr after each of the first two, before that same final `No OAK device found` line (about 8 s total). `--help` never touches the device.

Other exit codes: 0 normal, 1 pipeline/build error, 130 Ctrl-C outside the streaming loop (each script's docstring lists its own).

`07_record_dataset.py` takes the same three flags (`--seconds` is again an alias of `--duration`; there is no window unless `--preview`, so `--headless` is redundant but harmless). `08_cuvslam_track.py` has `--device-id` and `--duration` but no `--headless` (it opens no window unless `--show`) and adds exit code 3: `cuvslam` not importable and `--fake-cuvslam` not given.

## The test ladder

Run them in order the first time. Windowed mode is the default; add `--headless --duration N` for a fixed-length run without windows. Keys in the windows: `q` / Esc quit; `s` saves a snapshot in `01`, `02`, `03` and `06`.

| Script | What it verifies | Run | What you should see |
|---|---|---|---|
| `00_device_info.py` | Host sees, boots and can query the camera: identity, USB speed, firmware, IMU, camera sockets, stereo calibration. No pipeline started. | `python camera/00_device_info.py`<br>`python camera/00_device_info.py --json > device.json`<br>`python camera/00_device_info.py --duration 10` | `Device: OAK-D-S2 (OAK-D-S2-AF) id=... platform=RVC2 usb=SUPER`; CAM_A IMX378, CAM_B/CAM_C OV9282; IMU BNO086 3.9.9; baseline 75.00 mm spec / ~74.97 mm calibrated. |
| `01_rgb_preview.py` | `Camera` node on CAM_A delivers BGR frames at the requested size/FPS with no sequence gaps, small host latency, populated exposure/ISO/lens metadata, PNG snapshots. | `python camera/01_rgb_preview.py`<br>`python camera/01_rgb_preview.py --width 1920 --height 1080`<br>`python camera/01_rgb_preview.py --headless --duration 10 --snapshot-on-exit` | Window `01 RGB preview (CAM_A)` with FPS/timestamp/exposure overlay; FPS settles at 30.0 after ~1 s; exit summary reports frames, missing frames (expect 0) and snapshot paths. |
| `02_stereo_depth.py` | CAM_B/CAM_C through `StereoDepth` give uint16 mm depth at the requested FPS; presets and knobs (LR-check, subpixel, extended, median, confidence, decimation) apply and are read back; values are plausible. | `python camera/02_stereo_depth.py`<br>`python camera/02_stereo_depth.py --preset ROBOTICS --no-lrcheck --median 5 --decimation 1`<br>`python camera/02_stereo_depth.py --headless --duration 10 --snapshot-on-exit`<br>`python camera/02_stereo_depth.py --align-rgb --show-disparity` | Colour-mapped depth window (red near, blue far, black invalid) with a centre ROI and a mouse-follow ROI; one console line per second: `fps | valid % | min / med / max | center ROI`. A flat wall at 1.5 m should read ~1.45-1.55 m. |
| `03_point_cloud.py` | `StereoDepth` feeds `PointCloud`; points come back in metres with sane count, bounding box and centroid; optional RGB colouring and PLY export. | `python camera/03_point_cloud.py`<br>`python camera/03_point_cloud.py --color`<br>`python camera/03_point_cloud.py --headless --duration 10 --save-on-exit`<br>`python camera/03_point_cloud.py --max-depth 3 --downsample 4` | Open3D window `03_point_cloud` (cloud coloured by distance) plus a depth window; headless: one line per second with point count, bbox, centroid, fps. `s` or `--save-on-exit` writes `snapshots/pointcloud_<stamp>.ply`. |
| `04_imu.py` | IMU type/firmware reported; accel + gyro stream at `--rate` Hz with the measured rate matching (wall clock and device timestamps); stationary gyro bias and accel norm are sane; optional fused rotation vector. | `python camera/04_imu.py --duration 10`<br>`python camera/04_imu.py --rotation`<br>`python camera/04_imu.py --rate 400 --duration 10 --headless` | `IMU: BNO086 firmware 3.9.9`; after the 3 s still period a calibration block (gyro bias ~1e-3 rad/s, accel norm ~9.9-10.0 m/s^2 on this unit); live lines at `rate 200.0 Hz`; final summary with 0 timestamp gaps. Hold the camera still for the first 3 s. |
| `05_record.py` | RGB, stereo and IMU can stream while being written to disk; the files DepthAI writes are complete (every mp4 is inspected at the end); `session.json` describes the session and prints the replay command. | `python camera/05_record.py --seconds 10 --headless`<br>`python camera/05_record.py --mode video --seconds 10`<br>`python camera/05_record.py --mode video --depth-stream disparity --align-rgb` | Device summary, output folder, then a preview window `05_record` (or a status line per second); on exit every file with its size and stored frame count, measured per-stream fps, the integrity result and the exact `06_replay.py` command. Exit 0 = replayable. |
| `06_replay.py` | Recordings decode again: `--mode video` replays mp4/mkv/avi + the depth PNG sequence host-side with recorded sequence numbers and timestamps (no camera); `--mode holistic` feeds a `recording_*.tar` back into Camera/IMU nodes (camera required). | `python camera/06_replay.py recordings/<folder>`<br>`python camera/06_replay.py recordings/<folder>/rgb.mp4 --loop --speed 2`<br>`python camera/06_replay.py recordings/<folder> --headless --duration 10 --imu`<br>`python camera/06_replay.py recordings/holistic_<stamp>/recording_*.tar --mode holistic --imu --duration 15` | One window per stream (`replay: rgb`, `replay: depth`, ...) with frame index, recorded seq/timestamp and measured fps; on end of file a summary with `N of M frames (complete)` per stream and the effective speed; exit 0. |
| `07_record_dataset.py` | Records a cuVSLAM dataset: the hardware-synchronised CAM_B/CAM_C GRAY8 pair and the BNO086 raw IMU on one device clock, host-undistorted with all 14 EEPROM coefficients, written in EuRoC ASL layout (`mav0/cam0`, `mav0/cam1`, `mav0/imu0` + `dataset.json`, `calibration.json`, `README.txt`) with drop counters and measured rates. | `python datasets/07_record_dataset.py --duration 30`<br>`python datasets/07_record_dataset.py --width 640 --height 400 --seconds 60 --headless`<br>`python datasets/07_record_dataset.py --preview --note "slow loop back to the tape mark"`<br>`python datasets/07_record_dataset.py --out D:/cuvslam/euroc_bench01 --duration 45` | Device summary, the plan (`baseline 0.074967 m, IMU extrinsics available`), a warm-up line (~1.5 s of frames discarded), then one line per second: `frames 360  fps 29.9  L/R dt +12.4 us ...  imu 200.1 Hz  png queue 1/64  drops 0`. On exit the counts, measured rates, drops, the folder `recordings/euroc_<stamp>/` and the two tracking commands. Exit 0; `png queue` climbing means the PNG writer is falling behind (see the cuVSLAM section). |
| `08_cuvslam_track.py` | Runs cuVSLAM (PyCuVSLAM 17.0.0) on a recording (`DATASET` / `--dataset`: offline, deterministic, no camera, no `depthai` needed) or on the live camera (`--live`, Jetson), in `--mode stereo` (Multicamera) or `vio` (Inertial); writes a TUM trajectory and an optional JSON summary. On Windows only the `--fake-cuvslam` dry run works (synthetic poses, announced loudly). | `python methods/cuvslam-vio/08_cuvslam_track.py recordings/euroc_<stamp> --fake-cuvslam --max-frames 100`<br>`python methods/cuvslam-vio/08_cuvslam_track.py ~/datasets/euroc_<stamp> --mode stereo --summary` (WSL2 / Jetson)<br>`python methods/cuvslam-vio/08_cuvslam_track.py --live --mode vio --duration 60 --summary` (Jetson) | The rig it built (`baseline: 0.0750 m`, `imu: rig_from_imu t=(-0.0006, -0.0011, +0.0011) m`), one statistics line per second, then a `Summary:` block (tracked %, failed, resets, gaps, path length, end-point offset) and `trajectory_<mode>.tum` (`# FAKE cuvslam` in its header for a dry run). Exit 0; 2 for a missing or invalid dataset; 3 on Windows without `--fake-cuvslam`. |

Notes on the ladder:

- `00 --json` prints only the JSON document on stdout (device table and logs go to stderr), so `> device.json` is a valid file. `--duration N` holds the device open and logs chip temperature, USB speed and memory once per second; `--dump-calibration FILE` writes the full EEPROM calibration.
- `02`: the DEFAULT, ROBOTICS, HIGH_DETAIL and FACE presets decimate by 2, so a 640x400 mono request gives 320x200 depth (shown upscaled, centre `(160,100)`); `--decimation 1` gives full-size depth. v3 confidence is inverted vs v2: 255 = most confident, a **higher** `--confidence` gives **fewer** valid pixels (presets use 15 or 55). `--duration` starts after boot and the 1.5 s auto-exposure warm-up.
- `03` runs the DEFAULT preset with decimation switched off (full 640x400 depth), which costs frame rate on RVC2 (see the verified numbers). `--fps 15` is honoured exactly. Without a working Open3D it prints the import error and runs like `--headless`.
- `04`: with `--rotation-rate` lower than `--rate` the whole IMU packet stream runs at the slower rate (all sensors on one IMU node share the slowest enabled rate). Raw sensors reporting `accuracy = UNRELIABLE` is by design.
- `05` on RVC2: the **default holistic mode records CAM_B + CAM_C + IMU only** (640x400, H264, one `recording_*.tar`); depth is not stored and is recomputed on holistic replay, which needs the camera. Three encoded holistic streams fail on this device, so RGB can only be added as raw `.avi` with `--cameras all --no-encode` (~26 MB per 5 s). **Use `--mode video` for RGB + stored depth + IMU** (`rgb.mp4`, one 16-bit PNG per depth frame, `imu.mcap`). In video mode the preset's spatial/temporal post-filters are off by default so depth reaches 30 fps at 640x400; `--preset-filters` keeps them (~17 fps) and `--decimation 2` gets 30 fps back.
- `06 --mode video` needs no camera and is the mode to use when the camera is unplugged; it also opens a holistic tar (videos are extracted to a temp folder). `--mode holistic` still needs the connected OAK in depthai 3.10.0 and has no `--speed`/`--loop`. `--imu` replays `imu.mcap` / `IMU.mcap` too; `--stream depth` selects only the depth PNG sequence.

## Verified on hardware (2026-09-18)

All seven scripts ran on the OAK-D-S2-AF over USB3 from the Windows laptop, windowed and headless. The scene was a **dim desk at night**, not a calibrated target: RGB auto-exposure sat at its ceiling (30 ms, ISO 1599) and the valid-depth fraction was scene-limited. **Depth accuracy against a tape measure has not been done yet.**

| Script | Result | Key measurements |
|---|---|---|
| `00_device_info.py` | pass | Device open 3.4-4.3 s (enumerate + boot); `usb=SUPER`; chip 38-40 C over a 5 s soak; calibrated baseline 74.97 mm; `--json` output parses (383 pretty-printed lines). |
| `01_rgb_preview.py` | pass | 1280x720@30: 30.09 FPS avg, 238 frames, 0 missing; 1920x1080@30: 30.29 FPS, 149 frames, 0 missing; host latency 40 ms (720p) / 53 ms (1080p); snapshot written at the requested size. |
| `02_stereo_depth.py` | pass | DEFAULT 640x400 (320x200 depth): 30.0-30.1 FPS, valid 15-18 %, median 1.49-1.64 m, centre ROI 1.6-2.6 m; all 8 presets 30 FPS at 640x400; `--no-lrcheck` 55-62 % valid, DENSITY 35-37 %; `--align-rgb` 30 FPS, 30-32 % valid; 1280x800 DEFAULT only 18.3 FPS. |
| `03_point_cloud.py` | pass | 640x400, decimation 1: 16.6-16.7 FPS, 42.7-48.3k points per cloud, centroid z 1.65-1.81 m; `--color`: 8.6-9.8 FPS, 71-76k coloured points; `--fps 15`: 15.0 FPS; `--max-depth 1.5` keeps 18.5-19.7k points; PLY 46,632 points, 1.1 MB, reads back in Open3D. |
| `04_imu.py` | pass | 200.2 Hz by wall clock and by device timestamps (1552 packets in 7.75 s, spacing mean 5.00 ms, 0 gaps); gyro bias max 0.0013 rad/s; accel norm 9.96 m/s^2 (+1.6 % vs g); `--rate 100`: 100.1 Hz; rotation quaternion norm 1.0000. |
| `05_record.py` | pass | Holistic 5 s: CAM_B + CAM_C 640x400 H264, 150 of 151 frames stored per camera (30.2 FPS), IMU 196 Hz, tar 4.8 MB. Video 5 s: `rgb.mp4` 1280x720 148 frames (29.4 FPS host-side), 141 depth PNGs with 0 sequence gaps (30.00 fps by device timestamps), IMU 194 Hz. |
| `06_replay.py` | pass | Video mode: camera stayed unbooted across 23 headless runs; rgb delivered 24.2 fps = 0.81x real time on Windows, depth PNGs 30.0 fps = 1.00x; `--speed 2` gives depth 2.00x; `--loop` and `--imu` (1070 packets in 6 s) work. Holistic mode (camera): CAM_B + CAM_C 23.7 fps, 0 sequence gaps. |

Snapshots left from the run: `snapshots/rgb_1920x1080_20260918_023808_seq148.png` (near-black, dark room) and `snapshots/pointcloud_20260918_025220.ply`. The `recordings/` folders from the session are on disk but git-ignored.

### Known quirks

All minor; none blocked a test.

- **USB2 fallback happens.** One of five back-to-back boots during the IMU test came up `usb=HIGH` with no cable change; the other four were `SUPER`. The IMU rate was unaffected, but video streams would be. Always read the `Device:` line.
- `00 --json`: `device.os_version` is `""` instead of `null` on RVC2; every other unavailable field is `null`.
- `01`: the `ts` column is the host steady clock since boot formatted as HH:MM:SS.mmm, so hours exceed 23; it is not wall-clock time. The snapshot from the run is near-black because the room was dark, not because of the script (the mono cameras saw a mean of 18 where CAM_A saw 0.05).
- `02`: presets without a threshold filter (FAST_ACCURACY, FAST_DENSITY, DENSITY, ACCURACY) print `max 59.9 m` in a room: that is the firmware's far cap for the smallest non-zero disparity, reported faithfully. At 1280x800 the DEFAULT preset runs at ~18 FPS instead of 30 and the script prints the number without flagging the shortfall.
- `03`: the default configuration delivers ~16.7 FPS although 30 is requested and the docstring says "roughly the requested fps"; the cause is decimation 1 at 640x400 on RVC2 (decimation 2 gives 30 FPS at 320x200). `--color` roughly halves the cloud rate to ~9 FPS while aligned depth still arrives at ~16 FPS.
- `04`: the docstring's expected accel norm of 9.7-9.9 m/s^2 is low for this unit, which reads 9.96 in every run. With `--rotation` the accuracy estimate stays at 3.14 rad / UNRELIABLE for the whole stationary run (the BNO086 rotation vector needs motion to converge), so the docstring's "drops well below 0.1 rad" does not happen while the camera sits still. With `--rotation-rate` below `--rate` the summary reads like 50 % packet loss (`requested 200 Hz -> measured 100.1 Hz`) although the throttling is expected.
- `05`: the printed "Files written" list and the `files` array in `session.json` omit `session.json` itself. Host-measured fps divides by the whole session including start-up, so it understates (`depth 28.2 fps` printed while device timestamps show exactly 30.00).
- `06`: `ReplayVideo` "real time" pacing is ~0.8x on Windows because of the ~16 ms sleep granularity (the host-paced depth PNGs hit exactly 1.00x / 2.00x / 0.50x); a 6 s `--duration` therefore does not finish a 4.8 s recording. Holistic-mode summaries still print `(requested 2x)` after saying `--speed` is ignored. An undecodable header-only mp4 produces three stderr lines (two OpenCV warnings plus the message) instead of one. `--device-id` is silently ignored in `--mode video`.

## Recording and replay workflow

1. Record with `05_record.py`. Each session goes to `recordings/<mode>_<YYYYmmdd_HHMMSS>/` (or under `--out DIR`) and ends with the integrity check and the exact replay command, which is also stored as `replay_command` in `session.json`.

   ```
   recordings/video_<stamp>/       rgb.mp4 + rgb.mcap            H264 at --width x --height, --fps
                                   depth/depth_NNNNNN.png        one 16-bit PNG per frame, uint16 mm, 0 = invalid
                                   depth_index.csv               sequence number, timestamps, file per frame
                                   (or disparity.mkv + .mcap     with --depth-stream disparity)
                                   imu.mcap                      accel + gyro RAW, 200 Hz
                                   calibration.json              full EEPROM calibration dump
                                   session.json                  device, settings, intrinsics, files, fps, warnings, replay command

   recordings/holistic_<stamp>/    recording_YYYY-MM-DD_HH-MM-SS.tar
                                     CameraCAM_B.mp4 + .mcap, CameraCAM_C.mp4 + .mcap (raw .avi with --no-encode)
                                     IMU.mcap, camera_info.json, record_config.json
                                   session.json
   ```

   Read a depth PNG back with `cv2.imread("depth/depth_000123.png", cv2.IMREAD_UNCHANGED)` -> `(H, W)` uint16 millimetres. Depth lives in the rectified CAM_C frame at 640x400 unless `--align-rgb` was given, in which case `rgb.mp4` is 640x400 undistorted and depth is pixel-aligned to it.

2. Replay with `06_replay.py`, pointing at the folder, a single video file, or the tar:

   ```bat
   python camera/06_replay.py recordings/video_<stamp>                       # rgb + depth, no camera needed
   python camera/06_replay.py recordings/video_<stamp> --imu --headless --duration 10
   python camera/06_replay.py recordings/video_<stamp> --stream depth --speed 0.5
   python camera/06_replay.py recordings/holistic_<stamp>                    # the tar's mp4s, host-only
   python camera/06_replay.py recordings/holistic_<stamp>/recording_*.tar --mode holistic --imu   # camera required
   ```

   Video mode builds one host-only pipeline per file so every stream plays to its own end; the summary says `N of M frames (complete)` per stream. Holistic mode recomputes depth on the device from the recorded mono streams and loops forever (stop with `q`, `--duration` or Ctrl-C). A tar whose mp4s are header-only (a failed holistic recording, which `05` reports with exit 1) is rejected by `06` with an exit-2 message in both modes; a recording that merely lost frames replays with exit 0 and the summary shows fewer frames than the container's count.

## Troubleshooting

- **`No OAK device found. Check the USB-C cable and that no other program has the camera open.`** (at once with `--device-id`; after three retries, ~8 s, without it): nothing enumerated. Re-seat the cable, try another USB3 port, run `python camera/00_device_info.py` to see the enumeration table (`OAK devices found: none`). A camera that is still rebooting from the previous script also shows up as nothing for a few seconds; the retries are meant to ride that out, so leave `--device-id` off when chaining scripts.
- **Device busy / in use by another process.** On this platform a camera held by another process simply vanishes from enumeration (there is no "already in use" exception), so it looks unplugged; if the open itself fails you get `Device <id> is in use by another process.` Close the other script (check Task Manager for stray `python.exe`), wait ~10 s for the device to come back to `X_LINK_UNBOOTED`, retry. One process per camera.
- **`usb=HIGH (WARNING: not USB3)`** on the `Device:` line plus a `!!! WARNING: USB speed is HIGH instead of SUPER` line on stderr: USB2 fallback, ~10x less bandwidth, dropped frames. It happened on one of five back-to-back boots here with a good cable and port, so first just re-run or re-plug; if it repeats, change the port or cable and remove any hub.
- **Low FPS at 800p on RVC2.** `02 --width 1280 --height 800` with the DEFAULT preset (7x7 median + subpixel + LR-check + post-filters) manages ~18 FPS, `03` at 640x400 with decimation off ~17 FPS, `05 --preset-filters` ~17 FPS. That is the VPU, not USB. Stay at 640x400, use `--decimation 2`, or drop the spatial/temporal filters (`FAST_*` presets, or `05`'s default).
- **OneDrive.** This repo lives inside a OneDrive folder. `.venv/`, `recordings/` and `snapshots/` are git-ignored but OneDrive still syncs them, which adds disk I/O while recording and can hold files open. Pause syncing while recording, or point `05` at a folder outside OneDrive with `--out`, and the snapshot scripts (`01`, `02`, `06`) elsewhere with `--snapshot-dir` (`03` has no such flag and always writes its PLYs to `snapshots/`). Do not commit anything from those folders.
- **Camera warmth.** The housing gets warm after long streaming; the device is rated for -20 to +50 C ambient and DepthAI shuts it down at 105 C die temperature (see the spec sheet). `00_device_info.py --duration 60` logs chip temperature once a second (38-40 C in today's 5 s soak). Unplug it when idle.
- **cuVSLAM.** `png queue` climbing / `stereo_write_queue_dropped` while recording, IMU gaps, tracking failures and resets, `cuvslam not importable` (exit 3), wrong `vio` gravity, a dataset that fails validation (exit 2): [methods/cuvslam-vio/docs/cuvslam.md](../methods/cuvslam-vio/docs/cuvslam.md) section 6, one entry each.
