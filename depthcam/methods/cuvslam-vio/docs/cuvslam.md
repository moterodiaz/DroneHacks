# cuVSLAM on the OAK-D S2: user guide

How to get a trajectory out of this camera with NVIDIA's cuVSLAM (PyCuVSLAM **v17.0.0**): what the
integration is, the two-machine workflow (record on the Windows laptop, track in WSL2; live on the
Jetson), the exact commands for every step, how to judge the result, the independent check with
NVIDIA's own example, troubleshooting, the Jetson day-one checklist and, at the end, an honest list of
what has and has not been verified. Every flag quoted here was checked against the scripts' own
`--help` output and every evo flag against evo 1.37.1 on 2026-09-20 (section 10).

| Item | Value |
|---|---|
| Camera | OAK-D-S2-AF, device id `OAKDS2EXAMPLEID000`: OV9282 1280x800 global-shutter mono pair (CAM_B = left, CAM_C = right, calibrated baseline 0.0750 m), BNO086 IMU, colour CAM_A unused |
| Library | PyCuVSLAM 17.0.0 (`cuvslam` 17.0.0+cu12), Linux only, CUDA 12: [cuvslam-setup.md](cuvslam-setup.md) |
| Two modes | `stereo` = `OdometryMode.Multicamera` (cameras only, mirrors NVIDIA's `examples/oak-d/run_stereo.py`); `vio` = `OdometryMode.Inertial` (cameras + BNO086) |
| Record | `datasets/07_record_dataset.py` (Windows or Jetson) -> `recordings/euroc_<YYYYmmdd_HHMMSS>/` in EuRoC ASL layout: [cuvslam-dataset-format.md](../../../datasets/dataset-format.md) |
| Track | `methods/cuvslam-vio/08_cuvslam_track.py <dataset>` (WSL2 / Jetson) or `--live` (Jetson) -> TUM trajectory `timestamp_s tx ty tz qx qy qz qw` |
| Evaluate | the script's summary (tracked %, resets, path length, end-point offset) and `evo_traj` / `evo_ape` on the TUM file |
| Windows | no `cuvslam` wheel exists; the laptop records, and runs the whole pipeline as a dry run against `cuvslam_oak/fake_cuvslam.py` (`--fake-cuvslam`) |
| Status | **nothing has run on CUDA hardware yet** (no WSL2 on the laptop, no Jetson on hand): section 9 |

Companion references (read them for the *why*; this page is the *how*): [cuvslam-setup.md](cuvslam-setup.md)
(install on WSL2 and the Jetson), [cuvslam-api-v17.md](cuvslam-api-v17.md) (the exact v17.0.0 Python
API), [oak-to-cuvslam-calibration.md](oak-to-cuvslam-calibration.md) (EEPROM -> `Rig`, frames, units,
timestamps), [cuvslam-dataset-format.md](../../../datasets/dataset-format.md) (what a recording contains),
[depthai-v3-cheatsheet.md](../../../camera/docs/depthai-v3-cheatsheet.md) (the DepthAI side), [jetson-orin-nano-setup.md](../../../camera/docs/jetson-orin-nano-setup.md)
(the board itself) and the repo [README](../../../README.md) (scripts 00-06, common flags, exit codes).

---

## 0. The sixty-second version

```bash
# Windows laptop, repo root, venv active (.venv\Scripts\activate)
python datasets/07_record_dataset.py --duration 45 --note "slow loop, back to the tape mark"
python methods/cuvslam-vio/08_cuvslam_track.py recordings/euroc_<stamp> --fake-cuvslam --max-frames 100   # plumbing dry run only

# WSL2 (Ubuntu-22.04), after methods/cuvslam-vio/tools/setup_wsl2.sh once; venv: source ~/venvs/cuvslam/bin/activate
mkdir -p ~/datasets && cp -r ~/DroneDepthCamTesting/recordings/euroc_<stamp> ~/datasets/
cd ~/DroneDepthCamTesting
python methods/cuvslam-vio/08_cuvslam_track.py ~/datasets/euroc_<stamp> --mode stereo --summary
python methods/cuvslam-vio/08_cuvslam_track.py ~/datasets/euroc_<stamp> --mode vio --summary
evo_traj tum ~/datasets/euroc_<stamp>/trajectory_stereo.tum ~/datasets/euroc_<stamp>/trajectory_vio.tum \
    --ref ~/datasets/euroc_<stamp>/trajectory_stereo.tum --align --plot --plot_mode xz

# Jetson Orin Nano, after methods/cuvslam-vio/tools/setup_jetson.sh once; venv: source ~/DroneDepthCamTesting/.venv/bin/activate
python methods/cuvslam-vio/08_cuvslam_track.py --live --mode stereo --duration 60 --summary
```

`~/DroneDepthCamTesting` is the symlink to the Windows checkout that [cuvslam-setup.md](cuvslam-setup.md)
section 2.5 creates (`/mnt/c/<path to the repo checkout>`,
quote it: the path has spaces).

---

## 1. What this integration is

### 1.1 Two tracking modes, one loop

- **Stereo visual odometry** (`--mode stereo`): the two synchronised OV9282 frames go into
  `cuvslam.Tracker` in `OdometryMode.Multicamera`, exactly the configuration of NVIDIA's own OAK-D
  example (`examples/oak-d/run_stereo.py` at tag v17.0.0): unrectified GRAY8 frames,
  `rectified_stereo_camera=False`, the EEPROM intrinsics scaled to the frame size, the first 8 OpenCV
  distortion coefficients as `Distortion.Model.Polynomial`, the example's border masks (50 px top,
  70 px left/right at 1280x800), 60 warm-up frames on a live camera.
- **Stereo-inertial** (`--mode vio`): the same rig plus one `ImuCalibration` for the BNO086
  (`ACCELEROMETER_RAW` + `GYROSCOPE_RAW` at 200 Hz, `rig_from_imu` from the EEPROM), in
  `OdometryMode.Inertial`. Every IMU sample stamped at or before an image is registered with
  `register_imu_measurement(0, m)` before that image's `track()`, the pattern of NVIDIA's RealSense
  `run_vio.py`. NVIDIA's own ranking puts plain stereo *above* stereo-inertial in accuracy (their
  `TROUBLESHOOTING.md`, step 8, "Pick the best odometry mode"); the IMU buys robustness through short
  visual dropouts, not precision (step 9, "IMU integration").
  So: get `stereo` right first, then compare `vio` against it.

Both modes run through `cuvslam_oak.tracking.run_tracking`, which counts failed frames, restarts of
the world frame (`resets`), frame and IMU gaps, and writes the trajectory in TUM format.

### 1.2 The package

| Module | Job | Runs on |
|---|---|---|
| `cuvslam_oak/__init__.py` | `import_cuvslam()`: the real wheel, or the fake when `--fake-cuvslam` / `CUVSLAM_FAKE=1` asks for it; never the fake silently on Linux | everywhere |
| `cuvslam_oak/rig.py` | EEPROM calibration (`calibration.json` or the live device) -> `cuvslam.Rig`; EuRoC `sensor.yaml` helpers; host undistortion maps | everywhere (stdlib + numpy) |
| `cuvslam_oak/oak_source.py` | the live DepthAI v3 graph (Camera CAM_B/CAM_C -> Sync, IMU node), device-monotonic mid-exposure timestamps, drop counters | Windows (record), Jetson (record, live) |
| `cuvslam_oak/dataset.py` | EuRoC ASL writer (background PNG thread, bounded queue) and reader/validator | everywhere |
| `cuvslam_oak/tracking.py` | the tracking loop, IMU feeding, statistics, TUM writer/reader | WSL2, Jetson; Windows with the fake |
| `cuvslam_oak/fake_cuvslam.py` | test double of PyCuVSLAM 17.0.0 (same classes and checks, synthetic constant-velocity poses) | Windows |
| `datasets/07_record_dataset.py` | record a dataset from the camera | Windows, Jetson |
| `methods/cuvslam-vio/08_cuvslam_track.py` | track a dataset or the live camera, write TUM + JSON summary | WSL2, Jetson (Windows: dry run) |
| `methods/cuvslam-vio/tools/check_env.py`, `methods/cuvslam-vio/tools/setup_wsl2.sh`, `methods/cuvslam-vio/tools/setup_jetson.sh` | host diagnosis and one-shot installs | see [cuvslam-setup.md](cuvslam-setup.md) |

### 1.3 Frames, units, timestamps (the conventions everything below assumes)

- Axes are OpenCV's: x right, y **down**, z forward. A floor plan is therefore the **x-z** plane
  (`evo_traj ... --plot_mode xz`).
- The trajectory is `world_from_rig`: the pose of the rig in a world frame that *is* the rig pose at
  the first tracked frame (identity), and that restarts after a tracking loss (section 4.1).
- The rig frame is the **left camera (CAM_B) optical frame** in both paths: a dataset always uses
  cam0 = CAM_B as its body frame ([cuvslam-dataset-format.md](../../../datasets/dataset-format.md) section 6),
  and `08 --live` defaults to `--rig-socket CAM_B` so live and recorded trajectories compare directly
  (`--rig-socket CAM_A` reproduces NVIDIA's example, whose rig is the colour camera).
- Metres, radians, m/s^2, rad/s; quaternions `x y z w` (TUM order, no reordering).
- Timestamps are integer nanoseconds on **one device-monotonic clock** for cameras and IMU, taken
  mid-exposure for images ([oak-to-cuvslam-calibration.md](oak-to-cuvslam-calibration.md) section 6);
  a recording adds one constant epoch offset (`dataset.json.time.epoch_offset_ns`) so its stamps are
  19-digit Unix nanoseconds ([cuvslam-dataset-format.md](../../../datasets/dataset-format.md) section 8).
- No axis flips anywhere; the only non-identity rotation in the rig is the EEPROM's `diag(1,-1,-1)`
  between the IMU and the cameras, applied through `rig_from_imu`, never to the data.

---

## 2. The two-machine workflow

```
 Windows laptop (this repo, .venv, camera on USB 3)        WSL2 Ubuntu-22.04 (same laptop, RTX 1000 Ada)
 ┌─────────────────────────────────────────────┐          ┌──────────────────────────────────────────┐
 │ 07_record_dataset.py  -> recordings/euroc_* │ copy --> │ ~/datasets/euroc_*                        │
 │ 08 ... --fake-cuvslam (plumbing dry run)    │          │ 08_cuvslam_track.py --mode stereo / vio   │
 │ pytest tests/ against fake_cuvslam          │          │ evo_traj / evo_ape; NVIDIA track_euroc.py │
 └─────────────────────────────────────────────┘          └──────────────────────────────────────────┘
                                   Jetson Orin Nano (JetPack 6.2.x, camera on a Type-A port)
                                   ┌──────────────────────────────────────────────────────────┐
                                   │ 07 (record) ; 08 <dataset> ; 08 --live --mode stereo/vio │
                                   └──────────────────────────────────────────────────────────┘
```

Why split: there is no Windows `cuvslam` wheel (Linux x86_64/aarch64, Python 3.10, glibc 2.35, CUDA
12), and the WSL2 route to the live camera (usbipd) is functional-test only. So the Windows side owns
the camera and the dataset, WSL2 owns the GPU, and the Jetson is the only host that does both.

| Step | Host | Command | Section |
|---|---|---|---|
| 0 | any | `python methods/cuvslam-vio/tools/check_env.py` (`--warm-up` on a CUDA host) | 3.1 |
| 1 | Windows or Jetson | `07_record_dataset.py --duration 45 ...` | 3.2 |
| 2 | Windows | `08_cuvslam_track.py <dataset> --fake-cuvslam` (proves the dataset loads; poses are synthetic) | 3.3 |
| 3 | WSL2 | copy to `~/datasets`, `08 ... --mode stereo`, then `--mode vio` | 3.4 |
| 4 | WSL2 / Jetson | `evo_traj`, return-to-start numbers, stereo-vs-vio comparison | 4 |
| 5 | WSL2 / Jetson | NVIDIA's unmodified `track_euroc.py` on the same folder | 5 |
| 6 | Jetson | `08 --live --mode stereo --duration 60`, then `--mode vio` | 3.5, 7 |

---

## 3. Exact commands for each step

Run everything from the repository root. `python` means the venv interpreter of that host:
`.venv\Scripts\python.exe` on Windows, `~/venvs/cuvslam/bin/python` in WSL2 (after
`methods/cuvslam-vio/tools/setup_wsl2.sh`), `~/DroneDepthCamTesting/.venv/bin/python` on the Jetson (after
`methods/cuvslam-vio/tools/setup_jetson.sh`). `--help` never touches the camera.

### 3.1 Check the host

```bash
python methods/cuvslam-vio/tools/check_env.py            # exit 0 always; one "key: value" line per fact
python methods/cuvslam-vio/tools/check_env.py --warm-up  # WSL2 / Jetson: also cuvslam.warm_up_gpu() (raises if CUDA, cuBLAS or cuSOLVER is missing)
python methods/cuvslam-vio/tools/check_env.py --json     # machine-readable
```

Expected lines per host are tabulated in [cuvslam-setup.md](cuvslam-setup.md) section 4.1. On Windows
`cuvslam: NOT importable` is normal. On the camera hosts also confirm the USB link once per session:

```bash
timeout 90 python camera/00_device_info.py     # prints "Device: OAK-D-S2 (OAK-D-S2-AF) id=... usb=SUPER" and exits
```

`usb=HIGH` means a USB 2 link; see section 6.5 before recording anything. (`timeout 90` is the GNU
guard used on the Linux hosts so a hung boot cannot hold the camera; on Windows run the script bare.)

### 3.2 Record a dataset (Windows or Jetson, camera attached)

```bash
python datasets/07_record_dataset.py --duration 45 --note "slow loop around the lab, back to the tape mark"
python datasets/07_record_dataset.py --duration 45 --preview                     # plus a window (q stops)
python datasets/07_record_dataset.py --width 640 --height 400 --duration 120     # light option for long runs
python datasets/07_record_dataset.py --out D:/cuvslam/euroc_bench01 --duration 30 # outside OneDrive (recommended)
python datasets/07_record_dataset.py --device-id OAKDS2EXAMPLEID000 --duration 10 # explicit id: never retries
python datasets/07_record_dataset.py --help
```

What it does, in order: enumerates and opens the camera (`Device: ... usb=SUPER` line), reads the
EEPROM, prints the plan (`Calibration: baseline 0.074967 m, IMU extrinsics available (T_BS =
getImuToCameraExtrinsics(CAM_B)), body = cam0 = CAM_B, ...`; six decimals here, `baseline: 0.0750 m`
in `08`'s rig print), warms up
for 1.5 s (`--warmup`, auto-exposure settles, those frames are discarded), then streams the
hardware-synchronised pair at 1280x800 GRAY8 @ 30 fps and the raw IMU at 200 Hz, undistorts every
frame on the host with all 14 EEPROM coefficients (intrinsics unchanged) and writes:

```
recordings/euroc_<YYYYmmdd_HHMMSS>/
├── dataset.json        counts, measured rates, drops, device, settings, epoch offset, warnings
├── calibration.json    the EEPROM dump (CalibrationHandler.eepromToJson())
├── README.txt          summary + the exact commands to track this folder
└── mav0/
    ├── cam0/{data/<ts_ns>.png, data.csv, sensor.yaml}   left  = CAM_B = body frame
    ├── cam1/{data/<ts_ns>.png, data.csv, sensor.yaml}   right = CAM_C
    └── imu0/{data.csv, sensor.yaml}                     gyro rad/s then accel m/s^2
```

One status line per second while recording:

```
[  12.0 s | 33.0 s left] frames   360  fps 29.9  L/R dt  +12.4 us (max 15.8)  imu 200.1 Hz (2413)  png queue  1/64  drops 0
```

Watch two things: `png queue` must stay near `1/64` (the PNG writer is a single thread; at 1280x800
a laptop disk can fall behind, and a full queue *drops pairs*, counted in
`drops.stereo_write_queue_dropped`), and `drops` must stay `0`. If the queue climbs: `--width 640
--height 400`, `--png-compression 0`, a lower `--fps`, or an `--out` on a faster disk outside
OneDrive. `--headless` is the default behaviour (no window); `--preview` opens one; `--seconds` is an
alias of `--duration`; `--imu-rate 400` is the recommended ceiling (BNO08X gyroscope jitter above it; `08`
refuses more); `--raw`
records distorted frames as delivered (only `08` can track those, NVIDIA's loader cannot);
`--no-strict-intrinsics` turns the first-frame intrinsics guard into a warning (bench diagnostics only).

Stopping: `--duration`, `q` in the preview window, or Ctrl-C (keeps what was recorded: the PNG queue is
drained and the csv files trimmed to complete pairs; a *second* Ctrl-C aborts and leaves no
`dataset.json`, repairable with `cuvslam_oak.dataset.trim_dataset(<folder>/mav0)`). Exit codes: 0
complete, 1 no pairs / stream fault / disk error, 2 no device or wrong `--device-id`, 130 Ctrl-C before
streaming. The exit summary prints counts, measured rates, drops, warnings, the dataset path and the
tracking commands of sections 3.4 and 5, already filled in with this folder's name.

How to move the camera for a dataset that can be evaluated (section 4):

1. Put a tape mark on the floor and a second one for the heading. Lights on; a textured scene
   (posters, furniture, not a bare white wall or a window-lit corridor).
2. Start the recording with the camera resting on the mark, upright and level, lens horizontal.
   Hold still for about 3 s (the recorder discards its warm-up anyway; the stillness gives the
   inertial mode a clean gravity estimate and gives you a known start pose).
3. Walk a loop at walking pace, 10-30 m, no fast rotations (keep well under 90 deg/s: at 30 fps a
   fast pan blurs the global-shutter frames and drops feature overlap), no long stares at blank walls.
4. Return to the mark with the same heading, hold still 3 s, stop. The end-point offset the tracker
   prints is now the loop drift.
5. Record a second, different-length loop, and a "shuttle" (forward 5 m, back to the mark) for the
   sanity check NVIDIA recommends when no ground truth exists.

30-60 s is plenty. Size, measured on the 2026-09-20 recordings (section 9): 362 MB for 363 pairs at
1280x800 (about 0.5 MB per PNG at compression 1, two cameras: ~1.8 GB per minute at 30 fps, inside
the 0.3-1.0 MB per PNG range of [cuvslam-dataset-format.md](../../../datasets/dataset-format.md) section 2.2)
and 70 MB for 300 pairs at 640x400 (~0.12 MB per PNG, ~0.4 GB per minute). Size the USB stick and
`~/datasets` for it, and read the 1280x800 drop finding in section 9 before recording at full
resolution on the laptop.

### 3.3 Prove the dataset loads: the Windows dry run

```bash
python methods/cuvslam-vio/08_cuvslam_track.py recordings/euroc_<stamp> --fake-cuvslam --max-frames 100
python methods/cuvslam-vio/08_cuvslam_track.py recordings/euroc_<stamp> --fake-cuvslam --mode vio --summary --verbose
```

`--fake-cuvslam` swaps in `cuvslam_oak.fake_cuvslam`: the reader validates the folder, the rig is
built from its `sensor.yaml` files, the loop feeds images and IMU rows in order and writes
`recordings/euroc_<stamp>/trajectory_<mode>.tum`, but **the poses are a synthetic constant-velocity
line, not tracking**. The script says so with a banner on stderr, a header line in the TUM file and
`FAKE` in the summary. What the dry run does prove: the dataset passes validation (equal pair
counts, every PNG present and named `<ts_ns>.png`, strictly increasing frame stamps, non-decreasing
IMU stamps, `dataset.json` counts matching the csv files; it *warns* when the IMU starts after the
first frame, when a measured rate is off by more than 10 % or when the recorder logged drops), the
calibration produced a sane rig (`baseline: 0.0750 m`, `imu: rig_from_imu t=(-0.0006, -0.0011,
+0.0011) m`), and the output files are the ones evo reads. Without the flag the script exits 3 on
Windows (section 6.4).

The same fake backs the test suite: `python -m pip install -r requirements-dev.txt` once (pytest),
then `python -m pytest tests -q` (61 tests, about 6 s on this laptop). The camera is never opened:
`tests/conftest.py` replaces `dai.Device`, `dai.DeviceBase` and `dai.Pipeline` with functions that
fail the test, and the CLI smoke tests only enumerate (`--device-id nonexistent`). Section 9 lists
what the suite covers.

### 3.4 Track the dataset in WSL2

Once: install per [cuvslam-setup.md](cuvslam-setup.md) section 2 (`wsl --install -d Ubuntu-22.04`,
then `bash methods/cuvslam-vio/tools/setup_wsl2.sh` inside Ubuntu; it makes `~/venvs/cuvslam`, installs the wheel, the
toolkit, `evo`, and runs the smoke test). Then, per dataset:

```bash
source ~/venvs/cuvslam/bin/activate
cd ~/DroneDepthCamTesting
mkdir -p ~/datasets && cp -r recordings/euroc_<stamp> ~/datasets/     # PNGs read through /mnt/c are slow: copy first
python methods/cuvslam-vio/08_cuvslam_track.py ~/datasets/euroc_<stamp> --mode stereo --summary
python methods/cuvslam-vio/08_cuvslam_track.py ~/datasets/euroc_<stamp> --mode vio --summary
python methods/cuvslam-vio/08_cuvslam_track.py ~/datasets/euroc_<stamp> --mode stereo --summary --rerun     # NVIDIA-style viewer (WSLg)
python methods/cuvslam-vio/08_cuvslam_track.py ~/datasets/euroc_<stamp> --mode stereo --max-frames 300 --show # OpenCV window, q stops
```

Notes:

- The dataset path may be the folder that holds `mav0/` or `mav0` itself. `--mode` defaults to `vio`
  when the recording has IMU rows and real IMU extrinsics, else `stereo`; give it explicitly so the
  output name is what you expect. Aliases accepted: `stereo-only`, `multicamera`, `vo`; `inertial`,
  `stereo-inertial`, `imu`.
- Outputs: `<dataset>/trajectory_<mode>.tum` (or `--out FILE`, or `--out DIR` for
  `DIR/trajectory_<mode>.tum`) and, with `--summary`, `trajectory_<mode>_summary.json` next to it
  (`--summary-path PATH` to put it elsewhere). One TUM row per successfully tracked frame; failed
  frames are skipped, not written as NaN.
- A dataset run is deterministic (`async_sba=False`, the setting every NVIDIA example uses) and skips
  no frames (`--warmup-frames` defaults to 0 for a recording, 60 live).
- `--stitch-resets` chains the segments after a tracking loss onto the last good pose so the TUM file
  stays one continuous path (zero motion during the loss is assumed); without it each loss restarts
  the world frame at identity, which is what cuVSLAM itself does. Use it for plots; leave it off when
  counting losses.
- The starvation guard is on by default (`--no-guard` switches it off). On a blank wall from close
  range cuVSLAM still returns a pose, but a garbage one (measured on hand-held walks: steps of
  0.4-52 m per frame, rotations up to 63 deg; in `vio` mode the filter then runs away for hundreds
  of metres) and never reports a failure. A pose is skipped, and the trajectory continues from the
  last good pose across the gap, when it moved faster than `--max-speed` (3 m/s) or rotated faster
  than `--max-rot-rate` (720 deg/s) since the last accepted pose, or when its position covariance
  is exactly zero (`--no-zero-covariance-guard` keeps those; there is no covariance *magnitude* rule
  because the engine reports a placeholder variance of 1.0 on ~30 % of good frames of a still
  camera). `--min-observations N` adds an observation-count rule (default 0 = off: on this camera
  the count sat at the engine's cap of 450 on the glitch frames). After `--guard-reset-after` (30)
  consecutive guarded frames the tracker is re-created. A racing drone needs a higher `--max-speed`;
  the JSON summary carries `frames_guarded`, `guard_*`, `guard_gaps` and
  `observations_min/median/max` for tuning.
- `--calibration PATH` builds the rig with `cuvslam_oak.rig.build_rig` from a `calibration.json`
  (or a holistic `camera_info.json`) instead of the dataset's `sensor.yaml`: the A/B tool when the
  yaml is suspected. `--rig-socket` then picks CAM_B (default) or CAM_A.
- `--multicam-mode {Performance,Precision,Moderate}` (default `Precision`).
- `--slam`, `--slam-map-size N`, `--slam-async`, `--slam-save DIR`: cuVSLAM's SLAM layer (loop closure,
  pose graph) next to the odometry; writes `trajectory_<mode>_slam.tum` and reports loop closures in the
  summary. Measured 2026-09-30: no correction on any of our recordings ([README](../../../README.md)
  "Verification status"); the SLAM pose jumps when a loop closes, so a flight controller gets the
  odometry rows, never these.
- `--no-coast`, `--coast-decay S`, `--coast-max S`, `--coast-velocity-window S`, `--no-coast-gyro`,
  `--coast-keep-engine-rotation`: coasting through dropouts (section 6.3); on by default.
- `--imu-calibration PATH|auto|none` (`vio` only, default `none`): corrects every IMU sample with the file
  written by `datasets/fit_imu_calibration.py` (`auto` = `datasets/calibration/imu_<device id>.json`) before it is
  registered; section 6.7. `--denoising` switches on cuVSLAM's `use_denoising` (no effect measured).
- `--verbose` prints the full rig (quaternions, coefficients, borders), the `TrackConfig` and every
  dataset validation warning; otherwise one line summarises them.
- WSL2 with `WITH_DEPTHAI=0`: dataset tracking works (the script only needs `depthai` for `--live`).

Exit codes: 0 finished (end of data, `--duration`, `--max-frames`, `q`, Ctrl-C during tracking);
1 tracking / calibration / stream error; 2 dataset missing or failed validation (details on stdout,
one line on stderr), bad `--mode` for the data, no device for `--live`; 3 `cuvslam` not importable.

### 3.5 Live on the Jetson

Once: `bash ~/DroneDepthCamTesting/methods/cuvslam-vio/tools/setup_jetson.sh` ([cuvslam-setup.md](cuvslam-setup.md) section 3;
udev rule, wheel, `depthai`, `evo`, `jtop`). Then:

```bash
source ~/DroneDepthCamTesting/.venv/bin/activate
cd ~/DroneDepthCamTesting
timeout 90 python camera/00_device_info.py                                   # usb=SUPER
python methods/cuvslam-vio/08_cuvslam_track.py --live --mode stereo --duration 60 --summary
python methods/cuvslam-vio/08_cuvslam_track.py --live --mode vio --duration 60 --summary
python methods/cuvslam-vio/08_cuvslam_track.py --live --mode stereo --size 640x400 --multicam-mode Performance --duration 60
python methods/cuvslam-vio/08_cuvslam_track.py --live --mode stereo --show --duration 120  # window with observations; q stops
python datasets/07_record_dataset.py --duration 45 --note "jetson, lab loop"    # recording works here too
```

Live runs use the EEPROM read from the device (`--calibration PATH` to override), raw frames with the
Polynomial model and the example's border masks, `async_sba=True` (real time), 60 warm-up frames, and
write `recordings/trajectory_<mode>_<YYYYmmdd_HHMMSS>.tum` (`--out` to choose). `--fps` (default 30)
and `--imu-rate` (default 200, max 400) shape the stream; `--size` accepts 1280x800 (native) or
640x400 (an exact half-scale of the calibration). Ctrl-C ends the run cleanly and still writes the
file; `--device-id ID` fails at once with one line if that id is not enumerated. The camera is
released at exit (`pipeline.stop()` closes the device and it reboots for a few seconds, so a script
started right after may retry its open).

### 3.6 Live view on the laptop (camera on Windows, engine in WSL2)

```
.venv\Scripts\python methods\cuvslam-vio\09_cuvslam_live_view.py                       # camera; until q
.venv\Scripts\python methods\cuvslam-vio\09_cuvslam_live_view.py --duration 60 --save-tum recordings\live.tum
.venv\Scripts\python methods\cuvslam-vio\09_cuvslam_live_view.py --replay recordings\euroc_<stamp>            # a recording
.venv\Scripts\python methods\cuvslam-vio\09_cuvslam_live_view.py --replay recordings\euroc_<stamp> --speed 2
```

`09` starts `methods/cuvslam-vio/10_cuvslam_bridge_server.py` inside WSL2 (`wsl.exe ... bash methods/cuvslam-vio/tools/run_bridge_server.sh`,
lines prefixed `[wsl]`), waits for `BRIDGE LISTENING`, opens the camera, sends the calibration, waits for
`Engine ready`, then streams. Nothing has to be started by hand and nothing is installed. The window:

- **camera image**, the frame the last pose belongs to, with the tracked features as green dots; its border is
  green while poses arrive and red while frames are skipped;
- **top view**: the trajectory on a metric grid (spacing printed top right), the start as a cross, the camera as
  a triangle pointing where it looks; coasted stretches (estimate without vision, section 6.3) are orange and
  the state reads `COASTING`, a red dot marks frames that got neither a pose nor an estimate; with `--slam`
  the SLAM trajectory is a thin magenta line;
- **height** over the last 60 s, and the numbers: position, distance from the start, distance travelled (steps
  under 2 cm are not counted, so a camera at rest reads 0), poses / skipped / engine restarts, frames the bridge
  had to drop, poses per second, engine time and capture-to-pose delay.

In `vio` mode the view is levelled with the gravity the tracker reports, so "up" is real height even when the
camera started tilted; in `stereo` mode the start pose defines level. `q` or Esc stops, `c` clears the drawn
trail (the tracker keeps its origin), `f` toggles the features. `--headless` prints one line per second instead
of opening a window; `--snapshot FILE.png` writes the last view.

A run lasts until `q` (`--duration S` stops it by itself). The camera heats up: the window shows how long
it has been on and turns that orange after five minutes; the script prints `Camera released: safe to
unplug.` when the device is closed. Tracking options are those of `08`
(`--mode`, `--no-guard`, `--max-speed`, `--max-rot-rate`, `--guard-reset-after`, `--imu-calibration`).
`--server HOST:PORT` uses a server that is already running, `--local-server` runs the engine in the same
process (Linux with the wheel), `--fake-cuvslam` is a dry run with synthetic poses. Exit codes: 0 finished,
1 stream or tracking error, 2 no camera / unusable recording, 3 the engine could not be started or reached
(the `[wsl]` lines say why; `python methods/cuvslam-vio/tools/check_env.py` inside WSL is the next step).

Measured 2026-09-29 (section 9): 451 of 451 camera frames tracked at 30 fps, none dropped, 25-34 ms from
capture to pose; the bridge queue holds one second of frames so the slow first `vio` frame of the engine
(0.3-0.5 s) costs a moment of delay instead of frames.

---

## 4. Evaluating a run

### 4.1 Read the summary first

`08` ends with a block of this shape. **The block below is a real run**: `euroc_20260920_145714`
(640x400, 20 s, camera still on the desk in daylight) in `vio` mode on WSL2, 2026-09-20 (section 9):

```
Summary: mode vio (Inertial) on dataset /home/<user>/datasets/euroc_20260920_145714 | cuvslam 17.0.0+57f42cc
  frames: seen 600, warm-up 0, attempted 600, tracked 600 (100.0 %), failed 0, resets 0, out-of-order 0, gaps 0 (max 0.0 ms)
  timing: 19.97 s of data in 4.96 s wall, mean 121.2 fps (engine 342.4 fps, track 2.92 ms mean / 317.12 ms max)
  imu: registered 4004, held back 3, dropped old 0, out of order 0, gaps 1061 (max 8.9 ms), frames without imu 0, gravity (-0.08, -0.13, -9.81) m/s^2
  trajectory: path length 0.373 m, end-point offset 0.007 m, max from start 0.009 m, end (+0.004, +0.005, -0.001)
  stopped by: end
  TUM: /home/<user>/datasets/euroc_20260920_145714/trajectory_vio.tum (600 rows)
```

(The `imu: gaps 1061` count came from the original 6 ms threshold; the default is now 10 ms and reads 0 for this IMU.)

| Line | Meaning | What good looks like (expectations, section 9) |
|---|---|---|
| `tracked N (%)`, `failed` | frames whose `world_from_rig` came back; `failed` = `None` poses | >= 99 % tracked on a lit, slow loop; a handful of failures at fast turns is normal, clusters are not |
| `resets` | a valid pose after a failure: cuVSLAM restarted its world frame (new segment); the guard's engine resets are included | 0. Each reset makes the end-point offset meaningless unless `--stitch-resets` was used |
| `guard: N frame(s) skipped (...)` | poses the starvation guard dropped, by rule (starved / degenerate / jump / spin), its engine resets, and the min/median/max camera-0 observation count | a handful on a walk past blank walls; hundreds mean the limits are too tight for the motion (raise `--max-speed` / `--max-rot-rate`) |
| `gaps (max ms)` | image gaps above 35 ms (the examples' jitter threshold); `max` is the largest gap that exceeded it, so `0.0 ms` when `gaps` is 0 | 0 gaps. A few gaps with `max` under ~70 ms are single dropped frames; `max` above 1000 ms (`max_frame_delta_s`) means a restart (6.1) |
| `imu: gaps (max ms)`, `frames without imu` | IMU gaps above 10 ms (`max` is the largest such gap, `0.0 ms` when there is none); frames with no sample since the previous one | 0 / 0; a few 8-9 ms gaps per minute are the BNO086 itself (6.2) |
| `gravity` (`vio`) | `get_last_gravity()` in the rig frame (y down) | about `(0, +9.8, 0)` with the camera upright at the start. Measured 2026-09-20 with the camera lying on its back (lens up): `(-0.08, -0.13, -9.81)`, i.e. gravity along -z of the rig, which is what that pose should give; `(0, -9.8, 0)` with an *upright* camera would mean the IMU frame/extrinsics are wrong (section 6.7) |
| `track ms mean / max` | engine time per frame | well below the frame period; on the Jetson keep the mean under ~25 ms at 30 fps or drop to 640x400 / `Performance` |
| `path length` | sum of pose-to-pose distances | matches the walked distance within ~5 % (pace it out) |
| `end-point offset` | distance between the first and last pose: the **return-to-start drift** when you ended on the tape mark | see 4.2 |
| `max from start` | farthest excursion | matches the room |

The `--summary` JSON carries every counter (`tracking.*`), the rig as built (`rig`), the dataset's own
counts and validation warnings, and the arguments, so a run can be compared later without re-running.

### 4.2 Return-to-start drift

With the loop protocol of section 3.2 (start and end on the same mark, same heading, still at both
ends), **drift = end-point offset / path length**. There is no motion-capture ground truth in this
project, so this ratio, the shuttle test (forward and back: the end point must land on the start) and
the stereo-vs-vio agreement are the three numbers that decide whether a recording is good.

| Drift (offset / path) | Reading |
|---|---|
| < 1 % (e.g. < 20 cm on a 20 m loop) | good stereo VO for a hand-held indoor loop; NVIDIA's release KPI for stereo odometry on KITTI (a car, outdoors) is 0.85 % translational ATE, so do not expect much better |
| 1-3 % | usable for first flights; look at where it accumulates (`evo_traj --plot`: a bend usually coincides with a fast turn or a blank wall) |
| > 5 %, or resets > 0 | something is wrong with the *inputs* (calibration, sync, dropped frames, lighting), not with the algorithm: NVIDIA's troubleshooting guide says so in its first sentence. Section 6 |

Also compare the two modes on the same recording: `stereo` and `vio` should agree to a few percent
of the path length; if `vio` is clearly worse, suspect the IMU extrinsics or noise parameters
(section 6.7) before anything else, because NVIDIA states IMU fusion adds robustness, not accuracy.

**First measurements (WSL2, 2026-09-20, still camera, no loop yet):** in daylight (`euroc_20260920_145714`,
20 s) the end-point offset was 0.005 m (`stereo`) and 0.007 m (`vio`) with 600/600 frames tracked and
no resets, and NVIDIA's unmodified `track_euroc.py` produced the identical numbers. In a dark room
(`euroc_20260920_034416`, mean brightness 8-16/255) the same still camera drifted 0.347 m (`stereo`) and
1.335 m (`vio`) in 10 s. Lighting and texture decide the result before anything in this table does.

**Full-resolution loop (2026-09-29, 57 s, 1280x800, common room):** `vio` with the guard 2.3 % (8 of 10 runs;
3.6 % in the other 2), `stereo` with the guard 2.7 % (8 of 10; 13.9 % in 2); the same walk downscaled to
640x400 gave 12-15 %, and without the guard `stereo` drifted 12.5 % and `vio` ran away. So at full
resolution, with the guard, a lit room with some furniture reaches the "usable" band of the table; the
engine is not strictly repeatable (about one run in five differs), so judge a configuration on several
runs. Numbers per configuration: [README](../../../README.md) "Verification status".

**First walked loops (2026-09-21, three 60 s hand-held loops in a dorm room at night, 640x400, back to
the floor mark):** with the starvation guard (section 6.3) the end-point drift was 2.9 % / 10 % / 21 %
in `stereo` and 12 % / 2.4 % / 11 % in `vio`; without it 18-60 % (stereo) and 6-19 % (vio), because every
walk contained blank-wall stretches (4-14 % of the frames) on which the engine returned garbage poses.
So the table above is still the target, not the measurement: the walks had too many featureless
views to reach it. The [README](../../../README.md) "Verification status" has the per-recording numbers.

### 4.3 evo on the TUM files

The TUM file is `timestamp_s tx ty tz qx qy qz qw` with `#` comment lines (the header records the
source, mode, cuvslam version, frame conventions and the timestamp domain). `evo` is installed by both
setup scripts.

```bash
evo_traj tum trajectory_stereo.tum                                   # stats only: poses, path length, duration (headless)
evo_traj tum trajectory_stereo.tum --plot --plot_mode xz             # floor plan (y is down, so x-z is the floor)
evo_traj tum trajectory_stereo.tum --plot_mode xz --save_plot run1.png   # no display: writes run1_trajectories.png etc.
evo_traj tum trajectory_stereo.tum trajectory_vio.tum --ref trajectory_stereo.tum --align --plot --plot_mode xz
evo_ape tum trajectory_stereo.tum trajectory_vio.tum -a --plot       # APE of vio against stereo after SE(3) alignment
evo_ape tum trajectory_stereo.tum trajectory_vio.tum --align_origin  # no alignment beyond the common start (both start at identity anyway)
evo_rpe tum trajectory_stereo.tum trajectory_vio.tum -d 1 -u m -a    # relative error per metre travelled
evo_ape tum run_a.tum run_b.tum -a --t_max_diff 0.01                 # two runs of the same dataset (association by timestamp, 10 ms)
```

Two runs of the same recording share timestamps (Unix-epoch ns from the recorder), so evo associates
them directly; two *live* runs do not overlap in time and can only be compared through their summaries
(`--t_offset` can shift one file if you insist). `evo_ape`/`evo_rpe` with `-a` need a trajectory with
some 3-D extent: on a perfectly straight path Umeyama alignment reports "Degenerate covariance rank",
which is a property of the path, not an error in the file. `--plot` needs a display (WSLg, the Jetson
desktop); `--save_plot` does not.

When a reference exists later (a motion-capture room, a total station, or a long SLAM run treated as
truth), `evo_ape tum ref.tum est.tum -a` is the number to report, `evo_rpe ... -d 1 -u m` the local one.

---

## 5. The independent check: NVIDIA's own EuRoC example

The recorder writes the EuRoC ASL layout so that `examples/euroc/track_euroc.py` from the v17.0.0 tag
loads the folder **unmodified**: NVIDIA's loader, NVIDIA's rig construction, NVIDIA's mode selection,
none of `cuvslam_oak`. If it tracks and `08` does not, the bug is ours; if neither tracks, the
recording is the problem. The script has no CLI: it hard-codes `dataset/mav0` next to itself and
opens a rerun window (`rerun-sdk`, installed with `WITH_RERUN=1`).

```bash
git clone --depth 1 --branch v17.0.0 https://github.com/nvidia-isaac/cuVSLAM.git ~/cuvslam-src
ln -sfn ~/datasets/euroc_<stamp> ~/cuvslam-src/examples/euroc/dataset   # the recording root: it contains mav0/
cd ~/cuvslam-src/examples/euroc
python track_euroc.py                     # default mode Inertial; edit line 90 to OdometryMode.Multicamera for stereo-only
```

Expected: `cuVSLAM Tracker initilized with odometry mode: OdometryMode.Inertial` (sic), a rerun window
with both camera views, the trajectory and, in inertial mode, a red gravity arrow pointing down.
`Warning: Failed to track frame N` should be rare; `Warning: No IMU measurements between timestamps`
means the IMU csv has gaps (it is the example's own check, not the tracker's). Without a display,
change `rr.init("cuVSLAM Visualizer", spawn=True)` to `spawn=False` and add `rr.save("run.rrd")`. The
example keeps its trajectory in memory and writes no file; compare its end point on screen with the
`08` summary. Two constraints of the loader worth remembering: it builds a Brown model from exactly
four zero coefficients (our undistorted PNGs; a `--raw` recording would be mis-modelled), and it reads
`imu0/sensor.yaml` even in stereo mode (the recorder always writes one, with an identity placeholder
and a warning if the EEPROM had no IMU extrinsics). Details: [cuvslam-setup.md](cuvslam-setup.md)
section 4.3, [cuvslam-dataset-format.md](../../../datasets/dataset-format.md) sections 9-10.

---

## 6. Troubleshooting

Quick table, then the longer entries. Every script prints one human-readable stderr line and exits
non-zero instead of a traceback; a traceback is a bug worth reporting with the command line.

| Symptom | Where it shows | Go to |
|---|---|---|
| `png queue 40/64`, `drops N (stereo_write_queue_dropped ...)` while recording | 07 status line, `dataset.json.drops` | 6.1 |
| `gaps N (max 120.0 ms)`, `resets` > 0, `frame_gaps_over_max` in the JSON | 08 summary | 6.1, 6.3 |
| `imu: gaps N`, `frames without imu N`, `--mode vio needs IMU data but ...` | 08 summary / stderr | 6.2 |
| `tracked 62.0 %`, `failed 500`, end-point offset metres off | 08 summary | 6.3 |
| `cuvslam (PyCuVSLAM 17.0.0) is not importable: ...` exit 3 | 08 stderr | 6.4 |
| `Device: ... usb=HIGH (WARNING: not USB3)` | 07/08 start-up | 6.5 |
| black/grey PNGs, long exposure, blur, `tracked` collapsing indoors at night | look at `mav0/cam0/data/*.png` | 6.6 |
| `gravity (+0.1, -9.8, +0.0)` or gravity on z in `vio` | 08 summary | 6.7 |
| `dataset ... failed validation: ...` exit 2 | 08 stderr (details on stdout) | 6.8 |
| tracking a dataset runs at 3 fps in WSL2 | | dataset read through `/mnt/c`: copy to `~/datasets` (3.4) |
| `No OAK device found ...` / `Device X not found ...` exit 2 | 07/08 stderr | camera unplugged, held by another process, or rebooting after the previous script (retries ride that out unless `--device-id` was given); [README](../../../README.md) troubleshooting |
| rerun viewer does not open in WSL2 | `--rerun` | `WGPU_BACKEND=gl`, `unset WAYLAND_DISPLAY`, or skip `--rerun` ([cuvslam-setup.md](cuvslam-setup.md) 2.6) |
| `evo_traj --plot` says no display | | drop `--plot` (stats print anyway) or `--save_plot` |

### 6.1 Frame drops

Three places can lose frames, and each one reports it:

1. **The recorder's PNG writer** (host disk). `png queue` climbing in the status line, then
   `drops.stereo_write_queue_dropped` in `dataset.json` and in the exit summary. Fixes, in order:
   `--out` on a local SSD outside OneDrive (OneDrive holds files open and adds I/O), `--width 640
   --height 400`, `--png-compression 0`, a lower `--fps`. The stream itself is never stalled by the
   writer, so the IMU csv stays complete; only pairs vanish, which the tracker later sees as gaps.
2. **The camera link / host CPU** (USB 2 fallback, a busy laptop, a display-heavy preview). The
   source counts `stereo_sequence_gaps` (missing sequence numbers), `stereo_sync_violations` (left and
   right stamps more than 1 ms apart, pair dropped), `frames_out_of_order`; the recorder folds them
   into `dataset.json.drops` and `measured.max_stereo_gap_ms`. Fixes: USB 3 (6.5), `--headless`
   (default) instead of `--preview`, nothing else on the camera, 640x400.
3. **The tracker**: `gaps N (max X ms)` in the summary counts image gaps above 35 ms; a gap above
   `max_frame_delta_s` (1.0 s) makes cuVSLAM return `None` poses and restart its world frame
   (`resets`). NVIDIA: missing frames "can significantly degrade quality, up to tracking lost" and
   make results non-deterministic. On a live Jetson run the same counters come from
   `Source stats: frames ..., frame gaps ..., sequence gaps ...`; if they are non-zero at 1280x800,
   run `--size 640x400`, `--multicam-mode Performance`, without `--show` (`08` opens no window unless
   asked; `--headless` is a `07` flag), and check
   `nvpmodel -q` / `jtop` for a throttled board ([jetson-orin-nano-setup.md](../../../camera/docs/jetson-orin-nano-setup.md)).

A recording with drops is still tracked; it just is not a benchmark. Re-record.

### 6.2 IMU gaps and IMU-less datasets

- `dataset.json.drops.imu_gaps` (gaps over 2.5 sample periods), `measured.max_imu_gap_ms`,
  `drops.imu_out_of_order`; in `08`: `imu: gaps N (max X ms)` (over 10 ms) and `frames without imu N`.
  The BNO086 at 200 Hz spaces samples 4.99 ms apart with occasional 8-9 ms outliers; a few gaps per
  minute are the sensor, hundreds are the link or the host. Keep `--imu-rate 200` (400 is the ceiling;
  the gyroscope jitters above it), never put another IMU sensor on the node, use USB 3, and do not
  run anything else on the camera.
- The IMU stream must start before the first pair: the recorder skips leading pairs until an IMU row
  precedes them (`record.frames_skipped_before_imu`, normally 1) and warns if none arrives within 1 s.
  `08` and NVIDIA's `track_euroc.py` both warn about frame intervals without samples.
- `--mode vio needs IMU data but imu0/data.csv has no samples` / `... identity placeholder of a
  stereo-only recording` / `dataset.json says imu.available = false`: the calibration had no IMU
  extrinsics or the recording has no IMU rows. Track it with `--mode stereo`; a placeholder
  `imu0/sensor.yaml` exists only so NVIDIA's loader does not crash.
- `imu_dropped_old` in the summary: samples older than the last tracker timestamp, dropped (a few
  right after warm-up are normal). `held back` at the very end is the tail after the last image.

### 6.3 Tracking failures and resets

Order of investigation (NVIDIA's `TROUBLESHOOTING.md`, condensed; the root cause "is almost always in
the inputs, not the algorithm parameters"):

1. **Look at the images** (`mav0/cam0/data/*.png` in any viewer): sharp, evenly exposed, no clipped
   highlights or crushed shadows, corners visible everywhere. Blur or black frames: section 6.6.
2. **Sync and timestamps**: `dataset.json.measured.max_left_right_skew_us` must be tens of
   microseconds (11-16 on this unit); `stereo_sync_violations` 0; frame stamps strictly increasing
   (the reader rejects the rest). A live run prints the same from the source stats.
3. **Calibration**: `08` prints the rig it built; `baseline: 0.0750 m`, `cam1 t=(+0.0750, ...)`,
   `imu: rig_from_imu t=(-0.0006, -0.0011, +0.0011) m` at CAM_B are this unit's numbers
   ([oak-to-cuvslam-calibration.md](oak-to-cuvslam-calibration.md) section 8). A rig with the right
   camera at negative x, a baseline of 7.5 (centimetres leaked in) or a 3.7 cm IMU offset when the
   rig is CAM_B is wrong. `--calibration recordings/euroc_<stamp>/calibration.json` rebuilds the rig
   from the EEPROM dump through `rig.build_rig` as an A/B against the yaml.
4. **Mode**: run `--mode stereo` first. If stereo works and `vio` fails, it is the IMU (6.7). If
   neither works, run NVIDIA's example (section 5); if that fails too, re-record.
5. **Motion and scene**: fast pans, blank walls, repetitive texture (tiles, blinds), a person walking
   through the frame, glass and reflections. Slow down, add texture, keep 30 fps. cuVSLAM re-tracks a
   frame without its motion model automatically when prediction fails, so a single failed frame is
   not a loss; a `None` pose that persists is.
6. **Borders**: live raw frames use the example's 50/0/70/70 masks (scaled with the size) to keep
   features out of the strongly distorted edge; datasets are undistorted and use none. Both are set
   automatically; nothing to tune here first.
7. Only then parameters: `--multicam-mode Precision` (default) vs `Performance`; `use_denoising`
   and `use_motion_model` live in `cuvslam_oak.tracking.TrackConfig` (not on the CLI) for a small
   experiment script.

For plotting a run with losses, `--stitch-resets` keeps the file continuous; the summary still counts
every reset.

**Blank walls at close range are the failure mode measured on this camera (2026-09-21).** On a bare
wall, door edge or light switch filling a 640x400 frame from under a metre, the engine keeps returning
poses (never `None`), reports its 450-feature cap (it detects sensor noise as features), and the poses
are garbage: 0.4-52 m steps in one frame, rotations up to 63 deg, and in `vio` mode a run-away of
hundreds of metres. The **starvation guard** in `cuvslam_oak.tracking` (on by default) skips a pose when
the step since the last accepted pose exceeds `--max-speed` (3 m/s) or `--max-rot-rate` (720 deg/s), or
when the position covariance is exactly zero (never on the first pose of a segment); it re-anchors the
trajectory across the skipped stretch and, after `--guard-reset-after` (30) consecutive skipped frames,
re-creates the tracker and chains it onto the last good pose. Without the reset the engine never
recovered (281-564 frames lost per walk); with it the walks went from 18-60 % to 3-21 % drift (stereo)
and from 6-19 % to 2-12 % (vio). The summary prints `guard: N frame(s) skipped (starved a, degenerate b,
jump c, spin d), R engine reset(s)` and the JSON carries `guard_gaps`; `--no-guard` reproduces the raw
engine output. `--min-observations` exists but stays 0: the count did not drop on this camera's starved
frames. The guard treats symptoms; the cure is the scene: keep textured surfaces in view, stay a metre
from walls, no doorways, and record at 1280x800 where fine wall texture survives.

**Coasting (2026-09-30, on by default).** While the engine gives no usable pose (guard skip, loss,
non-finite), the loop still writes a row per frame: the rotation follows the gyroscope (the IMU is read in
`stereo` mode too, never registered with the engine), the position continues at the median velocity of the
last second, decaying with a 1 s time constant, for at most 3 s (`--coast-max`); the returning pose is
re-anchored onto the coasted estimate for its own time, also after a loss. Rows made this way are flagged:
`frames_coasted` and `coast_gaps` in the summary, `coasted: True` on the `PoseEvent`, orange in the live
view, and a `# N of M rows are coasted estimates` line in a TUM saved by `09`. Measured on the four walks
([README](../../../README.md) "Verification status"), the gyro removes most of the heading error a dropout used
to leave (22.7 -> 2.1 deg at the end of the 2026-09-29 walk in `vio`, 59.6 -> 11.3 deg on `001246`), while
the position coasting is neutral at best because the poses just before a guard event are already corrupted
and set the velocity; `--no-coast` restores the frozen behaviour, `--coast-velocity-window 0` keeps the
gyro and holds the position.

### 6.4 `cuvslam not importable` (exit 3)

`cuvslam (PyCuVSLAM 17.0.0) is not importable: ModuleNotFoundError: No module named 'cuvslam'; install
PyCuVSLAM 17.0.0 as described in methods/cuvslam-vio/docs/cuvslam-setup.md (WSL2 / Jetson), or pass --fake-cuvslam for a
dry run with synthetic poses`

- **Windows**: expected, there is no wheel. `--fake-cuvslam` (or `CUVSLAM_FAKE=1`) for the dry run.
- **WSL2 / Jetson**: the venv is not active (`source ~/venvs/cuvslam/bin/activate` /
  `source ~/DroneDepthCamTesting/.venv/bin/activate`), or the wheel was never installed (`bash
  methods/cuvslam-vio/tools/setup_wsl2.sh` / `methods/cuvslam-vio/tools/setup_jetson.sh`, re-runnable), or the interpreter is not 3.10.
  `python methods/cuvslam-vio/tools/check_env.py` says which.
- `ImportError: libcublas.so.12: cannot open shared object file` (also `libcusolver.so.11`,
  `libcusparse.so.12`): the CUDA 12 libraries are missing or not in the loader cache. WSL2:
  `sudo apt-get install cuda-toolkit-12-6`; Jetson: `sudo apt-get install nvidia-jetpack`; then
  `sudo ldconfig`.
- `warm_up_gpu()` raises: the GPU is not visible (WSL2: `nvidia-smi` must work inside Ubuntu and
  `libcuda.so.1` must come from `/usr/lib/wsl/lib`; never install a Linux driver in WSL). Jetson:
  `jtop` must show the GPU.
- A `RuntimeWarning: cuvslam X found but cuvslam_oak targets the 17.x API`: a newer wheel changed the
  `Tracker` constructor; install the v17.0.0 wheel.
- The fake is never chosen silently on Linux: if `08` prints the FAKE banner there, you asked for it.

### 6.5 USB 2 boot (`usb=HIGH`)

`Device: OAK-D-S2 (OAK-D-S2-AF) id=... usb=HIGH (WARNING: not USB3)` plus a `!!! WARNING: USB speed
is HIGH instead of SUPER` line. Two 1280x800 GRAY8 streams at 30 fps are ~61 MB/s before the IMU,
more than a USB 2 link carries, so expect sequence gaps and sync violations. It happened on one of
five back-to-back boots on this laptop with a good cable and port, so first just re-run (or
re-plug); only a repeatable `HIGH` points at the cable (USB 3, < 2 m), the port, or a USB 2 hub.
On the Jetson use a Type-A port directly ([jetson-orin-nano-setup.md](../../../camera/docs/jetson-orin-nano-setup.md)
step 6). As a stopgap, `--width 640 --height 400` (07) / `--size 640x400` (08) fits in USB 2. The
camera over usbipd into WSL2 is a USB/IP tunnel: functional tests only, never timing.

### 6.6 Dark scenes and exposure

The OV9282 auto-exposure runs up to 30 ms; in a dim room every frame is long-exposed, so any motion
blurs, and cuVSLAM loses corners. Symptoms: grey, soft PNGs; `tracked %` that is fine while the camera
is still and collapses when it moves; failures clustered in the dark part of a loop. What helps, in
order: more light (the cheapest fix by far), slower motion, shorter loops in the lit part, and
640x400 (NVIDIA: "frame rate is often more important than resolution"; a resized mode also smooths
noise, "validate it instead of assuming"). Avoid exposure "pumping" (walking past a window or a
lamp): the mid-exposure timestamps stay right, but the feature set changes abruptly. The recorder's
1.5 s warm-up exists because the first second after start is taken at maximum gain; if the room is
dark that second is longer, so hold still at the start. `use_denoising` (a `TrackConfig` field, not
a CLI flag) is NVIDIA's suggestion for noisy low-light images; compare on the same recording before
adopting it. Check the images cuVSLAM actually receives, not a preview.

### 6.7 `vio` gravity wrong, `vio` worse than `stereo`

`get_last_gravity()` is reported in the rig frame (y down): about `(0, +9.8, 0)` with the camera
upright. `(0, -9.8, 0)` or gravity along z means the IMU samples are in a different frame than
`rig_from_imu` assumes. This unit pairs the `*_RAW` streams with the EEPROM rotation
`diag(1,-1,-1)`; if the firmware delivered frame-aligned data on `RAW` (older firmware did), the
rotation should be the identity instead. Bench test from [oak-to-cuvslam-calibration.md](oak-to-cuvslam-calibration.md)
section 4.4: camera upright, lens horizontal, `python camera/04_imu.py --duration 10 --headless`
must show `ACCELEROMETER_RAW` about `(0, +9.81, 0)`. The IMU noise densities in `imu0/sensor.yaml`
are NVIDIA's RealSense placeholders (the EEPROM's own block has undocumented units) until an Allan
measurement replaces them; too-small values make the filter trust a drifting IMU. NVIDIA's
`debug_imu_mode` (rotation from the IMU alone, translation locked: rotate the camera and compare) is
the definitive check; it is not a `TrackConfig` field, so it takes a small script that builds
`Tracker.OdometryConfig(odometry_mode=Inertial, debug_imu_mode=True)` itself and passes the tracker to
`run_tracking(..., tracker=(tracker, odom_cfg))`.

**Measured on this unit (2026-09-29).** The frame is right: upright and still, `ACCELEROMETER_RAW` reads about
`(0, +8.9, +1.9)` and the tracker's gravity is `(-0.67, +9.71, +1.22)`. The accelerometer is biased, though:
a still camera reads 9.05 to 10.44 m/s^2 depending on which way is up (y bias -0.69 m/s^2). To measure and
store the correction, record 60 s with the camera resting still on each of its six faces in turn (any
order, levelling not needed), then

```bash
python datasets/fit_imu_calibration.py recordings/euroc_<stamp>            # writes datasets/calibration/imu_<device id>.json
python datasets/fit_imu_calibration.py --check recordings/euroc_<other>    # |accel| of every still stretch, before and after
python methods/cuvslam-vio/08_cuvslam_track.py ~/datasets/euroc_<stamp> --mode vio --imu-calibration auto --summary
```

On four walks the correction changed the drift by less than the run-to-run spread, so it is off by
default; the engine evidently absorbs a constant bias. Refit when `--check` is off by more than about
0.1 m/s^2 (the bias moves with temperature).

### 6.8 A dataset fails validation (exit 2)

`08` prints every error and warning on stdout and one summarising line on stderr. Typical:
`frame timestamps not strictly increasing`, `N PNG file(s) missing`, `dataset.json counts.stereo_pairs
= X but data.csv has Y pairs`, `cam0/sensor_recalibrated.yaml exists` (never create it: NVIDIA's loader
would switch to a fisheye model). Warnings (rates off by more than 10 %, gaps, recorder drops) do not
stop the run; `--verbose` lists them all and the JSON keeps them. A folder without `dataset.json` (a
foreign EuRoC sequence, or a recording aborted with a second Ctrl-C) loads with one warning.

Repairing a damaged folder:
`python -c "from cuvslam_oak.dataset import trim_dataset; print(trim_dataset('recordings/euroc_<stamp>/mav0'))"`
rewrites the two camera csv files to the pairs whose PNGs both exist and deletes unreferenced PNGs
(it returns `rows_kept` / `rows_removed` / `pngs_removed`). That fixes `PNG file(s) missing` and
unequal pair counts, but it **does not touch `dataset.json`**: if that file exists, its
`counts.stereo_pairs` (and `counts.imu_samples`) now disagree with the csv files and `08` still exits 2
with the counts error. Either delete `dataset.json` (the folder then loads as a foreign sequence, with
the one warning above) or edit those two counts by hand. A recording that lost its `dataset.json` to a
second Ctrl-C needs only the trim.

### 6.9 Keep the camera cool and free

The owner's rule for this unit: never leave it booted. Both scripts release the device at exit (also
on Ctrl-C); a stray `python.exe` holding it shows up as "no device" for everyone else (one process per
camera). Unplug it when idle; `00_device_info.py --duration 60` logs the chip temperature.

---

## 7. Jetson day-one checklist (15-20 minutes)

Step 0 at home; the rest at the venue with a flashed board and the camera on a short USB 3 cable in a
**Type-A** port. Times assume `nvidia-jetpack` is already on the image; otherwise add the apt download.

| # | Min | Do | Done when |
|---|---|---|---|
| 0 | home | Flash JetPack 6.2.x ([jetson-orin-nano-setup.md](../../../camera/docs/jetson-orin-nano-setup.md) step 1). Put on a USB stick: the aarch64 wheel `cuvslam-17.0.0+cu12-cp310-cp310-manylinux_2_35_aarch64.whl` (113 MB, sha256 in [cuvslam-setup.md](cuvslam-setup.md) 1.1), this repo without `.venv/`, `recordings/`, `snapshots/`, and one good recording folder `euroc_<stamp>` made on Windows with `07` before travel (section 3.2; none exists yet, section 9, so record one as soon as the camera has been recorded from at all; without it, do step 7 before step 6) | stick in pocket |
| 1 | 2 | Boot, create the user, join the network (`ssh` is fine) | shell prompt |
| 2 | 1 | `sudo nvpmodel -q` (note the mode; MAXN per the Jetson doc step 2) | mode noted |
| 3 | 1 | `rsync -av --exclude .venv --exclude recordings --exclude snapshots /media/$USER/<stick>/DroneDepthCamTesting/ ~/DroneDepthCamTesting/` | `ls ~/DroneDepthCamTesting/tools` |
| 4 | 5-8 | `CUVSLAM_WHEEL_DIR=/media/$USER/<stick> bash ~/DroneDepthCamTesting/methods/cuvslam-vio/tools/setup_jetson.sh` (udev rule, venv, wheel with sha256 check, depthai, evo, jtop, smoke test) | `warm_up_gpu(): ok`; `python methods/cuvslam-vio/tools/check_env.py --warm-up` shows `cuvslam: importable version=17.0.0` |
| 5 | 1 | Re-plug the camera (udev rule), `source ~/DroneDepthCamTesting/.venv/bin/activate && cd ~/DroneDepthCamTesting && timeout 90 python camera/00_device_info.py` | `Device: OAK-D-S2 ... usb=SUPER` |
| 6 | 1 | `timeout 90 python methods/cuvslam-vio/08_cuvslam_track.py ~/euroc_<stamp> --mode stereo --summary` on the recording from the stick (copy it to the Jetson's disk first) | a `trajectory_stereo.tum`, `tracked` near 100 %, `resets 0`, end-point offset written down |
| 7 | 2 | `timeout 120 python datasets/07_record_dataset.py --duration 30 --note "venue loop"` while walking the camera in a slow loop back to the mark | `recordings/euroc_<stamp>/` with `drops 0` |
| 8 | 1 | `python methods/cuvslam-vio/08_cuvslam_track.py recordings/euroc_<stamp> --mode stereo --summary`, then `--mode vio --summary` | two TUM files; `gravity (0, +9.8, 0)` in the vio summary |
| 9 | 1 | `timeout 90 python methods/cuvslam-vio/08_cuvslam_track.py --live --mode stereo --duration 30 --summary` with `jtop` in a second terminal | `track ... ms mean` under the frame period, GPU load visible, a `recordings/trajectory_stereo_<stamp>.tum` |
| 10 | 1 | `evo_traj tum recordings/euroc_<stamp>/trajectory_stereo.tum recordings/euroc_<stamp>/trajectory_vio.tum --ref recordings/euroc_<stamp>/trajectory_stereo.tum --align` (add `--plot --plot_mode xz` on the desktop) | numbers written down: tracked %, resets, path length, end-point offset, track ms, for stereo and vio |

If step 6 fails with exit 3, section 6.4; exit 2 at step 5, [jetson-orin-nano-setup.md](../../../camera/docs/jetson-orin-nano-setup.md)
troubleshooting (udev, `usb=HIGH`); a `PlgSrcMipi` crash dump, the same doc's notes. Keep the camera
unplugged between steps.

---

## 8. Files this guide refers to

| Path | Role |
|---|---|
| [datasets/07_record_dataset.py](../../../datasets/07_record_dataset.py) | recorder (`--help` for every flag) |
| [08_cuvslam_track.py](../08_cuvslam_track.py) | tracker, dataset or live, TUM + JSON summary |
| [09_cuvslam_live_view.py](../09_cuvslam_live_view.py), [10_cuvslam_bridge_server.py](../10_cuvslam_bridge_server.py), [tools/run_bridge_server.sh](../tools/run_bridge_server.sh) | live view: camera or recording on Windows, engine in WSL2 (section 3.6) |
| [cuvslam_oak/bridge.py](../../../cuvslam_oak/bridge.py), [cuvslam_oak/live_view.py](../../../cuvslam_oak/live_view.py), [cuvslam_oak/imu_calib.py](../../../cuvslam_oak/imu_calib.py), [datasets/fit_imu_calibration.py](../../../datasets/fit_imu_calibration.py) | the bridge protocol, the drawing, the IMU correction and its fit |
| [camera/00_device_info.py](../../../camera/00_device_info.py), [camera/04_imu.py](../../../camera/04_imu.py) | USB-speed check; IMU at-rest check |
| [camera/common.py](../../../camera/common.py) | `--device-id`, `--duration`, `--headless`, `open_device` (exit 2 rules) |
| [cuvslam_oak/__init__.py](../../../cuvslam_oak/__init__.py), [cuvslam_oak/rig.py](../../../cuvslam_oak/rig.py), [cuvslam_oak/oak_source.py](../../../cuvslam_oak/oak_source.py), [cuvslam_oak/dataset.py](../../../cuvslam_oak/dataset.py), [cuvslam_oak/tracking.py](../../../cuvslam_oak/tracking.py), [cuvslam_oak/fake_cuvslam.py](../../../cuvslam_oak/fake_cuvslam.py) | the package (section 1.2); each module docstring is its reference |
| [tools/check_env.py](../tools/check_env.py), [tools/setup_wsl2.sh](../tools/setup_wsl2.sh), [tools/setup_jetson.sh](../tools/setup_jetson.sh) | host check and installs |
| [cuvslam-setup.md](cuvslam-setup.md) | WSL2 / Jetson install, wheel digests, pins, usbipd, NVIDIA example |
| [cuvslam-api-v17.md](cuvslam-api-v17.md) | the exact PyCuVSLAM 17.0.0 API and the official examples' call patterns |
| [oak-to-cuvslam-calibration.md](oak-to-cuvslam-calibration.md) | EEPROM -> Rig: intrinsics scaling, Polynomial 8, extrinsics direction, IMU chain, noise, timestamps |
| [cuvslam-dataset-format.md](../../../datasets/dataset-format.md) | the EuRoC layout the recorder writes and NVIDIA's loader reads; `dataset.json`; TUM |
| [depthai-v3-cheatsheet.md](../../../camera/docs/depthai-v3-cheatsheet.md), [jetson-orin-nano-setup.md](../../../camera/docs/jetson-orin-nano-setup.md), [oak-d-s2-specs.md](../../../camera/docs/oak-d-s2-specs.md) | DepthAI 3.10.0 facts, the board, the camera |

---

## 9. Known unverified

**Update 2026-09-20 (afternoon): the integration has now run on CUDA in WSL2 on the laptop.**
`methods/cuvslam-vio/tools/setup_wsl2.sh` installed `cuda-toolkit-12-6` and the `cuvslam` 17.0.0+cu12 wheel on WSL2
Ubuntu 22.04.5 (Python 3.10.12, RTX 1000 Ada through Windows driver 581.95), `warm_up_gpu()` passed,
and `08` tracked two real recordings in both modes with NVIDIA's unmodified `track_euroc.py` giving the
identical end points (numbers in the [README](../../../README.md) "Verification status" and section 4.2).
Still not run anywhere: `08 --live`, `--rerun`, `--show`, the Jetson path (`methods/cuvslam-vio/tools/setup_jetson.sh`),
a moving recording, and anything against ground truth. The table below keeps the per-component
detail; rows changed by the WSL2 runs say so.

| Component | Verified (how, where) | Not verified |
|---|---|---|
| `cuvslam_oak/fake_cuvslam.py` | Windows: NVIDIA's own v17.0.0 `python/test` suites (`test_api`, `test_bindings`, `test_image_format`, `test_tracking`) run against it, 70 collected: with `set_default_pose_source(lambda ts, origin: Pose())` 68 pass, 1 is skipped and only `test_api::test_public_api` fails (the documented top-level `Odometry` name); with the fake's default constant-velocity source the 8 mode/mask sub-cases of `test_tracking::test_tracking` also fail because they expect zero motion | that the real engine's error *messages* match (tests match on exception type only) |
| `cuvslam_oak/rig.py` | Windows, offline: every number of [oak-to-cuvslam-calibration.md](oak-to-cuvslam-calibration.md) recomputed from this unit's `calibration.json`; cross-checked against `dai.CalibrationHandler` (19 checks, max diff 6e-8, no device) | the first-frame `getTransformation()` intrinsics guard on a live stream |
| `cuvslam_oak/dataset.py` | Windows: synthetic round trips (frames + IMU, yaml parsed identically by pyyaml), negative cases, trim/repair | that NVIDIA's `dataset_utils.py` loads a real recording (reasoned from its source, never executed) |
| `cuvslam_oak/oak_source.py` | Windows: 100+ assertions against a stub pipeline (node graph, GRAY8/CROP, Sync threshold, IMU config, ordering, drop counting). **On the camera, 2026-09-20, through `07`** (two static headless runs under `timeout 90`; the full log is the [README](../../../README.md) "Verification status"): 451 frames delivered with 0 gaps at 1280x800 @ 30 fps and 300 at 640x400 (spacing `{33 ms: 299}`, 30.000 fps); device stamps strictly increasing, cam0 = cam1 row by row (max L/R skew 18-19 us); BNO086 at 200.2 Hz with no interval over 12.5 ms, samples starting before the first frame and ending after the last; the 640x400 binned-mode `OakSourceWarning` with intrinsics matching `getCameraIntrinsics(640x400)`; static accelerometer norm 9.916 m/s^2, gravity on IMU -z | RAW-vs-aligned IMU frame (the camera lay on its back, not upright: calibration doc 4.4 item 1 stays open); the 1 ms L/R skew rule and the watchdog never fired (nothing to reject); a moving camera; `--fps` other than 30; BNO086 spacing is bimodal (clusters at 2.9-4.8 ms and 7.6-8.9 ms), which tripped `08`'s original 6 ms IMU gap counter (README finding 2; the default is now 10 ms and the real tracker tracked 100 % through the gaps) |
| `cuvslam_oak/tracking.py` | Windows: 159 checks with the fake (stereo/vio, warm-up IMU window, held-back samples, resets/stitching, gaps, TUM round trip) | **WSL2 2026-09-20**: the real `Tracker` ran through this loop on two recordings in both modes (600/600 and 300/300 frames tracked, 0 resets, gravity `(-0.08, -0.13, -9.81)` for a camera on its back, engine 0.9-4.7 ms/frame); still unseen: a `None` pose from the real engine, resets on real data, `RuntimeError` paths **2026-09-21**: starvation guard added and measured on three walks (see 6.3); the engine re-creation on reset works with the real wheel (1-9 resets per run, no errors); `tests/test_guard.py` (10 tests) pins the logic with the fake; **2026-09-30**: coasting (`tests/test_coast.py`, 10 tests) and the gyro convention checked against the vision increments of two walks (0.02 deg per frame residual), effect on four walks measured (README); SLAM layer (`tests/test_slam.py`, 5 tests) run on every walk with the real engine: no correction, see README |
| `datasets/07_record_dataset.py` | Windows: device-free self-check (CLI, a stub source writing a real dataset that `EurocReader` validates); `--device-id nonexistent --duration 1` exits 2 with one line at once and leaves no folder. **Two recordings from the camera on 2026-09-20** ([README](../../../README.md) "Verification status"; both `--headless`, static, exit 0, about 42 s of device time): `recordings/euroc_20260920_034208` (1280x800 @ 30 fps, 15 s, 363 pairs + 3005 IMU samples, 348 MB) and `recordings/euroc_20260920_034416` (640x400, 10 s, 300 pairs + 2005 samples, 0 drops, 69 MB); `dataset.json` counts equal the csv rows, filenames are `<ts>.png` with 19-digit strictly increasing stamps; run 1 passes `EurocReader` validation and a `08 --fake-cuvslam` dry run. **PNG cost measured**: 49.8 ms/pair at 1280x800, 14.0 ms/pair at 640x400 (compression 1, OneDrive folder) | **1280x800 @ 30 fps drops pairs on this laptop** (README finding 1): the single PNG writer thread cannot keep up, the 64-pair queue fills and 86 of 449 pairs were dropped in run 1 (24.3 fps written, max gap 100 ms), so run 1 is not a benchmark recording; until fixed record at 640x400 (0 drops), or use `--png-compression 0`, `--out` on a local SSD outside OneDrive, or a lower `--fps`. Also untested: a moving recording, runs longer than 15 s, the `--preview` window **2026-09-21**: parallel PNG writer (`--png-workers`, auto = 4): 15 s at 1280x800 on the camera wrote 446/446 pairs at 30 fps with 0 write drops; four 60 s walks at 640x400 recorded drop-free (3-6 stereo-sync rejections each); `datasets/bench_png_writer.py` + `tests/test_writer_workers.py` |
| `methods/cuvslam-vio/08_cuvslam_track.py` | Windows: this guide's dry run on a 45-pair synthetic dataset with `--fake-cuvslam --mode vio --summary` (exit 0, 45 TUM rows, summary JSON); `--live --device-id nonexistent` exits 2 (with the fake) / 3 (without) with one line | **WSL2 2026-09-20**: real trajectories for `euroc_20260920_034416` and `euroc_20260920_145714` in both modes, matching NVIDIA's example exactly (README "Verification status"). Still untested: `--live` on a camera, `--rerun`, `--show` **2026-09-21**: guard flags exercised on three real walks in both modes in WSL2 (sweeps of `--max-speed` 1-5 and `--guard-reset-after` 0/10/30) |
| `methods/cuvslam-vio/09_cuvslam_live_view.py` + `methods/cuvslam-vio/10_cuvslam_bridge_server.py` | Windows: `tests/test_bridge.py` (15 tests: wire format, server and client against the fake engine, the drawing, both scripts). **WSL2 + camera 2026-09-29**: two camera runs of 20 s and 15 s (camera still on the desk), 451 of 451 frames tracked at 30 fps in the second, delay 25-34 ms; the 57 s walk replayed at recorded pace, 3.6 % drift; the window opened and closed on a 10 s replay | the view while the camera is carried around; runs longer than 60 s; `--local-server` on the Jetson; `--server` across two machines |
| TUM output and evo commands | Windows: evo 1.37.1 reads the dry-run file (`45 poses, 0.147m path length`, matching the summary); every flag in section 4.3 is in `evo_traj/evo_ape/evo_rpe tum --help`; `evo_traj --save_plot` wrote the PNGs; `evo_ape`/`evo_rpe` run without alignment | `-a` on a real (non-degenerate) trajectory; `--plot` on a display |
| `methods/cuvslam-vio/tools/setup_wsl2.sh`, `methods/cuvslam-vio/tools/setup_jetson.sh` | `setup_wsl2.sh`: **ran on this laptop 2026-09-20** (apt steps as root, then `SKIP_APT=1` as the user): toolkit 12.6.3-1, wheel sha256 verified, smoke test ok, `check_env.py` matched the WSL column of [cuvslam-setup.md](cuvslam-setup.md) 4.1. `setup_jetson.sh`: `bash -n` only | `setup_jetson.sh` on a Jetson; the Jetson column of the 4.1 table |
| NVIDIA `track_euroc.py` on our data | **WSL2 2026-09-20**: run unmodified (headless copy: `spawn=False`) on both recordings in Inertial and Multicamera modes; it loaded our folders and its end points equal `08`'s to the millimetre | a display / the rerun window itself |
| "What good looks like" (section 4) | NVIDIA's release KPI (KITTI stereo odometry 0.85 % ATE, 294 fps on x86), their troubleshooting guidance, and the first measurements of section 4.2 (still camera: <= 1 cm over 20 s in daylight, 0.3-1.3 m in the dark) | **the walked-loop drift ratios of section 4.2 are still expectations**; no moving recording exists yet |
| Jetson checklist (section 7) | flags checked against `--help`; timings from [cuvslam-setup.md](cuvslam-setup.md) 3.2 | never executed; whether the JetPack image pre-installs `nvidia-jetpack`; USB power on a Type-A port under load |
| Bench items from the calibration doc | **2026-09-29**: the frame of `*_RAW` (sensor-native, from the still stretches of the walks and the six-pose recording); accelerometer bias and scale (six-pose fit, residual 0.001 m/s^2) | mono `enableUndistortion`, IMU noise densities (placeholders), the CAD-only IMU translation |
| `tests/test_cuvslam_oak.py` | Windows: `python -m pytest tests -q -p no:cacheprovider` -> `61 passed in 5.97s`, exit 0, against `cuvslam_oak.fake_cuvslam`. Per its docstring: rig math on the vendored EEPROM dump `tests/data/calibration_OAKDS2EXAMPLEID000.json` (baseline, 640x400 half-scale, `rig_from_imu` vs the calibration doc's worked matrix, 8 Polynomial coefficients); `EurocWriter` / `EurocReader` round trip checked file by file against the format doc plus its negative cases; tracking in both modes with the fake (IMU hold-back per source type, warm-up, `max_frames`, out-of-order frames, TUM write/read); the fake's input validation; `07`/`08` CLI smoke tests (`--help`, `--device-id nonexistent` -> 2, `--dataset nonexistent` -> 2, `--fake-cuvslam` on a tiny dataset -> 0, no cuvslam -> 3). `tests/conftest.py` replaces `dai.Device`, `dai.DeviceBase` and `dai.Pipeline`, so the camera is never opened | never run against the real wheel |

Housekeeping notes: the recorder's default folder is `recordings/euroc_<stamp>` (the format doc's
examples say `cuvslam_<stamp>`; the script is right), `08` accepts `--mode vio` rather than the
`--mode inertial` quoted in the older setup doc (both spellings work, `inertial` is an alias), and
`08`'s live path needs `depthai` while its dataset path does not.

---

## 10. How this guide was checked (2026-09-20, Windows, camera never booted)

- `python datasets/07_record_dataset.py --help` and `python methods/cuvslam-vio/08_cuvslam_track.py --help`: every
  `--flag` used in sections 0, 3, 6 and 7 appears there; `methods/cuvslam-vio/tools/check_env.py --json/--warm-up` likewise.
- `python methods/cuvslam-vio/08_cuvslam_track.py <synthetic dataset> --fake-cuvslam --mode vio --summary --out ...`:
  exit 0, TUM + JSON written; `evo_traj tum` on that file; `evo_traj`/`evo_ape`/`evo_rpe tum --help`
  for the flags in section 4.3 (evo 1.37.1 installed into a scratch folder, not the repo venv).
- `08 --live --device-id nonexistent` (exit 3 without the fake, 2 with it) and `07 --device-id
  nonexistent --duration 1` (exit 2, no folder left behind): enumeration only, the camera stayed
  `X_LINK_UNBOOTED`.
- `python -m pytest tests -q -p no:cacheprovider`: `61 passed`, exit 0; the device enumerated as
  `X_LINK_UNBOOTED` afterwards.
- Every relative link in this file resolves to an existing file.
- The two camera recordings cited in the `oak_source.py` and `07` rows of section 9 come from a
  separate hardware session on 2026-09-20 (two runs under `timeout 90`, about 42 s of device time;
  the full log is the README's "Verification status"). The checks in this list never booted the
  camera.
