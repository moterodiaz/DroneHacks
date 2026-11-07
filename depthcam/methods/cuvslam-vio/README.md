# cuVSLAM VIO: stereo visual-inertial odometry on the OAK-D S2

NVIDIA cuVSLAM (PyCuVSLAM 17.0.0) fed by the OAK-D S2's synchronised stereo pair and BNO086 IMU: the
tracking loop with its starvation guard and coasting (`cuvslam_oak/`), the tracker script `08`, the
live view `09` with its WSL2 bridge server `10`, the install scripts in [tools/](../../tools/) and the
reference docs in [docs/](../../docs/). **Status: working and measured** (section "Verification status"
below); the best configuration is stereo + IMU at 1280x800 with the guard at 3 m/s, 2.3 % return-to-
start drift on the one full-resolution walk so far.

Quick start (Windows laptop with WSL2 set up once per [docs/cuvslam-setup.md](docs/cuvslam-setup.md)):

```
.venv\Scripts\python methods\cuvslam-vio\09_cuvslam_live_view.py                 # live view, q stops
.venv\Scripts\python datasets\07_record_dataset.py --seconds 60 --preview      # record a walk
# in WSL2: track it
source ~/venvs/cuvslam/bin/activate
cp -r "/mnt/c/<repo>/recordings/euroc_<stamp>" ~/datasets/
python "/mnt/c/<repo>/methods/cuvslam-vio/08_cuvslam_track.py" ~/datasets/euroc_<stamp> --mode vio --summary
```

## cuVSLAM hosts (WSL2 and Jetson)

There is no Windows `cuvslam` wheel, so tracking needs a Linux host with CUDA 12: `bash methods/cuvslam-vio/tools/setup_wsl2.sh` inside WSL2 Ubuntu-22.04 on this laptop (x86_64 wheel, `cuda-toolkit-12-6`, a venv at `~/venvs/cuvslam`, `evo`), or `bash methods/cuvslam-vio/tools/setup_jetson.sh` on the Orin Nano (aarch64 wheel, CUDA 12.6 from `nvidia-jetpack`, `depthai`, `evo`, `jtop`). Both are idempotent, verify the wheel's sha256 and end with an import + `warm_up_gpu()` smoke test; `python methods/cuvslam-vio/tools/check_env.py` reports what any host has (exit 0 always). Step by step, with the facts behind every pin: [methods/cuvslam-vio/docs/cuvslam-setup.md](docs/cuvslam-setup.md). `setup_wsl2.sh` ran on this laptop on 2026-09-20 (WSL2 Ubuntu 22.04.5, CUDA 12.6.3, `cuvslam` 17.0.0+cu12 imported, `warm_up_gpu()` ok, first trajectories tracked); `setup_jetson.sh` has not run yet (no Jetson on hand) and is `bash -n` checked only.


Added 2026-09-19/20: a bridge from this camera to NVIDIA's **cuVSLAM** (PyCuVSLAM **v17.0.0**, package `cuvslam` 17.0.0+cu12; Linux x86_64/aarch64, Python 3.10, CUDA 12 only) so the stereo pair, and optionally the BNO086, produce a camera trajectory. Two modes: `stereo` = `OdometryMode.Multicamera` (cameras only, the configuration of NVIDIA's own `examples/oak-d/run_stereo.py`) and `vio` = `OdometryMode.Inertial` (cameras + IMU). Conventions everywhere: rig frame = left camera (CAM_B) optical frame, x right, y down, z forward; metres; quaternions `x y z w`; integer nanoseconds on one device-monotonic clock, mid-exposure for images; no axis flips. The user guide is [methods/cuvslam-vio/docs/cuvslam.md](docs/cuvslam.md); this section is the summary.

## What was added

| Path | What it is |
|---|---|
| `cuvslam_oak/__init__.py` | `import_cuvslam()`: the real wheel, or the test double when `--fake-cuvslam` / `CUVSLAM_FAKE=1` asks for it; the fake is never picked silently |
| `cuvslam_oak/rig.py` | EEPROM calibration (`calibration.json` or the live device) -> `cuvslam.Rig` (cm -> m, intrinsics scaled to the frame size, `Distortion.Model.Polynomial` with 8 coefficients, `rig_from_imu` from `getImuToCameraExtrinsics`), EuRoC `sensor.yaml` helpers, host undistortion maps; `python -m cuvslam_oak.rig <calibration.json> --verify` cross-checks against `dai.CalibrationHandler` |
| `cuvslam_oak/oak_source.py` | the live DepthAI v3 graph: `Camera` CAM_B/CAM_C GRAY8 -> `Sync`, `IMU` node with `ACCELEROMETER_RAW` + `GYROSCOPE_RAW`; device-monotonic mid-exposure stamps, L/R skew rule (1 ms), first-frame intrinsics guard, drop counters |
| `cuvslam_oak/dataset.py` | EuRoC ASL writer (background PNG thread with a bounded queue, `dataset.json`, `README.txt`) and reader/validator, `trim_dataset()` repair |
| `cuvslam_oak/tracking.py` | the loop (`register_imu_measurement(0, m)` for every IMU sample with `t <= image t`, then `track(t, [left, right])`), counters (failed frames, resets, frame and IMU gaps), TUM writer/reader |
| `cuvslam_oak/fake_cuvslam.py` | test double of PyCuVSLAM 17.0.0 (same classes, same input checks, synthetic constant-velocity poses) so the whole pipeline runs on Windows |
| `datasets/07_record_dataset.py` | record a dataset from the camera into `recordings/euroc_<YYYYmmdd_HHMMSS>/` |
| `methods/cuvslam-vio/08_cuvslam_track.py` | track a dataset or the live camera; `trajectory_<mode>.tum` (+ `--summary` JSON); `--slam` adds cuVSLAM's loop-closure layer (`trajectory_<mode>_slam.tum`) |
| `methods/cuvslam-vio/09_cuvslam_live_view.py`, `methods/cuvslam-vio/10_cuvslam_bridge_server.py` | live view on the laptop: camera or recording on Windows, engine in WSL2 (`cuvslam_oak/bridge.py`, `cuvslam_oak/live_view.py`) |
| `tests/` | `test_cuvslam_oak.py` (61 tests: rig math, dataset round trip against the format doc, tracking with the fake, the fake's validation, `07`/`08` CLI smoke tests), `conftest.py` (any in-process camera open fails the test), `data/calibration_OAKDS2EXAMPLEID000.json` (this unit's EEPROM dump, since `recordings/` is git-ignored) |
| `tools/` (this folder) | `check_env.py` (host report: OS/WSL/JetPack, driver, CUDA, `cuvslam`, `depthai`, package versions; exit 0 always), `setup_wsl2.sh`, `setup_jetson.sh`, `bench_png_writer.py`, `fit_imu_calibration.py` (six-pose accelerometer / gyroscope fit and `--check`), `run_bridge_server.sh` (starts the bridge server in WSL2 / on the Jetson) |
| `cuvslam_oak/imu_calib.py`, `datasets/calibration/` | the IMU correction (per-axis accelerometer bias and scale, gyroscope bias), its fit, and the stored file of this unit |
| `requirements-dev.txt` | `pytest` |
| `docs/` (this folder) | the references listed at the end of this file |

## The two-machine workflow

1. **Windows, camera on USB 3:** `python datasets/07_record_dataset.py --duration 45 --note "..."` writes `recordings/euroc_<stamp>/` in EuRoC ASL layout (8-bit undistorted PNGs named `<ts_ns>.png`, `data.csv` per sensor, `sensor.yaml` with zero distortion, body = cam0 = CAM_B).
2. **Windows, dry run:** `python methods/cuvslam-vio/08_cuvslam_track.py recordings/euroc_<stamp> --fake-cuvslam --max-frames 100` proves the folder loads (validation, rig from its `sensor.yaml`, TUM written); the poses are synthetic and the script says so on stderr, in the TUM header and in the summary.
3. **WSL2 Ubuntu-22.04, after `bash methods/cuvslam-vio/tools/setup_wsl2.sh` once:** copy the folder to `~/datasets/` (PNG reads through `/mnt/c` are slow), then `08 ... --mode stereo --summary` and `08 ... --mode vio --summary`.
4. **WSL2 or Jetson, evaluate:** the `Summary:` block (tracked %, resets, path length, end-point offset = return-to-start drift when the loop ends on the start mark), `evo_traj tum ... --plot --plot_mode xz`, and NVIDIA's unmodified `examples/euroc/track_euroc.py` on the same folder as the independent check.
5. **Jetson Orin Nano, after `bash methods/cuvslam-vio/tools/setup_jetson.sh` once:** `python methods/cuvslam-vio/08_cuvslam_track.py --live --mode stereo --duration 60 --summary`, then `--mode vio`; `07` records there too.

## Exact commands

```
# Windows laptop, repo root, venv active (.venv\Scripts\activate)
python methods/cuvslam-vio/tools/check_env.py                                                    # "cuvslam: NOT importable" is normal here
python datasets/07_record_dataset.py --duration 45 --note "slow loop, back to the tape mark"
python datasets/07_record_dataset.py --width 640 --height 400 --duration 120  # light option: no PNG-queue drops on this laptop
python methods/cuvslam-vio/08_cuvslam_track.py recordings/euroc_<stamp> --fake-cuvslam --max-frames 100   # plumbing dry run only
python -m pytest tests -q

# WSL2 (Ubuntu-22.04), after methods/cuvslam-vio/tools/setup_wsl2.sh once; venv: source ~/venvs/cuvslam/bin/activate
mkdir -p ~/datasets && cp -r ~/DroneDepthCamTesting/recordings/euroc_<stamp> ~/datasets/
cd ~/DroneDepthCamTesting
python methods/cuvslam-vio/08_cuvslam_track.py ~/datasets/euroc_<stamp> --mode stereo --summary
python methods/cuvslam-vio/08_cuvslam_track.py ~/datasets/euroc_<stamp> --mode vio --summary
evo_traj tum ~/datasets/euroc_<stamp>/trajectory_stereo.tum ~/datasets/euroc_<stamp>/trajectory_vio.tum \
    --ref ~/datasets/euroc_<stamp>/trajectory_stereo.tum --align --plot --plot_mode xz

# Jetson Orin Nano, after methods/cuvslam-vio/tools/setup_jetson.sh once; venv: source ~/DroneDepthCamTesting/.venv/bin/activate
timeout 90 python camera/00_device_info.py                                  # usb=SUPER
python methods/cuvslam-vio/08_cuvslam_track.py --live --mode stereo --duration 60 --summary
python methods/cuvslam-vio/08_cuvslam_track.py --live --mode vio --duration 60 --summary
```

`~/DroneDepthCamTesting` is the symlink to this checkout that [methods/cuvslam-vio/docs/cuvslam-setup.md](docs/cuvslam-setup.md) section 2.5 creates (the Windows path has spaces: quote it). `08` accepts the recording folder or its `mav0/`; `--mode` defaults to `vio` when the recording has IMU rows and real IMU extrinsics, else `stereo`. Outputs go next to the dataset (`trajectory_<mode>.tum`, `trajectory_<mode>_summary.json`) or to `--out`. Every other flag: `--help`, or [methods/cuvslam-vio/docs/cuvslam.md](docs/cuvslam.md) section 3.

## Live view on the laptop

```
.venv\Scripts\python methods\cuvslam-vio\09_cuvslam_live_view.py                       # camera, until q
.venv\Scripts\python methods\cuvslam-vio\09_cuvslam_live_view.py --duration 60         # or stop by itself
.venv\Scripts\python methods\cuvslam-vio\09_cuvslam_live_view.py --replay recordings\euroc_20260929_181933    # a recording, no camera
```

One window: the left camera image with the features cuVSLAM tracks, a top-down map of the trajectory (levelled with gravity), the height, and the tracker's state (`TRACKING` green, `NO POSE (guard / loss)` red, with a red dot on the map wherever frames were skipped). Keys: `q` quit, `c` clear the trail, `f` features on/off. It runs the best configuration measured so far (stereo + IMU, 1280x800, guard at 3 m/s); `--mode stereo`, `--max-speed`, `--imu-calibration auto` change it.

How it works: cuVSLAM cannot run on Windows and the camera cannot be in WSL2, so `09` captures and undistorts exactly as the recorder does and streams the frames and IMU samples over TCP (`localhost`) to `methods/cuvslam-vio/10_cuvslam_bridge_server.py`, which it starts inside WSL2 by itself (`methods/cuvslam-vio/tools/run_bridge_server.sh`) and which runs the same tracking loop, rig and guard as `08`; the poses come back over the same connection (`cuvslam_oak/bridge.py`, drawing in `cuvslam_oak/live_view.py`). Measured 2026-09-29: the path carries 148 raw 1280x800 pairs per second; on the camera (15 s, still on the desk) 451 of 451 frames were tracked at 30 fps with none dropped and 25-34 ms from capture to pose (98 ms once, while the engine initialises); replaying the 57 s walk at recorded pace gave 8.15 m / 0.29 m = 3.6 %, inside the 2.3-3.6 % of the offline runs. Not yet tried: walking with the camera while the view runs. On the Jetson the same view can run in one process (`--local-server`), untested.

## Where it runs

| Host | Does | Does not |
|---|---|---|
| Windows laptop (this repo, `.venv`, camera on USB 3) | record datasets (`07`), the pytest suite and `08 --fake-cuvslam` dry runs against `cuvslam_oak/fake_cuvslam.py` | track: no `cuvslam` wheel exists for Windows (`08` exits 3 without the fake) |
| WSL2 Ubuntu 22.04 (same laptop, RTX 1000 Ada, Windows driver 581.95) | track recorded datasets (`08 <dataset>`), `evo`, NVIDIA's `track_euroc.py` | the camera (usbipd into WSL2 is a USB/IP tunnel: functional tests only, never timing) |
| Jetson Orin Nano (JetPack 6.2.x, CUDA 12.6 on board) | everything: `07`, `08 <dataset>`, `08 --live` | |

## Verification status (2026-09-20)

Honest list; the long form with per-component evidence is [methods/cuvslam-vio/docs/cuvslam.md](docs/cuvslam.md) section 9.

**Tests pass on Windows with the fake cuVSLAM.** `python -m pytest tests -q -p no:cacheprovider` -> `61 passed in 6.37s` (pytest 9.1.1, Python 3.13.3), exit 0; the camera enumerated `X_LINK_UNBOOTED` afterwards. The fake itself was run against NVIDIA's own v17.0.0 `python/test` suites (70 collected: with an identity pose source only the documented top-level `Odometry` name assertion fails); `rig.py` reproduces every worked number of the calibration doc from this unit's EEPROM dump and agrees with `dai.CalibrationHandler` to 6e-8 (19 checks, no device); `dataset.py` and `tracking.py` are covered by synthetic round trips and 100+ stub-based assertions.

**The recorder was exercised on the camera on 2026-09-20** (two short static runs, camera on its back on the desk, both under `timeout 90`):

- Pre-check: `python -c 'import depthai as dai; print(len(dai.Device.getAllAvailableDevices()))'` -> `1` (`OAKDS2EXAMPLEID000`, `X_LINK_UNBOOTED`). Device time: run 1 03:42:08-03:42:32 (24 s wall including boot), run 2 03:44:16-03:44:34 (18 s); about 42 s of the 180 s budget. No python/depthai process was left behind and the device enumerated `X_LINK_UNBOOTED` afterwards. No repo file was edited by the runs; the recordings are kept (git-ignored): `recordings/euroc_20260920_034208` (348 MB) and `recordings/euroc_20260920_034416` (69 MB).
- **Run 1** (`--seconds 15 --headless --note 'hw check'`, default 1280x800 @ 30 fps, IMU 200 Hz): exit 0, stderr empty. `Warm-up done: 44 pairs discarded, epoch offset 1789890130508685400 ns`; `Recording complete (--duration reached): 363 stereo pairs (363 accepted, 363 written, 0 failed), 3005 IMU samples, 15.007 s of data in 15.7 s`; `Measured: stereo 24.295 fps, imu 200.173 Hz, max stereo gap 100.001 ms, max imu gap 8.941 ms, max L/R skew 18.0 us, accel/gyro skew 4.202 ms, PNG 49.752 ms/pair mean, 362.1 MB`; `Drops: 86 (stereo_write_queue_dropped 86)` (warm-up discarded 44 pairs and 275 IMU packets; 2 leading pairs skipped so the IMU stream starts first). The source delivered 451 frames with 0 gaps; 449 reached the writer = 363 written + 86 dropped.
- **Run 2** (`--seconds 10 --headless --width 640 --height 400`): exit 0. `Recording complete (--duration reached): 300 stereo pairs (300 accepted, 300 written, 0 failed), 2005 IMU samples, 10.008 s of data in 11.6 s`; `Measured: stereo 30.0 fps, imu 200.249 Hz, max stereo gap 33.334 ms, max imu gap 8.925 ms, max L/R skew 19.0 us, accel/gyro skew 4.212 ms, PNG 14.035 ms/pair mean, 70.3 MB`; `Drops: 0`. stderr: two `OakSourceWarning` lines (640x400 is the 2x2-binned sensor mode; the delivered intrinsics match `getCameraIntrinsics(640x400)`). Frame spacing histogram `{33 ms: 299}`, 30.000 fps written.
- Counts vs expectation: run 1 cam0 363 rows / 363 PNGs, cam1 363 / 363 (expected ~450: 86 dropped by the PNG queue, issue 1 below), imu0 3005 rows (expected ~3000, OK); run 2 300 / 300 / 2005. `dataset.json` `counts.stereo_pairs`, `imu_samples`, `first_timestamp_ns`, `last_timestamp_ns` equal the csv values in both runs (run 1: 363, 3005, 1789890133837903400, 1789890148844885400).
- Timestamps: cam0 and cam1 stamps identical row by row in both runs (max |diff| = 0 ns); cam0 strictly increasing (min diff 33332000 ns); 19-digit unsigned; filename == `<ts>.png`; lexicographic order == numeric order; microsecond resolution (device stamp + epoch offset). IMU strictly increasing (min diff 2.856 ms, no duplicates), mean spacing 4.9957 ms (run 1) / 4.9938 ms (run 2) = 200.17 / 200.25 Hz, but bimodal (std 2.12 ms; clusters at 2.9-4.8 ms and 7.6-8.9 ms; 793 of 3004 and 529 of 2004 intervals over 7.5 ms; none over 12.5 ms), issue 2 below. The IMU starts 28.3 ms (run 1) / 17.9 ms (run 2) before the first frame and ends 78.7 / 23.0 ms after the last; 6-20 IMU rows per frame interval (min 6), no empty interval. Epoch offsets 1789890130508685400 / 1789890258838125000 put the first frame at device clock 3.358 s / 3.567 s and 18.3 s / 11.7 s before `dataset.json` was written, consistent with `record.start_wall_time` 2026-09-20T03:42:12-04:00 / 03:44:20-04:00.
- IMU values (camera static on its back): run 1 accel mean `[-0.0112, -0.2879, -9.9119]` m/s^2, |a| 9.916 +/- 0.021; gyro mean `[-0.00089, -0.00056, -0.00103]` rad/s, |gyro| max 0.0128; run 2 accel mean `[-0.0064, -0.3006, -9.9110]`. Gravity on IMU -z, as in the cheat-sheet's earlier at-rest sample; the RAW-frame question of [methods/cuvslam-vio/docs/oak-to-cuvslam-calibration.md](docs/oak-to-cuvslam-calibration.md) section 4.4 item 1 is still open (the pose was not upright). csv sample: `1789890133837903400,0.002130528912,0,0.001065264456,-0.0287304204,-0.2873041928,-9.911994934` (gyro first; ASCII, LF, no BOM, trailing newline; headers verbatim EuRoC).
- A `--fake-cuvslam` dry run of `08` on run 1 left `recordings/euroc_20260920_034208/trajectory_vio.tum` (100 rows, `# FAKE cuvslam ... NOT tracking results` header): the real recording passes the reader's validation and the loop, rig and TUM writer accept it.

**First runs on CUDA hardware (WSL2 on this laptop, 2026-09-20 afternoon).** `wsl --install -d Ubuntu-22.04` + `methods/cuvslam-vio/tools/setup_wsl2.sh` (apt steps as root, the rest with `SKIP_APT=1`) gave Ubuntu 22.04.5, Python 3.10.12, `cuda-toolkit-12-6` 12.6.3-1 (`nvcc` 12.6.85, 5.0 GB under `/usr/local/cuda-12.6`), `cuvslam` 17.0.0+cu12 (`get_version()` -> `('17.0.0+57f42cc', 17, 0, 0)`), `warm_up_gpu()` ok on the RTX 1000 Ada through the Windows driver 581.95; `methods/cuvslam-vio/tools/check_env.py` reports `all expected pieces present`. Datasets were copied to `~/datasets/` first (a copy made while the C: drive was full came out truncated: 219 zero-byte files; always check `find <copy> -type f -size 0`).

| Recording | `08 --mode stereo` | `08 --mode vio` | NVIDIA `track_euroc.py`, unmodified |
|---|---|---|---|
| `euroc_20260920_034416`: 640x400, 10 s, 300 pairs, camera still on its back facing the ceiling, **dark room** (mean 8-16/255) | 300/300 tracked, 0 resets, path 3.205 m, **end-point offset 0.347 m**, 101 fps (engine 214 fps, 4.7 ms/frame) | 300/300 tracked, path 4.554 m, **offset 1.335 m**, gravity (+0.03, -0.19, -9.81) m/s^2, 68 fps | Multicamera: 300 frames, offset 0.313 m, path 2.910 m. Inertial: 300 frames, offset 1.3351 m, path 4.5535 m (**identical to `08`**) |
| `euroc_20260920_145714`: 640x400, 20 s, 600 pairs + 4007 IMU rows, 0 drops, camera still on the desk, **daylight** (mean 123/255) | 600/600 tracked, 0 resets, path 0.246 m, **end-point offset 0.005 m**, max from start 0.007 m, 167 fps (engine 1150 fps, 0.9 ms/frame) | 600/600 tracked, path 0.373 m, **offset 0.007 m**, max 0.009 m, gravity (-0.08, -0.13, -9.81) m/s^2, 121 fps (2.9 ms/frame) | Multicamera: 600 frames, offset 0.0047 m, path 0.2464 m. Inertial: 600 frames, offset 0.0067 m, path 0.3730 m (**identical to `08`** in both modes) |

Reading: with light and texture a stationary camera stays within 1 cm over 20 s in both modes; in the dark the same still camera "moves" 0.3-1.3 m, so lighting dominates everything else. `evo_traj tum` reads the files (`600 poses, 0.246m path length, 19.967s duration`). The trajectories and summary JSONs sit next to each recording (git-ignored). The `vio` run counted 528 / 1061 IMU "gaps" over the old 6 ms threshold (the BNO086's bimodal 3-9 ms spacing, finding 2) and tracked 100 % regardless, so the counter's default is now 10 ms, the Isaac ROS node's `imu_jitter_threshold_ms`.

**Walking recordings and the starvation guard (2026-09-21).** Four 60 s hand-held walks through a dorm room at night (`recordings/euroc_20260921_00*`, 640x400, 0 write drops, 3-6 pairs each rejected by the 1 ms stereo-sync check) exposed the engine's blank-wall failure: whenever the camera looked at a bare wall or door edge from close range (4-14 % of frames, stretches up to 3.8 s, image sharpness 80-180 against 300-1300 elsewhere) `Tracker.track()` still returned a pose, but a garbage one: single-frame steps of 0.4-52 m and rotations up to 63 deg, and in `vio` mode the engine then ran away for hundreds of metres. It never reported a failure (`None`), reported its 450-feature cap on those frames (sensor noise detected as features), and NVIDIA's unmodified example reproduced our numbers exactly, so this is the engine's behaviour, not a bug of ours. The calibration and pairing were re-verified on the walking images (hundreds of consistent matches per pair, positive disparity, 12 px vertical offset = the principal-point difference, sub-pixel epipolar error).

`cuvslam_oak.tracking` now has a **starvation guard** (on by default; `08 --no-guard` to compare): a returned pose is skipped when the step since the last accepted pose exceeds `--max-speed` (5 m/s then, 3 m/s since 2026-09-29) or `--max-rot-rate` (720 deg/s), or when the engine's position covariance is exactly zero (a degenerate marker seen on about half of the glitch frames and on 0.1-0.6 % of good ones; never on the first pose of a segment); the trajectory is re-anchored across the skipped stretch, and after `--guard-reset-after` (30) consecutive skipped frames the tracker is re-created and chained onto the last good pose (measured: without the reset the engine never recovers, 281-564 frames lost). A minimum-observations rule exists (`--min-observations`, default 0 = off) but is useless on this camera. Measured on the three walks that returned to their floor mark (end-point offset / path length):

| Recording (60 s, 640x400) | stereo, guard off | stereo, guard on | vio, guard off | vio, guard on |
|---|---|---|---|---|
| `euroc_20260921_001403` | 75.7 m / 45.4 m = 60 % | 10.7 m / 0.31 m = **2.9 %** | 12.1 m / 1.43 m = 12 % | 12.1 m / 1.43 m = 12 % |
| `euroc_20260921_001246` | 57.1 m / 10.2 m = 18 % | 12.6 m / 1.26 m = 10 % | 118.8 m / 22.8 m = 19 % | 13.0 m / 0.31 m = **2.4 %** |
| `euroc_20260921_001519` | 169.2 m / 83.3 m = 49 % | 17.7 m / 3.74 m = 21 % | 20.8 m / 1.28 m = 6 % | 19.1 m / 2.16 m = 11 % |

The still daylight recording is unchanged by the guard (0 frames skipped). A speed limit of 1-3 m/s instead of 5 changes these figures by a few percent either way (sweep in the scratch logs); the remaining 3-20 % drift is what the blank-wall stretches cost even when no jump survives, so the next step is a walk that keeps furniture in view and, on the Jetson, full-resolution frames. The trajectories and summaries sit next to each recording (git-ignored).

**Parallel PNG writer (2026-09-21).** `EurocWriter` now encodes PNGs on a thread pool (`--png-workers`, default 0 = auto = 4 on this laptop) behind one ordered committer thread, so the csv order and the "rows only after both PNGs exist" rule are unchanged. `datasets/bench_png_writer.py` measured 1 thread = 23 pairs/s (cannot hold 30 fps), 4 threads = 83 pairs/s capacity; a real 15 s 1280x800 recording on the camera (`recordings/euroc_20260921_011329`) wrote 446 of 446 pairs at 30.0 fps with 0 write drops (4.7 ms per pair on the commit path, 45 ms encode per pair spread over 4 threads), which closes finding 1 below. Total camera time for that check: about 25 s.

**Full-resolution walk, guard speed limit, IMU fit, repeatability (2026-09-29).** One 57 s hand-held loop in a common room (`recordings/euroc_20260929_181933`, 1280x800, 1698 pairs, 0 write drops, 6 stereo-sync rejections; couch, fridge and shelves in view but still 11 % of the frames on bare wall, longest stretch 2.9 s), tracked in WSL2, every configuration at least twice (end-point offset / path length; the camera was carried level, so the end height should be 0):

| Configuration | Result | End height |
|---|---|---|
| `stereo`, guard off, or guard at the old 5 m/s limit (0 frames skipped) | 10.7 m / 1.34 m = 12.5 % | -1.27 m |
| `stereo`, guard at 3 m/s (new default) | 9.53 m / 0.26 m = **2.7 %** in 8 of 10 runs, 9.36 m / 1.30 m = 13.9 % in 2 | 0.00 m (-0.86 m) |
| `stereo`, `--max-speed 2` / `1.5` | 5.4 % / 3.1 % | +0.38 / +0.16 m |
| `vio`, guard off | 11.2 m / 1.42 m once, a 7.7 km run-away in 3 of 4 runs | |
| `vio`, guard at 3 m/s (new default) | 8.11 m / 0.18 m = **2.3 %** in 8 of 10 runs, 8.36 m / 0.30 m = 3.6 % in 2 | +0.07 m |
| `vio`, `--max-speed 2` | 7.90 m / 0.13 m = 1.7 % | -0.06 m |
| `vio`, `--imu-calibration auto` | 7.79 m / 0.31 m = 4.0 % | 0.00 m |
| `--denoising` (either mode) | trajectories identical to the runs without it | |
| same walk downscaled offline to 640x400, `stereo` / `vio` | 9.00 m / 1.33 m = 14.8 % / 8.92 m / 1.06 m = 11.9 % (8 runs each, identical) | -0.12 / +0.69 m |

What this showed:

- **The guard's speed limit was too loose.** At 45 s (a dark door edge filling the frame) the stereo estimate sank 1.8 m in 0.8 s at 2.0-3.6 m/s, under the 5 m/s limit, so nothing was skipped. The default is now **3 m/s** (`DEFAULT_MAX_SPEED_MPS`); on the three 2026-09-21 walks 3 m/s and 5 m/s differ by a few percent either way. It is a platform setting: a vehicle that really moves faster than 3 m/s needs `--max-speed` raised, or the guard will skip real motion.
- **Full resolution helps**: 2.3-2.7 % at 1280x800 against 12-15 % for the same walk at 640x400.
- **`vio` with the guard is the most consistent configuration**; stereo-only errors show up mostly as height and tilt drift, which gravity from the IMU holds.
- **The engine is not strictly repeatable.** The same command on the same files usually gives a bit-identical trajectory, but about one run in five at 1280x800 lands on a different one (end points up to 1.2 m apart), also on an otherwise idle machine; at 640x400 every repeat was identical except one 2 mm difference. Compare configurations on repeated runs, never on one.
- **Accelerometer bias** (`cuvslam_oak/imu_calib.py`, `datasets/fit_imu_calibration.py`, `datasets/datasets/calibration/imu_OAKDS2EXAMPLEID000.json`): a six-pose recording (`recordings/euroc_20260929_182100`) gave bias (+0.054, -0.692, -0.124) m/s^2 and scale (1.0065, 1.0060, 1.0022); a still camera reads 9.05 to 10.44 m/s^2 raw depending on which way is up and 9.807 +/- 0.001 corrected. Independent check: the still recordings of 2026-09-20/21 correct from 9.90-9.93 to 9.80-9.82, the hand-held upright stretches of the walks from 9.07-9.12 to 9.82-9.88. Applying it (`08 --imu-calibration auto`) changed the drift of four walks by less than the run-to-run spread (11.8 -> 16.5 %, 2.5 -> 2.3 %, 10.6 -> 10.1 %, 2.3-3.6 -> 2.7-5.9 %), so it is **off by default**.
- **The RAW IMU frame question is closed** (finding 3 below): upright and still, `ACCELEROMETER_RAW` reads about (0, +8.9, +1.9) m/s^2, i.e. sensor-native axes with y up, and the tracker's gravity comes out as (-0.67, +9.71, +1.22) in the rig frame (y down), as `rig_from_imu = diag(1,-1,-1)` predicts.

**Coasting through dropouts (2026-09-30).** Until now a guard skip or a loss froze the trajectory and the returning pose was chained onto the last measured one with zero motion, so every turn made during a blank-wall stretch was lost. `cuvslam_oak.tracking` now *coasts* (on by default, `08 --no-coast` / `09 --no-coast` to compare): while the engine gives no usable pose, each frame still gets a row, its rotation integrated from the gyroscope (rates mapped into the rig with `rig_from_imu`; the IMU is read in `stereo` mode too, without being registered with the engine) and its position continued at the median frame-to-frame velocity of the last second, decaying with a 1 s time constant, up to 3 s without vision; the returning pose is re-anchored onto the coasted estimate for its own time, also after a loss. Coasted rows are flagged (`PoseEvent.coasted`, `frames_coasted`, `coast_gaps` in the summary, orange in the live view, a header line in a saved TUM) so a downstream consumer (the PX4 bridge, later) can drop or down-weight them. Measured on the four walks, no coasting -> coasting, one run each (the engine's run-to-run spread applies):

| Walk, mode | End-point offset | Heading error at the end (the walks ended facing the start heading) |
|---|---|---|
| 2026-09-29 stereo | 0.26 m -> 0.17 / 1.62 m (the engine's two outcomes) | 19.5 -> 26.2 / 24.4 deg |
| 2026-09-29 vio | 0.18 -> 0.24 m | 22.7 -> **2.1** deg |
| 001403 stereo | 0.26 -> 0.28 m | 17.8 -> **3.6** deg |
| 001403 vio | 1.43 -> 1.50 m | 34.5 -> **17.2** deg |
| 001246 stereo | 1.21 -> 1.06 m | 27.7 -> **11.6** deg |
| 001246 vio | 0.70 -> 1.74 m | 59.6 -> **11.3** deg |
| 001519 stereo | 4.02 -> 6.23 m | 35.7 -> 34.3 deg |
| 001519 vio | 2.23 -> 5.33 m | 22.5 -> 29.5 deg |

Reading: the gyro rotation is right (checked against the vision increments of two walks: `rig_from_imu = diag(1,-1,-1)` reproduces them to 0.02 deg per frame) and it removes most of the heading error a dropout leaves behind; the position is a different story. The velocity just before a guard event is already corrupted (1.4-2.9 m/s for 0.3 s before the 2026-09-29 dive, at walking pace), which is why the first version (least-squares velocity over 0.3 s) made every walk worse and the shipped one uses a 1 s median; even so, position coasting is neutral at best and the two walks with four engine restarts each got worse. The end-point offset of a walk with several dropouts is a poor judge of any single mechanism; the heading error is the cleaner one. Keeping the engine's own rotation across a gap it survived (`--coast-keep-engine-rotation`) was tried and left a larger heading error, so the gyro wins by default. On the still recording nothing is coasted.

**cuVSLAM's SLAM layer (2026-09-30):** tried, measured, not useful here; the whole account is in [../cuvslam-vslam/README.md](../cuvslam-vslam/README.md).

**Still unverified:** `08 --live`, `--rerun` and `--show`; `methods/cuvslam-vio/tools/setup_jetson.sh` and the whole Jetson path (no Jetson on hand yet); accuracy against any ground truth (every figure above is return-to-start drift of a hand-held walk); more than one walk at 1280x800; the guard on a flying platform; the IMU noise densities (still NVIDIA's RealSense placeholders).

Findings from the hardware run, still open:

1. **PNG throughput at 1280x800 on this laptop** (issue 1): **fixed 2026-09-21** by the parallel writer (above). Before the fix the single PNG thread needed ~50 ms per pair, the 64-pair queue filled and 86 of 449 pairs were dropped in run 1 (24.3 fps written); run 1 is therefore not a benchmark recording. After the fix: 446 of 446 pairs at 30 fps, 0 drops (`euroc_20260921_011329`).
2. **BNO086 sample spacing is bimodal** (issue 2): at 200 Hz the mean is right (4.99 ms) and no gap exceeds 12.5 ms, but the intervals cluster at 2.9-4.8 ms and 7.6-8.9 ms, so `08`'s IMU gap counter reported hundreds of "gaps" per minute at its original 6 ms threshold. The real tracker's inertial mode tracked 100 % of frames through them, so the default is now 10 ms (`cuvslam_oak.tracking.DEFAULT_IMU_JITTER_THRESHOLD_NS`, the Isaac ROS node's `imu_jitter_threshold_ms`), which is quiet for this IMU. Still open: the walks also lose 3-6 pairs per minute to the 1 ms stereo-sync check (`drops.stereo_sync_violations`), each leaving a 62-72 ms gap; harmless for tracking so far, cause not investigated.
3. The RAW IMU frame: **answered 2026-09-29** (sensor-native, the EEPROM rotation `diag(1,-1,-1)` is right; see above). Still open: the IMU noise densities in `imu0/sensor.yaml` are NVIDIA's RealSense placeholders, not measured, and the raw accelerometer carries a -0.69 m/s^2 bias on y that is only corrected with `--imu-calibration`.

## cuVSLAM docs

- [methods/cuvslam-vio/docs/cuvslam.md](docs/cuvslam.md): the user guide (workflow, every command, evaluating a run with the summary and `evo`, NVIDIA's example as the independent check, troubleshooting, the Jetson day-one checklist, what is and is not verified).
- [methods/cuvslam-vio/docs/cuvslam-setup.md](docs/cuvslam-setup.md): installing PyCuVSLAM 17.0.0 on WSL2 and the Jetson (wheels, digests, CUDA 12.6, pins, usbipd, `check_env.py` expectations).
- [methods/cuvslam-vio/docs/cuvslam-api-v17.md](docs/cuvslam-api-v17.md): the exact v17.0.0 Python API read from the wheel and the source tag (`Tracker(rig, cfg)`, `track(ts_ns, images)`, `register_imu_measurement(0, m)`, the examples' call patterns).
- [methods/cuvslam-vio/docs/oak-to-cuvslam-calibration.md](docs/oak-to-cuvslam-calibration.md): EEPROM -> `Rig` with every number worked for this unit (rig frame, cm -> m, Polynomial 8 coefficients, IMU chain, mid-exposure timestamps, no axis flips).
- [datasets/dataset-format.md](../../datasets/dataset-format.md): the EuRoC ASL layout the recorder writes and NVIDIA's loader reads (`T_BS` rules, `data.csv` columns, `dataset.json`, TUM output).
