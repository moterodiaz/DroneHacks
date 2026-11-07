# cuVSLAM dataset format: EuRoC ASL layout for OAK-D S2 recordings

What `datasets/07_record_dataset.py` must write and what `cuvslam_oak/dataset.py` must read so that
**one recording loads in two places without modification**: NVIDIA's own
`examples/euroc/track_euroc.py` + `dataset_utils.py` (PyCuVSLAM **v17.0.0**) and our
`methods/cuvslam-vio/08_cuvslam_track.py`. Trajectories go out in TUM format for `evo`.

Written 2026-09-19 from the v17.0.0 tag (every file listed in section 13 was read at
`https://raw.githubusercontent.com/NVlabs/PyCuVSLAM/v17.0.0/...`; `main` has an unreleased breaking
change to `Tracker`, so nothing here comes from `main`). The camera was **not** opened. All calibration
numbers are from this unit's saved EEPROM dump `recordings/video_20260918_030056/calibration.json`
(device `OAKDS2EXAMPLEID000`, OAK-D-S2-AF), re-read through `dai.CalibrationHandler(path)` with
`depthai==3.10.0` and no device attached.

At a glance:

| Item | Value |
|---|---|
| Layout | EuRoC ASL: `mav0/cam0/{data/*.png,data.csv,sensor.yaml}`, `mav0/cam1/...`, `mav0/imu0/{data.csv,sensor.yaml}` |
| Loader entry point | `get_rig("<...>/mav0")`, `prepare_frame_metadata_euroc("<...>/mav0", mode)`: the path handed to NVIDIA's code is the **`mav0` directory**, not its parent |
| Images | 8-bit grayscale PNG (PIL mode `L`), named `<timestamp_ns>.png`, **undistorted** (section 2.3), same resolution on both cameras, cam0 = CAM_B (left), cam1 = CAM_C (right) |
| Timestamps | integer nanoseconds, one device clock for cameras and IMU, strictly increasing per stream, **identical** for the cam0 and cam1 rows of a pair, 19 digits (Unix-epoch magnitude) |
| IMU csv | `timestamp, gx, gy, gz, ax, ay, az` in **rad/s then m/s^2** (gyro first) |
| Camera model in the loader | `sensor.yaml` -> `Distortion.Model.Brown` with `[k1, k2, 0, p1, p2]` built from exactly **4** `distortion_coefficients`; the `distortion_model` key is **ignored** |
| Rig frame | cam0 (left camera optical frame, x right, y down, z forward); `T_BS` = body_from_sensor and we choose body = cam0, so `T_BS(cam0)` is the identity |
| Trajectory | TUM text: `timestamp_s tx ty tz qx qy qz qw`, quaternion **x y z w** = the order of `cuvslam.Pose.rotation` |
| Our additions | top-level `dataset.json`, `calibration.json` (EEPROM dump), `README.txt`; nothing extra inside `mav0/` |

---

## 1. Directory tree

One recording = one folder `recordings/cuvslam_<YYYYmmdd_HHMMSS>/` (created with
`common.timestamp_dir(base, "cuvslam")`):

```
cuvslam_20260919_203107/
├── dataset.json                 # ours (section 12.1): device, settings, counts, drops
├── calibration.json             # ours: CalibrationHandler.eepromToJson(), same as 05_record.py
├── README.txt                   # ours (section 12.2): human-readable summary + replay commands
└── mav0/                        # <- this is the path NVIDIA's loader takes
    ├── cam0/                    # left  = CAM_B (OV9282, global shutter)
    │   ├── data/
    │   │   ├── 1758326123456789000.png
    │   │   ├── 1758326123490122000.png
    │   │   └── ...
    │   ├── data.csv
    │   └── sensor.yaml
    ├── cam1/                    # right = CAM_C
    │   ├── data/
    │   │   ├── 1758326123456789000.png   # same names as cam0
    │   │   └── ...
    │   ├── data.csv
    │   └── sensor.yaml
    └── imu0/                    # BNO086
        ├── data.csv
        └── sensor.yaml
```

Rules the loader imposes (`dataset_utils.py`):

- `cam0/sensor.yaml`, `cam1/sensor.yaml` and `imu0/sensor.yaml` are **all mandatory**, even for a
  stereo-only dataset: `get_rig()` loads the IMU yaml unconditionally (lines 142-147) and builds
  `rig.imus = [imu]` in every mode. `imu0/data.csv` is only read in `OdometryMode.Inertial`
  (line 260), but write it always.
- **Never** create `cam0/sensor_recalibrated.yaml`. If that file exists the loader switches all three
  sensors to `sensor_recalibrated.yaml` and to the **Fisheye** model (lines 100-107, 131-135), which is
  wrong for our lenses. (NVIDIA's `examples/euroc/sensor_*.yaml` are those recalibrated fisheye files;
  they are not a template for us.)
- Only PNG files under `cam*/data/`; nothing else in `mav0/` is required, and extra directories there
  (e.g. `state_groundtruth_estimate0/`) are ignored.
- The loader never lists the `data/` directory: it reads `data.csv` and joins `cam0/data/<filename>`
  (lines 253-256). File names still follow EuRoC so other tools that glob work too.

---

## 2. Image files

### 2.1 Naming and pairing

- File name = `<timestamp_ns>.png` where `<timestamp_ns>` is the decimal integer nanosecond timestamp of
  the stereo pair, no padding, no sign; the same string appears in `data.csv`. Use 19-digit
  (Unix-epoch magnitude) timestamps (section 8) so lexicographic and numeric order agree.
- cam0 and cam1 must have the **same number of rows** and **identical timestamps row by row**:
  `prepare_frame_metadata_euroc` raises `ValueError("Number of frames differs...")` and
  `"Timestamp mismatch at frame i"` otherwise (lines 242-247). It pairs by row index, not by search.
- The OAK left/right frames are hardware-synchronised but their timestamps differ by 11-16 us and are
  never equal (`camera/docs/depthai-v3-cheatsheet.md`, section 3). The recorder therefore assigns **one**
  timestamp per pair: the **left (CAM_B) frame's device timestamp**, used for both rows and both file
  names. Pair by `getSequenceNum()` or a `dai.node.Sync` group (as the official OAK-D example does,
  threshold `0.5 / FPS`), then require `|t_left - t_right| <= 1 ms` (cuVSLAM's stated tolerance for
  "almost synchronized" rigs, `track()` docstring). Pairs outside that are dropped and counted in
  `dataset.json.drops.stereo_sync_violations`; a frame whose partner never arrives is dropped and
  counted in `drops.stereo_unpaired`.
- Write the two `data.csv` rows only after **both** PNGs are on disk; on shutdown (Ctrl-C, `--duration`)
  drain the writer queue, then make a final pass that removes any dangling PNG or row so the two csv
  files are equal length. This is what keeps a truncated recording loadable.

### 2.2 Pixel format

`load_frame()` (lines 27-54) opens the file with **PIL** and accepts three modes only:

| PIL mode | Produced by | What the loader does |
|---|---|---|
| `L` (8-bit gray) | `cv2.imwrite(path, uint8 HxW array)` | passes the `(H, W)` uint8 array straight to `track()` -> **this is what we write** |
| `RGB` | 3-channel PNG | converts RGB -> BGR, passes `(H, W, 3)` |
| `I;16` (16-bit gray) | `cv2.imwrite` of a uint16 array | passes uint16, meant for depth images: **never** write mono frames as 16-bit |
| anything else (`P`, `LA`, `RGBA`, `1`) | palette / alpha PNGs | `ValueError("Unsupported image mode")` |

So: `GRAY8` frames from `Camera.requestOutput(size, type=dai.ImgFrame.Type.GRAY8)` (the mono
default `RAW8` decodes to the same `(H, W)` uint8 `getCvFrame()`), written with `cv2.imwrite` and
`[cv2.IMWRITE_PNG_COMPRESSION, 1]` (fast; a 1280x800 frame is 0.3-1.0 MB, so 2 x 30 fps is up to
60 MB/s - use a writer thread with a bounded queue and count `drops.stereo_write_queue_dropped`).
Our reader uses `cv2.imread(path, cv2.IMREAD_UNCHANGED)` and asserts `dtype == uint8`, `ndim == 2`
and `shape == (height, width)` from `sensor.yaml`; `IMREAD_GRAYSCALE` would silently convert a
colour PNG and hide a recorder bug.

### 2.3 Resolution and the undistortion decision

Both cameras use the same resolution, written to `sensor.yaml` as `resolution: [width, height]`.
Default **1280x800 @ 30 fps** (the OV9282 native mode; the EEPROM default
`getDefaultIntrinsics(socket)` is at 1280x800). `640x400` is the light option (native binned mode,
~4x smaller files; `getCameraIntrinsics(socket, 640, 400)` gives fx 399.56, cx 324.73, cy 195.77 for
CAM_B). Avoid 1280x720: it is a crop and cy shifts by 40 px (`getCameraIntrinsics(CAM_B, 1280, 720)`
returns cy 351.54 vs 391.54); DepthAI handles it but it is one more thing to get wrong.

**The PNGs are undistorted, not raw.** Reason: this unit's EEPROM stores the OpenCV **rational**
model, 14 coefficients `[k1 k2 p1 p2 k3 k4 k5 k6 s1 s2 s3 s4 tx ty]` with large, mutually cancelling
values (CAM_B: k1 = 23.21, k2 = -36.26, k3 = 15.94, k4 = 22.59, k5 = -34.47, k6 = 14.63). NVIDIA's
loader can only build `Brown(k1, k2, 0, p1, p2)` from a `sensor.yaml`, and the truncated polynomial is
not approximately right, it is catastrophically wrong:

| normalised radius r | rational model radial factor (truth) | `Brown(k1, k2, 0)` as the loader would build it |
|---|---|---|
| 0.1 | 1.0049 | 1.2284 |
| 0.3 | 1.0152 | 2.7948 |
| 0.5 | 1.0134 | 4.5352 |

cuVSLAM's `Distortion.Model.Polynomial` takes exactly the first 8 OpenCV coefficients (the official
`examples/oak-d/run_stereo.py` does `getDistortionCoefficients(socket)[:8]` on raw frames), but the
EuRoC loader has no way to express that. The only representation that is exact in **both** readers is
zero distortion, so the recorder undistorts on the host:

```python
K = np.array(calib.getCameraIntrinsics(socket, width, height))       # 3x3 at the recorded size
D = np.array(calib.getDistortionCoefficients(socket)[:14])            # full rational + thin prism + tilt
map1, map2 = cv2.initUndistortRectifyMap(K, D, None, K, (width, height), cv2.CV_32FC1)
gray_undist = cv2.remap(gray_raw, map1, map2, cv2.INTER_LINEAR)      # per frame, ~2 ms at 1280x800
```

`newCameraMatrix = K` keeps `fu fv cu cv` identical to the EEPROM values, so `sensor.yaml.intrinsics`
is just `K` and `distortion_coefficients: [0.0, 0.0, 0.0, 0.0]` -> `Brown(0, 0, 0, 0, 0)` = pinhole in
NVIDIA's loader, `Distortion.Model.Pinhole` in ours. Undistortion does not rotate the camera frame, so
the extrinsics (section 6) are unchanged. Black corners appear where the undistorted image reaches
outside the sensor; they produce no features and need no mask. The maps are computed once per camera
per session, the same code runs on Windows, WSL2 and the Jetson, and it is unit-testable without a
camera (synthetic grid through `cv2.projectPoints` with `D`, then `remap`, then check straightness).

If the recorder is given a raw mode (frames as delivered, for A/B tests against the official OAK-D
example), the dataset must say so: `sensor.yaml` gets `distortion_model: rational-polynomial` with
the 8 OpenCV coefficients and `dataset.json.cameras.cam0.undistorted: false`; our reader maps that to
`Polynomial`; and `README.txt` must state that NVIDIA's `track_euroc.py` will build a wrong camera
model from it. The default is always undistorted.

---

## 3. `cam0/data.csv` and `cam1/data.csv`

```
#timestamp [ns],filename
1758326123456789000,1758326123456789000.png
1758326123490122000,1758326123490122000.png
```

- Line 1 is skipped unconditionally (`next(f)`, line 202), whatever it says; keep the EuRoC header
  text above for other tools.
- Then `csv.reader` rows; `int(row[0])` is the timestamp, `row[1]` the file name (lines 210-213). At
  least 2 fields, no spaces after the comma, no quotes. Empty lines are skipped.
- Rows in strictly increasing timestamp order, cam0 and cam1 in lockstep (section 2.1). The loader
  pairs cam0/cam1 by row index (lines 245-247) **before** its single stable sort by timestamp
  (line 274, run in every mode), so write rows in order or the pairing and the sort will disagree.
- ASCII, LF line endings (`open(..., "w", newline="\n")` on Windows), trailing newline, no BOM.

---

## 4. `imu0/data.csv`

```
#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]
1758326123451234000,0.0012207031,-0.0006103516,0.0003051758,-0.0598145,-0.7910156,-9.9414062
```

- Header skipped as above. Each row needs **>= 7 fields**: `row[1:4]` -> `gyro`, `row[4:7]` -> `accel`
  (lines 215-221). **Gyroscope first, accelerometer second**, both in the IMU sensor frame `S`.
- Units are exactly cuVSLAM's `ImuMeasurement` units: `angular_velocities` in **rad/s**,
  `linear_accelerations` in **m/s^2** (nanobind docstrings, `cuvslam2.cpp` lines 386-387). DepthAI's
  `IMUPacket.gyroscope.{x,y,z}` is rad/s and `IMUPacket.acceleroMeter.{x,y,z}` is m/s^2 (cheat-sheet
  section 6), so the values are copied, not converted. The accelerometer reports specific force
  (about +9.8 m/s^2 opposite to gravity at rest); that is what cuVSLAM expects - do not subtract gravity.
- Source sensors: `ACCELEROMETER_RAW` + `GYROSCOPE_RAW` on one `dai.node.IMU`, `setBatchReportThreshold(1)`,
  default **200 Hz** (`--imu-rate 400` allowed: the cheat-sheet measured 400.3 packets/s). Raw, not
  `_CALIBRATED`: a VIO estimates its own biases and the BNO086's on-the-fly calibration updates would
  show up as bias steps. Do not put `ROTATION_VECTOR` on the same node (all sensors on one node run at
  the slowest enabled rate).
- One row per `IMUPacket`. Row timestamp = the **accelerometer report's device timestamp**
  (`packet.acceleroMeter.tsDevice`, `.sec`/`.nsec`, nanosecond resolution), converted as in section 8;
  this is the field `camera/04_imu.py` already uses for its spacing statistics. Record the largest
  `|t_gyro - t_accel|` seen in `dataset.json.measured.imu_max_accel_gyro_skew_ms`.
- Timestamps **non-decreasing** (cuVSLAM: `register_imu_measurement` "must be called in non-decreasing
  timestamp order"; out of order raises `ValueError`). Drop and count any sample that would go
  backwards (`drops.imu_out_of_order`). Count gaps > 2.5 sample periods as `drops.imu_gaps`
  (same rule as `04_imu.py`; the BNO086 `sequence` byte steps by 2 routinely, so do not use it).
- The IMU stream should start **before** the first stereo pair and end after the last one; NVIDIA's
  `track_euroc.py` (lines 136-142, its own counter; the tracker itself prints nothing) warns
  `Warning: No IMU measurements between timestamps ...` for any frame interval without samples.
- Float formatting: `format(v, ".10g")` is enough (values are O(10) with ~1e-4 resolution).

---

## 5. `sensor.yaml` (cameras)

### 5.1 What the loader reads

`_create_camera_from_config` (lines 117-137) and `get_rig` (140-188):

| Key | Loader use | Mandatory | Our value |
|---|---|---|---|
| `T_BS.data` | 16 floats, `reshape(4, 4)` -> **row-major** 4x4; the key is found as *the first key starting with `T_`* (`_find_transform_key`), so no other `T_*` keys | yes | cam0: identity; cam1: `B_from_C` in metres (section 6) |
| `T_BS.rows`, `T_BS.cols` | ignored | no (EuRoC has them) | `4`, `4` |
| `resolution` | `cam.size = [width, height]` | yes | `[1280, 800]` |
| `intrinsics` | `cam.focal = [0:2]` = `(fu, fv)`, `cam.principal = [2:4]` = `(cu, cv)` | yes | `getCameraIntrinsics(socket, w, h)` -> `[K00, K11, K02, K12]` |
| `distortion_coefficients` | `Brown` with `coeffs[:2] + [0] + coeffs[2:]`: **exactly 4** values `[k1, k2, p1, p2]` -> `[k1, k2, 0, p1, p2]` (5 values would become 6, not a valid Brown set: Brown = exactly 5, `cuvslam2.cpp` lines 225-227; `Distortion` accepts any length, lines 253-254, and neither loader checks the count, so the error, if any, surfaces only inside the closed-source core at `Tracker` construction) | yes | `[0.0, 0.0, 0.0, 0.0]` |
| `distortion_model` | **ignored** (model is chosen by file name only) | no | `radial-tangential` (EuRoC spelling) |
| `camera_model` | ignored | no | `pinhole` |
| `rate_hz` | ignored for cameras | no | `30` |
| `sensor_type`, `comment` | ignored | no | `camera`, free text with device id, socket, sensor, mode |

`Camera.border_*` are left at 0 by the loader (no masks), unlike the official OAK-D example
(`50/0/70/70` for raw frames). Our reader may set them from `dataset.json` if a raw dataset is used.

### 5.2 Float formatting rule

`yaml.safe_load` follows YAML 1.1: a scalar is a float only if the mantissa contains a `.` **and** any
exponent has an explicit sign. `1e-05`, `1.5e5` and `5e-3` load as **strings** and the nanobind setter
raises; `1.0e-05`, `0.0001`, `-5.06e-05` are floats. Either write the files with `yaml.safe_dump`
(it emits `1.0e-05`) or, when hand-writing the EuRoC-style template with comments, pass every number
through a helper that takes `repr(float(x))` and inserts `.0` before the `e` if the mantissa has no
dot. Write `0.0`, not `0`. Ints (`rate_hz: 200`, `resolution: [1280, 800]`) are fine.
`pyyaml` is not in the Windows venv today; it becomes a dependency of the recorder and the reader
(`pyyaml==6.0.2`, the version pinned by `examples/requirements.txt`).

### 5.3 Worked example: `mav0/cam0/sensor.yaml` (this unit, 1280x800, undistorted)

```yaml
# OAK-D-S2-AF OAKDS2EXAMPLEID000, CAM_B (OV9282 mono global shutter) = stereo left = cam0.
# Frames are undistorted on the host with the 14 EEPROM coefficients; intrinsics unchanged.
sensor_type: camera
comment: OAK-D-S2-AF OAKDS2EXAMPLEID000 CAM_B left OV9282 1280x800 GRAY8 30fps undistorted

# Sensor extrinsics wrt. the body frame; body = cam0, so this is the identity.
T_BS:
  cols: 4
  rows: 4
  data: [1.0, 0.0, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0,
         0.0, 0.0, 1.0, 0.0,
         0.0, 0.0, 0.0, 1.0]

# Camera specific definitions.
rate_hz: 30
resolution: [1280, 800]
camera_model: pinhole
intrinsics: [799.1265869140625, 798.9035034179688, 649.467529296875, 391.53851318359375] # fu, fv, cu, cv
distortion_model: radial-tangential
distortion_coefficients: [0.0, 0.0, 0.0, 0.0] # k1, k2, p1, p2 - frames are pre-undistorted
```

`mav0/cam1/sensor.yaml` differs in `comment`, `intrinsics`
(`[797.5361938476562, 797.4699096679688, 652.9262084960938, 368.2416687011719]`) and `T_BS`:

```yaml
T_BS:
  cols: 4
  rows: 4
  data: [ 0.9999974966,  -0.0007925885,  0.0020770160,  0.0749669075,
          0.0007950460,   0.9999989867, -0.0011828458, -0.0000506308,
         -0.0020760756,   0.0011844947,  0.9999971390,  0.0001151837,
          0.0,            0.0,           0.0,           1.0]
```

That is `getCameraExtrinsics(CAM_C, CAM_B)` with the translation divided by 100 (cm -> m): the right
camera sits 74.967 mm along +x of the left camera, as it must (calibrated baseline
`getBaselineDistance(useSpecTranslation=False)` = 7.4967 cm). The recorder writes full `repr`
precision; the example is rounded for width.

---

## 6. Frames, extrinsics and units

### 6.1 `T_BS` semantics and the body frame

EuRoC defines `T_BS` as the transform from sensor frame `S` to body frame `B`
(`p_B = T_BS * p_S`, "body_from_sensor"). The loader uses it exactly so (lines 78-83, 159-177):

```
cam0.rig_from_camera = identity                                    # cam0 is the rig, whatever T_BS(cam0) is
cam1.rig_from_camera = inv(T_BS(cam0)) @ T_BS(cam1)                # = cam0_from_cam1
imu.rig_from_imu     = inv(T_BS(cam0)) @ T_BS(imu0)                # = cam0_from_imu
```

So the **rig frame is always cam0**, not the body, and the body frame is only an intermediate. We
choose **body = cam0** (the CAM_B optical frame: x right, y down, z forward, origin at the left
camera's optical centre). Then `T_BS(cam0) = I` and the two products above are the file contents
unchanged, so what you read in the yaml is what the tracker gets, and `world_from_rig` in the
trajectory is the pose of the left camera. NVIDIA's OAK-D live example uses CAM_A as rig frame
instead; `cuvslam_oak/rig.py` should use CAM_B so live and offline trajectories share a frame.
`transform_to_pose` converts with `scipy Rotation.from_matrix(...).as_quat()` (x, y, z, w) - the
rotation blocks must be proper rotations (det +1, orthonormal); the EEPROM ones are to 1e-7.

### 6.2 DepthAI extrinsics direction and units (verified on the saved EEPROM, no device)

`CalibrationHandler.getCameraExtrinsics(src, dst)` returns **dst_from_src** (maps points expressed in
`src` into `dst`), translation in **centimetres** (`lengthUnit: 1`). Evidence from this unit:

| Call | translation (cm) | reading |
|---|---|---|
| `getCameraExtrinsics(CAM_B, CAM_C)` | `(-7.4966, 0.0110, -0.0271)` | left camera is at -x of the right camera, in the right camera's frame -> C_from_B |
| `getCameraExtrinsics(CAM_C, CAM_B)` | `(+7.4967, -0.0051, +0.0115)` | right camera at +x of the left -> **B_from_C = T_BS(cam1)** |
| `getCameraExtrinsics(CAM_B, CAM_A)` | `(-3.7373, 0.0334, -0.1100)` | A_from_B; the official OAK-D example uses this as `rig_from_camera` with rig = CAM_A, consistent |
| `getCameraExtrinsics(CAM_C, CAM_A)` | `(+3.7591, 0.0806, -0.0594)` | A_from_C |
| `getImuToCameraExtrinsics(CAM_A)` | `R = diag(1, -1, -1)`, `t = (-3.7948, -0.0790, 0.0)` | equals the raw EEPROM `imuExtrinsics` (`toCameraSocket = 0` = CAM_A): **A_from_IMU** |
| `getCameraToImuExtrinsics(CAM_A)` | `t = (+3.7948, -0.0790, 0.0)` | its inverse, IMU_from_A |
| `getImuToCameraExtrinsics(CAM_B)` | `t = (-0.0577, -0.1127, +0.1096)` | **B_from_IMU = T_BS(imu0)**; equals `inv(A_from_B) @ A_from_IMU` to 3e-7 |

Therefore the recorder writes, with `t / 100`:

- `T_BS(cam1) = getCameraExtrinsics(CAM_C, CAM_B)`
- `T_BS(imu0) = getImuToCameraExtrinsics(CAM_B)` =
  `[0.9999622107, -0.0069612898, -0.0052090380, -0.0005770779, -0.0069251354, -0.9999520183, 0.0069269701, -0.0011273669, -0.0052570086, -0.0068906355, -0.9999624491, 0.0010956685, 0, 0, 0, 1]`

The IMU sits within 1.2 mm of the left camera's optical centre and its axes are x = camera x (right),
y = camera -y (up), z = camera -z (backward), i.e. `diag(1, -1, -1)` up to the 0.4 deg CAM_A -> CAM_B
rotation. Consistency check with the cheat-sheet's at-rest reading `(-0.06, -0.79, -9.94) m/s^2`:
specific force points along IMU -z = camera +z, so the camera was lying on its back facing the
ceiling when that was logged - plausible for a desk test. In the tracker, `get_last_gravity()` returns
gravity in the rig (cam0) frame and should read about `[0, +9.8, 0]` when the camera is upright
(y down); if it points elsewhere the IMU extrinsics or axis signs are wrong (NVIDIA's EuRoC README
says the same).

---

## 7. `imu0/sensor.yaml`

| Key | Loader use (lines 176-182) | Our value |
|---|---|---|
| `T_BS.data` | `imu.rig_from_imu = inv(T_BS(cam0)) @ T_BS(imu0)` | `B_from_IMU` in metres (section 6.2) |
| `rate_hz` | `imu.frequency` (Hz, float) | `200` or `400`, the enabled report rate |
| `gyroscope_noise_density` | `imu.gyroscope_noise_density` [rad / s / sqrt(Hz)] | see below |
| `gyroscope_random_walk` | `imu.gyroscope_random_walk` [rad / s^2 / sqrt(Hz)] | see below |
| `accelerometer_noise_density` | `imu.accelerometer_noise_density` [m / s^2 / sqrt(Hz)] | see below |
| `accelerometer_random_walk` | `imu.accelerometer_random_walk` [m / s^3 / sqrt(Hz)] | see below |
| `sensor_type`, `comment` | ignored | `imu`, `OAK-D-S2-AF OAKDS2EXAMPLEID000 BNO086 ACCELEROMETER_RAW+GYROSCOPE_RAW 200Hz` |

All four noise keys are mandatory (plain `dict[...]` access). Nobody has measured the BNO086's
Allan deviation for this project yet, and the EEPROM's `imuCalibrationParams.noise` block
(`gyroscope.x.noiseDensity: 0.465`, `randomWalk: 0.049`, `accelerometer.x.noiseDensity: 0.069`,
`randomWalk: 4.6e-05`, per axis) has undocumented units, so it is copied verbatim into `dataset.json`
for reference but **not** used. The recorder writes the values NVIDIA ships for a consumer MEMS IMU
in `examples/realsense/camera_utils.py` (lines 29-32), which are known to work with cuVSLAM's VIO,
until a bench measurement replaces them:

```yaml
sensor_type: imu
comment: OAK-D-S2-AF OAKDS2EXAMPLEID000 BNO086 ACCELEROMETER_RAW+GYROSCOPE_RAW 200Hz

# Sensor extrinsics wrt. the body frame (body = cam0 = CAM_B): B_from_IMU, metres.
T_BS:
  cols: 4
  rows: 4
  data: [ 0.9999622107, -0.0069612898, -0.0052090380, -0.0005770779,
         -0.0069251354, -0.9999520183,  0.0069269701, -0.0011273669,
         -0.0052570086, -0.0068906355, -0.9999624491,  0.0010956685,
          0.0,           0.0,           0.0,           1.0]
rate_hz: 200

# Inertial sensor noise model parameters (static). Placeholder values from NVIDIA's RealSense example;
# replace with measured BNO086 values (see dataset.json imu.noise_source).
gyroscope_noise_density: 6.0673370376614875e-03     # [ rad / s / sqrt(Hz) ]
gyroscope_random_walk: 3.6211951458325785e-05       # [ rad / s^2 / sqrt(Hz) ]
accelerometer_noise_density: 3.3621979208052800e-02 # [ m / s^2 / sqrt(Hz) ]
accelerometer_random_walk: 9.8256589971851467e-04   # [ m / s^3 / sqrt(Hz) ]
```

(For scale: EuRoC's ADIS16448 uses `1.6968e-04`, `1.9393e-05`, `2.0e-03`, `3.0e-03`.) Write the
exponent form with a dot and a signed exponent, as in section 5.2.

---

## 8. Timestamps

- **Clock:** one clock for everything, the **device monotonic clock**: `ImgFrame.getTimestampDevice()`
  for frames and `IMUReport.tsDevice` for IMU reports. Both come from the Myriad X's clock, so
  cameras and IMU are consistent ("IMU sensors and cameras clocks must be synchronized"). The
  host-synchronised `getTimestamp()` is not used for the data because its device->host offset is
  re-estimated continuously; it is only used once, to compute the epoch offset below.
- **Integer conversion:** frame timestamps are `datetime.timedelta` (microsecond resolution):
  `ns = td.days * 86_400_000_000_000 + td.seconds * 1_000_000_000 + td.microseconds * 1_000`. Never
  `int(td.total_seconds() * 1e9)` at epoch magnitude (float64 resolution there is ~250 ns). IMU
  reports: `ts.sec * 1_000_000_000 + ts.nsec`.
- **Epoch offset:** the device clock starts near zero at boot, which would make file names change
  digit count mid-session. The recorder adds one constant `epoch_offset_ns` to every device
  timestamp so the values are Unix-epoch nanoseconds (19 digits, as EuRoC's own
  `1403636579763555584`). Computed once from the first frame:
  `epoch_offset_ns = (time.time_ns() - ns(dai.Clock.now())) + (ns(frame.getTimestamp()) - ns(frame.getTimestampDevice()))`
  and stored in `dataset.json.time.epoch_offset_ns`; subtract it to get back to the raw device clock.
  Accuracy is DepthAI's host sync (~ms), which is irrelevant to tracking (only differences matter) and
  good enough for matching a flight log later.
- **Ordering rules from cuVSLAM** (`tracker.py` docstrings): `track()` timestamps **strictly
  increasing**; `register_imu_measurement()` **non-decreasing**; the two interleaved in
  non-decreasing order. `ValueError` is raised for out-of-order timestamps, wrong IMU order or
  inconsistent image sizes (`tracker.py` lines 120-121, 161-162). Separately, cuVSLAM *recommends*
  that the real capture times of the cameras in one `track()` call be within 1 ms (`track()`
  docstring, line 98); the Python API takes one timestamp per call and stamps it on every image
  (`cuvslam2.cpp` lines 643-655), so per-camera skew cannot be expressed to it, let alone raise.
- **Gaps:** the tracker has `max_frame_delta_s` in `OdometryConfig`; the official OAK-D example warns
  when a camera gap exceeds 35 ms at 30 fps. The recorder counts frame gaps > 1.5 periods in
  `drops.stereo_sequence_gaps` (from `getSequenceNum()`) and stores the largest gap in `measured`.
- **Warm-up:** the first ~1 s after `pipeline.start()` is over/under-exposed (cheat-sheet section 3;
  the official example discards 60 frames). The recorder discards frames until `common.warm_up()`
  returns and records the count in `dataset.json.record.warmup_frames_discarded`; IMU rows begin at
  the first kept frame minus one IMU period or earlier.

---

## 9. How `dataset_utils.py` (v17.0.0) loads a dataset, step by step

1. `get_rig(euroc_path)`: `_get_sensor_paths` checks for `cam0/sensor_recalibrated.yaml`; absent ->
   `cam0/sensor.yaml`, `cam1/sensor.yaml`, `imu0/sensor.yaml`, `is_default = True`. Each is
   `yaml.safe_load`ed (`FileNotFoundError` if missing).
2. `get_transform_from_config` takes the first `T_*` key, `np.array(...['data']).reshape(4, 4)`.
3. `_create_camera_from_config`: `focal = intrinsics[0:2]`, `principal = intrinsics[2:4]`,
   `size = resolution`, `distortion = Distortion(Model.Brown, coeffs[:2] + [0] + coeffs[2:])`.
   (`is_default = False` would mean `Model.Fisheye` with the 4 coefficients as given.)
4. `cam0.rig_from_camera = Pose([0,0,0,1], [0,0,0])`; `cam1.rig_from_camera =
   transform_to_pose(inv(T_cam0) @ T_cam1)`; `imu.rig_from_imu = transform_to_pose(inv(T_cam0) @ T_imu)`;
   `imu.*_noise_density / *_random_walk / frequency` from the IMU yaml;
   `rig = Rig(); rig.cameras = [cam0, cam1]; rig.imus = [imu]`.
5. `prepare_frame_metadata_euroc(euroc_path, mode)`: reads both camera csvs (`read_csv_data`), requires
   equal length and per-row equal timestamps, builds
   `{'type': 'stereo', 'timestamp', 'images_paths': [cam0/data/f, cam1/data/f]}` per pair; in
   `OdometryMode.Inertial` appends `{'type': 'imu', 'timestamp', 'gyro', 'accel'}` per IMU row; then
   **one stable sort by timestamp**. Because stereo entries were appended first, an IMU sample with a
   timestamp exactly equal to a frame's is delivered **after** that frame. (NVIDIA's own RealSense
   example does the opposite, registering all IMU samples with `t <= frame_t` before `track()`; our
   reader follows the RealSense order, which is the physically right one, and both satisfy cuVSLAM's
   non-decreasing rule.)
6. Images are loaded lazily per frame with `load_frame` (section 2.2).

Loader dependencies (`examples/requirements.txt` at the tag): `numpy==2.2.4`, `opencv-python~=4.11.0`,
`pillow==12.2.0`, `pyyaml==6.0.2`, `rerun-sdk==0.33.1`, `scipy==1.14.1` (scipy only for
`Rotation.from_matrix`).

---

## 10. How `track_euroc.py` (v17.0.0) runs, and how to point it at our data

`track_euroc.py` has **no command-line interface**. Everything is hard-coded and changed by editing
the file:

| What | Where | Value at the tag |
|---|---|---|
| dataset path | line 25: `sequence_path = os.path.join(os.path.dirname(__file__), "dataset/mav0")` | so put or symlink our recording at `examples/euroc/dataset/mav0` (`ln -s /path/to/cuvslam_<stamp>/mav0 examples/euroc/dataset/mav0` on WSL2/Jetson), or edit the line |
| mode | line 90: `euroc_tracking_mode = cuvslam.Tracker.OdometryMode.Inertial` | `Multicamera` (= stereo only, `OdometryMode(0)`), `Inertial` (stereo + IMU, `1`), `Mono` (`3`); the README suggests validating `Mono` and `Multicamera` first if `Inertial` misbehaves |
| tracker config | lines 93-99: `OdometryConfig(async_sba=False, enable_observations_export=True, enable_final_landmarks_export=True, rectified_stereo_camera=False, odometry_mode=...)` | our undistorted-but-not-rectified frames are exactly this case |
| visualiser | line 37: `rr.init("cuVSLAM Visualizer", spawn=True)` | always spawns a Rerun viewer window; there is no headless flag (on WSL2 that needs a display; on the Jetson it works on the desktop) |
| output | lines 117, 159: `odom_trajectory.append([timestamp] + list(translation) + list(rotation))` | accumulated in memory and **never written**; the row order is already TUM's `t tx ty tz qx qy qz qw`, but `t` is in ns |

Sequencing loop (lines 119-149): for each metadata entry in sorted order, an `imu` entry becomes
`m = ImuMeasurement()` followed by `m.timestamp_ns = int(t)`, `m.linear_accelerations = np.asarray(accel)`,
`m.angular_velocities = np.asarray(gyro)` (lines 125-128; the three-argument constructor is
keyword-only, `cuvslam2.cpp` lines 383-384, so a positional timestamp raises `TypeError`) and
`tracker.register_imu_measurement(0, m)`; a `stereo` entry loads the two PNGs and calls
`tracker.track(timestamp, images)` (the wrapper returns `(PoseEstimate, slam_pose_or_None)`);
`world_from_rig is None` means tracking failed for that frame; otherwise
`world_from_rig.pose.translation` / `.rotation` are logged. In `Inertial` mode
`tracker.get_last_gravity()` is drawn as the red arrow. Run `python3 track_euroc.py` from `examples/euroc`
as the README does (it also works from any directory: the `dataset_utils` import and `sequence_path`
are both resolved relative to the script file).

---

## 11. Trajectory output: TUM format for evo

`methods/cuvslam-vio/08_cuvslam_track.py` writes `trajectory_<mode>.tum` (and NVIDIA's rows, if patched to save,
would be the same after `t / 1e9`):

```
# timestamp_s tx ty tz qx qy qz qw   (cuVSLAM world_from_rig, rig = cam0 optical frame, metres)
1758326123.456789000 0.000000 0.000000 0.000000 0.000000 0.000000 0.000000 1.000000
1758326123.490122000 0.001204 -0.000310 0.004871 0.000122 -0.000940 0.000018 0.999999
```

- One pose per successfully tracked frame, space separated, `#` lines are comments.
- `timestamp_s` = `f"{ns // 10**9}.{ns % 10**9:09d}"` (exact; avoid `ns / 1e9` formatting noise).
- `tx ty tz` = `pose.translation` in metres; `qx qy qz qw` = `pose.rotation` **as is**: cuVSLAM's
  `Pose.rotation` is documented as "quaternion in x, y, z, w order" (`cuvslam2.cpp` lines 183, 192),
  which is TUM's order. No reordering, no inversion: the pose is `world_from_rig` (rig in world), the
  TUM convention.
- "world" is the rig pose at the first tracked frame (identity), so x right / y **down** / z forward
  of the starting left-camera view. For a floor-plan plot use the x-z plane:
  `evo_traj tum trajectory_inertial.tum --plot --plot_mode xz`.
- Compare two runs / two modes: `evo_traj tum a.tum b.tum --ref a.tum --align --plot`;
  against a reference: `evo_ape tum ref.tum est.tum --align` (`-a` = SE(3) Umeyama alignment,
  `--correct_scale` for mono), `evo_rpe tum ref.tum est.tum --delta 1 --delta_unit m`.
  evo associates by timestamp with `--t_max_diff 0.01` s default, fine at 30 fps.
- Frames that failed (`world_from_rig is None`) are skipped, not written as NaN; the count goes to
  the tracking script's summary.

---

## 12. Our additions (do not touch the loader)

### 12.1 `dataset.json` (top level, next to `mav0/`)

Written once at the end of the recording (`json.dump(..., indent=2)`), read by our reader and tests.
Everything the EuRoC files cannot carry lives here. Keys and an example from a fictitious 60 s run:

```json
{
  "format": "euroc-asl",
  "format_version": 1,
  "generator": {
    "script": "datasets/07_record_dataset.py",
    "depthai_version": "3.10.0",
    "opencv_version": "5.0.0",
    "python_version": "3.13.3",
    "platform": "Windows-11-10.0.26200",
    "hostname": "laptop"
  },
  "device": {
    "device_id": "OAKDS2EXAMPLEID000",
    "name": "OAK-D-S2",
    "product": "OAK-D-S2-AF",
    "platform": "RVC2",
    "usb_speed": "SUPER",
    "board_name": "DM9098",
    "board_rev": "R9M2E9",
    "imu": "BNO086",
    "imu_firmware": "3.9.9",
    "eeprom_version": 7,
    "eeprom_batch_time": 1764034136
  },
  "record": {
    "start_wall_time": "2026-09-19T20:31:07-04:00",
    "start_unix_ns": 1758328267123456000,
    "requested_duration_s": 60.0,
    "duration_s": 60.03,
    "headless": true,
    "warmup_frames_discarded": 45,
    "stopped_by": "duration",
    "png_workers": 4
  },
  "time": {
    "clock": "device",
    "epoch_offset_ns": 1758328265512345678,
    "epoch_offset_source": "time.time_ns() - ns(dai.Clock.now()) + (ns(getTimestamp()) - ns(getTimestampDevice())) at the first kept frame",
    "unit": "ns"
  },
  "cameras": {
    "cam0": {"socket": "CAM_B", "role": "left",  "sensor": "OV9282", "resolution": [1280, 800], "fps": 30.0,
             "type": "GRAY8", "undistorted": true, "distortion_model_in_yaml": "radial-tangential",
             "undistort": {"method": "cv2.initUndistortRectifyMap + cv2.remap INTER_LINEAR", "coefficients_used": 14,
                           "new_camera_matrix": "same as intrinsics"},
             "sensor_yaml": "mav0/cam0/sensor.yaml", "data_csv": "mav0/cam0/data.csv"},
    "cam1": {"socket": "CAM_C", "role": "right", "sensor": "OV9282", "resolution": [1280, 800], "fps": 30.0,
             "type": "GRAY8", "undistorted": true, "distortion_model_in_yaml": "radial-tangential",
             "undistort": {"method": "cv2.initUndistortRectifyMap + cv2.remap INTER_LINEAR", "coefficients_used": 14,
                           "new_camera_matrix": "same as intrinsics"},
             "sensor_yaml": "mav0/cam1/sensor.yaml", "data_csv": "mav0/cam1/data.csv"}
  },
  "stereo": {
    "pairing": "dai.node.Sync, threshold_s 0.0167, timestamp of the pair = left frame device timestamp",
    "max_left_right_skew_ms_allowed": 1.0,
    "baseline_m": 0.074967,
    "body_frame": "cam0 (CAM_B optical frame)"
  },
  "imu": {
    "sensors": ["ACCELEROMETER_RAW", "GYROSCOPE_RAW"],
    "rate_hz": 200,
    "batch_report_threshold": 1,
    "timestamp_source": "acceleroMeter.tsDevice",
    "data_csv": "mav0/imu0/data.csv",
    "sensor_yaml": "mav0/imu0/sensor.yaml",
    "noise_source": "placeholder: NVIDIA examples/realsense/camera_utils.py values; not measured on this BNO086",
    "eeprom_noise": {"name": "BNO086", "gyroscope": {"x": {"noiseDensity": 0.4652, "randomWalk": 0.0492, "biasStability": 3.9986}},
                     "accelerometer": {"x": {"noiseDensity": 0.0692, "randomWalk": 4.61e-05, "biasStability": 0.0387}},
                     "units": "undocumented, copied verbatim from calibration.json imuCalibrationParams.noise"}
  },
  "counts": {
    "stereo_pairs": 1800,
    "imu_samples": 12006,
    "first_timestamp_ns": 1758328267123456000,
    "last_timestamp_ns": 1758328327090122000
  },
  "measured": {
    "stereo_fps": 29.98,
    "imu_hz": 199.9,
    "max_stereo_gap_ms": 34.1,
    "max_imu_gap_ms": 8.9,
    "max_left_right_skew_us": 16,
    "imu_max_accel_gyro_skew_ms": 0.4,
    "png_write_ms_mean": 9.8,
    "png_encode_ms_mean": 42.1,
    "png_throughput_pairs_per_s": 30.0,
    "png_bytes_total": 1734520000
  },
  "drops": {
    "stereo_sequence_gaps": 0,
    "stereo_unpaired": 0,
    "stereo_sync_violations": 0,
    "stereo_write_queue_dropped": 0,
    "imu_gaps": 0,
    "imu_out_of_order": 0,
    "imu_queue_dropped": 0
  },
  "calibration": {
    "source": "device.readCalibration() (EEPROM)",
    "dump": "calibration.json",
    "extrinsics_convention": "getCameraExtrinsics(src, dst) = dst_from_src, cm -> m; T_BS = body_from_sensor, body = cam0",
    "T_BS_cam1_from": "getCameraExtrinsics(CAM_C, CAM_B)",
    "T_BS_imu0_from": "getImuToCameraExtrinsics(CAM_B)"
  },
  "warnings": []
}
```

Rules: every key above is present (use `null` when unknown, e.g. `usb_speed` on a replayed source);
`counts` and `drops` are integers; `measured` values are floats; numbers are plain JSON (no NaN).
`format_version` is bumped when a key changes meaning. `calibration.json` is the unmodified
`CalibrationHandler.eepromToJson()` dict exactly as `05_record.py` writes it (`json.dump(..., indent=2,
default=str)`), so the 14 distortion coefficients, the raw cm extrinsics and the IMU noise block are
always recoverable from a dataset.

### 12.2 `README.txt`

Plain text, written with `dataset.json`, for a human opening the folder on any machine:

```
cuVSLAM recording  cuvslam_20260919_203107   (EuRoC ASL layout, see datasets/dataset-format.md)
Device   OAK-D-S2-AF OAKDS2EXAMPLEID000, depthai 3.10.0, USB SUPER
Streams  cam0 = CAM_B left, cam1 = CAM_C right: 1280x800 GRAY8 @ 30 fps, undistorted, 1800 pairs
         imu0 = BNO086 ACCELEROMETER_RAW + GYROSCOPE_RAW @ 200 Hz, 12006 samples
Time     device clock + epoch offset (Unix ns); 60.03 s; first 1758328267123456000, last 1758328327090122000
Drops    stereo gaps 0, unpaired 0, sync violations 0, write-queue 0; imu gaps 0, out-of-order 0
Frames   T_BS = body_from_sensor, body = cam0 (left camera optical frame, x right, y down, z forward)
Caveats  IMU noise densities in imu0/sensor.yaml are placeholders (not measured on this unit).

Track with our reader (WSL2 / Jetson, cuvslam 17.0.0 installed):
  python methods/cuvslam-vio/08_cuvslam_track.py recordings/cuvslam_20260919_203107 --mode stereo-inertial --out trajectory.tum
Track with NVIDIA's example, unmodified:
  ln -s "$PWD/recordings/cuvslam_20260919_203107/mav0" PyCuVSLAM/examples/euroc/dataset/mav0
  cd PyCuVSLAM/examples/euroc && python3 track_euroc.py       # edit line 90 for stereo-only
Plot:  evo_traj tum trajectory.tum --plot --plot_mode xz
```

### 12.3 What `cuvslam_oak/dataset.py` (our reader) must do

1. Accept either the recording root (contains `mav0/`) or the `mav0` directory itself; resolve with
   `pathlib`.
2. Refuse to load if `cam0/sensor_recalibrated.yaml` exists (report it: NVIDIA's loader would switch
   to Fisheye).
3. Parse the three `sensor.yaml` files with `yaml.safe_load`; require the keys marked mandatory in
   sections 5.1 and 7; check every `intrinsics`/`T_BS`/noise value is a `float`/`int` (a `str`
   means the section 5.2 formatting bug). Honour `distortion_model` + coefficient count:
   `radial-tangential` with 4 -> `Brown [k1,k2,0,p1,p2]` (NVIDIA-identical), with 5 -> `Brown`;
   all-zero -> `Pinhole`; `equidistant` with 4 -> `Fisheye`; `rational-polynomial` with 8 ->
   `Polynomial`; anything else -> error. Assert both cameras share `resolution`.
4. Build the `Rig` exactly as section 6.1 / step 4 of section 9 (cam0 identity, `inv(T0) @ T1`,
   `inv(T0) @ T_imu`), with a scipy-free 3x3 -> xyzw quaternion routine if scipy is unwanted (20
   lines), and `border_*` from `dataset.json` when `undistorted` is false.
5. Read both camera csvs and the IMU csv with the same rules as `read_csv_data` (skip line 1, `int`
   timestamp, `float` values, skip blank lines), then **validate**: equal pair counts, identical
   timestamps per row, strictly increasing frames, non-decreasing IMU, every PNG present, first IMU
   sample <= first frame (warn otherwise), IMU rate within 10 % of `rate_hz` (warn), and the
   `dataset.json` counts match the csv row counts.
6. Yield one merged stream sorted by timestamp with **IMU samples before a frame of equal timestamp**
   (RealSense-example order), as `("imu", t, gyro, accel)` / `("stereo", t, [path0, path1])` tuples
   so `tracking.py` can feed `register_imu_measurement(0, m)` then `track(t, [img0, img1])`.
7. Load images with `cv2.imread(path, cv2.IMREAD_UNCHANGED)`, assert `uint8`, 2-D, `(height, width)`,
   return C-contiguous arrays.
8. Everything above runs on Windows with the fake cuvslam (`fake_cuvslam.py` provides `Pose`,
   `Camera`, `Distortion`, `ImuCalibration`, `Rig`, `ImuMeasurement` with the same field names), so
   `tests/` can build a tiny synthetic dataset (3 pairs, 30 IMU rows, 8x8 PNGs) and assert every rule
   in this document, including the negative cases (mismatched counts, string floats, 16-bit PNG,
   `sensor_recalibrated.yaml` present, non-monotonic timestamps).

---

## 13. Sources read (PyCuVSLAM tag v17.0.0, 2026-09-19)

| File | Used for |
|---|---|
| `examples/euroc/dataset_utils.py` | everything in sections 1-9 about the loader (line numbers refer to this file) |
| `examples/euroc/track_euroc.py` | section 10 |
| `examples/euroc/README.md` | modes, distortion-model list, `sensor_recalibrated.yaml` copy step |
| `examples/euroc/sensor_cam0.yaml`, `sensor_cam1.yaml`, `sensor_imu0.yaml` | EuRoC key layout (these are the recalibrated fisheye variants) |
| `examples/requirements.txt` | loader dependencies |
| `examples/oak-d/run_stereo.py` | official raw-frame OAK-D rig (Polynomial 8, rig = CAM_A, Sync node, borders, 60-frame warm-up, 35 ms gap warning) |
| `examples/realsense/run_vio.py`, `camera_utils.py` | IMU-before-frame ordering (`register_imu_until`), placeholder noise densities |
| `python/cuvslam2.cpp` | `Pose` quaternion order, `Distortion` models and coefficient orders, `Camera.size = (width, height)`, `ImuMeasurement` units, `track()` / `register_imu_measurement()` contracts, `OdometryConfig` fields |
| `python/tracker.py`, `python/__init__.py`, `python/docs/api.rst` | `Tracker(rig, odom_config)` wrapper, `track()` returning `(PoseEstimate, slam_pose)`, `get_last_gravity()` frame |
| `camera/docs/depthai-v3-cheatsheet.md` sections 2.2, 3, 6, 9.1; `camera/04_imu.py`, `05_record.py`, `common.py` | DepthAI timestamp/IMU/calibration facts and the repo's recording conventions |
| `recordings/video_20260918_030056/calibration.json` via `dai.CalibrationHandler` | every number in sections 2.3, 5.3, 6.2, 7 |
