# OAK-D S2 calibration -> cuVSLAM rig

How to turn the calibration stored in this OAK-D-S2-AF (device id `OAKDS2EXAMPLEID000`) into a
PyCuVSLAM v17.0.0 `Rig` (two `Camera`s + one `ImuCalibration`), with every number worked from the
unit's own EEPROM dump and every claim tied to the source that supports it.

Written 2026-09-19 **without opening the camera**. All numbers come from
`recordings/video_20260918_034017/calibration.json` (the `eepromToJson()` dump saved by
`camera/05_record.py`) loaded offline with `dai.CalibrationHandler(Path(json))` in depthai 3.10.0
(constructor overload 2 in `help(dai.CalibrationHandler)`); the other four `recordings/video_*/calibration.json`
files are the same JSON (checked with `json.load(a) == json.load(b)`; batchTime 1764034136, EEPROM version 7).
The cuVSLAM side was read from the **v17.0.0 tag** of NVlabs/PyCuVSLAM (`python/cuvslam2.cpp`,
`python/tracker.py`, `examples/oak-d/run_stereo.py`, `examples/realsense/{run_vio,camera_utils}.py`,
`examples/euroc/{dataset_utils,track_euroc}.py`, `examples/euroc/sensor_*.yaml`); the DepthAI side from the
installed 3.10.0 docstrings, the `v3.10.0` tag of luxonis/depthai-core (`src/device/CalibrationHandler.cpp`,
`src/pipeline/node/Sync.cpp`, `src/pipeline/datatype/ImgFrame.cpp`, `src/basalt/BasaltVIO.cpp`,
`src/rtabmap/RTABMapVIO.cpp`, `include/depthai/properties/{IMUProperties,SyncProperties}.hpp`,
`include/depthai/common/{ImuParameters,Extrinsics}.hpp`, `tests/src/ondevice_tests/pipeline/node/imu_test.cpp`,
`examples/python/VSLAM/basalt_vio.py`) and docs.luxonis.com. Section 9 is the evidence log.

Conventions: `X_from_Y` is the 4x4 SE(3) matrix that maps points expressed in frame Y into frame X
(`p_X = X_from_Y @ p_Y`); its translation column is the origin of Y expressed in X. cuVSLAM names its
fields the same way (`rig_from_camera`, `rig_from_imu`, `world_from_rig`).

---

## 0. The answer in one table

| cuVSLAM field | DepthAI call (3.10.0) | Value for this unit |
|---|---|---|
| `Camera.size` | the size passed to `requestOutput`: `(1280, 800)` native or `(640, 400)` | see section 1 |
| `Camera.focal`, `Camera.principal` | `calib.getCameraIntrinsics(socket, w, h)` (default `keepAspectRatio=True`) | left@1280x800 fx 799.127 fy 798.904 cx 649.468 cy 391.539; right fx 797.536 fy 797.470 cx 652.926 cy 368.242 |
| `Camera.distortion` | `Distortion(Model.Polynomial, calib.getDistortionCoefficients(socket)[:8])` | order identical on both sides: k1 k2 p1 p2 k3 k4 k5 k6 (section 2) |
| `Camera.rig_from_camera` (rig = CAM_A) | `calib.getCameraExtrinsics(socket, CAM_A, False, dai.LengthUnit.METER)` **as is, no inversion** | left t = (-0.03737, +0.00033, -0.00110) m, right t = (+0.03759, +0.00081, -0.00059) m (section 3) |
| `ImuCalibration.rig_from_imu` | `calib.getImuToCameraExtrinsics(CAM_A, False, dai.LengthUnit.METER)` **as is**, paired with `ACCELEROMETER_RAW`/`GYROSCOPE_RAW` | R = diag(1,-1,-1) (quaternion xyzw = (1,0,0,0)), t = (-0.037948, -0.00079, 0) m (section 4) |
| `ImuCalibration.frequency` | the rate passed to `enableIMUSensor` | 200 (measured 200.2 Hz spacing on this unit) |
| noise densities / random walks | **not** from the EEPROM until Luxonis documents its units | conservative defaults in section 5 |
| `track(timestamp, ...)` (`tracker.py`: `def track(self, timestamp: int, images, ...)`; positional), `ImuMeasurement.timestamp_ns` | `getTimestampDevice(dai.CameraExposureOffset.MIDDLE)` for frames, `report.getTimestampDevice()` for IMU, both converted to integer ns | one device-monotonic domain (section 6) |
| axis flips | none for cameras; none for IMU data (the rotation lives in `rig_from_imu`) | section 7 |

`OdometryConfig`: `rectified_stereo_camera=False` for raw (unrectified) GRAY8 frames, `odometry_mode=Multicamera`
for stereo-only or `Inertial` for stereo + BNO086 (v17.0.0 `vslam.Tracker(rig, odom_cfg)`; `main` already has a
breaking `Tracker.Mode` argument, so pin v17.0.0).

---

## 1. `getCameraIntrinsics(socket, w, h)` scaling vs the native 1280x800 mono calibration

### 1.1 What the EEPROM holds

`M, w, h = calib.getDefaultIntrinsics(CAM_B)` -> `w, h = 1280, 800` (the 3.10.0 binding returns a flat 3-tuple
`tuple[list[list[float]], int, int]`, as the cheat-sheet section 2.2 unpacks it; `M, (w, h) = ...` would fail); the
mono cameras were calibrated at their native
1280x800 (`cameraData[1].width/height = 1280/800`, `getSourceWidth/Height(CAM_B)` = 1280/800).

| socket | fx | fy | cx | cy |
|---|---|---|---|---|
| CAM_B (stereo left) @1280x800 | 799.1266 | 798.9035 | 649.4675 | 391.5385 |
| CAM_C (stereo right) @1280x800 | 797.5362 | 797.4699 | 652.9262 | 368.2417 |

`getFov(CAM_B)` = 71.86 deg (spec) / 77.38 deg from intrinsics; `getDistortionModel` = `CameraModel.Perspective`.

### 1.2 The exact algorithm (depthai-core v3.10.0 `CalibrationHandler.cpp` lines 347-425)

Signature: `getCameraIntrinsics(cameraId, resizeWidth=-1, resizeHeight=-1, topLeftPixelId=Point2f(0,0),
bottomRightPixelId=Point2f(0,0), keepAspectRatio=True)`; `-1` means "no resize"; one of `w`/`h` may be `-1` and is
then derived from the native aspect ratio.

1. `keepAspectRatio=True` (default): one uniform scale `s`; if the requested aspect differs from the native one the
   image is **scaled to fill and centre-cropped** (`cy -= (H*s - h)/2` or `cx -= (W*s - w)/2`). There is a special
   branch (`resizeRatio <= 1.34 and 1.5 < originalRatio <= 1.778`, i.e. 4:3-ish requests from a 16:9/16:10 sensor)
   that uses `s = min(sw, sh)` and *pads* instead (`cy += (h - H*s)/2`); it does not fire for any request in this doc.
2. `keepAspectRatio=False`: independent `sx = w/W`, `sy = h/H` (a **stretch**: fx and fy scale differently).
3. The optional crop rectangle is applied **after** the resize, in resized-image pixels
   (`cx -= topLeft.x; cy -= topLeft.y`); asking for a crop larger than the resized image throws
   `RuntimeError: Invalid Crop size. Crop width or height is more than the original resized height and width`
   (observed when passing `(1280,720)` + crop `(0,40)-(1280,760)`; use `(1280,800)` + that crop instead).

Nothing in this function touches the distortion coefficients: OpenCV-style normalised-coordinate coefficients are
scale-invariant, so the same 14 numbers are valid at every resolution, and a centre crop is fine because cx,cy are
shifted consistently (the distortion centre is still cx,cy).

### 1.3 Worked numbers (CAM_B; CAM_C behaves identically)

| request | keepAspectRatio | fx | fy | cx | cy | what it models |
|---|---|---|---|---|---|---|
| (-1,-1) or (1280,800) | any | 799.1266 | 798.9035 | 649.4675 | 391.5385 | native, unchanged |
| (640,400) | True or False (same) | 399.5633 | 399.4518 | 324.7338 | 195.7693 | **exact halving** of all four |
| (320,200) | any | 199.7816 | 199.7259 | 162.3669 | 97.8846 | exact quartering |
| (1280,720) | True | 799.1266 | 798.9035 | 649.4675 | **351.5385** | 80-row centre crop (cy - 40) = `ImgResizeMode.CROP` |
| (1280,720) | False | 799.1266 | **719.0131** | 649.4675 | **352.3846** | vertical stretch by 0.9 = `ImgResizeMode.STRETCH` |
| (640,360) | True | 399.5633 | 399.4518 | 324.7338 | 175.7693 | halve then crop 40 rows |
| (1280,800) + crop (0,40)-(1280,760) | - | 799.1266 | 798.9035 | 649.4675 | 351.5385 | explicit crop, same as the CROP row |

CAM_C at 640x400: fx 398.7681 fy 398.7350 cx 326.4631 cy 184.1208.

So for 640x400 the answer is "exact halving, no crop, `keepAspectRatio` irrelevant" because 640x400 has the native
16:10 aspect. The 0.25 px half-pixel subtlety: a 2x2 binned sensor mode (CAM_B/C list a native 640x400@255.7 mode)
strictly maps `cx -> (cx+0.5)/2-0.5 = 324.484`, DepthAI returns `cx/2 = 324.734`; the 0.25 px difference is below
what cuVSLAM can resolve, ignore it.

`ImgResizeMode` (`CROP` default, `STRETCH`, `LETTERBOX`) has no mapping for `LETTERBOX` in `getCameraIntrinsics`:
letterboxing scales by `min(sw, sh)` and *adds* padding (`cy += (h - H*s)/2`), which the function only does in
the special 4:3 branch. Never request a letterboxed output for VSLAM.

### 1.4 What our source must request

* `cam.requestOutput((1280, 800), dai.ImgFrame.Type.GRAY8, fps=30)` (native: `getCameraIntrinsics(sock, 1280, 800)`
  is the EEPROM matrix verbatim), or `cam.requestOutput((640, 400), dai.ImgFrame.Type.GRAY8, fps=30)` (exact halving).
  `GRAY8` is what NVIDIA's `run_stereo.py` requests; on this unit mono defaults to `RAW8`, and both deliver a
  `(H, W)` uint8 array from `getCvFrame()` (cheat-sheet section 3), which is what `ImageFromNDArray` in
  `cuvslam2.cpp` consumes (2-D uint8, pitch = `stride(0)`). Request `GRAY8` explicitly to stay identical to the example.
* Do **not** copy the example's 1280x720: it silently moves cy by -40 px (CROP) or scales fy by 0.9 (STRETCH), and
  whether the firmware realises a 1280x720 request as a centred crop of the 800-row sensor (there is a native
  1280x720@143 sensor mode) could not be verified without the device.
* Runtime guard (mandatory in `oak_source.py`): on the first delivered frame compare
  `np.array(frame.getTransformation().getIntrinsicMatrix())` with `calib.getCameraIntrinsics(sock, w, h)` and
  `frame.getTransformation().getSourceSize()` with `(1280, 800)`; abort if they differ by more than 1e-2 px.
  `ImgTransformation` also exposes `getDistortionCoefficients()`/`getDistortionModel()`, so the same guard can check
  that the delivered stream is *not* undistorted. The cheat-sheet (section 4, 'Observed') saw the mono intrinsics
  (fx 399.6 at 640x400) on the StereoDepth **depth** output's `getTransformation()`; the same accessor on a
  `Camera.requestOutput` mono frame is expected to carry them but is untested (see 'Not verified' in section 9).
* Frames must be "images of the same camera": never mix a 640x400 request with a 1280x800 rig, and give
  `Camera.size = (w, h)` of the delivered frames.

---

## 2. Distortion: OpenCV 14 -> cuVSLAM `Distortion.Model.Polynomial` (8)

### 2.1 Order and this unit's values

`getDistortionCoefficients` docstring (3.10.0): "[k1,k2,p1,p2,k3,k4,k5,k6,s1,s2,s3,s4,tx,ty] for
CameraModel::Perspective ... (Rational Polynomial Model)". cuVSLAM (`cuvslam2.cpp`, `Distortion.Model.Polynomial`):
"Polynomial distortion model with 8 coefficients, order: (k1, k2, p1, p2, k3, k4, k5, k6)", "Coefficients are
compatible with first 8 coefficients of OpenCV distortion model", projection
`radial = (1 + k1 r^2 + k2 r^4 + k3 r^6) / (1 + k4 r^2 + k5 r^4 + k6 r^6)` plus the usual p1/p2 tangential terms.
**Same order on both sides; the official example's `[:8]` slice is exactly right and needs no reordering.**

| socket | k1 | k2 | p1 | p2 | k3 | k4 | k5 | k6 | s1..s4 | tx | ty |
|---|---|---|---|---|---|---|---|---|---|---|---|
| CAM_B | 23.2061 | -36.2617 | 0.000239 | 0.001831 | 15.9420 | 22.5879 | -34.4736 | 14.6299 | 0 | -0.005444 | -0.001956 |
| CAM_C | 25.5888 | -40.2922 | -0.000052 | 0.001028 | 18.8577 | 24.9089 | -38.3200 | 17.3955 | 0 | -0.000544 | 0.000385 |

Non-zero on this unit: k1..k6, p1, p2 and the **tilted-sensor terms tx, ty** (indices 12, 13); the thin-prism
terms s1..s4 are zero.

### 2.2 What the truncation costs (computed with `cv2.projectPoints`, full 14 vs first 8, 1280x800)

| socket | max error anywhere in frame | p95 | max inside the example's border mask (top 50, left/right 70) | max inside the central 640x400 |
|---|---|---|---|---|
| CAM_B | 3.28 px | 1.97 px | 2.90 px | 0.82 px |
| CAM_C | 0.43 px | 0.27 px | 0.38 px | 0.11 px |

(At 640x400 halve these.) The left camera's tilt is 10x the right one's, so the stereo pair is asymmetrically wrong
after truncation. Options, in order of preference: (a) accept it and use larger border masks on the left camera
(the example's 50/70/70 mask "to avoid using features from highly distorted regions" leaves up to 2.9 px);
(b) feed on-device **undistorted** mono streams (`requestOutput(..., enableUndistortion=True)`, verified on CAM_A in
the cheat-sheet, **untested on mono**) with `Distortion.Model.Pinhole` and the intrinsics read from
`frame.getTransformation().getIntrinsicMatrix()` (undistortion changes K); (c) feed `StereoDepth.rectifiedLeft/Right`
with `rectified_stereo_camera=True`. Decide with the week-2 bench protocol, not on paper.

Never drop k4..k6 to fit the 5-coefficient `Brown` model: the rational numerator and denominator nearly cancel
(at r^2 = 0.89 num 4.17 / den 4.11 = 1.014 for CAM_B), so `Brown(k1,k2,k3,p1,p2)` mis-projects by ~2700 px.
This also means NVIDIA's EuRoC loader (`dataset_utils._create_camera_from_config`: `Brown` with `[k1,k2,0,p1,p2]`
for `sensor.yaml`, `Fisheye` for `sensor_recalibrated.yaml`) **cannot** represent our raw frames; to let
`track_euroc.py` load our recordings unmodified, the EuRoC copy must contain undistorted (or rectified) images with
zero coefficients, while our own `dataset.py` keeps the 8-coefficient model. That is a dataset-writer decision; the
rig builder itself is unaffected.

---

## 3. `getCameraExtrinsics(src, dst)`: which transform, which units, which sign

### 3.1 Semantics from the source

`getCameraExtrinsics(srcCamera, dstCamera, useSpecTranslation=False, unit=CENTIMETER)`; docstring: "srcCamera:
Camera Id of the camera which will be considered as origin. dstCamera: ... destination camera to which we are
fetching the rotation and translation from the SrcCamera". The implementation (`CalibrationHandler.cpp` 518-556)
chains the per-camera EEPROM links (`cameraData[src].extrinsics` is the 3x3 rotation + translation towards
`toCameraSocket`) up to the common origin camera, inverts the destination chain and returns
`inv(origin_from_dst) @ origin_from_src` = **`dst_from_src`**, the SE(3) that maps points from the src camera
frame into the dst camera frame; translation = origin of src expressed in dst, in **centimetres** unless
`unit=dai.LengthUnit.METER`. `Extrinsics.hpp`: `translation` is "(x, y, z) pose of destCameraSocket w.r.t
currentCameraSocket obtained through calibration", `specTranslation` "measured through CAD design". **That header
comment is worded the wrong way round**: the implementation (`computeExtrinsicMatrix`, `CalibrationHandler.cpp`
v3.10.0 line 946, `matMul(destTransformationMatrixCurr, currTransformationMatrixSrc)`) treats each EEPROM link as
`toCameraSocket_from_camera`, so `getCameraExtrinsics(src, dst)` = `dst_from_src` and its translation is the origin of
src expressed in dst (the CAM_B link's t.x = -7.4966 cm puts the left camera to the left of the right camera, as it
must; read literally as "pose of dest w.r.t current" the signs of sections 3.2-3.3 would all flip). The same header's
`getInverseTransformationMatrix` comment ("transformation from the toCameraSocket to the current camera socket")
agrees with the code, not with the `translation` comment; do not "correct" the direction by inverting. Luxonis'
calibration_reader example (docs.luxonis.com/software/depthai/examples/calibration_reader) labels
`getCameraExtrinsics(CAM_B, CAM_C)` "Transformation matrix of where left Camera is W.R.T right Camera's optical
center" and `getCameraExtrinsics(CAM_B, CAM_A)` "... W.R.T RGB Camera's optical center", and Luxonis (jakaskerl,
forum thread 5997, about an OAK-D-S2 `getCameraExtrinsics(CAM_C, CAM_A, False)`) confirmed "SE(3)" in "RDF (right
hand)".

Trap: `getCameraExtrinsics` defaults to `useSpecTranslation=False` (calibrated), `getCameraTranslationVector` and
`getBaselineDistance` default to `useSpecTranslation=True` (CAD). Always pass it explicitly.

### 3.2 Worked numbers for this unit (cm, `useSpecTranslation=False`)

EEPROM links: CAM_B -> CAM_C: t = (-7.4966, 0.0110, -0.0271); CAM_C -> CAM_A: t = (3.7591, 0.0806, -0.0594);
CAM_A is the origin (no link). Right camera is at +x of the colour camera, the left camera is at -x of the right one,
consistent with RDF (x = image right).

| call | translation (cm) | meaning |
|---|---|---|
| `getCameraExtrinsics(CAM_B, CAM_A)` | (-3.7373, +0.0334, -0.1100) | `A_from_B`: left camera sits 3.74 cm to the left of CAM_A |
| `getCameraExtrinsics(CAM_C, CAM_A)` | (+3.7591, +0.0806, -0.0594) | `A_from_C`: right camera 3.76 cm to the right |
| `getCameraExtrinsics(CAM_B, CAM_C)` | (-7.4966, +0.0110, -0.0271) | `C_from_B` |
| `getCameraExtrinsics(CAM_C, CAM_B)` | (+7.4967, -0.0051, +0.0115) | `B_from_C` = inverse of the row above |
| `getBaselineDistance()` / `(useSpecTranslation=False)` | 7.5000 / 7.4967 | spec vs calibrated |

Round-off check only: `A_from_C @ C_from_B` equals `A_from_B` to < 2e-7. This is **not** independent evidence for the
direction: `computeExtrinsicMatrix` builds `getCameraExtrinsics(CAM_B, CAM_A)` as exactly that product (recursion
CAM_B -> CAM_C -> CAM_A), so the comparison only measures float32 round-off. The checks that do constrain the
convention (all recomputed from the JSON): (a) physical sign: `t_right - t_left` = (+0.074964, +0.000472, +0.000506) m,
i.e. the right camera is at +x (image right, RDF) of the left one, |.| = 0.074967 m vs the 0.075 m spec; (b) inverse
consistency: `getCameraExtrinsics(CAM_C, CAM_B)` (t = (+7.4967, -0.0051, +0.0115) cm) equals
`inv(getCameraExtrinsics(CAM_B, CAM_C))` (t = (-7.4966, +0.0110, -0.0271) cm) to 5e-7; (c) IMU lever arm:
`inv(A_from_B) @ getImuToCameraExtrinsics(CAM_A)` has t = (-0.58, -1.13, +1.10) mm and equals
`getImuToCameraExtrinsics(CAM_B)` to 2e-8 m, placing the IMU at the left mono camera as the CAD layout says.

### 3.3 Does the official example's `getCameraExtrinsics(CAM_B, CAM_A)` make rig = CAM_A? Yes.

cuVSLAM: `Camera.rig_from_camera` = "Transformation from the camera coordinate frame to the rig coordinate frame"
= `rig_from_camera`. `run_stereo.py` passes `getCameraExtrinsics(CAM_B, CAM_A)` = `A_from_B` for the left camera and
`getCameraExtrinsics(CAM_C, CAM_A)` = `A_from_C` for the right, dividing the translation by 100. Hence
`rig == CAM_A frame` (the colour camera's optical centre, RDF), and the example is dimensionally and directionally
correct. In metres:

```
rig_from_left  = A_from_B = [[ 0.999962 -0.006925 -0.005257 -0.037373]
                             [ 0.006961  0.999952  0.006891  0.000334]
                             [ 0.005209 -0.006927  0.999962 -0.001100]
                             [ 0         0         0         1       ]]
rig_from_right = A_from_C = [[ 0.999965 -0.007724 -0.003172  0.037591]
                             [ 0.007742  0.999954  0.005722  0.000806]
                             [ 0.003128 -0.005747  0.999979 -0.000594]
                             [ 0         0         0         1       ]]
t_right - t_left = (0.074964, 0.000472, 0.000506) m, |.| = 0.074967 m, direction (0.99996, 0.0063, 0.0068)
```

So `|right - left| = 0.0750 m` along **+x** of the rig, as required (the 0.14 deg relative rotation between the
mono cameras is the un-rectified reality that `rectified_stereo_camera=False` accounts for).

**Rule for `rig.py`: no inversion, no transpose.** `rig_from_camera = getCameraExtrinsics(camera_socket, CAM_A,
False, dai.LengthUnit.METER)` verbatim (or cm/100 like the example). Inverting would put the left camera at +x and
break stereo. The only conversion is rotation matrix -> quaternion in **(x, y, z, w)** order for `vslam.Pose`
(`Pose` docstring); scipy is not in the Windows venv, so `rig.py` should carry a small, tested matrix-to-quaternion
routine. If the rig frame were instead chosen as the left camera (EuRoC style, `cam0 = identity`), the values would be
`left_from_right = inv(A_from_B) @ A_from_C`, t = (0.074967, -0.000051, 0.000115) m, and `rig_from_imu` in section 4
would have to be re-based the same way; keep rig = CAM_A so the numbers match the official example and
`getImuToCameraExtrinsics(CAM_A)` can be used directly.

---

## 4. IMU: `imuExtrinsics`, `getImuToCameraExtrinsics`, BNO086 axes, `rig_from_imu`

### 4.1 What the EEPROM stores

```
imuExtrinsics.toCameraSocket = CAM_A
imuExtrinsics.rotationMatrix = [[1,0,0],[0,-1,0],[0,0,-1]]          (180 deg about x; quaternion xyzw = (1,0,0,0))
imuExtrinsics.translation     = (-3.7948, -0.0790, 0.0) cm  == specTranslation (CAD value, not calibrated)
imuCalibrationParams.accelerometer = gyroscope = identity affine [[1,0,0,0],[0,1,0,0],[0,0,1,0]]
```

The translation says the IMU sits 3.79 cm to the left of and 0.8 mm above the colour camera, i.e. essentially at
the left mono camera: `getImuToCameraExtrinsics(CAM_B)` has t = (-0.6, -1.1, +1.1) mm. Rotation is a pure 180 deg
about x: IMU x = camera x, IMU y = -camera y, IMU z = -camera z.

### 4.2 Semantics of `getImuToCameraExtrinsics(cameraId, useSpecTranslation=False, unit=CENTIMETER)`

Source (`CalibrationHandler.cpp` 804-835): it builds `[R | t]` from `imuExtrinsics` and, when `cameraId !=
toCameraSocket`, left-multiplies by `getCameraExtrinsics(toCameraSocket, cameraId)`, i.e. it returns
`camera_from_toSocket @ toSocket_from_imu` = **`camera_from_imu`**. `getCameraToImuExtrinsics` is literally its
inverse (`invertSe3Matrix4x4InPlace`, verified numerically: inverse equal to 2e-8). Luxonis' own VIO glue agrees:
`BasaltVIO.cpp` feeds `getCameraToImuExtrinsics(cam, ..., METER)` as Basalt's `T_i_c` (imu_from_camera), and
`RTABMapVIO.cpp` feeds `getImuToCameraExtrinsics(cam, true, METER)` as RTAB-Map's IMU `localTransform`
(camera_from_imu). For this unit:

```
getImuToCameraExtrinsics(CAM_A, False, METER) = [[ 1  0  0 -0.037948]
                                                 [ 0 -1  0 -0.000790]
                                                 [ 0  0 -1  0.000000]
                                                 [ 0  0  0  1       ]]   == A_from_IMU == rig_from_imu (rig = CAM_A)
```

### 4.3 Which frame the IMU samples are in (this decides the rotation)

* `IMUProperties.hpp` / docstrings (3.10.0): `ACCELEROMETER_RAW` = "Raw accelerometer measurement in the
  sensor-native frame. No IMU extrinsics or affine calibration are applied. Units are [m/s^2]", with the note
  "Prior firmware versions incorrectly delivered frame-aligned data on this stream (equivalent to what is now
  ACCELEROMETER_UNCALIBRATED)". `GYROSCOPE_RAW`: "sensor-native frame ... Units are [rad/s]".
  `*_UNCALIBRATED` = "aligned to the DepthAI IMU frame without the stored affine calibration",
  `*_CALIBRATED` = same plus the affine calibration (identity on this unit, so CALIBRATED == UNCALIBRATED here).
* docs.luxonis.com/hardware/platform/sensors/imu (verbatim): the frame is the "Luxonis camera coordinate convention"
  / "Luxonis RDF coordinate system", "+X right +Y down +Z forward"; "a vector measured in the IMU frame is rotated
  into the camera frame using the rotation component of `imuExtrinsics`"; Raw = "direct sensor outputs as reported by
  the IMU itself ... in the sensor's native frame"; Uncalibrated = "constructed from the raw sensor streams after
  applying the frame rotation needed to align them to the Luxonis camera coordinate convention" ("No factory
  intrinsic correction is applied at this stage."); Calibrated = "frame rotation into the Luxonis RDF coordinate
  system", "factory intrinsic calibration terms", "compensation for deterministic sensor errors". The wording
  "Luxonis RDF frame" is the IMU node page's (docs.luxonis.com/software-v3/depthai/depthai-components/nodes/imu):
  "`*_RAW`: direct sensor output in the sensor-native frame", "`*_UNCALIBRATED`: rotated into the Luxonis RDF frame
  using `imuExtrinsics`, without IMU calibration parameters applied", "`*_CALIBRATED`: rotated into the Luxonis RDF
  frame and corrected using IMU calibration parameters".
* Direction of that rotation, from the on-device test `imu_test.cpp` ("Test setImuRotation applies extrinsics
  rotation to accelerometer via rotateImuVector"): after `setImuExtrinsics(rotation=R)`, the CALIBRATED accelerometer
  equals `R @ RAW`. So `imuExtrinsics.rotationMatrix` is **camera_from_imu** applied to raw vectors, exactly what
  cuVSLAM's `rig_from_imu` rotation must be.
* Luxonis' Basalt example (`examples/python/VSLAM/basalt_vio.py`) enables `ACCELEROMETER_RAW` + `GYROSCOPE_RAW`
  at 200 Hz and `BasaltVIO.cpp` uses the EEPROM IMU extrinsics with them: RAW + EEPROM rotation is Luxonis' own
  pairing.

Therefore, two consistent recipes (pick one and never mix):

| IMU streams recorded | `rig_from_imu` rotation | `rig_from_imu` translation |
|---|---|---|
| `ACCELEROMETER_RAW` + `GYROSCOPE_RAW` (recommended, mirrors Luxonis' VIO) | `diag(1,-1,-1)` = `getImuToCameraExtrinsics(CAM_A)[:3,:3]` | (-0.037948, -0.00079, 0) m |
| `ACCELEROMETER_UNCALIBRATED`/`_CALIBRATED` + `GYROSCOPE_UNCALIBRATED`/`_CALIBRATED` | identity (already RDF = CAM_A orientation) | same |

Both give the same physics; the first keeps the dataset "sensor-native" like EuRoC/Kalibr conventions.

Implied BNO086 sensor-native axes on the S2 (derived, not documented by Luxonis): x = camera right, y = camera **up**,
z = camera **backward** (towards the user). This matches the BNO08X datasheet's Android device-frame convention
("X axis horizontal, positive to the right, Y axis vertical, positive along the face of the device and Z axis positive
towards the outside of the front face") for a chip mounted flat on the front PCB, and matches a 2021 user report
(depthai-core issue 319) of +10.19 m/s^2 on **y** with the camera upright.

### 4.4 Uncertainties, each with the test that removes it

1. **Firmware frame of `*_RAW`.** *Answered 2026-09-29 on this unit (firmware 3.9.9): sensor-native.* Upright,
   lens horizontal and still, `ACCELEROMETER_RAW` read `(-0.07..-0.41, +8.84..+9.08, +0.76..+2.15)` m/s^2 at
   the start of four walks (hand-held, tilted back 5-13 degrees; the magnitude is 9.1 instead of 9.8 because
   the y axis carries a -0.69 m/s^2 bias, README "Verification status"), and cuVSLAM's gravity came out as
   `(-0.67, +9.71, +1.22)` in the rig frame. The gyro sign test below was not run. Original note:
   The docstring itself says older firmware delivered frame-aligned data on RAW. The
   cheat-sheet's only at-rest sample on this unit (`ACCELEROMETER_RAW`, 200 Hz) was `(-0.06, -0.79, -9.94)` m/s^2 with
   the pose of the camera unrecorded: gravity on -z is consistent with sensor-native axes and the camera lying on its
   back (lens up), or with RDF-aligned data and the camera face-down; it is not consistent with an upright camera in
   either frame. Test (bench, 30 s, camera upright, lens horizontal, level): RAW must read approximately
   `(0, +9.81, 0)`, UNCALIBRATED approximately `(0, -9.81, 0)`; then yaw the camera to the left (counter-clockwise seen
   from above): RAW gyro y > 0, UNCALIBRATED gyro y < 0. Record both stream families once and keep the log with the
   dataset. If RAW reads `(0, -9.81, 0)` upright, the firmware is frame-aligning RAW and the identity row of the table
   applies.
2. **Translation is CAD, not calibrated** (`translation == specTranslation`). 3.8 cm lever arm errors of a few mm are
   irrelevant at VIO accelerations; nothing to do.
3. **Rotation is exactly diag(1,-1,-1)**: no sub-degree IMU-to-camera misalignment is stored. Luxonis said per-unit
   IMU calibration is "on the roadmap" (forum 2445, Mar 2025). A Kalibr camera-IMU calibration would refine it; for now
   accept the CAD value.
4. **cuVSLAM's own check**: `tracker.get_last_gravity()` (tracker.py) returns gravity "in the rig / VO frame; +Y is
   down (OpenCV), so the vector is approximately [0, +g, 0] when the rig is upright". Log it after convergence; a
   sign error in `rig_from_imu` shows up as [0, -g, 0] or as gravity on z.
5. **Accel/gyro packet timestamps** can differ inside one `IMUPacket` (RTAB-Map buffers them separately). Use the
   accelerometer report's timestamp for `ImuMeasurement.timestamp_ns` (Basalt glue does the same) and log
   `|t_acc - t_gyro|` in the recorder.

### 4.5 Units

`IMUReportAccelerometer` "Units are [m/s^2]", `IMUReportGyroscope` "Units are [rad/s]" (docstrings; measured at
rest |a| = 9.97 m/s^2). cuVSLAM `ImuMeasurement.linear_accelerations` "in m/s^2" (specific force incl. gravity, as
in EuRoC) and `angular_velocities` "in rad/s". No scaling.

---

## 5. IMU noise parameters for `ImuCalibration`

cuVSLAM units (`cuvslam2.cpp`): `gyroscope_noise_density` rad/(s*sqrt(Hz)), `gyroscope_random_walk`
rad/(s^2*sqrt(Hz)), `accelerometer_noise_density` m/(s^2*sqrt(Hz)), `accelerometer_random_walk` m/(s^3*sqrt(Hz)),
`frequency` Hz. These are Kalibr's definitions.

### 5.1 What the EEPROM says (`getImuNoiseParameters()`, `name = "BNO086"`)

| axis | accel noiseDensity | accel randomWalk | accel biasStability | gyro noiseDensity | gyro randomWalk | gyro biasStability |
|---|---|---|---|---|---|---|
| x | 0.06924 | 4.61e-05 | 0.03868 | 0.4652 | 0.04919 | 3.999 |
| y | 0.07421 | 1.01e-04 | 0.05859 | 0.5860 | 0.09815 | 5.283 |
| z | 0.08520 | 3.60e-05 | 0.04871 | 0.3907 | 0.06098 | 4.388 |

The docstring says "Accelerometer noise terms are stored in [m/s^2]-based units. Gyroscope noise terms are stored in
[rad/s]-based units"; `ImuParameters.hpp` only says "Allan-variance-derived" and gives no units. Taken literally the
gyroscope numbers are impossible (0.47 rad/s/sqrt(Hz) = 27 deg/s/sqrt(Hz), about 1000x a consumer MEMS gyro; a
4 rad/s bias instability would be unusable). Read in the IMU-industry units that Allan tools print
(deg/sqrt(h), deg/h, m/s/sqrt(h), mg) every number is plausible: gyro ARW 0.39-0.59 deg/sqrt(h) = 1.1e-4..1.7e-4
rad/s/sqrt(Hz), gyro bias instability 4-5 deg/h, accel VRW 0.069-0.085 m/s/sqrt(h) = 1.2e-3..1.4e-3 m/s^2/sqrt(Hz),
accel bias instability 39-59 ug. That is a hypothesis, not a documented fact, and the affine calibration next to it is
identity, so the block looks like a per-model table rather than a per-unit measurement. **Do not feed these numbers to
cuVSLAM until Luxonis documents the units**; keep them in the dataset's `sensor.yaml` as a comment.

### 5.2 Conservative defaults to ship (and why)

```
gyroscope_noise_density      = 3.0e-3   # rad/s/sqrt(Hz)
gyroscope_random_walk        = 3.0e-5   # rad/s^2/sqrt(Hz)
accelerometer_noise_density  = 3.0e-2   # m/s^2/sqrt(Hz)
accelerometer_random_walk    = 1.0e-3   # m/s^3/sqrt(Hz)
frequency                    = 200      # Hz (what enableIMUSensor was asked for; measured 200.2 Hz)
```

Sources: (a) NVIDIA's own consumer-MEMS defaults in `examples/realsense/camera_utils.py` (D435i BMI055-class IMU):
gyro nd 6.0673e-03, gyro rw 3.6212e-05, accel nd 3.3622e-02, accel rw 9.8257e-04 (the values above are the same
order of magnitude, slightly tighter on the gyro); (b) the EuRoC ADIS16448 tactical-grade values shipped in
`examples/euroc/sensor_imu0.yaml` (1.6968e-04, 1.9393e-05, 2.0e-3, 3.0e-3) as the optimistic bound - do not use them
for a BNO086; (c) Kalibr's IMU noise model wiki recommends inflating measured Allan values, since datasheet numbers are
optimistic for a mounted, vibrating sensor; (d) the BNO08X datasheet Revision 1.17 (CEVA 2023, 59 pages; text
extracted with pypdf 6.19.0 from the scratchpad `pylib` folder, which is not in the project venv - see section 9)
publishes no noise density or bias-instability figures at all, only mg / rad/s output units and accuracy classes, and
Luxonis stated in Nov 2023 that they had not measured them (forum thread 2445). The chosen defaults are about 18-26x above
the industry-unit reading of the EEPROM table (gyro 17.6-26.4x, accel 21.1-26.0x per axis from the unrounded EEPROM
values, converted at 1 deg/sqrt(h) = 2.909e-4 rad/s/sqrt(Hz) and 1 m/s/sqrt(h) = 1/60 m/s^2/sqrt(Hz)), which is the
safe side for a VIO estimator: too-large noise makes the
filter trust vision more, too-small noise makes it trust a drifting IMU. Tune down only after the week-2 bench loop
shows the inertial mode holding through short visual dropouts.

---

## 6. Timestamps: one monotonic nanosecond domain for images and IMU

### 6.1 The three clocks (docstrings 3.10.0 + `ImgFrame.cpp`)

* `getTimestamp()` (ImgFrame, IMUReport, IMUData, MessageGroup, Buffer): "Retrieves timestamp related to
  `dai::Clock::now()`" = device time re-based onto the **host** steady clock by DepthAI's host-device sync. Comparable
  with `dai.Clock.now()` (cheat-sheet: host latency about 2 ms for IMU packets).
* `getTimestampDevice()`: "directly captured from device's monotonic clock, not synchronized to host time. Used mostly
  for debugging" and, for the exposure-offset overload, "Used when monotonicity is required". Same clock for cameras
  and IMU (both are stamped on the RVC2).
* `getTimestampSystem()`: "directly captured from device's system clock, that can be synchronized using PTP", returns
  `Optional[datetime.datetime]` (3.10.0 docstring); whether it is `None` on a USB-connected unit was not observed (the
  camera was not opened for this doc). Ignore.

Exposure offsets (`ImgFrame.cpp` 41-66): `getTimestamp(offset)` / `getTimestampDevice(offset)` return
`ts - exposure` for `CameraExposureOffset.START`, `ts - exposure/2` for `MIDDLE`, `ts` for `END`; the plain getters
equal `END`. So **the stored frame timestamp is the end of exposure**; DepthAI does not stamp mid-exposure by default.
The OV9282 is a global shutter, so mid-exposure is the single instant of capture; with auto-exposure up to 30 ms
(observed at start-up on this unit) END and MIDDLE differ by up to 15 ms, i.e. 0.9 deg of yaw at 1 rad/s.
`IMUReport` exposes `getTimestamp()`, `getTimestampDevice()`, `getTimestampSystem()` and `getSequenceNum()` plus the raw
`timestamp`/`tsDevice` (`dai.Timestamp` with `.sec`/`.nsec`), `sequence` and `accuracy` fields
(`[n for n in dir(dai.IMUReport) if not n.startswith('_')]`, 3.10.0); none of the getters take a
`CameraExposureOffset`, so no offset applies.

### 6.2 What the Sync node does (`Sync.cpp`, `SyncProperties.hpp`)

`setTimestampSource(DEFAULT|DEVICE|HOST|SYSTEM)`; "On device the default is DEVICE, on host the default is HOST"
(`SyncProperties.hpp`); the host implementation resolves `DEFAULT` to `HOST` (`getDefaultTimestampSource()`).
The node only *compares* the chosen domain; the output `MessageGroup` copies its metadata from the **newest member**
(`outputGroup->setBufferMetadataFrom(newestFrame)`), so `group.getTimestamp()` is the newest frame's host timestamp
and `group.getTimestampDevice()` its device timestamp, both END-of-exposure (no exposure info on a group). Left and
right frames of one sequence number differ by 11-16 us on this unit (cheat-sheet), so "newest" is a coin flip at that
scale; take the timestamp from the **left frame object itself** rather than from the group. The official example's
`int(message_group.getTimestamp().total_seconds() * 1e9)` therefore uses host time, END of exposure, through a float.

### 6.3 The rule for our code

```python
NS = 1_000_000_000
def td_ns(td) -> int:                       # datetime.timedelta -> exact integer nanoseconds (us resolution)
    return (td.days * 86_400 + td.seconds) * NS + td.microseconds * 1_000

t_img = td_ns(left.getTimestampDevice(dai.CameraExposureOffset.MIDDLE))   # both cameras: use the left frame
t_imu = td_ns(pkt.acceleroMeter.getTimestampDevice())                     # accel report's stamp for the packet
```

1. One domain for everything that cuVSLAM sees: **device monotonic** (`getTimestampDevice`), mid-exposure for images.
   This is exactly Luxonis' choice in `RTABMapVIO.cpp` (`getTimestampDevice(CameraExposureOffset::MIDDLE)` for frames,
   `getTimestampDevice()` for accel/gyro). Host-synced stamps are re-estimated and can step; cuVSLAM requires
   `track()` and `register_imu_measurement()` "in non-decreasing timestamp order" on a clock shared by cameras and IMU
   (`register_imu_measurement` docstring), and `max_frame_delta_s` guards frame gaps.
2. Pair left/right by `getSequenceNum()` (or the Sync node with `setSyncThreshold(timedelta(seconds=0.5/FPS))` as
   the example), never by timestamp equality; give both images the left frame's timestamp (cuVSLAM: "all cameras must
   be synchronized ... within 1 millisecond").
3. Feed IMU samples with `t_imu <= t_img` before each `track(t_img, ...)`, in order, from the same thread (the
   RealSense example's `register_imu_until` pattern); drop any sample whose timestamp is not greater than the last one
   registered (guards `sequence` wrap and duplicated reports).
4. The EuRoC copy stores the same device-domain integers (`<ts_ns>.png`, `imu0/data.csv` with gyro then accel columns
   as `read_csv_data` expects: `row[1:4]` gyro, `row[4:7]` accel). Store host timestamps too (a sidecar
   `timestamps.csv`) so PX4/host events can be aligned later; never mix the two columns in one stream.
5. Python `timedelta` carries microseconds only, so sub-microsecond device time is truncated; irrelevant at 4.99 ms IMU
   spacing (200 Hz) and 33 ms frames.

---

## 7. Frame conventions and axis flips

* cuVSLAM: "uses OpenCV coordinate system convention: x is right, y is down, z is forward" (`Pose` docstring);
  `Camera`: "top left pixel has (0, 0) coordinate (y is down, x is right)"; `world_from_rig` starts at the rig pose of
  the first frame; `get_last_gravity()` is approximately `[0, +g, 0]` upright.
* OAK/DepthAI: camera extrinsics are RDF right-handed (Luxonis, forum 5997), point clouds are "x right, y down,
  z forward" (cheat-sheet section 5), IMU data is in the "Luxonis RDF frame" (IMU node page) = "+X right +Y down
  +Z forward" (hardware IMU page).
* Result: **no axis flip for `rig_from_camera`**, none for image data, none for the IMU samples. The only
  non-identity rotation in the whole rig is `diag(1,-1,-1)` inside `rig_from_imu`, and it comes straight from the
  EEPROM. The RealSense example's `opengl_to_opencv_transform` applies only to transforms typed in an OpenGL-convention
  YAML; do not apply it to OAK data.
* Trajectory output: cuVSLAM poses are `world_from_rig` with rig = CAM_A optical frame (RDF). For a body/PX4 FLU/FRD
  frame a fixed `body_from_rig` must be applied downstream (planner/EKF2 bridge), not inside the rig.

---

## 8. Rig builder contract (`cuvslam_oak/rig.py`)

```python
# inputs: dai.CalibrationHandler (device or JSON), delivered frame size (w, h), imu stream family
A, B, C = dai.CameraBoardSocket.CAM_A, dai.CameraBoardSocket.CAM_B, dai.CameraBoardSocket.CAM_C
M = dai.LengthUnit.METER
def camera(sock):
    K = np.array(calib.getCameraIntrinsics(sock, w, h))                    # keepAspectRatio default; (w,h) in {(1280,800),(640,400)}
    d8 = list(calib.getDistortionCoefficients(sock))[:8]                   # k1 k2 p1 p2 k3 k4 k5 k6
    T = np.array(calib.getCameraExtrinsics(sock, A, False, M))             # rig_from_camera, rig = CAM_A, no inversion
    cam = vslam.Camera(size=(w, h), principal=(K[0, 2], K[1, 2]), focal=(K[0, 0], K[1, 1]),
                       rig_from_camera=vslam.Pose(rotation=quat_xyzw(T[:3, :3]), translation=T[:3, 3]),
                       distortion=vslam.Distortion(vslam.Distortion.Model.Polynomial, d8),
                       border_top=50 * h // 800, border_left=70 * w // 1280, border_right=70 * w // 1280)
    return cam
T_imu = np.array(calib.getImuToCameraExtrinsics(A, False, M))            # rig_from_imu for *_RAW streams
if imu_family == "uncalibrated": T_imu[:3, :3] = np.eye(3)                # RDF-aligned streams: rotation already applied on device
imu = vslam.ImuCalibration(rig_from_imu=vslam.Pose(rotation=quat_xyzw(T_imu[:3, :3]), translation=T_imu[:3, 3]),
                           gyroscope_noise_density=3.0e-3, gyroscope_random_walk=3.0e-5,
                           accelerometer_noise_density=3.0e-2, accelerometer_random_walk=1.0e-3, frequency=200)
rig = vslam.Rig(cameras=[camera(B), camera(C)], imus=[imu])              # order: left = index 0, right = index 1
```

Expected values for the unit tests (from this unit's JSON, tolerance 1e-4): left principal (649.4675, 391.5385) at
1280x800 / (324.7338, 195.7693) at 640x400; `rig_from_camera` translations (-0.037373, 0.000334, -0.001100) and
(0.037591, 0.000806, -0.000594); `rig_from_imu` quaternion (1, 0, 0, 0), translation (-0.037948, -0.00079, 0);
`|t_right - t_left| = 0.074967`; distortion[0] = 23.206055 (left) / 25.588825 (right); `getCameraExtrinsics(B, C)`
translation x = -0.074966 m.

---

## 9. Evidence log

Offline probes (no device), run with the project venv from the scratchpad
`.../scratchpad/cuvslam/{calib_probe.py, calib_probe2.py}` plus inline scripts; all sources are cached in the same
folder (`python_cuvslam2.cpp`, `examples_oak-d__run_stereo.py`, `examples_realsense__{run_vio,camera_utils}.py`,
`examples_euroc__*`, `pycuvslam_tracker.py`, `depthai_CalibrationHandler.cpp`, `depthai_ImgFrame.cpp`, `dai_Sync.cpp`,
`dai_SyncProperties.hpp`, `dai_ImuParameters.hpp`, `dai_Extrinsics.hpp`, `dai_BasaltVIO.cpp`, `dai_RTABMapVIO.cpp`,
`dai_imu_test.cpp`, `bno080_datasheet.pdf`).

* `calib = dai.CalibrationHandler(Path("recordings/video_20260918_034017/calibration.json"))`; `hasCalibrationData()`
  True; `getStereoLeftCameraId()/RightCameraId()` CAM_B/CAM_C; `getCameraWithLowestId()` CAM_A.
* Intrinsics table: `getCameraIntrinsics(CAM_B, w, h[, keepAspectRatio=False])` for (1280,800), (640,400), (1280,720),
  (640,360), (320,200), (-1,-1); crop overloads with `dai.Point2f`. `getCameraIntrinsics(CAM_A, 640, 400)` reproduced
  the cheat-sheet's device-read 483.77/314.94/192.28, confirming the JSON path equals the device path.
* Distortion truncation: `cv2.projectPoints` (opencv-python 5.0) of a 81x53 grid of normalised points with the 14
  coefficients vs the first 8 vs the first 5.
* Extrinsics: `getCameraExtrinsics` for all six socket pairs, `useSpecTranslation` both ways, `unit=METER`;
  round-off check `A_from_C @ C_from_B == A_from_B` (not independent, see section 3.2), inverse check
  `inv(C_from_B) == B_from_C`, `t_right - t_left`, and `inv(A_from_B) @ getImuToCameraExtrinsics(CAM_A) ==
  getImuToCameraExtrinsics(CAM_B)`.
* IMU: `getImuToCameraExtrinsics`/`getCameraToImuExtrinsics` for CAM_A/B/C, `getEepromData().imuExtrinsics`,
  `getImuNoiseParameters()`, `getImuParameters()`, `getAccelerometerCalibration()`, `getGyroscopeCalibration()`.
* PyCuVSLAM v17.0.0 files fetched from `raw.githubusercontent.com/NVlabs/PyCuVSLAM/v17.0.0/...`; depthai-core files
  from `raw.githubusercontent.com/luxonis/depthai-core/v3.10.0/...`; directory listings via the GitHub contents API
  with `?ref=v3.10.0`; `gh search code "biasStability" --repo luxonis/depthai-core` located `ImuParameters.hpp`.
* Web: docs.luxonis.com/hardware/platform/sensors/imu ("+X right +Y down +Z forward", the `imuExtrinsics` rotation
  sentence and the Raw/Uncalibrated/Calibrated prose quoted in section 4.3; the page contains neither "400" nor
  "jitter"); docs.luxonis.com/software-v3/depthai/depthai-components/nodes/imu ("Luxonis RDF frame" wording of the
  `*_RAW`/`*_UNCALIBRATED`/`*_CALIBRATED` streams and, verbatim, "On RVC2 devices with BNO08X, gyroscope frequencies
  above 400 Hz can produce occasional jitter due to sensor hardware limitations.");
  docs.luxonis.com/software/depthai/examples/calibration_reader (`getCameraExtrinsics(CAM_B, CAM_C)` = "where left
  Camera is W.R.T right Camera's optical center", `getCameraExtrinsics(CAM_B, CAM_A)` = "... W.R.T RGB Camera's
  optical center");
  docs.luxonis.com/software-v3/depthai/depthai-components/messages/img_frame (timestamp getters);
  discuss.luxonis.com threads 5997 (extrinsics are SE(3) in RDF), 2445 (no IMU noise measurements, per-unit IMU
  calibration on the roadmap), 397 (BNO085 mounted rotated on OAK-D-IoT-75, no axis statement) read through the
  Flarum JSON API; github.com/luxonis/depthai-core/issues/319 (+10.19 m/s^2 on y, camera upright, 2021 firmware).
* BNO08X datasheet: `bno080_datasheet.pdf` read with pypdf 6.19.0 installed in `.../scratchpad/cuvslam/pylib`
  (via `PYTHONPATH`; pypdf is **not** in the project venv). Page 1: "BNO08X Datasheet Revision 1.17 1 / 59", (c) 2023
  CEVA. Page 30: "In Android, the device frame of reference is represented as X axis horizontal, positive to the
  right, Y axis vertical, positive along the face of the device and Z axis positive towards the outside of the front
  face of the device." Whole-text search (case-insensitive): 0 hits for "Allan", "noise density", "bias
  instability"; "noise" occurs 3 times (a "Noise StdDev (deg)" column of the rotation-vector prediction table on
  page 33 and "noise floor of the gyroscope" prose), i.e. no noise-density or bias-instability figure is published.
* Not verified (needs the camera): the firmware frame of `*_RAW` on depthai 3.10.0 (section 4.4 item 1), mono
  `enableUndistortion=True`, whether a 1280x720 request is a centred sensor crop, and the first-frame
  `getTransformation()` guard.
