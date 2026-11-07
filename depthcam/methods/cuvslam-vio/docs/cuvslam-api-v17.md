# PyCuVSLAM 17.0.0 Python API (verified from the wheel and the v17.0.0 source tag)

Everything below was read on 2026-09-19 from **(a)** the shipped wheel
`cuvslam-17.0.0+cu12-cp310-cp310-manylinux_2_35_x86_64.whl` (its `pycuvslam.pyi` stub,
`__init__.py`, `tracker.py`, `utils.py`, `METADATA`, `RECORD`) and **(b)** the git tag
`v17.0.0` of `NVlabs/PyCuVSLAM` (which now redirects to `nvidia-isaac/cuVSLAM`):
`python/cuvslam2.cpp` (the binding source; it is **nanobind**, not pybind11),
`python/tracker.py`, `python/__init__.py`, `python/utils.py`, `python/docs/api.rst`,
`python/test/{test_api,test_bindings,test_tracking,test_image_format,data_gen}.py`,
`examples/oak-d/run_stereo.py`, `examples/realsense/{run_vio,camera_utils}.py`,
`examples/euroc/{track_euroc,dataset_utils}.py` + `sensor_*.yaml`,
`examples/kitti/{track_kitti_slam,track_kitti_masks}.py`, `examples/requirements.txt`,
`CHANGELOG.md`, `DESIGN_CONCEPTS.md`, `TROUBLESHOOTING.md`, `README.md`.
The wheel's `__init__.py`, `tracker.py` and `utils.py` are byte-identical to the tag's
`python/` copies (checked with `diff`). Nothing in this file comes from memory.

Local copies of every file read: `<scratchpad>/cuvslam/{wheel,src,main}/` (the scratchpad
path is in the task context). `<scratchpad>/cuvslam/wheel/` holds the unzipped wheel minus
the two `.so` files.

Companion docs in this repo: `camera/docs/depthai-v3-cheatsheet.md` (DepthAI side of the same
pipeline), `camera/docs/oak-d-s2-specs.md`, `camera/docs/jetson-orin-nano-setup.md`.

---

## 0. Release assets (v17.0.0, published 2026-07-23T21:55:31Z)

All assets live under `https://github.com/nvidia-isaac/cuVSLAM/releases/download/v17.0.0/`
(`gh release download v17.0.0 -R NVlabs/PyCuVSLAM -p '<name>'` works through the redirect).

| asset | bytes | sha256 |
|---|---|---|
| `cuvslam-17.0.0+cu12-cp310-cp310-manylinux_2_35_x86_64.whl` (WSL2 Ubuntu 22.04, CUDA 12) | 116,412,244 | `0ae3113e43902e0c15423992450510aef241f84207718ed77a1d2d39f28744c1` (verified locally after download) |
| `cuvslam-17.0.0+cu12-cp310-cp310-manylinux_2_35_aarch64.whl` (Jetson Orin, JetPack 6.x) | 113,375,996 | `658250d97c96c5db0d61ed68d8c0e99514c130574339367f0d917ad60f05d315` |
| `cuvslam-17.0.0+cu12-cp312-abi3-manylinux_2_39_x86_64.whl` (Ubuntu 24.04) | 116,479,970 | `39a6adfedbdf01f8e8e746f93095b1ba04c71e3efe258c1efc0251bab1310018` |
| `cuvslam-17.0.0+cu13-cp310-cp310-manylinux_2_35_x86_64.whl` | 89,491,691 | `5d7fbb5662a9a0b027481bfe64a476a1aa1d8e09f90c897252cf071a12a1e90d` |
| `cuvslam-17.0.0+cu13-cp312-abi3-manylinux_2_39_aarch64.whl` (Jetson Thor) | 84,410,378 | `03c34deee80a935d8d581937c12678d68efb525ea7006a9fd795fa34bba079a8` |
| `cuvslam-17.0.0+cu13-cp312-abi3-manylinux_2_39_x86_64.whl` | 89,558,791 | `aa3c38640c806d879add44369eff3757a73695954c7edaa5ebfaceaca356fbb4` |
| `cuvslam-cpp-17.0.0-orin-cuda12.6.3-ubuntu22.04.tar.gz` | 113,820,984 | `57a7b8ccab126491b4ac2a9a3e41f2c3bc123d50b278dffa31afed898f460f0d` |
| `cuvslam-cpp-17.0.0-x86_64-cuda12.6.3-ubuntu22.04.tar.gz` | 116,867,323 | `888f10b4ca9e8ace26beb6847a063d7105b7ea6ef1a8da8e9d4002ae7df4fcac` |
| `cuvslam-docs-17.0.0.tar.gz` (offline Sphinx/Doxygen docs) | 8,054,388 | `d180656ce63e7f6a0397f86fa9297f863f919666f2018de5b87b5b4ce13b80ec` |
| `cuvslam-evaluation-17.0.0.tar.gz` | 16,474,961 | `cee2bf062393d400fdb8059ecee296e15b9eb9e2b92ef7b715afa5b55aa2beab` |

(Also present: `cuvslam-cpp-17.0.0-{thor-cuda13.0.1-ubuntu24.04, x86_64-cuda12.6.3-ubuntu24.04,
x86_64-cuda13.2.0-ubuntu22.04, x86_64-cuda13.2.0-ubuntu24.04}.tar.gz`.)

README wheel table: Ubuntu 22.04 x86_64 -> `cp310`, `cu12`/`cu13`; Jetson Orin (JetPack 6.x)
-> `cp310` + `cu12` aarch64 only; Ubuntu 24.04 / Jetson Thor -> `cp312-abi3`. **CUDA Toolkit
12 (or 13) must be installed separately**; the wheel bundles only `libcudart` (see below), not
cusolver/cublas. Official wheels include cuNLS (needed only for `Multisensor` mode).

### 0.1 Wheel contents (x86_64 cp310 cu12)

```
cuvslam/__init__.py                                  1,615 B   (identical to python/__init__.py)
cuvslam/libcuvslam.so                          170,853,552 B   the engine
cuvslam/py.typed                                         0 B
cuvslam/pycuvslam.cpython-310-x86_64-linux-gnu.so  433,224 B   nanobind extension = "core"
cuvslam/pycuvslam.pyi                               56,689 B   generated stub (ground truth for signatures)
cuvslam/tracker.py                                  13,500 B   pure-Python Tracker wrapper
cuvslam/utils.py                                    15,583 B   YAML config loaders
cuvslam.libs/libcudart-09529672.so.12.6.77         732,712 B   bundled CUDA runtime 12.6.77
cuvslam-17.0.0+cu12.dist-info/{METADATA,WHEEL,RECORD,sboms/auditwheel.cdx.json}
```

`METADATA`: `Name: cuvslam`, `Version: 17.0.0+cu12`, `Requires-Python: >=3.9`,
`Requires-Dist: pyyaml>=5.3.1` (the only runtime dependency; numpy is not declared),
generator `scikit-build-core 1.0.3`, tag `cp310-cp310-manylinux_2_35_x86_64`
(glibc 2.35 = Ubuntu 22.04). There is **no** `cuvslam/core.py`: `core` is an alias of the
extension module.

### 0.2 Module layout (from `cuvslam/__init__.py`)

```python
from .pycuvslam import (get_version, set_verbosity, warm_up_gpu, Pose, Distortion, Camera,
                        ImuCalibration, ImuMeasurement, Rig, PoseStamped, PoseWithCovariance,
                        PoseEstimate, Observation, Landmark)
from . import pycuvslam as core        # cuvslam.core.Odometry, cuvslam.core.Slam, ...
from .tracker import Tracker           # pure-Python wrapper
from . import utils                    # YAML loaders (needs pyyaml)
__version__ = get_version()[0].split('+')[0]     # "17.0.0"
```

`python/test/test_api.py` asserts: `'Tracker' in dir(cuvslam)`, `'Odometry' not in dir(cuvslam)`,
`'Slam' not in dir(cuvslam)`, both present in `dir(cuvslam.core)`. So in v17.0.0 the odometry
and SLAM classes are reached as `cuvslam.core.Odometry` / `cuvslam.core.Slam`, or through the
`Tracker.*` aliases listed in section 5. The examples import the package as plain `cuvslam`
(`euroc/track_euroc.py`, `euroc/dataset_utils.py`, `kitti/track_kitti*.py`) or as
`cuvslam as vslam` (`oak-d/run_stereo.py`, `realsense/*.py`) and use only top-level classes
(`Pose`, `Camera`, `Distortion`, `Rig`, `ImuCalibration`, `ImuMeasurement`, `Observation`)
plus the `Tracker.*` aliases: `Tracker.OdometryConfig`, `Tracker.OdometryMode`,
`Tracker.OdometryMultisensorSettings` (`run_vio.py` line 294), `Tracker.SlamConfig` and
`Tracker.SlamLocalizationSettings` (`track_kitti_slam.py` lines 185 and 208). None of them
touches `cuvslam.core` directly.

### 0.3 What changed on `main` after v17.0.0 (do NOT copy from main)

`main` `CHANGELOG.md` "Unreleased": *"`Tracker` now takes a required `Tracker::Mode`
(`cuvslam.Tracker.Mode` in Python) as its second constructor argument ... A `slam_config` is
now rejected in the odometry-only modes."* On `main`, `python/tracker.py` is deleted,
`Tracker` is a C++-bound class (`python/cuvslam2.cpp` line ~1110) with
`Tracker.Mode.{OdometryOnlyRealtime, OdometryOnlyOffline, OdometryWithSlamRealtime,
OdometryWithSlamOffline}` and signature `Tracker(rig, mode, odom_config=None, slam_config=None)`,
and `Odometry`/`Slam` are re-exported at top level. **v17.0.0 has none of that**:
`Tracker(rig, odom_config=None, slam_config=None)`. Always read sources at `?ref=v17.0.0`.

---

## 1. Free functions (`cuvslam.*`)

| call | returns / effect | source |
|---|---|---|
| `get_version() -> tuple[str, int, int, int]` | `(semver_string, major, minor, patch)`; test asserts the string starts with `f"{major}.{minor}.{patch}+"` (e.g. `"17.0.0+..."`) | cuvslam2.cpp `m.def("get_version")` |
| `set_verbosity(arg: int, /) -> None` | 0 (default) silent, 1 errors, 2 warnings, 3 info; positional-only and unnamed (the binding is `m.def("set_verbosity", [](int verbosity) {...})` with no `nb::arg`, so the stub prints `arg`) | `pycuvslam.pyi` line 16; cuvslam2.cpp line 167 |
| `warm_up_gpu() -> None` | optional CUDA/cusolver/cublas warm-up; **throws** if CUDA init fails | cuvslam2.cpp |
| `cuvslam.__version__` | `"17.0.0"` | `__init__.py` |

---

## 2. Data classes (all in `cuvslam.*`, defined in `cuvslam2.cpp`)

Array-typed attributes (`Pose.rotation/translation`, `Camera.size/principal/focal`,
`Slam.PoseGraphEdge.covariance`) are bound with `bind_array_accessors`: the **getter returns a
numpy view of the C++ `std::array`** (so `p.translation[0] = 9` really mutates the pose; test
`test_pose_element_access`), and the **setter accepts a 1-D ndarray of the exact dtype/shape
or any sequence of exactly N elements**, else `ValueError("Sequence must have exactly N
elements")` / `TypeError`. `list[float]`-typed attributes (`Distortion.parameters`,
`ImuMeasurement.*`, `Landmark.coords`, `covariance_xyz_rpy`) are **copies**: element
assignment silently does nothing (test `test_distortion_assignment`); assign the whole list.

### 2.1 `Pose`

```python
Pose()                                              # identity: rotation [0,0,0,1], translation [0,0,0]
Pose(rotation: Sequence[float], translation: Sequence[float])   # positional or keyword, this order
p.rotation    : np.ndarray float32 shape (4,)   # quaternion in (x, y, z, w) order  == scipy Rotation.as_quat()
p.translation : np.ndarray float32 shape (3,)   # metres
str(p) -> "(rotation=..., translation=...)";  repr -> "cuvslam.Pose(...)"
```

Docstring: *"Transformation from one frame to another. ... cuVSLAM uses OpenCV coordinate
system convention: x is right, y is down, z is forward."* Naming convention everywhere is
`a_from_b` = the transform that maps points expressed in frame `b` into frame `a`
(`rig_from_camera`, `rig_from_imu`, `world_from_rig`).

### 2.2 `Distortion` and `Distortion.Model`

```python
Distortion()                                        # model Pinhole, parameters []
Distortion(model: Distortion.Model, parameters: Sequence[float] = [])   # positional or keyword
d.model      : Distortion.Model
d.parameters : list[float]                          # copy; assign whole list
```

`Distortion.Model` (nanobind enum with `export_values()`, so `Distortion.Pinhole` etc. also exist):

| member | int | parameter count and **order** |
|---|---|---|
| `Pinhole` | 0 | none |
| `Fisheye` | 1 | 4: `(k1, k2, k3, k4)` equidistant, identical to OpenCV `cv::fisheye` / kalibr `pinhole-equi`; FOV must be < 180 deg |
| `Brown` | 2 | 5: `(k1, k2, k3, p1, p2)` radial x3 then tangential x2 (NOT OpenCV order) |
| `Polynomial` | 3 | 8: `(k1, k2, p1, p2, k3, k4, k5, k6)` = **first 8 OpenCV coefficients**, rational model `(1+k1r^2+k2r^4+k3r^6)/(1+k4r^2+k5r^4+k6r^6)` |

The binding does not validate the parameter count at construction time; the projection
formulas are in the class docstring (`pycuvslam.pyi` lines 65-140).

### 2.3 `Camera`

```python
Camera()                                            # size (0,0), focal (0,0), principal (0,0), Pinhole, identity, borders 0
Camera(*, size: Sequence[int], principal: Sequence[float], focal: Sequence[float],
       rig_from_camera: Pose = Pose(), distortion: Distortion = Distortion(),
       border_top: int = 0, border_bottom: int = 0, border_left: int = 0, border_right: int = 0)
# keyword-only: Camera((640,480),(320,240),(320,320)) -> TypeError
c.size            : np.ndarray int32   shape (2,)   # (width, height)
c.principal       : np.ndarray float32 shape (2,)   # (cx, cy) pixels
c.focal           : np.ndarray float32 shape (2,)   # (fx, fy) pixels
c.rig_from_camera : Pose                            # camera frame -> rig frame
c.distortion      : Distortion
c.border_top/bottom/left/right : int                # pixels ignored for feature selection, 0 = full frame
```

Docstring: *"For camera coordinate system, top left pixel has (0, 0) coordinate (y is down,
x is right)."* All cameras of a rig must have the same `size` (test `test_init_different_sizes`
-> `ValueError` at `Tracker`/`Odometry` construction); negative size -> `ValueError`.

### 2.4 `ImuCalibration`

```python
ImuCalibration()                                    # rig_from_imu identity; noise fields = C++ struct defaults (unspecified, not tested) -> always set them
ImuCalibration(*, rig_from_imu: Pose = Pose(), gyroscope_noise_density: float,
               gyroscope_random_walk: float, accelerometer_noise_density: float,
               accelerometer_random_walk: float, frequency: float)      # keyword-only; positional -> TypeError
```

| field | unit (from binding docstrings) |
|---|---|
| `rig_from_imu: Pose` | IMU frame -> rig frame |
| `gyroscope_noise_density: float` | `rad / (s * sqrt(Hz))` |
| `gyroscope_random_walk: float` | `rad / (s^2 * sqrt(Hz))` |
| `accelerometer_noise_density: float` | `m / (s^2 * sqrt(Hz))` |
| `accelerometer_random_walk: float` | `m / (s^3 * sqrt(Hz))` |
| `frequency: float` | Hz |

Values used by NVIDIA: RealSense D4xx (`camera_utils.py`): gyro nd `6.0673370376614875e-03`,
gyro rw `3.6211951458325785e-05`, accel nd `3.3621979208052800e-02`, accel rw
`9.8256589971851467e-04`, `frequency = 200`. EuRoC ADIS16448 (`sensor_imu0.yaml`): gyro nd
`1.6968e-04`, gyro rw `1.9393e-05`, accel nd `2.0000e-3`, accel rw `3.0000e-3`, `rate_hz: 200`.

### 2.5 `Rig`

```python
Rig(cameras: Sequence[Camera] = [], imus: Sequence[ImuCalibration] = [])   # positional or keyword
r.cameras : list[Camera]            # setter takes a whole list; r.cameras.append(...) does NOT persist (copy)
r.imus    : list[ImuCalibration]    # 0 or 1 entries; 2 -> ValueError at tracker construction
```

Camera **index = position in `rig.cameras`**; `track()` images must be in the same order.

### 2.6 `ImuMeasurement`

```python
ImuMeasurement()
ImuMeasurement(*, timestamp_ns: int, linear_accelerations: Sequence[float], angular_velocities: Sequence[float])  # keyword-only
m.timestamp_ns         : int          # nanoseconds, same clock as image timestamps
m.linear_accelerations : list[float]  # binding docstring: "Linear accelerations in m/s^2" (nothing more)
m.angular_velocities   : list[float]  # binding docstring: "Angular velocities in rad/s"
```

The binding (cuvslam2.cpp lines 386-387) states only the units. That the values are the raw
sensor output (specific force, gravity included) expressed in the IMU frame is what the
examples do, not a binding statement: `run_vio.py` lines 111-118 feed `get_motion_data()`
accel/gyro unmodified, and `dataset_utils.py` lines 218-221 feed the EuRoC CSV columns
unmodified; the frame is implied by `ImuCalibration.rig_from_imu` ("Transformation from IMU
coordinate frame to the rig coordinate frame", cuvslam2.cpp lines 306-307). EuRoC
`imu0/data.csv` is read by `read_csv_data` (`dataset_utils.py` lines 191-223): header line
skipped with `next(f)`, then by index `row[0]` timestamp, **`row[1:4]` gyro (rad/s), `row[4:7]`
accel (m/s^2)**. The EuRoC ASL header names (`#timestamp [ns],w_RS_S_x [rad s^-1],...,
a_RS_S_z [m s^-2]`) are an external dataset convention that the NVIDIA loader ignores; they
do not occur anywhere in the v17.0.0 tag.

### 2.7 Outputs: `PoseEstimate`, `PoseWithCovariance`, `PoseStamped`, `Observation`, `Landmark`

```python
class PoseEstimate:                      # returned by Odometry.track / Tracker.track()[0]
    timestamp_ns  : int                                    # read-only
    world_from_rig: PoseWithCovariance | None              # read-only; None == tracking failed for this frame
# NOTE: despite the C++ docstring mentioning "is_valid", there is NO is_valid attribute in Python; test for None.

class PoseWithCovariance:
    pose              : Pose            # read-only
    covariance_xyz_rpy: list[float]     # 36 floats, 6x6 row-major, order (x, y, z, rot about X, rot about Y, rot about Z), fixed-axis

class PoseStamped:                       # get_all_slam_poses(), get_loop_closure_poses()
    timestamp_ns: int;  pose: Pose       # both read-write

class Observation:                       # get_last_observations(cam)
    id: int; u: float; v: float; camera_index: int      # pixel coords, origin top-left

class Landmark:                          # get_last_landmarks()
    id: int; coords: list[float]        # 3 floats. Class doc says "world coordinate frame";
                                         # Odometry.get_last_landmarks doc says "3D points in the last camera frame". Verify before relying on it.
```

`PoseEstimate` docstring: *"The pose is world_from_rig ... The rig coordinate frame is
user-defined and depends on the extrinsic parameters of the cameras. The world coordinate
frame is an arbitrary 3D coordinate frame, corresponding to the rig frame at the start of
tracking."*

---

## 3. `cuvslam.core.Odometry`

```python
Odometry(rig: Rig, cfg: Odometry.Config = Odometry.Config())     # arg names: rig, cfg
```

Construction raises `ValueError` for: empty `rig.cameras`; cameras with different sizes;
non-positive sizes; `Inertial` mode without exactly one IMU; more than one IMU (any mode);
`Multisensor` without an RGB-D camera or an overlapping camera pair (message matches
`"at least one RGB-D camera.*or one camera pair"`); duplicate `depth_camera_ids`.

### 3.1 Enums

| enum | members (int) | doc |
|---|---|---|
| `Odometry.OdometryMode` | `Multicamera=0` | "multiple synchronized cameras, all cameras need to have frustum overlap with at least one another. Simplest case: stereo camera pair" (**default**) |
| | `Inertial=1` | "stereo camera and IMU measurements. A single stereo-camera with a single IMU is supported" |
| | `RGBD=2` | single RGB-D camera, RGB and depth aligned |
| | `Mono=3` | single camera, "accurate up to scale" |
| | `Multisensor=4` | EXPERIMENTAL cuNLS mode; any mix of RGB / RGB-D + optional IMU; pinhole cameras only; depth must be uint16 |
| `Odometry.MulticameraMode` | `Performance=0`, `Precision=1` (**default**), `Moderate=2` | speed/accuracy trade-off |

`export_values()` is called, so `Odometry.Inertial`, `Odometry.Precision`, ... exist as shortcuts.
TROUBLESHOOTING accuracy ranking (least to most): Mono -> RGBD -> Stereo Inertial -> Stereo ->
Multicamera; and *"IMU fusion does not add extra accuracy in scenarios where visual input
works ... The main reason to add IMU is robustness"*.

### 3.2 `Odometry.Config` (single keyword-only constructor; every field read-write)

| field | type | default (Python) | doc |
|---|---|---|---|
| `multicam_mode` | `MulticameraMode` | `Precision` | |
| `odometry_mode` | `OdometryMode` | `Multicamera` | |
| `use_gpu` | bool | `True` | |
| `async_sba` | bool | `True` | bundle adjustment on a background thread; **construction-time only** |
| `use_motion_model` | bool | `True` | predicts 2D observations; auto-retracks without it on failure |
| `use_denoising` | bool | `False` | input image denoising |
| `rectified_stereo_camera` | bool | `False` | "Enable if stereo cameras are rectified and horizontally aligned"; TROUBLESHOOTING: set True **only** for rectified, horizontally aligned, distortion-free images |
| `enable_observations_export` | bool | `True` (Python default; source comment "enable by default; in Python convenience is a priority") | needed for `get_last_observations` and `get_state()` |
| `enable_landmarks_export` | bool | `True` (Python default) | needed for `get_last_landmarks` |
| `enable_final_landmarks_export` | bool | `False` | needed for `get_final_landmarks`; "Also sets enable_landmarks_export and enable_observations_export" |
| `max_frame_delta_s` | float | `1.0` | "Maximum time difference between frames in seconds" |
| `debug_dump_directory` | str | `''` | |
| `debug_imu_mode` | bool | `False` | IMU-only rotation tracking for checking `rig_from_imu` (TROUBLESHOOTING step 9; note it calls the matrix `imu_from_rig`, lines 508/520, the inverse naming of the Python field `rig_from_imu` = IMU -> rig, cuvslam2.cpp lines 306-307; trust the binding and the examples) |
| `rgbd_settings` | `Odometry.RGBDSettings` | `RGBDSettings()` | |
| `multisensor_settings` | `Odometry.MultisensorSettings` | `MultisensorSettings()` | |

Defaults confirmed by `test_tracker_config_constructor`. Positional arguments -> `TypeError`.

`Odometry.RGBDSettings(*, depth_scale_factor=1.0, depth_camera_id=-1, enable_depth_stereo_tracking=False)`.
`Odometry.MultisensorSettings(*, depth_camera_ids=[], depth_scale_factor=1.0, enable_depth_stereo_tracking=True)`.

### 3.3 `Odometry.Internals` (per-frame expert overrides, stateless)

`Internals()` then set fields; pass as `track(..., internals=...)`; affects that call only
(DESIGN_CONCEPTS.md section 1). Fields: `num_desired_tracks`, `border_top/bottom/left/right`,
`box3_prefilter`, `ransac_filter`, `kf_survivor_from_last` (0-100 %),
`kf_max_timedelta_between_kfs_s` (int seconds; tests say the default is 60 s),
`kf_override_frame_selection: bool | None`, `vo_pnp_*` (lambda, huber, max_iteration,
recalculate_cov, filter_new_observations, max_obs_per_camera, point_z_thresh,
min_observations, cost_thresh), `inertial_stereo_pnp_*` (same set), `imu_pnp_robustifier_scale`,
`imu_pnp_max_iteration`, `imu_pnp_min_observations`, `icp_*`. Unstable API; we do not use it.

`Odometry.State` (from `get_state()`; consumed by `Slam.track`): `frame_id: int`,
`timestamp_ns: int`, `delta: Pose`, `keyframe: bool`, `warming_up: bool`,
`gravity: list[float] | None`, `observations: list[Observation]`, `landmarks: list[Landmark]`,
`context` (opaque).

### 3.4 `Odometry` methods

```python
track(timestamp: int, images: Sequence[ndarray], masks: Sequence[ndarray] | None = None,
      depths: Sequence[ndarray] | None = None, internals: Internals | None = None) -> PoseEstimate
register_imu_measurement(sensor_index: int, imu_measurement: ImuMeasurement) -> None   # sensor_index must be 0
get_last_observations(camera_index: int) -> list[Observation]      # needs enable_observations_export
get_last_landmarks() -> list[Landmark]                             # needs enable_landmarks_export
get_last_gravity() -> list[float] | None                           # Inertial / Multisensor+IMU only, else ValueError (test_get_gravity_non_inertial); None until estimated
get_final_landmarks() -> dict[int, list[float]]                    # needs enable_final_landmarks_export; "3D points in the odometry start frame"
get_state() -> Odometry.State
get_primary_cameras() -> list[int]                                  # cameras whose observations are always present; needed for Slam()
apply_expert_parameters(parameters: dict[str, str]) -> None         # keys sba.num_sba_frames, sba.num_inertial_sba_frames, sba.num_fixed_sba_frames, sba.num_sba_iterations, sba.robustifier_scale, sba.use_sba_winsorizer, sm.gravity_update_period_ns, sm.max_integration_time_ns, sm.min_num_kf_for_gravity, sm.min_time_period_ns, sm.max_time_period_ns
```

### 3.5 `track()` input contract (ndarray checks from `ImageFromNDArray` in cuvslam2.cpp, verified by `test_image_format.py`; timestamp/size checks are docstring-only, see below)

* `timestamp`: `int64` nanoseconds. Camera timestamps must be **strictly increasing**
  (`register_imu_measurement` docstring, `tracker.py` lines 154-155). The `Tracker.track`
  docstring (`tracker.py` line 121) says `ValueError` "if data checks fail (e.g. timestamps are
  out of order, images sizes are inconsistent, etc.)"; those two phrases are illustrative
  docstring text, not verified exception messages: the checks live inside `libcuvslam.so`
  (cuvslam2.cpp's `ImageFromNDArray`, lines 100-145, only checks ndim/dtype/stride/device),
  and no v17.0.0 test exercises them (`test_image_format.py` creates a fresh tracker per test
  and tracks once at timestamp 1000; `test_tracking.py`/`test_bindings.py` have no
  out-of-order or size-mismatch case). Match on exception type, never on message text.
* `images`: one entry per rig camera, **same order as `rig.cameras`**. Any array/tensor
  implementing the buffer or DLPack protocol (numpy, torch CPU or CUDA); all entries must be
  on the same device. An **empty array** (`size == 0`) means "no image for this camera" and is
  skipped (camera index still advances).
* Image array: dtype **`uint8`**; shape `(H, W)` (MONO), `(H, W, 1)` (MONO) or `(H, W, 3)`
  (3-channel, `Encoding::RGB`); channels/width contiguous; for CPU arrays the whole buffer
  must be contiguous (`stride(0)*shape(0) == size`), so a strided view such as `img[::2, ::2]`
  raises `ValueError`. `float32`, 1-D, 4-D, 2- or 4-channel -> `ValueError`. Use
  `np.ascontiguousarray` after any slicing. Image `(H, W)` must match `Camera.size`
  `(width, height)` (listed as a `ValueError` data check only in the `Tracker.track` docstring;
  engine-side, untested upstream, exact message unverified, see the `timestamp` bullet).
  DepthAI `GRAY8`/`RAW8` frames via `ImgFrame.getCvFrame()` are already `(H, W) uint8`.
  For 3-channel input the EuRoC loader converts RGB to **BGR** "for cuvslam compatibility";
  we use mono only, so this never matters for us.
* `masks`: optional; if given and non-empty its length must equal `len(images)` else
  `ValueError`; each mask `(H, W) uint8`. Semantics (KITTI masks example): **255 = excluded
  pixels (dynamic car), 0 = trackable**; the example passes `torch.zeros_like(...)` for the
  camera without a mask. Static borders should use `Camera.border_*` instead (TROUBLESHOOTING step 4).
* `depths`: optional, `(H, W) uint16`, only for `RGBD`/`Multisensor`.
* Returns `PoseEstimate`; on failure `world_from_rig is None` (examples `continue` on it).
  Docstring: *"In Inertial mode ... if visual odometry tracker fails to compute a pose, the
  function returns the position calculated from a user-provided IMU data. If after several
  calls ... not able to recover, then invalid pose will be returned. The track will output
  poses in the same coordinate system until a loss of tracking."* -> after a loss the world
  frame restarts; our code must count resets.
* All cameras must be synchronized ("almost synchronized" frames within **1 ms**).
* Thread-safety: `track` and `register_imu_measurement` must be called in non-decreasing
  timestamp order, externally serialized on the same tracker; buffer IMU samples captured on
  another thread and drain them from the camera thread (as `run_vio.py::register_imu_until`).

---

## 4. `cuvslam.core.Slam`

```python
Slam(rig: Rig, primary_cameras: Sequence[int], config: Slam.Config = Slam.Config())
```

### 4.1 `Slam.Config`

Two constructors: `Slam.Config()` (C++ defaults) and the keyword-only one. **Gotcha**: the
keyword constructor defaults `enable_reading_internals=True` ("Python convenience") but the
no-argument constructor leaves it at the C++ default `False` (`test_slam_config_default`).
Set it explicitly if `get_pose_graph()`/`get_slam_landmarks()` are needed.

| field | default | doc |
|---|---|---|
| `map_cache_path: str` | `''` | empty = in-memory map; else LMDB on disk (overwritten) |
| `use_gpu: bool` | `True` | |
| `sync_mode: bool` | `False` | True = loop closure/PGO run in the calling thread (deterministic; TROUBLESHOOTING "blocking mode" = `async_sba=False` + `sync_mode=True`) |
| `enable_reading_internals: bool` | `False` via `Config()`, `True` via kw ctor | |
| `planar_constraints: bool` | `False` | force motion on a horizontal plane |
| `gt_align_mode: bool` | `False` | |
| `map_cell_size: float` | `0.0` | 0 = auto from baseline |
| `max_landmarks_distance: float` | `100.0` | metres |
| `max_map_size: int` | `300` | poses in graph, 0 = unlimited |
| `throttling_time_ms: int` | `0` | min time between loop-closure events |
| `retention_time_ms: int` | `5000` | how far back `localize_in_map` timestamps may reach |

### 4.2 Other nested types

`Slam.DataLayer` enum: `Landmarks=0`, `Map=1`, `LoopClosure=2`. Because of `export_values()`
`Slam.Map` and `Slam.LoopClosure` are shortcuts, but `Slam.Landmarks` is the **class**
(collection: `timestamp_ns: int`, `landmarks: list[Slam.Landmark]`), so always write
`Slam.DataLayer.Landmarks` for the enum.
`Slam.Landmark`: `id: int`, `weight: float`, `coords: list[float]` (world frame).
`Slam.LocalizationSettings(*, horizontal_search_radius, vertical_search_radius, horizontal_step, vertical_step, angular_step_rads)` (metres / radians, all required, keyword-only).
`Slam.Metrics`: `timestamp_ns, lc_status: bool, pgo_status: bool, lc_selected_landmarks_count, lc_tracked_landmarks_count, lc_pnp_landmarks_count, lc_good_landmarks_count`.
`Slam.PoseGraph{nodes: list[PoseGraphNode], edges: list[PoseGraphEdge]}`, `PoseGraphNode{id, node_pose: Pose}`, `PoseGraphEdge{node_from, node_to, transform: Pose, covariance: float32[36]}`.

### 4.3 `Slam` methods

```python
track(state: Odometry.State, gt_pose: Pose | None = None) -> None   # call after each successful Odometry.track
get_pose() -> Pose                                                  # current SLAM world_from_rig; identity before first keyframe (CHANGELOG 17.0.0: "Split Slam::Track() into a void tracking call and Slam::GetPose()")
get_all_slam_poses(max_poses_count: int = 0) -> list[PoseStamped]   # re-optimised, smooth; may block on slam thread
save_map(folder_name: str, callback: Callable[[bool], None]) -> None          # async unless sync_mode; folder overwritten
localize_in_map(folder_name: str, timestamp_ns: int, guess_pose: Pose, images: Sequence[ndarray],
                settings: LocalizationSettings, start_cb: Callable[[], None],
                finish_cb: Callable[[Pose | None, str], None]) -> None
get_landmarks(layer: DataLayer) -> Landmarks
get_pose_graph() -> PoseGraph
get_slam_metrics() -> Metrics
get_loop_closure_poses() -> list[PoseStamped]                       # last 10
```

Callbacks are invoked with the GIL held from the SLAM thread (`nb::gil_scoped_acquire`).

---

## 5. `cuvslam.Tracker` (pure Python, `tracker.py`, v17.0.0 exact behaviour)

```python
class Tracker:
    # aliases (so user code never imports cuvslam.core):
    OdometryMode = Odometry.OdometryMode;  MulticameraMode = Odometry.MulticameraMode
    OdometryConfig = Odometry.Config;      OdometryRGBDSettings = Odometry.RGBDSettings
    OdometryMultisensorSettings = Odometry.MultisensorSettings;   Internals = Odometry.Internals
    SlamConfig = Slam.Config;  SlamMetrics = Slam.Metrics;  SlamLocalizationSettings = Slam.LocalizationSettings
    SlamDataLayer = Slam.DataLayer;  PoseGraphNode/PoseGraphEdge/PoseGraph = Slam.*;  SlamLandmark = Slam.Landmark;  SlamLandmarks = Slam.Landmarks

    def __init__(self, rig: Rig, odom_config: Odometry.Config | None = None, slam_config: Slam.Config | None = None):
        if odom_config is None: odom_config = Odometry.Config()
        if slam_config is not None:                       # SLAM needs exports
            odom_config.enable_observations_export = True  # (mutates the caller's config object)
            odom_config.enable_landmarks_export = True
        self.odom = Odometry(rig, odom_config)
        self.slam = Slam(rig, self.odom.get_primary_cameras(), slam_config) if slam_config is not None else None

    def track(self, timestamp: int, images, masks=None, depths=None, gt_pose: Pose | None = None,
              internals: Odometry.Internals | None = None) -> tuple[PoseEstimate, Pose | None]:
        pose_estimate = self.odom.track(timestamp, images, masks, depths, internals)
        slam_pose = None
        if self.slam and pose_estimate.world_from_rig:
            self.slam.track(self.odom.get_state(), gt_pose); slam_pose = self.slam.get_pose()
        return pose_estimate, slam_pose

    register_imu_measurement(sensor_index: int, imu_measurement: ImuMeasurement) -> None   # -> odom
    get_last_observations(camera_index: int) -> list[Observation]                          # -> odom
    get_last_landmarks() -> list[Landmark]                                                 # -> odom
    get_last_gravity() -> list[float] | None   # doc: "m/s^2 in the rig / VO frame; +Y is down (OpenCV), so approximately [0, +g, 0] when the rig is upright"
    get_final_landmarks() -> dict[int, list[float]]                                        # -> odom
    get_all_slam_poses(max_poses_count: int = 0) -> list[PoseStamped]                      # [] if no SLAM
    save_map(folder_name: str, callback) -> None                                           # callback(False) immediately if no SLAM
    localize_in_map(folder_name, timestamp, guess_pose, images, settings, start_cb, finish_cb)  # ValueError("SLAM is not enabled.") if no SLAM
    get_slam_landmarks(layer) -> Slam.Landmarks | None;  get_pose_graph() -> PoseGraph | None
    get_slam_metrics() -> Metrics | None;  get_loop_closure_poses() -> list[PoseStamped] | None
```

Accepted constructor spellings (test `test_init_arguments`, `test_tracking.py` lines 32-41):
`Tracker(rig)`, `Tracker(rig, cfg)`, `Tracker(rig=rig)`, `Tracker(rig=rig, odom_config=cfg)`,
`Tracker(rig=rig, slam_config=Tracker.SlamConfig())`; `Tracker()` -> `TypeError`. The
three-positional form `Tracker(rig, cfg, s_cfg)` is not in that test but is used by every
`test_slam_*` test in `test_tracking.py` (lines 319-414) and by `examples/kitti/track_kitti_slam.py`
line 186, and is valid per the `tracker.py` signature. There is **no** `get_slam_pose()`;
the SLAM pose is `track()[1]` or `tracker.slam.get_pose()`. `tracker.odom` / `tracker.slam`
are public attributes (tests use `tracker.odom.get_state()`).

---

## 6. Coordinate, unit and timing conventions (all quoted from source)

| item | convention | where stated |
|---|---|---|
| camera / rig / world axes | right-handed **OpenCV**: `+x right, +y down, +z forward` | `Pose` docstring; `track_euroc.py`/`track_kitti.py` log `rr.ViewCoordinates.RIGHT_HAND_Y_DOWN` |
| image pixel origin | `(0, 0)` = top-left pixel; `u` right, `v` down | `Camera` docstring, `Observation` |
| intrinsics | `focal=(fx, fy)`, `principal=(cx, cy)` in pixels at the tracked resolution; `size=(width, height)` | `Camera` |
| rig frame | user-defined by the extrinsics; **NVIDIA OAK example uses CAM_A (the colour camera) as rig origin** (`getCameraExtrinsics(CAM_B, CAM_A)`, `getCameraExtrinsics(CAM_C, CAM_A)`); EuRoC example makes cam0 the rig origin (identity) and re-expresses cam1/IMU relative to it | `run_stereo.py`, `dataset_utils.get_rig` |
| `rig_from_camera`, `rig_from_imu` | `T_rig_sensor`: maps sensor-frame points into the rig frame (`p_rig = R p_sensor + t`) | attribute docstrings |
| world frame | "= rig frame at the start of tracking" (first tracked frame); restarts after a tracking loss | `PoseEstimate` docstring, `track` docstring |
| `world_from_rig` | `T_world_rig` = rig pose in world; translation = rig origin in world | `PoseEstimate` |
| quaternion | `(x, y, z, w)`; matches `scipy.spatial.transform.Rotation.as_quat()` (examples call it directly) and the TUM trajectory format `qx qy qz qw` | `Pose` docstring |
| lengths | metres (OAK example divides DepthAI cm extrinsics by 100; EuRoC `T_BS` is metres) | `run_stereo.py` `CM_TO_METERS = 100` |
| timestamps | `int` **nanoseconds**, one monotonic clock for cameras and IMU; OAK example: `int(message_group.getTimestamp().total_seconds() * 1e9)`; RealSense: `int(frame.timestamp * 1e6)` (ms -> ns); EuRoC: CSV integers | examples |
| IMU | accel `m/s^2`, gyro `rad/s` (binding docstrings state only the units); "raw incl. gravity, in the IMU frame" is what the examples feed (`run_vio.py` lines 111-118, `dataset_utils.py` lines 218-221) with the frame implied by `rig_from_imu`; register with `sensor_index = 0` | `ImuMeasurement` docstrings (cuvslam2.cpp lines 386-387), examples, tests |
| gravity output | `get_last_gravity()` in the rig/VO frame, about `[0, +9.81, 0]` when upright (down = +y). **Docstring-only**: `test_tracking.py` lines 106-108 feed `[0.0, 9.81, 0.0]` as the synthetic accelerometer *input* (a real accelerometer at rest measures specific force, i.e. `[0, -9.81, 0]` with +Y down, so the test input does not corroborate the sign), and the output check (lines 123-127, `assertAlmostEqual(gravity[1], 9.81)`) is commented out under `# TODO: fix gravity`. No upstream test verifies the sign; the empirical check in section 8 (`debug_imu_mode` + `get_last_gravity()` on the real rig) is the acceptance test | `tracker.py` `get_last_gravity` docstring only (lines 180-186) |
| covariance | 6x6 row-major `(x, y, z, rotX, rotY, rotZ)` fixed-axis | `PoseWithCovariance` |
| ordering | `register_imu_measurement` and `track` in non-decreasing timestamp order; frame timestamps strictly increasing; IMU samples with `ts <= image ts` are registered **before** that image's `track`; samples older than the last tracker timestamp are dropped | `run_vio.py::register_imu_until`, `track_euroc.py` (merged stream sorted by timestamp) |
| frame gaps | `max_frame_delta_s` (1.0 s) is the max allowed gap; examples warn above 35 ms image jitter and 6 ms IMU jitter | `Odometry.Config`, examples |
| warm-up | OAK example skips the first `WARMUP_FRAMES = 60` sync groups before the first `track`; the source gives no rationale ("Warmup for specified number of frames"), auto-exposure settling is our own inference | `run_stereo.py` lines 36, 222-225 |

---

## 7. Exact call patterns from the official v17.0.0 examples

### 7.1 `examples/oak-d/run_stereo.py` (stereo-only, unrectified GRAY8, DepthAI v3)

`examples/oak-d/README.md` line 6 warns: *"The provided script has been developed and validated
on the OAK-D W Pro stereo camera ... Distortion models and order of cameras and its frames may
differ for other OAK-D models."* Why it still applies to our OAK-D S2: `getDistortionModel()`
returns `CameraModel.Perspective`, i.e. the 14-coefficient OpenCV rational model, so
`Distortion.Model.Polynomial` with `[:8]` is the right cuVSLAM model (not Fisheye), and
`getStereoLeftCameraId()` / `getStereoRightCameraId()` return `CAM_B` / `CAM_C` (cheatsheet
lines 249, 252-253), matching the example's CAM_B = left, CAM_C = right order.

```python
FPS = 30; RESOLUTION = (1280, 720); WARMUP_FRAMES = 60; IMAGE_JITTER_THRESHOLD_NS = 35 * 1e6
BORDER_TOP = 50; BORDER_BOTTOM = 0; BORDER_LEFT = 70; BORDER_RIGHT = 70; CM_TO_METERS = 100

def oak_transform_to_pose(oak_extrinsics):                  # 4x4 from calib.getCameraExtrinsics(src, dst)
    extrinsics_array = np.array(oak_extrinsics)
    rotation_matrix = extrinsics_array[:3, :3]
    translation_vector = extrinsics_array[:3, 3] / CM_TO_METERS
    rotation_quat = Rotation.from_matrix(rotation_matrix).as_quat()
    return vslam.Pose(rotation=rotation_quat, translation=translation_vector)

def set_cuvslam_camera(oak_params):
    cam = vslam.Camera()
    cam.distortion = vslam.Distortion(vslam.Distortion.Model.Polynomial, oak_params['distortion'])   # first 8 OpenCV coeffs
    cam.focal = (oak_params['intrinsics'][0][0], oak_params['intrinsics'][1][1])
    cam.principal = (oak_params['intrinsics'][0][2], oak_params['intrinsics'][1][2])
    cam.size = oak_params['resolution']                     # (width, height)
    cam.rig_from_camera = oak_transform_to_pose(oak_params['extrinsics'])
    cam.border_top, cam.border_bottom, cam.border_left, cam.border_right = BORDER_TOP, BORDER_BOTTOM, BORDER_LEFT, BORDER_RIGHT
    return cam

# calibration (before pipeline creation):
calib_data = device.readCalibration()
intrinsics = calib_data.getCameraIntrinsics(dai.CameraBoardSocket.CAM_B, resolution[0], resolution[1])   # scaled to 1280x720
extrinsics = calib_data.getCameraExtrinsics(dai.CameraBoardSocket.CAM_B, dai.CameraBoardSocket.CAM_A)   # left -> CAM_A (rig)
distortion = calib_data.getDistortionCoefficients(dai.CameraBoardSocket.CAM_B)[:8]
# (same for CAM_C)

cfg = vslam.Tracker.OdometryConfig(async_sba=False, enable_final_landmarks_export=True,
                                   enable_observations_export=True, rectified_stereo_camera=False)
tracker = vslam.Tracker(vslam.Rig(cameras), cfg)             # cameras = [left(CAM_B), right(CAM_C)]

# pipeline: dai.node.Camera x2 (CAM_B, CAM_C, sensorFps=FPS) -> requestOutput(RESOLUTION, type=dai.ImgFrame.Type.GRAY8)
#           -> dai.node.Sync with setSyncThreshold(timedelta(seconds=0.5 / FPS)), inputs "left"/"right"
message_group = sync_queue.get()
timestamp_ns = int(message_group.getTimestamp().total_seconds() * 1e9)
if frame_id > WARMUP_FRAMES:
    left_img = message_group["left"].getCvFrame(); right_img = message_group["right"].getCvFrame()
    odom_pose_estimate, _ = tracker.track(timestamp_ns, (left_img, right_img))
    odom_pose = odom_pose_estimate.world_from_rig
    if odom_pose is None: print(f"Tracking failed at frame {frame_id}"); continue
    trajectory.append(odom_pose.pose.translation)
    observations = tracker.get_last_observations(0)
```

### 7.2 `examples/realsense/run_vio.py` + `camera_utils.py` (stereo-inertial)

```python
cfg = vslam.Tracker.OdometryConfig(async_sba=False, enable_final_landmarks_export=True,
                                   enable_observations_export=True, debug_imu_mode=False,
                                   odometry_mode=vslam.Tracker.OdometryMode.Inertial,
                                   rectified_stereo_camera=True)       # RealSense IR pair is rectified & pinhole
rig = vslam.Rig()
rig.cameras = [get_rs_camera(left_intrinsics),                          # rig frame = left IR camera (identity)
               get_rs_camera(right_intrinsics, right_extrinsics)]       # right.rig_from_camera = extrinsics right->left
rig.imus = [get_rs_imu(camera_params['imu']['cam_from_imu'])]           # rs: frames[2].profile.get_extrinsics_to(frames[0].profile)
tracker = vslam.Tracker(rig, cfg)

def get_rs_imu(imu_extrinsics, frequency=200):
    imu = vslam.ImuCalibration()
    imu.rig_from_imu = vslam.Pose(rotation=Rotation.identity().as_quat(), translation=imu_extrinsics.translation)
    # ^ RealSense-only shortcut: camera_utils.rig_from_imu_pose (lines 90-102) deliberately drops
    #   rs_transform.rotation because the D4xx IMU axes are aligned with IR1. Do NOT copy for the
    #   OAK: use the full 3x3 from getImuToCameraExtrinsics (R = diag(1, -1, -1) on this unit,
    #   section 8), otherwise the IMU Y and Z axes are flipped.
    imu.gyroscope_noise_density = 6.0673370376614875e-03; imu.gyroscope_random_walk = 3.6211951458325785e-05
    imu.accelerometer_noise_density = 3.3621979208052800e-02; imu.accelerometer_random_walk = 9.8256589971851467e-04
    imu.frequency = frequency
    return imu

# IMU thread -> queue of ImuSample(timestamp_ns, accel xyz, gyro xyz); camera thread drains it:
def register_imu_until(tracker, imu_queue, pending_imu, timestamp_ns, last_tracker_timestamp_ns):
    ...
    while pending_imu and pending_imu[0].timestamp_ns <= timestamp_ns:
        sample = pending_imu.popleft()
        if last_tracker_timestamp_ns is not None and sample.timestamp_ns < last_tracker_timestamp_ns: continue
        imu_measurement = vslam.ImuMeasurement()
        imu_measurement.timestamp_ns = sample.timestamp_ns
        imu_measurement.linear_accelerations = sample.linear_accelerations
        imu_measurement.angular_velocities = sample.angular_velocities
        tracker.register_imu_measurement(0, imu_measurement)
        last_tracker_timestamp_ns = sample.timestamp_ns
    return last_tracker_timestamp_ns

current_timestamp = int(ir_left_frame.timestamp * 1e6)
images = (np.asanyarray(ir_left_frame.get_data()), np.asanyarray(ir_right_frame.get_data()))
last_tracker_timestamp = register_imu_until(tracker, imu_queue, pending_imu, current_timestamp, last_tracker_timestamp)
odom_pose_estimate, _ = tracker.track(current_timestamp, images)
last_tracker_timestamp = current_timestamp
if odom_pose_estimate.world_from_rig is None: continue
gravity = tracker.get_last_gravity()
```

Constants: `IMU_FREQUENCY_ACCEL = IMU_FREQUENCY_GYRO = 200`, `IMU_JITTER_THRESHOLD_NS = 6e6`,
`IMU_QUEUE_MAX_SIZE = 200 * 5`.

### 7.3 `examples/euroc/track_euroc.py` + `dataset_utils.py` (dataset replay; our dataset must load with this unmodified)

```python
cfg = cuvslam.Tracker.OdometryConfig(async_sba=False, enable_observations_export=True,
                                     enable_final_landmarks_export=True, rectified_stereo_camera=False,
                                     odometry_mode=cuvslam.Tracker.OdometryMode.Inertial)
rig = get_rig(sequence_path)            # sequence_path = <dataset>/mav0
tracker = cuvslam.Tracker(rig, cfg)
frames_metadata = prepare_frame_metadata_euroc(sequence_path, cfg.odometry_mode)   # stereo + imu entries sorted by timestamp
for fm in frames_metadata:
    if fm['type'] == 'imu':
        m = cuvslam.ImuMeasurement(); m.timestamp_ns = int(fm['timestamp'])
        m.linear_accelerations = np.asarray(fm['accel']); m.angular_velocities = np.asarray(fm['gyro'])
        tracker.register_imu_measurement(0, m); continue
    images = [load_frame(p) for p in fm['images_paths']]        # PIL 'L' -> (H,W) uint8
    odom_pose_estimate, _ = tracker.track(fm['timestamp'], images)
    if odom_pose_estimate.world_from_rig is None: continue
    odom_pose = odom_pose_estimate.world_from_rig.pose
    odom_trajectory.append([timestamp] + list(odom_pose.translation) + list(odom_pose.rotation))   # ts tx ty tz qx qy qz qw
```

Loader requirements (`dataset_utils.py`), i.e. **what our EuRoC-layout recorder must produce**:

* `mav0/cam0/data.csv`, `mav0/cam1/data.csv`: header line skipped, then `timestamp,filename`;
  both cameras must have the same number of rows and **identical timestamps row by row**
  (else `ValueError`); images at `mav0/camN/data/<filename>`.
* `mav0/imu0/data.csv`: header line skipped (`next(f)`), then columns read by index: 0 timestamp
  (int ns), 1-3 gyro (rad/s), 4-6 accel (m/s^2) (`read_csv_data`, `dataset_utils.py` lines
  191-223). The header text is ignored; EuRoC ASL-style names are an external convention.
* `mav0/{cam0,cam1,imu0}/sensor.yaml`; if `cam0/sensor_recalibrated.yaml` exists,
  `_get_sensor_paths` (`dataset_utils.py` lines 94-114) switches **all three** sensors to
  `sensor_recalibrated.yaml`, so `cam1/` and `imu0/` copies must then exist too (else
  `FileNotFoundError`). Either way each yaml needs:
  key starting with `T_` holding `{rows, cols, data: [16 floats row-major]}`, `resolution: [w, h]`,
  `intrinsics: [fu, fv, cu, cv]`, `distortion_coefficients`, IMU yaml: `rate_hz`,
  `gyroscope_noise_density`, `gyroscope_random_walk`, `accelerometer_noise_density`,
  `accelerometer_random_walk`.
* Distortion mapping: **default `sensor.yaml`** -> `Distortion.Model.Brown` with
  `coeffs[:2] + [0] + coeffs[2:]` (EuRoC radtan `k1 k2 p1 p2` -> `k1 k2 k3=0 p1 p2`);
  **`sensor_recalibrated.yaml`** -> `Distortion.Model.Fisheye` with the 4 coefficients as-is.
  There is no Polynomial path in this loader, and **the unmodified loader cannot consume the
  OAK 8-coefficient rational model** (Brown-4 or Fisheye-4 only). Do NOT write the OAK
  `[k1, k2, p1, p2]` as a Brown set with k3 dropped: the OAK `k1..k6` are only meaningful
  together (section 8). For this unit's CAM_B (`[23.206, -36.262, 0.00024, 0.00183, 15.942,
  22.588, -34.474, 14.630]`, `recordings/video_20260918_034017/calibration.json`) the true
  rational radial factor at r = 0.5 is `(1 + 5.80 - 2.27 + 0.25) / (1 + 5.65 - 2.15 + 0.23) = 1.013`,
  whereas truncated Brown `(k1 = 23.2, k2 = -36.3, k3 = 0)` gives `1 + 5.80 - 2.27 = 4.53`:
  a ~350 % radial error instead of 1.3 %. Options that keep NVIDIA's loader unmodified:
  **(a)** record host-undistorted/rectified mono frames (`cv2.initUndistortRectifyMap` with the
  full 14 OpenCV coefficients and the 1280x800 intrinsics, or DepthAI `StereoDepth` rectified
  outputs) and write `distortion_coefficients: [0, 0, 0, 0]` with the rectified intrinsics
  (loader -> Brown all-zero == pinhole; `rectified_stereo_camera=False` in `track_euroc.py`
  remains valid); or **(b)** numerically re-fit a plain 4-parameter Brown model (k1, k2, p1,
  p2; k3 fixed at 0) to the rational model over the image grid and write those 4 values.
  Either way keep the raw 8 Polynomial coefficients under an extra key that only our
  `08_cuvslam_track.py` reads (the NVIDIA loader ignores unknown keys). Source:
  `dataset_utils.py` lines 117-137; cuvslam2.cpp `Distortion` docstring (Polynomial = first 8
  OpenCV coefficients, rational).
* Extrinsics: `cam0.rig_from_camera` is set to identity **unconditionally** (line 159, both
  branches). With the default yaml (Brown model) `cam1.rig_from_camera = inv(T_BS_cam0) @ T_BS_cam1`,
  `imu.rig_from_imu = inv(T_BS_cam0) @ T_BS_imu` (`transform_to_cam0_reference`, lines 158-170);
  with recalibrated yaml (Fisheye model) the `T_*` are used as-is (already cam0-relative).
  So: **rig frame = cam0 = LEFT camera** for datasets, and `T_BS` in the yaml = `body_from_sensor`
  in metres.

### 7.4 `examples/kitti/track_kitti_slam.py` (SLAM save / localize)

```python
s_cfg = cuvslam.Tracker.SlamConfig(sync_mode=SLAM_SYNC_MODE)
tracker = cuvslam.Tracker(cuvslam.Rig(cameras), cfg, s_cfg)
loc_settings = cuvslam.Tracker.SlamLocalizationSettings(horizontal_search_radius=8., vertical_search_radius=2.,
                                                        horizontal_step=0.5, vertical_step=0.2, angular_step_rads=0.03)
_, _ = tracker.track(timestamp, init_images)
tracker.localize_in_map(map_path, timestamp, guess_pose, init_images, loc_settings, localization_start_cb, localization_finish_cb)
...
tracker.save_map(map_path, save_callback)      # then poll a flag set by save_callback(success)
```

### 7.5 `examples/requirements.txt` (v17.0.0 tag)

`numpy==2.2.4`, `opencv-python~=4.11.0`, `pillow==12.2.0`, `pyyaml==6.0.2`,
`rerun-sdk==0.33.1`, `scipy==1.14.1`. (The task brief's "numpy 2.2.6" is from `main`, where
the CHANGELOG bumps NumPy to 2.2.6 for CVEs.)

---

## 8. Mapping the OAK-D S2 calibration onto the cuVSLAM rig (cross-checked with `recordings/video_*/calibration.json`)

* `getDistortionCoefficients(sock)` returns 14 floats in OpenCV order
  `[k1, k2, p1, p2, k3, k4, k5, k6, s1, s2, s3, s4, tx, ty]` (cheatsheet line 252; the saved
  dump for CAM_B has 8 large rational-model values then four zeros then two small `tx, ty`).
  `[:8]` is exactly cuVSLAM `Distortion.Model.Polynomial` order. **Do not pass `[:5]` as
  Brown**: Brown order is `k1 k2 k3 p1 p2`, and the OAK values are a rational model whose
  `k1..k6` are only meaningful together.
* `getCameraIntrinsics(sock, w, h)` scales the 3x3 to the requested output size; `focal =
  (M[0][0], M[1][1])`, `principal = (M[0][2], M[1][2])`; `size = (w, h)` must equal the frame
  shape `(h, w)` we feed. Saved dump at 1280x800: CAM_B fx 799.13 fy 798.90 cx 649.47 cy 391.54;
  CAM_C fx 797.54 fy 797.47 cx 652.93 cy 368.24.
* `getCameraExtrinsics(src, dst)` returns the 4x4 that maps `src`-frame points into `dst`
  (`dst_from_src`), translation in **centimetres** (`lengthUnit: 1` = `dai.LengthUnit.CENTIMETER`;
  cheatsheet line 256). Hence `rig_from_camera = getCameraExtrinsics(cam, rig_cam)` with
  `t / 100`. Rig = CAM_A (NVIDIA example) or rig = CAM_B (EuRoC-style, then
  `left.rig_from_camera = identity`, `right.rig_from_camera = getCameraExtrinsics(CAM_C, CAM_B)`).
  Baseline check: `getCameraTranslationVector(CAM_B, CAM_C) = [-7.5, 0, ~0]` cm, calibrated
  7.4967 cm (`getBaselineDistance(useSpecTranslation=False)`).
* IMU: EEPROM `imuExtrinsics` has `rotationMatrix = diag(1, -1, -1)`, `translation =
  (-3.7948, -0.079, 0)` cm, `toCameraSocket = 0` (CAM_A). depthai 3.10.0 accessors (from
  `__doc__` in the project venv; the cheatsheet's line-948 probe names only
  `getCameraToImuExtrinsics`), both with signature `(cameraId, useSpecTranslation: bool = False,
  unit: LengthUnit = CENTIMETER) -> 4x4`: `getImuToCameraExtrinsics(cameraId)` = `cameraId_from_IMU`
  (docstring: "cameraId: Camera Id of the camera which will be considered as destination. To
  which Transformation matrix from the IMU will be found") and `getCameraToImuExtrinsics(cameraId)`
  = `IMU_from_cameraId` (its inverse). Hence **`rig_from_imu = getImuToCameraExtrinsics(rig_socket)`**
  with `t / 100`, never `getCameraToImuExtrinsics` (that is `imu_from_rig`). DepthAI chains
  through CAM_A itself: with `dai.CalibrationHandler(recordings/video_20260918_034017/calibration.json)`,
  `getImuToCameraExtrinsics(CAM_A)` returns `R = diag(1, -1, -1)`, `t = (-3.7948, -0.079, 0)` cm
  (identical to the EEPROM `imuExtrinsics`), `getCameraToImuExtrinsics(CAM_A)` returns the
  inverse (`t = (+3.7948, -0.079, 0)`), and `getImuToCameraExtrinsics(CAM_B)` returns
  `t = (-0.058, -0.113, 0.110)` cm. Verify the sign convention with `debug_imu_mode=True`
  (TROUBLESHOOTING step 9: rotate the camera; reported rotation must match. Note that
  TROUBLESHOOTING.md lines 508 and 520 call that matrix `imu_from_rig`, the inverse naming of
  the Python field `ImuCalibration.rig_from_imu` = "Transformation from IMU coordinate frame to
  the rig coordinate frame", cuvslam2.cpp lines 306-307; trust the binding docstring and the
  examples: `cam_from_imu` in `camera_utils.py` line 303, `inv(T_BS_cam0) @ T_BS_imu` in
  `dataset_utils.py` lines 167-177) and with `get_last_gravity()` pointing to `+y` when
  upright (this empirical check is the acceptance test: no upstream test verifies the gravity
  sign, section 6).
* IMU noise: EEPROM `imuCalibrationParams.noise.{accelerometer,gyroscope}.{x,y,z}` carry
  `noiseDensity / randomWalk / biasStability` for the BNO086 but in **undocumented units**
  (gyro `noiseDensity` about 0.4-0.6 cannot be rad/s/sqrt(Hz)); start from the RealSense
  constants in 7.2 (consumer MEMS at 200 Hz) and refine with an Allan-variance recording.
* Raw IMU from DepthAI: `ACCELEROMETER_RAW` in m/s^2, `GYROSCOPE_RAW` in rad/s (cheatsheet
  lines 578-579), 200 Hz verified (400 Hz also works); packet `getTimestamp()` is host-synced
  and comparable with `ImgFrame.getTimestamp()`; convert with `int(td.total_seconds() * 1e9)`
  or, better, `td.days * 86400 * 10**9 + td.seconds * 10**9 + td.microseconds * 1000` to avoid
  float rounding.
* Frames: `dai.node.Camera.requestOutput((w, h), type=dai.ImgFrame.Type.GRAY8)` on CAM_B /
  CAM_C through a `dai.node.Sync`; `getCvFrame()` -> `(h, w) uint8` contiguous. Left/right
  frames of one pair do **not** share a bit-identical timestamp (cheatsheet line 354); use the
  `MessageGroup.getTimestamp()` as the pair timestamp like the NVIDIA example.

---

## 9. Environment facts for `methods/cuvslam-vio/tools/check_env.py` / setup scripts

* `import cuvslam` requires: Linux x86_64 or aarch64, glibc >= 2.35, Python 3.10 for the
  `cp310` wheel, CUDA Toolkit 12.x installed (the wheel only bundles `libcudart.so.12.6.77`),
  `pyyaml`. `cuvslam.get_version()` and `cuvslam.warm_up_gpu()` are the two smoke tests;
  `warm_up_gpu()` raising means CUDA/cusolver/cublas are missing.
* `pip install cuvslam-17.0.0+cu12-cp310-cp310-manylinux_2_35_<arch>.whl` (README: "Only the
  combinations listed above are provided as pre-built wheels").
* Not importable on Windows or macOS (`Operating System :: POSIX :: Linux` only).
* Deterministic offline mode = `Odometry.Config(async_sba=False)` (+ `Slam.Config(sync_mode=True)`
  if SLAM); real-time mode = defaults (`async_sba=True`).

---

## 10. Fake module contract (`cuvslam_oak/fake_cuvslam.py`, used by pytest on Windows)

Goal: a drop-in double for the subset of `cuvslam` our code paths touch, so `rig.py`,
`tracking.py`, `dataset.py`, `datasets/07_record_dataset.py`, `methods/cuvslam-vio/08_cuvslam_track.py`
and the tests import one symbol table whether the real wheel is present or not. It must be
selectable explicitly (e.g. `tracking.load_cuvslam(fake=True)` or an env var) and must be
what the tests use on Windows; it must never be picked silently on Linux when the real wheel
imports.

### 10.1 Names the fake must expose (top level, mirroring `cuvslam/__init__.py`)

`get_version() -> ("17.0.0+fake", 17, 0, 0)`, `set_verbosity(int)`, `warm_up_gpu()`,
`__version__ = "17.0.0"`, classes `Pose, Distortion, Camera, ImuCalibration, ImuMeasurement,
Rig, PoseStamped, PoseWithCovariance, PoseEstimate, Observation, Landmark, Tracker`, and a
`core` namespace with `Odometry` and `Slam` (`utils` is not needed).

### 10.2 Data classes: same constructors, attribute names, dtypes and defaults as section 2

* `Pose()` = identity; `Pose(rotation, translation)` positional or keyword; `rotation`
  float32 `(4,)` `(x,y,z,w)`, `translation` float32 `(3,)`; setters accept ndarray or
  sequence and raise `ValueError("Sequence must have exactly N elements")` on wrong length;
  getters return an array whose element writes persist.
* `Distortion(model=Model.Pinhole, parameters=[])` with `Distortion.Model` an `enum.Enum`
  (`Pinhole=0, Fisheye=1, Brown=2, Polynomial=3`) and the `Distortion.Pinhole` etc. shortcuts.
* `Camera()` all-zero; `Camera(*, size, principal, focal, rig_from_camera=Pose(),
  distortion=Distortion(), border_top=0, border_bottom=0, border_left=0, border_right=0)`,
  keyword-only (positional -> `TypeError`); `size` int32 `(2,)`, `principal`/`focal` float32 `(2,)`.
* `ImuCalibration()` / keyword-only ctor with the five required numeric fields and
  `rig_from_imu=Pose()`.
* `ImuMeasurement()` / keyword-only `(timestamp_ns, linear_accelerations, angular_velocities)`;
  list-valued fields accept sequences and ndarrays.
* `Rig(cameras=[], imus=[])` with list attributes (assignment replaces).
* `PoseEstimate` with read-only `timestamp_ns` and `world_from_rig: PoseWithCovariance | None`;
  `PoseWithCovariance` read-only `pose` and `covariance_xyz_rpy` (36 floats);
  `PoseStamped(timestamp_ns, pose)`; `Observation(id, u, v, camera_index)`; `Landmark(id, coords)`.

### 10.3 `core.Odometry`

* Nested: `OdometryMode` (`Multicamera=0, Inertial=1, RGBD=2, Mono=3, Multisensor=4`),
  `MulticameraMode` (`Performance=0, Precision=1, Moderate=2`), `Config` (keyword-only, all
  15 fields of section 3.2 with the same defaults, plain attributes), `RGBDSettings`,
  `MultisensorSettings`, `Internals` (at least `num_desired_tracks`, `border_*`,
  `kf_override_frame_selection=None`), `State` (`frame_id, timestamp_ns, delta, keyframe,
  warming_up, gravity, observations, landmarks`).
* `Odometry(rig, cfg=Config())` validation (raise `ValueError` like the real one): no cameras;
  camera sizes differ; non-positive size; `Inertial` with `len(rig.imus) != 1`; `len(rig.imus) > 1`.
* `track(timestamp, images, masks=None, depths=None, internals=None)`: validate
  `len(images) == len(rig.cameras)`; each non-empty image is `uint8`, `(H,W)`/`(H,W,1)`/`(H,W,3)`,
  C-contiguous, `(H, W) == (size[1], size[0])`; masks length rule; timestamp strictly greater
  than the previous frame timestamp and `>=` the last IMU timestamp (else `ValueError`);
  returns a `PoseEstimate` whose `world_from_rig` comes from an injectable motion source
  (default: identity; tests can supply a callable `timestamp_ns -> Pose | None` to script a
  trajectory or a dropout, so `08_cuvslam_track.py`'s reset counting and TUM writer are
  testable). Record every call (`calls: list`) for assertions.
* `register_imu_measurement(sensor_index, m)`: `sensor_index != 0` -> `ValueError`; mode not
  `Inertial`/`Multisensor`-with-IMU -> `ValueError`; timestamp decreasing -> `ValueError`;
  append to `imu_log`.
* `get_last_observations(cam)` -> `[]` unless `enable_observations_export`; may return a few
  synthetic `Observation`s inside the image bounds. `get_last_landmarks()` -> `[]` /
  synthetic. `get_last_gravity()` -> `ValueError` when IMU fusion is off, `None` for the
  first N frames, then `[0.0, 9.81, 0.0]`. `get_final_landmarks()` -> `{}` unless
  `enable_final_landmarks_export`. `get_state()` -> `State`. `get_primary_cameras()` ->
  `[0]` (stereo). `apply_expert_parameters(dict)` -> no-op.

### 10.4 `core.Slam` (minimal, only if `tracking.py` exposes SLAM)

`Slam(rig, primary_cameras, config=Config())`; `Config` with the 11 fields/defaults of 4.1
(`enable_reading_internals=False` from the bare ctor); `DataLayer` enum; `LocalizationSettings`
keyword-only; `Metrics` zeros; `track(state, gt_pose=None)` no-op; `get_pose()` -> identity
(or the odometry pose if wired); `get_all_slam_poses(max_poses_count=0)` -> `list[PoseStamped]`
of tracked frames; `save_map(folder, cb)` creates the folder and calls `cb(True)` synchronously;
`localize_in_map(...)` calls `start_cb()` then `finish_cb(guess_pose, "")`; `get_landmarks(layer)`
-> empty `Landmarks`; `get_pose_graph()` -> empty; `get_slam_metrics()`; `get_loop_closure_poses()` -> `[]`.

### 10.5 `Tracker`

Reuse the v17.0.0 `tracker.py` logic verbatim (section 5) on top of the fake `Odometry`/`Slam`:
same aliases (`OdometryConfig`, `OdometryMode`, `MulticameraMode`, `SlamConfig`, ...), same
`__init__(rig, odom_config=None, slam_config=None)` including the export-flag mutation, same
`track` returning `(PoseEstimate, Pose | None)`, same pass-through methods, public `odom`/`slam`.

### 10.6 Contract tests worth writing against both the fake and (on Linux) the real wheel

Default values of `Odometry.Config()` / `Slam.Config()` (section 3.2 / 4.1), `Pose()` identity,
`Camera()` zeros, keyword-only `TypeError`s, `ValueError` on `uint8` violations, non-contiguous
input, `Tracker.track` tuple shape, `world_from_rig is None` path,
`get_last_gravity` `ValueError` in `Multicamera` mode, `get_version()[0].startswith("17.0.0+")`.
Out-of-order timestamps and image/`Camera.size` mismatch are docstring-only upstream (section
3.5): on the real wheel make those tests **observational** first (record the actual exception
type on Linux) rather than asserting `ValueError`, and never match on message text.
