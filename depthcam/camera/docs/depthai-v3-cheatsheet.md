# DepthAI 3.10.0 Python API cheat-sheet (verified on OAK-D S2)

Every name below was confirmed by introspecting the **installed** package
(`depthai==3.10.0`, Python 3.13 venv on Windows 11) and every behavioural
claim was run on the physical OAK-D S2 (`OAKDS2EXAMPLEID000`, RVC2, USB3) on
2026-09-18. Official v3 examples read: `Camera/{camera_output,camera_all,
camera_multiple_outputs,camera_roi_exposure_focus,camera_max_resolution}.py`,
`StereoDepth/{stereo,stereo_depth_filters}.py`, `PointCloud/{point_cloud,
point_cloud_visualizer}.py`, `RGBD/{rgbd,rgbd_o3d}.py`, `Align/custom_message_align.py`,
`IMU/{imu_gyroscope_accelerometer,imu_rotation_vector,imu_version}.py`,
`RecordReplay/{holistic_record,holistic_replay,record_video,replay_video}.py`,
`HostNodes/{display,host_camera,threaded_host_nodes}.py`, `Sync/sync.py`,
`Calibration/calibration_dump.py`, `Misc/MultiDevice/multi_device_frame_sync.py`
(all from `luxonis/depthai-core` `main`, `examples/python/`).

Conventions used by every snippet: `import depthai as dai`, pathlib for paths,
no `cv2.imshow`, bounded loops. Everything runs unchanged on Jetson Orin Nano
(Ubuntu, Python 3.10) as far as syntax goes; no Windows-only API is used.

Ground truth for this camera (see section 2 for how each value was read):

| item | value |
|---|---|
| `getDeviceName()` / `getProductName()` | `'OAK-D-S2'` / `'OAK-D-S2-AF'` |
| `getPlatform()` | `Platform.RVC2` (`getPlatformAsString()` -> `'RVC2'`) |
| `getUsbSpeed()` | `UsbSpeed.SUPER` (normal case; one of five back-to-back boots came up `HIGH` = USB 2 - see the USB2 check in section 2) |
| cameras | CAM_A IMX378 4056x3040 COLOR (autofocus), CAM_B/CAM_C OV9282 1280x800 MONO |
| stereo pair | left = CAM_B, right = CAM_C, baseline 7.5 cm (spec) / 7.4967 cm (calibrated) |
| IMU | BNO086, firmware 3.9.9 |
| bootloader | `getBootloaderVersion()` -> `None` (USB boot) |
| EEPROM | boardName `DM9098`, boardRev `R9M2E9`, productName `OAK-D-S2-AF`, version 7 |

---

## 1. Pipeline creation, lifecycle, device discovery, exceptions

### 1.1 Pipeline with implicit device (boots the camera in the constructor)

```python
import depthai as dai

with dai.Pipeline() as pipeline:              # == dai.Pipeline(createImplicitDevice=True)
    cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    q = cam.requestOutput((640, 400)).createOutputQueue(maxSize=4, blocking=False)
    pipeline.start()
    for _ in range(30):
        frame = q.get()                       # dai.ImgFrame
    pipeline.stop()
# __exit__ stops the pipeline and closes the implicit device
```

`dai.Pipeline.__init__` overloads (from `help`):
`__init__(self, createImplicitDevice: bool = True)` and `__init__(self, defaultDevice: dai.Device)`.

### 1.2 Pipeline with an explicit `dai.Device` (the device is closed by `pipeline.stop()`; it cannot be reused)

```python
device = dai.Device()                         # or dai.Device("OAKDS2EXAMPLEID000")
print(device.getDeviceName(), device.getUsbSpeed())
with dai.Pipeline(device) as pipeline:
    cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    q = cam.requestOutput((640, 400)).createOutputQueue()
    pipeline.start()
    q.get()
    pipeline.stop()
# NOTE: `pipeline.stop()` itself closes the device: device.isClosed() is True right after stop(), still inside the
# with-block, and also with no with-block at all (getAllAvailableDevices() is [] for a moment while the camera
# reboots, then X_LINK_UNBOOTED). A second `dai.Pipeline(device)` on the closed device does NOT raise a Python
# exception: the interpreter crashes natively inside `Camera.build()` ("Attempt to access invalid address", stack
# dai::node::Camera::build -> dai::DeviceBase::getConnectedCameraFeatures -> nanorpc call; exit code 139, no traceback).
# Create a fresh dai.Device() (or a fresh dai.Pipeline()) for every pipeline.
device.close()                                # harmless: the device is already closed
```

### 1.3 Lifecycle methods (all confirmed on `dai.Pipeline`)

`start()`, `isRunning() -> bool`, `stop()`, `run()` (blocks until stopped; needed when a
HostNode calls `sendProcessingToPipeline(True)`), `processTasks(waitForTasks=False, timeoutSeconds=-1.0)`,
`wait()`, `build()`, `isBuilt()`, `getDefaultDevice()`, `getDefaultDeviceProperties()`,
`enableHolisticRecord(recordConfig)`, `enableHolisticReplay(recordingPath: str)`,
`__enter__/__exit__` (context manager). `isHolisticRecordEnabled/ReplayEnabled` from the
C++ docs are **not** exposed in the Python binding.

### 1.4 Host-only pipeline (no device)

```python
p = dai.Pipeline(createImplicitDevice=False)  # constructor takes 0.00 s, camera stays UNBOOTED
p.getDefaultDevice()                          # -> None
# only host nodes are allowed:
# p.create(dai.node.StereoDepth) -> RuntimeError: Pipeline is host only, cannot create device node
```

### 1.5 Device discovery

```python
infos = dai.Device.getAllAvailableDevices()   # list[dai.DeviceInfo]
for di in infos:
    print(di.deviceId, di.name, di.state, di.protocol, di.platform, di.status, di.getDeviceId())
found, di = dai.Device.getDeviceById("OAKDS2EXAMPLEID000")        # (bool, DeviceInfo)
found, di = dai.Device.getDeviceByIdOrName("OAKDS2EXAMPLEID000")  # also accepts IP / USB name
found, di = dai.Device.getAnyAvailableDevice()                    # (bool, DeviceInfo)
dev = dai.Device("OAKDS2EXAMPLEID000")        # open a specific device by id (str)
dev = dai.Device(dai.DeviceInfo("OAKDS2EXAMPLEID000"))
dev = dai.Device(di, dai.UsbSpeed.SUPER)      # (DeviceInfo, maxUsbSpeed)
dev = dai.Device(dai.UsbSpeed.HIGH)           # force USB2 speed
```

`dai.DeviceInfo` fields: `deviceId`, `name`, `state` (`dai.XLinkDeviceState`), `protocol`
(`dai.XLinkProtocol`), `platform` (`dai.XLinkPlatform`), `status` (`dai.XLinkError_t`);
method `getDeviceId()`.
`dai.XLinkDeviceState` members: `X_LINK_ANY_STATE, X_LINK_BOOTED, X_LINK_UNBOOTED, X_LINK_BOOTLOADER,
X_LINK_FLASH_BOOTED, X_LINK_BOOTED_NON_EXCLUSIVE, X_LINK_GATE, X_LINK_GATE_BOOTED, X_LINK_GATE_SETUP`.
`dai.XLinkPlatform`: `X_LINK_ANY_PLATFORM, X_LINK_MYRIAD_2, X_LINK_MYRIAD_X, X_LINK_RVC3, X_LINK_RVC4`.
`dai.Platform`: `RVC2, RVC3, RVC4`.

### 1.6 Exceptions (verbatim, observed)

(a) **Requested device id does not exist** (bogus id, camera plugged in, tested with
`dai.Device("0000000000000000BOGUS")` and `dai.Device(dai.DeviceInfo("0000000000000000BOGUS"))`):

```
builtins.RuntimeError: 'Cannot find any device with given deviceInfo'
```
raised after **10.3 s** (default search time), preceded by the log line
`[host] [error] Searched, but no actual device found by given DeviceInfo: DeviceInfo(name=, deviceId=0000000000000000BOGUS, X_LINK_ANY_STATE, X_LINK_ANY_PROTOCOL, X_LINK_ANY_PLATFORM, X_LINK_SUCCESS)`.
`dai.Device.getDeviceById("...BOGUS")` returns `(False, DeviceInfo(name=, deviceId=, X_LINK_ANY_STATE, ...))` after ~1.6 s
(one USB enumeration pass, no 10 s search).

(b) **Device already in use by another process** (second process while the first one is
streaming a pipeline):

```
dai.Device()                      -> builtins.RuntimeError: 'No available devices'                       (after 13.7 s)
dai.Device('OAKDS2EXAMPLEID000')  -> builtins.RuntimeError: 'Cannot find any device with given deviceInfo' (after 11.5 s)
```
While busy, `dai.Device.getAllAvailableDevices()` and `getAllConnectedDevices()` return `[]`
and depthai logs `[depthai] [warning] skipping X_LINK_BOOTED device having name "1.2" (status: X_LINK_ERROR)`.
No `X_LINK_DEVICE_ALREADY_IN_USE` exception was ever raised on this Windows host.
Both exception types are plain `RuntimeError` (MRO `RuntimeError -> Exception`); `dai.XLinkError`
(`XLinkReadError`, `XLinkWriteError`) also derive from `RuntimeError` but were not seen here.

Two important observed corner cases:
* The constructor keeps **searching for ~10 s**; if the other process releases the camera inside
  that window the call **succeeds** (observed: `dai.Pipeline()` succeeded after 7.7 s / 11.0 s
  once the holder exited, and the caller then streamed normally).
* An idle `dai.Device()` (no running pipeline) in another process is only **sometimes** exclusive: in 2 of 7 attempts
  a second `dai.Device()` **took the camera over (after 2.7 s and 12.6 s)** and the first process's connection was
  silently lost (its later `close()` did not raise); in the other 5 attempts the second call raised
  `'No available devices'` after ~13-14 s and the holder kept a `X_LINK_BOOTED` connection whose `close()` succeeded.
  Do not rely on either outcome: never use a bare `dai.Device()` as a lock, and never assume a takeover will succeed.

Minimal busy-retry pattern (the repo's `camera/common.py` `open_device()` does the same job with `retries=3`, 3 s between
attempts, enumeration first, and no retry when a requested id is absent from a non-empty enumeration; the scripts pass
`retries=1` whenever `--device-id` is given, so an explicit id never retries even when nothing enumerates):

```python
import time
def open_device(retries=5, wait_s=10.0):
    for attempt in range(retries):
        try:
            return dai.Device()
        except RuntimeError as e:          # 'No available devices' / 'Cannot find any device with given deviceInfo'
            if attempt == retries - 1:
                raise
            print(f"device busy ({e}); retry {attempt + 1}/{retries} in {wait_s:.0f} s")
            time.sleep(wait_s)
```

### 1.7 Auto-reconnect and crash handling (observed)

If the firmware crashes (reproducible on this device: one CAM_A full-resolution stream plus three more CAM_A outputs runs the
RVC2 ISP out of memory - see section 3), the device prints `[Camera(2)] [critical] SIPP error : E_OUT_OF_MEM` /
`E_FINALISE_FAIL` and the host logs `[host] [warning] Closed connection`, `Attempting to reconnect. Timeout is 10000ms`,
`Reconnection successful`, and `[depthai] [error] Device with id ... has crashed. Crash dump logs are stored in:
C:\Users\<user>\AppData\Local\depthai\cache\crashdumps\...\crash_dump_<date>.tar.gz` (the tar.gz holds `crash_reports.json`
with the device-side prints). The crash dump of a previous run is reported again on the next boot. After a reconnect the same
pipeline crashes again straight away (a new crash dump every ~8-9 s, six in 45 s) until every queue raises
`dai.MessageQueue.QueueException`. `dai.Device.setMaxReconnectionAttempts(maxAttempts: int, callback=None)` (0 disables
reconnection) exists to tune this. Always run scripts under a timeout, because the reconnect loop can go on forever.

**Observed on OAK-D S2:** `dai.Device()` boot = 2.8-2.9 s; `dai.Pipeline()` constructor = 2.89 s
(it boots the device; `getAllAvailableDevices()` returns `[]` while the pipeline object is alive,
and `X_LINK_UNBOOTED` again after `del pipeline`); `pipeline.start()` = 0.1-0.3 s;
`with dai.Device(id) as device:` -> `device.isClosed()` is `False` inside, `True` after the block;
`DeviceInfo` before boot: `DeviceInfo(name=2.7, deviceId=OAKDS2EXAMPLEID000, X_LINK_UNBOOTED, X_LINK_USB_VSC, X_LINK_MYRIAD_X, X_LINK_SUCCESS)`;
`device.getDeviceInfo().state` while open = `XLinkDeviceState.X_LINK_BOOTED`.

---

## 2. Device introspection and calibration

```python
with dai.Device() as device:
    device.getDeviceName()              # 'OAK-D-S2'
    device.getProductName()             # 'OAK-D-S2-AF'
    device.getDeviceId()                # 'OAKDS2EXAMPLEID000'  (getMxId() works but prints DeprecationWarning)
    device.getPlatform()                # Platform.RVC2
    device.getPlatformAsString()        # 'RVC2'
    device.getUsbSpeed()                # UsbSpeed.SUPER
    device.getConnectedIMU()            # 'BNO086'
    device.getIMUFirmwareVersion()      # dai.Version -> str(...) '3.9.9'; .toStringSemver() '3.9.9'; .getBuildInfo() ''
    device.getEmbeddedIMUFirmwareVersion()  # 3.9.9 (firmware shipped in this depthai build)
    device.getBootloaderVersion()       # None when booted over USB without bootloader
    device.getConnectedCameras()        # [CAM_A, CAM_B, CAM_C]
    device.getConnectedCameras(dai.CameraSensorType.MONO)   # [CAM_B, CAM_C]
    device.getCameraSensorNames()       # {CAM_C: 'OV9282', CAM_A: 'IMX378', CAM_B: 'OV9282'}
    device.getStereoPairs()             # [StereoPair(left=CAM_B, right=CAM_C, baseline=7.4966, isVertical=False)]
    device.getAvailableStereoPairs()    # same list, filtered by calibration
    device.isEepromAvailable()          # True
    device.hasGPU(), device.isGpuStereoSupported(), device.isNeuralDepthSupported()  # False, False, False on RVC2
    device.getIrDrivers()               # [] (S2 has no IR projector)
    device.getConnectionInterfaces()    # [connectionInterface.USB]
    device.getChipTemperature().average # ~38 C idle
```

USB2 fallback detection:

```python
speed = device.getUsbSpeed()                       # dai.UsbSpeed: UNKNOWN, LOW, FULL, HIGH, SUPER, SUPER_PLUS
if speed not in (dai.UsbSpeed.SUPER, dai.UsbSpeed.SUPER_PLUS):
    print(f"WARNING: running at {speed} (USB2 fallback) - expect ~10x less bandwidth")
# forcing USB2 for a test: dai.Device(dai.UsbSpeed.HIGH) -> getUsbSpeed() == UsbSpeed.HIGH (observed)
# observed 2026-09-18: 4 of 5 back-to-back boots came up SUPER, one came up HIGH (USB 2) with no cable change -
# always check getUsbSpeed() after every open.
```

### 2.1 `getConnectedCameraFeatures()` -> `list[dai.CameraFeatures]`

Fields: `socket`, `sensorName`, `name`, `width`, `height`, `orientation` (`dai.CameraImageOrientation`),
`hasAutofocus`, `hasAutofocusIC`, `supportedTypes` (`list[dai.CameraSensorType]`), `configs`
(`list[dai.CameraSensorConfig]` with `width, height, minFps, maxFps, type, fov`), `calibrationResolution`.

```python
for cf in device.getConnectedCameraFeatures():
    print(cf.socket, cf.sensorName, cf.width, cf.height, cf.hasAutofocus, cf.supportedTypes,
          [(c.width, c.height, c.maxFps) for c in cf.configs])
```

### 2.2 Calibration (`device.readCalibration()` -> `dai.CalibrationHandler`)

```python
calib = device.readCalibration()          # also: readCalibration2() (throws if none), readCalibrationOrDefault(), device.getCalibration()
calib.getBaselineDistance()                                  # 7.5 -> CENTIMETERS by default (spec translation)
calib.getBaselineDistance(unit=dai.LengthUnit.MILLIMETER)    # 75.0
calib.getBaselineDistance(useSpecTranslation=False)          # 7.4967 (calibrated)
# signature: getBaselineDistance(cam1=CameraBoardSocket.CAM_C (2), cam2=CameraBoardSocket.CAM_B (1), useSpecTranslation=True, unit=LengthUnit.CENTIMETER) -> float
# (help() prints the defaults as the raw enum values 2 / 1; the docstring's "RIGHT"/"LEFT" are deprecated aliases of CAM_C/CAM_B
#  that raise "DeprecationWarning: RIGHT is deprecated, use CAM_C or address camera by name instead." on access)
calib.getStereoLeftCameraId(), calib.getStereoRightCameraId()   # CAM_B, CAM_C
M = calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, 640, 400)   # 3x3 list[list[float]] scaled to 640x400
M, w, h = calib.getDefaultIntrinsics(dai.CameraBoardSocket.CAM_A)     # matrix + calibration resolution (4056x3040)
calib.getDistortionCoefficients(dai.CameraBoardSocket.CAM_A)  # 14 floats [k1,k2,p1,p2,k3,k4,k5,k6,s1..s4,tx,ty]
calib.getDistortionModel(dai.CameraBoardSocket.CAM_A)         # CameraModel.Perspective
calib.getFov(dai.CameraBoardSocket.CAM_A)                     # 68.8 (spec); getFov(sock, False) -> 67.0 from intrinsics
calib.getLensPosition(dai.CameraBoardSocket.CAM_A)            # 130 (AF lens position at calibration)
calib.getCameraExtrinsics(dai.CameraBoardSocket.CAM_B, dai.CameraBoardSocket.CAM_C)   # 4x4, translation in cm
calib.getCameraTranslationVector(dai.CameraBoardSocket.CAM_B, dai.CameraBoardSocket.CAM_C)  # [-7.5, 0.0, ~0]
calib.getCameraToImuExtrinsics(dai.CameraBoardSocket.CAM_A)   # 4x4
ee = calib.getEepromData()                                    # dai.EepromData
calib.eepromToJson()                                          # dict (as in Calibration/calibration_dump.py)
```

`dai.EepromData` fields: `version, boardName, boardRev, boardConf, boardCustom, hardwareConf, productName,
deviceName, batchName, batchTime, boardOptions, cameraData (dict socket -> dai.CameraInfo),
stereoRectificationData (.leftCameraSocket/.rightCameraSocket), imuExtrinsics (.toCameraSocket, .translation,
.specTranslation), imuCalibrationParams, housingExtrinsics, miscellaneousData, stereoUseSpecTranslation,
stereoEnableDistortionCorrection, verticalCameraSocket`.
`dai.CameraInfo` fields: `width, height, intrinsicMatrix, distortionCoeff, extrinsics, cameraType, specHfovDeg`
(there is **no** `lensPosition` on CameraInfo; use `CalibrationHandler.getLensPosition`).
`dai.LengthUnit`: `METER, CENTIMETER, MILLIMETER, INCH, FOOT, CUSTOM`.

**Observed on OAK-D S2:** intrinsics CAM_A@4056x3040 fx=3065.9 fy=3066.0 cx=1995.9 cy=1471.1;
CAM_A@1920x1080 fx=1451.3 cx=944.8; CAM_A@640x400 fx=483.8 cx=314.9 cy=192.3; CAM_B@1280x800 fx=799.1 cx=649.5 cy=391.5;
CAM_B@640x400 fx=399.6; CAM_C@1280x800 fx=797.5. Mono FOV 71.9 (spec) / 77.4 (intrinsics). EEPROM version 7,
boardConf `nIR-C21M05-00`, hardwareConf `F0-FV00-BC001`, batchTime 1764034136, `imuExtrinsics.toCameraSocket = CAM_A`,
`stereoUseSpecTranslation=False`, `stereoEnableDistortionCorrection=True`. Camera features: CAM_A configs
1920x1080@60, 3840x2160@42, 4056x3040@30, 1352x1012@52, 2024x1520@85 (COLOR); CAM_B/C 1280x720@143, 1280x800@129.6,
640x400@255.7 (MONO, plus COLOR-typed duplicates); `supportedTypes` CAM_A `[COLOR]`, CAM_B/C `[MONO, COLOR]`;
`hasAutofocus=True` only on CAM_A.

---

## 3. Camera node

Signatures (from `help(dai.node.Camera)`):

* `build(boardSocket: dai.CameraBoardSocket = AUTO, sensorResolution: tuple[int,int] | None = None, sensorFps: float | None = None) -> Camera`
  (overloads with `replayNode: dai.node.ReplayVideo` exist for mocking)
* `requestOutput(size: tuple[int,int], type: dai.ImgFrame.Type | None = None, resizeMode: dai.ImgResizeMode = CROP, fps: float | None = None, enableUndistortion: bool | None = None, alphaScaling: float | None = None) -> dai.Node.Output`
  (second overload: `requestOutput(capability: dai.Capability, onHost: bool)` - annotated with the base class; a
  `dai.ImgFrameCapability` is accepted as a `Capability` subclass. "Mainly to be used by custom node writers.")
* `requestFullResolutionOutput(type: dai.ImgFrame.Type | None = None, fps: float | None = None, useHighestResolution: bool = False) -> dai.Node.Output`
* `requestIspOutput(...)`, `initialControl` (dai.CameraControl), `inputControl` (Input), `getBoardSocket()`.
* `dai.ImgResizeMode`: `CROP, STRETCH, LETTERBOX`.

```python
with dai.Pipeline() as p:
    camA = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    camB = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    rgb  = camA.requestOutput((640, 400), dai.ImgFrame.Type.BGR888i, fps=30)          # BGR for OpenCV
    rgbU = camA.requestOutput((1280, 720), resizeMode=dai.ImgResizeMode.STRETCH, enableUndistortion=True)
    mono = camB.requestOutput((640, 400), dai.ImgFrame.Type.GRAY8)
    full = camA.requestFullResolutionOutput()                                            # 4056x3040 YUV420p
    qs = {n: o.createOutputQueue(maxSize=4, blocking=False) for n, o in
          {"rgb": rgb, "rgbU": rgbU, "mono": mono, "full": full}.items()}
    p.start()
    for name, q in qs.items():
        f = q.get()
        img = f.getCvFrame()          # numpy BGR (H,W,3) or gray (H,W); always a copy
        print(name, f.getType(), f.getWidth(), f.getHeight(), img.shape, img.dtype,
              f.getSequenceNum(), f.getTimestamp(), f.getTimestampDevice())
    p.stop()
```

`dai.ImgFrame` getters (all confirmed): `getCvFrame()`, `getFrame()` (raw planar data, e.g. (600,640) for
640x400 YUV420p/NV12), `getType()`, `getWidth()`, `getHeight()`, `getTimestamp() -> datetime.timedelta`
(host-synced, comparable with `dai.Clock.now()`), `getTimestampDevice()` (device monotonic), `getTimestampSystem()`,
`getSequenceNum()`, `getInstanceNum()` (socket index), `getFps()`, `getExposureTime() -> timedelta`,
`getSensitivity()` (ISO), `getLensPosition()` (0..255, -1 when N/A), `getLensPositionRaw()` (0..1),
`getColorTemperature()`, `getTransformation() -> dai.ImgTransformation` (`getSize()`, `getSourceSize()`,
`getIntrinsicMatrix()`, `isAlignedTo(other)`, `remapPointTo(...)`). Setters for host-made frames:
`setCvFrame(array, type)`, `setFrame(array)`, `setWidth/setHeight/setType`, `setTimestamp(timedelta)`, `setSequenceNum(int)`.
`dai.ImgFrame.Type` members: `YUV422i, YUV444p, YUV420p, YUV422p, YUV400p, RGBA8888, RGB161616, RGB888p, BGR888p, RGB888i,
BGR888i, RGBF16F16F16p, BGRF16F16F16p, RGBF16F16F16i, BGRF16F16F16i, GRAY8, GRAYF16, LUT2, LUT4, LUT16, RAW16, RAW14, RAW12,
RAW10, RAW8, PACK10, PACK12, YUV444i, NV12, NV21, BITSTREAM, HDR, RAW32, NONE`.

Which types are valid where (observed, all delivered frames): CAM_A (color): default = `YUV420p`;
`NV12`, `BGR888i`, `RGB888i`, `GRAY8` all work. CAM_B/CAM_C (mono): default = `RAW8` (single channel);
`GRAY8`, `RAW8`, `NV12` and even `BGR888i` (gray replicated to 3 channels) all work. So the `type`
argument is a conversion request, not restricted by sensor type; `getCvFrame()` always yields BGR
for 3-channel types and (H,W) uint8 for GRAY8/RAW8.

### 3.1 Autofocus / manual focus (CAM_A is the AF variant)

```python
ctrlQ = camA.inputControl.createInputQueue()     # dai.InputQueue (send / trySend)
ctrl = dai.CameraControl()
ctrl.setManualFocus(60)                          # lens position 0..255
ctrlQ.send(ctrl)
ctrl = dai.CameraControl()
ctrl.setAutoFocusMode(dai.CameraControl.AutoFocusMode.CONTINUOUS_VIDEO)   # OFF, AUTO, MACRO, CONTINUOUS_VIDEO, CONTINUOUS_PICTURE, EDOF
ctrl.setAutoFocusTrigger()
ctrlQ.send(ctrl)
# read back: frame.getLensPosition(); other CameraControl helpers: setManualFocusRaw(0..1),
# setAutoFocusRegion(x, y, w, h), setAutoFocusLensRange(inf, macro), setManualExposure(us, iso),
# setAutoExposureEnable(), setAutoExposureLock(bool), setAutoExposureRegion(...)
# camA.initialControl.setManualFocus(120) applies before start.
```

**Observed on OAK-D S2:** `requestOutput((640,400))` on CAM_A -> `Type.YUV420p`, `getCvFrame().shape == (400,640,3)`
uint8, `getFrame().shape == (600,640)`; source sensor mode chosen automatically = 1352x1012 for 640x400 / 1280x720
requests; `requestFullResolutionOutput()` -> CAM_A 4056x3040 `YUV420p` (`getFrame()` (4560,4056)), CAM_C 1280x800 `RAW8`.
Mono default 640x400 -> `Type.RAW8`, `(400,640)` uint8, `getLensPosition() == -1`. Timestamps: `getTimestamp()` is a
`timedelta` like `22:22:31.970422`, `getTimestampDevice()` ~`0:00:01.6` after boot; left/right mono frames with the same
`getSequenceNum()` are hardware-synchronised to within tens of microseconds but **not identical** (observed 11-16 us apart on
both `getTimestamp()` and `getTimestampDevice()`; `==` was False for all 29 pairs checked) - pair them by sequence number or
with `dai.node.Sync`, never with `timestamp ==`. Requesting `fps=15` on one CAM_A output made **all** CAM_A outputs
report `getFps() == 15.0` (one sensor, one fps). Autofocus: `setManualFocus(60)` -> `getLensPosition()==60`
(`raw 0.235`), `setManualFocus(200)` -> 200 (`0.784`), `CONTINUOUS_VIDEO` + trigger -> settled at 91-95; the lens
sometimes stays at the previous manual position for ~3 s before it starts moving (one run sampled every 0.5 s:
60,60,60,60,60,60,95,95; another run: 95 within 0.5 s), so poll for several seconds before concluding AF is not working.
Auto-exposure needs ~1 s: the first frames after `start()` are taken at maximum gain/exposure (ISO ~1550-1599, 30 ms)
before AE converges - in a bright scene they saturate (all 255 on mono), in a dim scene they are merely wrong-exposed
(observed mono seq 0: min 0 / max 106-125 / mean 49; the first CAM_A frame was all black) - either way
**sleep ~1 s and `q.tryGetAll()` to drain before measuring anything** (the official
`point_cloud_visualizer.py` does exactly this). **The output budget is RVC2 ISP/SIPP memory, not a number of outputs.**
Verified fine: the 4-output snippet above (CAM_A 640x400 BGR888i + 1280x720 STRETCH undistorted + CAM_B GRAY8 + CAM_A
4056x3040 full-res) and 7 x 640x400 outputs across all three sockets with no full-res stream (CAM_A default / BGR888i@15 /
GRAY8 / NV12 + CAM_B default / BGR888i + CAM_C default). Verified crashing, every time: one CAM_A full-res stream **plus three
more CAM_A 640x400 outputs** (default, BGR888i@15, GRAY8) plus three mono outputs - the firmware dies right after `start()`
with `[Camera(2)] [critical] SIPP error : E_OUT_OF_MEM` / `E_FINALISE_FAIL`, reconnects and crashes again every ~8-9 s (six
crash dumps in 45 s) until every queue raises `QueueException` (section 1.7). With a full-res CAM_A stream keep the other
CAM_A outputs to one or two, and always run under a timeout because the crash/reconnect loop repeats.

---

## 4. StereoDepth

Signatures (3.10.0):

* `build(left: dai.Node.Output, right: dai.Node.Output, presetMode: dai.node.StereoDepth.PresetMode = DEFAULT) -> StereoDepth`
* `build(autoCreateCameras: bool, presetMode = DEFAULT, size: tuple[int,int] = (640, 400), fps: float | None = None) -> StereoDepth`
* `dai.node.StereoDepth.PresetMode` members actually present: `FAST_ACCURACY(0), FAST_DENSITY(1), DEFAULT(2), FACE(3),
  HIGH_DETAIL(4), ROBOTICS(5), DENSITY(6), ACCURACY(7)`.
* Node setters: `setDefaultProfilePreset(mode)`, `setLeftRightCheck(bool)`, `setSubpixel(bool)`,
  `setSubpixelFractionalBits(3|4|5)`, `setExtendedDisparity(bool)`, `setRectification(bool)`,
  `setDepthAlign(dai.StereoDepthConfig.AlgorithmControl.DepthAlign)` **or** `setDepthAlign(dai.CameraBoardSocket)`,
  `setOutputSize(w, h)`, `setOutputKeepAspectRatio(bool)`, `setRectifyEdgeFillColor(int)`, `setAlphaScaling(float)`,
  `setBaseline(cm)`, `setFocalLength(px)`, `enableDistortionCorrection(bool)`, `useHomographyRectification(bool)`,
  `setPostProcessingHardwareResources(shaves, slices)`, `setRuntimeModeSwitch(bool)`, `setInputResolution(w, h)`.
* **There is no `StereoDepth.setMedianFilter` / `setConfidenceThreshold` on the node** in 3.10.0 - use
  `stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)` and `initialConfig.setConfidenceThreshold(0..255)`
  (**higher = stricter** in v3: 255 is the most confident value; presets use 15 or 55 - see the Observed note).
* `dai.MedianFilter` (== `dai.filters.params.MedianFilter`): `MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7`.
* `dai.StereoDepthConfig.AlgorithmControl.DepthAlign`: `AUTO, RECTIFIED_RIGHT, RECTIFIED_LEFT, CENTER, RIGHT, LEFT`.
* `dai.StereoDepthConfig` getters: `getMaxDisparity()`, `getSubpixel()`, `getSubpixelFractionalBits()`, `getExtendedDisparity()`,
  `getLeftRightCheck()`, `getConfidenceThreshold()`, `getMedianFilter()`, `getDepthUnit()`; setters `setDepthUnit(dai.LengthUnit)`,
  `setDisparityShift`, `setNumInvalidateEdgePixels`, `setLeftRightCheckThreshold`; sub-structs `algorithmControl`
  (`.depthAlign`, `.disparityShift`), `postProcessing` (`.decimationFilter.decimationFactor/.decimationMode`,
  `.spatialFilter.enable`, `.temporalFilter.enable`, `.speckleFilter.enable`, `.thresholdFilter.minRange/.maxRange`,
  `.median`, `.filteringOrder`).
* Inputs: `left`, `right`, `inputConfig`, `inputAlignTo`. Outputs: `depth`, `disparity`, `syncedLeft`, `syncedRight`,
  `rectifiedLeft`, `rectifiedRight`, `confidenceMap`, `outConfig` (+ debug outputs).

```python
import numpy as np
with dai.Pipeline() as p:
    left  = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    right = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
    stereo = p.create(dai.node.StereoDepth).build(
        left=left.requestOutput((640, 400), fps=30),
        right=right.requestOutput((640, 400), fps=30),
        presetMode=dai.node.StereoDepth.PresetMode.DEFAULT)
    stereo.setLeftRightCheck(True)
    stereo.setSubpixel(True)                  # docstring says "incompatible with extended disparity" - stale: the FACE/HIGH_DETAIL
                                              # presets enable both and it runs on RVC2. With 3-bit subpixel + extended (DEFAULT +
                                              # setExtendedDisparity(True)) the node logs "Maximum disparity value 1520 exceeds ... 1024
                                              # by median filter. Disabling median filter!"; the 5-bit FACE/HIGH_DETAIL presets have the
                                              # median filter off anyway
    stereo.setSubpixelFractionalBits(3)       # 3 (DEFAULT/ROBOTICS presets), 4 or 5 (config-object default is 5); median filter only with 3
    stereo.setExtendedDisparity(False)
    stereo.setRectification(True)
    stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
    stereo.initialConfig.setConfidenceThreshold(55)      # 255 = most confident; higher threshold = FEWER valid pixels (presets: 15 / 55; 255 -> empty depth)
    stereo.initialConfig.postProcessing.decimationFilter.decimationFactor = 1   # DEFAULT preset uses 2 (-> 320x200)
    qDepth = stereo.depth.createOutputQueue(maxSize=4, blocking=False)
    qDisp  = stereo.disparity.createOutputQueue(maxSize=4, blocking=False)
    p.start()
    import time; time.sleep(1.5); qDepth.tryGetAll(); qDisp.tryGetAll()     # let AE settle
    depth = qDepth.get().getFrame()           # uint16, millimetres, 0 == invalid
    disp  = qDisp.get().getFrame()            # uint16 (RAW16) on this build
    valid = depth[depth > 0]
    dispVis = (disp * (255.0 / stereo.initialConfig.getMaxDisparity())).astype(np.uint8)  # as stereo_depth_filters.py
    colour = dai.utility.colorizeDepthFrame(qDepth.get()).getCvFrame()       # BGR888i, invalid -> black
    p.stop()
```

Depth aligned to the colour camera - two ways, the second is the one that reports `isAlignedTo() == True`:

```python
camA = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
rgb = camA.requestOutput((640, 400), dai.ImgFrame.Type.RGB888i, fps=30, enableUndistortion=True)
# (A) classic: perspective of CAM_A + explicit size
stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
stereo.setOutputSize(640, 400)
# (B) v3 way: link the (undistorted) colour stream into inputAlignTo
rgb.link(stereo.inputAlignTo)
```

Presets as configured by `build(..., preset)` on this device (dumped from `initialConfig` after build):

| preset | subpixel/bits | extended | LR | conf | median | decimation | spatial/temporal/speckle | threshold mm | `getMaxDisparity()` |
|---|---|---|---|---|---|---|---|---|---|
| DEFAULT | yes/3 | no | yes | 15 | 7x7 | 2 | on/on/on | 0-15000 | 8192 |
| ROBOTICS | yes/3 | no | yes | 15 | 7x7 | 2 | on/off/on | 0-10000 | 8192 |
| HIGH_DETAIL | yes/5 | yes | yes | 55 | off | 2 | on/on/on | 0-15000 | 8192 |
| FACE | yes/5 | yes | yes | 15 | off | 2 | on/on/on | 30-3000 | 8192 |
| FAST_ACCURACY / ACCURACY | yes/5 | no | yes | 55 | off | 1 | off | 0-65535 | 3040 |
| FAST_DENSITY / DENSITY | yes/5 | no | yes | 15 | off | 1 | off | 0-65535 | 3040 |

A bare `dai.StereoDepthConfig()` defaults to **subpixel on with 5 fractional bits**: `getMaxDisparity()` = 3040 out of
the box (760 with 3 bits, 95 after `setSubpixel(False)`, 190 extended, 1520 extended + 3-bit subpixel, 6080 extended +
5-bit); default confidence 55, LR-check True, median `MEDIAN_OFF`, `getDepthUnit()` = `LengthUnit.MILLIMETER`. Only the
DEFAULT/ROBOTICS presets set 3 bits - the node's `setSubpixelFractionalBits` docstring ("Default value: 3") is stale.

**Observed on OAK-D S2:** `depth` frames are `Type.RAW16`, numpy `uint16`, shape `(400, 640)` for a 640x400 input
(`(200, 320)` when the preset's decimation factor 2 is left on), **units millimetres**, **invalid pixels == 0**
(12-68 % zeros with the presets; the confidence threshold is the dominant knob, see below), valid values 1368-1490 mm p5-p95 in the test scene;
`disparity` is also `RAW16/uint16` (even with `setSubpixel(False)`), scaled so that `initialConfig.getMaxDisparity()`
(8192 for filter presets) is the full range. The raw value is **not** `px * 8192/95`: inverting the synchronised depth frame
(px = fx*B/depth, fx 399.6 from `depth.getTransformation()`, B 74.966 mm) gave raw/px = **80.0** (p10-p90 79.98-80.02,
n 138814) with the DEFAULT preset (3-bit subpixel), **40.0** with DEFAULT + `setExtendedDisparity(True)`, **32.0** with
HIGH_DETAIL (5-bit subpixel + extended) and **65.9** when the depth is aligned to CAM_A (fx 483.8), while `getMaxDisparity()`
stayed 8192 in all four cases. So normalise with `getMaxDisparity()` for display and use the `depth` output for metric
values instead of converting disparity yourself. `syncedLeft/Right`, `rectifiedLeft/Right`, `confidenceMap` are `RAW8` 640x400. **`confidenceMap` is 255 = most confident,
0 = least** ("Lower values mean lower confidence of the calculated disparity value" - docstring of `StereoDepth.confidenceMap`),
and `setConfidenceThreshold(t)` invalidates pixels whose confidence is below `t`, so a **higher threshold means fewer valid
pixels - the opposite of the v2 semantics**. The percentages are scene-dependent; only the direction is general, and only
255 always empties the map. Two runs (DEFAULT preset, decimation 1, threshold changed at runtime through `inputConfig`):
dim scene: threshold 0 -> 70-74 % invalid, 15 -> 85-86 %, 55 -> 95 %, 128 -> 99 %, 200 and 255 -> 100 % invalid (mean
confidence at the surviving pixels 77 / 91 / 134 / 162); bright scene: 0 -> 21-24 %, 15 -> 38-40 %, 55 -> 72-74 %,
128 -> 81 %, 200 -> 88 % (~12 % still valid), 255 -> 100 % invalid (mean confidence 106 / 113 / 179 / 199 / 222).
Presets use 15 (DEFAULT/ROBOTICS/FACE/density) or 55 (accuracy/HIGH_DETAIL): stay in that range.
Depth `getTransformation()` carries the mono intrinsics (fx 399.6) or, when aligned, the CAM_A ones
(fx 483.8, cx 314.9). Aligning to a **distorted** colour stream logs
`[StereoDepth] [warning] The input connected to inputAlignTo is distorted ... undistort the alignTo frame with camera.requestOutput(..., enableUndistortion=true)`;
with an undistorted stream and the `inputAlignTo` link, `depth.getTransformation().isAlignedTo(rgb.getTransformation())` is `True`
(`setDepthAlign(CAM_A)` alone gave `False`). Subpixel+extended on a preset logs
`[StereoDepth] [error] Maximum disparity value '1520' exceeds the maximum supported '1024' by median filter. Disabling median filter!`.
`dai.node.Depth` (`build(dai.node.Depth.Algorithm.AUTO, None, (640, 400))`, `setAlignTo(output)`, `.depth`) is the
newer abstraction used by the official PointCloud/RGBD examples; it exists in 3.10.0 (`Algorithm`: `AUTO, STEREO, NEURAL,
NEURAL_ASSISTED_STEREO, TOF, GPU_STEREO`) but was not device-tested here.

---

## 5. PointCloud node (and RGBD)

`dai.node.PointCloud`: inputs `inputDepth`, `inputColor`, `inputConfig`; outputs `outputPointCloud`
(`dai.PointCloudData`), `passthroughDepth`; `initialConfig` (`dai.PointCloudConfig`: `setLengthUnit(dai.LengthUnit)`,
`setOrganized(bool)`, `getLengthUnit()`, `getOrganized()`, `setTransformationMatrix(...)`, `setSparse` deprecated);
`setRunOnHost(bool)` (**default: host**), `setTargetCoordinateSystem(socket | dai.HousingCoordinateSystem)`,
`useCPU()/useCPUMT()/useGPU()`. There is no `build()` on PointCloud - link `stereo.depth` into `inputDepth`.

`dai.PointCloudData`: `getPoints() -> np.ndarray float32 (N,3)`, `getPointsRGB() -> (points (N,3) float32, colors (N,4) uint8 RGBA)`,
`isColor()`, `isOrganized()`, `getWidth()`, `getHeight()`, `getMinX/MaxX/MinY/MaxY/MinZ/MaxZ()`, `getSequenceNum()`, `getTimestamp()`.

```python
with dai.Pipeline() as p:
    left  = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    right = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
    stereo = p.create(dai.node.StereoDepth).build(left.requestOutput((640, 400), fps=30),
                                                  right.requestOutput((640, 400), fps=30))
    pc = p.create(dai.node.PointCloud)
    pc.initialConfig.setLengthUnit(dai.LengthUnit.METER)   # default MILLIMETER
    pc.initialConfig.setOrganized(False)                    # default False: only z>0 points; True: W*H points
    stereo.depth.link(pc.inputDepth)
    q = pc.outputPointCloud.createOutputQueue(maxSize=4, blocking=False)
    p.start()
    import time; time.sleep(1.5); q.tryGetAll()
    pcd = q.get()
    pts = pcd.getPoints()                                   # (N,3) float32, metres, camera frame (x right, y down, z forward)
    print(pts.shape, pcd.isOrganized(), pcd.getMinZ(), pcd.getMaxZ())
    p.stop()
```

Colourised cloud, option 1 - `RGBD` node (used by `RGBD/rgbd.py`):

```python
camA = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
rgbd = p.create(dai.node.RGBD).build(camA, stereo, (640, 400), 30)   # build(camera, depthSource, size, fps)
# other overload: build(autocreate: bool, mode=PresetMode.DEFAULT, size=(640, 400), fps=None) - the keyword is `mode`, NOT `presetMode`
qPcl  = rgbd.pcl.createOutputQueue(maxSize=4, blocking=False)         # PointCloudData, isColor()==True, organized
qRgbd = rgbd.rgbd.createOutputQueue(maxSize=4, blocking=False)        # dai.RGBDData: getRGBFrame(), getDepthFrame()
points, colors = qPcl.get().getPointsRGB()                             # (256000,3) float32, (256000,4) uint8
```

Colourised cloud, option 2 - RGB-aligned depth + `inputColor`:

```python
rgb = camA.requestOutput((640, 400), dai.ImgFrame.Type.RGB888i, fps=30, enableUndistortion=True)
rgb.link(stereo.inputAlignTo)          # align depth to the colour stream (see section 4)
stereo.depth.link(pc.inputDepth)
rgb.link(pc.inputColor)
points, colors = q.get().getPointsRGB()
```

Open3D (optional import) as in `point_cloud_visualizer.py`: `pcd.points = o3d.utility.Vector3dVector(points.astype(np.float64))`,
`pcd.colors = o3d.utility.Vector3dVector(np.delete(colors / 255.0, 3, 1))`.

**Observed on OAK-D S2:** default config -> `getPoints()` shape `(38707, 3)` `float32`, `isOrganized()=False`,
`getWidth()=38707, getHeight()=1`, units mm (`z` 1247..14979, median 1618 mm, matches `passthroughDepth` median 1625 mm),
no `z==0` entries (invalid depth pixels are dropped when unorganised); with `setLengthUnit(METER)` `z` median 1.531;
`RGBD.pcl`: `isColor()=True`, `isOrganized()=True`, points `(256000, 3)`, colors `(256000, 4)` uint8 RGBA, invalid points
have `z==0` (their colour is usually but not always `[0,0,0,255]`: dark non-zero colours such as `[0,0,2,255]` and `[0,5,20,255]`
were seen on `z==0` points, so filter on `z==0`, not on the colour); `RGBD.rgbd` -> `RGBDData` rgb `RGB888i` 640x400 + depth `RAW16` uint16 640x400.
`rgbd.setDepthUnits(dai.LengthUnit.METER)` did **not** change the values (still mm) with `build(camera, stereo, size)`;
divide by 1000 yourself or use `PointCloud.initialConfig.setLengthUnit`, which does work. Using `setDepthAlign(CAM_A)`
instead of the `inputAlignTo` link with `inputColor` spams
`[ThreadedNode] [warning] PointCloud: depth and color transformations are not aligned (intrinsics/distortion differ) -- colorization may be misaligned`
but still produces `(193173,3)` coloured points.

---

## 6. IMU node

* `enableIMUSensor(sensor: dai.IMUSensor, reportRate: int)`, `enableIMUSensor(sensors: list[dai.IMUSensor], reportRate: int)`,
  `enableIMUSensor(sensorConfig: dai.IMUSensorConfig)`, `enableIMUSensor(sensorConfigs: list[...])`.
* `setBatchReportThreshold(int)` ("above this packet threshold data will be sent to host"), `setMaxBatchReports(int)`,
  `getBatchReportThreshold()`, `getMaxBatchReports()`, `enableFirmwareUpdate(bool)`, output `out`.
* `dai.IMUSensor` members: `ACCELEROMETER_RAW, ACCELEROMETER_UNCALIBRATED, ACCELEROMETER_CALIBRATED, GYROSCOPE_RAW,
  GYROSCOPE_CALIBRATED, GYROSCOPE_UNCALIBRATED, MAGNETOMETER_RAW, MAGNETOMETER_CALIBRATED, MAGNETOMETER_UNCALIBRATED,
  ROTATION_VECTOR, GAME_ROTATION_VECTOR, GEOMAGNETIC_ROTATION_VECTOR, ARVR_STABILIZED_ROTATION_VECTOR,
  ARVR_STABILIZED_GAME_ROTATION_VECTOR`, plus three **deprecated** members that still exist but raise a `DeprecationWarning`
  merely on attribute access in 3.10.0: `ACCELEROMETER` ("use ACCELEROMETER_CALIBRATED instead"), `LINEAR_ACCELERATION` ("use
  ACCELEROMETER_CALIBRATED and subtract gravity on the host") and `GRAVITY` ("use ACCELEROMETER_CALIBRATED and estimate
  gravity on the host") - prefer `ACCELEROMETER_CALIBRATED`.
* `dai.IMUSensorConfig` fields: `sensorId, reportRate (default 100), sensitivityEnabled, sensitivityRelative, changeSensitivity`.
* `dai.IMUData.packets -> list[dai.IMUPacket]`; `dai.IMUPacket` fields `acceleroMeter` (`IMUReportAccelerometer`: `x, y, z`
  in m/s^2), `gyroscope` (`IMUReportGyroscope`: `x, y, z` in rad/s), `rotationVector` (`IMUReportRotationVectorWAcc`:
  `i, j, k, real` unit quaternion, `rotationVectorAccuracy` in rad), `magneticField` (`IMUReportMagneticField`). Every report
  has `accuracy` (`dai.IMUReport.Accuracy`: `UNRELIABLE, LOW, MEDIUM, HIGH`), `sequence` (uint8, wraps), `timestamp` /
  `tsDevice` (`dai.Timestamp` with `.sec`, `.nsec`, `.get()`), and `getTimestamp() -> timedelta`, `getTimestampDevice()`.

```python
with dai.Pipeline() as p:
    imu = p.create(dai.node.IMU)
    imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], 200)
    imu.setBatchReportThreshold(1)      # 1 = one packet per IMUData message; 10 = 10 packets per message
    imu.setMaxBatchReports(10)
    q = imu.out.createOutputQueue(maxSize=50, blocking=False)
    p.start()
    for _ in range(200):
        imuData = q.get()               # dai.IMUData
        for pk in imuData.packets:
            a, g = pk.acceleroMeter, pk.gyroscope
            latency = dai.Clock.now() - a.getTimestamp()
            print(f"acc {a.x:.3f} {a.y:.3f} {a.z:.3f} m/s^2  gyro {g.x:.4f} {g.y:.4f} {g.z:.4f} rad/s  "
                  f"acc={a.accuracy} seq={a.sequence} ts={a.getTimestamp()} latency={latency}")
    p.stop()
```

**Observed on OAK-D S2 (BNO086):** accel+gyro RAW at 200 Hz, `batchReportThreshold=1`: **200.0 IMUData msgs/s,
200.0 packets/s, 1 packet/msg, accelerometer timestamp spacing mean 4.99 ms (200.2 Hz), min 2.9 / max 8.9 ms**.
At 400 Hz: 400.3 packets/s, 2.50 ms spacing. With `setBatchReportThreshold(10)`: 20 msgs/s x 10 packets = still 200 packets/s.
Adding `ROTATION_VECTOR` at 100 Hz to the same node dropped the packet stream to **100 packets/s** (packets are
emitted at the slowest enabled rate), so put the rotation vector on its own `IMU` node or accept 100 Hz.
Sample values at rest: accel `(-0.06, -0.79, -9.94)` m/s^2 (gravity on -z), gyro ~`0.00` rad/s, `accuracy=Accuracy.UNRELIABLE`
for RAW sensors, rotation vector `rotationVectorAccuracy=3.1416` right after start; host latency `dai.Clock.now() - ts` ~2 ms.
The `sequence` byte steps by 1 or 2 between consecutive packets (735 x 1, 264 x 2 over 5 s) - do not treat a step of 2 as a drop.

---

## 7. Recording

Nodes (all confirmed): `dai.node.VideoEncoder.build(input, bitrate=0, frameRate=30.0, profile=Profile.H264_BASELINE,
keyframeFrequency=30, lossless=False, quality=80)` with `out` (`dai.EncodedFrame`: `getData()`, `getProfile()`);
`dai.node.RecordVideo`: `setRecordVideoFile(PathLike)`, `setRecordMetadataFile(PathLike)`, `setFps(int)`,
`input`, `getRecordVideoFile()`, `getRecordMetadataFile()`;
`dai.node.RecordMetadataOnly`: `setRecordFile(PathLike)`, `input`, `getRecordFile()`.
`setCompressionLevel` / `getCompressionLevel` exist on both record nodes but are **not callable from Python** in 3.10.0:
the parameter type `dai::RecordConfig::CompressionLevel` is not bound (`dai.RecordConfig.CompressionLevel` does not exist),
so `setCompressionLevel(3)` / `('3')` raise `TypeError: incompatible function arguments` and `getCompressionLevel()` raises
`TypeError: Unable to convert function return value to a Python type` - leave the default (3, as `record_config.json` shows).
`dai.VideoEncoderProperties.Profile`: `H264_BASELINE, H264_HIGH, H264_MAIN, H265_MAIN, MJPEG` (what `VideoEncoder.build` takes).
`EncodedFrame.getProfile()` returns a **different** enum, `dai.EncodedFrame.Profile`: `JPEG, AVC, HEVC` (`AVC` is not a member of
`VideoEncoderProperties.Profile`, so compare against `dai.EncodedFrame.Profile.AVC`).

```python
import time
from pathlib import Path
out = Path("recordings"); out.mkdir(exist_ok=True)
with dai.Pipeline() as p:
    cam = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    enc = p.create(dai.node.VideoEncoder).build(cam.requestOutput((1280, 720), dai.ImgFrame.Type.NV12, fps=30),
                                                frameRate=30, profile=dai.VideoEncoderProperties.Profile.H264_MAIN)
    rec = p.create(dai.node.RecordVideo)
    rec.setRecordVideoFile(out / "video.mp4")          # encoded -> MUST be .mp4
    rec.setRecordMetadataFile(out / "video.mcap")      # per-frame metadata (timestamps, seq) for replay
    enc.out.link(rec.input)
    raw = p.create(dai.node.RecordVideo)
    raw.setRecordVideoFile(out / "mono.mkv")           # raw ImgFrame -> MUST be .avi or .mkv
    p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B).requestOutput((640, 400), dai.ImgFrame.Type.NV12).link(raw.input)
    imu = p.create(dai.node.IMU)
    imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], 200)
    imu.setBatchReportThreshold(10)
    recm = p.create(dai.node.RecordMetadataOnly)
    recm.setRecordFile(out / "imu.mcap")
    imu.out.link(recm.input)
    p.start(); time.sleep(3.0); p.stop()
```

Holistic record (`RecordReplay/holistic_record.py`):

```python
cfg = dai.RecordConfig()                # fields: outputDir (pathlib.Path, default Path('.'); a str or Path is accepted and
cfg.outputDir = out                     #         stored as Path), syncCameraOutputs (bool, default True),
                                        #         videoEncoding (RecordConfig.VideoEncoding: enabled=True, bitrate=0,
cfg.videoEncoding.enabled = True        #         lossless=False, profile=Profile.H264_MAIN, quality=80),
cfg.videoEncoding.profile = dai.VideoEncoderProperties.Profile.H264_MAIN   #  compressionLevel (not readable from Python)
with dai.Pipeline(True) as p:
    p.enableHolisticRecord(cfg)         # call BEFORE creating nodes
    camA = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    qa = camA.requestOutput((640, 400), fps=30).createOutputQueue()
    camB = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    camB.requestOutput((640, 400), fps=30).createOutputQueue()
    imu = p.create(dai.node.IMU); imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], 200)
    imu.setBatchReportThreshold(10); imu.out.createOutputQueue()
    p.start(); time.sleep(3.0); p.stop()
# the .tar is written when the pipeline object is destroyed (after the with-block / at interpreter exit)
```

**Observed on OAK-D S2 (3-4 s recordings):**
* RecordVideo H264 720p30: `video.mp4` 1.48 MB (118 EncodedFrame in 4 s, first frame 9.7 KB,
  `getProfile() == dai.EncodedFrame.Profile.AVC` - the `EncodedFrame.Profile` enum, not `VideoEncoderProperties.Profile`)
  + `video.mcap` 13 KB. Raw NV12 640x400: `raw_nv12.mkv` 15.2 MB and `raw_nv12.avi` 15.2 MB (+ 13 KB `.mcap` each).
  `RecordMetadataOnly` IMU: `imu.mcap` (8.6 KB for ~3 s at 20 msgs/s).
* Verbatim node errors: **`Encoded video can only be recorded in .mp4 format`** (encoded -> .mkv/.avi) and
  **`Raw video can only be recorded in .avi or .mkv format`** (raw -> .mp4). A node exception
  (`[ThreadedNode] [error] Node threw exception, stopping the node.`) stops the whole pipeline: the other queues then raise
  `dai.MessageQueue.QueueException: 'MessageQueue was closed'` and the mp4 of the sibling RecordVideo is left as a
  436-byte header-only file. Detect the failure by catching `QueueException` around `get()`: `pipeline.isRunning()` still
  returned `True` immediately after the sibling queue raised (both wrong-extension cases, two runs each), so polling it does
  not notice the failure at that moment.
* Holistic record (CAM_A + CAM_B + IMU, encoding on) produced in `outputDir` while running:
  `OAKDS2EXAMPLEID000_b_IMU.mcap` (32 KB), `OAKDS2EXAMPLEID000_camera_info.json` (26.9 KB, full calibration dump),
  `OAKDS2EXAMPLEID000_record_config.json`, `OAKDS2EXAMPLEID000_v_CameraCAM_A.mcap` (8.8 KB) + `..._v_CameraCAM_A.mp4`
  (396-466 KB), `..._v_CameraCAM_B.mcap` + `.mp4` (207-214 KB); on destruction they are packed into
  **`recording_2026-09-18_00-29-46.tar`** (693 KB; entries `record_config.json, camera_info.json, CameraCAM_A.mcap,
  CameraCAM_A.mp4, CameraCAM_B.mcap, CameraCAM_B.mp4, IMU.mcap`) and the loose files are removed. The file name is
  **timestamped**, not `recording.tar` as the example's default argument assumes - glob for `recording_*.tar`.
  `record_config.json` content: `{"compressionLevel": 3, "outputDir": ..., "syncCameraOutputs": true, "videoEncoding": {"bitrate": 0, "enabled": true, "lossless": false, "profile": "H264_MAIN", "quality": 80}}`.
* With `videoEncoding.enabled = False` the video is a raw **`.avi`** (`CameraCAM_A.avi` 20 MB for 3 s of 640x400) inside a 20 MB tar.
* Recording three encoded cameras (CAM_A+B+C) at once on RVC2 produced empty video mcaps, no mp4 and
  `[depthai] [error] Record: Failed to create tar file: file_size: The system cannot find the file specified.: ..._v_CameraCAM_A.mp4`
  (truncated tar); two encoded cameras + IMU worked. Keep holistic recordings to <= 2 encoded streams on the S2.
* Even the 2-camera encoded holistic recording can **silently produce header-only 436-byte mp4s** (seen in 1 of 4
  identical 3 s CAM_A+CAM_B+IMU runs: the tar held `CameraCAM_A.mp4` / `CameraCAM_B.mp4` of 436 bytes each next to
  normal-looking 7 KB mcaps, with no warning or error logged; an earlier run left a 30 KB tar truncated after
  `CameraCAM_A.mcap`). After recording, check that every `*.mp4` in the tar is larger than 436 bytes and re-record if
  not: such a tar makes `enableHolisticReplay` log `[warning] Replay disabled: Failed to open video file`,
  `[depthai] [warning] Could not set up holistic replay. Record and replay disabled.` and `Node threw exception ...
  Video file not found or could not be opened`, after which the Camera node stops the pipeline and the queues raise
  `QueueException`.

---

## 8. Replay

`dai.node.ReplayVideo`: `setReplayVideoFile(PathLike)`, `setReplayMetadataFile(PathLike)`, `setOutFrameType(dai.ImgFrame.Type)`,
`setFps(float)`, `setSize(w, h)`, `setLoop(bool)`, output `out`, `isRunning()`, getters `getFps/getLoop/getOutFrameType/getSize`.
`dai.node.ReplayMetadataOnly`: `setReplayFile(PathLike)`, `setFps`, `setLoop`, output `out`.
`pipeline.enableHolisticReplay(recordingPath: str)`.

### 8.1 Host-only replay - **works with NO device** (evidence below)

```python
import datetime
from pathlib import Path
p = dai.Pipeline(createImplicitDevice=False)         # host-only: the camera is never touched
replay = p.create(dai.node.ReplayVideo)
replay.setReplayVideoFile(Path("recordings/video.mp4"))
replay.setReplayMetadataFile(Path("recordings/video.mcap"))   # optional: restores timestamps / sequence numbers
replay.setOutFrameType(dai.ImgFrame.Type.NV12)
replay.setLoop(False)
q = replay.out.createOutputQueue(maxSize=8, blocking=True)
p.start()
n = 0
while p.isRunning():
    try:
        frame = q.get(datetime.timedelta(seconds=3))  # returns None on timeout
    except dai.MessageQueue.QueueException:          # 'MessageQueue was closed' -> replay finished, pipeline stopped
        break
    if frame is None:
        break
    n += 1
p.stop()

p2 = dai.Pipeline(createImplicitDevice=False)
rm = p2.create(dai.node.ReplayMetadataOnly)
rm.setReplayFile(Path("recordings/imu.mcap")); rm.setLoop(False)
qi = rm.out.createOutputQueue(maxSize=200, blocking=True)
p2.start()
msg = qi.get()                                        # dai.IMUData with .packets
```

**Evidence (observed):** `dai.Pipeline(createImplicitDevice=False)` + `ReplayVideo`: `getDefaultDevice()` -> `None`,
`start()` took 0.01 s, `dai.Device.getAllAvailableDevices()` reported the camera as **`X_LINK_UNBOOTED` before, during and
after** the replay (i.e. it was never opened); 88 frames of 1280x720 `NV12` (with metadata: `seq` 9..96, recorded
timestamps) replayed from the mp4 in 1.2-1.5 s at `setFps(120)`, 109 frames (`seq` 9..117) 640x400 from the raw `.mkv`; when the file
ends the node stops the pipeline (`pipeline.isRunning()` -> False) and `get()` raises `QueueException('MessageQueue was closed')`.
`ReplayMetadataOnly` on `imu.mcap` replayed `IMUData` messages with 10 packets each (values identical to the recording).
The `dai.Pipeline.__init__` docstring confirms `createImplicitDevice=False` means no device is created, and the official
`HostNodes/host_camera.py` and `threaded_host_nodes.py` run exactly such pipelines. So **ReplayVideo / ReplayMetadataOnly
+ host nodes need no camera** (cannot be tested with the cable pulled here, but the device state never changed).

### 8.2 Holistic replay - **needs a device in 3.10.0**

```python
with dai.Pipeline(True) as p:                                   # createImplicitDevice must be True
    p.enableHolisticReplay(str(Path("recordings") / "recording_2026-09-18_00-29-46.tar"))
    camA = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)   # same sockets/rates as the recording
    qa = camA.requestOutput((640, 400), fps=30).createOutputQueue(maxSize=8, blocking=False)
    imu = p.create(dai.node.IMU)
    imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], 200)
    imu.setBatchReportThreshold(10)
    qi = imu.out.createOutputQueue(maxSize=100, blocking=False)
    p.start()
    frame = qa.get()                                            # frames come from the tar, not the sensor
```

**Evidence (observed):** with `dai.Pipeline(createImplicitDevice=False)` + `enableHolisticReplay(tar)` the first
`p.create(dai.node.Camera)` raises **`RuntimeError: Pipeline is host only, cannot create device node`**. With
`dai.Pipeline(True)` the constructor booted the camera (`getAllAvailableDevices()` -> `[]` while running), `start()` took
0.2 s and replay delivered 86-129 CAM_A frames (`YUV420p` 640x400, first `seq=11`) and 108 `IMUData` in 6 s (it keeps
looping/streaming, `isRunning()` stayed True); an invalid path raises `RuntimeError('Replay file does not exist or is invalid')`.
Conclusion: holistic replay of Camera/IMU nodes requires a connected OAK in this version; only the ReplayVideo /
ReplayMetadataOnly host nodes run device-free.

---

## 9. HostNode, ThreadedHostNode, Sync

* `dai.node.ThreadedHostNode`: subclass, call `super().__init__()`, create ports with
  `createInput(name='', group='', blocking=True, queueSize=3, types=[...], waitForMessage=False)` and
  `createOutput(name='', group='', possibleDatatypes=[dai.Node.DatatypeHierarchy(dai.DatatypeEnum.X, True)])`,
  implement `run(self)` looping `while self.mainLoop():`; optional `onStart()/onStop()`; `createSubnode(cls, ...)`.
* `dai.node.HostNode`: subclass with `build(self, *outputs)` that calls **`self.link_args(*outputs)`** (added to subclasses
  by the Python binding; not on the base class) and `process(self, msg1, msg2, ...)`; the base provides `out`
  (`dai.Node.Output`), `inputs`, `sendProcessingToPipeline(bool)`, `runSyncingOnHost()/runSyncingOnDevice()`, `stopPipeline()`.
  `link_args` type-checks the **annotations of `process()`** against each output's declared datatypes.
* `dai.Node.Output.setPossibleDatatypes([...])`, `dai.DatatypeEnum` (`Buffer, ImgFrame, IMUData, PointCloudData, MessageGroup, ...`).
* `dai.MessageQueue`: `get()`, `get(timedelta) -> message or None on timeout`, `tryGet()`, `tryGetAll()`, `getAll()`, `has()`,
  `getSize()`, `close()`; `dai.MessageQueue.QueueException` (derives from `Exception`). `dai.Node.Input.createInputQueue(maxSize=16, blocking=False) -> dai.InputQueue` with `send()/trySend()`.

```python
import time, datetime, numpy as np

class Source(dai.node.ThreadedHostNode):
    def __init__(self):
        super().__init__()
        # declare the datatype, otherwise HostNode.link_args() refuses to link an ImgFrame-annotated process()
        self.output = self.createOutput(possibleDatatypes=[dai.Node.DatatypeHierarchy(dai.DatatypeEnum.ImgFrame, True)])
        self.n = 0
    def run(self):
        while self.mainLoop():
            img = dai.ImgFrame()
            img.setCvFrame(np.full((40, 60, 3), self.n, dtype=np.uint8), dai.ImgFrame.Type.BGR888i)
            img.setSequenceNum(self.n)
            img.setTimestamp(datetime.timedelta(seconds=time.monotonic()))
            self.output.send(img)
            self.n += 1
            time.sleep(0.05)

class Doubler(dai.node.HostNode):
    def build(self, frames: dai.Node.Output):
        self.link_args(frames)
        self.out.setPossibleDatatypes([dai.Node.DatatypeHierarchy(dai.DatatypeEnum.ImgFrame, True)])  # `out` defaults to Buffer
        return self
    def process(self, frame: dai.ImgFrame):          # annotation = expected input type
        o = dai.ImgFrame(); o.setCvFrame(frame.getCvFrame() * 2, dai.ImgFrame.Type.BGR888i)
        o.setSequenceNum(frame.getSequenceNum()); self.out.send(o)

class Pair(dai.node.HostNode):                       # several inputs are synchronised by link_args
    def build(self, a: dai.Node.Output, b: dai.Node.Output):
        self.link_args(a, b); return self
    def process(self, a: dai.ImgFrame, b: dai.ImgFrame):
        g = dai.Buffer(); g.setSequenceNum(a.getSequenceNum() * 1000 + b.getSequenceNum()); self.out.send(g)

with dai.Pipeline(createImplicitDevice=False) as p:
    src = p.create(Source)
    dbl = p.create(Doubler).build(src.output)
    pair = p.create(Pair).build(src.output, dbl.out)
    q, qp = dbl.out.createOutputQueue(), pair.out.createOutputQueue()
    p.start()
    for _ in range(3):
        f, g = q.get(), qp.get()
    p.stop()
```

Verbatim error when the datatype is not declared:
`TypeError: Input 'frame' cannot be linked due to incompatible message types. Input type: DatatypeEnum.ImgFrame Output type: DatatypeEnum.Buffer`.
For GUI work (`cv2.imshow`) call `self.sendProcessingToPipeline(True)` in `build` and drive with `pipeline.run()`
(`HostNodes/display.py`). Custom message classes derive from `dai.TransformableBuffer` (`Align/custom_message_align.py`).

### 9.1 `dai.node.Sync`

`inputs[name]` (InputMap, any names), output `out` (`dai.MessageGroup`), `setSyncThreshold(datetime.timedelta)`,
`setSyncAttempts(int)` (-1 default = only emit when synced, 0 = emit as soon as the group is full),
`setRunOnHost(bool)` (default device), `setTimestampSource(dai.node.Sync.TimestampSource)` (`DEFAULT, DEVICE, HOST, SYSTEM`).
`dai.MessageGroup`: `group["name"]`, `getMessageNames()`, `getNumMessages()`, `isSynced(thresholdNs: int)`, `getIntervalNs()`,
`getTimestamp()`.

```python
with dai.Pipeline() as p:
    camA  = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    left  = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    right = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
    sync = p.create(dai.node.Sync)
    sync.setSyncThreshold(datetime.timedelta(milliseconds=10))
    sync.setRunOnHost(True)
    camA.requestOutput((640, 400), fps=30).link(sync.inputs["rgb"])
    left.requestOutput((640, 400), fps=30).link(sync.inputs["left"])
    right.requestOutput((640, 400), fps=30).link(sync.inputs["right"])
    q = sync.out.createOutputQueue(maxSize=4, blocking=False)
    p.start()
    group = q.get()                                # dai.MessageGroup
    rgb, l, r = group["rgb"], group["left"], group["right"]
    print(group.getMessageNames(), group.isSynced(10_000_000), group.getIntervalNs())
    p.stop()
```

**Observed on OAK-D S2:** host-only pipeline `Source -> Doubler -> Pair` ran with the camera `X_LINK_UNBOOTED`
(`onStart`/`onStop` were called; `Pair` received matching sequence numbers 0/0, 1/1, 2/2). Sync of rgb+left+right at 30 fps:
`getMessageNames() == ['left', 'rgb', 'right']`, `isSynced(10_000_000) == True`, `getIntervalNs() == 161259` (0.16 ms spread),
identical `getSequenceNum()` on all three frames, types `YUV420p / RAW8 / RAW8`. `MessageQueue.get(timedelta)` returned
`None` after the timeout (0.54 s for 0.5 s), `tryGet()` -> `None`, `has()` -> `False`; after `pipeline.stop()`, `get()` raises
`dai.MessageQueue.QueueException: 'MessageQueue was closed'`.

---

## 10. Top v2 -> v3 migration pitfalls (all hit or verified here)

1. **No XLinkOut/XLinkIn, no `device.getOutputQueue()`**: create queues on node outputs with
   `output.createOutputQueue(maxSize=16, blocking=False)` and on inputs with `input.createInputQueue()`; the pipeline is
   started with `pipeline.start()`, not `dai.Device(pipeline)`.
2. **`dai.node.Camera` replaces `ColorCamera`/`MonoCamera`** (both still exist in `dai.node` but are legacy). Resolution and
   format are chosen per output via `requestOutput((w, h), type, resizeMode, fps, enableUndistortion)`; the sensor mode is picked
   automatically (e.g. 1352x1012 for 640x400). Default type on CAM_A is `YUV420p` (not NV12) and on mono sockets `RAW8`.
3. **`dai.Pipeline()` boots the camera immediately** (2.9 s) - a script that only inspects the device should use
   `dai.Device()`; a script with only host nodes must pass `createImplicitDevice=False`, otherwise it grabs the camera.
4. **Device exclusivity is soft**: while one process streams, another's `dai.Device()` raises `RuntimeError('No available devices')`
   only after ~10-14 s of searching (or succeeds if the first process quits meanwhile), and an *idle* `dai.Device()` is
   sometimes stolen (2 of 7 attempts) and sometimes not - neither outcome can be relied on.
   There is no `X_LINK_DEVICE_ALREADY_IN_USE` exception on this host; always `close()`/`stop()` and run under a timeout.
5. **`StereoDepth.build(left, right, presetMode)` presets silently enable post-processing**: DEFAULT/ROBOTICS/HIGH_DETAIL/FACE
   set decimation factor 2 (depth is half the input size), subpixel, spatial/speckle filters (temporal too, except ROBOTICS)
   and a threshold filter (0-15 m for DEFAULT/HIGH_DETAIL, 0-10 m for ROBOTICS, 0.03-3 m for FACE - see the table in section 4).
   Override via `initialConfig.postProcessing...` after `build`. `setMedianFilter`/`setConfidenceThreshold` live on
   `initialConfig`, not on the node, and **the confidence threshold is inverted vs v2**: 255 = most confident, higher
   threshold = fewer valid pixels (255 empties the depth map; 200 left 0 % valid in a dim scene and ~12 % in a bright one;
   presets use 15 or 55).
6. **Disparity is uint16 and normalised to `initialConfig.getMaxDisparity()`** (8192 with filters on), not "0-95 uint8" as in v2 and
   not simply `px * 8192/95` (measured 80x px with 3-bit subpixel, see section 4); depth stays uint16 mm with 0 = invalid.
7. **Depth-to-RGB alignment**: use `enableUndistortion=True` on the colour output and link it to `stereo.inputAlignTo`
   (or `setDepthAlign(dai.CameraBoardSocket.CAM_A)` + `setOutputSize`); otherwise you get the "input connected to inputAlignTo is
   distorted" warning and `isAlignedTo()` is False.
8. **Auto-exposure warm-up**: the first ~1 s of frames is at maximum gain (ISO ~1550-1599, 30 ms) - saturated in a bright scene,
   under-exposed/black in a dim one; depth is all zero then. Sleep 1-1.5 s and
   `tryGetAll()` before measuring.
9. **Node exceptions stop the pipeline**: any host node error (e.g. wrong record extension) closes all queues -> catch
   `dai.MessageQueue.QueueException` around `get()` (`pipeline.isRunning()` may still be `True` right after the node exception,
   so do not poll it to detect the failure); `get(timedelta)` returns `None` on timeout instead of raising.
10. **Recording formats are enforced**: encoded (`VideoEncoder.out`) -> `.mp4` only; raw `ImgFrame` -> `.avi`/`.mkv` only;
    holistic tar names are timestamped (`recording_YYYY-MM-DD_HH-MM-SS.tar`) and only appear after the pipeline object is destroyed;
    verify every mp4 inside is > 436 bytes (header-only mp4s happen silently, see section 7).
11. **`HostNode.link_args` enforces `process()` annotations** against `Output.setPossibleDatatypes`; `ThreadedHostNode.createOutput()`
    and `HostNode.out` default to `Buffer`, so declare `ImgFrame` explicitly.
12. **IMU rates are per node**: mixing a 100 Hz `ROTATION_VECTOR` with 200 Hz raw accel/gyro on one node yields 100 packets/s.
13. **`dai.Device.getMxId()` is deprecated** (`DeprecationWarning: Use getDeviceId() instead.`); `getBootloaderVersion()` is `None` on USB boot.
14. **RVC2 resource limits are ISP/SIPP memory, not an output count**: a CAM_A 4056x3040 full-res stream plus three more CAM_A
    640x400 outputs (+ 3 mono outputs) crashed the firmware every time (`SIPP error : E_OUT_OF_MEM`, then a crash/reconnect
    loop), while 4 outputs incl. one full-res and 7 x 640x400 outputs without full-res ran fine; 3 encoded holistic streams
    failed every time (3/3). With a full-res stream keep the other outputs of that socket to one or two, and always run under
    a timeout because `[host] [warning] Attempting to reconnect` loops (10 s each) repeat after every re-crash.
15. **`Pipeline(createImplicitDevice=False)` cannot create device nodes** (`RuntimeError: Pipeline is host only, cannot create device node`),
    which is also why holistic replay still needs the camera.

---

## Appendix A - Introspection log (exact commands, run with the project venv)

`PY="<repo checkout>/.venv/Scripts/python.exe"`; every
command was run as `"$PY" -c "<code>"` with `PYTHONIOENCODING=utf-8` (some docstrings contain non-cp1252 characters).

```
import depthai as dai, sys, numpy, cv2; print(sys.version, dai.__version__, numpy.__version__, cv2.__version__); print(dai.Device.getAllAvailableDevices())
   -> 3.13.3, depthai 3.10.0, numpy 2.5.3, cv2 5.0.0, [DeviceInfo(name=2.7, deviceId=OAKDS2EXAMPLEID000, X_LINK_UNBOOTED, X_LINK_USB_VSC, X_LINK_MYRIAD_X, X_LINK_SUCCESS)]
print([n for n in dir(dai.node) if not n.startswith('_')])            # node list (Camera, StereoDepth, PointCloud, RGBD, IMU, Record*/Replay*, HostNode, ThreadedHostNode, Sync, Depth, ...)
print([n for n in dir(dai) if not n.startswith('_')])                  # top-level names
print(dai.Pipeline.__init__.__doc__); print([n for n in dir(dai.Pipeline) if not n.startswith('_')])
print(dai.Pipeline.enableHolisticRecord.__doc__, dai.Pipeline.enableHolisticReplay.__doc__, dai.Pipeline.processTasks.__doc__)
print(dai.Device.__init__.__doc__); print([n for n in dir(dai.Device) if not n.startswith('_')])
print([n for n in dir(dai.DeviceInfo) if not n.startswith('_')]); print(dai.DeviceInfo.__init__.__doc__)
print(list(dai.UsbSpeed.__members__), list(dai.XLinkDeviceState.__members__), list(dai.XLinkProtocol.__members__), list(dai.XLinkPlatform.__members__), list(dai.Platform.__members__), list(dai.CameraBoardSocket.__members__), list(dai.CameraSensorType.__members__))
for name in ['getDeviceName','getProductName','getDeviceId','getPlatform','getPlatformAsString','getUsbSpeed','getConnectedCameraFeatures','getConnectedCameras','getConnectedIMU','getIMUFirmwareVersion','getBootloaderVersion','readCalibration','readCalibration2','readCalibrationOrDefault','getDeviceInfo','getMxId','close','isClosed','isPipelineRunning','getCameraSensorNames','getStereoPairs','getAvailableStereoPairs']: print(name, getattr(dai.Device, name).__doc__)
print(dai.Device.getAllAvailableDevices.__doc__, dai.Device.getDeviceById.__doc__, dai.Device.getAnyAvailableDevice.__doc__, dai.Device.getDeviceByIdOrName.__doc__)
print([n for n in dir(dai.CameraFeatures) if not n.startswith('_')]); print([n for n in dir(dai.CameraInfo) if not n.startswith('_')])
print([n for n in dir(dai.CalibrationHandler) if not n.startswith('_')]); for name in ['getBaselineDistance','getCameraIntrinsics','getDistortionCoefficients','getFov','getCameraExtrinsics','getLensPosition','getCameraToImuExtrinsics','getStereoLeftRectificationRotation','getCameraTranslationVector','getDefaultIntrinsics','getDistortionModel','getStereoLeftCameraId','getStereoRightCameraId','getEepromData']: print(getattr(dai.CalibrationHandler, name).__doc__)
print([n for n in dir(dai.EepromData) if not n.startswith('_')]); print(list(dai.LengthUnit.__members__), list(dai.CameraModel.__members__))
print([n for n in dir(dai.node.Camera) if not n.startswith('_')]); print(dai.node.Camera.build.__doc__); print(dai.node.Camera.requestOutput.__doc__); print(dai.node.Camera.requestFullResolutionOutput.__doc__); print(list(dai.ImgResizeMode.__members__))
print(list(dai.ImgFrame.Type.__members__)); print([n for n in dir(dai.ImgFrame) if not n.startswith('_')]); for name in ['getCvFrame','getTimestamp','getTimestampDevice','getSequenceNum','getFrame','getLensPosition','getLensPositionRaw','getExposureTime','getSensitivity','setCvFrame','setTimestamp']: print(getattr(dai.ImgFrame, name).__doc__)
print([n for n in dir(dai.CameraControl) if not n.startswith('_')]); print(list(dai.CameraControl.AutoFocusMode.__members__)); for name in ['setAutoFocusMode','setAutoFocusTrigger','setManualFocus','setManualFocusRaw','setAutoFocusRegion','setAutoFocusLensRange','setManualExposure','setAutoExposureEnable','setAutoExposureLock']: print(getattr(dai.CameraControl, name).__doc__)
S = dai.node.StereoDepth; print([n for n in dir(S) if not n.startswith('_')]); print(S.build.__doc__); print(list(S.PresetMode.__members__), {k:int(v) for k,v in S.PresetMode.__members__.items()}); print(S.setDefaultProfilePreset.__doc__)
for name in ['setLeftRightCheck','setSubpixel','setSubpixelFractionalBits','setExtendedDisparity','setMedianFilter','setDepthAlign','setRectification','setOutputSize','setRectifyEdgeFillColor','setConfidenceThreshold','setAlphaScaling','setOutputKeepAspectRatio','setBaseline','setFocalLength','enableDistortionCorrection','useHomographyRectification','setInputResolution']: print(name, getattr(S, name, None) and getattr(S, name).__doc__)   # setMedianFilter / setConfidenceThreshold -> MISSING on the node
print(list(dai.MedianFilter.__members__), dai.filters.params.MedianFilter is dai.MedianFilter); print([n for n in dir(dai.StereoDepthConfig) if not n.startswith('_')]); print(dai.StereoDepthConfig.getMaxDisparity.__doc__, dai.StereoDepthConfig.setMedianFilter.__doc__, dai.StereoDepthConfig.setDepthUnit.__doc__, dai.StereoDepthConfig.setConfidenceThreshold.__doc__)
print(list(dai.StereoDepthConfig.AlgorithmControl.DepthAlign.__members__), list(dai.StereoDepthConfig.AlgorithmControl.DepthUnit.__members__), [n for n in dir(dai.StereoDepthConfig.PostProcessing) if not n.startswith('_')], list(dai.StereoDepthConfig.PostProcessing.Filter.__members__))
print(dai.utility.colorizeDepthFrame.__doc__)
print([n for n in dir(dai.node.PointCloud) if not n.startswith('_')]); print([n for n in dir(dai.PointCloudData) if not n.startswith('_')]); for name in ['getPoints','getPointsRGB','isColor','getWidth','getHeight','isSparse','getMinZ','getMaxZ']: print(getattr(dai.PointCloudData, name).__doc__)
print([n for n in dir(dai.PointCloudConfig) if not n.startswith('_')]); print(list(dai.PointCloudConfig.CoordinateSystemType.__members__)); print(dai.PointCloudConfig.setLengthUnit.__doc__, dai.PointCloudConfig.setOrganized.__doc__, dai.node.PointCloud.setRunOnHost.__doc__, dai.node.PointCloud.setTargetCoordinateSystem.__doc__)
print([n for n in dir(dai.node.RGBD) if not n.startswith('_')]); print(dai.node.RGBD.build.__doc__, dai.node.RGBD.setDepthUnits.__doc__); print([n for n in dir(dai.RGBDData) if not n.startswith('_')])
print([n for n in dir(dai.node.Depth) if not n.startswith('_')]); print(dai.node.Depth.build.__doc__, list(dai.node.Depth.Algorithm.__members__), dai.node.Depth.setAlignTo.__doc__)
print([n for n in dir(dai.node.ImageAlign) if not n.startswith('_')])
print([n for n in dir(dai.node.IMU) if not n.startswith('_')]); print(dai.node.IMU.enableIMUSensor.__doc__, dai.node.IMU.setBatchReportThreshold.__doc__, dai.node.IMU.setMaxBatchReports.__doc__); print(list(dai.IMUSensor.__members__))
for c in [dai.IMUData, dai.IMUPacket, dai.IMUReportAccelerometer, dai.IMUReportGyroscope, dai.IMUReportRotationVectorWAcc, dai.IMUReport, dai.IMUSensorConfig, dai.Timestamp]: print(c.__name__, [n for n in dir(c) if not n.startswith('_')]); print(list(dai.IMUReport.Accuracy.__members__))
for cls in ['RecordVideo','RecordMetadataOnly','ReplayVideo','ReplayMetadataOnly']: c = getattr(dai.node, cls); print(cls, [n for n in dir(c) if not n.startswith('_')]); [print(name, getattr(c, name).__doc__) for name in ['setRecordVideoFile','setRecordMetadataFile','setRecordFile','setReplayVideoFile','setReplayMetadataFile','setReplayFile','setOutFrameType','setFps','setSize','setLoop','setCompressionLevel'] if hasattr(c, name)]
print([n for n in dir(dai.RecordConfig) if not n.startswith('_')]); rc = dai.RecordConfig(); print(rc.outputDir, rc.syncCameraOutputs, rc.videoEncoding.enabled, rc.videoEncoding.bitrate, rc.videoEncoding.lossless, rc.videoEncoding.profile, rc.videoEncoding.quality); print(rc.compressionLevel)   # -> TypeError: Unable to convert function return value to a Python type! (dai::RecordConfig::CompressionLevel)
print(list(dai.VideoEncoderProperties.Profile.__members__)); print(dai.node.VideoEncoder.build.__doc__)
print([n for n in dir(dai.node.HostNode) if not n.startswith('_')]); print(dai.node.HostNode.sendProcessingToPipeline.__doc__, hasattr(dai.node.HostNode, 'process'), hasattr(dai.node.HostNode, 'link_args'))   # process/link_args are added on subclasses
print([n for n in dir(dai.node.ThreadedHostNode) if not n.startswith('_')]); print(dai.node.ThreadedHostNode.createInput.__doc__, dai.node.ThreadedHostNode.createOutput.__doc__, dai.node.ThreadedHostNode.mainLoop.__doc__)
print([n for n in dir(dai.node.Sync) if not n.startswith('_')]); print(dai.node.Sync.setSyncThreshold.__doc__, dai.node.Sync.setSyncAttempts.__doc__, dai.node.Sync.setRunOnHost.__doc__, list(dai.node.Sync.TimestampSource.__members__)); print([n for n in dir(dai.MessageGroup) if not n.startswith('_')], hasattr(dai.MessageGroup, '__getitem__'))
print([n for n in dir(dai.Node.Output) if not n.startswith('_')], dai.Node.Output.createOutputQueue.__doc__); print([n for n in dir(dai.Node.Input) if not n.startswith('_')], dai.Node.Input.createInputQueue.__doc__); print([n for n in dir(dai.MessageQueue) if not n.startswith('_')], dai.MessageQueue.get.__doc__, dai.MessageQueue.tryGet.__doc__, dai.MessageQueue.tryGetAll.__doc__); print([n for n in dir(dai.InputQueue) if not n.startswith('_')]); print([n for n in dir(dai.Buffer) if not n.startswith('_')], dai.Buffer.setData.__doc__)
print(list(dai.DatatypeEnum.__members__)[:40], dai.Node.DatatypeHierarchy.__init__.__doc__, dai.XLinkError.__mro__, dai.XLinkReadError.__mro__, [c.__name__ for c in dai.MessageQueue.QueueException.__mro__])
print(hasattr(dai.Pipeline, 'isHolisticRecordEnabled'), hasattr(dai.Pipeline, 'isHolisticReplayEnabled'))   # False, False
print(type(dai.ImgFrame().getTimestamp()).__name__, type(dai.Clock.now()).__name__)   # timedelta, timedelta
```

Official example sources were fetched with
`curl -sfL https://raw.githubusercontent.com/luxonis/depthai-core/main/examples/python/<Dir>/<file>.py` and directory
listings with `gh api repos/luxonis/depthai-core/contents/examples/python/<Dir> --jq '.[] | .name'`.

## Appendix B - Device test scripts behind the "Observed" notes

All run as `timeout 90 "$PY" <script> [args]` from the session scratchpad
`<scratchpad>\tests\`:
`t1_device.py` (discovery, bogus id, introspection, calibration), `t2a_camera.py {basic,types,fullres,af,badtype,usb2}`,
`t3b_stereo.py {presets,A,B,C}`, `t4_pointcloud.py {A,B,C,D}`, `t5b_imu.py <rate> [rot] [threshold]`,
`t6_record.py {nodes,holistic}`, `t6b_record.py {formats,holistic_raw,holistic_enc} [outdir]`,
`t7b_replay.py {video,meta,holistic} <file>`, `t8_busy.py`, `t8b_busy.py`, `t8c_busy_long.py`, `t9_hostnodes.py {host,sync}`.
Recordings used for the replay evidence: `.../tests/rec_out/imu.mcap`, `.../tests/rec_out2/{h264.mp4,h264.mcap,raw_nv12.mkv,raw_nv12.avi}`,
`.../tests/rec_out2/holistic_raw/recording_2026-09-18_00-29-34.tar`, `.../tests/rec_out2/holistic_enc/recording_2026-09-18_00-33-54.tar`, plus a
temporary short-path copy (`%LOCALAPPDATA%\Temp\claude\dai_rec\{fmt,holistic_enc}`, used for the mp4/mkv/holistic replay runs)
that was deleted after testing.
Fact-check re-runs behind the corrected notes (confidence-threshold sweep, `pipeline.stop()` closing the device and the
native crash on reuse, L/R timestamp deltas, first-frame exposure, continuous-AF timing, idle-holder takeover, header-only
holistic mp4s): `.../scratchpad/fc/{d2_lifecycle,d3_camera,d4_stereo,d4c_conf,d7_record,d8_replay,d10_exceptions}.py`
and `.../scratchpad/fix/{a_lifecycle,a2_reuse,b_camera,c_conf,holder,contender}.py`.
Second fact-check pass (full-res + extra CAM_A outputs -> SIPP out-of-memory crash, disparity raw/px factors, bright-scene
confidence sweep, RGBD invalid-point colours, replay frame counts, `isRunning()` after a node exception):
`.../scratchpad/fc2/{d2_camera,d2a_snippet,d2b_nofull,d3b_conf,d3d_dispscale,d4_rgbd,d6a_record,n4_replay}.py`
(`d3d_dispscale.py` variants: none, `ext3`, `ext`, `distorted`) and `.../scratchpad/fc3_replay_count.py`; the crash cause was
read from `%LOCALAPPDATA%\depthai\cache\crashdumps\<hash>\crash_dump_2026-09-18_09_50_30.tar.gz` -> `crash_reports.json`.
