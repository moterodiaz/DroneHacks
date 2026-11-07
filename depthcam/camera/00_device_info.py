#!/usr/bin/env python3
"""00_device_info.py - enumerate the connected OAK camera and print a full device report.

What this test checks
---------------------
The first thing to run after plugging in the OAK-D S2.  It proves that the host can see
the camera over USB, boot it and read everything static about it - without starting a
camera pipeline.  The report contains:

* enumeration before boot (device id, XLink state / protocol / platform),
* device name, product name, device id, platform, the time ``open_device`` took (one
  enumeration pass ~0.5 s + the ~3 s firmware boot) and USB speed - with a loud warning
  on stderr when the link is not USB 3 (``SUPER``): a USB 2 fallback (``HIGH``) costs
  ~10x bandwidth and silently drops frames,
* every firmware version the API exposes: depthai library version + commit, the RVC2
  device firmware embedded in the library (``dai.__device_version__``), the embedded
  bootloader version and the device bootloader version (``None`` when booted over USB
  straight from the library, which is the normal case for the S2),
* IMU type and IMU firmware version (plus the embedded/upgradable version),
* every connected camera socket: sensor name, native resolution, supported sensor
  types, autofocus flag, orientation and the sensor modes it offers,
* the stereo pairs the device reports (baseline in mm), and
* from ``device.readCalibration()``: stereo left / right sockets, baseline in mm
  (spec and calibrated), CAM_A / CAM_B / CAM_C intrinsics (fx, fy, cx, cy) at native
  sensor size, FOV, distortion model and the EEPROM board information.

How to run (from the repo root)
-------------------------------
    python camera/00_device_info.py                     # human-readable report
    python camera/00_device_info.py --json              # same report as JSON on stdout
    python camera/00_device_info.py --json > device.json
    python camera/00_device_info.py --duration 10       # + hold the device open 10 s and log
                                                         #   chip temperature / USB speed / memory 1x per s
    python camera/00_device_info.py --device-id OAKDS2EXAMPLEID000
    python camera/00_device_info.py --dump-calibration out/calibration.json

What you should see (OAK-D S2, depthai 3.10.0)
-----------------------------------------------
    Device: OAK-D-S2 (OAK-D-S2-AF) id=OAKDS2EXAMPLEID000 platform=RVC2 usb=SUPER
    cameras: CAM_A IMX378 4056x3040 COLOR autofocus, CAM_B / CAM_C OV9282 1280x800 MONO
    IMU: BNO086 firmware 3.9.9;  bootloader: none (USB boot)
    stereo: left=CAM_B right=CAM_C baseline 75.00 mm (spec) / ~74.97 mm (calibrated)
    CAM_A @4056x3040 fx~3066 fy~3066 cx~1996 cy~1471; CAM_B/CAM_C @1280x800 fx~798
A ``!!! WARNING`` line on stderr means the camera negotiated USB 2 (``HIGH``): use a
USB 3 cable and port.

Keys / exit codes
-----------------
No OpenCV window is opened, so there are no keys; Ctrl-C stops the optional
``--duration`` monitor loop cleanly (exit 130): the samples gathered so far are kept and
the report is still printed, with ``monitor.interrupted = true``.  ``--headless`` is
accepted for CLI consistency with the other scripts and changes nothing here.
Exit 0 = report printed;  2 = no device / wrong id / device busy (one final line on
stderr, via ``common.open_device``: an explicit ``--device-id`` never retries and fails
at once, ~1 s, whether or not a camera is plugged in; only a run WITHOUT ``--device-id``
retries an EMPTY enumeration - unplugged, held by another process, or still rebooting
after another script - 3x with a ``Retrying in 3 s`` notice on stderr before that line,
roughly 8 s in total);  1 = the device answered but an essential query failed
(the partial report is still printed, the failures are listed under ``errors``).

``--json`` prints ONLY the JSON document on stdout.  As soon as the script starts - before
``depthai`` is even imported - file descriptor 1 is pointed at stderr for the rest of the
process, so the device table, the summary line and every log line the depthai / XLink
libraries write to stdout (their import-time lines with ``DEPTHAI_LEVEL`` / ``XLINK_LEVEL``
set, and lines they buffer until process exit) end up on stderr; the JSON document itself
is written through a private copy of the original stdout.  ``--json > file.json`` therefore
yields a valid JSON file even with those debug levels enabled.

Verified against depthai 3.10.0 on the OAK-D S2; see ``camera/docs/depthai-v3-cheatsheet.md``
sections 1.5, 2, 2.1 and 2.2 for the API facts this script relies on.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Sequence, TypeVar

sys.path.insert(0, str(Path(__file__).resolve().parent))


def _divert_stdout_fd() -> Optional[int]:
    """Point file descriptor 1 (stdout) at stderr for the rest of the process.

    Returns a duplicate of the ORIGINAL stdout descriptor - the ``--json`` document is
    written through it - or ``None`` when the redirect could not be set up (stdout or
    stderr closed); the caller then falls back to a plain ``print``.

    Redirecting the descriptor rather than ``sys.stdout`` also catches what the depthai /
    XLink C++ libraries write to stdout, and because it is never restored, lines they keep
    in a C-stdio buffer until process exit land on stderr as well instead of trailing the
    JSON.  ``os.dup`` / ``os.dup2`` are portable (Windows and Linux).
    """
    try:
        sys.stdout.flush()
        saved_fd = os.dup(1)
        os.dup2(2, 1)
        return saved_fd
    except (OSError, AttributeError, ValueError):
        return None


#: --json: the saved original-stdout descriptor.  Armed here, before depthai is imported, so
#: that even the library's import-time log lines (seen with DEPTHAI_LEVEL / XLINK_LEVEL set)
#: cannot land in the JSON document; ``main()`` arms it lazily when called programmatically.
_JSON_STDOUT_FD: Optional[int] = (
    _divert_stdout_fd()
    if "--json" in sys.argv[1:] and not ({"-h", "--help"} & set(sys.argv[1:]))
    else None)

import depthai as dai  # noqa: E402

from common import (  # noqa: E402
    Periodic,
    RunClock,
    add_device_arg,
    add_run_args,
    ensure_dir,
    list_devices,
    open_device,
    print_device_summary,
)

__all__ = ["main", "collect_report", "format_report", "EXIT_OK", "EXIT_ERROR", "EXIT_INTERRUPTED"]

EXIT_OK = 0
EXIT_ERROR = 1
EXIT_INTERRUPTED = 130

_T = TypeVar("_T")

#: USB speeds that are NOT a USB 2 fallback.
_GOOD_USB_SPEEDS = (dai.UsbSpeed.SUPER, dai.UsbSpeed.SUPER_PLUS)

#: Sockets whose intrinsics are always reported first (the S2 has exactly these three).
_PRIMARY_SOCKETS = (dai.CameraBoardSocket.CAM_A, dai.CameraBoardSocket.CAM_B, dai.CameraBoardSocket.CAM_C)

_MIB = 1024.0 * 1024.0


# --------------------------------------------------------------------------------------
# Small helpers
# --------------------------------------------------------------------------------------

def _enum_name(value: Any) -> Optional[str]:
    """Short name of a pybind enum member (``UsbSpeed.SUPER`` -> ``'SUPER'``); None stays None."""
    if value is None:
        return None
    return str(getattr(value, "name", value))


def _one_line(text: Any) -> str:
    return " ".join(str(text).split())


def _version_str(value: Any) -> Optional[str]:
    """``dai.Version`` (or anything) -> ``'3.9.9'``; ``None`` stays ``None``."""
    if value is None:
        return None
    to_semver = getattr(value, "toStringSemver", None)
    if callable(to_semver):
        try:
            return str(to_semver())
        except Exception:  # noqa: BLE001 - fall back to str()
            pass
    return str(value)


def _round(value: Any, digits: int = 3) -> Optional[float]:
    try:
        return round(float(value), digits)
    except (TypeError, ValueError):
        return None


def _matrix_to_intrinsics(matrix: Sequence[Sequence[float]]) -> Dict[str, float]:
    """3x3 intrinsic matrix -> ``{'fx', 'fy', 'cx', 'cy'}`` (rounded to 0.01 px)."""
    return {
        "fx": round(float(matrix[0][0]), 2),
        "fy": round(float(matrix[1][1]), 2),
        "cx": round(float(matrix[0][2]), 2),
        "cy": round(float(matrix[1][2]), 2),
    }


class _Collector:
    """Runs getters and records failures instead of raising.

    ``essential`` failures land in ``errors`` (-> exit 1), the rest in ``notes``
    (APIs that are simply not supported on this platform, e.g. ``getOSVersion`` on RVC2).
    """

    def __init__(self) -> None:
        self.errors: List[str] = []
        self.notes: List[str] = []

    def get(self, label: str, getter: Callable[[], _T], default: _T, essential: bool = True) -> _T:
        try:
            return getter()
        except Exception as exc:  # noqa: BLE001 - every failure becomes a report entry
            message = "{}: {}: {}".format(label, type(exc).__name__, _one_line(exc) or "(no message)")
            (self.errors if essential else self.notes).append(message)
            return default


def _json_stdout_fd() -> Optional[int]:
    """The saved original-stdout descriptor for ``--json``, arming the redirect on first use."""
    global _JSON_STDOUT_FD
    if _JSON_STDOUT_FD is None:
        _JSON_STDOUT_FD = _divert_stdout_fd()
    return _JSON_STDOUT_FD


def _emit_json(report: Dict[str, Any], fd: Optional[int]) -> None:
    """Write the report as a JSON document to the saved original stdout (fallback: ``print``).

    ``json.dumps`` escapes non-ASCII by default, so the bytes are plain ASCII and safe for a
    console, a pipe and a file alike; ``os.write`` is unbuffered, so nothing can interleave.
    """
    text = json.dumps(report, indent=2, default=str) + "\n"
    if fd is None:
        print(text, end="", flush=True)
        return
    data = text.encode("utf-8")
    offset = 0
    while offset < len(data):
        offset += os.write(fd, data[offset:])


# --------------------------------------------------------------------------------------
# Report collection
# --------------------------------------------------------------------------------------

def _device_info_dict(info: dai.DeviceInfo) -> Dict[str, Any]:
    return {
        "device_id": str(getattr(info, "deviceId", "") or ""),
        "name": str(getattr(info, "name", "") or ""),
        "state": _enum_name(getattr(info, "state", None)),
        "protocol": _enum_name(getattr(info, "protocol", None)),
        "platform": _enum_name(getattr(info, "platform", None)),
    }


def _camera_dicts(device: dai.Device, col: _Collector) -> List[Dict[str, Any]]:
    features = col.get("getConnectedCameraFeatures", device.getConnectedCameraFeatures, [])
    sensor_names = col.get("getCameraSensorNames", device.getCameraSensorNames, {}, essential=False)
    cameras: List[Dict[str, Any]] = []
    for feat in features:
        modes: List[Dict[str, Any]] = []
        for cfg in list(getattr(feat, "configs", []) or []):
            modes.append({
                "width": int(cfg.width),
                "height": int(cfg.height),
                "type": _enum_name(cfg.type),
                "min_fps": _round(cfg.minFps, 2),
                "max_fps": _round(cfg.maxFps, 2),
            })
        calres = getattr(feat, "calibrationResolution", None)
        calibration_resolution = None
        if calres is not None:
            calibration_resolution = {
                "width": int(getattr(calres, "width", -1)),
                "height": int(getattr(calres, "height", -1)),
            }
        socket = feat.socket
        sensor = str(feat.sensorName or "")
        if not sensor and isinstance(sensor_names, dict):
            sensor = str(sensor_names.get(socket, "") or "")
        cameras.append({
            "socket": _enum_name(socket),
            "sensor": sensor or "?",
            "name": str(getattr(feat, "name", "") or ""),
            "width": int(feat.width),
            "height": int(feat.height),
            "supported_types": [_enum_name(t) for t in list(feat.supportedTypes or [])],
            "has_autofocus": bool(feat.hasAutofocus),
            "has_autofocus_ic": bool(getattr(feat, "hasAutofocusIC", False)),
            "orientation": _enum_name(getattr(feat, "orientation", None)),
            "calibration_resolution": calibration_resolution,
            "modes": modes,
            "_socket": socket,  # removed before output; used for the calibration section
        })
    return cameras


def _calibration_dict(device: dai.Device, cameras: List[Dict[str, Any]],
                      stereo_pairs: List[Dict[str, Any]], col: _Collector) -> Optional[Dict[str, Any]]:
    calib = col.get("readCalibration", device.readCalibration, None)
    if calib is None:
        return None

    result: Dict[str, Any] = {
        "has_calibration_data": bool(col.get("hasCalibrationData", calib.hasCalibrationData, False)),
        "eeprom": None,
        "stereo": None,
        "cameras": {},
    }

    eeprom = col.get("getEepromData", calib.getEepromData, None)
    if eeprom is not None:
        imu_ext = getattr(eeprom, "imuExtrinsics", None)
        result["eeprom"] = {
            "version": int(getattr(eeprom, "version", -1)),
            "board_name": str(getattr(eeprom, "boardName", "") or ""),
            "board_rev": str(getattr(eeprom, "boardRev", "") or ""),
            "board_conf": str(getattr(eeprom, "boardConf", "") or ""),
            "hardware_conf": str(getattr(eeprom, "hardwareConf", "") or ""),
            "product_name": str(getattr(eeprom, "productName", "") or ""),
            "device_name": str(getattr(eeprom, "deviceName", "") or ""),
            "batch_name": str(getattr(eeprom, "batchName", "") or ""),
            "batch_time": int(getattr(eeprom, "batchTime", 0) or 0),
            "stereo_use_spec_translation": bool(getattr(eeprom, "stereoUseSpecTranslation", False)),
            "stereo_distortion_correction": bool(getattr(eeprom, "stereoEnableDistortionCorrection", False)),
            "imu_reference_socket": (_enum_name(getattr(imu_ext, "toCameraSocket", None))
                                     if imu_ext is not None else None),
        }

    # Stereo sockets: from the calibration; fall back to the first device stereo pair.
    left = col.get("getStereoLeftCameraId", calib.getStereoLeftCameraId, None)
    right = col.get("getStereoRightCameraId", calib.getStereoRightCameraId, None)
    if left in (None, dai.CameraBoardSocket.AUTO) or right in (None, dai.CameraBoardSocket.AUTO):
        if stereo_pairs:
            left = getattr(dai.CameraBoardSocket, stereo_pairs[0]["left"], None)
            right = getattr(dai.CameraBoardSocket, stereo_pairs[0]["right"], None)
    if left is not None and right is not None and left != dai.CameraBoardSocket.AUTO:
        mm = dai.LengthUnit.MILLIMETER
        spec = col.get("getBaselineDistance(spec)", lambda: calib.getBaselineDistance(
            cam1=right, cam2=left, useSpecTranslation=True, unit=mm), None)
        measured = col.get("getBaselineDistance(calibrated)", lambda: calib.getBaselineDistance(
            cam1=right, cam2=left, useSpecTranslation=False, unit=mm), None)
        translation = col.get("getCameraTranslationVector", lambda: calib.getCameraTranslationVector(
            left, right, False, mm), None, essential=False)
        result["stereo"] = {
            "left": _enum_name(left),
            "right": _enum_name(right),
            "baseline_mm_spec": _round(spec, 3),
            "baseline_mm_calibrated": _round(measured, 3),
            "left_to_right_translation_mm": [_round(v, 3) for v in translation] if translation else None,
        }

    # Intrinsics at native sensor size for every connected camera (CAM_A/B/C first).
    ordered = sorted(cameras, key=lambda c: (c["_socket"] not in _PRIMARY_SOCKETS, int(c["_socket"])))
    for cam in ordered:
        socket = cam["_socket"]
        name = cam["socket"]
        has_cal = col.get("hasCameraCalibration({})".format(name),
                          lambda s=socket: calib.hasCameraCalibration(s), False, essential=False)
        if not has_cal:
            col.notes.append("no calibration data for {}".format(name))
            continue
        entry: Dict[str, Any] = {}
        default = col.get("getDefaultIntrinsics({})".format(name),
                          lambda s=socket: calib.getDefaultIntrinsics(s), None)
        if default is not None:
            matrix, cal_w, cal_h = default
            entry["calibrated_at"] = {"width": int(cal_w), "height": int(cal_h)}
            entry["calibrated_at"].update(_matrix_to_intrinsics(matrix))
            native_w, native_h = int(cam["width"]), int(cam["height"])
            if native_w <= 0 or native_h <= 0:
                native_w, native_h = int(cal_w), int(cal_h)
            if (native_w, native_h) == (int(cal_w), int(cal_h)):
                native_matrix = matrix
            else:
                native_matrix = col.get(
                    "getCameraIntrinsics({},{}x{})".format(name, native_w, native_h),
                    lambda s=socket, w=native_w, h=native_h: calib.getCameraIntrinsics(s, w, h),
                    None)
            if native_matrix is not None:
                entry["native"] = {"width": native_w, "height": native_h}
                entry["native"].update(_matrix_to_intrinsics(native_matrix))
        entry["fov_deg_spec"] = _round(col.get(
            "getFov({})".format(name), lambda s=socket: calib.getFov(s, True), None, essential=False), 2)
        entry["fov_deg_from_intrinsics"] = _round(col.get(
            "getFov({},intrinsics)".format(name), lambda s=socket: calib.getFov(s, False), None,
            essential=False), 2)
        entry["distortion_model"] = _enum_name(col.get(
            "getDistortionModel({})".format(name), lambda s=socket: calib.getDistortionModel(s), None,
            essential=False))
        coeffs = col.get("getDistortionCoefficients({})".format(name),
                         lambda s=socket: calib.getDistortionCoefficients(s), None, essential=False)
        entry["distortion_coefficients"] = [_round(c, 6) for c in coeffs] if coeffs is not None else None
        if cam["has_autofocus"]:
            entry["lens_position"] = col.get("getLensPosition({})".format(name),
                                             lambda s=socket: calib.getLensPosition(s), None, essential=False)
        result["cameras"][name] = entry
    return result


def collect_report(device: dai.Device, enumeration: Sequence[dai.DeviceInfo], open_s: float) -> Dict[str, Any]:
    """Query an open device and return the full report as a JSON-serialisable dict.

    *open_s* is what ``common.open_device`` took: one enumeration pass (~0.5 s) plus the
    firmware boot (~3 s on the S2 over USB 3).  It is reported as ``device.open_time_s``;
    the API exposes no pure boot timer, so the enumeration share is included by design.
    """
    col = _Collector()

    speed = col.get("getUsbSpeed", device.getUsbSpeed, None)
    speed_name = _enum_name(speed)
    usb3 = speed in _GOOD_USB_SPEEDS
    usb_warning = None if usb3 else (
        "USB speed is {} instead of SUPER (USB 3): expect ~10x less bandwidth and dropped frames; "
        "use a USB 3 cable and port.".format(speed_name or "unknown"))

    bootloader = col.get("getBootloaderVersion", device.getBootloaderVersion, None, essential=False)
    embedded_bl = col.get("DeviceBootloader.getEmbeddedBootloaderVersion",
                          dai.DeviceBootloader.getEmbeddedBootloaderVersion, None, essential=False)
    temperature = col.get("getChipTemperature", device.getChipTemperature, None, essential=False)
    post_boot = col.get("getDeviceInfo", device.getDeviceInfo, None, essential=False)
    ir_drivers = col.get("getIrDrivers", device.getIrDrivers, [], essential=False)

    device_section: Dict[str, Any] = {
        "name": str(col.get("getDeviceName", device.getDeviceName, "") or ""),
        "product_name": str(col.get("getProductName", device.getProductName, "") or ""),
        "device_id": str(col.get("getDeviceId", device.getDeviceId, "") or ""),
        "platform": str(col.get("getPlatformAsString", device.getPlatformAsString, "") or ""),
        "open_time_s": round(float(open_s), 2),  # enumerate + boot (see the docstring)
        "usb_speed": speed_name,
        "usb3": bool(usb3),
        "usb_warning": usb_warning,
        "connection_interfaces": [_enum_name(i) for i in col.get(
            "getConnectionInterfaces", device.getConnectionInterfaces, [], essential=False)],
        "xlink_after_boot": _device_info_dict(post_boot) if post_boot is not None else None,
        "bootloader_version": _version_str(bootloader),
        "bootloader_note": ("booted over USB straight from the depthai library; no bootloader involved"
                            if bootloader is None else "booted through the on-device bootloader"),
        "embedded_bootloader_version": _version_str(embedded_bl),
        "device_firmware_embedded_in_library": str(getattr(dai, "__device_version__", "") or "") or None,
        "os_version": col.get("getOSVersion", device.getOSVersion, None, essential=False),
        "chip_temperature_c": ({
            "average": _round(temperature.average, 1), "css": _round(temperature.css, 1),
            "mss": _round(temperature.mss, 1), "upa": _round(temperature.upa, 1),
            "dss": _round(temperature.dss, 1),
        } if temperature is not None else None),
        "has_crash_dump": col.get("hasCrashDump", device.hasCrashDump, None, essential=False),
        "eeprom_available": col.get("isEepromAvailable", device.isEepromAvailable, None, essential=False),
        "ir_drivers": [{"name": str(d[0]), "i2c_bus": int(d[1]), "i2c_address": int(d[2])}
                       for d in ir_drivers] if ir_drivers else [],
        "capabilities": {
            "gpu": col.get("hasGPU", device.hasGPU, None, essential=False),
            "gpu_stereo": col.get("isGpuStereoSupported", device.isGpuStereoSupported, None, essential=False),
            "neural_depth": col.get("isNeuralDepthSupported", device.isNeuralDepthSupported, None,
                                    essential=False),
        },
    }

    imu_section: Dict[str, Any] = {
        "type": str(col.get("getConnectedIMU", device.getConnectedIMU, "") or "") or None,
        "firmware_version": _version_str(col.get("getIMUFirmwareVersion", device.getIMUFirmwareVersion, None)),
        "embedded_firmware_version": _version_str(col.get(
            "getEmbeddedIMUFirmwareVersion", device.getEmbeddedIMUFirmwareVersion, None, essential=False)),
    }

    cameras = _camera_dicts(device, col)

    pairs = col.get("getStereoPairs", device.getStereoPairs, [], essential=False)
    stereo_pairs = [{
        "left": _enum_name(p.left),
        "right": _enum_name(p.right),
        "baseline_mm": _round(float(p.baseline) * 10.0, 3),  # StereoPair.baseline is in cm
        "vertical": bool(p.isVertical),
    } for p in pairs]

    calibration = _calibration_dict(device, cameras, stereo_pairs, col)
    for cam in cameras:
        cam.pop("_socket", None)

    return {
        "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "host": {
            "platform": sys.platform,
            "python": "{}.{}.{}".format(*sys.version_info[:3]),
            "depthai": str(getattr(dai, "__version__", "?")),
            "depthai_commit": str(getattr(dai, "__commit__", "") or "") or None,
            "depthai_build_datetime": str(getattr(dai, "__build_datetime__", "") or "") or None,
        },
        "enumeration": [_device_info_dict(i) for i in enumeration],
        "device": device_section,
        "imu": imu_section,
        "cameras": cameras,
        "stereo_pairs": stereo_pairs,
        "calibration": calibration,
        "monitor": None,
        "errors": col.errors,
        "notes": col.notes,
    }


def sample_device(device: dai.Device, elapsed_s: float) -> Dict[str, Any]:
    """One monitor sample: chip temperature, USB speed and on-device memory usage."""
    col = _Collector()
    temp = col.get("getChipTemperature", device.getChipTemperature, None, essential=False)
    speed = col.get("getUsbSpeed", device.getUsbSpeed, None, essential=False)
    ddr = col.get("getDdrMemoryUsage", device.getDdrMemoryUsage, None, essential=False)
    cmx = col.get("getCmxMemoryUsage", device.getCmxMemoryUsage, None, essential=False)
    temp_max = None
    if temp is not None:
        temp_max = round(max(float(temp.css), float(temp.mss), float(temp.upa), float(temp.dss)), 1)
    return {
        "t_s": round(float(elapsed_s), 2),
        "usb_speed": _enum_name(speed),
        "chip_temp_c": _round(temp.average, 1) if temp is not None else None,
        "chip_temp_max_c": temp_max,
        "ddr_used_mib": _round(ddr.used / _MIB, 2) if ddr is not None else None,
        "ddr_total_mib": _round(ddr.total / _MIB, 2) if ddr is not None else None,
        "cmx_used_mib": _round(cmx.used / _MIB, 2) if cmx is not None else None,
        "cmx_total_mib": _round(cmx.total / _MIB, 2) if cmx is not None else None,
    }


# --------------------------------------------------------------------------------------
# Text rendering
# --------------------------------------------------------------------------------------

def _fmt_mm(value: Optional[float]) -> str:
    return "n/a" if value is None else "{:.2f} mm".format(value)


def _fmt_intrinsics(block: Optional[Dict[str, Any]]) -> str:
    if not block:
        return "n/a"
    return "@{}x{}  fx={:.1f} fy={:.1f} cx={:.1f} cy={:.1f}".format(
        block["width"], block["height"], block["fx"], block["fy"], block["cx"], block["cy"])


def _fmt_modes(modes: Sequence[Dict[str, Any]], multi_type: bool) -> str:
    seen: List[str] = []
    for m in modes:
        fps = m["max_fps"] if m["max_fps"] is not None else 0
        text = "{}x{}@{:g}".format(m["width"], m["height"], fps)
        if multi_type and m.get("type"):
            text += "({})".format(str(m["type"])[0])
        if text not in seen:
            seen.append(text)
    return ", ".join(seen) if seen else "n/a"


def _tri(value: Optional[bool], yes: str = "yes", no: str = "no") -> str:
    if value is None:
        return "?"
    return yes if value else no


def _or_unknown(value: Any) -> Any:
    """``'?'`` for ``None`` (a query that failed), otherwise the value unchanged."""
    return "?" if value is None else value


def format_report(report: Dict[str, Any]) -> str:
    """Render the report dict as the human-readable text block."""
    dev = report["device"]
    imu = report["imu"]
    host = report["host"]
    lines: List[str] = []
    add = lines.append

    add("=" * 78)
    add("OAK device report  (depthai {} on {} / Python {})".format(
        host["depthai"], host["platform"], host["python"]))
    add("=" * 78)
    if report["enumeration"]:
        for i, e in enumerate(report["enumeration"], start=1):
            add("Enumerated [{}]: id={} state={} protocol={} platform={} name={}".format(
                i, e["device_id"] or "?", e["state"], e["protocol"], e["platform"], e["name"] or "-"))
    else:
        add("Enumerated: (nothing before boot - the device appeared during the retry)")
    add("Device:       {} ({})  id={}  platform={}  open {:.1f} s (enumerate + boot)".format(
        dev["name"] or "?", dev["product_name"] or "?", dev["device_id"] or "?", dev["platform"] or "?",
        dev["open_time_s"]))
    usb_text = "{} ({})".format(dev["usb_speed"] or "?", "USB 3" if dev["usb3"] else "NOT USB 3 - WARNING")
    add("USB speed:    {}   interfaces: {}".format(
        usb_text, ", ".join(x for x in dev["connection_interfaces"] if x) or "?"))
    xl = dev.get("xlink_after_boot")
    if xl:
        add("XLink now:    state={} protocol={} platform={} name={}".format(
            xl["state"], xl["protocol"], xl["platform"], xl["name"] or "-"))
    add("Library:      depthai {}  commit {}  built {}".format(
        host["depthai"], (host["depthai_commit"] or "?")[:12], host["depthai_build_datetime"] or "?"))
    add("Device FW:    embedded RVC2 firmware {}  (loaded from the library at boot)".format(
        (dev["device_firmware_embedded_in_library"] or "?")[:12]))
    add("Bootloader:   {}  ({}; embedded bootloader in library: {})".format(
        dev["bootloader_version"] or "none", dev["bootloader_note"], dev["embedded_bootloader_version"] or "?"))
    if dev.get("os_version"):
        add("Device OS:    {}".format(dev["os_version"]))
    add("IMU:          {}  firmware {}  (embedded/upgradable: {})".format(
        imu["type"] or "none", imu["firmware_version"] or "?", imu["embedded_firmware_version"] or "?"))
    temp = dev.get("chip_temperature_c")
    add("Chip temp:    {}   crash dump on device: {}   EEPROM: {}   IR drivers: {}".format(
        "{:.1f} C".format(temp["average"]) if temp and temp.get("average") is not None else "n/a",
        _tri(dev.get("has_crash_dump"), "YES", "no"),
        _tri(dev.get("eeprom_available")),
        ", ".join(d["name"] for d in dev["ir_drivers"]) or "none"))

    add("")
    add("Cameras ({}):".format(len(report["cameras"])))
    for cam in report["cameras"]:
        types = ",".join(t for t in cam["supported_types"] if t) or "?"
        add("  {:<6} {:<8} {:>4}x{:<4} {:<11} autofocus={:<3} orientation={}".format(
            cam["socket"], cam["sensor"], cam["width"], cam["height"], types,
            "yes" if cam["has_autofocus"] else "no", cam["orientation"] or "?"))
        add("         modes: {}".format(_fmt_modes(cam["modes"], len(cam["supported_types"]) > 1)))

    add("")
    if report["stereo_pairs"]:
        add("Stereo pairs (device): " + "; ".join(
            "{}-{} baseline {} ({})".format(p["left"], p["right"], _fmt_mm(p["baseline_mm"]),
                                            "vertical" if p["vertical"] else "horizontal")
            for p in report["stereo_pairs"]))
    else:
        add("Stereo pairs (device): none")

    cal = report.get("calibration")
    add("")
    if not cal:
        add("Calibration: could not be read")
    else:
        ee = cal.get("eeprom") or {}
        add("Calibration (EEPROM v{}, board {} rev {}, product {}, has_data={}):".format(
            ee.get("version", "?"), ee.get("board_name") or "?", ee.get("board_rev") or "?",
            ee.get("product_name") or "?", cal["has_calibration_data"]))
        st = cal.get("stereo")
        if st:
            add("  stereo: left={} right={}  baseline spec {} / calibrated {}".format(
                st["left"], st["right"], _fmt_mm(st["baseline_mm_spec"]), _fmt_mm(st["baseline_mm_calibrated"])))
        else:
            add("  stereo: n/a")
        for name, entry in cal["cameras"].items():
            add("  {:<6} native {}".format(name, _fmt_intrinsics(entry.get("native"))))
            cal_at = entry.get("calibrated_at")
            native = entry.get("native")
            if cal_at and native and (cal_at["width"], cal_at["height"]) != (native["width"], native["height"]):
                add("         calibrated {}".format(_fmt_intrinsics(cal_at)))
            coeffs = entry.get("distortion_coefficients")
            extra = "         fov {} deg (spec) / {} deg (intrinsics)  distortion {} ({} coeffs)".format(
                _or_unknown(entry.get("fov_deg_spec")), _or_unknown(entry.get("fov_deg_from_intrinsics")),
                entry.get("distortion_model") or "?", len(coeffs) if coeffs is not None else "?")
            if "lens_position" in entry:
                extra += "  lens position {}".format(_or_unknown(entry.get("lens_position")))
            add(extra)

    mon = report.get("monitor")
    if mon and mon.get("samples"):
        temps = [s["chip_temp_c"] for s in mon["samples"] if s.get("chip_temp_c") is not None]
        add("")
        add("Monitor: {} samples over {:.1f} s{}, chip temp {}..{} C, USB {}".format(
            len(mon["samples"]), mon["duration_s"],
            " (stopped early by Ctrl-C)" if mon.get("interrupted") else "",
            min(temps) if temps else "?", max(temps) if temps else "?",
            "/".join(sorted(set(str(s["usb_speed"]) for s in mon["samples"])))))

    if report["notes"]:
        add("")
        add("Notes (optional APIs not available here):")
        for note in report["notes"]:
            add("  - " + note)
    if report["errors"]:
        add("")
        add("ERRORS (essential queries that failed):")
        for err in report["errors"]:
            add("  - " + err)
    return "\n".join(lines)


def _format_sample(sample: Dict[str, Any]) -> str:
    def num(value: Any, fmt: str = "{:.1f}") -> str:
        return "n/a" if value is None else fmt.format(value)

    return "[{:6.1f} s] chip {} C (max {})  usb={}  ddr {}/{} MiB  cmx {}/{} MiB".format(
        sample["t_s"], num(sample["chip_temp_c"]), num(sample["chip_temp_max_c"]),
        sample["usb_speed"] or "?",
        num(sample["ddr_used_mib"], "{:.2f}"), num(sample["ddr_total_mib"], "{:.2f}"),
        num(sample["cmx_used_mib"], "{:.2f}"), num(sample["cmx_total_mib"], "{:.2f}"))


# --------------------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------------------

def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Enumerate the connected OAK device and print a full report: identity, USB speed, "
                    "firmware, IMU, camera sockets and calibration (stereo baseline, intrinsics).",
        epilog="Exit codes: 0 ok, 1 essential query failed, 2 no device / wrong id / busy, 130 Ctrl-C. "
               "--duration N keeps the device open for N seconds and logs chip temperature / USB "
               "speed / memory once per second (a link-stability soak); --headless changes nothing "
               "here because this script never opens a window.")
    add_device_arg(parser)
    add_run_args(parser)
    parser.add_argument(
        "--json",
        action="store_true",
        help="print the report as a JSON document on stdout (everything else goes to stderr)",
    )
    parser.add_argument(
        "--dump-calibration",
        dest="dump_calibration",
        metavar="FILE",
        default=None,
        help="also write the full EEPROM calibration (CalibrationHandler.eepromToJson) to FILE",
    )
    return parser


def _dump_calibration(device: dai.Device, path: Path) -> Optional[str]:
    """Write ``eepromToJson()`` to *path*; returns an error message or ``None``."""
    try:
        data = device.readCalibration().eepromToJson()
        ensure_dir(path.parent)
        path.write_text(json.dumps(data, indent=2, default=str), encoding="utf-8")
        return None
    except Exception as exc:  # noqa: BLE001 - reported, never fatal
        return "could not write calibration dump {}: {}: {}".format(path, type(exc).__name__, _one_line(exc))


def _run_monitor(device: dai.Device, clock: RunClock, json_mode: bool) -> Dict[str, Any]:
    """Hold the device open until *clock* expires, sampling once per second.

    Ctrl-C ends the loop early: the samples gathered so far are returned with
    ``interrupted=True`` (``main`` turns that into exit 130) instead of being discarded.
    """
    samples: List[Dict[str, Any]] = []
    interrupted = False
    tick = Periodic(1.0, first_immediately=True)
    if not json_mode:
        print("\nMonitoring for {:.1f} s (Ctrl-C to stop early)...".format(clock.duration), flush=True)
    try:
        while not clock.expired():
            if tick.ready():
                sample = sample_device(device, clock.elapsed())
                samples.append(sample)
                if not json_mode:
                    print(_format_sample(sample), flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        interrupted = True
    return {"duration_s": round(clock.elapsed(), 2), "samples": samples, "interrupted": interrupted}


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    json_mode = bool(args.json)
    # --json: from here on (normally since module import, see _JSON_STDOUT_FD) descriptor 1 is
    # stderr, so everything we or the depthai library print lands there; only the final JSON
    # document goes to the real stdout, through json_fd.
    json_fd = _json_stdout_fd() if json_mode else None
    report: Optional[Dict[str, Any]] = None
    exit_code = EXIT_OK
    device: Optional[dai.Device] = None
    try:
        enumeration = list_devices()  # the pre-boot picture for the report (open_device enumerates again)
        t0 = time.monotonic()
        # An explicit --device-id never retries: one stderr line even when nothing enumerates.
        # Without an id the open is retried while a camera reboots after another script.
        device = open_device(args.device_id, retries=1 if args.device_id else 3)  # exits 2 on failure
        open_s = time.monotonic() - t0
        print_device_summary(device)  # one-line summary + the stderr warning when not USB 3

        report = collect_report(device, enumeration, open_s)
        if args.dump_calibration:
            dump_path = Path(args.dump_calibration)
            problem = _dump_calibration(device, dump_path)
            if problem:
                report["errors"].append(problem)
            else:
                report["calibration_dump_file"] = str(dump_path.resolve())
                if not json_mode:
                    print("Calibration written to {}".format(report["calibration_dump_file"]), flush=True)
        if not json_mode:
            print(format_report(report), flush=True)
        if args.duration > 0:
            monitor = _run_monitor(device, RunClock(args.duration), json_mode)
            report["monitor"] = monitor
            if monitor["interrupted"]:
                print("Interrupted (Ctrl-C): monitor stopped early, keeping the {} sample(s) taken.".format(
                    len(monitor["samples"])), file=sys.stderr, flush=True)
                exit_code = EXIT_INTERRUPTED
            if not json_mode:
                print("Monitor finished: {} samples in {:.1f} s{}.".format(
                    len(monitor["samples"]), monitor["duration_s"],
                    " (stopped early by Ctrl-C)" if monitor["interrupted"] else ""), flush=True)
    except KeyboardInterrupt:
        print("Interrupted (Ctrl-C).", file=sys.stderr, flush=True)
        exit_code = EXIT_INTERRUPTED
    except Exception as exc:  # noqa: BLE001 - e.g. the device dropped off the bus mid-report
        print("Device error: {}: {}".format(type(exc).__name__, _one_line(exc)), file=sys.stderr, flush=True)
        exit_code = EXIT_ERROR
    finally:
        if device is not None:
            try:
                device.close()
            except Exception:  # noqa: BLE001 - closing a dead device must not mask the real outcome
                pass

    if report is not None:
        if report["errors"]:
            for err in report["errors"]:
                print("ERROR: " + err, file=sys.stderr, flush=True)
            if exit_code == EXIT_OK:
                exit_code = EXIT_ERROR
        if json_mode:
            _emit_json(report, json_fd)
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
