"""Test double for NVIDIA's ``cuvslam`` package (PyCuVSLAM v17.0.0) for hosts without the wheel.

``cuvslam`` only ships Linux/CUDA wheels, so on this Windows laptop nothing that touches the
tracker could be tested.  This module is a drop-in stand-in with the **same public surface** as
``cuvslam`` 17.0.0 (see ``methods/cuvslam-vio/docs/cuvslam-api-v17.md``, section 10 "Fake module contract"):

* the same names, constructor signatures (positional vs keyword-only), attribute names, numpy
  dtypes/shapes and defaults as the nanobind extension (``python/cuvslam2.cpp``, verified
  against the wheel's ``pycuvslam.pyi`` and NVIDIA's own ``test_bindings.py``);
* the same copy-vs-view semantics: ``Pose.rotation``/``translation`` and ``Camera.size``/
  ``principal``/``focal`` are live numpy views (element writes persist), while ``list``-typed
  attributes (``Distortion.parameters``, ``ImuMeasurement.*``, ``Landmark.coords``,
  ``Rig.cameras``) are copies (``rig.cameras.append(...)`` does not persist, but
  ``rig.cameras[1].size[0] = 480`` does, exactly like the real binding);
* the same ``ValueError``/``TypeError`` behaviour for bad rigs, arrays and timestamps:
  wrong-length sequences assigned to the ndarray-typed fields
  (``ValueError('Sequence must have exactly N elements')``), positional arguments to
  keyword-only or argument-less constructors (``TypeError``; ``PoseStamped()``,
  ``Observation()``, ``Landmark()`` and the ``Slam.*`` records take no arguments, exactly like
  the binding: set their attributes afterwards), empty rigs, mixed camera sizes, negative
  sizes, IMU count vs mode, non-``uint8`` / non-contiguous / wrongly shaped images,
  mask/image count mismatch, non-increasing frame timestamps, IMU samples out of order;
* a ``Tracker`` whose logic is the v17.0.0 ``python/tracker.py`` verbatim (same aliases, same
  export-flag mutation, same ``(PoseEstimate, Pose | None)`` return, same pass-throughs).

What is synthetic (there is no vision here):

* ``Odometry.track`` returns a deterministic trajectory from an injectable **pose source**.  The
  default :class:`ConstantVelocityModel` moves the rig straight ahead (+z, OpenCV convention) at
  ``DEFAULT_VELOCITY_MPS`` (0.1 m/s) integrated from the frame timestamps, restarting from the
  identity whenever tracking is (re)initialised, so tests can predict the TUM trajectory exactly.
  Assign ``odom.pose_source = fn`` or call :func:`set_default_pose_source` to script trajectories
  and dropouts.  ``fn`` is called as ``fn(timestamp_ns, origin_ns)``; a one-argument
  ``fn(timestamp_ns)`` or a zero-argument callable is adapted automatically (the arity is
  inspected once, on assignment).  It returns a ``Pose`` or ``None`` (= tracking failed for that
  frame, ``world_from_rig is None``).  NVIDIA's own ``python/test/test_tracking.py`` asserts
  zero motion on blank frames, so run it against the fake only after
  ``set_default_pose_source(lambda ts, origin: Pose())``.
* The number of synthetic observations per camera and the covariance of a returned pose are
  scriptable the same way (``odom.observations_source`` / ``odom.covariance_source``, module-wide
  :func:`set_default_observations_source` / :func:`set_default_covariance_source`): callables
  ``fn(timestamp_ns) -> int | None`` and ``fn(timestamp_ns) -> Sequence[float] (36) | None``
  (a zero-argument callable is adapted; ``None`` = the default grid / the default diagonal), so
  the starvation guard of ``cuvslam_oak.tracking`` can be exercised without the real engine.
* A frame gap larger than ``Config.max_frame_delta_s`` is treated as a tracking loss (``None``
  pose), after which the world frame restarts at the next frame and ``odom.resets`` grows.
* Observations/landmarks are a fixed grid inside each camera's border mask; gravity becomes
  ``[0, 9.81, 0]`` after a few IMU samples; SLAM is a pass-through that remembers the poses.
* Call bookkeeping for tests: module-level ``call_counts`` (a ``Counter``), per-instance
  ``odom.calls``, ``odom.frame_timestamps``, ``odom.poses``, ``odom.imu_log``, ``odom.resets``,
  and :func:`last_odometry` to reach the tracker a script created internally.

Deviations that are deliberate:

* ``Odometry`` and ``Slam`` are exposed at top level here for convenience (``TOP_LEVEL_EXTRAS``),
  whereas the real ``cuvslam`` 17.0.0 only offers them as ``cuvslam.core.Odometry`` /
  ``cuvslam.core.Slam`` or through the ``Tracker.*`` aliases.  Portable code must use the
  latter; a contract test can skip these two names.
* Only CPU ``numpy.ndarray`` images are accepted (the real one also takes DLPack/CUDA tensors).
* ``get_version()`` returns ``("17.0.0+fake", 17, 0, 0)`` and ``warm_up_gpu()`` is a no-op.
* ``Camera()`` with its all-zero default ``size`` is accepted by ``Odometry`` as "size not set"
  and the image-shape check is skipped for that camera.  The real constructor is documented to
  reject non-positive sizes (upstream pins only ``-1`` -> ``ValueError``), so tests that must
  also run on Linux build rigs with an explicit ``size=(w, h)``.
* ``Odometry.get_state()`` never raises: it returns the last snapshot (``frame_id == -1`` before
  the first valid frame; empty observation/landmark lists when the exports are off).  The
  binding fills a ``State`` unconditionally and documents no error, so none is asserted here.
* Wrong-length values for the ``list``-typed 3-vectors (``ImuMeasurement.linear_accelerations``
  / ``angular_velocities``, ``Landmark.coords``, ``Slam.Landmark.coords``) raise ``TypeError``,
  mirroring nanobind's ``std::array`` caster on a ``def_rw`` member (the ndarray-typed fields
  raise ``ValueError`` from ``bind_array_accessors``).  Upstream tests pin only the
  ``ValueError`` side; a cross-implementation test should accept ``(TypeError, ValueError)``.
* ``core`` is additionally registered as the importable submodule ``cuvslam_oak.fake_cuvslam.core``
  so ``from cuvslam_oak.fake_cuvslam.core import Odometry`` works like ``from cuvslam.core import Odometry``.

Never import this module directly from production code: go through
``cuvslam_oak.import_cuvslam(prefer_fake=...)`` so the real wheel is used whenever it imports.
Python 3.10+, numpy >= 1.24 (tested with 2.x); no other dependencies.
"""

from __future__ import annotations

import collections
import copy
import enum
import importlib.machinery
import inspect
import json
import math
import numbers
import sys
import types
import weakref
from pathlib import Path
from typing import Any, Callable, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import numpy as np

__all__ = [
    # real cuvslam top-level names
    "get_version", "set_verbosity", "warm_up_gpu",
    "Pose", "Distortion", "Camera", "ImuCalibration", "ImuMeasurement", "Rig",
    "PoseStamped", "PoseWithCovariance", "PoseEstimate", "Observation", "Landmark",
    "Tracker", "core",
    # extras of the fake
    "IS_FAKE", "TOP_LEVEL_EXTRAS", "Odometry", "Slam",
    "ConstantVelocityModel", "DEFAULT_VELOCITY_MPS", "GRAVITY_MPS2",
    "call_counts", "reset_call_counts", "set_default_pose_source", "get_default_pose_source",
    "set_default_observations_source", "get_default_observations_source",
    "set_default_covariance_source", "get_default_covariance_source",
    "instances", "last_odometry",
]

#: Marks this module as the test double (``cuvslam_oak.is_fake`` reads it).
IS_FAKE = True

#: Same value as the real package: ``get_version()[0].split('+')[0]``.
__version__ = "17.0.0"
VERSION_STRING = "17.0.0+fake"

#: Names exported here that the real ``cuvslam`` 17.0.0 does NOT export at top level.
TOP_LEVEL_EXTRAS = ("Odometry", "Slam")

#: Forward speed of the default synthetic trajectory (metres per second along +z).
DEFAULT_VELOCITY_MPS = 0.1
#: Gravity magnitude reported by ``get_last_gravity`` once "estimated".
GRAVITY_MPS2 = 9.81
#: IMU samples and tracked frames needed before ``get_last_gravity`` stops returning ``None``.
GRAVITY_MIN_IMU_SAMPLES = 10
GRAVITY_MIN_FRAMES = 2
#: Every n-th tracked frame is reported as a keyframe (the first frame after a restart always is).
KEYFRAME_EVERY = 5
#: Synthetic observations form an N x N grid inside the camera's border mask (36 per camera:
#: comfortably above any sensible ``TrackConfig.min_observations``; script fewer with
#: ``observations_source``).
OBSERVATION_GRID = 6
#: Depth (metres, camera z) at which synthetic landmarks are placed.
LANDMARK_DEPTH_M = 2.0
#: File written by ``Slam.save_map`` so ``localize_in_map`` can recognise a saved map.
MAP_MARKER_FILE = "fake_cuvslam_map.json"

#: Global call counter: keys such as ``'get_version'``, ``'Odometry.track'``, ``'Slam.save_map'``.
call_counts: "collections.Counter[str]" = collections.Counter()

_verbosity: List[int] = [0]
_default_pose_source: List[Optional["PoseSource"]] = [None]
_default_observations_source: List[Optional[Callable[..., Any]]] = [None]
_default_covariance_source: List[Optional[Callable[..., Any]]] = [None]
_instances: List["weakref.ReferenceType[Odometry]"] = []


def reset_call_counts() -> None:
    """Clear :data:`call_counts` (tests call this in a fixture)."""
    call_counts.clear()


def _count(key: str) -> None:
    call_counts[key] += 1


def _log(level: int, message: str) -> None:
    """Print *message* to stderr when ``set_verbosity`` is at least *level* (1 errors, 2 warnings, 3 info)."""
    if _verbosity[0] >= level:
        print("[fake_cuvslam] " + message, file=sys.stderr, flush=True)


# ---------------------------------------------------------------------------------------------
# Value converters (mimic nanobind type casters: TypeError for the wrong kind of object)
# ---------------------------------------------------------------------------------------------

def _is_bool(value: Any) -> bool:
    return isinstance(value, (bool, np.bool_))


def _as_bool(value: Any) -> bool:
    if _is_bool(value):
        return bool(value)
    raise TypeError("expected a bool, got {}".format(type(value).__name__))


def _as_int(value: Any) -> int:
    if _is_bool(value) or not isinstance(value, numbers.Integral):
        raise TypeError("expected an int, got {}".format(type(value).__name__))
    return int(value)


def _as_float(value: Any) -> float:
    if _is_bool(value) or not isinstance(value, numbers.Real):
        raise TypeError("expected a float, got {}".format(type(value).__name__))
    return float(value)


def _as_str(value: Any) -> str:
    if not isinstance(value, str):
        raise TypeError("expected a str, got {}".format(type(value).__name__))
    return value


def _is_sequence(value: Any) -> bool:
    """Python-level equivalent of nanobind's ``nb::sequence`` check (no str/bytes/dict/ndarray)."""
    if isinstance(value, (str, bytes, bytearray, dict, np.ndarray)):
        return False
    return hasattr(value, "__len__") and hasattr(value, "__getitem__")


def _as_enum(enum_cls: type) -> Callable[[Any], Any]:
    def convert(value: Any) -> Any:
        if not isinstance(value, enum_cls):
            raise TypeError("expected {}, got {}".format(enum_cls.__qualname__, type(value).__name__))
        return value
    return convert


def _as_instance(cls: type) -> Callable[[Any], Any]:
    """Value semantics like a C++ struct member: type-checked and copied on assignment."""
    def convert(value: Any) -> Any:
        if not isinstance(value, cls):
            raise TypeError("expected {}, got {}".format(cls.__qualname__, type(value).__name__))
        return copy.deepcopy(value)
    return convert


def _as_optional_instance(cls: type) -> Callable[[Any], Any]:
    inner = _as_instance(cls)

    def convert(value: Any) -> Any:
        return None if value is None else inner(value)
    return convert


def _as_list_of(cls: type) -> Callable[[Any], List[Any]]:
    """``std::vector<T>`` of a bound class: any iterable of *cls* instances, elements copied."""
    def convert(value: Any) -> List[Any]:
        if isinstance(value, (str, bytes, bytearray, dict)) or not hasattr(value, "__iter__"):
            raise TypeError("expected a sequence of {}, got {}".format(cls.__qualname__, type(value).__name__))
        items = list(value)
        for item in items:
            if not isinstance(item, cls):
                raise TypeError("expected a sequence of {}, got an element of type {}".format(
                    cls.__qualname__, type(item).__name__))
        return [copy.deepcopy(item) for item in items]
    return convert


def _items_of(value: Any, what: str) -> List[Any]:
    if isinstance(value, np.ndarray):
        if value.ndim != 1:
            raise TypeError("{} must be a 1-D array or sequence, got an array of shape {}".format(
                what, value.shape))
        return value.tolist()
    if _is_sequence(value):
        return list(value)
    raise TypeError("{} must be a sequence or 1-D ndarray, got {}".format(what, type(value).__name__))


def _as_float_list(value: Any, n: Optional[int] = None, what: str = "value") -> List[float]:
    """``std::vector<float>`` (any length) or ``std::array<float, n>`` (exact length, TypeError otherwise)."""
    items = _items_of(value, what)
    if n is not None and len(items) != n:
        raise TypeError("{} must have exactly {} elements, got {}".format(what, n, len(items)))
    return [_as_float(x) for x in items]


def _as_int_list(value: Any, what: str = "value") -> List[int]:
    return [_as_int(x) for x in _items_of(value, what)]


def _as_optional_float3(value: Any) -> Optional[List[float]]:
    return None if value is None else _as_float_list(value, 3, "gravity")


def _take_kwargs(cls_name: str, kwargs: Dict[str, Any], required: Sequence[str],
                 optional: Mapping[str, Any]) -> Dict[str, Any]:
    """Validate a keyword-only constructor call the way nanobind does (TypeError on trouble)."""
    for key in kwargs:
        if key not in required and key not in optional:
            raise TypeError("{}() got an unexpected keyword argument '{}'".format(cls_name, key))
    for key in required:
        if key not in kwargs:
            raise TypeError("{}() missing required keyword-only argument '{}'".format(cls_name, key))
    result = dict(optional)
    result.update(kwargs)
    return result


# ---------------------------------------------------------------------------------------------
# Attribute descriptors (``def_rw`` and ``bind_array_accessors`` equivalents)
# ---------------------------------------------------------------------------------------------

class _Attr:
    """A typed read-write attribute stored in the slot ``'_' + name``.

    ``copy_get=True`` returns ``list(value)`` from the getter, reproducing nanobind's behaviour
    for ``std::vector``/``std::array`` members: element assignment on the result is lost.
    """

    __slots__ = ("convert", "copy_get", "name", "slot")

    def __init__(self, convert: Callable[[Any], Any], *, copy_get: bool = False) -> None:
        self.convert = convert
        self.copy_get = copy_get
        self.name = ""
        self.slot = ""

    def __set_name__(self, owner: type, name: str) -> None:
        self.name = name
        self.slot = "_" + name

    def __get__(self, obj: Any, objtype: Optional[type] = None) -> Any:
        if obj is None:
            return self
        value = getattr(obj, self.slot)
        return list(value) if self.copy_get else value

    def __set__(self, obj: Any, value: Any) -> None:
        setattr(obj, self.slot, self.convert(value))


class _ArrayAttr:
    """A fixed-size numpy array attribute with the real binding's ``bind_array_accessors`` rules.

    The getter returns the live array (so ``pose.translation[0] = 9`` persists); the setter
    accepts a 1-D ndarray or any sequence of exactly *n* elements (``ValueError('Sequence must
    have exactly N elements')`` otherwise) and copies the values into that same array.
    """

    __slots__ = ("dtype", "elem", "n", "name", "slot")

    def __init__(self, dtype: Any, n: int) -> None:
        self.dtype = np.dtype(dtype)
        self.n = n
        self.elem = _as_int if self.dtype.kind in "iu" else _as_float
        self.name = ""
        self.slot = ""

    def __set_name__(self, owner: type, name: str) -> None:
        self.name = name
        self.slot = "_" + name

    def new_array(self, values: Optional[Sequence[float]] = None) -> np.ndarray:
        arr = np.zeros(self.n, dtype=self.dtype)
        if values is not None:
            arr[...] = np.asarray(values, dtype=self.dtype)
        return arr

    def coerce(self, value: Any) -> np.ndarray:
        if isinstance(value, np.ndarray):
            if value.ndim != 1:
                raise TypeError("Data must be an np.array or sequence of size {}. Array must be 1D with "
                                "element type {}".format(self.n, self.dtype.name))
            if value.shape[0] != self.n:
                raise ValueError("Sequence must have exactly {} elements".format(self.n))
            if value.dtype == self.dtype:
                return value
            items = value.tolist()
        elif _is_sequence(value):
            if len(value) != self.n:
                raise ValueError("Sequence must have exactly {} elements".format(self.n))
            items = list(value)
        else:
            raise TypeError("Data must be an np.array or sequence of size {}. Array must be 1D with "
                            "element type {}".format(self.n, self.dtype.name))
        return np.array([self.elem(x) for x in items], dtype=self.dtype)

    def __get__(self, obj: Any, objtype: Optional[type] = None) -> Any:
        if obj is None:
            return self
        return getattr(obj, self.slot)

    def __set__(self, obj: Any, value: Any) -> None:
        values = self.coerce(value)
        arr = getattr(obj, self.slot, None)
        if arr is None:
            arr = self.new_array()
            setattr(obj, self.slot, arr)
        arr[...] = values


def _fmt(value: Any) -> str:
    if isinstance(value, np.ndarray):
        return repr(value.tolist())
    if isinstance(value, float):
        return repr(round(value, 6))
    return repr(value)


class _Struct:
    """Base for value-type classes: slot-based deepcopy and a ``cuvslam.X(...)`` repr."""

    __slots__ = ()

    def _all_slots(self) -> Iterable[str]:
        for cls in type(self).__mro__:
            for slot in getattr(cls, "__slots__", ()):
                if slot != "__weakref__":
                    yield slot

    def __deepcopy__(self, memo: Dict[int, Any]) -> "_Struct":
        new = object.__new__(type(self))
        memo[id(self)] = new
        for slot in self._all_slots():
            if hasattr(self, slot):
                setattr(new, slot, copy.deepcopy(getattr(self, slot), memo))
        return new

    def __copy__(self) -> "_Struct":
        return copy.deepcopy(self)

    def __repr__(self) -> str:
        fields = []
        for slot in self._all_slots():
            if not hasattr(self, slot) or slot.startswith("_fake_"):
                continue
            fields.append("{}={}".format(slot.lstrip("_"), _fmt(getattr(self, slot))))
        return "cuvslam.{}({})".format(type(self).__qualname__, ", ".join(fields))


# ---------------------------------------------------------------------------------------------
# Free functions
# ---------------------------------------------------------------------------------------------

def get_version() -> Tuple[str, int, int, int]:
    """``(semver_string, major, minor, patch)``; the string starts with ``'17.0.0+'`` like the real one."""
    _count("get_version")
    return (VERSION_STRING, 17, 0, 0)


def set_verbosity(level, /) -> None:
    """Set the log level: 0 silent (default), 1 errors, 2 warnings, 3 info (positional-only, like the binding)."""
    _count("set_verbosity")
    _verbosity[0] = _as_int(level)


def warm_up_gpu() -> None:
    """No-op stand-in for the CUDA/cuBLAS/cuSOLVER warm-up (the real one throws if CUDA is broken)."""
    _count("warm_up_gpu")
    _log(3, "warm_up_gpu(): nothing to do in the fake")


# ---------------------------------------------------------------------------------------------
# Quaternion / pose helpers (x, y, z, w order, OpenCV axes)
# ---------------------------------------------------------------------------------------------

Vec3 = Tuple[float, float, float]
Quat = Tuple[float, float, float, float]


def _quat_mul(q: Quat, r: Quat) -> Quat:
    x1, y1, z1, w1 = q
    x2, y2, z2, w2 = r
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


def _quat_conj(q: Quat) -> Quat:
    return (-q[0], -q[1], -q[2], q[3])


def _quat_rotate(q: Quat, v: Vec3) -> Vec3:
    r = _quat_mul(_quat_mul(q, (v[0], v[1], v[2], 0.0)), _quat_conj(q))
    return (r[0], r[1], r[2])


def _pose_parts(pose: "Pose") -> Tuple[Quat, Vec3]:
    q = tuple(float(x) for x in pose.rotation)
    t = tuple(float(x) for x in pose.translation)
    return q, t  # type: ignore[return-value]


def _pose_compose(a_from_b: "Pose", b_from_c: "Pose") -> "Pose":
    qa, ta = _pose_parts(a_from_b)
    qb, tb = _pose_parts(b_from_c)
    rt = _quat_rotate(qa, tb)
    return Pose(_quat_mul(qa, qb), (rt[0] + ta[0], rt[1] + ta[1], rt[2] + ta[2]))


def _pose_inverse(a_from_b: "Pose") -> "Pose":
    q, t = _pose_parts(a_from_b)
    qi = _quat_conj(q)
    rt = _quat_rotate(qi, t)
    return Pose(qi, (-rt[0], -rt[1], -rt[2]))


def _relative_pose(world_from_a: "Pose", world_from_b: "Pose") -> "Pose":
    """``a_from_b`` given both poses in the world frame."""
    return _pose_compose(_pose_inverse(world_from_a), world_from_b)


# ---------------------------------------------------------------------------------------------
# Data classes (section 2 of methods/cuvslam-vio/docs/cuvslam-api-v17.md)
# ---------------------------------------------------------------------------------------------

class Pose(_Struct):
    """Transformation from one frame to another (quaternion x, y, z, w + translation, OpenCV axes).

    ``Pose()`` is the identity; ``Pose(rotation, translation)`` positional or keyword, this order.
    Both attributes are live ``float32`` views: ``p.translation[0] = 9`` mutates the pose.
    """

    __slots__ = ("_rotation", "_translation")

    rotation = _ArrayAttr(np.float32, 4)
    translation = _ArrayAttr(np.float32, 3)

    def __init__(self, rotation: Optional[Sequence[float]] = None,
                 translation: Optional[Sequence[float]] = None) -> None:
        self._rotation = Pose.rotation.new_array((0.0, 0.0, 0.0, 1.0))
        self._translation = Pose.translation.new_array()
        if (rotation is None) != (translation is None):
            raise TypeError("Pose() takes either no arguments or both rotation and translation")
        if rotation is not None:
            self.rotation = rotation
            self.translation = translation

    def __str__(self) -> str:
        return "(rotation={}, translation={})".format(_fmt(self._rotation), _fmt(self._translation))

    def __repr__(self) -> str:
        return "cuvslam.Pose" + str(self)


class _DistortionModel(enum.Enum):
    """Distortion model types for camera calibration."""
    Pinhole = 0      # no parameters
    Fisheye = 1      # k1, k2, k3, k4 (equidistant, OpenCV fisheye)
    Brown = 2        # k1, k2, k3, p1, p2 (NOT OpenCV order)
    Polynomial = 3   # k1, k2, p1, p2, k3, k4, k5, k6 = first 8 OpenCV coefficients (rational)


class Distortion(_Struct):
    """Camera distortion model with parameters; ``parameters`` is a copy (assign the whole list)."""

    __slots__ = ("_model", "_parameters")

    Model = _DistortionModel
    Pinhole = _DistortionModel.Pinhole
    Fisheye = _DistortionModel.Fisheye
    Brown = _DistortionModel.Brown
    Polynomial = _DistortionModel.Polynomial

    model = _Attr(_as_enum(_DistortionModel))
    parameters = _Attr(lambda v: _as_float_list(v, None, "Distortion.parameters"), copy_get=True)

    def __init__(self, model: Optional[_DistortionModel] = None,
                 parameters: Optional[Sequence[float]] = None) -> None:
        self._model = _DistortionModel.Pinhole
        self._parameters: List[float] = []
        if model is None:
            if parameters is not None:
                raise TypeError("Distortion(): 'model' is required when 'parameters' is given")
            return
        self.model = model
        self.parameters = [] if parameters is None else parameters

    def __str__(self) -> str:
        return "(model={}, parameters={})".format(self._model, _fmt(self._parameters))

    def __repr__(self) -> str:
        return "cuvslam.Distortion" + str(self)


_DistortionModel.__name__ = "Model"
_DistortionModel.__qualname__ = "Distortion.Model"

_CAMERA_REQUIRED = ("size", "principal", "focal")
_CAMERA_OPTIONAL: Dict[str, Any] = {"rig_from_camera": None, "distortion": None, "border_top": 0,
                                    "border_bottom": 0, "border_left": 0, "border_right": 0}


class Camera(_Struct):
    """Camera calibration: ``size`` (width, height) int32, ``principal`` (cx, cy), ``focal`` (fx, fy).

    ``Camera()`` is all zeros; the other constructor is keyword-only with ``size``, ``principal``
    and ``focal`` required (positional arguments raise ``TypeError``).  Image origin is the top-left
    pixel, x right, y down; ``rig_from_camera`` maps camera-frame points into the rig frame.
    """

    __slots__ = ("_size", "_principal", "_focal", "_rig_from_camera", "_distortion",
                 "_border_top", "_border_bottom", "_border_left", "_border_right")

    size = _ArrayAttr(np.int32, 2)
    principal = _ArrayAttr(np.float32, 2)
    focal = _ArrayAttr(np.float32, 2)
    rig_from_camera = _Attr(_as_instance(Pose))
    distortion = _Attr(_as_instance(Distortion))
    border_top = _Attr(_as_int)
    border_bottom = _Attr(_as_int)
    border_left = _Attr(_as_int)
    border_right = _Attr(_as_int)

    def __init__(self, **kwargs: Any) -> None:
        self._size = Camera.size.new_array()
        self._principal = Camera.principal.new_array()
        self._focal = Camera.focal.new_array()
        self._rig_from_camera = Pose()
        self._distortion = Distortion()
        self._border_top = self._border_bottom = self._border_left = self._border_right = 0
        if not kwargs:
            return
        args = _take_kwargs("Camera", kwargs, _CAMERA_REQUIRED, _CAMERA_OPTIONAL)
        self.size = args["size"]
        self.principal = args["principal"]
        self.focal = args["focal"]
        if args["rig_from_camera"] is not None:
            self.rig_from_camera = args["rig_from_camera"]
        if args["distortion"] is not None:
            self.distortion = args["distortion"]
        self.border_top = args["border_top"]
        self.border_bottom = args["border_bottom"]
        self.border_left = args["border_left"]
        self.border_right = args["border_right"]

    def __str__(self) -> str:
        return "(size={}, principal={}, focal={}, rig_from_cam={}, distortion={})".format(
            _fmt(self._size), _fmt(self._principal), _fmt(self._focal), self._rig_from_camera,
            self._distortion)

    def __repr__(self) -> str:
        return "cuvslam.Camera" + str(self)


_IMU_CALIB_REQUIRED = ("gyroscope_noise_density", "gyroscope_random_walk",
                       "accelerometer_noise_density", "accelerometer_random_walk", "frequency")
_IMU_CALIB_OPTIONAL: Dict[str, Any] = {"rig_from_imu": None}


class ImuCalibration(_Struct):
    """IMU calibration: ``rig_from_imu`` plus noise densities / random walks and ``frequency`` [Hz].

    Units: gyro ``rad/(s*sqrt(Hz))`` and ``rad/(s^2*sqrt(Hz))``, accel ``m/(s^2*sqrt(Hz))`` and
    ``m/(s^3*sqrt(Hz))``.  The keyword constructor requires the five numeric fields.
    """

    __slots__ = ("_rig_from_imu", "_gyroscope_noise_density", "_gyroscope_random_walk",
                 "_accelerometer_noise_density", "_accelerometer_random_walk", "_frequency")

    rig_from_imu = _Attr(_as_instance(Pose))
    gyroscope_noise_density = _Attr(_as_float)
    gyroscope_random_walk = _Attr(_as_float)
    accelerometer_noise_density = _Attr(_as_float)
    accelerometer_random_walk = _Attr(_as_float)
    frequency = _Attr(_as_float)

    def __init__(self, **kwargs: Any) -> None:
        self._rig_from_imu = Pose()
        self._gyroscope_noise_density = 0.0
        self._gyroscope_random_walk = 0.0
        self._accelerometer_noise_density = 0.0
        self._accelerometer_random_walk = 0.0
        self._frequency = 0.0
        if not kwargs:
            return
        args = _take_kwargs("ImuCalibration", kwargs, _IMU_CALIB_REQUIRED, _IMU_CALIB_OPTIONAL)
        if args["rig_from_imu"] is not None:
            self.rig_from_imu = args["rig_from_imu"]
        for key in _IMU_CALIB_REQUIRED:
            setattr(self, key, args[key])

    def __repr__(self) -> str:
        return "cuvslam.ImuCalibration(rig_from_imu={})".format(self._rig_from_imu)


class Rig(_Struct):
    """Rig of cameras and 0 or 1 IMU.  Camera index = position in ``cameras`` = image order in ``track``.

    The list attributes are fresh lists on every read: ``rig.cameras.append(cam)`` does not
    persist (assign a whole list), but ``rig.cameras[1].size[0] = 480`` does, as in the binding.
    """

    __slots__ = ("_cameras", "_imus")

    cameras = _Attr(_as_list_of(Camera), copy_get=True)
    imus = _Attr(_as_list_of(ImuCalibration), copy_get=True)

    def __init__(self, cameras: Sequence[Camera] = (), imus: Sequence[ImuCalibration] = ()) -> None:
        self.cameras = cameras
        self.imus = imus

    def __repr__(self) -> str:
        return "cuvslam.Rig(cameras={}, imus={})".format(self._cameras, self._imus)


_IMU_MEAS_REQUIRED = ("timestamp_ns", "linear_accelerations", "angular_velocities")


class ImuMeasurement(_Struct):
    """One IMU sample: ``timestamp_ns`` (same clock as the images), accel ``m/s^2`` (3), gyro ``rad/s`` (3).

    Feed raw sensor values (specific force, gravity included) in the IMU frame; the frame is
    defined by ``ImuCalibration.rig_from_imu``.  The list attributes are copies.
    """

    __slots__ = ("_timestamp_ns", "_linear_accelerations", "_angular_velocities")

    timestamp_ns = _Attr(_as_int)
    linear_accelerations = _Attr(lambda v: _as_float_list(v, 3, "linear_accelerations"), copy_get=True)
    angular_velocities = _Attr(lambda v: _as_float_list(v, 3, "angular_velocities"), copy_get=True)

    def __init__(self, **kwargs: Any) -> None:
        self._timestamp_ns = 0
        self._linear_accelerations = [0.0, 0.0, 0.0]
        self._angular_velocities = [0.0, 0.0, 0.0]
        if not kwargs:
            return
        args = _take_kwargs("ImuMeasurement", kwargs, _IMU_MEAS_REQUIRED, {})
        self.timestamp_ns = args["timestamp_ns"]
        self.linear_accelerations = args["linear_accelerations"]
        self.angular_velocities = args["angular_velocities"]


class PoseStamped(_Struct):
    """Pose with timestamp (both read-write); ``PoseStamped()`` takes no arguments, set the attributes."""

    __slots__ = ("_timestamp_ns", "_pose")

    timestamp_ns = _Attr(_as_int)
    pose = _Attr(_as_instance(Pose))

    def __init__(self) -> None:
        self._timestamp_ns = 0
        self._pose = Pose()

    @classmethod
    def _make(cls, timestamp_ns: int, pose: Pose) -> "PoseStamped":
        obj = cls()
        obj.timestamp_ns = timestamp_ns
        obj.pose = pose
        return obj

    def __repr__(self) -> str:
        return "cuvslam.PoseStamped(timestamp={}, pose={})".format(self._timestamp_ns, self._pose)


class PoseWithCovariance(_Struct):
    """Read-only ``pose`` and ``covariance_xyz_rpy`` (36 floats, 6x6 row-major, x y z rotX rotY rotZ)."""

    __slots__ = ("_pose", "_covariance_xyz_rpy")

    def __init__(self) -> None:
        self._pose = Pose()
        self._covariance_xyz_rpy: List[float] = [0.0] * 36

    @classmethod
    def _make(cls, pose: Pose, covariance: Sequence[float]) -> "PoseWithCovariance":
        obj = cls()
        obj._pose = copy.deepcopy(pose)
        obj._covariance_xyz_rpy = [float(x) for x in covariance]
        if len(obj._covariance_xyz_rpy) != 36:
            raise ValueError("covariance must have 36 entries")
        return obj

    @property
    def pose(self) -> Pose:
        return self._pose

    @property
    def covariance_xyz_rpy(self) -> List[float]:
        return list(self._covariance_xyz_rpy)

    def __repr__(self) -> str:
        return "cuvslam.PoseWithCovariance(pose={}, covariance_xyz_rpy={})".format(
            self._pose, _fmt(self._covariance_xyz_rpy))


class PoseEstimate(_Struct):
    """Result of ``Odometry.track``: read-only ``timestamp_ns`` and ``world_from_rig`` (``None`` = failed).

    There is no ``is_valid`` attribute in Python; test ``world_from_rig is None``.
    """

    __slots__ = ("_timestamp_ns", "_world_from_rig")

    def __init__(self) -> None:
        self._timestamp_ns = 0
        self._world_from_rig: Optional[PoseWithCovariance] = None

    @classmethod
    def _make(cls, timestamp_ns: int, world_from_rig: Optional[PoseWithCovariance]) -> "PoseEstimate":
        obj = cls()
        obj._timestamp_ns = int(timestamp_ns)
        obj._world_from_rig = world_from_rig
        return obj

    @property
    def timestamp_ns(self) -> int:
        return self._timestamp_ns

    @property
    def world_from_rig(self) -> Optional[PoseWithCovariance]:
        return self._world_from_rig

    def __repr__(self) -> str:
        return "cuvslam.PoseEstimate(timestamp={}, world_from_rig={})".format(
            self._timestamp_ns, self._world_from_rig)


class Observation(_Struct):
    """2-D observation of a landmark: ``id``, pixel ``u``/``v`` (origin top-left), ``camera_index``.

    ``Observation()`` takes no arguments (like the binding); set the attributes afterwards.
    """

    __slots__ = ("_id", "_u", "_v", "_camera_index")

    id = _Attr(_as_int)
    u = _Attr(_as_float)
    v = _Attr(_as_float)
    camera_index = _Attr(_as_int)

    def __init__(self) -> None:
        self._id = 0
        self._u = 0.0
        self._v = 0.0
        self._camera_index = 0

    @classmethod
    def _make(cls, id: int, u: float, v: float, camera_index: int) -> "Observation":  # noqa: A002
        obj = cls()
        obj.id = id
        obj.u = u
        obj.v = v
        obj.camera_index = camera_index
        return obj


class Landmark(_Struct):
    """3-D landmark: ``id`` and ``coords`` (3 floats; a copy, assign the whole list).  ``Landmark()`` only."""

    __slots__ = ("_id", "_coords")

    id = _Attr(_as_int)
    coords = _Attr(lambda v: _as_float_list(v, 3, "Landmark.coords"), copy_get=True)

    def __init__(self) -> None:
        self._id = 0
        self._coords: List[float] = [0.0, 0.0, 0.0]

    @classmethod
    def _make(cls, id: int, coords: Sequence[float]) -> "Landmark":  # noqa: A002
        obj = cls()
        obj.id = id
        obj.coords = coords
        return obj


# ---------------------------------------------------------------------------------------------
# Image validation (ImageFromNDArray in cuvslam2.cpp, plus the engine-side size check)
# ---------------------------------------------------------------------------------------------

_ARRAY_MESSAGES = {
    "Image": "Image must be 2D or 3D uint8 ndarray of shape (height, width) or (height, width, channels), "
             "with 1 or 3 channels.",
    "Mask": "Mask must be 2D uint8 ndarray of shape (height, width).",
    "Depth": "Depth must be 2D uint16 ndarray of shape (height, width).",
}


def _check_array(arr: Any, kind: str, camera_size: Tuple[int, int], index: int) -> bool:
    """Validate one track() array; return ``False`` when it is the 'no data' empty array."""
    if not isinstance(arr, np.ndarray):
        raise TypeError("{} {} must be a numpy.ndarray (the fake accepts CPU numpy arrays only), got {}".format(
            kind, index, type(arr).__name__))
    if arr.size == 0:
        return False
    message = _ARRAY_MESSAGES[kind]
    if kind == "Image":
        if arr.ndim not in (2, 3) or arr.dtype != np.uint8:
            raise ValueError("{} (camera {}: ndim={}, dtype={})".format(message, index, arr.ndim, arr.dtype))
        if arr.ndim == 3 and arr.shape[2] not in (1, 3):
            raise ValueError("{} (camera {}: {} channels)".format(message, index, arr.shape[2]))
    elif kind == "Mask":
        if arr.ndim != 2 or arr.dtype != np.uint8:
            raise ValueError("{} (camera {}: ndim={}, dtype={})".format(message, index, arr.ndim, arr.dtype))
    else:
        if arr.ndim != 2 or arr.dtype != np.uint16:
            raise ValueError("{} (camera {}: ndim={}, dtype={})".format(message, index, arr.ndim, arr.dtype))
    if not arr.flags.c_contiguous:
        raise ValueError("For CPU and CUDA Host ndarrays, the data must be contiguous "
                         "(camera {}: use np.ascontiguousarray after slicing)".format(index))
    height, width = int(arr.shape[0]), int(arr.shape[1])
    cam_w, cam_h = camera_size
    if (cam_w, cam_h) != (0, 0) and (height, width) != (cam_h, cam_w):
        raise ValueError("{} {} has shape (height={}, width={}) but Camera.size of camera {} is "
                         "(width={}, height={})".format(kind, index, height, width, index, cam_w, cam_h))
    return True


def _check_image_list(images: Any, cameras: Sequence[Camera], kind: str,
                      require_count: bool) -> int:
    """Validate a list of arrays against the rig; return how many carry data."""
    if isinstance(images, np.ndarray) or not _is_sequence(images):
        raise TypeError("{}s must be a list or tuple of numpy arrays, one per rig camera, got {}".format(
            kind.lower(), type(images).__name__))
    if require_count and len(images) != len(cameras):
        raise ValueError("track() expects one {} per rig camera ({}), got {}".format(
            kind.lower(), len(cameras), len(images)))
    present = 0
    for index, arr in enumerate(images):
        if index >= len(cameras):
            raise ValueError("{} {} has no camera in the rig ({} cameras)".format(kind, index, len(cameras)))
        cam = cameras[index]
        if _check_array(arr, kind, (int(cam.size[0]), int(cam.size[1])), index):
            present += 1
    return present


# ---------------------------------------------------------------------------------------------
# Pose sources (the injectable "motion" behind Odometry.track)
# ---------------------------------------------------------------------------------------------

#: ``fn(timestamp_ns, origin_ns) -> Pose | None``; *origin_ns* is the timestamp of the first
#: frame since tracking (re)started, so ``timestamp_ns - origin_ns`` is time since the world
#: frame was defined.  ``None`` means "tracking failed for this frame".  One- and zero-argument
#: callables (``fn(timestamp_ns)``, ``fn()``) are wrapped into this form by ``_adapt_pose_source``.
PoseSource = Callable[[int, int], Optional[Pose]]


class ConstantVelocityModel:
    """Default pose source: straight-line motion at ``velocity_mps`` along ``direction`` (+z = forward).

    ``world_from_rig.translation = direction * velocity_mps * (timestamp_ns - origin_ns) / 1e9``
    with identity rotation, so a 30 fps stream at 0.1 m/s advances 3.333 mm per frame.  Use
    ``velocity_mps=0`` (or ``set_default_pose_source(lambda ts, origin: Pose())``) when a test
    expects a static rig, e.g. NVIDIA's ``test_tracking.py``.
    """

    def __init__(self, velocity_mps: float = DEFAULT_VELOCITY_MPS,
                 direction: Sequence[float] = (0.0, 0.0, 1.0)) -> None:
        self.velocity_mps = float(velocity_mps)
        norm = math.sqrt(sum(float(d) * float(d) for d in direction)) or 1.0
        self.direction = tuple(float(d) / norm for d in direction)

    def __call__(self, timestamp_ns: int, origin_ns: int) -> Optional[Pose]:
        dt_s = (int(timestamp_ns) - int(origin_ns)) / 1e9
        distance = self.velocity_mps * dt_s
        return Pose((0.0, 0.0, 0.0, 1.0), tuple(d * distance for d in self.direction))

    def __repr__(self) -> str:
        return "ConstantVelocityModel(velocity_mps={}, direction={})".format(self.velocity_mps, self.direction)


def _adapt_pose_source(source: Any) -> PoseSource:
    """Return *source* as a ``(timestamp_ns, origin_ns)`` callable, wrapping the 1- and 0-argument forms.

    The arity is read once with :func:`inspect.signature`; callables that cannot be inspected
    (some builtins) are assumed to take the two-argument form.  ``TypeError`` for a non-callable.
    """
    if not callable(source):
        raise TypeError("pose source must be callable (timestamp_ns, origin_ns) -> Pose | None, got {}".format(
            type(source).__name__))
    try:
        parameters = list(inspect.signature(source).parameters.values())
    except (TypeError, ValueError):
        return source
    positional = 0
    for param in parameters:
        if param.kind is inspect.Parameter.VAR_POSITIONAL:
            return source
        if param.kind in (inspect.Parameter.POSITIONAL_ONLY, inspect.Parameter.POSITIONAL_OR_KEYWORD):
            positional += 1
    if positional >= 2:
        return source
    if positional == 1:
        return lambda timestamp_ns, origin_ns: source(timestamp_ns)
    return lambda timestamp_ns, origin_ns: source()


def set_default_pose_source(source: Optional[PoseSource]) -> None:
    """Pose source given to every ``Odometry`` created afterwards (``None`` = :class:`ConstantVelocityModel`).

    Accepts the ``fn(timestamp_ns, origin_ns)``, ``fn(timestamp_ns)`` and ``fn()`` forms.
    """
    if source is not None:
        _adapt_pose_source(source)  # validates now; every Odometry adapts its own copy on assignment
    _default_pose_source[0] = source


def get_default_pose_source() -> Optional[PoseSource]:
    """The module-wide default pose source (``None`` means the constant-velocity model)."""
    return _default_pose_source[0]


def _adapt_timestamp_source(source: Any, what: str) -> Callable[[int], Any]:
    """Return *source* as a ``fn(timestamp_ns)`` callable, wrapping a zero-argument one; ``TypeError`` otherwise."""
    if not callable(source):
        raise TypeError("{} must be callable (timestamp_ns) -> value | None, got {}".format(
            what, type(source).__name__))
    try:
        parameters = list(inspect.signature(source).parameters.values())
    except (TypeError, ValueError):
        return source
    positional = 0
    for param in parameters:
        if param.kind is inspect.Parameter.VAR_POSITIONAL:
            return source
        if param.kind in (inspect.Parameter.POSITIONAL_ONLY, inspect.Parameter.POSITIONAL_OR_KEYWORD):
            positional += 1
    if positional >= 1:
        return source
    return lambda timestamp_ns: source()


def set_default_observations_source(source: Optional[Callable[..., Any]]) -> None:
    """Observation-count source given to every ``Odometry`` created afterwards (test hook, not in the real API).

    ``fn(timestamp_ns) -> int | None`` (or ``fn()``): how many synthetic observations each camera
    reports for that frame; ``None`` keeps the default grid.  ``None`` as *source* restores it.
    """
    if source is not None:
        _adapt_timestamp_source(source, "observations_source")  # validates now
    _default_observations_source[0] = source


def get_default_observations_source() -> Optional[Callable[..., Any]]:
    """The module-wide default observation-count source (``None`` = the fixed grid)."""
    return _default_observations_source[0]


def set_default_covariance_source(source: Optional[Callable[..., Any]]) -> None:
    """Covariance source given to every ``Odometry`` created afterwards (test hook, not in the real API).

    ``fn(timestamp_ns) -> Sequence[float] | None`` (or ``fn()``): the 36 ``covariance_xyz_rpy``
    values of that frame's ``PoseWithCovariance``; ``None`` keeps the default diagonal
    (``1e-4`` position, ``1e-6`` rotation).  ``None`` as *source* restores it.
    """
    if source is not None:
        _adapt_timestamp_source(source, "covariance_source")  # validates now
    _default_covariance_source[0] = source


def get_default_covariance_source() -> Optional[Callable[..., Any]]:
    """The module-wide default covariance source (``None`` = the fixed diagonal)."""
    return _default_covariance_source[0]


def instances() -> List["Odometry"]:
    """All live ``Odometry`` objects, oldest first (weak references; dead ones are dropped)."""
    alive = []
    for ref in list(_instances):
        obj = ref()
        if obj is None:
            _instances.remove(ref)
        else:
            alive.append(obj)
    return alive


def last_odometry() -> Optional["Odometry"]:
    """The most recently created live ``Odometry`` (``tracker.odom``), or ``None``."""
    alive = instances()
    return alive[-1] if alive else None


# ---------------------------------------------------------------------------------------------
# Odometry nested types (defined at module level, attached to the class below)
# ---------------------------------------------------------------------------------------------

class _MulticameraMode(enum.Enum):
    Performance = 0
    Precision = 1
    Moderate = 2


class _OdometryMode(enum.Enum):
    Multicamera = 0
    Inertial = 1
    RGBD = 2
    Mono = 3
    Multisensor = 4


_RGBD_DEFAULTS: Dict[str, Any] = {"depth_scale_factor": 1.0, "depth_camera_id": -1,
                                  "enable_depth_stereo_tracking": False}


class _RGBDSettings(_Struct):
    """Settings for RGB-D odometry mode (keyword-only)."""

    __slots__ = ("_depth_scale_factor", "_depth_camera_id", "_enable_depth_stereo_tracking")

    depth_scale_factor = _Attr(_as_float)
    depth_camera_id = _Attr(_as_int)
    enable_depth_stereo_tracking = _Attr(_as_bool)

    def __init__(self, **kwargs: Any) -> None:
        args = _take_kwargs("Odometry.RGBDSettings", kwargs, (), _RGBD_DEFAULTS)
        for key, value in args.items():
            setattr(self, key, value)


_MULTISENSOR_DEFAULTS: Dict[str, Any] = {"depth_camera_ids": (), "depth_scale_factor": 1.0,
                                         "enable_depth_stereo_tracking": True}


class _MultisensorSettings(_Struct):
    """Settings for the experimental Multisensor mode (keyword-only)."""

    __slots__ = ("_depth_camera_ids", "_depth_scale_factor", "_enable_depth_stereo_tracking")

    depth_camera_ids = _Attr(lambda v: _as_int_list(v, "depth_camera_ids"), copy_get=True)
    depth_scale_factor = _Attr(_as_float)
    enable_depth_stereo_tracking = _Attr(_as_bool)

    def __init__(self, **kwargs: Any) -> None:
        args = _take_kwargs("Odometry.MultisensorSettings", kwargs, (), _MULTISENSOR_DEFAULTS)
        for key, value in args.items():
            setattr(self, key, value)


class _OdometryConfig(_Struct):
    """``Odometry.Config``: keyword-only, same 15 fields and defaults as v17.0.0 (see api doc 3.2)."""

    __slots__ = ("_multicam_mode", "_odometry_mode", "_use_gpu", "_async_sba", "_use_motion_model",
                 "_use_denoising", "_rectified_stereo_camera", "_enable_observations_export",
                 "_enable_landmarks_export", "_enable_final_landmarks_export", "_max_frame_delta_s",
                 "_debug_dump_directory", "_debug_imu_mode", "_rgbd_settings", "_multisensor_settings")

    multicam_mode = _Attr(_as_enum(_MulticameraMode))
    odometry_mode = _Attr(_as_enum(_OdometryMode))
    use_gpu = _Attr(_as_bool)
    async_sba = _Attr(_as_bool)
    use_motion_model = _Attr(_as_bool)
    use_denoising = _Attr(_as_bool)
    rectified_stereo_camera = _Attr(_as_bool)
    enable_observations_export = _Attr(_as_bool)
    enable_landmarks_export = _Attr(_as_bool)
    enable_final_landmarks_export = _Attr(_as_bool)
    max_frame_delta_s = _Attr(_as_float)
    debug_dump_directory = _Attr(_as_str)
    debug_imu_mode = _Attr(_as_bool)
    rgbd_settings = _Attr(_as_instance(_RGBDSettings))
    multisensor_settings = _Attr(_as_instance(_MultisensorSettings))

    def __init__(self, **kwargs: Any) -> None:
        defaults: Dict[str, Any] = {
            "multicam_mode": _MulticameraMode.Precision,
            "odometry_mode": _OdometryMode.Multicamera,
            "use_gpu": True,
            "async_sba": True,
            "use_motion_model": True,
            "use_denoising": False,
            "rectified_stereo_camera": False,
            "enable_observations_export": True,
            "enable_landmarks_export": True,
            "enable_final_landmarks_export": False,
            "max_frame_delta_s": 1.0,
            "debug_dump_directory": "",
            "debug_imu_mode": False,
            "rgbd_settings": _RGBDSettings(),
            "multisensor_settings": _MultisensorSettings(),
        }
        args = _take_kwargs("Odometry.Config", kwargs, (), defaults)
        for key in defaults:
            setattr(self, key, args[key])


class _OdometryState(_Struct):
    """Snapshot consumed by ``Slam.track``: frame id, timestamp, delta pose, keyframe flag, exports."""

    __slots__ = ("_frame_id", "_timestamp_ns", "_delta", "_keyframe", "_warming_up", "_gravity",
                 "_observations", "_landmarks", "_context", "_fake_world_from_rig")

    frame_id = _Attr(_as_int)
    timestamp_ns = _Attr(_as_int)
    delta = _Attr(_as_instance(Pose))
    keyframe = _Attr(_as_bool)
    warming_up = _Attr(_as_bool)
    gravity = _Attr(_as_optional_float3)
    observations = _Attr(_as_list_of(Observation), copy_get=True)
    landmarks = _Attr(_as_list_of(Landmark), copy_get=True)
    context = _Attr(lambda v: dict(v))

    def __init__(self) -> None:
        self._frame_id = 0
        self._timestamp_ns = 0
        self._delta = Pose()
        self._keyframe = False
        self._warming_up = False
        self._gravity: Optional[List[float]] = None
        self._observations: List[Observation] = []
        self._landmarks: List[Landmark] = []
        self._context: Dict[int, Any] = {}
        self._fake_world_from_rig: Optional[Pose] = None


_INTERNALS_DEFAULTS: Dict[str, Any] = {
    "num_desired_tracks": 0, "border_top": 0, "border_bottom": 0, "border_left": 0, "border_right": 0,
    "box3_prefilter": False, "ransac_filter": False, "kf_survivor_from_last": 0.0,
    "kf_max_timedelta_between_kfs_s": 60, "kf_override_frame_selection": None,
    "vo_pnp_lambda": 0.0, "vo_pnp_huber": 0.0, "vo_pnp_max_iteration": 0, "vo_pnp_recalculate_cov": False,
    "vo_pnp_filter_new_observations": False, "vo_pnp_max_obs_per_camera": 0, "vo_pnp_point_z_thresh": 0.0,
    "vo_pnp_min_observations": 0, "vo_pnp_cost_thresh": 0.0,
    "inertial_stereo_pnp_lambda": 0.0, "inertial_stereo_pnp_huber": 0.0,
    "inertial_stereo_pnp_max_iteration": 0, "inertial_stereo_pnp_recalculate_cov": False,
    "inertial_stereo_pnp_filter_new_observations": False, "inertial_stereo_pnp_max_obs_per_camera": 0,
    "inertial_stereo_pnp_point_z_thresh": 0.0, "inertial_stereo_pnp_min_observations": 0,
    "inertial_stereo_pnp_cost_thresh": 0.0,
    "imu_pnp_robustifier_scale": 0.0, "imu_pnp_max_iteration": 0, "imu_pnp_min_observations": 0,
    "icp_lambda": 0.0, "icp_huber_vis": 0.0, "icp_huber_depth": 0.0, "icp_max_iteration": 0,
    "icp_cost_thresh": 0.0, "icp_min_scale_level": 0, "icp_max_scale_level": 0,
    "icp_num_iters_per_scale": 0, "icp_blending_alpha": 0.0,
}


def _internals_converter(name: str, default: Any) -> Callable[[Any], Any]:
    if name == "kf_override_frame_selection":
        return lambda v: None if v is None else _as_bool(v)
    if isinstance(default, bool):
        return _as_bool
    if isinstance(default, int):
        return _as_int
    return _as_float


class _Internals(_Struct):
    """Per-frame expert overrides (stateless; ``Internals()`` then set fields).

    Field names follow the v17.0.0 stub; the numeric defaults here are placeholders (the real
    defaults live in the C++ struct and are not documented) except
    ``kf_max_timedelta_between_kfs_s = 60`` and ``kf_override_frame_selection = None``.
    """

    __slots__ = tuple("_" + name for name in _INTERNALS_DEFAULTS)

    def __init__(self) -> None:
        for name, default in _INTERNALS_DEFAULTS.items():
            setattr(self, name, default)

    def __repr__(self) -> str:
        return "Internals(num_desired_tracks={}, kf_survivor_from_last={}, ...)".format(
            self.num_desired_tracks, self.kf_survivor_from_last)  # type: ignore[attr-defined]


for _name, _default in _INTERNALS_DEFAULTS.items():
    _descriptor = _Attr(_internals_converter(_name, _default))
    _descriptor.__set_name__(_Internals, _name)
    setattr(_Internals, _name, _descriptor)
del _name, _default, _descriptor


# ---------------------------------------------------------------------------------------------
# Odometry
# ---------------------------------------------------------------------------------------------

class Odometry:
    """Visual(-inertial) odometry stand-in: validates like the binding, moves like the pose source.

    ``Odometry(rig, cfg=Odometry.Config())``.  Construction raises ``ValueError`` for an empty rig,
    cameras of different or negative size, more than one IMU, ``Inertial`` without exactly
    one IMU, and ``Multisensor`` without depth cameras or a camera pair (or with duplicate depth
    ids), matching the real constructor.

    Test hooks (not in the real API): ``pose_source``, ``observations_source``,
    ``covariance_source``, ``calls``, ``frame_timestamps``,
    ``poses`` (valid ``PoseStamped`` in order), ``imu_log``, ``resets``, ``lost_frames``,
    ``rig``/``config`` (deep copies of the constructor arguments), ``expert_parameters``.
    """

    MulticameraMode = _MulticameraMode
    OdometryMode = _OdometryMode
    RGBDSettings = _RGBDSettings
    MultisensorSettings = _MultisensorSettings
    Config = _OdometryConfig
    State = _OdometryState
    Internals = _Internals

    # export_values() shortcuts of the real binding
    Performance = _MulticameraMode.Performance
    Precision = _MulticameraMode.Precision
    Moderate = _MulticameraMode.Moderate
    Multicamera = _OdometryMode.Multicamera
    Inertial = _OdometryMode.Inertial
    RGBD = _OdometryMode.RGBD
    Mono = _OdometryMode.Mono
    Multisensor = _OdometryMode.Multisensor

    EXPERT_KEYS = ("sba.num_sba_frames", "sba.num_inertial_sba_frames", "sba.num_fixed_sba_frames",
                   "sba.num_sba_iterations", "sba.robustifier_scale", "sba.use_sba_winsorizer",
                   "sm.gravity_update_period_ns", "sm.max_integration_time_ns", "sm.min_num_kf_for_gravity",
                   "sm.min_time_period_ns", "sm.max_time_period_ns")

    def __init__(self, rig: Rig, cfg: Optional[_OdometryConfig] = None) -> None:
        _count("Odometry.__init__")
        if not isinstance(rig, Rig):
            raise TypeError("Odometry(): rig must be a cuvslam.Rig, got {}".format(type(rig).__name__))
        if cfg is None:
            cfg = _OdometryConfig()
        elif not isinstance(cfg, _OdometryConfig):
            raise TypeError("Odometry(): cfg must be an Odometry.Config, got {}".format(type(cfg).__name__))
        self.rig: Rig = copy.deepcopy(rig)
        self.config: _OdometryConfig = copy.deepcopy(cfg)
        self._validate_rig()

        cfg = self.config
        self._export_observations = bool(cfg.enable_observations_export or cfg.enable_final_landmarks_export)
        self._export_landmarks = bool(cfg.enable_landmarks_export or cfg.enable_final_landmarks_export)
        self._export_final = bool(cfg.enable_final_landmarks_export)
        mode = cfg.odometry_mode
        self._imu_fusion = mode is _OdometryMode.Inertial or (
            mode is _OdometryMode.Multisensor and len(self.rig.imus) == 1)

        # injectable motion (property below: the setter adapts 1-/0-argument callables once)
        self.pose_source = _default_pose_source[0] or ConstantVelocityModel()
        # injectable observation count / covariance (None = the fixed grid / diagonal)
        self.observations_source = _default_observations_source[0]
        self.covariance_source = _default_covariance_source[0]

        # tracking state
        self._last_frame_ts: Optional[int] = None
        self._last_imu_ts: Optional[int] = None
        self._origin_ts: Optional[int] = None
        self._needs_restart = True
        self._had_valid_pose = False
        self._frames_since_restart = 0
        self._frame_id = -1
        self._last_pose: Optional[Pose] = None
        self._last_state: Optional[_OdometryState] = None
        self._last_observations: List[List[Observation]] = [[] for _ in self.rig.cameras]
        self._last_landmarks: List[Landmark] = []
        self._final_landmarks: Dict[int, List[float]] = {}

        # bookkeeping for tests
        self.calls: List[Dict[str, Any]] = []
        self.frame_timestamps: List[int] = []
        self.poses: List[PoseStamped] = []
        self.imu_log: List[ImuMeasurement] = []
        self.resets = 0
        self.lost_frames = 0
        self.expert_parameters: Dict[str, str] = {}

        _instances[:] = [ref for ref in _instances if ref() is not None]  # drop dead weakrefs first
        _instances.append(weakref.ref(self))
        _log(3, "Odometry created: {} camera(s), {} IMU(s), mode {}".format(
            len(self.rig.cameras), len(self.rig.imus), mode.name))

    @property
    def pose_source(self) -> Any:
        """The injected motion callable exactly as assigned (test hook, not in the real API)."""
        return self._pose_source_user

    @pose_source.setter
    def pose_source(self, source: Any) -> None:
        self._pose_source_call = _adapt_pose_source(source)
        self._pose_source_user = source

    @property
    def observations_source(self) -> Any:
        """Scripted observation count ``fn(timestamp_ns) -> int | None`` (test hook, not in the real API)."""
        return self._observations_source_user

    @observations_source.setter
    def observations_source(self, source: Any) -> None:
        self._observations_source_call = (None if source is None
                                          else _adapt_timestamp_source(source, "observations_source"))
        self._observations_source_user = source

    @property
    def covariance_source(self) -> Any:
        """Scripted ``covariance_xyz_rpy`` ``fn(timestamp_ns) -> 36 floats | None`` (test hook, not in the real API)."""
        return self._covariance_source_user

    @covariance_source.setter
    def covariance_source(self, source: Any) -> None:
        self._covariance_source_call = (None if source is None
                                        else _adapt_timestamp_source(source, "covariance_source"))
        self._covariance_source_user = source

    def _scripted_observations(self, timestamp: int) -> Optional[int]:
        if self._observations_source_call is None:
            return None
        value = self._observations_source_call(timestamp)
        if value is None:
            return None
        if isinstance(value, bool) or not isinstance(value, numbers.Integral) or int(value) < 0:
            raise TypeError("observations_source must return a non-negative int or None, got {!r}".format(value))
        return int(value)

    def _scripted_covariance(self, timestamp: int) -> Optional[List[float]]:
        if self._covariance_source_call is None:
            return None
        value = self._covariance_source_call(timestamp)
        if value is None:
            return None
        cov = [float(v) for v in value]
        if len(cov) != 36:
            raise ValueError("covariance_source must return 36 floats or None, got {}".format(len(cov)))
        return cov

    # ----------------------------------------------------------------------------- validation

    def _validate_rig(self) -> None:
        cameras = self.rig.cameras
        imus = self.rig.imus
        cfg = self.config
        if not cameras:
            raise ValueError("Rig has no cameras: Odometry needs at least one camera")
        sizes = []
        for index, cam in enumerate(cameras):
            width, height = int(cam.size[0]), int(cam.size[1])
            if width < 0 or height < 0:
                raise ValueError("Camera {} has a negative size (width={}, height={})".format(
                    index, width, height))
            # (0, 0) is the Camera() default and means "size not set": the image-shape check in
            # track() is then skipped for that camera (upstream only pins down negative -> ValueError).
            sizes.append((width, height))
        if len(set(sizes)) != 1:
            raise ValueError("All cameras of a rig must have the same size, got {}".format(sizes))
        if len(imus) > 1:
            raise ValueError("Rig has {} IMUs; at most one IMU is supported".format(len(imus)))
        mode = cfg.odometry_mode
        if mode is _OdometryMode.Inertial and len(imus) != 1:
            raise ValueError("OdometryMode.Inertial requires exactly one IMU in the rig, got {}".format(len(imus)))
        if mode is _OdometryMode.Multisensor:
            depth_ids = cfg.multisensor_settings.depth_camera_ids
            if len(set(depth_ids)) != len(depth_ids):
                raise ValueError("MultisensorSettings.depth_camera_ids contains a duplicate camera id: {}".format(
                    depth_ids))
            for cam_id in depth_ids:
                if cam_id < 0 or cam_id >= len(cameras):
                    raise ValueError("MultisensorSettings.depth_camera_ids refers to camera {} but the rig "
                                     "has {} cameras".format(cam_id, len(cameras)))
            if not depth_ids and len(cameras) < 2:
                raise ValueError("Multisensor mode requires at least one RGB-D camera "
                                 "(MultisensorSettings.depth_camera_ids) or one camera pair with "
                                 "overlapping frustums")

    # ----------------------------------------------------------------------------- tracking

    def track(self, timestamp: int, images: Sequence[np.ndarray],
              masks: Optional[Sequence[np.ndarray]] = None,
              depths: Optional[Sequence[np.ndarray]] = None,
              internals: Optional[_Internals] = None) -> PoseEstimate:
        """Validate the frame like the real binding, then return the pose source's pose.

        Raises ``TypeError`` for a non-int timestamp or non-array images and ``ValueError`` for:
        image count != rig cameras, non-``uint8`` / non-2-D-or-3-D / non-contiguous images,
        image shape != ``Camera.size``, masks count != images count (unless empty), bad masks or
        depths, timestamp not strictly greater than the previous frame's, or older than the last
        registered IMU sample.  A gap above ``max_frame_delta_s`` yields ``world_from_rig None``.
        """
        _count("Odometry.track")
        timestamp = _as_int(timestamp)
        if internals is not None and not isinstance(internals, _Internals):
            raise TypeError("internals must be an Odometry.Internals or None")
        cameras = self.rig.cameras
        present = _check_image_list(images, cameras, "Image", require_count=True)
        if masks is not None:
            if isinstance(masks, np.ndarray) or not _is_sequence(masks):
                raise TypeError("masks must be a list or tuple of numpy arrays, or None")
            if len(masks) != len(images) and len(masks) != 0:
                raise ValueError("If the masks vector is not empty, its size must match the images vector "
                                 "size. Got masks size: {}, images size: {}".format(len(masks), len(images)))
            _check_image_list(masks, cameras, "Mask", require_count=False)
        if depths is not None:
            _check_image_list(depths, cameras, "Depth", require_count=False)
        if present == 0:
            raise ValueError("track() received no image data (every array is empty)")
        if self._last_frame_ts is not None and timestamp <= self._last_frame_ts:
            raise ValueError("Frame timestamps must be strictly increasing: got {} after {}".format(
                timestamp, self._last_frame_ts))
        if self._last_imu_ts is not None and timestamp < self._last_imu_ts:
            raise ValueError("track() timestamp {} is older than the last registered IMU measurement {}: "
                             "track and register_imu_measurement must be called in non-decreasing "
                             "timestamp order".format(timestamp, self._last_imu_ts))

        gap_ns = None if self._last_frame_ts is None else timestamp - self._last_frame_ts
        self._last_frame_ts = timestamp
        self._frame_id += 1
        self.frame_timestamps.append(timestamp)

        pose: Optional[Pose] = None
        if gap_ns is not None and gap_ns > self.config.max_frame_delta_s * 1e9:
            _log(2, "frame gap {:.3f} s exceeds max_frame_delta_s={}: tracking lost".format(
                gap_ns / 1e9, self.config.max_frame_delta_s))
            self._needs_restart = True
        else:
            if self._needs_restart:
                self._origin_ts = timestamp
                self._frames_since_restart = 0
            origin = self._origin_ts if self._origin_ts is not None else timestamp
            pose = self._pose_source_call(timestamp, origin)
            if pose is not None and not isinstance(pose, Pose):
                raise TypeError("pose_source must return a cuvslam.Pose or None, got {}".format(
                    type(pose).__name__))
            if pose is not None:
                if self._needs_restart and self._had_valid_pose:
                    self.resets += 1
                    _log(2, "tracking re-initialised at {} (reset #{})".format(timestamp, self.resets))
                self._needs_restart = False
                self._had_valid_pose = True
            else:
                self._needs_restart = True

        self.calls.append({"method": "track", "timestamp_ns": timestamp, "images": len(images),
                           "present": present, "masks": None if masks is None else len(masks),
                           "depths": None if depths is None else len(depths),
                           "internals": internals is not None, "valid": pose is not None})

        if pose is None:
            self.lost_frames += 1
            self._last_pose = None
            self._last_state = None
            _log(3, "track({}): tracking failed".format(timestamp))
            return PoseEstimate._make(timestamp, None)

        pose = copy.deepcopy(pose)
        first_after_restart = self._frames_since_restart == 0
        self._frames_since_restart += 1
        delta = Pose() if (self._last_pose is None) else _relative_pose(self._last_pose, pose)
        keyframe = first_after_restart or (self._frames_since_restart - 1) % KEYFRAME_EVERY == 0
        if internals is not None and internals.kf_override_frame_selection is not None:
            keyframe = bool(internals.kf_override_frame_selection)
        self._last_pose = pose
        self.poses.append(PoseStamped._make(timestamp, pose))
        self._update_features(pose, self._scripted_observations(timestamp))

        state = _OdometryState()
        state.frame_id = self._frame_id
        state.timestamp_ns = timestamp
        state.delta = delta
        state.keyframe = keyframe
        state.warming_up = False
        state.gravity = self._gravity_estimate()
        state.observations = [obs for per_cam in self._last_observations for obs in per_cam]
        state.landmarks = self._last_landmarks
        state._fake_world_from_rig = copy.deepcopy(pose)
        self._last_state = state

        covariance = self._scripted_covariance(timestamp)
        if covariance is None:
            covariance = [0.0] * 36
            for i in range(6):
                covariance[i * 6 + i] = 1e-4 if i < 3 else 1e-6
        _log(3, "track({}): translation {}".format(timestamp, _fmt(pose.translation)))
        return PoseEstimate._make(timestamp, PoseWithCovariance._make(pose, covariance))

    def _update_features(self, pose: Pose, count: Optional[int] = None) -> None:
        """Synthetic observations (grid inside the border mask) and landmarks (2 m in front of cam 0).

        *count* (from ``observations_source``) sets how many observations every camera reports:
        the grid is made fine enough (``ceil(sqrt(count))`` per side) and truncated to *count*
        (the camera-0 landmarks with it); ``None`` keeps the ``OBSERVATION_GRID`` x
        ``OBSERVATION_GRID`` default.
        """
        self._last_observations = []
        landmarks: List[Landmark] = []
        n = OBSERVATION_GRID if count is None else max(1, int(math.ceil(math.sqrt(count))))
        for cam_index, cam in enumerate(self.rig.cameras):
            per_cam: List[Observation] = []
            if self._export_observations or self._export_landmarks:
                width, height = int(cam.size[0]), int(cam.size[1])
                x0, x1 = cam.border_left, width - cam.border_right
                y0, y1 = cam.border_top, height - cam.border_bottom
                if x1 - x0 >= n and y1 - y0 >= n:
                    fx = float(cam.focal[0]) or 1.0
                    fy = float(cam.focal[1]) or 1.0
                    cx, cy = float(cam.principal[0]), float(cam.principal[1])
                    for gy in range(n):
                        for gx in range(n):
                            u = x0 + (gx + 0.5) * (x1 - x0) / n
                            v = y0 + (gy + 0.5) * (y1 - y0) / n
                            landmark_id = gy * n + gx
                            per_cam.append(Observation._make(landmark_id, float(u), float(v), cam_index))
                            if cam_index == 0:
                                z = LANDMARK_DEPTH_M
                                landmarks.append(Landmark._make(
                                    landmark_id, ((u - cx) / fx * z, (v - cy) / fy * z, z)))
            if count is not None:
                per_cam = per_cam[:count]
            self._last_observations.append(per_cam if self._export_observations else [])
        if count is not None:
            landmarks = landmarks[:count]  # camera 0 only, appended in step with its observations
        self._last_landmarks = landmarks if self._export_landmarks else []
        if self._export_final:
            q, t = _pose_parts(pose)
            for lm in landmarks:
                if lm.id not in self._final_landmarks:
                    p = _quat_rotate(q, tuple(lm.coords))  # type: ignore[arg-type]
                    self._final_landmarks[lm.id] = [p[0] + t[0], p[1] + t[1], p[2] + t[2]]

    def _gravity_estimate(self) -> Optional[List[float]]:
        if not self._imu_fusion:
            return None
        if len(self.imu_log) < GRAVITY_MIN_IMU_SAMPLES or len(self.poses) < GRAVITY_MIN_FRAMES:
            return None
        return [0.0, GRAVITY_MPS2, 0.0]

    # ----------------------------------------------------------------------------- IMU

    def register_imu_measurement(self, sensor_index: int, imu_measurement: ImuMeasurement) -> None:
        """Accept one IMU sample; ``ValueError`` unless IMU fusion is on, index is 0 and time is non-decreasing."""
        _count("Odometry.register_imu_measurement")
        sensor_index = _as_int(sensor_index)
        if sensor_index < 0:
            raise TypeError("sensor_index must be a non-negative integer (uint32)")
        if not isinstance(imu_measurement, ImuMeasurement):
            raise TypeError("imu_measurement must be a cuvslam.ImuMeasurement, got {}".format(
                type(imu_measurement).__name__))
        if sensor_index != 0:
            raise ValueError("sensor_index must be 0: only one IMU sensor is supported")
        if not self._imu_fusion:
            raise ValueError("IMU fusion is disabled: register_imu_measurement requires OdometryMode.Inertial "
                             "(or Multisensor with an IMU in the rig), current mode is {}".format(
                                 self.config.odometry_mode.name))
        ts = imu_measurement.timestamp_ns
        if self._last_imu_ts is not None and ts < self._last_imu_ts:
            raise ValueError("IMU timestamps must be non-decreasing: got {} after {}".format(ts, self._last_imu_ts))
        if self._last_frame_ts is not None and ts < self._last_frame_ts:
            raise ValueError("IMU measurement at {} is older than the last tracked frame {}: register IMU "
                             "samples before the frame they precede (non-decreasing order)".format(
                                 ts, self._last_frame_ts))
        values = imu_measurement.linear_accelerations + imu_measurement.angular_velocities
        if not all(math.isfinite(v) for v in values):
            raise ValueError("IMU measurement contains non-finite values: {}".format(values))
        self._last_imu_ts = ts
        self.imu_log.append(copy.deepcopy(imu_measurement))
        self.calls.append({"method": "register_imu_measurement", "timestamp_ns": ts})

    # ----------------------------------------------------------------------------- queries

    def get_last_observations(self, camera_index: int) -> List[Observation]:
        """Synthetic observations of the last valid frame for *camera_index* (``[]`` if export is off)."""
        _count("Odometry.get_last_observations")
        camera_index = _as_int(camera_index)
        if camera_index < 0 or camera_index >= len(self.rig.cameras):
            raise ValueError("camera_index {} out of range for a rig with {} cameras".format(
                camera_index, len(self.rig.cameras)))
        if not self._export_observations:
            return []
        return [copy.deepcopy(obs) for obs in self._last_observations[camera_index]]

    def get_last_landmarks(self) -> List[Landmark]:
        """Synthetic landmarks of the last valid frame, in the camera-0 frame (``[]`` if export is off)."""
        _count("Odometry.get_last_landmarks")
        if not self._export_landmarks:
            return []
        return [copy.deepcopy(lm) for lm in self._last_landmarks]

    def get_last_gravity(self) -> Optional[List[float]]:
        """``[0, 9.81, 0]`` after a few IMU samples and frames; ``None`` before; ``ValueError`` without IMU fusion."""
        _count("Odometry.get_last_gravity")
        if not self._imu_fusion:
            raise ValueError("get_last_gravity() requires OdometryMode.Inertial (or Multisensor with an IMU); "
                             "IMU fusion is disabled in mode {}".format(self.config.odometry_mode.name))
        return self._gravity_estimate()

    def get_final_landmarks(self) -> Dict[int, List[float]]:
        """``{id: [x, y, z]}`` in the odometry start frame; ``{}`` unless ``enable_final_landmarks_export``."""
        _count("Odometry.get_final_landmarks")
        if not self._export_final:
            return {}
        return {key: list(value) for key, value in self._final_landmarks.items()}

    def get_state(self) -> _OdometryState:
        """Snapshot of the last valid frame; ``frame_id == -1`` before one.  Never raises.

        With ``enable_observations_export`` off the observation/landmark lists are empty (the
        binding fills a ``State`` unconditionally and documents no error for that case).
        """
        _count("Odometry.get_state")
        if self._last_state is None:
            state = _OdometryState()
            state.frame_id = -1
            return state
        return copy.deepcopy(self._last_state)

    def get_primary_cameras(self) -> List[int]:
        """Cameras whose observations are always present; ``[0]`` here (needed to build a ``Slam``)."""
        _count("Odometry.get_primary_cameras")
        return [0]

    def apply_expert_parameters(self, parameters: Mapping[str, str]) -> None:
        """Record string key/value tuning parameters; unknown keys are ignored, bad values raise ``RuntimeError``."""
        _count("Odometry.apply_expert_parameters")
        if not isinstance(parameters, Mapping):
            raise TypeError("parameters must be a dict of str -> str")
        for key, value in parameters.items():
            if not isinstance(key, str) or not isinstance(value, str):
                raise TypeError("expert parameter keys and values must be str, got {}: {}".format(
                    type(key).__name__, type(value).__name__))
            if key not in self.EXPERT_KEYS:
                _log(2, "apply_expert_parameters: unknown key '{}' ignored".format(key))
                continue
            try:
                float(value)
            except ValueError as exc:
                raise RuntimeError("invalid value for expert parameter '{}': {!r}".format(key, value)) from exc
            self.expert_parameters[key] = value


for _cls, _nested_name in ((_MulticameraMode, "MulticameraMode"), (_OdometryMode, "OdometryMode"),
                           (_RGBDSettings, "RGBDSettings"), (_MultisensorSettings, "MultisensorSettings"),
                           (_OdometryConfig, "Config"), (_OdometryState, "State"), (_Internals, "Internals")):
    _cls.__name__ = _nested_name
    _cls.__qualname__ = "Odometry." + _nested_name
    _init = _cls.__dict__.get("__init__")  # enums define none of their own
    if _init is not None:  # so a bad call reads 'Odometry.Config.__init__() takes 1 positional argument ...'
        _init.__qualname__ = _cls.__qualname__ + ".__init__"
del _cls, _nested_name, _init


# ---------------------------------------------------------------------------------------------
# Slam nested types
# ---------------------------------------------------------------------------------------------

class _SlamDataLayer(enum.Enum):
    Landmarks = 0
    Map = 1
    LoopClosure = 2


class _SlamLandmark(_Struct):
    """SLAM landmark: ``id``, ``weight`` and world-frame ``coords`` (3 floats, copy)."""

    __slots__ = ("_id", "_weight", "_coords")

    id = _Attr(_as_int)
    weight = _Attr(_as_float)
    coords = _Attr(lambda v: _as_float_list(v, 3, "Slam.Landmark.coords"), copy_get=True)

    def __init__(self) -> None:
        self._id = 0
        self._weight = 0.0
        self._coords: List[float] = [0.0, 0.0, 0.0]

    @classmethod
    def _make(cls, id: int, weight: float, coords: Sequence[float]) -> "_SlamLandmark":  # noqa: A002
        obj = cls()
        obj.id = id
        obj.weight = weight
        obj.coords = coords
        return obj


class _SlamLandmarks(_Struct):
    """Collection of ``Slam.Landmark`` with a capture ``timestamp_ns``."""

    __slots__ = ("_timestamp_ns", "_landmarks")

    timestamp_ns = _Attr(_as_int)
    landmarks = _Attr(_as_list_of(_SlamLandmark), copy_get=True)

    def __init__(self) -> None:
        self._timestamp_ns = 0
        self._landmarks: List[_SlamLandmark] = []

    @classmethod
    def _make(cls, timestamp_ns: int, landmarks: Sequence[_SlamLandmark]) -> "_SlamLandmarks":
        obj = cls()
        obj.timestamp_ns = timestamp_ns
        obj.landmarks = landmarks
        return obj


_SLAM_CONFIG_DEFAULTS: Dict[str, Any] = {
    "map_cache_path": "", "use_gpu": True, "sync_mode": False, "enable_reading_internals": False,
    "planar_constraints": False, "gt_align_mode": False, "map_cell_size": 0.0,
    "max_landmarks_distance": 100.0, "max_map_size": 300, "throttling_time_ms": 0, "retention_time_ms": 5000,
}


class _SlamConfig(_Struct):
    """``Slam.Config``: bare ``Config()`` uses the C++ defaults (``enable_reading_internals=False``);
    the keyword-only constructor defaults ``enable_reading_internals=True`` (Python convenience),
    exactly like the binding's two overloads."""

    __slots__ = tuple("_" + name for name in _SLAM_CONFIG_DEFAULTS)

    map_cache_path = _Attr(_as_str)
    use_gpu = _Attr(_as_bool)
    sync_mode = _Attr(_as_bool)
    enable_reading_internals = _Attr(_as_bool)
    planar_constraints = _Attr(_as_bool)
    gt_align_mode = _Attr(_as_bool)
    map_cell_size = _Attr(_as_float)
    max_landmarks_distance = _Attr(_as_float)
    max_map_size = _Attr(_as_int)
    throttling_time_ms = _Attr(_as_int)
    retention_time_ms = _Attr(_as_int)

    def __init__(self, **kwargs: Any) -> None:
        defaults = dict(_SLAM_CONFIG_DEFAULTS)
        if kwargs:
            defaults["enable_reading_internals"] = True
        args = _take_kwargs("Slam.Config", kwargs, (), defaults)
        for key in defaults:
            setattr(self, key, args[key])


_LOCALIZATION_REQUIRED = ("horizontal_search_radius", "vertical_search_radius", "horizontal_step",
                          "vertical_step", "angular_step_rads")


class _SlamLocalizationSettings(_Struct):
    """Keyword-only, all five fields required (metres / radians)."""

    __slots__ = tuple("_" + name for name in _LOCALIZATION_REQUIRED)

    horizontal_search_radius = _Attr(_as_float)
    vertical_search_radius = _Attr(_as_float)
    horizontal_step = _Attr(_as_float)
    vertical_step = _Attr(_as_float)
    angular_step_rads = _Attr(_as_float)

    def __init__(self, **kwargs: Any) -> None:
        args = _take_kwargs("Slam.LocalizationSettings", kwargs, _LOCALIZATION_REQUIRED, {})
        for key in _LOCALIZATION_REQUIRED:
            setattr(self, key, args[key])


class _SlamMetrics(_Struct):
    """Loop-closure / pose-graph metrics (all zero in the fake)."""

    __slots__ = ("_timestamp_ns", "_lc_status", "_pgo_status", "_lc_selected_landmarks_count",
                 "_lc_tracked_landmarks_count", "_lc_pnp_landmarks_count", "_lc_good_landmarks_count")

    timestamp_ns = _Attr(_as_int)
    lc_status = _Attr(_as_bool)
    pgo_status = _Attr(_as_bool)
    lc_selected_landmarks_count = _Attr(_as_int)
    lc_tracked_landmarks_count = _Attr(_as_int)
    lc_pnp_landmarks_count = _Attr(_as_int)
    lc_good_landmarks_count = _Attr(_as_int)

    def __init__(self) -> None:
        self.timestamp_ns = 0
        self.lc_status = False
        self.pgo_status = False
        self.lc_selected_landmarks_count = 0
        self.lc_tracked_landmarks_count = 0
        self.lc_pnp_landmarks_count = 0
        self.lc_good_landmarks_count = 0


class _PoseGraphNode(_Struct):
    __slots__ = ("_id", "_node_pose")

    id = _Attr(_as_int)
    node_pose = _Attr(_as_instance(Pose))

    def __init__(self) -> None:
        self._id = 0
        self._node_pose = Pose()

    @classmethod
    def _make(cls, id: int, node_pose: Pose) -> "_PoseGraphNode":  # noqa: A002
        obj = cls()
        obj.id = id
        obj.node_pose = node_pose
        return obj


class _PoseGraphEdge(_Struct):
    __slots__ = ("_node_from", "_node_to", "_transform", "_covariance")

    node_from = _Attr(_as_int)
    node_to = _Attr(_as_int)
    transform = _Attr(_as_instance(Pose))
    covariance = _ArrayAttr(np.float32, 36)

    def __init__(self) -> None:
        self._node_from = 0
        self._node_to = 0
        self._transform = Pose()
        self._covariance = _PoseGraphEdge.covariance.new_array()

    @classmethod
    def _make(cls, node_from: int, node_to: int, transform: Pose) -> "_PoseGraphEdge":
        obj = cls()
        obj.node_from = node_from
        obj.node_to = node_to
        obj.transform = transform
        return obj


class _PoseGraph(_Struct):
    __slots__ = ("_nodes", "_edges")

    nodes = _Attr(_as_list_of(_PoseGraphNode), copy_get=True)
    edges = _Attr(_as_list_of(_PoseGraphEdge), copy_get=True)

    def __init__(self) -> None:
        self._nodes: List[_PoseGraphNode] = []
        self._edges: List[_PoseGraphEdge] = []

    @classmethod
    def _make(cls, nodes: Sequence[_PoseGraphNode], edges: Sequence[_PoseGraphEdge]) -> "_PoseGraph":
        obj = cls()
        obj.nodes = nodes
        obj.edges = edges
        return obj


# ---------------------------------------------------------------------------------------------
# Slam
# ---------------------------------------------------------------------------------------------

class Slam:
    """SLAM stand-in: remembers the odometry poses it is fed and answers every query consistently.

    ``Slam(rig, primary_cameras, config=Slam.Config())``.  ``track(state)`` stores the pose carried
    by the fake ``Odometry.State``; ``get_pose()`` returns the identity before the first frame;
    ``save_map`` creates the folder plus a marker file and calls ``callback(True)`` synchronously;
    ``localize_in_map`` calls ``start_cb()`` then ``finish_cb(guess_pose, "")`` when the folder
    holds a saved map, else ``finish_cb(None, "<error>")``.  No loop closures ever happen.
    """

    DataLayer = _SlamDataLayer
    Landmarks = _SlamLandmarks      # the collection class, NOT the enum member (as in the binding)
    Landmark = _SlamLandmark
    Config = _SlamConfig
    LocalizationSettings = _SlamLocalizationSettings
    Metrics = _SlamMetrics
    PoseGraphNode = _PoseGraphNode
    PoseGraphEdge = _PoseGraphEdge
    PoseGraph = _PoseGraph
    Map = _SlamDataLayer.Map
    LoopClosure = _SlamDataLayer.LoopClosure

    def __init__(self, rig: Rig, primary_cameras: Sequence[int], config: Optional[_SlamConfig] = None) -> None:
        _count("Slam.__init__")
        if not isinstance(rig, Rig):
            raise TypeError("Slam(): rig must be a cuvslam.Rig, got {}".format(type(rig).__name__))
        if config is None:
            config = _SlamConfig()
        elif not isinstance(config, _SlamConfig):
            raise TypeError("Slam(): config must be a Slam.Config, got {}".format(type(config).__name__))
        self.rig: Rig = copy.deepcopy(rig)
        self.primary_cameras: List[int] = _as_int_list(primary_cameras, "primary_cameras")
        for cam in self.primary_cameras:
            if cam < 0 or cam >= len(self.rig.cameras):
                raise ValueError("primary camera {} out of range for a rig with {} cameras".format(
                    cam, len(self.rig.cameras)))
        self.config: _SlamConfig = copy.deepcopy(config)
        self._pose = Pose()
        self._poses: List[PoseStamped] = []
        self._last_timestamp_ns = 0
        self.calls: List[Dict[str, Any]] = []

    def track(self, state: _OdometryState, gt_pose: Optional[Pose] = None) -> None:
        """Update the SLAM pose from an ``Odometry.State`` (call after each successful odometry frame)."""
        _count("Slam.track")
        if not isinstance(state, _OdometryState):
            raise TypeError("Slam.track(): state must be an Odometry.State, got {}".format(type(state).__name__))
        if gt_pose is not None and not isinstance(gt_pose, Pose):
            raise TypeError("gt_pose must be a cuvslam.Pose or None")
        pose = state._fake_world_from_rig
        if pose is None:
            pose = _pose_compose(self._pose, state.delta)
        self._pose = copy.deepcopy(pose)
        self._last_timestamp_ns = state.timestamp_ns
        self._poses.append(PoseStamped._make(state.timestamp_ns, self._pose))
        self.calls.append({"method": "track", "timestamp_ns": state.timestamp_ns, "gt_pose": gt_pose is not None})

    def get_pose(self) -> Pose:
        """Current SLAM ``world_from_rig`` (identity before the first tracked frame)."""
        _count("Slam.get_pose")
        return copy.deepcopy(self._pose)

    def get_all_slam_poses(self, max_poses_count: int = 0) -> List[PoseStamped]:
        """All tracked poses with timestamps (last *max_poses_count* if > 0)."""
        _count("Slam.get_all_slam_poses")
        max_poses_count = _as_int(max_poses_count)
        poses = self._poses if max_poses_count <= 0 else self._poses[-max_poses_count:]
        return [copy.deepcopy(p) for p in poses]

    def save_map(self, folder_name: str, callback: Callable[[bool], None]) -> None:
        """Create *folder_name* with a marker file and call ``callback(True)`` (``False`` on OSError)."""
        _count("Slam.save_map")
        folder = Path(_as_str(folder_name))
        try:
            folder.mkdir(parents=True, exist_ok=True)
            marker = {"format": "fake_cuvslam_map", "version": VERSION_STRING,
                      "poses": len(self._poses), "last_timestamp_ns": self._last_timestamp_ns}
            (folder / MAP_MARKER_FILE).write_text(json.dumps(marker, indent=2), encoding="utf-8")
            ok = True
        except OSError as exc:
            _log(1, "save_map({}) failed: {}".format(folder, exc))
            ok = False
        self.calls.append({"method": "save_map", "folder": str(folder), "ok": ok})
        callback(ok)

    def localize_in_map(self, folder_name: str, timestamp_ns: int, guess_pose: Pose,
                        images: Sequence[np.ndarray], settings: _SlamLocalizationSettings,
                        start_cb: Callable[[], None],
                        finish_cb: Callable[[Optional[Pose], str], None]) -> None:
        """Validate the inputs, then succeed with *guess_pose* if *folder_name* holds a saved map."""
        _count("Slam.localize_in_map")
        folder = Path(_as_str(folder_name))
        timestamp_ns = _as_int(timestamp_ns)
        if not isinstance(guess_pose, Pose):
            raise TypeError("guess_pose must be a cuvslam.Pose")
        if not isinstance(settings, _SlamLocalizationSettings):
            raise TypeError("settings must be a Slam.LocalizationSettings")
        _check_image_list(images, self.rig.cameras, "Image", require_count=True)
        start_cb()
        if (folder / MAP_MARKER_FILE).is_file():
            self._pose = copy.deepcopy(guess_pose)
            self._last_timestamp_ns = timestamp_ns
            self.calls.append({"method": "localize_in_map", "folder": str(folder), "ok": True})
            finish_cb(copy.deepcopy(guess_pose), "")
        else:
            self.calls.append({"method": "localize_in_map", "folder": str(folder), "ok": False})
            finish_cb(None, "no saved map in {}".format(folder))

    def get_landmarks(self, layer: _SlamDataLayer) -> _SlamLandmarks:
        """Empty landmark collection stamped with the last timestamp (``TypeError`` for a non-DataLayer)."""
        _count("Slam.get_landmarks")
        if not isinstance(layer, _SlamDataLayer):
            raise TypeError("layer must be a Slam.DataLayer (use Slam.DataLayer.Landmarks, not Slam.Landmarks)")
        return _SlamLandmarks._make(self._last_timestamp_ns, [])

    def get_pose_graph(self) -> _PoseGraph:
        """One node per tracked frame, consecutive edges carrying the relative pose (empty if internals are off)."""
        _count("Slam.get_pose_graph")
        if not self.config.enable_reading_internals:
            return _PoseGraph()
        nodes = [_PoseGraphNode._make(i, ps.pose) for i, ps in enumerate(self._poses)]
        edges = [_PoseGraphEdge._make(i - 1, i, _relative_pose(self._poses[i - 1].pose, self._poses[i].pose))
                 for i in range(1, len(self._poses))]
        return _PoseGraph._make(nodes, edges)

    def get_slam_metrics(self) -> _SlamMetrics:
        _count("Slam.get_slam_metrics")
        metrics = _SlamMetrics()
        metrics.timestamp_ns = self._last_timestamp_ns
        return metrics

    def get_loop_closure_poses(self) -> List[PoseStamped]:
        """Always ``[]``: the fake never detects a loop closure."""
        _count("Slam.get_loop_closure_poses")
        return []


for _cls, _nested_name in ((_SlamDataLayer, "DataLayer"), (_SlamLandmark, "Landmark"),
                           (_SlamLandmarks, "Landmarks"), (_SlamConfig, "Config"),
                           (_SlamLocalizationSettings, "LocalizationSettings"), (_SlamMetrics, "Metrics"),
                           (_PoseGraphNode, "PoseGraphNode"), (_PoseGraphEdge, "PoseGraphEdge"),
                           (_PoseGraph, "PoseGraph")):
    _cls.__name__ = _nested_name
    _cls.__qualname__ = "Slam." + _nested_name
    _init = _cls.__dict__.get("__init__")
    if _init is not None:
        _init.__qualname__ = _cls.__qualname__ + ".__init__"
del _cls, _nested_name, _init


# ---------------------------------------------------------------------------------------------
# Tracker: the v17.0.0 python/tracker.py wrapper, logic verbatim on top of the fake Odometry/Slam
# ---------------------------------------------------------------------------------------------
# Derived from PyCuVSLAM v17.0.0 ``python/tracker.py`` (Copyright (c) 2026 NVIDIA CORPORATION,
# NVIDIA Community License).  Method bodies are unchanged; docstrings are shortened.

ImageArray = Any  # np.ndarray, torch.Tensor, etc. (CPU or CUDA)


class Tracker:
    """A wrapper that combines cuVSLAM Odometry and SLAM functionality (v17.0.0 ``tracker.py``)."""

    # Expose inner classes of Odometry & Slam
    OdometryMode = Odometry.OdometryMode
    MulticameraMode = Odometry.MulticameraMode
    OdometryConfig = Odometry.Config
    OdometryRGBDSettings = Odometry.RGBDSettings
    OdometryMultisensorSettings = Odometry.MultisensorSettings
    Internals = Odometry.Internals

    SlamConfig = Slam.Config
    SlamMetrics = Slam.Metrics
    SlamLocalizationSettings = Slam.LocalizationSettings
    SlamDataLayer = Slam.DataLayer
    PoseGraphNode = Slam.PoseGraphNode
    PoseGraphEdge = Slam.PoseGraphEdge
    PoseGraph = Slam.PoseGraph
    SlamLandmark = Slam.Landmark
    SlamLandmarks = Slam.Landmarks

    def __init__(self,
                 rig: Rig,
                 odom_config: Optional[Odometry.Config] = None,
                 slam_config: Optional[Slam.Config] = None) -> None:
        """``Tracker(rig, odom_config=None, slam_config=None)``; SLAM only when *slam_config* is given."""
        if odom_config is None:
            odom_config = self.OdometryConfig()
        # need to export observations/landmarks for Slam
        if slam_config is not None:
            odom_config.enable_observations_export = True
            odom_config.enable_landmarks_export = True
        # Create odometry
        self.odom = Odometry(rig, odom_config)
        # Create SLAM
        primary_cams = self.odom.get_primary_cameras()
        self.slam = Slam(rig, primary_cams, slam_config) if slam_config is not None else None

    def track(self,
              timestamp: int,
              images: Sequence[ImageArray],
              masks: Optional[Sequence[ImageArray]] = None,
              depths: Optional[Sequence[ImageArray]] = None,
              gt_pose: Optional[Pose] = None,
              internals: Optional[Odometry.Internals] = None) -> Tuple[PoseEstimate,
                                                                         Optional[Pose]]:
        """Track one frame: ``(PoseEstimate, slam_pose | None)``; ``world_from_rig`` is ``None`` on failure."""
        pose_estimate = self.odom.track(timestamp, images, masks, depths, internals)

        slam_pose = None
        if self.slam and pose_estimate.world_from_rig:
            state = self.odom.get_state()
            self.slam.track(state, gt_pose)
            slam_pose = self.slam.get_pose()

        return pose_estimate, slam_pose

    def register_imu_measurement(self, sensor_index: int, imu_measurement: ImuMeasurement) -> None:
        """Register an IMU measurement (Inertial mode; non-decreasing timestamps)."""
        self.odom.register_imu_measurement(sensor_index, imu_measurement)

    def get_last_observations(self, camera_index: int) -> List[Observation]:
        """Observations from the last frame for one camera (needs ``enable_observations_export``)."""
        return self.odom.get_last_observations(camera_index)

    def get_last_landmarks(self) -> List[Landmark]:
        """Landmarks from the last frame (needs ``enable_landmarks_export``)."""
        return self.odom.get_last_landmarks()

    def get_last_gravity(self) -> Optional[List[float]]:
        """Gravity in the rig/VO frame (about ``[0, +g, 0]`` upright); ``None`` until available."""
        return self.odom.get_last_gravity()

    def get_final_landmarks(self) -> Dict[int, List[float]]:
        """All final landmarks in the odometry start frame (needs ``enable_final_landmarks_export``)."""
        return self.odom.get_final_landmarks()

    def get_all_slam_poses(self, max_poses_count: int = 0) -> List[PoseStamped]:
        """Smoothed SLAM poses for every frame (``[]`` without SLAM)."""
        return self.slam.get_all_slam_poses(max_poses_count) if self.slam else []

    def save_map(self, folder_name: str, callback: Callable[[bool], None]) -> None:
        """Save the SLAM map; ``callback(False)`` immediately when SLAM is disabled."""
        if self.slam:
            self.slam.save_map(folder_name, callback)
        else:
            callback(False)  # No SLAM instance to save the map

    def localize_in_map(self,
                        folder_name: str,
                        timestamp: int,
                        guess_pose: Pose,
                        images: Sequence[ImageArray],
                        settings: Slam.LocalizationSettings,
                        start_cb: Callable[[], None],
                        finish_cb: Callable[[Optional[Pose], str], None]) -> None:
        """Localize in a saved map; ``ValueError`` when SLAM is disabled."""
        if not self.slam:
            raise ValueError("SLAM is not enabled.")

        self.slam.localize_in_map(folder_name, timestamp, guess_pose, images, settings, start_cb, finish_cb)

    def get_slam_landmarks(self, layer: SlamDataLayer) -> Optional[SlamLandmarks]:
        """Landmarks of a SLAM data layer (``None`` without SLAM)."""
        return self.slam.get_landmarks(layer) if self.slam else None

    def get_pose_graph(self) -> Optional[PoseGraph]:
        """Pose graph of keyframes and loop closures (``None`` without SLAM)."""
        return self.slam.get_pose_graph() if self.slam else None

    def get_slam_metrics(self) -> Optional[SlamMetrics]:
        """SLAM metrics (``None`` without SLAM)."""
        return self.slam.get_slam_metrics() if self.slam else None

    def get_loop_closure_poses(self) -> Optional[List[PoseStamped]]:
        """Last 10 loop-closure poses (``None`` without SLAM)."""
        return self.slam.get_loop_closure_poses() if self.slam else None


# ---------------------------------------------------------------------------------------------
# ``core`` namespace: what ``cuvslam.core`` (= the nanobind extension ``pycuvslam``) exposes
# ---------------------------------------------------------------------------------------------

_CORE_NAMES = ("get_version", "set_verbosity", "warm_up_gpu", "Pose", "Distortion", "Camera",
               "ImuCalibration", "ImuMeasurement", "Rig", "PoseStamped", "PoseWithCovariance",
               "PoseEstimate", "Observation", "Landmark", "Odometry", "Slam")

core = types.ModuleType(__name__ + ".core", "Fake stand-in for cuvslam.pycuvslam (cuvslam.core).")
for _core_name in _CORE_NAMES:
    setattr(core, _core_name, globals()[_core_name])
core.__all__ = list(_CORE_NAMES)  # type: ignore[attr-defined]
core.__spec__ = importlib.machinery.ModuleSpec(core.__name__, None)  # keeps importlib.util.find_spec() happy
# Register the namespace as a real submodule so ``from cuvslam_oak.fake_cuvslam.core import Odometry``
# works exactly like ``from cuvslam.core import Odometry`` (attribute access worked already).
sys.modules[core.__name__] = core
del _core_name
