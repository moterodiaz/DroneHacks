"""cuvslam_oak: OAK-D S2 (DepthAI v3) glue for NVIDIA cuVSLAM (PyCuVSLAM v17.0.0).

Planned modules (see ``methods/cuvslam-vio/docs/cuvslam-api-v17.md``, ``datasets/dataset-format.md`` and
``methods/cuvslam-vio/docs/cuvslam-setup.md``):

* ``rig.py``          - build a ``cuvslam.Rig`` from the OAK EEPROM calibration
* ``oak_source.py``   - live stereo GRAY8 + BNO086 IMU frames from the camera
* ``dataset.py``      - EuRoC-ASL layout writer/reader for recorded datasets
* ``tracking.py``     - stereo / stereo-inertial tracking loop and TUM trajectory writer
* ``fake_cuvslam.py`` - a test double for the ``cuvslam`` module (no GPU, runs on Windows)

Only this file and ``fake_cuvslam.py`` know how the real ``cuvslam`` module is located; every
other module calls :func:`import_cuvslam` instead of ``import cuvslam`` so the same code runs
on the three targets:

* Windows laptop: no wheel exists, tests run against the fake (``CUVSLAM_FAKE=1`` or
  ``prefer_fake=True``);
* WSL2 Ubuntu 22.04 and Jetson Orin Nano (JetPack 6.x): the real wheel
  ``cuvslam-17.0.0+cu12-cp310-cp310-manylinux_2_35_{x86_64,aarch64}.whl`` plus CUDA toolkit 12.

Importing this package has no side effects: nothing here imports ``cuvslam``, ``numpy``,
``depthai`` or ``cv2`` at import time.
"""

from __future__ import annotations

import importlib
import os
import warnings
from types import ModuleType

__all__ = [
    "CUVSLAM_MAJOR",
    "FAKE_ENV_VAR",
    "FAKE_MODULE",
    "env_allows_fake",
    "import_cuvslam",
    "install_hint",
    "is_fake",
]

__version__ = "0.1.0"

#: Major version of the PyCuVSLAM API this package was written against (``Tracker(rig, cfg)``;
#: ``main`` after v17.0.0 adds a required ``Tracker.Mode`` argument).
CUVSLAM_MAJOR = 17

#: Environment variable that allows the fake to be used when the real module is missing.
FAKE_ENV_VAR = "CUVSLAM_FAKE"

#: Dotted name of the test double.
FAKE_MODULE = __name__ + ".fake_cuvslam"

_TRUE_VALUES = ("1", "true", "yes", "on")


def env_allows_fake() -> bool:
    """``True`` when ``CUVSLAM_FAKE`` is set to ``1``/``true``/``yes``/``on`` (case-insensitive)."""
    return os.environ.get(FAKE_ENV_VAR, "").strip().lower() in _TRUE_VALUES


def is_fake(module: ModuleType) -> bool:
    """``True`` when *module* is the test double returned by :func:`import_cuvslam`."""
    return bool(getattr(module, "IS_FAKE", False))


def install_hint(reason: str = "") -> str:
    """Multi-line explanation of how to get the real ``cuvslam`` on a supported host."""
    lines = [
        "cuvslam (PyCuVSLAM 17.0.0) is not importable" + (": " + reason if reason else "."),
        "NVIDIA ships Linux-only wheels (Python 3.10, glibc 2.35, CUDA toolkit 12 installed separately):",
        "  - WSL2 Ubuntu 22.04 on this laptop:  bash methods/cuvslam-vio/tools/setup_wsl2.sh    "
        "(x86_64 wheel + cuda-toolkit-12-6 from NVIDIA's wsl-ubuntu apt repo)",
        "  - Jetson Orin Nano, JetPack 6.x:     bash methods/cuvslam-vio/tools/setup_jetson.sh  "
        "(aarch64 wheel; CUDA 12.6 comes with nvidia-jetpack)",
        "  - wheels: https://github.com/nvidia-isaac/cuVSLAM/releases/tag/v17.0.0",
        "  - diagnose the host with: python methods/cuvslam-vio/tools/check_env.py",
        "There is no Windows wheel. For unit tests and dry runs use the built-in test double: "
        "set {}=1 or call import_cuvslam(prefer_fake=True).".format(FAKE_ENV_VAR),
    ]
    return "\n".join(lines)


def _load_fake() -> ModuleType:
    return importlib.import_module(FAKE_MODULE)


def _check_version(module: ModuleType) -> None:
    """Warn when the installed cuvslam is not the 17.x API this package targets."""
    try:
        version = module.get_version()
        major = int(version[1])
        text = str(version[0])
    except Exception as exc:  # noqa: BLE001 - a broken install is reported, not fatal here
        warnings.warn("cuvslam.get_version() failed ({}: {}); the API may not match v17.0.0".format(
            type(exc).__name__, exc), RuntimeWarning, stacklevel=3)
        return
    if major != CUVSLAM_MAJOR:
        warnings.warn(
            "cuvslam {} found but cuvslam_oak targets the {}.x API (Tracker(rig, odom_config); "
            "later versions add a required Tracker.Mode argument)".format(text, CUVSLAM_MAJOR),
            RuntimeWarning, stacklevel=3)


def import_cuvslam(prefer_fake: bool = False) -> ModuleType:
    """Return the ``cuvslam`` module to use: the real wheel, or the test double when allowed.

    Resolution order:

    1. ``prefer_fake=True`` returns :mod:`cuvslam_oak.fake_cuvslam` without even trying the real
       module (explicit choice: deterministic tests and ``--fake`` dry runs, also on Linux).
    2. Otherwise ``import cuvslam`` is attempted.  On success the module is returned (a
       ``RuntimeWarning`` is emitted if its major version is not 17).
    3. If that import fails and ``CUVSLAM_FAKE`` is truthy in the environment, the fake is
       returned instead.
    4. Otherwise an ``ImportError`` is raised whose message explains the WSL2 / Jetson install
       (see :func:`install_hint`) and how to opt into the fake.

    The fake is therefore never picked silently: it takes either the flag or the environment
    variable, and it never shadows a real wheel that imports.  Use :func:`is_fake` to tell the
    two apart (the fake sets ``IS_FAKE = True``).
    """
    if prefer_fake:
        return _load_fake()
    try:
        module = importlib.import_module("cuvslam")
    except ImportError as exc:
        if env_allows_fake():
            return _load_fake()
        raise ImportError(install_hint("{}: {}".format(type(exc).__name__, exc))) from exc
    _check_version(module)
    return module
