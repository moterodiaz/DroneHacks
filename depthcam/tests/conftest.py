"""pytest configuration for the DroneDepthCamTesting suite.

* Puts the repository root on ``sys.path`` so ``import cuvslam_oak`` works without installing
  the package (the scripts do the same for themselves).
* Guards the camera: the owner requires that the OAK-D S2 is never booted by the tests, so
  every in-process test gets ``depthai.Device``, ``depthai.DeviceBase`` and ``depthai.Pipeline``
  replaced by functions that fail loudly.  ``dai.Pipeline()`` must be covered too: in DepthAI
  3.10 its default ``createImplicitDevice=True`` boots the camera without ever calling
  ``dai.Device``.  The CLI smoke tests run in subprocesses and only ever enumerate
  (``--device-id nonexistent`` fails before ``dai.Device`` is called).

Nothing here imports ``cuvslam``; the suite runs against ``cuvslam_oak.fake_cuvslam``.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

#: Ported tests that drive tools left out of the DroneHacks port. Those tools command PX4 directly
#: (MAVLink vision, parameter writes, arming or mode changes), which contracts/px4.md and
#: contracts/ros-graph.md forbid. See methods/lidar-vo-fusion/README.md, "Provenance".
#: The test files stay byte-identical to their source; the skips live here.
NOT_PORTED = {
    "tests/test_bench_map.py::test_localize_cli_in_the_room": "course/18_lidar_live.py",
    "tests/test_bench_map.py::test_bridge_runs_on_the_room_map": "course/19_lidar_px4_bridge.py",
    "tests/test_ms200.py::test_cli_check_and_localize_on_a_recording": "course/18_lidar_live.py",
    "tests/test_room_map.py::test_build_maps_every_inside_face": "course/21_room_map.py",
    "tests/test_room_map.py::test_mapper_poses_follow_the_walk": "course/21_room_map.py",
    "tests/test_room_map.py::test_live_mapper_tracks_the_walk_from_the_start": "course/21_room_map.py",
}

#: Tests that need the ``fork`` start method; Windows has only ``spawn``.
NEEDS_FORK = {"tests/test_live_view.py::test_supervisor_restarts_a_dead_cuvslam_and_the_view_re_anchors"}


def pytest_collection_modifyitems(config: pytest.Config, items: list) -> None:
    import multiprocessing

    has_fork = "fork" in multiprocessing.get_all_start_methods()
    for item in items:
        nodeid = item.nodeid.replace("\\", "/")
        key = nodeid[nodeid.index("tests/"):] if "tests/" in nodeid else nodeid
        if key in NOT_PORTED:
            item.add_marker(pytest.mark.skip(reason="needs {}, not ported: it commands PX4 directly".format(
                NOT_PORTED[key])))
        elif key in NEEDS_FORK and not has_fork:
            item.add_marker(pytest.mark.skip(reason="needs the multiprocessing 'fork' start method"))


@pytest.fixture(autouse=True)
def no_camera(monkeypatch: pytest.MonkeyPatch) -> None:
    """Make any in-process attempt to open the OAK camera an immediate test failure."""
    try:
        import depthai as dai  # noqa: WPS433 - optional on tracking-only hosts
    except ImportError:  # pragma: no cover - WSL2 host without the depthai wheel
        return

    def refuse_for(name: str) -> Any:
        def refuse(*_args: Any, **_kwargs: Any) -> None:
            raise AssertionError("tests must never open the OAK camera (dai.{} was called)".format(name))
        return refuse

    for name in ("Device", "DeviceBase", "Pipeline"):
        monkeypatch.setattr(dai, name, refuse_for(name), raising=False)
