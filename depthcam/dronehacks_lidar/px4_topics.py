"""PX4 topic names, resolved against the installed px4_msgs.

PX4 v1.16 suffixes a uORB topic with ``_v<N>`` when its message declares ``MESSAGE_VERSION = N``
with N > 0, and uses the bare name otherwise. The kit's flight interface resolves names the same
way (``drone-snapshot/.../skybeta_flight_interface/topics.py``); this is a short
re-implementation of that rule, not a copy. Hard-coding either spelling breaks the other.

Reading only: these nodes subscribe to ``/fmu/out/*`` and never publish to ``/fmu/in/*``.
"""

from __future__ import annotations

import os
import re
from typing import Optional

_VERSION_RE = re.compile(r"^\s*uint32\s+MESSAGE_VERSION\s*=\s*(\d+)\s*(?:#.*)?$")


def _msg_dir() -> Optional[str]:
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory("px4_msgs")
    except Exception:  # noqa: BLE001 - px4_msgs absent: fall back to bare names
        return None
    path = os.path.join(share, "msg")
    return path if os.path.isdir(path) else None


def message_version(msg_name: str, msg_dir: Optional[str] = None) -> int:
    """``MESSAGE_VERSION`` of a px4_msgs message, 0 when absent or unreadable."""
    msg_dir = msg_dir if msg_dir is not None else _msg_dir()
    if not msg_dir:
        return 0
    try:
        with open(os.path.join(msg_dir, msg_name + ".msg"), "r", encoding="utf-8") as fh:
            for line in fh:
                m = _VERSION_RE.match(line)
                if m:
                    return int(m.group(1))
    except OSError:
        return 0
    return 0


def fmu_out(base: str, msg_name: str, msg_dir: Optional[str] = None) -> str:
    """``/fmu/out/<base>``, with ``_v<N>`` when the message is versioned."""
    n = message_version(msg_name, msg_dir)
    return "/fmu/out/{}{}".format(base, "_v{}".format(n) if n > 0 else "")
