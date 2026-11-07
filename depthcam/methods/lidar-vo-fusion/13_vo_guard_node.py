#!/usr/bin/env python3
"""13_vo_guard_node.py - run ``dronehacks_lidar.vo_guard_node`` from a source checkout.

On the drone: ``ros2 run dronehacks_lidar vo_guard`` (or ``launch/lidar_nav.launch.py``).
From ``depthcam/``, with ROS 2 Humble sourced:

    python3 methods/lidar-vo-fusion/13_vo_guard_node.py --ros-args --params-file <yaml>

The node's documentation is in ``dronehacks_lidar/vo_guard_node.py``.
"""

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from dronehacks_lidar.vo_guard_node import main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main(sys.argv))
