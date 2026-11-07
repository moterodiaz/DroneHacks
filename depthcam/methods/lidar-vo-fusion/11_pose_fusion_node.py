#!/usr/bin/env python3
"""11_pose_fusion_node.py - run ``dronehacks_lidar.pose_fusion_node`` from a source checkout.

On the drone the package is built in the team workspace and run as
``ros2 run dronehacks_lidar pose_fusion`` (or launched, ``launch/lidar_nav.launch.py``). This
wrapper runs the same node from ``depthcam/`` without building, with ROS 2 Humble sourced:

    python3 methods/lidar-vo-fusion/11_pose_fusion_node.py --ros-args --params-file <yaml> -p lidar_policy:=allowed

The node's documentation is in ``dronehacks_lidar/pose_fusion_node.py``.
"""

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from dronehacks_lidar.pose_fusion_node import main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main(sys.argv))
