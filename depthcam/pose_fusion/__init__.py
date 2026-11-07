"""pose_fusion: remove visual-odometry drift with a 360 lidar's map-matched pose.

EKF2 has exactly one external-vision input, so the lidar cannot reach PX4 as a second stream.
This package merges the two upstream instead. The cuVSLAM pose that the Guard passes on stays
the backbone (smooth, ~30 Hz). The lidar pose, scan-matched against the published course map,
estimates how far that pose has drifted. The corrected pose goes to the kit's
``px4_odom_bridge.py`` in the same message, frame and variances as before.

Modules:

* ``se2.py``     - planar pose algebra and quaternion helpers
* ``frames.py``  - course frame -> tracker frame, per contracts/frames.md
* ``policy.py``  - the per-run lidar policy (``allowed`` / ``forbidden``), with no default
* ``fusion.py``  - the drift filter and the slew-limited output (:class:`PoseFusion`)
* ``ros_io.py``  - ROS 2 message <-> plain types, without importing rclpy

The ROS 2 node is ``methods/lidar-vo-fusion/11_pose_fusion_node.py``. Importing this package
pulls in only numpy and the standard library.
"""

from __future__ import annotations

__version__ = "0.1.0"
