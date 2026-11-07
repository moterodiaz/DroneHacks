"""dronehacks_lidar: the ROS 2 nodes of methods/lidar-vo-fusion, packaged for the drone's team workspace.

Entry points (``ros2 run dronehacks_lidar <name>``):

* ``vo_guard``        - withholds cuVSLAM poses that can't be trusted         (``vo_guard_node``)
* ``lidar_localizer`` - MS200K revolutions -> course-frame body pose           (``lidar_localizer_node``)
* ``pose_fusion``     - the drift C from the lidar; true pose; C⁻¹ ∘ setpoints (``pose_fusion_node``)
* ``follower``        - the planner's route -> true-frame setpoints            (``follower_node``)

``launch/lidar_nav.launch.py`` starts them with the kit's ``flight_interface``. The ROS-free logic
lives in ``pose_fusion``, ``follower`` and ``course``, which are installed with this package.
"""

from __future__ import annotations
