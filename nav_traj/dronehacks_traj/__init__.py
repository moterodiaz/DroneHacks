"""dronehacks_traj: the no-lidar flight stack, flown by dronehacks_planner's trajectory Follower.

Entry points (``ros2 run dronehacks_traj <name>``):

* ``mission``  - the course map -> setpoints for the kit's flight interface (``mission_node``)
* ``dry_run``  - the whole mission, arming to disarm, against a stand-in PX4, offline (``dry_run``)

``launch/traj_nav.launch.py`` starts ``mission`` with the kit's ``flight_interface`` and the Guard.
The ROS-free logic lives in ``mission``, ``frames``, ``factory`` and ``vehicle_sim``.
"""
