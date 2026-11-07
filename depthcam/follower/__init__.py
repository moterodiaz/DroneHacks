"""follower: the planner's route in, setpoints in the true frame out (ROS-free).

* ``route.py``   - read a ``dronehacks-route/0`` file (the planner stream's ``Route.to_dict()``)
  and convert it to the tracker frame of contracts/frames.md
* ``mission.py`` - the state machine: wait for Offboard, take off, follow the route, hold over
  the pad, land

The node is ``dronehacks_lidar.follower_node``. Its setpoints go to ``pose_fusion``, which
corrects them into PX4's frame before the kit's flight interface sees them.
"""

from __future__ import annotations
