"""dronesim: software-in-the-loop test environment for the DroneHacks drone.

The package runs the team's ROS 2 Python autonomy and the organizer kit's
``skybeta_flight_interface`` / ``px4_odom_bridge`` *unmodified* on a
deterministic simulated ROS 2 graph (``dronesim.rosfake``), against a model of
PX4 v1.16 (``dronesim.px4``), a 6-DOF airframe (``dronesim.vehicle``), sensor
models of cuVSLAM, the VL53L1X rangefinder and the MS200K lidar
(``dronesim.sensors``) and the course geometry read from the STEP file
(``dronesim.world``).  See ``sitl/README.md``.
"""

__version__ = "0.1.0"
