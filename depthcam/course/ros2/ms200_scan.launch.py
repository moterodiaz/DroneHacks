"""MS200K through the vendor ROS 2 driver (``oradar_lidar`` / ``oradar_scan``) on Humble, for rviz2.

The vendor's ``launch/ms200_scan.launch.py`` passes Foxy-era arguments (``node_executable``,
``node_name``) that Humble rejects.  This is the same launch with Humble's ``executable`` /
``name``, plus:
* the port from the udev rule;
* ``range_min`` raised past the drone's own arms;
* the lidar's height and yaw on the drone as launch arguments.

The vendor node's per-point timing is wrong for scan-smear correction (see course/README.md), so
the localiser reads the sensor itself (``course/18_lidar_live.py``).  Only one program can hold
the port: stop this launch before running that.

    ros2 launch course/ros2/ms200_scan.launch.py
    ros2 launch course/ros2/ms200_scan.launch.py port:=/dev/ttyUSB0 z:=0.12 yaw:=0.0

Not run yet: the dev laptop has no ROS 2.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    port, z, yaw = LaunchConfiguration("port"), LaunchConfiguration("z"), LaunchConfiguration("yaw")
    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/ms200", description="serial port (the udev name)"),
        DeclareLaunchArgument("z", default_value="0.10", description="scan plane above base_link, m"),
        DeclareLaunchArgument("yaw", default_value="0.0", description="laser_frame yaw from the nose, rad"),
        Node(package="oradar_lidar", executable="oradar_scan", name="MS200", output="screen",
             parameters=[{"device_model": "MS200", "frame_id": "laser_frame", "scan_topic": "MS200/scan",
                          "port_name": port, "baudrate": 230400, "angle_min": 0.0, "angle_max": 360.0,
                          "range_min": 0.25, "range_max": 12.0, "clockwise": False, "motor_speed": 10}]),
        Node(package="tf2_ros", executable="static_transform_publisher", name="base_link_to_laser_frame",
             arguments=["--z", z, "--yaw", yaw, "--frame-id", "base_link", "--child-frame-id", "laser_frame"]),
    ])
