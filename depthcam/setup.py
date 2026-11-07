"""ament_python package ``dronehacks_lidar``, built by colcon (the kit's ``team_build.sh``) from ``~/team_ws/src``.

Installs the ROS-free packages the nodes use (``pose_fusion``, ``follower``, ``course``), the node
package ``dronehacks_lidar``, the launch files, the parameter file, the course map (from the STEP
model) and the routes planned through the ground-truth checkpoints (``routes/``). The rest of
``depthcam/`` (the camera ladder, ``cuvslam_oak``, the research methods) stays out of the install.
"""

from glob import glob

from setuptools import find_packages, setup

PACKAGE = "dronehacks_lidar"

setup(
    name=PACKAGE,
    version="0.1.0",
    packages=find_packages(include=["dronehacks_lidar", "dronehacks_lidar.*", "pose_fusion", "pose_fusion.*",
                                    "follower", "follower.*", "course", "course.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE]),
        ("share/" + PACKAGE, ["package.xml"]),
        ("share/" + PACKAGE + "/launch", glob("launch/*.launch.py")),
        ("share/" + PACKAGE + "/config", ["methods/lidar-vo-fusion/config/pose_fusion.example.yaml"]),
        ("share/" + PACKAGE + "/maps", ["course/maps/course_v3_map.json", "course/maps/course_v3_rules.json"]),
        ("share/" + PACKAGE + "/routes", glob("routes/*.json")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="DroneHacks team",
    description="cuVSLAM Guard, MS200K lidar localiser, drift correction on the flight commands, route Follower",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "vo_guard = dronehacks_lidar.vo_guard_node:main",
            "lidar_localizer = dronehacks_lidar.lidar_localizer_node:main",
            "pose_fusion = dronehacks_lidar.pose_fusion_node:main",
            "follower = dronehacks_lidar.follower_node:main",
        ],
    },
)
