"""ament_python package ``dronehacks_planner``, built by colcon (the kit's ``team_build.sh``) from ``~/team_ws/src``.

Installs the library and two commands: ``ros2 run dronehacks_planner plan MAP.yaml`` and
``ros2 run dronehacks_planner bench MAP.yaml``. The provisional map is installed under
``share/dronehacks_planner/maps/`` for dry runs only: fly the surveyed map.
"""

from setuptools import setup

PACKAGE = "dronehacks_planner"

setup(
    name=PACKAGE,
    version="0.1.0",
    packages=[PACKAGE],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE]),
        ("share/" + PACKAGE, ["package.xml", "README.md"]),
        ("share/" + PACKAGE + "/maps", ["test/fixtures/published_v3_fieldguide.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="DroneHacks team",
    description="Course map -> route -> smoothed trajectory -> follower setpoints (ROS-free)",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "plan = dronehacks_planner.__main__:main",
            "bench = dronehacks_planner.bench:main",
        ],
    },
)
