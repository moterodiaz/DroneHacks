"""ament_python package ``dronehacks_traj``, built by colcon (the kit's ``team_build.sh``) from ``~/team_ws/src``.

Installs the node package ``dronehacks_traj``, its launch files and parameter file. It needs
``dronehacks_planner`` (``planner/`` in this repo) in the same workspace.
"""

from glob import glob

from setuptools import setup

PACKAGE = "dronehacks_traj"

setup(
    name=PACKAGE,
    version="0.1.0",
    packages=[PACKAGE],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE]),
        ("share/" + PACKAGE, ["package.xml", "README.md"]),
        ("share/" + PACKAGE + "/launch", glob("launch/*.launch.py")),
        ("share/" + PACKAGE + "/config", ["config/traj_nav.example.yaml", "config/flight_line_window_2mps.yaml",
                                             "config/flight_line_window_0p5mps.yaml"]),
        ("share/" + PACKAGE + "/docs", ["docs/deploy.md"]),
        ("share/" + PACKAGE + "/maps", ["maps/course_v3_step.yaml", "maps/course_arena_2026-11-08.yaml",
                                           "maps/README.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="DroneHacks team",
    description="No-lidar flight stack flown by the dronehacks_planner trajectory Follower",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mission = dronehacks_traj.mission_node:main",
            "dry_run = dronehacks_traj.dry_run:main",
        ],
    },
)
