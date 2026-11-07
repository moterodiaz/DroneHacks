"""ament_python package ``dronehacks_autonomy``, built by the kit's ``team_build.sh`` from ``~/team_ws/src``."""

from glob import glob

from setuptools import setup

PACKAGE = "dronehacks_autonomy"

setup(
    name=PACKAGE,
    version="0.1.0",
    packages=[PACKAGE, PACKAGE + ".planning"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE]),
        ("share/" + PACKAGE, ["package.xml", "README.md"]),
        ("share/" + PACKAGE + "/launch", glob("launch/*.launch.py")),
        ("share/" + PACKAGE + "/config", ["config/autonomy.example.yaml"]),
        ("share/" + PACKAGE + "/maps", ["maps/course_v3_layout.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="DroneHacks team",
    description="Map-driven autonomy for the Sky Beta",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mission = dronehacks_autonomy.mission_node:main",
            "dry_run = dronehacks_autonomy.dry_run:main",
            "measure_frame = dronehacks_autonomy.measure_frame:main",
        ],
    },
)
