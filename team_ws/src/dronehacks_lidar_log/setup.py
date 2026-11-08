from glob import glob
from setuptools import setup

setup(
    name="dronehacks_lidar_log",
    version="0.1.0",
    packages=["dronehacks_lidar_log"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/dronehacks_lidar_log"]),
        ("share/dronehacks_lidar_log", ["package.xml"]),
        ("share/dronehacks_lidar_log/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={"console_scripts": ["lidar_logger = dronehacks_lidar_log.logger:main",
                                      "lidar_probe = dronehacks_lidar_log.probe:main"]},
)
