from setuptools import find_packages, setup

package_name = "dronehacks_course_follower"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test", "tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/competitor.launch.py"]),
        ("share/" + package_name + "/maps", ["maps/course_map.json"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "follower = dronehacks_course_follower.node:main",
            "dry_run = dronehacks_course_follower.dry_run:main",
        ],
    },
)
