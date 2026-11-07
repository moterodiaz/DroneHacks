from setuptools import find_packages, setup

package_name = 'dronehacks_manual_maze_tunnel_chicane_gateb_landing'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test', 'tests']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/competitor.launch.py']),
        # Verbatim copy of contracts/course_v3_layout.yaml for on-drone use;
        # pass course_map_yaml with the surveyed map before flying
        # (contracts/course.md layout tolerance).
        ('share/' + package_name + '/maps', ['maps/course_v3_layout.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'mission = dronehacks_manual_maze_tunnel_chicane_gateb_landing.mission:main',
        ],
    },
)
