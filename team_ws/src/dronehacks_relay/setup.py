from setuptools import find_packages, setup

package_name = 'dronehacks_relay'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/competitor.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'relay = dronehacks_relay.relay:main',
            'mission = dronehacks_relay.mission:main',
        ],
    },
)
