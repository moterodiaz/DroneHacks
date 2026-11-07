from setuptools import find_packages, setup

package_name = 'dronehacks_telemetry'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test', 'tests']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/scripts', ['scripts/record_flight.sh']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'analyze = dronehacks_telemetry.analyze:main',
            'ulog_triage = dronehacks_telemetry.ulog_triage:main',
        ],
    },
)
