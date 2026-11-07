from glob import glob
from setuptools import setup

package_name = 'dronehacks_square_test'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='TheSleepyPP',
    maintainer_email='linchenyoung1126@gmail.com',
    description='Cage test: take off, 0.5 m box with holds, touch-and-go, land and disarm.',
    license='Apache-2.0',
    entry_points={'console_scripts': ['square_test = dronehacks_square_test.square_test_node:main']},
)
