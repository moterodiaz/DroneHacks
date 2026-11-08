from glob import glob
from setuptools import setup

package_name = 'dronehacks_pathfly'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md', 'BRINGUP.md']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/data',
         glob('data/*.csv') + glob('data/*.md')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='jsun4231',
    maintainer_email='jsun4231@users.noreply.github.com',
    description='Fly the path-planning repo path, touch down, hop, turn, '
                'move forward, touch down.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'pathfly = dronehacks_pathfly.node:main',
            'pathfly_sim = dronehacks_pathfly.sim:main',
            'pathfly_check = dronehacks_pathfly.check:main',
        ],
    },
)
