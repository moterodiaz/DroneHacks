"""Runs team_ws/src/dronehacks_pathfly's own tests under the repo's CI test
discovery (which only looks in tests/). The package tests need no ROS, no
network and write no files; they run the real mission against its offline
PX4 model. Locally: python3 -m unittest tests.test_dronehacks_pathfly
"""
import importlib.util
import os
import unittest

_TEST = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                     'team_ws', 'src', 'dronehacks_pathfly', 'test',
                     'test_mission.py')
_spec = importlib.util.spec_from_file_location('dronehacks_pathfly_test_mission',
                                               _TEST)
_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_mod)

for _name, _obj in vars(_mod).items():
    if isinstance(_obj, type) and issubclass(_obj, unittest.TestCase) \
            and _obj is not unittest.TestCase:
        globals()[_name] = _obj
del _name, _obj     # or discovery would load the last class a second time
