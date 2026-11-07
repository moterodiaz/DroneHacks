"""Import guard: the ROS node needs rclpy; skip where it is unavailable.

Pure logic (derivation, engine, monitor) is tested without ROS in
``test_waypoints.py`` and ``test_tracking_engine.py``.
"""
import os
import sys

import pytest

rclpy = pytest.importorskip('rclpy')

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from dronehacks_manual_maze_tunnel_chicane_gateb_landing import mission  # noqa: E402


def test_module_imports():
    assert hasattr(mission, 'FullCourseMission')
    assert hasattr(mission, 'main')
