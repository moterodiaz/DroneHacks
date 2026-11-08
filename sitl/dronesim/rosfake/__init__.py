"""A simulated ROS 2 Humble runtime that runs unmodified rclpy code.

Call :func:`activate` once, before anything imports ``rclpy``: it puts the
stand-in packages (``rclpy``, ``launch``, ``launch_ros``, ``ament_index_python``,
``tf2_ros``, ``serial``...) at the front of ``sys.path`` and registers the
message import hook.  :func:`patch_globals` additionally routes ``time.*``,
``threading`` and stdio of node code through the simulation; only the engine
process calls it.
"""
import os
import sys

SITE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'site')
_ACTIVE = False

# Real packages that must never shadow the stand-ins once activated.
_SHADOWED = ('rclpy', 'launch', 'launch_ros', 'ament_index_python', 'tf2_ros', 'tf2_geometry_msgs',
             'serial', 'rosidl_runtime_py')


def activate():
    global _ACTIVE
    if _ACTIVE:
        return
    for name in list(sys.modules):
        if name.split('.')[0] in _SHADOWED and not getattr(sys.modules[name], '__file__', '') \
                .startswith(SITE_DIR):
            del sys.modules[name]
    if SITE_DIR not in sys.path:
        sys.path.insert(0, SITE_DIR)
    from . import msggen
    msggen.install()
    _ACTIVE = True


def patch_globals():
    from . import launch_impl, runtime
    runtime.patch_process_globals()
    launch_impl.install_argv_proxy()
