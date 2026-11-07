"""Stand-ins for the ROS 2 modules the node and launch files import, for testing without ROS.

Only the names this package uses exist, and the parameter rules ROS enforces
are copied: a parameter is declared once, and its override must have the
declared type (a float parameter refuses an int). It checks *our* code paths.
It is not ROS: run the real thing on the drone (docs/deploy.md).

``install(...)`` puts the fakes in ``sys.modules`` and returns a handle to
inspect what the code did. ``uninstall()`` removes them again.
"""

from __future__ import annotations

import sys
import types
from types import SimpleNamespace
from typing import Dict, Iterable, Optional

_NAMES = ["rclpy", "rclpy.node", "rclpy.qos", "geometry_msgs", "geometry_msgs.msg", "nav_msgs", "nav_msgs.msg",
          "std_msgs", "std_msgs.msg", "px4_msgs", "px4_msgs.msg", "skybeta_flight_interface",
          "skybeta_flight_interface.topics", "ament_index_python", "ament_index_python.packages", "launch",
          "launch.actions", "launch.conditions", "launch.substitutions", "launch.launch_description_sources", "launch_ros",
          "launch_ros.actions"]
_saved: Dict[str, object] = {}


class ParameterTypeError(TypeError):
    pass


class PackageNotFoundError(KeyError):
    pass


# -- messages ---------------------------------------------------------------------------------


def _vec():
    return SimpleNamespace(x=0.0, y=0.0, z=0.0)


def _header():
    return SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0), frame_id="")


class PoseStamped:
    def __init__(self):
        self.header = _header()
        self.pose = SimpleNamespace(position=_vec(), orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))


class TwistStamped:
    def __init__(self):
        self.header = _header()
        self.twist = SimpleNamespace(linear=_vec(), angular=_vec())


class Odometry:
    def __init__(self):
        self.header = _header()
        self.pose = SimpleNamespace(pose=SimpleNamespace(position=_vec(),
                                                         orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)))
        self.twist = SimpleNamespace(twist=SimpleNamespace(linear=_vec(), angular=_vec()))


class String:
    def __init__(self, data: str = ""):
        self.data = data


class VehicleLandDetected:
    def __init__(self, landed: bool = True):
        self.landed = landed


class VehicleAttitude:
    def __init__(self, q=(1.0, 0.0, 0.0, 0.0)):
        self.q = list(q)


# -- rclpy ---------------------------------------------------------------------------------------


class FakeClock:
    def __init__(self, handle):
        self.handle = handle

    def now(self):
        ns = int(self.handle.now_s * 1e9)
        return SimpleNamespace(nanoseconds=ns, to_msg=lambda: SimpleNamespace(sec=ns // 10**9, nanosec=ns % 10**9))


class FakePublisher:
    def __init__(self, msg_type, topic):
        self.msg_type, self.topic, self.sent = msg_type, topic, []

    def publish(self, msg):
        if not isinstance(msg, self.msg_type):
            raise TypeError(f"{self.topic}: published {type(msg).__name__}, declared {self.msg_type.__name__}")
        self.sent.append(msg)


class Handle:
    """What the code under test did, and the knobs a test turns."""

    def __init__(self, parameters: Optional[dict] = None, installed: Iterable[str] = ()):
        self.parameters = dict(parameters or {})
        self.installed = set(installed)
        self.now_s = 100.0
        self.nodes = []
        self.share: Dict[str, str] = {}

    def node(self):
        return self.nodes[-1]


def _make_node_class(handle: Handle):
    class Node:
        def __init__(self, name: str, **kwargs):
            self.name = name
            self.params: Dict[str, object] = {}
            self.publishers: Dict[str, FakePublisher] = {}
            self.subscriptions: Dict[str, tuple] = {}
            self.timers = []
            self.logged = []
            handle.nodes.append(self)

        def declare_parameter(self, name, value):
            if name in self.params:
                raise RuntimeError(f"parameter {name} declared twice")
            if name in handle.parameters:
                given = handle.parameters[name]
                if type(given) is not type(value) and not (isinstance(value, list) and isinstance(given, list)):
                    raise ParameterTypeError(f"parameter {name}: declared {type(value).__name__}, "
                                             f"given {type(given).__name__}")
                value = given
            self.params[name] = value

        def get_parameter(self, name):
            return SimpleNamespace(value=self.params[name])

        def create_publisher(self, msg_type, topic, qos):
            pub = FakePublisher(msg_type, topic)
            self.publishers[topic] = pub
            return pub

        def create_subscription(self, msg_type, topic, callback, qos):
            self.subscriptions[topic] = (msg_type, callback, qos)
            return object()

        def create_timer(self, period, callback):
            self.timers.append((period, callback))
            return object()

        def get_clock(self):
            return FakeClock(handle)

        def get_logger(self):
            return SimpleNamespace(info=self.logged.append, warn=self.logged.append, error=self.logged.append)

        def destroy_node(self):
            pass

    return Node


# -- launch ----------------------------------------------------------------------------------------


class LaunchDescription:
    def __init__(self, entities):
        self.entities = list(entities)


class DeclareLaunchArgument:
    def __init__(self, name, default_value=None, description=""):
        self.name, self.default_value, self.description = name, default_value, description


class LaunchConfiguration:
    def __init__(self, name):
        self.name = name

    def perform(self, context):
        if self.name not in context.launch_configurations:
            raise KeyError(f"launch argument {self.name!r} has no value")
        return context.launch_configurations[self.name]


class OpaqueFunction:
    def __init__(self, function):
        self.function = function


class LogInfo:
    def __init__(self, msg):
        self.msg = msg


class IfCondition:
    def __init__(self, predicate):
        self.predicate = predicate


class LaunchNode:
    def __init__(self, **kwargs):
        self.kwargs = kwargs


class IncludeLaunchDescription:
    def __init__(self, source, launch_arguments=()):
        self.source = source
        self.launch_arguments = dict(launch_arguments)


class PythonLaunchDescriptionSource:
    def __init__(self, path):
        self.path = path


# -- install --------------------------------------------------------------------------------------------


def install(handle: Handle) -> Handle:
    for name in _NAMES:
        if name in sys.modules:
            _saved[name] = sys.modules[name]
        sys.modules[name] = types.ModuleType(name)
    m = sys.modules
    m["rclpy"].init = lambda args=None: None
    m["rclpy"].ok = lambda: True
    m["rclpy"].shutdown = lambda: None
    m["rclpy"].spin = lambda node: None
    m["rclpy.node"].Node = _make_node_class(handle)
    q = m["rclpy.qos"]
    q.QoSProfile = lambda **kw: SimpleNamespace(**kw)
    q.HistoryPolicy = SimpleNamespace(KEEP_LAST="keep_last")
    q.ReliabilityPolicy = SimpleNamespace(BEST_EFFORT="best_effort", RELIABLE="reliable")
    q.DurabilityPolicy = SimpleNamespace(VOLATILE="volatile", TRANSIENT_LOCAL="transient_local")
    m["geometry_msgs.msg"].PoseStamped, m["geometry_msgs.msg"].TwistStamped = PoseStamped, TwistStamped
    m["nav_msgs.msg"].Odometry = Odometry
    m["std_msgs.msg"].String = String
    m["px4_msgs.msg"].VehicleLandDetected, m["px4_msgs.msg"].VehicleAttitude = VehicleLandDetected, VehicleAttitude
    m["skybeta_flight_interface.topics"].fmu_topic = lambda d, base, msg=None, msg_dir=None: f"/fmu/{d}/{base}_v1"

    def share(pkg):
        if pkg not in handle.share:
            raise PackageNotFoundError(pkg)
        return handle.share[pkg]

    def prefix(pkg):
        if pkg not in handle.installed and pkg not in handle.share:
            raise PackageNotFoundError(pkg)
        return "/opt/fake/" + pkg

    m["ament_index_python.packages"].get_package_share_directory = share
    m["ament_index_python.packages"].get_package_prefix = prefix
    m["ament_index_python.packages"].PackageNotFoundError = PackageNotFoundError
    m["launch"].LaunchDescription = LaunchDescription
    a = m["launch.actions"]
    a.DeclareLaunchArgument, a.OpaqueFunction, a.LogInfo = DeclareLaunchArgument, OpaqueFunction, LogInfo
    a.IncludeLaunchDescription = IncludeLaunchDescription
    m["launch.conditions"].IfCondition = IfCondition
    m["launch.substitutions"].LaunchConfiguration = LaunchConfiguration
    m["launch.launch_description_sources"].PythonLaunchDescriptionSource = PythonLaunchDescriptionSource
    m["launch_ros.actions"].Node = LaunchNode
    return handle


def uninstall() -> None:
    for name in _NAMES:
        sys.modules.pop(name, None)
    sys.modules.update(_saved)
    _saved.clear()
    for name in [n for n in sys.modules if n.startswith("dronehacks_autonomy.mission_node")]:
        sys.modules.pop(name, None)
