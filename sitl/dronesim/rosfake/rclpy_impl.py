"""Implementation of the rclpy API (ROS 2 Humble) on the simulated runtime.

The public ``rclpy`` package in ``rosfake/site/rclpy`` re-exports these names.
Behaviour follows Humble's rclpy where team code can observe it:

* parameters are statically typed (an INTEGER override for a DOUBLE default
  raises ``InvalidParameterTypeException``; ``[]`` infers ``BYTE_ARRAY``);
* ``Time`` values of different clock types cannot be compared or subtracted;
* ``QoSProfile()`` without history or depth raises;
* callbacks only run while some executor spins the node, one at a time, timers
  before subscriptions when several are ready at the same instant;
* an exception in a callback propagates out of ``spin()``.
"""
from __future__ import annotations

import array
import collections
import enum
import fnmatch
import itertools
import math
import re
import sys
import warnings

from . import runtime as _rt
from .runtime import NS, QoSInfo, Endpoint, RosArgs, get_runtime


# ---------------------------------------------------------------------------
# exceptions
# ---------------------------------------------------------------------------
class NotInitializedException(Exception):
    def __init__(self, *args):
        Exception.__init__(self, 'rclpy.init() has not been called', *args)


class InvalidHandle(Exception):
    pass


class NameValidationException(Exception):
    def __init__(self, name_type, name, error_msg, invalid_index):
        msg = f"Invalid {name_type}: {error_msg}:\n  '{name}'\n   " + ' ' * invalid_index + '^'
        Exception.__init__(self, msg)


class InvalidNamespaceException(NameValidationException):
    def __init__(self, name, error_msg, invalid_index):
        NameValidationException.__init__(self, 'namespace', name, error_msg, invalid_index)


class InvalidNodeNameException(NameValidationException):
    def __init__(self, name, error_msg, invalid_index):
        NameValidationException.__init__(self, 'node name', name, error_msg, invalid_index)


class InvalidTopicNameException(NameValidationException):
    def __init__(self, name, error_msg, invalid_index):
        NameValidationException.__init__(self, 'topic name', name, error_msg, invalid_index)


class InvalidServiceNameException(NameValidationException):
    def __init__(self, name, error_msg, invalid_index):
        NameValidationException.__init__(self, 'service name', name, error_msg, invalid_index)


class ParameterException(Exception):
    def __init__(self, error_msg, parameters, *args):
        Exception.__init__(self, f'{error_msg}: {parameters}', *args)


class ParameterNotDeclaredException(ParameterException):
    def __init__(self, parameters, *args):
        ParameterException.__init__(self, 'Invalid access to undeclared parameter(s)', parameters, *args)


class ParameterAlreadyDeclaredException(ParameterException):
    def __init__(self, parameters, *args):
        ParameterException.__init__(self, 'Parameter(s) already declared', parameters, *args)


class InvalidParameterException(ParameterException):
    def __init__(self, parameter, *args):
        ParameterException.__init__(self, 'Invalid parameter name', parameter, *args)


class InvalidParameterValueException(ParameterException):
    def __init__(self, parameter, value, reason, *args):
        Exception.__init__(self, f'Invalid parameter value ({value}) for parameter. Reason: {reason}',
                           parameter, *args)


class InvalidParameterTypeException(ParameterException):
    def __init__(self, desired_parameter, expected_type, *args):
        Exception.__init__(
            self,
            f"Trying to set parameter '{desired_parameter._name}' to '{desired_parameter._value}' "
            f"of type '{desired_parameter._type_.name}', expecting type '{expected_type}'", *args)


class ParameterImmutableException(ParameterException):
    def __init__(self, parameters, *args):
        ParameterException.__init__(self, 'Attempted to modify read-only parameter', parameters, *args)


class ParameterUninitializedException(ParameterException):
    def __init__(self, parameter_name, *args):
        Exception.__init__(self, f"The parameter '{parameter_name}' is not initialized", *args)


class NoParameterOverrideProvidedException(ParameterException):
    def __init__(self, parameter_name, *args):
        Exception.__init__(self, f"No parameter override provided for '{parameter_name}' when one was expected", *args)


class InvalidQoSProfileException(Exception):
    def __init__(self, *args):
        Exception.__init__(self, 'Invalid QoSProfile', *args)


class ROSInterruptException(Exception):
    pass


class ExternalShutdownException(Exception):
    """Context has been shutdown."""


class ShutdownException(Exception):
    pass


class TimeoutException(Exception):
    pass


class ConditionReachedException(Exception):
    pass


class RCLError(RuntimeError):
    pass


# ---------------------------------------------------------------------------
# names
# ---------------------------------------------------------------------------
_TOKEN_RE = re.compile(r'^[A-Za-z_][A-Za-z0-9_]*$|^\{[a-z_]+\}$')


def _validate_full_name(name, kind='topic'):
    exc = {'topic': InvalidTopicNameException, 'service': InvalidServiceNameException}[kind]
    if not name.startswith('/'):
        raise exc(name, 'must be absolute', 0)
    if name != '/' and name.endswith('/'):
        raise exc(name, "must not end with '/'", len(name) - 1)
    if '//' in name:
        raise exc(name, "must not contain repeated '/'", name.index('//'))
    for i, ch in enumerate(name):
        if not (ch.isalnum() or ch in '_/~{}'):
            raise exc(name, 'must not contain characters other than alphanumerics, \'_\', \'~\', \'{\', or \'}\'', i)
    for token in name.strip('/').split('/'):
        if token and token[0].isdigit():
            raise exc(name, 'must not have a token that starts with a number', name.index(token))


def _validate_node_name(name):
    if not name:
        raise InvalidNodeNameException(name, 'node name must not be empty', 0)
    for i, ch in enumerate(name):
        if not (ch.isalnum() or ch == '_'):
            raise InvalidNodeNameException(name, "node name must not contain characters other than alphanumerics or '_'", i)
    if name[0].isdigit():
        raise InvalidNodeNameException(name, 'node name must not start with a number', 0)


def _normalize_ns(ns):
    ns = ns or '/'
    if not ns.startswith('/'):
        ns = '/' + ns
    if len(ns) > 1:
        ns = ns.rstrip('/')
    return ns


def expand_name(name, node_name, namespace):
    if name.startswith('~'):
        base = (namespace.rstrip('/') + '/' + node_name)
        rest = name[1:]
        out = base + rest if rest.startswith('/') or not rest else base + '/' + rest
    elif name.startswith('/'):
        out = name
    else:
        out = namespace.rstrip('/') + '/' + name
    out = out.replace('{node}', node_name).replace('{ns}', namespace)
    return out


# ---------------------------------------------------------------------------
# QoS
# ---------------------------------------------------------------------------
class HistoryPolicy(enum.IntEnum):
    SYSTEM_DEFAULT = 0
    KEEP_LAST = 1
    KEEP_ALL = 2
    UNKNOWN = 3


class ReliabilityPolicy(enum.IntEnum):
    SYSTEM_DEFAULT = 0
    RELIABLE = 1
    BEST_EFFORT = 2
    UNKNOWN = 3


class DurabilityPolicy(enum.IntEnum):
    SYSTEM_DEFAULT = 0
    TRANSIENT_LOCAL = 1
    VOLATILE = 2
    UNKNOWN = 3


class LivelinessPolicy(enum.IntEnum):
    SYSTEM_DEFAULT = 0
    AUTOMATIC = 1
    MANUAL_BY_TOPIC = 3
    UNKNOWN = 4


QoSHistoryPolicy = HistoryPolicy
QoSReliabilityPolicy = ReliabilityPolicy
QoSDurabilityPolicy = DurabilityPolicy
QoSLivelinessPolicy = LivelinessPolicy


class QoSProfile:
    __slots__ = ('_history', '_depth', '_reliability', '_durability', '_lifespan', '_deadline',
                 '_liveliness', '_liveliness_lease_duration', '_avoid_ros_namespace_conventions')

    def __init__(self, **kwargs):
        if 'history' not in kwargs:
            if 'depth' not in kwargs:
                raise InvalidQoSProfileException('History and/or depth settings are required.')
            kwargs['history'] = HistoryPolicy.KEEP_LAST
        self.history = kwargs.get('history')
        if self.history == HistoryPolicy.KEEP_LAST and kwargs.get('depth', 0) == 0 and 'depth' not in kwargs:
            raise InvalidQoSProfileException('History set to KEEP_LAST without a depth setting.')
        self.depth = kwargs.get('depth', 0)
        self.reliability = kwargs.get('reliability', ReliabilityPolicy.RELIABLE)
        self.durability = kwargs.get('durability', DurabilityPolicy.VOLATILE)
        self.lifespan = kwargs.get('lifespan', Duration())
        self.deadline = kwargs.get('deadline', Duration())
        self.liveliness = kwargs.get('liveliness', LivelinessPolicy.SYSTEM_DEFAULT)
        self.liveliness_lease_duration = kwargs.get('liveliness_lease_duration', Duration())
        self.avoid_ros_namespace_conventions = kwargs.get('avoid_ros_namespace_conventions', False)

    def _prop(name, policy=None):
        def get(self):
            return getattr(self, '_' + name)

        def set_(self, value):
            if policy is not None and not isinstance(value, policy):
                value = policy(value)
            setattr(self, '_' + name, value)
        return property(get, set_)

    history = _prop('history', HistoryPolicy)
    reliability = _prop('reliability', ReliabilityPolicy)
    durability = _prop('durability', DurabilityPolicy)
    liveliness = _prop('liveliness', LivelinessPolicy)
    depth = _prop('depth')
    lifespan = _prop('lifespan')
    deadline = _prop('deadline')
    liveliness_lease_duration = _prop('liveliness_lease_duration')
    avoid_ros_namespace_conventions = _prop('avoid_ros_namespace_conventions')
    del _prop

    def __eq__(self, other):
        if not isinstance(other, QoSProfile):
            return False
        return all(getattr(self, s) == getattr(other, s) for s in self.__slots__)

    def __repr__(self):
        return (f'QoSProfile(history={self.history!r}, depth={self.depth}, reliability={self.reliability!r}, '
                f'durability={self.durability!r})')

    def to_info(self):
        reliable = self.reliability in (ReliabilityPolicy.RELIABLE, ReliabilityPolicy.SYSTEM_DEFAULT)
        transient = self.durability == DurabilityPolicy.TRANSIENT_LOCAL
        keep_all = self.history == HistoryPolicy.KEEP_ALL
        depth = self.depth if self.depth else (1000 if keep_all else 1)
        return QoSInfo(reliable=reliable, transient_local=transient, depth=depth, keep_all=keep_all)


def _qos_from_arg(qos):
    if isinstance(qos, QoSProfile):
        return qos
    if isinstance(qos, int) and not isinstance(qos, bool):
        if qos < 0:
            raise ValueError('history depth must be greater or equal to 0')
        return QoSProfile(depth=qos)
    raise TypeError('Expected QoSProfile or int, but received {!r}'.format(type(qos)))


# ---------------------------------------------------------------------------
# time
# ---------------------------------------------------------------------------
class ClockType(enum.IntEnum):
    UNINITIALIZED = 0
    ROS_TIME = 1
    SYSTEM_TIME = 2
    STEADY_TIME = 3


S_TO_NS = 1000 * 1000 * 1000


class Duration:
    __slots__ = ('_nanoseconds',)

    def __init__(self, *, seconds=0, nanoseconds=0):
        total = int(float(seconds) * S_TO_NS) + int(nanoseconds)
        if abs(total) > 2 ** 63 - 1:
            raise OverflowError('Total nanoseconds value is too large to store in C duration.')
        self._nanoseconds = total

    @property
    def nanoseconds(self):
        return self._nanoseconds

    def __repr__(self):
        return f'Duration(nanoseconds={self._nanoseconds})'

    def __str__(self):
        return f'{self._nanoseconds / S_TO_NS} seconds' if self._nanoseconds != 2 ** 63 - 1 else 'Infinite'

    def _cmp(self, other, op):
        if not isinstance(other, Duration):
            return NotImplemented
        return op(self._nanoseconds, other._nanoseconds)

    def __eq__(self, other):
        return self._cmp(other, lambda a, b: a == b)

    def __ne__(self, other):
        r = self.__eq__(other)
        return r if r is NotImplemented else not r

    def __lt__(self, other):
        return self._cmp(other, lambda a, b: a < b)

    def __le__(self, other):
        return self._cmp(other, lambda a, b: a <= b)

    def __gt__(self, other):
        return self._cmp(other, lambda a, b: a > b)

    def __ge__(self, other):
        return self._cmp(other, lambda a, b: a >= b)

    def __hash__(self):
        return hash(self._nanoseconds)

    def to_msg(self):
        from builtin_interfaces.msg import Duration as DurationMsg
        sec, nsec = divmod(self._nanoseconds, S_TO_NS)
        return DurationMsg(sec=int(sec), nanosec=int(nsec))

    @classmethod
    def from_msg(cls, msg):
        return cls(seconds=msg.sec, nanoseconds=msg.nanosec)

    def get_c_duration(self):
        return self


class Time:
    __slots__ = ('_nanoseconds', '_clock_type')

    def __init__(self, *, seconds=0, nanoseconds=0, clock_type=ClockType.SYSTEM_TIME):
        if not isinstance(clock_type, ClockType):
            raise TypeError('Clock type must be a ClockType enum')
        if seconds < 0:
            raise ValueError('Seconds value must not be negative')
        if nanoseconds < 0:
            raise ValueError('Nanoseconds value must not be negative')
        total = int(float(seconds) * S_TO_NS) + int(nanoseconds)
        if total >= 2 ** 63:
            raise OverflowError('Total nanoseconds value is too large to store in C time point.')
        self._nanoseconds = total
        self._clock_type = clock_type

    @property
    def nanoseconds(self):
        return self._nanoseconds

    @property
    def clock_type(self):
        return self._clock_type

    def seconds_nanoseconds(self):
        return divmod(self._nanoseconds, S_TO_NS)

    def __repr__(self):
        return f'Time(nanoseconds={self._nanoseconds}, clock_type={self._clock_type.name})'

    def __add__(self, other):
        if isinstance(other, Duration):
            return Time(nanoseconds=self._nanoseconds + other.nanoseconds, clock_type=self._clock_type)
        return NotImplemented

    __radd__ = __add__

    def __sub__(self, other):
        if isinstance(other, Time):
            if other._clock_type != self._clock_type:
                raise TypeError("Can't subtract times with different clock types")
            return Duration(nanoseconds=self._nanoseconds - other._nanoseconds)
        if isinstance(other, Duration):
            if self._nanoseconds - other.nanoseconds < 0:
                raise ValueError('Time cannot be negative')
            return Time(nanoseconds=self._nanoseconds - other.nanoseconds, clock_type=self._clock_type)
        return NotImplemented

    def _check(self, other, verb):
        if not isinstance(other, Time):
            return False
        if self._clock_type != other._clock_type:
            raise TypeError(f"Can't {verb} times with different clock types")
        return True

    def __eq__(self, other):
        if not self._check(other, 'compare'):
            return NotImplemented
        return self._nanoseconds == other._nanoseconds

    def __ne__(self, other):
        r = self.__eq__(other)
        return r if r is NotImplemented else not r

    def __lt__(self, other):
        if not self._check(other, 'compare'):
            return NotImplemented
        return self._nanoseconds < other._nanoseconds

    def __le__(self, other):
        if not self._check(other, 'compare'):
            return NotImplemented
        return self._nanoseconds <= other._nanoseconds

    def __gt__(self, other):
        if not self._check(other, 'compare'):
            return NotImplemented
        return self._nanoseconds > other._nanoseconds

    def __ge__(self, other):
        if not self._check(other, 'compare'):
            return NotImplemented
        return self._nanoseconds >= other._nanoseconds

    def __hash__(self):
        return hash((self._nanoseconds, self._clock_type))

    def to_msg(self):
        from builtin_interfaces.msg import Time as TimeMsg
        sec, nsec = self.seconds_nanoseconds()
        return TimeMsg(sec=int(sec), nanosec=int(nsec))

    @classmethod
    def from_msg(cls, msg, clock_type=ClockType.ROS_TIME):
        from builtin_interfaces.msg import Time as TimeMsg
        if not isinstance(msg, TimeMsg):
            raise TypeError('Must pass a builtin_interfaces.msg.Time object')
        return cls(seconds=msg.sec, nanoseconds=msg.nanosec, clock_type=clock_type)


class JumpThreshold:
    def __init__(self, *, min_forward=None, min_backward=None, on_clock_change=True):
        self.min_forward, self.min_backward, self.on_clock_change = min_forward, min_backward, on_clock_change


class JumpHandle:
    def unregister(self):
        pass


class Clock:
    def __init__(self, *, clock_type=ClockType.SYSTEM_TIME):
        if not isinstance(clock_type, ClockType):
            raise TypeError('Clock type must be a ClockType enum')
        self._clock_type = clock_type

    @property
    def clock_type(self):
        return self._clock_type

    def __repr__(self):
        return f'Clock(clock_type={self._clock_type.name})'

    def now(self):
        rt = get_runtime(required=False)
        if rt is None:
            ns = int(_rt.REAL_TIME() * S_TO_NS)
        elif self._clock_type == ClockType.STEADY_TIME:
            ns = 1000 * S_TO_NS + rt.now_ns
        else:
            ns = rt.ros_time_ns()
        return Time(nanoseconds=ns, clock_type=self._clock_type)

    def create_jump_callback(self, threshold, *, pre_callback=None, post_callback=None):
        return JumpHandle()

    def sleep_until(self, until, context=None):
        rt = get_runtime()
        target = until.nanoseconds - (rt.ros_time_ns() - rt.now_ns) if self._clock_type != ClockType.STEADY_TIME \
            else until.nanoseconds - 1000 * S_TO_NS
        rt.sleep(max(0, target - rt.now_ns) / S_TO_NS)
        return True

    def sleep_for(self, rel_time, context=None):
        get_runtime().sleep(rel_time.nanoseconds / S_TO_NS)
        return True


class ROSClock(Clock):
    def __init__(self):
        super().__init__(clock_type=ClockType.ROS_TIME)

    @property
    def ros_time_is_active(self):
        return False

    def _set_ros_time_is_active(self, enabled):
        pass


# QoS presets (need Duration)
qos_profile_sensor_data = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5,
                                     reliability=ReliabilityPolicy.BEST_EFFORT,
                                     durability=DurabilityPolicy.VOLATILE)
qos_profile_system_default = QoSProfile(history=HistoryPolicy.SYSTEM_DEFAULT, depth=0,
                                        reliability=ReliabilityPolicy.SYSTEM_DEFAULT,
                                        durability=DurabilityPolicy.SYSTEM_DEFAULT)
qos_profile_services_default = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                                          reliability=ReliabilityPolicy.RELIABLE,
                                          durability=DurabilityPolicy.VOLATILE)
qos_profile_parameters = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1000,
                                    reliability=ReliabilityPolicy.RELIABLE,
                                    durability=DurabilityPolicy.VOLATILE)
qos_profile_parameter_events = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1000,
                                          reliability=ReliabilityPolicy.RELIABLE,
                                          durability=DurabilityPolicy.VOLATILE)
qos_profile_unknown = QoSProfile(history=HistoryPolicy.UNKNOWN, depth=0,
                                 reliability=ReliabilityPolicy.UNKNOWN,
                                 durability=DurabilityPolicy.UNKNOWN)


class QoSPresetProfiles(enum.Enum):
    UNKNOWN = qos_profile_unknown
    SYSTEM_DEFAULT = qos_profile_system_default
    SENSOR_DATA = qos_profile_sensor_data
    SERVICES_DEFAULT = qos_profile_services_default
    PARAMETERS = qos_profile_parameters
    PARAMETER_EVENTS = qos_profile_parameter_events

    @classmethod
    def short_keys(cls):
        return [m.name.lower() for m in cls]

    @classmethod
    def get_from_short_key(cls, name):
        return cls[name.upper()].value


# ---------------------------------------------------------------------------
# logging
# ---------------------------------------------------------------------------
class LoggingSeverity(enum.IntEnum):
    UNSET = 0
    DEBUG = 10
    INFO = 20
    WARN = 30
    ERROR = 40
    FATAL = 50


_LOGGER_LEVELS = {}


def _process_default_level():
    rt = get_runtime(required=False)
    proc = rt.current_process() if rt else None
    if proc is not None and proc.ros_args.log_level:
        text = str(proc.ros_args.log_level).split(':=')[-1].upper()
        if text in LoggingSeverity.__members__:
            return LoggingSeverity[text]
    return LoggingSeverity.INFO


class RcutilsLogger:
    def __init__(self, name=''):
        self.name = name
        self._throttle = {}
        self._once = set()
        self._skip = set()

    def get_child(self, name):
        if not name:
            raise ValueError('Child logger name must not be empty.')
        return RcutilsLogger(name=f'{self.name}.{name}' if self.name else name)

    def set_level(self, level):
        _LOGGER_LEVELS[self.name] = LoggingSeverity(level)

    def get_effective_level(self):
        name = self.name
        while True:
            if name in _LOGGER_LEVELS and _LOGGER_LEVELS[name] != LoggingSeverity.UNSET:
                return _LOGGER_LEVELS[name]
            if '.' not in name:
                break
            name = name.rsplit('.', 1)[0]
        return _process_default_level()

    def is_enabled_for(self, severity):
        return LoggingSeverity(severity) >= self.get_effective_level()

    def log(self, message, severity, **kwargs):
        severity = LoggingSeverity(severity)
        allowed = {'throttle_duration_sec', 'throttle_time_source_type', 'skip_first', 'once', 'name'}
        bad = set(kwargs) - allowed
        if bad:
            raise TypeError(f'Unsupported logging feature(s): {", ".join(sorted(bad))}')
        if not self.is_enabled_for(severity):
            return False
        frame = sys._getframe(2)
        site = (frame.f_code.co_filename, frame.f_lineno)
        if kwargs.get('once'):
            if site in self._once:
                return False
            self._once.add(site)
        if kwargs.get('skip_first'):
            if site not in self._skip:
                self._skip.add(site)
                return False
        period = kwargs.get('throttle_duration_sec')
        if period:
            rt = get_runtime(required=False)
            now = rt.now_s if rt else _rt.REAL_MONOTONIC()
            last = self._throttle.get(site)
            if last is not None and now - last < float(period):
                return False
            self._throttle[site] = now
        rt = get_runtime(required=False)
        text = str(message)
        if rt is None:
            print(f'[{severity.name}] [{self.name}]: {text}', file=sys.__stderr__)
        else:
            rt.node_log(rt.current_process(), self.name, severity.name, text)
        return True

    def debug(self, message, **kwargs):
        return self.log(message, LoggingSeverity.DEBUG, **kwargs)

    def info(self, message, **kwargs):
        return self.log(message, LoggingSeverity.INFO, **kwargs)

    def warning(self, message, **kwargs):
        return self.log(message, LoggingSeverity.WARN, **kwargs)

    def warn(self, message, **kwargs):
        # Humble keeps the deprecated alias working.
        return self.log(message, LoggingSeverity.WARN, **kwargs)

    def error(self, message, **kwargs):
        return self.log(message, LoggingSeverity.ERROR, **kwargs)

    def fatal(self, message, **kwargs):
        return self.log(message, LoggingSeverity.FATAL, **kwargs)


def get_logger(name):
    if not name:
        raise ValueError('Logger name must not be empty.')
    return RcutilsLogger(name=name)


def set_logger_level(name, level):
    _LOGGER_LEVELS[name] = LoggingSeverity(level)


def get_logger_effective_level(name):
    return RcutilsLogger(name).get_effective_level()


# ---------------------------------------------------------------------------
# parameters
# ---------------------------------------------------------------------------
PARAMETER_SEPARATOR_STRING = '.'


class Parameter:
    class Type(enum.IntEnum):
        NOT_SET = 0
        BOOL = 1
        INTEGER = 2
        DOUBLE = 3
        STRING = 4
        BYTE_ARRAY = 5
        BOOL_ARRAY = 6
        INTEGER_ARRAY = 7
        DOUBLE_ARRAY = 8
        STRING_ARRAY = 9

        @classmethod
        def from_parameter_value(cls, parameter_value):
            if parameter_value is None:
                return cls.NOT_SET
            if isinstance(parameter_value, bool):
                return cls.BOOL
            if isinstance(parameter_value, int):
                return cls.INTEGER
            if isinstance(parameter_value, float):
                return cls.DOUBLE
            if isinstance(parameter_value, str):
                return cls.STRING
            if isinstance(parameter_value, (list, tuple, array.array)):
                if all(isinstance(v, bytes) for v in parameter_value):
                    return cls.BYTE_ARRAY
                if all(isinstance(v, bool) for v in parameter_value):
                    return cls.BOOL_ARRAY
                if all(isinstance(v, int) for v in parameter_value):
                    return cls.INTEGER_ARRAY
                if all(isinstance(v, float) for v in parameter_value):
                    return cls.DOUBLE_ARRAY
                if all(isinstance(v, str) for v in parameter_value):
                    return cls.STRING_ARRAY
                raise TypeError('The given value is not a list of one of the allowed types'
                                f" '{parameter_value}'.")
            raise TypeError(f"The given value is not one of the allowed types '{parameter_value}'.")

        def check(self, parameter_value):
            T = Parameter.Type
            if self == T.NOT_SET:
                return parameter_value is None
            if self == T.BOOL:
                return isinstance(parameter_value, bool)
            if self == T.INTEGER:
                return isinstance(parameter_value, int) and not isinstance(parameter_value, bool)
            if self == T.DOUBLE:
                return isinstance(parameter_value, float)
            if self == T.STRING:
                return isinstance(parameter_value, str)
            seq = isinstance(parameter_value, (list, tuple, array.array))
            if self == T.BYTE_ARRAY:
                return seq and all(isinstance(v, bytes) and len(v) == 1 for v in parameter_value)
            if self == T.BOOL_ARRAY:
                return seq and all(isinstance(v, bool) for v in parameter_value)
            if self == T.INTEGER_ARRAY:
                return seq and all(isinstance(v, int) and not isinstance(v, bool) for v in parameter_value)
            if self == T.DOUBLE_ARRAY:
                return seq and all(isinstance(v, float) for v in parameter_value)
            if self == T.STRING_ARRAY:
                return seq and all(isinstance(v, str) for v in parameter_value)
            return False

    @classmethod
    def from_parameter_msg(cls, param_msg):
        value = parameter_value_to_python(param_msg.value)
        return cls(param_msg.name, Parameter.Type(param_msg.value.type), value)

    def __init__(self, name, type_=None, value=None):
        if type_ is None:
            type_ = Parameter.Type.from_parameter_value(value)
        if not isinstance(type_, Parameter.Type):
            raise TypeError("type must be an instance of '{}'".format(repr(Parameter.Type)))
        if not type_.check(value):
            raise ValueError(f"Type '{type_}' and value '{value}' do not agree")
        self._type_ = type_
        self._name = name
        self._value = value

    @property
    def name(self):
        return self._name

    @property
    def type_(self):
        return self._type_

    @property
    def value(self):
        return self._value

    def __repr__(self):
        return f'Parameter(name={self._name!r}, type={self._type_.name}, value={self._value!r})'

    def get_parameter_value(self):
        from rcl_interfaces.msg import ParameterValue
        pv = ParameterValue(type=int(self._type_))
        T = Parameter.Type
        if self._type_ == T.BOOL:
            pv.bool_value = self._value
        elif self._type_ == T.INTEGER:
            pv.integer_value = self._value
        elif self._type_ == T.DOUBLE:
            pv.double_value = self._value
        elif self._type_ == T.STRING:
            pv.string_value = self._value
        elif self._type_ == T.BYTE_ARRAY:
            pv.byte_array_value = list(self._value)
        elif self._type_ == T.BOOL_ARRAY:
            pv.bool_array_value = list(self._value)
        elif self._type_ == T.INTEGER_ARRAY:
            pv.integer_array_value = list(self._value)
        elif self._type_ == T.DOUBLE_ARRAY:
            pv.double_array_value = list(self._value)
        elif self._type_ == T.STRING_ARRAY:
            pv.string_array_value = list(self._value)
        return pv

    def to_parameter_msg(self):
        from rcl_interfaces.msg import Parameter as ParameterMsg
        return ParameterMsg(name=self._name, value=self.get_parameter_value())


def parameter_value_to_python(parameter_value):
    T = Parameter.Type
    t = parameter_value.type
    if t == T.BOOL:
        return parameter_value.bool_value
    if t == T.INTEGER:
        return parameter_value.integer_value
    if t == T.DOUBLE:
        return parameter_value.double_value
    if t == T.STRING:
        return parameter_value.string_value
    if t == T.BYTE_ARRAY:
        return list(parameter_value.byte_array_value)
    if t == T.BOOL_ARRAY:
        return list(parameter_value.bool_array_value)
    if t == T.INTEGER_ARRAY:
        return list(parameter_value.integer_array_value)
    if t == T.DOUBLE_ARRAY:
        return list(parameter_value.double_array_value)
    if t == T.STRING_ARRAY:
        return list(parameter_value.string_array_value)
    if t == T.NOT_SET:
        return None
    raise RuntimeError(f'unexpected parameter type {t}')


def _flatten(prefix, mapping, out):
    for key, value in mapping.items():
        name = f'{prefix}.{key}' if prefix else str(key)
        if isinstance(value, dict):
            _flatten(name, value, out)
        else:
            out[name] = value
    return out


def _node_pattern_matches(pattern, fqn):
    pattern = str(pattern)
    if not pattern.startswith('/'):
        pattern = '/' + pattern
    if pattern == fqn or pattern == '/**':
        return True
    regex = ''
    i = 0
    while i < len(pattern):
        if pattern.startswith('**', i):
            regex += '.*'
            i += 2
        elif pattern[i] == '*':
            regex += '[^/]*'
            i += 1
        else:
            regex += re.escape(pattern[i])
            i += 1
    return re.fullmatch(regex, fqn) is not None


def overrides_for(fqn, ros_args):
    """Parameter overrides rcl would give node ``fqn`` from its process args."""
    import yaml
    out = {}
    for source in ros_args.params_files:
        if isinstance(source, dict):
            data = source
        else:
            with open(source, 'r', encoding='utf-8') as fh:
                data = yaml.safe_load(fh) or {}
        for key, block in data.items():
            if not isinstance(block, dict) or 'ros__parameters' not in block:
                continue
            if _node_pattern_matches(key, fqn):
                params = block['ros__parameters'] or {}
                _flatten('', params, out)
    for name, value in ros_args.params:
        out[name] = value
    return out


# ---------------------------------------------------------------------------
# futures / tasks / callback groups
# ---------------------------------------------------------------------------
class Future:
    def __init__(self, *, executor=None):
        self._state = 'PENDING'
        self._result = None
        self._exception = None
        self._exception_fetched = False
        self._callbacks = []
        self._executor = executor
        self._waiters = []

    def __await__(self):
        while not self.done():
            yield
        return self.result()

    def cancel(self):
        if self._state == 'PENDING':
            self._state = 'CANCELLED'
            self._schedule_or_invoke_done_callbacks()

    def cancelled(self):
        return self._state == 'CANCELLED'

    def done(self):
        return self._state != 'PENDING'

    def result(self):
        if self._exception:
            self._exception_fetched = True
            raise self._exception
        return self._result

    def exception(self):
        self._exception_fetched = True
        return self._exception

    def set_result(self, result):
        self._result = result
        self._state = 'FINISHED'
        self._schedule_or_invoke_done_callbacks()

    def set_exception(self, exception):
        self._exception = exception
        self._exception_fetched = False
        self._state = 'FINISHED'
        self._schedule_or_invoke_done_callbacks()

    def _schedule_or_invoke_done_callbacks(self):
        callbacks, self._callbacks = self._callbacks, []
        for cb in callbacks:
            if self._executor is not None:
                self._executor.create_task(cb, self)
            else:
                cb(self)
        waiters, self._waiters = self._waiters, []
        for w in waiters:
            w()

    def _set_executor(self, executor):
        self._executor = executor

    def add_done_callback(self, callback):
        if self.done():
            if self._executor is not None:
                self._executor.create_task(callback, self)
            else:
                callback(self)
        else:
            self._callbacks.append(callback)


class Task(Future):
    def __init__(self, handler, args=None, kwargs=None, executor=None):
        super().__init__(executor=executor)
        self._handler = handler
        self._args = args or ()
        self._kwargs = kwargs or {}
        self._executing = False

    def __call__(self):
        if self.done() or self._executing:
            return
        self._executing = True
        try:
            result = self._handler(*self._args, **self._kwargs)
            if hasattr(result, '__await__'):
                # coroutine callbacks: run to completion synchronously
                try:
                    result.send(None)
                    while True:
                        result.send(None)
                except StopIteration as stop:
                    result = stop.value
            self.set_result(result)
        except Exception as exc:  # noqa: BLE001
            self.set_exception(exc)
        finally:
            self._executing = False

    def executing(self):
        return self._executing


class CallbackGroup:
    def __init__(self):
        self.entities = set()

    def add_entity(self, entity):
        self.entities.add(entity)

    def has_entity(self, entity):
        return entity in self.entities

    def can_execute(self, entity):
        return True

    def beginning_execution(self, entity):
        return True

    def ending_execution(self, entity):
        pass


class ReentrantCallbackGroup(CallbackGroup):
    pass


class MutuallyExclusiveCallbackGroup(CallbackGroup):
    pass


# ---------------------------------------------------------------------------
# context + executors
# ---------------------------------------------------------------------------
class Context:
    def __init__(self):
        self._ok = False
        self._init_done = False
        self._callbacks = []
        self._domain_id = 0
        self.process = None
        self._executors = []

    def init(self, args=None, *, initialize_logging=True, domain_id=None):
        if self._init_done:
            raise RuntimeError('Context.init() must only be called once')
        self._init_done = True
        self._ok = True
        self._domain_id = int(domain_id) if domain_id is not None else _env_domain()

    def ok(self):
        return self._ok

    def shutdown(self):
        if not self._ok:
            raise RuntimeError('Context must be initialized before it can be shutdown')
        self._shutdown()

    def try_shutdown(self):
        if self._ok:
            self._shutdown()

    def _shutdown(self):
        self._ok = False
        for cb in list(self._callbacks):
            try:
                cb()
            except Exception:  # noqa: BLE001
                pass
        for ex in list(self._executors):
            ex._wake_for_shutdown()

    def on_shutdown(self, callback):
        self._callbacks.append(callback)

    def get_domain_id(self):
        return self._domain_id

    def destroy(self):
        pass

    def track_node(self, node):
        pass

    def untrack_node(self, node):
        pass


def _env_domain():
    import os
    try:
        return int(os.environ.get('ROS_DOMAIN_ID', '0'))
    except ValueError:
        return 0


_FALLBACK_CONTEXT = Context()


def get_default_context(*, shutting_down=False):
    rt = get_runtime(required=False)
    proc = rt.current_process() if rt else None
    if proc is None:
        return _FALLBACK_CONTEXT
    if proc.context is None:
        ctx = Context()
        ctx.process = proc
        proc.context = ctx
        proc.contexts.append(ctx)
    return proc.context


class Executor:
    """rclpy Executor running on the simulation scheduler."""

    def __init__(self, *, context=None):
        self._context = get_default_context() if context is None else context
        self._context._executors.append(self)
        self._nodes = []
        self._ready = collections.deque()
        self._mode = None            # None | 'spin' | 'once'
        self._thread = None
        self._stop_on = None         # future for spin_until_future_complete
        self._is_shutdown = False
        self._pending_exc = None

    @property
    def context(self):
        return self._context

    def add_node(self, node):
        if node in self._nodes:
            return False
        if node._sim_executor is not None and node._sim_executor is not self:
            node._sim_executor.remove_node(node)
        self._nodes.append(node)
        node._sim_executor = self
        while node._sim_pending:
            self._ready.append(node._sim_pending.popleft())
        self._dispatch()
        return True

    def remove_node(self, node):
        if node in self._nodes:
            self._nodes.remove(node)
        if node._sim_executor is self:
            node._sim_executor = None
            kept = collections.deque(t for t in self._ready if t[1] is not node)
            dropped = [t for t in self._ready if t[1] is node]
            self._ready = kept
            node._sim_pending.extend(dropped)

    def get_nodes(self):
        return list(self._nodes)

    def create_task(self, callback, *args, **kwargs):
        task = Task(callback, args, kwargs, executor=self)
        self._ready.append(('task', None, task))
        self._dispatch()
        return task

    def shutdown(self, timeout_sec=None):
        self._is_shutdown = True
        self._wake_for_shutdown()
        for node in list(self._nodes):
            self.remove_node(node)
        return True

    def _wake_for_shutdown(self):
        th = self._thread
        if th is not None and th.state == 'blocked':
            get_runtime().wake(th)

    def wake(self):
        self._dispatch()

    # -- the engine side ---------------------------------------------------
    def _enqueue(self, token):
        self._ready.append(token)
        self._dispatch()

    def _dispatch(self):
        if not self._ready:
            return
        rt = get_runtime(required=False)
        if rt is None:
            return
        if self._mode == 'spin':
            th = self._thread
            if th is not None and th.state != 'blocked':
                return  # the spinning thread itself is running; it will drain
            self._drain(rt)
        elif self._mode == 'once':
            th = self._thread
            if th is not None and th.state == 'blocked':
                rt.wake(th)

    def _drain(self, rt):
        while self._ready and self._mode == 'spin':
            token = self._ready[0]
            node = token[1]
            if node is not None and node._sim_busy_until > rt.now_ns:
                rt.at(node._sim_busy_until, self._dispatch)
                return
            self._ready.popleft()
            try:
                self._execute(token, rt)
            except BaseException as exc:  # noqa: BLE001 - re-raised in the spinning thread
                th = self._thread
                self._mode = None
                if th is not None and th.state == 'blocked':
                    rt.wake(th, exc)
                return
            if self._stop_on is not None and self._stop_on.done():
                th = self._thread
                if th is not None and th.state == 'blocked':
                    rt.wake(th)
                return

    def _execute(self, token, rt):
        kind, node, payload = token
        prev_proc = getattr(rt.tls, 'process', None)
        prev_node = getattr(rt.tls, 'executing_node', None)
        if node is not None:
            rt.tls.process = node._sim_process
        rt.tls.executing_node = node
        try:
            if kind == 'timer':
                timer = payload
                timer._sim_pending = False
                if timer.is_canceled() or timer._destroyed:
                    return
                timer._call()
            elif kind == 'sub':
                sub = payload
                if not sub._sim_queue or sub._destroyed:
                    return
                msg = sub._sim_queue.popleft()
                sub.callback(msg)
            elif kind == 'srv':
                service, request, reply = payload
                if service._destroyed:
                    return
                response = service.callback(request, service.srv_type.Response())
                reply(response)
            elif kind == 'resp':
                client, future, response = payload
                if not future.done():
                    future.set_result(response)
            elif kind == 'task':
                payload()
        finally:
            if node is not None and node._sim_busy_ns:
                node._sim_busy_until = rt.now_ns + node._sim_busy_ns
                node._sim_busy_ns = 0
            rt.tls.process = prev_proc
            rt.tls.executing_node = prev_node

    # -- the thread side ---------------------------------------------------
    def _sim_thread(self):
        rt = get_runtime()
        if not rt.in_sim_thread():
            raise RuntimeError('rclpy executors must be spun from a node process thread in the simulator')
        return rt, rt.current_thread()

    def spin(self):
        rt, th = self._sim_thread()
        self._thread = th
        try:
            while self._context.ok() and not self._is_shutdown:
                self._mode = 'spin'
                self._drain_here(rt)
                if not (self._context.ok() and not self._is_shutdown):
                    break
                rt.block('spin')
        finally:
            self._mode = None
            self._thread = None

    def _drain_here(self, rt):
        while self._ready and self._context.ok():
            token = self._ready[0]
            node = token[1]
            if node is not None and node._sim_busy_until > rt.now_ns:
                rt.sleep((node._sim_busy_until - rt.now_ns) / NS)
                continue
            self._ready.popleft()
            self._execute(token, rt)
            if self._stop_on is not None and self._stop_on.done():
                return

    def spin_once(self, timeout_sec=None):
        rt, th = self._sim_thread()
        if not self._ready:
            if timeout_sec is not None and timeout_sec == 0:
                return
            self._thread = th
            self._mode = 'once'
            try:
                until = None if timeout_sec is None or timeout_sec < 0 else \
                    rt.now_ns + int(round(float(timeout_sec) * NS))
                rt.block('spin_once', until_ns=until)
            finally:
                self._mode = None
                self._thread = None
        if self._ready and self._context.ok():
            token = self._ready.popleft()
            self._execute(token, rt)

    def spin_until_future_complete(self, future, timeout_sec=None):
        rt, th = self._sim_thread()
        if future.done():
            return
        future._set_executor(self)
        deadline = None if timeout_sec is None or timeout_sec < 0 else \
            rt.now_ns + int(round(float(timeout_sec) * NS))
        self._thread = th
        self._stop_on = future
        future._waiters.append(lambda: rt.wake(th) if th.state == 'blocked' else None)
        try:
            while not future.done() and self._context.ok() and not self._is_shutdown:
                self._mode = 'spin'
                self._drain_here(rt)
                if future.done():
                    break
                if deadline is not None and rt.now_ns >= deadline:
                    break
                rt.block('spin_until_future_complete', until_ns=deadline)
        finally:
            self._mode = None
            self._stop_on = None
            self._thread = None

    def spin_once_until_future_complete(self, future, timeout_sec=None):
        self.spin_once(timeout_sec)

    def spin_until_future_complete_or(self, future, timeout_sec=None):
        self.spin_until_future_complete(future, timeout_sec)


class SingleThreadedExecutor(Executor):
    pass


class MultiThreadedExecutor(Executor):
    def __init__(self, num_threads=None, *, context=None):
        super().__init__(context=context)
        self._num_threads = num_threads or 2


def get_global_executor():
    ctx = get_default_context()
    ex = getattr(ctx, '_global_executor', None)
    if ex is None:
        ex = SingleThreadedExecutor(context=ctx)
        ctx._global_executor = ex
    return ex


# ---------------------------------------------------------------------------
# entities
# ---------------------------------------------------------------------------
class Publisher:
    def __init__(self, node, msg_type, topic, qos_profile, callback_group=None):
        self._node = node
        self.msg_type = msg_type
        self.topic = topic
        self.qos_profile = qos_profile
        self.callback_group = callback_group
        self._ep = Endpoint('pub', node, topic, msg_type, qos_profile.to_info())
        get_runtime().graph.add(self._ep)
        self._destroyed = False

    @property
    def topic_name(self):
        return self.topic

    @property
    def handle(self):
        return self

    def publish(self, msg):
        if self._destroyed:
            raise InvalidHandle('publisher has been destroyed')
        if not isinstance(msg, self.msg_type):
            raise TypeError(f'Expected {self.msg_type}, got {type(msg)}')
        get_runtime().graph.publish(self._ep, msg)

    def get_subscription_count(self):
        rt = get_runtime()
        return sum(1 for s in rt.graph.subs.get(self.topic, ()) if s.id in self._ep.matched and s.alive)

    def assert_liveliness(self):
        pass

    def wait_for_all_acked(self, timeout=None):
        return True

    def destroy(self):
        if not self._destroyed:
            self._destroyed = True
            get_runtime().graph.remove(self._ep)


class Subscription:
    def __init__(self, node, msg_type, topic, callback, qos_profile, callback_group=None, raw=False):
        self._node = node
        self.msg_type = msg_type
        self.topic = topic
        self.callback = callback
        self.qos_profile = qos_profile
        self.callback_group = callback_group
        self.raw = raw
        info = qos_profile.to_info()
        self._sim_queue = collections.deque(maxlen=None if info.keep_all else max(1, info.depth))
        self._ep = Endpoint('sub', node, topic, msg_type, info)
        self._ep.subscription = self
        self._destroyed = False
        get_runtime().graph.add(self._ep)

    @property
    def topic_name(self):
        return self.topic

    @property
    def handle(self):
        return self

    def get_publisher_count(self):
        rt = get_runtime()
        return sum(1 for p in rt.graph.pubs.get(self.topic, ()) if p.id in self._ep.matched and p.alive)

    def destroy(self):
        if not self._destroyed:
            self._destroyed = True
            get_runtime().graph.remove(self._ep)


class Timer:
    def __init__(self, node, callback, period_ns, clock, callback_group=None):
        self._node = node
        self.callback = callback
        self.timer_period_ns = int(period_ns)
        self._clock = clock
        self.callback_group = callback_group
        self._canceled = False
        self._destroyed = False
        self._sim_pending = False
        self._next_ns = None
        self._event = None
        self._arm(get_runtime().now_ns)

    @property
    def clock(self):
        return self._clock

    @property
    def handle(self):
        return self

    def _arm(self, base_ns):
        rt = get_runtime()
        if self.timer_period_ns <= 0:
            self._next_ns = rt.now_ns
        else:
            self._next_ns = base_ns + self.timer_period_ns
        self._event = rt.at(self._next_ns, self._fire)

    def _fire(self):
        if self._canceled or self._destroyed:
            return
        rt = get_runtime()
        period = max(self.timer_period_ns, 1)
        nxt = self._next_ns + period
        if nxt <= rt.now_ns:
            missed = (rt.now_ns - nxt) // period + 1
            nxt += missed * period
        self._next_ns = nxt
        self._event = rt.at(nxt, self._fire)
        if not self._sim_pending:
            self._sim_pending = True
            self._node._sim_ready(('timer', self._node, self))

    def _call(self):
        try:
            self.callback()
        except TypeError as exc:
            # rclpy >= Iron passes the TimerInfo; Humble calls with no args.
            raise exc

    def cancel(self):
        self._canceled = True
        if self._event is not None:
            self._event.cancel()

    def is_canceled(self):
        return self._canceled

    def reset(self):
        self._canceled = False
        if self._event is not None:
            self._event.cancel()
        self._arm(get_runtime().now_ns)

    def is_ready(self):
        return self._sim_pending

    def time_since_last_call(self):
        return max(0, get_runtime().now_ns - (self._next_ns - self.timer_period_ns))

    def time_until_next_call(self):
        if self._canceled:
            return None
        return max(0, self._next_ns - get_runtime().now_ns)

    def destroy(self):
        self._destroyed = True
        self.cancel()


class Rate:
    def __init__(self, timer, *, context=None):
        self._timer = timer
        self._period_ns = timer.timer_period_ns
        self._next = get_runtime().now_ns + self._period_ns

    def sleep(self):
        rt = get_runtime()
        if not rt.in_sim_thread():
            rt.sim_log('ERROR', 'rclpy', 'Rate.sleep() called inside an executor callback: on a real '
                                         'single-threaded executor this blocks forever')
        if self._next <= rt.now_ns:
            self._next = rt.now_ns + self._period_ns
        rt.sleep((self._next - rt.now_ns) / NS)
        self._next += self._period_ns

    def destroy(self):
        self._timer.destroy()


class Service:
    def __init__(self, node, srv_type, srv_name, callback, qos_profile, callback_group=None):
        self._node = node
        self.srv_type = srv_type
        self.srv_name = srv_name
        self.callback = callback
        self.qos_profile = qos_profile
        self.callback_group = callback_group
        self._destroyed = False
        rt = get_runtime()
        self._visible_ns = rt.now_ns + int(rt.graph.discovery_delay_s * NS)
        rt.graph.services.setdefault(srv_name, []).append(self)

    @property
    def service_name(self):
        return self.srv_name

    @property
    def handle(self):
        return self

    def _sim_request(self, request, reply):
        self._node._sim_ready(('srv', self._node, (self, request, reply)))

    def destroy(self):
        self._destroyed = True
        rt = get_runtime()
        lst = rt.graph.services.get(self.srv_name, [])
        if self in lst:
            lst.remove(self)


class Client:
    def __init__(self, node, srv_type, srv_name, qos_profile, callback_group=None):
        self._node = node
        self.srv_type = srv_type
        self.srv_name = srv_name
        self.qos_profile = qos_profile
        self.callback_group = callback_group
        self._destroyed = False

    @property
    def service_name(self):
        return self.srv_name

    @property
    def handle(self):
        return self

    def _server(self):
        rt = get_runtime()
        for srv in rt.graph.services.get(self.srv_name, []):
            if not srv._destroyed and srv._visible_ns <= rt.now_ns and \
                    _rt.msggen.full_type_name(srv.srv_type) == _rt.msggen.full_type_name(self.srv_type):
                return srv
        return None

    def service_is_ready(self):
        return self._server() is not None

    def wait_for_service(self, timeout_sec=None):
        rt = get_runtime()
        deadline = None if timeout_sec is None else rt.now_ns + int(timeout_sec * NS)
        while not self.service_is_ready():
            if deadline is not None and rt.now_ns >= deadline:
                return False
            if not rt.in_sim_thread():
                return False
            rt.sleep(0.1)
        return True

    def call_async(self, request):
        if not isinstance(request, self.srv_type.Request):
            raise TypeError(f'Request must be instance of {self.srv_type.Request}')
        rt = get_runtime()
        future = Future()
        srv = self._server()
        if srv is None:
            return future  # never completes, as with a missing server
        lat = rt.graph.default_latency_s
        req = _rt.msggen.copy_message(request)

        def reply(response):
            resp = _rt.msggen.copy_message(response)
            rt.after(lat, self._node._sim_ready, ('resp', self._node, (self, future, resp)))

        rt.after(lat, srv._sim_request, req, reply)
        return future

    def call(self, request, timeout_sec=None):
        rt = get_runtime()
        future = self.call_async(request)
        if not rt.in_sim_thread():
            raise RuntimeError('Client.call() inside a callback deadlocks a single-threaded executor')
        deadline = None if timeout_sec is None else rt.now_ns + int(timeout_sec * NS)
        while not future.done():
            if deadline is not None and rt.now_ns >= deadline:
                return None
            rt.sleep(0.001)
        return future.result()

    def remove_pending_request(self, future):
        future.cancel()

    def destroy(self):
        self._destroyed = True


class GuardCondition:
    def __init__(self, callback, callback_group=None, context=None, node=None):
        self.callback = callback
        self._node = node

    def trigger(self):
        if self._node is not None and self.callback is not None:
            self._node._sim_ready(('task', self._node, Task(self.callback)))

    def destroy(self):
        pass


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------
class Node:
    PARAM_REL_TOL = 1e-6

    def __init__(self, node_name, *, context=None, cli_args=None, namespace=None,
                 use_global_arguments=True, enable_rosout=True, start_parameter_services=True,
                 parameter_overrides=None, allow_undeclared_parameters=False,
                 automatically_declare_parameters_from_overrides=False):
        rt = get_runtime()
        self._context = get_default_context() if context is None else context
        if not self._context.ok():
            raise RCLError("Failed to create node: rcl node's context is invalid, at "
                           "./src/rcl/node.c:276 (did you call rclpy.init()?)")
        proc = rt.current_process()
        self._sim_process = proc
        ros_args = proc.ros_args if (proc is not None and use_global_arguments) else RosArgs()
        local_args = RosArgs.parse(cli_args) if cli_args else RosArgs()
        name = local_args.node_name or ros_args.node_name or node_name
        _validate_node_name(name)
        ns = _normalize_ns(local_args.namespace or ros_args.namespace or namespace or '/')
        self._name = name
        self._namespace = ns
        self.sim_fqn = (ns.rstrip('/') + '/' + name) if ns != '/' else '/' + name
        self._remaps = list(ros_args.remaps) + list(local_args.remaps)
        self._logger = get_logger(self.sim_fqn.lstrip('/').replace('/', '.'))
        self._clock = ROSClock()
        self._parameters = {}
        self._descriptors = {}
        self._allow_undeclared = allow_undeclared_parameters
        self._set_param_callbacks = []
        overrides = {}
        for args in (ros_args, local_args):
            for k, v in overrides_for(self.sim_fqn, args).items():
                overrides[k] = v
        self._parameter_overrides = {}
        for k, v in overrides.items():
            try:
                self._parameter_overrides[k] = Parameter(k, value=v)
            except (TypeError, ValueError) as exc:
                raise RCLError(f"Failed to parse parameter override '{k}': {exc}") from exc
        if parameter_overrides is not None:
            for p in parameter_overrides:
                self._parameter_overrides[p.name] = p
        self.publishers = []
        self.subscriptions = []
        self.timers = []
        self.services = []
        self.clients = []
        self.guards = []
        self._default_callback_group = MutuallyExclusiveCallbackGroup()
        self._sim_executor = None
        self._sim_pending = collections.deque()
        self._sim_busy_ns = 0
        self._sim_busy_until = 0
        self._destroyed = False
        self.declare_parameter('use_sim_time', False)
        if automatically_declare_parameters_from_overrides:
            for p in list(self._parameter_overrides.values()):
                if p.name not in self._parameters:
                    self.declare_parameter(p.name, p.value)
        if proc is not None:
            proc.nodes.append(self)
        rt.internal_nodes.append(self)
        self._context.track_node(self)

    # -- identity ---------------------------------------------------------------
    def get_name(self):
        return self._name

    def get_namespace(self):
        return self._namespace

    def get_fully_qualified_name(self):
        return self.sim_fqn

    @property
    def context(self):
        return self._context

    @property
    def handle(self):
        return self

    @property
    def executor(self):
        return self._sim_executor

    @executor.setter
    def executor(self, new_executor):
        if new_executor is None:
            if self._sim_executor is not None:
                self._sim_executor.remove_node(self)
        else:
            new_executor.add_node(self)

    @property
    def default_callback_group(self):
        return self._default_callback_group

    def get_logger(self):
        return self._logger

    def get_clock(self):
        return self._clock

    def sim_log_incompatible(self, text):
        self._logger.warning(text)

    # -- names -----------------------------------------------------------------
    def resolve_topic_name(self, topic, *, only_expand=False):
        full = expand_name(topic, self._name, self._namespace)
        if only_expand:
            return full
        for src, dst in self._remaps:
            if expand_name(src, self._name, self._namespace) == full:
                full = expand_name(dst, self._name, self._namespace)
                break
        return full

    resolve_service_name = resolve_topic_name

    def _resolve(self, name, kind='topic'):
        if not isinstance(name, str):
            raise TypeError(f'{kind} name must be a str, got {type(name)}')
        full = self.resolve_topic_name(name)
        _validate_full_name(full, 'service' if kind == 'service' else 'topic')
        return full

    # -- executor plumbing -------------------------------------------------------
    def _sim_ready(self, token):
        if self._destroyed:
            return
        ex = self._sim_executor
        if ex is None:
            self._sim_pending.append(token)
            # bound the backlog of an unspun node (real DDS bounds per-reader history)
            if len(self._sim_pending) > 10000:
                self._sim_pending.popleft()
        else:
            ex._enqueue(token)

    def sim_deliver(self, ep, msg):
        sub = ep.subscription
        if sub._destroyed:
            return
        sub._sim_queue.append(msg)
        self._sim_ready(('sub', self, sub))

    def sim_teardown(self):
        self.destroy_node()

    # -- entities ----------------------------------------------------------------
    def create_publisher(self, msg_type, topic, qos_profile, *, callback_group=None,
                         event_callbacks=None, qos_overriding_options=None, publisher_class=None):
        qos = _qos_from_arg(qos_profile)
        pub = Publisher(self, msg_type, self._resolve(topic), qos, callback_group)
        self.publishers.append(pub)
        return pub

    def create_subscription(self, msg_type, topic, callback, qos_profile, *, callback_group=None,
                            event_callbacks=None, qos_overriding_options=None, raw=False):
        qos = _qos_from_arg(qos_profile)
        sub = Subscription(self, msg_type, self._resolve(topic), callback, qos, callback_group, raw)
        self.subscriptions.append(sub)
        return sub

    def create_timer(self, timer_period_sec, callback, callback_group=None, clock=None):
        period_ns = int(float(timer_period_sec) * S_TO_NS)
        timer = Timer(self, callback, period_ns, clock or self._clock, callback_group)
        self.timers.append(timer)
        return timer

    def create_rate(self, frequency, clock=None):
        if frequency <= 0:
            raise ValueError('frequency must be > 0')
        timer = Timer(self, lambda: None, int(S_TO_NS / frequency), clock or self._clock)
        return Rate(timer, context=self._context)

    def create_service(self, srv_type, srv_name, callback, *, qos_profile=qos_profile_services_default,
                       callback_group=None):
        srv = Service(self, srv_type, self._resolve(srv_name, 'service'), callback, qos_profile, callback_group)
        self.services.append(srv)
        return srv

    def create_client(self, srv_type, srv_name, *, qos_profile=qos_profile_services_default,
                      callback_group=None):
        cli = Client(self, srv_type, self._resolve(srv_name, 'service'), qos_profile, callback_group)
        self.clients.append(cli)
        return cli

    def create_guard_condition(self, callback, callback_group=None):
        gc = GuardCondition(callback, callback_group, node=self)
        self.guards.append(gc)
        return gc

    def destroy_publisher(self, publisher):
        if publisher in self.publishers:
            self.publishers.remove(publisher)
        publisher.destroy()
        return True

    def destroy_subscription(self, subscription):
        if subscription in self.subscriptions:
            self.subscriptions.remove(subscription)
        subscription.destroy()
        return True

    def destroy_timer(self, timer):
        if timer in self.timers:
            self.timers.remove(timer)
        timer.destroy()
        return True

    def destroy_service(self, service):
        if service in self.services:
            self.services.remove(service)
        service.destroy()
        return True

    def destroy_client(self, client):
        if client in self.clients:
            self.clients.remove(client)
        client.destroy()
        return True

    def destroy_guard_condition(self, guard):
        if guard in self.guards:
            self.guards.remove(guard)
        return True

    def destroy_rate(self, rate):
        rate.destroy()
        return True

    def destroy_node(self):
        if self._destroyed:
            return True
        for ent in self.publishers + self.subscriptions + self.timers + self.services + self.clients:
            ent.destroy()
        self.publishers, self.subscriptions, self.timers, self.services, self.clients = [], [], [], [], []
        if self._sim_executor is not None:
            self._sim_executor.remove_node(self)
        self._destroyed = True
        rt = get_runtime(required=False)
        if rt is not None and self in rt.internal_nodes:
            rt.internal_nodes.remove(self)
        if self._sim_process is not None and self in self._sim_process.nodes:
            self._sim_process.nodes.remove(self)
        return True

    # -- graph introspection ------------------------------------------------------
    def count_publishers(self, topic_name):
        rt = get_runtime()
        return len([p for p in rt.graph.pubs.get(self.resolve_topic_name(topic_name), ()) if p.alive])

    def count_subscribers(self, topic_name):
        rt = get_runtime()
        return len([s for s in rt.graph.subs.get(self.resolve_topic_name(topic_name), ()) if s.alive])

    def get_topic_names_and_types(self, no_demangle=False):
        return get_runtime().graph.topic_names_and_types()

    def get_node_names(self):
        rt = get_runtime()
        return [getattr(n, 'sim_fqn', '/?').rsplit('/', 1)[-1] for n in rt.internal_nodes]

    def get_node_names_and_namespaces(self):
        rt = get_runtime()
        out = []
        for n in rt.internal_nodes:
            fqn = getattr(n, 'sim_fqn', '/?')
            ns, _, name = fqn.rpartition('/')
            out.append((name, ns or '/'))
        return out

    def get_service_names_and_types(self):
        rt = get_runtime()
        return sorted((name, [_rt.msggen.full_type_name(s[0].srv_type)])
                      for name, s in rt.graph.services.items() if s)

    def get_publishers_info_by_topic(self, topic_name, no_mangle=False):
        rt = get_runtime()
        return [TopicEndpointInfo(p) for p in rt.graph.pubs.get(self.resolve_topic_name(topic_name), ()) if p.alive]

    def get_subscriptions_info_by_topic(self, topic_name, no_mangle=False):
        rt = get_runtime()
        return [TopicEndpointInfo(s) for s in rt.graph.subs.get(self.resolve_topic_name(topic_name), ()) if s.alive]

    # -- parameters ----------------------------------------------------------------
    def declare_parameter(self, name, value=None, descriptor=None, ignore_override=False):
        if value is None and descriptor is None:
            args = (name,)
        else:
            from rcl_interfaces.msg import ParameterDescriptor
            descriptor = ParameterDescriptor() if descriptor is None else descriptor
            args = (name, value, descriptor)
        return self.declare_parameters('', [args], ignore_override)[0]

    def declare_parameters(self, namespace, parameters, ignore_override=False):
        from rcl_interfaces.msg import ParameterDescriptor
        result = []
        for entry in parameters:
            if not isinstance(entry, tuple) or not 1 <= len(entry) <= 3:
                raise TypeError('Invalid parameter tuple length')
            name = entry[0]
            if not isinstance(name, str):
                raise TypeError('First element of a parameter tuple must be a string')
            full = f'{namespace}.{name}' if namespace else name
            value = entry[1] if len(entry) > 1 else None
            descriptor = entry[2] if len(entry) > 2 else ParameterDescriptor()
            if len(entry) == 1:
                warnings.warn(
                    f"when declaring parameter named '{full}', declaring a parameter only providing "
                    'its name is deprecated. You have to either:\n\t- Pass a name and a default value '
                    "different to 'PARAMETER NOT SET' (and optionally a descriptor).\n\t- Pass a name and "
                    'a parameter type.\n\t- Pass a name and a descriptor with `dynamic_typing=True',
                    UserWarning)
                descriptor.dynamic_typing = True
            if full in self._parameters:
                raise ParameterAlreadyDeclaredException(full)
            if not full or full.startswith('.') or '..' in full:
                raise InvalidParameterException(full)
            declared_type = None
            if isinstance(value, Parameter.Type):
                if value == Parameter.Type.NOT_SET:
                    raise ValueError(f"Cannot declare parameter '{full}' as statically typed of type 'NOT_SET'")
                declared_type = value
                value = None
            elif not descriptor.dynamic_typing:
                declared_type = Parameter.Type.from_parameter_value(value)
            if not ignore_override and full in self._parameter_overrides:
                override = self._parameter_overrides[full]
                if declared_type is not None and declared_type != Parameter.Type.NOT_SET and \
                        not descriptor.dynamic_typing and override.type_ != declared_type:
                    raise InvalidParameterTypeException(override, declared_type.name)
                value = override.value
            param = Parameter(full, value=value) if value is not None or declared_type is None \
                else Parameter(full, Parameter.Type.NOT_SET, None)
            if declared_type is not None and declared_type != Parameter.Type.NOT_SET:
                descriptor.type = int(declared_type)
            descriptor.name = full
            self._check_ranges(param, descriptor)
            self._parameters[full] = param
            self._descriptors[full] = (descriptor, declared_type)
            result.append(param)
        return result

    def _check_ranges(self, param, descriptor):
        if param.type_ == Parameter.Type.DOUBLE and descriptor.floating_point_range:
            rng = descriptor.floating_point_range[0]
            v = param.value
            if not (rng.from_value - abs(rng.from_value) * self.PARAM_REL_TOL <= v <=
                    rng.to_value + abs(rng.to_value) * self.PARAM_REL_TOL) and not math.isnan(v):
                raise InvalidParameterValueException(param.name, v, 'Parameter not within floating point range')
        if param.type_ == Parameter.Type.INTEGER and descriptor.integer_range:
            rng = descriptor.integer_range[0]
            if not rng.from_value <= param.value <= rng.to_value:
                raise InvalidParameterValueException(param.name, param.value, 'Parameter not within integer range')

    def undeclare_parameter(self, name):
        if name not in self._parameters:
            raise ParameterNotDeclaredException(name)
        del self._parameters[name]
        self._descriptors.pop(name, None)

    def has_parameter(self, name):
        return name in self._parameters

    def get_parameter(self, name):
        if name in self._parameters:
            param = self._parameters[name]
            desc = self._descriptors.get(name)
            if param.type_ == Parameter.Type.NOT_SET and desc and desc[1] not in (None, Parameter.Type.NOT_SET):
                raise ParameterUninitializedException(name)
            return param
        if self._allow_undeclared:
            return Parameter(name, Parameter.Type.NOT_SET, None)
        raise ParameterNotDeclaredException(name)

    def get_parameters(self, names):
        return [self.get_parameter(n) for n in names]

    def get_parameter_or(self, name, alternative_value=None):
        if alternative_value is None:
            alternative_value = Parameter(name, Parameter.Type.NOT_SET)
        return self._parameters.get(name, alternative_value)

    def get_parameters_by_prefix(self, prefix):
        prefix = prefix + PARAMETER_SEPARATOR_STRING if prefix else ''
        return {k[len(prefix):]: v for k, v in self._parameters.items() if k.startswith(prefix)}

    def describe_parameter(self, name):
        if name not in self._descriptors:
            raise ParameterNotDeclaredException(name)
        return self._descriptors[name][0]

    def describe_parameters(self, names):
        return [self.describe_parameter(n) for n in names]

    def add_on_set_parameters_callback(self, callback):
        self._set_param_callbacks.insert(0, callback)

    def remove_on_set_parameters_callback(self, callback):
        self._set_param_callbacks.remove(callback)

    def set_parameters(self, parameter_list):
        return [self._set_one(p) for p in parameter_list]

    def set_parameters_atomically(self, parameter_list):
        results = [self._set_one(p, dry=True) for p in parameter_list]
        from rcl_interfaces.msg import SetParametersResult
        if all(r.successful for r in results):
            for p in parameter_list:
                self._parameters[p.name] = p
            return SetParametersResult(successful=True)
        return SetParametersResult(successful=False, reason='; '.join(r.reason for r in results if r.reason))

    def _set_one(self, param, dry=False):
        from rcl_interfaces.msg import SetParametersResult
        if param.name not in self._parameters and not self._allow_undeclared:
            raise ParameterNotDeclaredException(param.name)
        desc = self._descriptors.get(param.name)
        if desc is not None:
            descriptor, declared_type = desc
            if descriptor.read_only:
                return SetParametersResult(successful=False, reason='Trying to set a read-only parameter: ' + param.name)
            if declared_type not in (None, Parameter.Type.NOT_SET) and not descriptor.dynamic_typing \
                    and param.type_ != declared_type:
                return SetParametersResult(successful=False,
                                           reason=f"Wrong parameter type, expected '{declared_type}' got '{param.type_}'")
        for cb in self._set_param_callbacks:
            res = cb([param])
            if not res.successful:
                return res
        if not dry:
            self._parameters[param.name] = param
        return SetParametersResult(successful=True)


class TopicEndpointInfo:
    def __init__(self, ep):
        fqn = getattr(ep.owner, 'sim_fqn', '/?')
        ns, _, name = fqn.rpartition('/')
        self.node_name = name
        self.node_namespace = ns or '/'
        self.topic_type = _rt.msggen.full_type_name(ep.msg_type)
        self.endpoint_type = 'PUBLISHER' if ep.kind == 'pub' else 'SUBSCRIPTION'
        q = ep.qos
        self.qos_profile = QoSProfile(
            depth=q.depth,
            reliability=ReliabilityPolicy.RELIABLE if q.reliable else ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL if q.transient_local else DurabilityPolicy.VOLATILE)


# ---------------------------------------------------------------------------
# module-level rclpy functions
# ---------------------------------------------------------------------------
def init(*, args=None, context=None, domain_id=None, signal_handler_options=None):
    rt = get_runtime()
    ctx = get_default_context() if context is None else context
    ctx.init(args, domain_id=domain_id)
    if args is not None:
        proc = rt.current_process()
        if proc is not None:
            parsed = RosArgs.parse(args)
            if parsed.node_name or parsed.remaps or parsed.params or parsed.params_files or parsed.namespace:
                base = proc.ros_args
                base.node_name = parsed.node_name or base.node_name
                base.namespace = parsed.namespace or base.namespace
                base.remaps = base.remaps + [r for r in parsed.remaps if r not in base.remaps]
                base.params_files = base.params_files + [p for p in parsed.params_files if p not in base.params_files]
                base.params = base.params + parsed.params


def _retire_default_context():
    """rclpy's get_default_context(shutting_down=True): the next init() gets a fresh one."""
    global _FALLBACK_CONTEXT
    ctx = get_default_context()
    rt = get_runtime(required=False)
    proc = rt.current_process() if rt else None
    if proc is not None:
        proc.context = None
    else:
        _FALLBACK_CONTEXT = Context()
    return ctx


def shutdown(*, context=None, uninstall_handlers=None):
    ctx = _retire_default_context() if context is None else context
    ctx.shutdown()


def try_shutdown(*, context=None, uninstall_handlers=None):
    ctx = _retire_default_context() if context is None else context
    ctx.try_shutdown()


def ok(*, context=None):
    ctx = get_default_context() if context is None else context
    return ctx.ok()


def create_node(node_name, *, context=None, cli_args=None, namespace=None, use_global_arguments=True,
                enable_rosout=True, start_parameter_services=True, parameter_overrides=None,
                allow_undeclared_parameters=False, automatically_declare_parameters_from_overrides=False):
    return Node(node_name, context=context, cli_args=cli_args, namespace=namespace,
                use_global_arguments=use_global_arguments, enable_rosout=enable_rosout,
                start_parameter_services=start_parameter_services, parameter_overrides=parameter_overrides,
                allow_undeclared_parameters=allow_undeclared_parameters,
                automatically_declare_parameters_from_overrides=automatically_declare_parameters_from_overrides)


def spin(node, executor=None):
    executor = get_global_executor() if executor is None else executor
    try:
        executor.add_node(node)
        executor.spin()
    finally:
        executor.remove_node(node)


def spin_once(node, *, executor=None, timeout_sec=None):
    executor = get_global_executor() if executor is None else executor
    try:
        executor.add_node(node)
        executor.spin_once(timeout_sec=timeout_sec)
    finally:
        executor.remove_node(node)


def spin_until_future_complete(node, future, executor=None, timeout_sec=None):
    executor = get_global_executor() if executor is None else executor
    try:
        executor.add_node(node)
        executor.spin_until_future_complete(future, timeout_sec)
    finally:
        executor.remove_node(node)


def remove_ros_args(args=None):
    args = list(sys.argv if args is None else args)
    if '--ros-args' not in args:
        return args
    out, inside = [], False
    for a in args:
        if a == '--ros-args':
            inside = True
            continue
        if inside and a == '--':
            inside = False
            continue
        if not inside:
            out.append(a)
    return out


def timeout_sec_to_nsec(timeout_sec):
    if timeout_sec is None or timeout_sec < 0:
        return -1
    return int(float(timeout_sec) * S_TO_NS)


_unused = itertools  # keep import for type hints in subclasses
_fnmatch = fnmatch
