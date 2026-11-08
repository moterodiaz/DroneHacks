"""``ros2 launch`` (launch 1.0 / launch_ros 0.19, i.e. Humble) on the simulated runtime.

Supports what Python launch files on the drone use: arguments, launch
configurations, substitutions, conditions, includes, opaque functions, groups,
timers, event handlers and ``launch_ros`` nodes.  Every ``Node`` becomes a
:class:`~dronesim.rosfake.runtime.SimProcess` that runs the package's real
console-script entry point with the ``--ros-args`` launch_ros would pass
(node name, namespace, remaps and a params file per parameter dict).

Parameter dictionaries are evaluated like launch_ros does it: strings and
substitutions are rendered and then parsed with ``yaml.safe_load``, so the
launch argument ``"3.0"`` reaches the node as the float ``3.0``.
"""
from __future__ import annotations

import importlib
import importlib.util
import itertools
import math
import os
import random
import string
import sys
from collections.abc import Mapping, Sequence

import yaml

from . import packages as _ws
from .runtime import RosArgs, get_runtime


class SubstitutionFailure(RuntimeError):
    pass


class InvalidLaunchFileError(Exception):
    pass


class InvalidConditionExpressionError(Exception):
    def __init__(self, expression, substitutions, valid):
        super().__init__(f"Unexpected condition expression '{expression}', expected one of: {valid}")


# ---------------------------------------------------------------------------
# context
# ---------------------------------------------------------------------------
class _Locals:
    def __init__(self, stack):
        object.__setattr__(self, '_stack', stack)

    def __getattr__(self, name):
        for frame in reversed(self._stack):
            if name in frame:
                return frame[name]
        raise AttributeError(f"context.locals does not contain attribute '{name}', it contains: "
                             f"[{', '.join(sorted({k for f in self._stack for k in f}))}]")


class LaunchContext:
    def __init__(self, *, argv=None, noninteractive=False, session=None):
        self.session = session
        self.launch_configurations = {}
        self._config_stack = []
        self._locals_stack = [{}]
        self.argv = argv or []
        self.noninteractive = noninteractive
        self.is_shutdown = False
        self.asyncio_loop = None

    @property
    def locals(self):
        return _Locals(self._locals_stack)

    @property
    def environment(self):
        return os.environ

    def extend_locals(self, new_locals):
        self._locals_stack.append(dict(new_locals))

    def _push_locals(self):
        self._locals_stack.append({})

    def _pop_locals(self):
        self._locals_stack.pop()

    def get_locals_as_dict(self):
        out = {}
        for frame in self._locals_stack:
            out.update(frame)
        return out

    def extend_globals(self, new_globals):
        self._locals_stack[0].update(new_globals)

    def _push_launch_configurations(self):
        self._config_stack.append(dict(self.launch_configurations))

    def _pop_launch_configurations(self):
        self.launch_configurations = self._config_stack.pop()

    def perform_substitution(self, substitution):
        return substitution.perform(self)

    def register_event_handler(self, handler, append=False):
        if self.session is not None:
            self.session.event_handlers.append(handler)

    def unregister_event_handler(self, handler):
        if self.session is not None and handler in self.session.event_handlers:
            self.session.event_handlers.remove(handler)

    def emit_event_sync(self, event):
        if self.session is not None:
            self.session.handle_event(event, self)

    def emit_event(self, event):
        self.emit_event_sync(event)


# ---------------------------------------------------------------------------
# substitutions
# ---------------------------------------------------------------------------
class Substitution:
    def perform(self, context):
        raise NotImplementedError

    def describe(self):
        return self.__class__.__name__


def normalize_to_list_of_substitutions(subs):
    if isinstance(subs, (str, Substitution)):
        subs = [subs]
    out = []
    for s in subs:
        if isinstance(s, str):
            out.append(TextSubstitution(text=s))
        elif isinstance(s, Substitution):
            out.append(s)
        elif isinstance(s, (int, float, bool)):
            out.append(TextSubstitution(text=str(s)))
        else:
            raise TypeError(f"Failed to normalize given substitutions '{subs}', expected Text or "
                            f"Substitution, got '{type(s)}'")
    return out


def perform_substitutions(context, subs):
    return ''.join(s.perform(context) for s in subs)


def _is_substitution_like(value):
    if isinstance(value, (str, Substitution)):
        return True
    if isinstance(value, Sequence) and not isinstance(value, (bytes, bytearray)):
        return all(isinstance(v, (str, Substitution)) for v in value) and \
            any(isinstance(v, Substitution) for v in value)
    return False


def _perform_any(context, value):
    if value is None:
        return None
    if isinstance(value, str):
        return value
    return perform_substitutions(context, normalize_to_list_of_substitutions(value))


class TextSubstitution(Substitution):
    def __init__(self, *, text):
        if not isinstance(text, str):
            raise TypeError(f"TextSubstitution expected Text object got '{type(text)}' instead.")
        self.text = text

    def perform(self, context):
        return self.text

    def describe(self):
        return repr(self.text)


class LaunchConfiguration(Substitution):
    def __init__(self, variable_name, *, default=None):
        self.variable_name = normalize_to_list_of_substitutions(variable_name)
        self.default = None if default is None else normalize_to_list_of_substitutions(default)

    def perform(self, context):
        name = perform_substitutions(context, self.variable_name)
        if name in context.launch_configurations:
            return context.launch_configurations[name]
        if self.default is not None:
            return perform_substitutions(context, self.default)
        raise SubstitutionFailure(f"launch configuration '{name}' does not exist")

    def describe(self):
        return f"LaunchConfig('{''.join(getattr(s, 'text', '?') for s in self.variable_name)}')"


class PathJoinSubstitution(Substitution):
    def __init__(self, substitutions):
        self.substitutions = [normalize_to_list_of_substitutions(s) for s in substitutions]

    def perform(self, context):
        return os.path.join(*[perform_substitutions(context, s) for s in self.substitutions])


class PythonExpression(Substitution):
    def __init__(self, expression, python_modules=None):
        self.expression = normalize_to_list_of_substitutions(expression)
        self.python_modules = python_modules or ['math']

    def perform(self, context):
        namespace = {}
        for mod in self.python_modules:
            namespace.update(importlib.import_module(mod).__dict__ if isinstance(mod, str) else {})
        return str(eval(perform_substitutions(context, self.expression), {}, namespace))  # noqa: S307


class EnvironmentVariable(Substitution):
    def __init__(self, name, *, default_value=None):
        self.name = normalize_to_list_of_substitutions(name)
        self.default_value = None if default_value is None else normalize_to_list_of_substitutions(default_value)

    def perform(self, context):
        name = perform_substitutions(context, self.name)
        value = os.environ.get(name)
        if value is None:
            if self.default_value is None:
                raise SubstitutionFailure(f"environment variable '{name}' does not exist")
            return perform_substitutions(context, self.default_value)
        return value


class ThisLaunchFileDir(Substitution):
    def perform(self, context):
        try:
            return context.locals.current_launch_file_directory
        except AttributeError as exc:
            raise SubstitutionFailure('ThisLaunchFileDir used outside of a launch file (in a script)') from exc


class ThisLaunchFile(Substitution):
    def perform(self, context):
        try:
            return context.locals.current_launch_file_path
        except AttributeError as exc:
            raise SubstitutionFailure('ThisLaunchFile used outside of a launch file (in a script)') from exc


class AnonName(Substitution):
    def __init__(self, name):
        self.name = normalize_to_list_of_substitutions(name)

    def perform(self, context):
        base = perform_substitutions(context, self.name)
        key = 'anon' + base
        store = context._locals_stack[0]
        if key not in store:
            store[key] = base + '_' + ''.join(random.choice(string.ascii_lowercase + string.digits) for _ in range(12))
        return store[key]


class LocalSubstitution(Substitution):
    def __init__(self, expression, description=None):
        self.expression = expression

    def perform(self, context):
        expr = self.expression
        attrs = context.get_locals_as_dict()
        try:
            return str(eval(expr, {}, attrs))  # noqa: S307
        except Exception as exc:  # noqa: BLE001
            raise SubstitutionFailure(f'LocalSubstitution {expr!r} failed: {exc}') from exc


class FindExecutable(Substitution):
    def __init__(self, *, name):
        self.name = normalize_to_list_of_substitutions(name)

    def perform(self, context):
        import shutil
        name = perform_substitutions(context, self.name)
        path = shutil.which(name)
        if path is None:
            raise SubstitutionFailure(f"executable '{name}' not found on the PATH")
        return path


class Command(Substitution):
    def __init__(self, command, *, on_stderr='fail'):
        self.command = command

    def perform(self, context):
        raise SubstitutionFailure('Command substitutions (running shell commands) are not supported '
                                  'in the simulator')


class LaunchLogDir(Substitution):
    def perform(self, context):
        import tempfile
        return tempfile.gettempdir()


def _truthy(text, context=None):
    v = str(text).lower()
    if v in ('true', '1'):
        return True
    if v in ('false', '0'):
        return False
    raise InvalidConditionExpressionError(text, [], ['true', 'false', '1', '0'])


class NotSubstitution(Substitution):
    def __init__(self, value):
        self.value = normalize_to_list_of_substitutions(value)

    def perform(self, context):
        return str(not _truthy(perform_substitutions(context, self.value))).lower()


class AndSubstitution(Substitution):
    def __init__(self, left, right):
        self.left, self.right = normalize_to_list_of_substitutions(left), normalize_to_list_of_substitutions(right)

    def perform(self, context):
        return str(_truthy(perform_substitutions(context, self.left)) and
                   _truthy(perform_substitutions(context, self.right))).lower()


class OrSubstitution(AndSubstitution):
    def perform(self, context):
        return str(_truthy(perform_substitutions(context, self.left)) or
                   _truthy(perform_substitutions(context, self.right))).lower()


# ---------------------------------------------------------------------------
# conditions
# ---------------------------------------------------------------------------
class Condition:
    def __init__(self, *, predicate=None):
        self._predicate = predicate

    def evaluate(self, context):
        return bool(self._predicate(context)) if self._predicate else True

    def describe(self):
        return self.__class__.__name__


def evaluate_condition_expression(context, expression):
    text = perform_substitutions(context, expression).lower()
    if text in ('true', '1'):
        return True
    if text in ('false', '0'):
        return False
    raise InvalidConditionExpressionError(text, expression, ['true', 'false', '1', '0'])


class IfCondition(Condition):
    def __init__(self, predicate_expression):
        self.expr = normalize_to_list_of_substitutions(predicate_expression)
        super().__init__(predicate=lambda ctx: evaluate_condition_expression(ctx, self.expr))


class UnlessCondition(Condition):
    def __init__(self, predicate_expression):
        self.expr = normalize_to_list_of_substitutions(predicate_expression)
        super().__init__(predicate=lambda ctx: not evaluate_condition_expression(ctx, self.expr))


class LaunchConfigurationEquals(Condition):
    def __init__(self, launch_configuration_name, expected_value):
        self.name = launch_configuration_name
        self.expected = None if expected_value is None else normalize_to_list_of_substitutions(expected_value)
        super().__init__(predicate=self._eval)

    def _eval(self, context):
        expected = None if self.expected is None else perform_substitutions(context, self.expected)
        return context.launch_configurations.get(self.name) == expected


class LaunchConfigurationNotEquals(LaunchConfigurationEquals):
    def _eval(self, context):
        return not super()._eval(context)


# ---------------------------------------------------------------------------
# entities and actions
# ---------------------------------------------------------------------------
class LaunchDescriptionEntity:
    def visit(self, context):
        return None

    def describe(self):
        return self.__class__.__name__

    def describe_sub_entities(self):
        return []

    def describe_conditional_sub_entities(self):
        return []

    def get_asyncio_future(self):
        return None


class Action(LaunchDescriptionEntity):
    def __init__(self, *, condition=None):
        self.__condition = condition

    @property
    def condition(self):
        return self.__condition

    def visit(self, context):
        if self.__condition is None or self.__condition.evaluate(context):
            return self.execute(context)
        return None

    def execute(self, context):
        return None


class LaunchDescription(LaunchDescriptionEntity):
    def __init__(self, initial_entities=None, *, deprecated_reason=None):
        self.entities = list(initial_entities or [])
        self.deprecated_reason = deprecated_reason

    def visit(self, context):
        return list(self.entities)

    def describe_sub_entities(self):
        return list(self.entities)

    def add_entity(self, entity):
        self.entities.append(entity)

    def add_action(self, action):
        self.entities.append(action)

    def get_launch_arguments(self, conditional_inclusion=False):
        found = []

        def walk(entities):
            for e in entities:
                if isinstance(e, DeclareLaunchArgument):
                    found.append(e)
                elif isinstance(e, GroupAction):
                    walk(e.actions)
        walk(self.entities)
        return found

    def get_launch_arguments_with_include_launch_description_actions(self, conditional_inclusion=False):
        return [(a, []) for a in self.get_launch_arguments(conditional_inclusion)]


class DeclareLaunchArgument(Action):
    def __init__(self, name, *, default_value=None, description=None, choices=None, **kwargs):
        super().__init__(**kwargs)
        self.name = name
        self.default_value = None if default_value is None else normalize_to_list_of_substitutions(default_value)
        self.description = description if description is not None else 'no description given'
        self.choices = choices

    def execute(self, context):
        if self.name not in context.launch_configurations:
            if self.default_value is None:
                given = ', '.join(f"'{k}'" for k in sorted(context.launch_configurations))
                raise RuntimeError(f"Included launch description missing required argument '{self.name}' "
                                   f"(description: '{self.description}'), given: [{given}]")
            context.launch_configurations[self.name] = perform_substitutions(context, self.default_value)
        value = context.launch_configurations[self.name]
        if self.choices is not None and value not in self.choices:
            raise RuntimeError(f"Argument '{self.name}' provided value '{value}' is not valid. "
                               f"Valid options are: {self.choices}")
        return None


class SetLaunchConfiguration(Action):
    def __init__(self, name, value, **kwargs):
        super().__init__(**kwargs)
        self.name = normalize_to_list_of_substitutions(name)
        self.value = normalize_to_list_of_substitutions(value)

    def execute(self, context):
        context.launch_configurations[perform_substitutions(context, self.name)] = \
            perform_substitutions(context, self.value)


class UnsetLaunchConfiguration(Action):
    def __init__(self, name, **kwargs):
        super().__init__(**kwargs)
        self.name = normalize_to_list_of_substitutions(name)

    def execute(self, context):
        context.launch_configurations.pop(perform_substitutions(context, self.name), None)


class PushLaunchConfigurations(Action):
    def execute(self, context):
        context._push_launch_configurations()


class PopLaunchConfigurations(Action):
    def execute(self, context):
        context._pop_launch_configurations()


class SetEnvironmentVariable(Action):
    def __init__(self, name, value, **kwargs):
        super().__init__(**kwargs)
        self.name = normalize_to_list_of_substitutions(name)
        self.value = normalize_to_list_of_substitutions(value)

    def execute(self, context):
        os.environ[perform_substitutions(context, self.name)] = perform_substitutions(context, self.value)


class UnsetEnvironmentVariable(Action):
    def __init__(self, name, **kwargs):
        super().__init__(**kwargs)
        self.name = normalize_to_list_of_substitutions(name)

    def execute(self, context):
        os.environ.pop(perform_substitutions(context, self.name), None)


class AppendEnvironmentVariable(Action):
    def __init__(self, name, value, *, prepend=False, separator=os.pathsep, **kwargs):
        super().__init__(**kwargs)
        self.name = normalize_to_list_of_substitutions(name)
        self.value = normalize_to_list_of_substitutions(value)
        self.prepend, self.separator = prepend, separator

    def execute(self, context):
        name = perform_substitutions(context, self.name)
        value = perform_substitutions(context, self.value)
        old = os.environ.get(name)
        os.environ[name] = value if not old else (value + self.separator + old if self.prepend
                                                  else old + self.separator + value)


class LogInfo(Action):
    def __init__(self, *, msg, **kwargs):
        super().__init__(**kwargs)
        self.msg = normalize_to_list_of_substitutions(msg)

    def execute(self, context):
        text = perform_substitutions(context, self.msg)
        rt = get_runtime()
        rt.node_log(None, 'launch.user', 'INFO', text)


class OpaqueFunction(Action):
    def __init__(self, *, function, args=None, kwargs=None, **left_over):
        super().__init__(**left_over)
        if not callable(function):
            raise TypeError("OpaqueFunction expected a callable for 'function'")
        self.function, self.args, self.kwargs = function, list(args or []), dict(kwargs or {})

    def execute(self, context):
        return self.function(context, *self.args, **self.kwargs)


class GroupAction(Action):
    def __init__(self, actions, *, scoped=True, forwarding=True, launch_configurations=None, **kwargs):
        super().__init__(**kwargs)
        self.actions = list(actions)
        self.scoped = scoped
        self.forwarding = forwarding
        self.group_configs = dict(launch_configurations or {})

    def execute(self, context):
        if self.scoped:
            context._push_launch_configurations()
            if not self.forwarding:
                context.launch_configurations = {}
        try:
            for k, v in self.group_configs.items():
                context.launch_configurations[_perform_any(context, k)] = _perform_any(context, v)
            visit_all(self.actions, context)
        finally:
            if self.scoped:
                context._pop_launch_configurations()
        return None

    def get_sub_entities(self):
        return self.actions


class TimerAction(Action):
    def __init__(self, *, period, actions, cancel_on_shutdown=True, **kwargs):
        super().__init__(**kwargs)
        self.period = period
        self.actions = list(actions)

    def execute(self, context):
        period = float(_perform_any(context, self.period) if not isinstance(self.period, (int, float))
                       else self.period)
        snapshot = dict(context.launch_configurations)
        locals_snapshot = context.get_locals_as_dict()

        def fire():
            if context.is_shutdown:
                return
            saved = context.launch_configurations
            context.launch_configurations = dict(snapshot)
            context.extend_locals(locals_snapshot)
            try:
                visit_all(self.actions, context)
            except Exception as exc:  # noqa: BLE001
                if context.session is not None:
                    context.session.fail(exc)
            finally:
                context._pop_locals()
                context.launch_configurations = saved
        get_runtime().after(period, fire)
        return None


class Shutdown(Action):
    def __init__(self, *, reason='reason not given', **kwargs):
        super().__init__(**kwargs)
        self.reason = reason

    def execute(self, context):
        if context.session is not None:
            context.session.shutdown(self.reason)


class ExecuteProcess(Action):
    def __init__(self, *, cmd, name=None, cwd=None, env=None, additional_env=None, shell=False,
                 output='log', emulate_tty=False, sigterm_timeout=5, sigkill_timeout=0, prefix=None,
                 respawn=False, respawn_delay=0.0, on_exit=None, log_cmd=False, **kwargs):
        super().__init__(**kwargs)
        self.cmd = [normalize_to_list_of_substitutions(c) for c in cmd]
        self.name = name

    def execute(self, context):
        cmd = [perform_substitutions(context, c) for c in self.cmd]
        if len(cmd) >= 4 and os.path.basename(cmd[0]) == 'ros2' and cmd[1] == 'run':
            node = _NodeSpec(package=cmd[2], executable=cmd[3], arguments=cmd[4:])
            context.session.start_node(node, context)
            return None
        get_runtime().node_log(None, 'launch', 'WARN',
                               f"ExecuteProcess {' '.join(cmd)!r} is not simulated (only 'ros2 run' "
                               'and launch_ros Nodes are); skipping it')
        return None


# -- event handling ---------------------------------------------------------------
class Event:
    name = 'launch.Event'


class ProcessStarted(Event):
    name = 'launch.events.process.ProcessStarted'

    def __init__(self, action, name, pid):
        self.action, self.process_name, self.pid = action, name, pid


class ProcessExited(Event):
    name = 'launch.events.process.ProcessExited'

    def __init__(self, action, name, pid, returncode):
        self.action, self.process_name, self.pid, self.returncode = action, name, pid, returncode


class ShutdownEvent(Event):
    name = 'launch.events.Shutdown'

    def __init__(self, *, reason='reason not given', due_to_sigint=False):
        self.reason, self.due_to_sigint = reason, due_to_sigint


class EventHandler:
    def __init__(self, *, matcher, entities=None, handle_once=False):
        self.matcher = matcher
        self.entities = entities
        self.handle_once = handle_once

    def matches(self, event):
        return self.matcher(event)

    def handle(self, event, context):
        context.extend_locals({'event': event})
        try:
            if callable(self.entities):
                return self.entities(event, context)
            return self.entities
        finally:
            context._pop_locals()


class _ProcessHandler(EventHandler):
    event_type = None

    def __init__(self, *, target_action=None, callback=None, **kwargs):
        self.target_action = target_action
        self._cb = callback
        super().__init__(matcher=self._match, entities=None, **kwargs)

    def _match(self, event):
        if not isinstance(event, self.event_type):
            return False
        if self.target_action is None:
            return True
        if callable(self.target_action) and not isinstance(self.target_action, Action):
            return bool(self.target_action(event.action))
        return event.action is self.target_action

    def handle(self, event, context):
        cb = self._cb
        if cb is None:
            return None
        if callable(cb) and not isinstance(cb, (list, LaunchDescriptionEntity)):
            return cb(event, context)
        return cb if isinstance(cb, list) else [cb]


class OnProcessExit(_ProcessHandler):
    event_type = ProcessExited

    def __init__(self, *, target_action=None, on_exit=None, **kwargs):
        super().__init__(target_action=target_action, callback=on_exit, **kwargs)


class OnProcessStart(_ProcessHandler):
    event_type = ProcessStarted

    def __init__(self, *, target_action=None, on_start=None, **kwargs):
        super().__init__(target_action=target_action, callback=on_start, **kwargs)


class OnShutdown(EventHandler):
    def __init__(self, *, on_shutdown, **kwargs):
        self._cb = on_shutdown
        super().__init__(matcher=lambda e: isinstance(e, ShutdownEvent), **kwargs)

    def handle(self, event, context):
        cb = self._cb
        return cb(event, context) if callable(cb) and not isinstance(cb, (list, LaunchDescriptionEntity)) \
            else (cb if isinstance(cb, list) else [cb])


class RegisterEventHandler(Action):
    def __init__(self, event_handler, **kwargs):
        super().__init__(**kwargs)
        self.event_handler = event_handler

    def execute(self, context):
        context.register_event_handler(self.event_handler)


class UnregisterEventHandler(Action):
    def __init__(self, event_handler, **kwargs):
        super().__init__(**kwargs)
        self.event_handler = event_handler

    def execute(self, context):
        context.unregister_event_handler(self.event_handler)


class EmitEvent(Action):
    def __init__(self, *, event, **kwargs):
        super().__init__(**kwargs)
        self.event = event

    def execute(self, context):
        context.emit_event_sync(self.event)


# -- includes ------------------------------------------------------------------------
class LaunchDescriptionSource:
    def __init__(self, launch_description=None, location='<script>', method='interpreted python launch file'):
        self._ld = launch_description
        self._location = location
        self.method = method

    @property
    def location(self):
        return self._location

    def try_get_launch_description_without_context(self):
        if self._ld is not None:
            return self._ld
        if isinstance(self._location, str):
            return self.get_launch_description(None)
        return None

    def get_launch_description(self, context):
        if self._ld is None:
            location = self._location if isinstance(self._location, str) else \
                perform_substitutions(context, normalize_to_list_of_substitutions(self._location))
            self._location = location
            self._ld = load_python_launch_file(location)
        return self._ld


class PythonLaunchDescriptionSource(LaunchDescriptionSource):
    def __init__(self, launch_file_path):
        super().__init__(None, launch_file_path, 'interpreted python launch file')


class AnyLaunchDescriptionSource(PythonLaunchDescriptionSource):
    def get_launch_description(self, context):
        location = self._location if isinstance(self._location, str) else \
            perform_substitutions(context, normalize_to_list_of_substitutions(self._location))
        if not location.endswith('.py'):
            raise InvalidLaunchFileError(f'{location}: only Python launch files are supported in the simulator')
        return super().get_launch_description(context)


class FrontendLaunchDescriptionSource(LaunchDescriptionSource):
    def __init__(self, launch_file_path, **kwargs):
        super().__init__(None, launch_file_path)

    def get_launch_description(self, context):
        raise InvalidLaunchFileError('XML/YAML launch files are not supported in the simulator')


_launch_counter = itertools.count()


def load_python_launch_file(path):
    if not os.path.isfile(path):
        raise InvalidLaunchFileError(f"launch file '{path}' does not exist")
    mod_name = f'_sim_launch_{next(_launch_counter)}_' + ''.join(
        c if c.isalnum() else '_' for c in os.path.basename(path))
    spec = importlib.util.spec_from_file_location(mod_name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    if not hasattr(module, 'generate_launch_description'):
        raise InvalidLaunchFileError(
            f"launch file at '{path}' does not contain the required function 'generate_launch_description()'")
    return module.generate_launch_description()


class IncludeLaunchDescription(Action):
    def __init__(self, launch_description_source, *, launch_arguments=None, **kwargs):
        super().__init__(**kwargs)
        if isinstance(launch_description_source, LaunchDescription):
            launch_description_source = LaunchDescriptionSource(launch_description_source)
        self.launch_description_source = launch_description_source
        self.launch_arguments = list(launch_arguments or [])

    def execute(self, context):
        ld = self.launch_description_source.get_launch_description(context)
        location = self.launch_description_source.location
        for key, value in self.launch_arguments:
            k = _perform_any(context, key)
            context.launch_configurations[k] = _perform_any(context, value)
        frame = {}
        if isinstance(location, str) and os.path.isfile(location):
            frame = {'current_launch_file_directory': os.path.dirname(os.path.abspath(location)),
                     'current_launch_file_path': os.path.abspath(location)}
        context.extend_locals(frame)
        try:
            visit_all(ld.entities, context)
        finally:
            context._pop_locals()
        return None


def visit_all(entities, context):
    for entity in entities:
        if entity is None:
            continue
        if not isinstance(entity, LaunchDescriptionEntity):
            raise TypeError(f'expected launch description entity, got {type(entity)!r}')
        sub = entity.visit(context)
        if sub:
            visit_all(sub, context)


# ---------------------------------------------------------------------------
# launch_ros
# ---------------------------------------------------------------------------
class ParameterValue:
    def __init__(self, value, *, value_type=None):
        self.value = value
        self.value_type = value_type

    def evaluate(self, context):
        if isinstance(self.value, (str, Substitution)) or _is_substitution_like(self.value):
            text = _perform_any(context, self.value)
            if self.value_type is str:
                return text
            loaded = yaml.safe_load(text)
            if self.value_type is not None and self.value_type is not type(loaded):
                if self.value_type is float and isinstance(loaded, int):
                    return float(loaded)
                raise TypeError(f'Expected type {self.value_type} got {type(loaded)} for {text!r}')
            return '' if loaded is None else loaded
        return self.value


class ParameterFile:
    def __init__(self, param_file, *, allow_substs=False):
        self.param_file = param_file
        self.allow_substs = allow_substs

    def evaluate(self, context):
        return _perform_any(context, self.param_file) if not isinstance(self.param_file, str) else self.param_file


class FindPackageShare(Substitution):
    def __init__(self, package):
        self.package = normalize_to_list_of_substitutions(package)

    def perform(self, context):
        return _ws.active().get(perform_substitutions(context, self.package)).share


class FindPackagePrefix(Substitution):
    def __init__(self, package):
        self.package = normalize_to_list_of_substitutions(package)

    def perform(self, context):
        return _ws.active().get(perform_substitutions(context, self.package)).prefix


class ExecutableInPackage(Substitution):
    def __init__(self, executable, package):
        self.executable = normalize_to_list_of_substitutions(executable)
        self.package = normalize_to_list_of_substitutions(package)

    def perform(self, context):
        pkg = _ws.active().get(perform_substitutions(context, self.package))
        exe = perform_substitutions(context, self.executable)
        if exe not in pkg.executables:
            raise SubstitutionFailure(f"executable '{exe}' not found on the libexec directory "
                                      f"'{os.path.join(pkg.prefix, 'lib', pkg.name)}' ")
        return os.path.join(pkg.prefix, 'lib', pkg.name, exe)


def _yaml_value(text):
    loaded = yaml.safe_load(text)
    if loaded is None:
        return ''
    if type(loaded) in (bool, int, float, str, bytes):
        return loaded
    if isinstance(loaded, list):
        if loaded and not all(type(v) is type(loaded[0]) for v in loaded):
            raise TypeError(f"Sequence of mixed types is not an allowed parameter value: {text!r}")
        return loaded
    raise TypeError(
        f'Allowed value types are bytes, bool, int, float, str, Sequence[bool], Sequence[int], '
        f'Sequence[float], Sequence[str]. Got {type(loaded)}. If the parameter is meant to be a string, '
        'try wrapping it in launch_ros.parameter_descriptions.ParameterValue(value, value_type=str)')


def evaluate_parameter_dict(context, params, prefix=''):
    out = {}
    for key, value in params.items():
        name = _perform_any(context, key) if not isinstance(key, str) else key
        full = f'{prefix}.{name}' if prefix else name
        if isinstance(value, Mapping):
            out.update(evaluate_parameter_dict(context, value, full))
        elif isinstance(value, ParameterValue):
            out[full] = value.evaluate(context)
        elif isinstance(value, (str, Substitution)) or _is_substitution_like(value):
            out[full] = _yaml_value(_perform_any(context, value))
        elif isinstance(value, (bool, int, float, bytes)):
            out[full] = value
        elif isinstance(value, Sequence):
            items = []
            for v in value:
                if isinstance(v, (str, Substitution)) or _is_substitution_like(v):
                    items.append(_yaml_value(_perform_any(context, v)))
                elif isinstance(v, ParameterValue):
                    items.append(v.evaluate(context))
                else:
                    items.append(v)
            if items and not all(type(v) is type(items[0]) for v in items):
                if all(isinstance(v, (int, float)) and not isinstance(v, bool) for v in items):
                    items = [float(v) for v in items]
                else:
                    raise TypeError(f"Expected a non-empty sequence, with items of uniform type. Got {value!r}")
            out[full] = list(items)
        else:
            raise TypeError(f'Unexpected type for parameter value {value!r}')
    return out


class _NodeSpec:
    """Already-evaluated node description (also used for ``ros2 run``)."""

    def __init__(self, package, executable, name=None, namespace=None, params_files=None,
                 remaps=None, arguments=None, ros_arguments=None, respawn=False, respawn_delay=0.0,
                 exec_name=None, action=None):
        self.package, self.executable = package, executable
        self.name, self.namespace = name, namespace
        self.params_files = list(params_files or [])
        self.remaps = list(remaps or [])
        self.arguments = list(arguments or [])
        self.ros_arguments = list(ros_arguments or [])
        self.respawn, self.respawn_delay = respawn, respawn_delay
        self.exec_name = exec_name
        self.action = action


class Node(Action):
    UNSPECIFIED_NODE_NAME = '<node_name_unspecified>'
    UNSPECIFIED_NODE_NAMESPACE = '<node_namespace_unspecified>'

    def __init__(self, *, executable, package=None, name=None, namespace=None, exec_name=None,
                 parameters=None, remappings=None, ros_arguments=None, arguments=None, output='log',
                 emulate_tty=False, respawn=False, respawn_delay=0.0, prefix=None, additional_env=None,
                 sigterm_timeout=None, sigkill_timeout=None, on_exit=None, cwd=None, env=None,
                 node_name=None, node_namespace=None, **kwargs):
        super().__init__(**kwargs)
        self.package = package
        self.node_executable = executable
        self.node_name = name if name is not None else node_name
        self.node_namespace = namespace if namespace is not None else node_namespace
        self.exec_name = exec_name
        self.parameters = parameters
        self.remappings = remappings
        self.ros_arguments = ros_arguments
        self.arguments = arguments
        self.respawn = respawn
        self.respawn_delay = respawn_delay
        self.additional_env = additional_env

    def execute(self, context):
        session = context.session
        package = _perform_any(context, self.package)
        executable = _perform_any(context, self.node_executable)
        name = _perform_any(context, self.node_name) if self.node_name is not None else None
        ns = _perform_any(context, self.node_namespace) if self.node_namespace is not None else None
        base_ns = context.launch_configurations.get('ros_namespace')
        if base_ns:
            if ns is None:
                ns = base_ns
            elif not ns.startswith('/'):
                ns = base_ns.rstrip('/') + '/' + ns
        params_files = []
        node_key = None
        if name is not None:
            ns_text = ns if ns else '/'
            node_key = (ns_text.rstrip('/') + '/' + name) if ns_text != '/' else '/' + name
        for gp in context.launch_configurations.get('global_params', []) or []:
            if isinstance(gp, tuple):
                params_files.append({'/**': {'ros__parameters': {gp[0]: gp[1]}}})
            else:
                params_files.append(gp)
        for entry in self.parameters or []:
            if isinstance(entry, Mapping):
                values = evaluate_parameter_dict(context, entry)
                params_files.append(session.write_params_file({node_key or '/**': {'ros__parameters': values}}))
            elif isinstance(entry, ParameterFile):
                params_files.append(_check_params_file(entry.evaluate(context)))
            else:
                params_files.append(_check_params_file(_perform_any(context, entry)))
        remaps = [(_perform_any(context, a), _perform_any(context, b)) for a, b in (self.remappings or [])]
        remaps = list(context.launch_configurations.get('global_remaps', []) or []) + remaps
        arguments = [_perform_any(context, a) for a in (self.arguments or [])]
        ros_arguments = [_perform_any(context, a) for a in (self.ros_arguments or [])]
        if self.additional_env:
            for k, v in self.additional_env.items():
                os.environ[_perform_any(context, k)] = _perform_any(context, v)
        spec = _NodeSpec(package, executable, name=name, namespace=ns, params_files=params_files,
                         remaps=remaps, arguments=arguments, ros_arguments=ros_arguments,
                         respawn=bool(_truthy(_perform_any(context, self.respawn)) if not isinstance(self.respawn, bool) else self.respawn),
                         respawn_delay=float(self.respawn_delay or 0.0), exec_name=self.exec_name, action=self)
        session.start_node(spec, context)
        return None


def _check_params_file(path):
    if not os.path.isfile(path):
        raise RuntimeError(f"Parameter file path is not a file: {path}")
    return path


class LifecycleNode(Node):
    pass


class PushRosNamespace(Action):
    def __init__(self, namespace, **kwargs):
        super().__init__(**kwargs)
        self.namespace = normalize_to_list_of_substitutions(namespace)

    def execute(self, context):
        ns = perform_substitutions(context, self.namespace)
        prev = context.launch_configurations.get('ros_namespace', '')
        if not ns.startswith('/'):
            ns = (prev.rstrip('/') + '/' + ns) if prev else '/' + ns
        context.launch_configurations['ros_namespace'] = ns


class SetParameter(Action):
    def __init__(self, name, value, **kwargs):
        super().__init__(**kwargs)
        self.name, self.value = name, value

    def execute(self, context):
        name = _perform_any(context, self.name)
        value = self.value.evaluate(context) if isinstance(self.value, ParameterValue) else (
            _yaml_value(_perform_any(context, self.value)) if _is_substitution_like(self.value) else self.value)
        gp = list(context.launch_configurations.get('global_params', []) or [])
        gp.append((name, value))
        context.launch_configurations['global_params'] = gp


class SetParametersFromFile(Action):
    def __init__(self, filename, **kwargs):
        super().__init__(**kwargs)
        self.filename = filename

    def execute(self, context):
        gp = list(context.launch_configurations.get('global_params', []) or [])
        gp.append(_check_params_file(_perform_any(context, self.filename)))
        context.launch_configurations['global_params'] = gp


class SetRemap(Action):
    def __init__(self, src, dst, **kwargs):
        super().__init__(**kwargs)
        self.src, self.dst = src, dst

    def execute(self, context):
        gr = list(context.launch_configurations.get('global_remaps', []) or [])
        gr.append((_perform_any(context, self.src), _perform_any(context, self.dst)))
        context.launch_configurations['global_remaps'] = gr


class ComposableNodeContainer(Action):
    def __init__(self, **kwargs):
        super().__init__(condition=kwargs.get('condition'))

    def execute(self, context):
        raise RuntimeError('ComposableNodeContainer (C++ components) is not supported by the simulator')


LoadComposableNodes = ComposableNodeContainer


class ComposableNode:
    def __init__(self, **kwargs):
        self.kwargs = kwargs


# ---------------------------------------------------------------------------
# the launch session
# ---------------------------------------------------------------------------
class LaunchSession:
    """One ``ros2 launch`` invocation inside the simulation."""

    def __init__(self, workspace, logger=None):
        self.ws = workspace
        self.rt = get_runtime()
        self.counter = itertools.count(1)
        self.processes = []          # (spec, SimProcess)
        self.event_handlers = []
        self.failed = None
        self.is_shutdown = False
        self.context = None
        self._params_dir = os.path.join(workspace.root, 'launch_params')
        os.makedirs(self._params_dir, exist_ok=True)
        self._param_counter = itertools.count(1)
        #: (package, executable) -> [callable() -> str]: run inside that process before main()
        self.pre_main = {}

    def log(self, level, text, who='launch'):
        self.rt.node_log(None, who, level, text)

    def write_params_file(self, data):
        path = os.path.join(self._params_dir, f'launch_params_{next(self._param_counter):03d}.yaml')
        with open(path, 'w', encoding='utf-8') as fh:
            yaml.safe_dump(data, fh, default_flow_style=False)
        return path

    def run(self, package, launch_file, arguments=None):
        """``ros2 launch <package> <launch_file> k:=v ...``"""
        try:
            pkg = self.ws.get(package)
        except _ws.PackageNotFoundError as exc:
            self.log('ERROR', f"Package '{package}' not found: {exc}")
            self.failed = exc
            return False
        path = None
        if os.path.isfile(os.path.join(pkg.share, launch_file)):
            path = os.path.join(pkg.share, launch_file)
        else:
            for dirpath, _dirs, files in os.walk(pkg.share):
                if launch_file in files:
                    path = os.path.join(dirpath, launch_file)
                    break
        if path is None:
            self.log('ERROR', f"file '{launch_file}' was not found in the share directory of package "
                              f"'{package}' which is at '{pkg.share}'")
            self.failed = FileNotFoundError(launch_file)
            return False
        return self.run_file(path, arguments)

    def run_file(self, path, arguments=None):
        context = LaunchContext(session=self)
        self.context = context
        for key, value in (arguments or {}).items():
            context.launch_configurations[str(key)] = str(value)
        self.log('INFO', f'All log files can be found below {self.ws.root}/log')
        self.log('INFO', 'Default logging verbosity is set to INFO')
        try:
            include = IncludeLaunchDescription(PythonLaunchDescriptionSource(path))
            visit_all([include], context)
        except Exception as exc:  # noqa: BLE001 - this is what ros2 launch prints
            self.fail(exc)
            return False
        return True

    def fail(self, exc):
        text = str(exc) if str(exc) else exc.__class__.__name__
        self.log('ERROR', f'Caught exception in launch (see debug for traceback): {text}')
        import traceback
        self.log('DEBUG', ''.join(traceback.format_exception(type(exc), exc, exc.__traceback__)))
        self.failed = exc
        self.shutdown('launch failed')

    def shutdown(self, reason='shutdown requested'):
        if self.is_shutdown:
            return
        self.is_shutdown = True
        if self.context is not None:
            self.context.is_shutdown = True
        for _spec, proc in self.processes:
            if proc.alive:
                self.rt.interrupt_process(proc)

    def handle_event(self, event, context):
        if isinstance(event, ShutdownEvent):
            self.shutdown(event.reason)
        for handler in list(self.event_handlers):
            if handler.matches(event):
                result = handler.handle(event, context)
                if handler.handle_once and handler in self.event_handlers:
                    self.event_handlers.remove(handler)
                if result:
                    try:
                        visit_all(result if isinstance(result, list) else [result], context)
                    except Exception as exc:  # noqa: BLE001
                        self.fail(exc)

    def start_node(self, spec, context):
        if self.is_shutdown:
            return None
        pkg = self.ws.get(spec.package)
        target = pkg.executables.get(spec.executable)
        libexec = os.path.join(pkg.prefix, 'lib', pkg.name)
        if target is None:
            raise SubstitutionFailure(f"executable '{spec.executable}' not found on the libexec directory '{libexec}' ")
        n = next(self.counter)
        proc_name = f'{spec.exec_name or spec.executable}-{n}'
        argv = [os.path.join(libexec, spec.executable)] + list(spec.arguments)
        ros = ['--ros-args']
        if spec.name:
            ros += ['-r', f'__node:={spec.name}']
        if spec.namespace:
            ros += ['-r', f"__ns:={spec.namespace if spec.namespace.startswith('/') else '/' + spec.namespace}"]
        for pf in spec.params_files:
            if isinstance(pf, dict):
                pf = self.write_params_file(pf)
            ros += ['--params-file', pf]
        for a, b in spec.remaps:
            ros += ['-r', f'{a}:={b}']
        argv += ros + list(spec.ros_arguments)
        ros_args = RosArgs.parse(argv)

        if callable(target):
            entry = target
        else:
            module_name, func_name = target

            def entry(module_name=module_name, func_name=func_name):
                module = importlib.import_module(module_name)
                return getattr(module, func_name)()

        hooks = list(self.pre_main.get((spec.package, spec.executable), []))

        def main():
            proc = self.rt.current_process()
            if proc is not None:
                proc.argv = argv
            for hook in hooks:
                note = hook()
                if note:
                    self.rt.node_log(proc, 'dronesim', 'WARN', f'SIMULATOR INJECTION: {note}')
            return entry()

        proc = self.rt.start_process(proc_name, main, argv=argv, ros_args=ros_args)
        proc.package = spec.package
        proc.executable = spec.executable
        self.processes.append((spec, proc))
        self.log('INFO', f'process started with pid [{proc.pid}]', who=proc_name)
        proc.on_exit.append(lambda p, spec=spec: self._on_exit(spec, p))
        self.handle_event(ProcessStarted(spec.action, proc_name, proc.pid), context)
        return proc

    def _on_exit(self, spec, proc):
        code = proc.exit_code
        if code == 0:
            self.log('INFO', f'process has finished cleanly [pid {proc.pid}]', who=proc.name)
        else:
            self.log('ERROR', f"process has died [pid {proc.pid}, exit code {code}, cmd "
                              f"'{' '.join(proc.argv)}'].", who=proc.name)
        if self.context is not None:
            self.handle_event(ProcessExited(spec.action, proc.name, proc.pid, code), self.context)
        if spec.respawn and not self.is_shutdown:
            self.rt.after(max(0.0, spec.respawn_delay), lambda: self.start_node(spec, self.context))


# ---------------------------------------------------------------------------
# process-aware sys.argv
# ---------------------------------------------------------------------------
class ArgvProxy(list):
    """``sys.argv`` that shows each simulated process its own argv."""

    def _target(self):
        rt = get_runtime(required=False)
        proc = rt.current_process() if rt is not None else None
        return proc.argv if proc is not None else list.__iter__(self)

    def __getitem__(self, item):
        return list(self._target())[item]

    def __len__(self):
        return len(list(self._target()))

    def __iter__(self):
        return iter(list(self._target()))

    def __contains__(self, item):
        return item in list(self._target())

    def __repr__(self):
        return repr(list(self._target()))

    def copy(self):
        return list(self._target())


def install_argv_proxy():
    if not isinstance(sys.argv, ArgvProxy):
        sys.argv = ArgvProxy(sys.argv)


_math = math
