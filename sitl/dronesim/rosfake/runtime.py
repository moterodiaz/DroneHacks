"""Deterministic simulated ROS 2 runtime: clock, scheduler, processes, DDS graph.

Design
------
* **One clock.**  Simulation time is an integer number of nanoseconds since the
  start of the run.  ROS time (``node.get_clock().now()``, ``time.time()``) is
  ``epoch0 + sim time``, so stamps look like the Jetson's system clock.
* **One scheduler.**  Everything that happens -- physics steps, sensor samples,
  message deliveries, timers, thread wake-ups -- is an event in one heap, run in
  ``(time, sequence)`` order.  Callbacks take zero simulated time.
* **Processes and threads.**  ``ros2 launch`` starts every node as its own OS
  process.  Here each launched node gets a :class:`SimProcess` with its own rclpy
  context, running the package's real ``main()`` on a real Python thread.  Only
  one thread runs at a time: a thread runs until it reaches a *simulation
  primitive* (``rclpy.spin``, ``spin_once``, ``time.sleep``, a fake serial read,
  waiting on a future), which hands the baton back to the engine.  This keeps
  runs deterministic and lets unmodified code with its own ``while rclpy.ok()``
  loops or reader threads run against simulated time.
* **The DDS graph.**  Publishers and subscriptions match only with compatible QoS
  (a RELIABLE reader never hears a BEST_EFFORT writer; a TRANSIENT_LOCAL reader
  needs a TRANSIENT_LOCAL writer) and only after a discovery delay, exactly the
  traps the contracts warn about.  Each reader receives its own copy.
"""
from __future__ import annotations

import collections
import heapq
import itertools
import os
import random
import sys
import threading
import time as _time_module
import traceback

from . import msggen

NS = 1_000_000_000

# Real clock functions, saved before anything is patched.
REAL_TIME = _time_module.time
REAL_MONOTONIC = _time_module.monotonic
REAL_PERF = _time_module.perf_counter
REAL_SLEEP = _time_module.sleep

LEVELS = {10: 'DEBUG', 20: 'INFO', 30: 'WARN', 40: 'ERROR', 50: 'FATAL'}

_RUNTIME = None


def get_runtime(required=True):
    """The running SimRuntime.  With ``required`` and none running, an idle one is
    created so rclpy objects can be constructed outside a simulation (unit tests
    that build a node and call its callbacks directly, as the kit's tests do)."""
    if _RUNTIME is None and required:
        SimRuntime()
    return _RUNTIME


class SimKilled(BaseException):
    """Raised inside a simulated thread whose process is being killed."""


# ---------------------------------------------------------------------------
# events
# ---------------------------------------------------------------------------
class Event:
    __slots__ = ('t_ns', 'seq', 'fn', 'args', 'cancelled')

    def __init__(self, t_ns, seq, fn, args):
        self.t_ns, self.seq, self.fn, self.args = t_ns, seq, fn, args
        self.cancelled = False

    def __lt__(self, other):
        return (self.t_ns, self.seq) < (other.t_ns, other.seq)

    def cancel(self):
        self.cancelled = True


# ---------------------------------------------------------------------------
# processes / threads
# ---------------------------------------------------------------------------
class SimProcess:
    """One ``ros2 launch``-style process."""

    _ids = itertools.count(1)

    def __init__(self, runtime, name, argv=None, ros_args=None, env=None):
        self.runtime = runtime
        self.pid = 4000 + next(self._ids)
        self.name = name                    # e.g. "flight_interface-1"
        self.argv = list(argv or [name])    # what sys.argv would be
        self.ros_args = ros_args or RosArgs()
        self.env = dict(env or {})
        self.threads = []
        self.main_thread = None
        self.context = None                 # rclpy Context, set by rclpy.init()
        self.contexts = []
        self.nodes = []
        self.exit_code = None
        self.alive = True
        self.on_exit = []                   # callables(process)
        self.started_ns = runtime.now_ns

    def __repr__(self):
        return f'<SimProcess {self.name} pid={self.pid}>'


class RosArgs:
    """Parsed ``--ros-args`` of a process (what rcl would read from argv)."""

    def __init__(self, node_name=None, namespace=None, remaps=None, params_files=None,
                 params=None, log_level=None):
        self.node_name = node_name
        self.namespace = namespace
        self.remaps = list(remaps or [])          # [(from, to)]
        self.params_files = list(params_files or [])  # [path]
        self.params = list(params or [])          # [(name, value)] from -p
        self.log_level = log_level

    @classmethod
    def parse(cls, args):
        out = cls()
        if not args:
            return out
        args = list(args)
        if '--ros-args' not in args:
            return out
        i = args.index('--ros-args') + 1
        while i < len(args):
            a = args[i]
            if a == '--':
                break
            if a in ('-r', '--remap') and i + 1 < len(args):
                src, _, dst = args[i + 1].partition(':=')
                if src == '__node' or src == '__name':
                    out.node_name = dst
                elif src == '__ns':
                    out.namespace = dst
                else:
                    out.remaps.append((src, dst))
                i += 2
                continue
            if a in ('-p', '--param') and i + 1 < len(args):
                name, _, value = args[i + 1].partition(':=')
                import yaml
                out.params.append((name, yaml.safe_load(value) if value != '' else '')); i += 2
                continue
            if a == '--params-file' and i + 1 < len(args):
                out.params_files.append(args[i + 1]); i += 2
                continue
            if a == '--log-level' and i + 1 < len(args):
                out.log_level = args[i + 1]; i += 2
                continue
            i += 1
        return out


class SimThread:
    """A real thread scheduled cooperatively on simulated time."""

    _ids = itertools.count(1)

    def __init__(self, runtime, process, target, name=None, daemon=False, main=False):
        self.runtime = runtime
        self.process = process
        self.target = target
        self.name = name or f'simthread-{next(self._ids)}'
        self.daemon = daemon
        self.main = main
        self.state = 'new'          # new | running | blocked | done
        self.block_token = 0
        self.resume_exc = None
        self.wait_reason = ''
        self.real = threading.Thread(target=self._bootstrap, name=f'sim:{self.name}', daemon=True)
        self.result = None
        self.exception = None
        self.time_reads = 0

    def _bootstrap(self):
        rt = self.runtime
        with rt.baton:
            while rt.current is not self:
                rt.baton.wait()
        rt.tls.thread = self
        rt.tls.process = self.process
        try:
            if self.resume_exc is not None:
                exc, self.resume_exc = self.resume_exc, None
                raise exc
            self.result = self.target()
        except SimKilled:
            pass
        except SystemExit as exc:
            self.result = exc.code
        except BaseException as exc:  # noqa: BLE001 - reported to the process log
            self.exception = exc
            if not isinstance(exc, KeyboardInterrupt):
                rt.write_stream(self.process, 'stderr', ''.join(
                    traceback.format_exception(type(exc), exc, exc.__traceback__)))
        finally:
            with rt.baton:
                self.state = 'done'
                try:
                    rt.on_thread_done(self)   # only schedules events; baton still held
                finally:
                    rt.current = None
                    rt.baton.notify_all()


# ---------------------------------------------------------------------------
# QoS + graph
# ---------------------------------------------------------------------------
class QoSInfo:
    """Plain QoS description used by the graph (built from rclpy QoSProfile)."""

    __slots__ = ('reliable', 'transient_local', 'depth', 'keep_all')

    def __init__(self, reliable=True, transient_local=False, depth=10, keep_all=False):
        self.reliable, self.transient_local = reliable, transient_local
        self.depth, self.keep_all = depth, keep_all

    def describe(self):
        return (f"{'RELIABLE' if self.reliable else 'BEST_EFFORT'}/"
                f"{'TRANSIENT_LOCAL' if self.transient_local else 'VOLATILE'}/"
                f"{'KEEP_ALL' if self.keep_all else f'KEEP_LAST({self.depth})'}")


def qos_incompatibility(pub_qos, sub_qos):
    """DDS request/offered rule: None if compatible, else the policy name."""
    if sub_qos.reliable and not pub_qos.reliable:
        return 'RELIABILITY'
    if sub_qos.transient_local and not pub_qos.transient_local:
        return 'DURABILITY'
    return None


class Endpoint:
    _ids = itertools.count(1)

    def __init__(self, kind, owner, topic, msg_type, qos, callback=None):
        self.id = next(self._ids)
        self.kind = kind            # 'pub' | 'sub'
        self.owner = owner          # rclpy Node, or an InternalNode
        self.topic = topic
        self.msg_type = msg_type
        self.qos = qos
        self.callback = callback
        self.created_ns = None
        self.alive = True
        self.history = collections.deque(maxlen=max(1, qos.depth)) if kind == 'pub' else None
        self.matched = set()        # ids of matched counterparts
        self.publish_count = 0

    @property
    def node_name(self):
        return getattr(self.owner, 'sim_fqn', '?')


class TopicStats:
    __slots__ = ('count', 'stamps', 'last_msg', 'type_name')

    def __init__(self):
        self.count = 0
        self.stamps = collections.deque(maxlen=64)
        self.last_msg = None
        self.type_name = ''


class Graph:
    """Topics, endpoint matching and message delivery."""

    def __init__(self, runtime):
        self.rt = runtime
        self.pubs = collections.defaultdict(list)   # topic -> [Endpoint]
        self.subs = collections.defaultdict(list)
        self.services = {}                          # name -> ServiceEndpoint
        self.stats = collections.defaultdict(TopicStats)
        self.discovery_delay_s = 0.25
        self.default_latency_s = 0.0004
        self.latency_jitter_s = 0.0002
        self.topic_latency = []                     # [(prefix, latency_s, jitter_s, drop_p)]
        self.record_last = True
        self.taps = []                              # callables(topic, msg, t_ns) on every publish

    # -- endpoints --------------------------------------------------------
    def add(self, ep):
        ep.created_ns = self.rt.now_ns
        table = self.pubs if ep.kind == 'pub' else self.subs
        table[ep.topic].append(ep)
        others = self.subs[ep.topic] if ep.kind == 'pub' else self.pubs[ep.topic]
        for other in list(others):
            pub, sub = (ep, other) if ep.kind == 'pub' else (other, ep)
            self._schedule_match(pub, sub)
        return ep

    def remove(self, ep):
        ep.alive = False
        table = self.pubs if ep.kind == 'pub' else self.subs
        if ep in table[ep.topic]:
            table[ep.topic].remove(ep)

    def _schedule_match(self, pub, sub):
        if msggen.full_type_name(pub.msg_type) != msggen.full_type_name(sub.msg_type):
            self.rt.sim_log('WARN', 'dds',
                            f"topic '{pub.topic}': {pub.node_name} publishes "
                            f"{msggen.full_type_name(pub.msg_type)} but {sub.node_name} subscribes "
                            f"{msggen.full_type_name(sub.msg_type)} -- they will never communicate")
            return
        delay = self.discovery_delay_s if not (getattr(pub.owner, 'instant_discovery', False)
                                               and getattr(sub.owner, 'instant_discovery', False)) else 0.0
        self.rt.after(delay, self._match, pub, sub)

    def _match(self, pub, sub):
        if not (pub.alive and sub.alive):
            return
        bad = qos_incompatibility(pub.qos, sub.qos)
        if bad:
            for ep, other, verb in ((sub, pub, 'received from'), (pub, sub, 'sent to')):
                logger = getattr(ep.owner, 'sim_log_incompatible', None)
                if logger:
                    role = 'publisher' if ep is sub else 'subscription'
                    logger(f"New {role} discovered on topic '{pub.topic}', "
                           f"{'offering' if ep is sub else 'requesting'} incompatible QoS. "
                           f"No messages will be {verb} it. Last incompatible policy: {bad}")
            return
        pub.matched.add(sub.id)
        sub.matched.add(pub.id)
        if pub.qos.transient_local and sub.qos.transient_local:
            for msg in list(pub.history):
                self._deliver(pub, sub, msg)

    # -- data -------------------------------------------------------------
    def _link(self, topic):
        for prefix, lat, jit, drop in self.topic_latency:
            if topic.startswith(prefix):
                return lat, jit, drop
        return self.default_latency_s, self.latency_jitter_s, 0.0

    def publish(self, pub, msg):
        if not pub.alive:
            return
        pub.publish_count += 1
        if pub.qos.transient_local:
            pub.history.append(msggen.copy_message(msg))
        stats = self.stats[pub.topic]
        stats.count += 1
        stats.stamps.append(self.rt.now_ns)
        if self.record_last:
            stats.last_msg = msg
            stats.type_name = msggen.full_type_name(pub.msg_type)
        for tap in self.taps:
            tap(pub.topic, msg, self.rt.now_ns)
        for sub in self.subs.get(pub.topic, ()):
            if sub.id in pub.matched and sub.alive:
                self._deliver(pub, sub, msg)

    def _deliver(self, pub, sub, msg):
        lat, jit, drop = self._link(pub.topic)
        if drop and not (pub.qos.reliable and sub.qos.reliable) and self.rt.rng.random() < drop:
            return
        delay = lat + (self.rt.rng.random() * jit if jit else 0.0)
        copy = msggen.copy_message(msg)
        self.rt.after(delay, self._arrive, sub, copy)

    def _arrive(self, sub, msg):
        if not sub.alive:
            return
        owner = sub.owner
        deliver = getattr(owner, 'sim_deliver', None)
        if deliver is not None:
            deliver(sub, msg)
        elif sub.callback is not None:
            sub.callback(msg)

    # -- introspection -----------------------------------------------------
    def topic_names_and_types(self):
        names = {}
        for table in (self.pubs, self.subs):
            for topic, eps in table.items():
                for ep in eps:
                    names.setdefault(topic, set()).add(msggen.full_type_name(ep.msg_type))
        return sorted((t, sorted(v)) for t, v in names.items() if v)

    def rate_hz(self, topic, window_s=2.0):
        stats = self.stats.get(topic)
        if not stats or len(stats.stamps) < 2:
            return 0.0
        now = self.rt.now_ns
        recent = [s for s in stats.stamps if now - s <= window_s * NS]
        if len(recent) < 2:
            return 0.0
        return (len(recent) - 1) / ((recent[-1] - recent[0]) / NS)


# ---------------------------------------------------------------------------
# the runtime
# ---------------------------------------------------------------------------
class SimRuntime:
    """Owns simulated time, the event heap, processes and the DDS graph."""

    def __init__(self, epoch0=1_762_560_000.0, seed=1):
        global _RUNTIME
        if _RUNTIME is not None:
            raise RuntimeError('a SimRuntime already exists in this process')
        msggen.install()
        self.epoch0_ns = int(round(epoch0 * NS))
        self.now_ns = 0
        self._heap = []
        self._seq = itertools.count()
        self.rng = random.Random(seed)
        self.graph = Graph(self)
        self.processes = []
        self.baton = threading.Condition()
        self.current = None                 # SimThread holding the baton, None = engine
        self.tls = threading.local()
        self.log_sinks = []                 # callables(record dict)
        self.stopping = False
        self.watchdog_s = 30.0
        self.internal_nodes = []
        _RUNTIME = self

    def close(self):
        """Kill every process and release the module singleton."""
        global _RUNTIME
        self.stopping = True
        for proc in list(self.processes):
            self.kill_process(proc)
        _RUNTIME = None

    # -- time -------------------------------------------------------------
    @property
    def now_s(self):
        return self.now_ns / NS

    def ros_time_ns(self):
        return self.epoch0_ns + self.now_ns

    def at(self, t_ns, fn, *args):
        ev = Event(int(t_ns), next(self._seq), fn, args)
        heapq.heappush(self._heap, ev)
        return ev

    def after(self, dt_s, fn, *args):
        return self.at(self.now_ns + max(0, int(round(dt_s * NS))), fn, *args)

    def next_event_ns(self):
        while self._heap and self._heap[0].cancelled:
            heapq.heappop(self._heap)
        return self._heap[0].t_ns if self._heap else None

    def run_until(self, t_ns):
        """Run every event due at or before ``t_ns``; leave the clock at ``t_ns``."""
        heap = self._heap
        while heap and heap[0].t_ns <= t_ns:
            ev = heapq.heappop(heap)
            if ev.cancelled:
                continue
            if ev.t_ns > self.now_ns:
                self.now_ns = ev.t_ns
            ev.fn(*ev.args)
        if t_ns > self.now_ns:
            self.now_ns = t_ns

    # -- context helpers -----------------------------------------------------
    def current_thread(self):
        return getattr(self.tls, 'thread', None)

    def current_process(self):
        return getattr(self.tls, 'process', None)

    def in_sim_thread(self):
        th = self.current_thread()
        return th is not None and th.state == 'running' and threading.current_thread() is th.real

    # -- thread handoff -----------------------------------------------------
    def spawn(self, process, target, name=None, daemon=False, main=False):
        th = SimThread(self, process, target, name=name, daemon=daemon, main=main)
        process.threads.append(th)
        if main:
            process.main_thread = th
        th.real.start()
        self.after(0, self._start_thread, th)
        return th

    def _start_thread(self, th):
        if th.state == 'new':
            self.resume(th)

    def resume(self, th, exc=None):
        """Engine side: give the baton to ``th`` until it blocks or ends."""
        if th.state not in ('new', 'blocked'):
            return
        prev_thread = getattr(self.tls, 'thread', None)
        prev_process = getattr(self.tls, 'process', None)
        with self.baton:
            th.resume_exc = exc
            th.state = 'running'
            th.block_token += 1
            self.current = th
            self.baton.notify_all()
            started = REAL_MONOTONIC()
            warned = 0
            while self.current is th:
                self.baton.wait(timeout=1.0)
                waited = REAL_MONOTONIC() - started
                if self.current is th and waited > self.watchdog_s * (warned + 1):
                    warned += 1
                    self.sim_log('WARN', 'sim',
                                 f'{th.process.name}/{th.name} has run for {waited:.0f} s of real '
                                 f'time without yielding (long computation, or blocked outside a '
                                 f'simulation primitive); simulated time is frozen meanwhile')
        self.tls.thread = prev_thread
        self.tls.process = prev_process

    def block(self, reason='', until_ns=None):
        """Thread side: yield the baton; return when resumed.

        ``until_ns`` schedules a wake-up; other wake-ups come from whoever
        calls :meth:`wake` with the returned token.
        """
        th = self.current_thread()
        if th is None or threading.current_thread() is not th.real:
            raise RuntimeError('block() called outside a simulated thread')
        th.time_reads = 0
        token = th.block_token
        if until_ns is not None:
            self.at(until_ns, self._timed_wake, th, token)
        with self.baton:
            th.state = 'blocked'
            th.wait_reason = reason
            self.current = None
            self.baton.notify_all()
            while self.current is not th:
                self.baton.wait()
        self.tls.thread = th
        self.tls.process = th.process
        if th.resume_exc is not None:
            exc, th.resume_exc = th.resume_exc, None
            raise exc
        return token

    def _timed_wake(self, th, token):
        if th.state == 'blocked' and th.block_token == token:
            self.resume(th)

    def wake(self, th, exc=None):
        """Schedule ``th`` to resume now (from any context)."""
        token = th.block_token
        self.after(0, self._wake_if, th, token, exc)

    def _wake_if(self, th, token, exc):
        if th.state == 'blocked' and th.block_token == token:
            self.resume(th, exc)

    def sleep(self, seconds):
        """``time.sleep`` replacement."""
        seconds = max(0.0, float(seconds))
        if self.in_sim_thread():
            self.block('sleep', until_ns=self.now_ns + int(round(seconds * NS)))
            return
        node = getattr(self.tls, 'executing_node', None)
        if node is not None:
            # A callback sleeping blocks its executor: delay that node's work.
            node._sim_busy_ns += int(round(seconds * NS))
            return
        REAL_SLEEP(seconds)

    def note_time_read(self):
        """Break busy-wait loops that poll the clock without ever yielding."""
        th = getattr(self.tls, 'thread', None)
        if th is None or th.state != 'running' or threading.current_thread() is not th.real:
            return
        th.time_reads += 1
        if th.time_reads > 2000:
            self.block('clock poll', until_ns=self.now_ns + 1_000_000)

    def on_thread_done(self, th):
        proc = th.process
        if not proc.alive:
            return
        if th.main or all(t.state == 'done' or t.daemon for t in proc.threads):
            main = proc.main_thread
            if main is not None and main.state != 'done':
                return
            if any(t.state != 'done' and not t.daemon for t in proc.threads):
                return
            code = 0
            if main is not None:
                if main.exception is not None:
                    code = -2 if isinstance(main.exception, KeyboardInterrupt) else 1
                elif isinstance(main.result, int) and not isinstance(main.result, bool):
                    code = main.result
                elif main.result not in (None, 0, True, False):
                    code = 1
            self.after(0, self._process_exited, proc, code)

    def _process_exited(self, proc, code):
        if not proc.alive:
            return
        proc.alive = False
        proc.exit_code = code
        for th in proc.threads:
            if th.state == 'blocked':
                self.resume(th, SimKilled())
        for node in list(proc.nodes):
            try:
                node.sim_teardown()
            except Exception:  # noqa: BLE001
                pass
        for cb in list(proc.on_exit):
            cb(proc)

    # -- processes -----------------------------------------------------------
    def start_process(self, name, main, argv=None, ros_args=None, env=None):
        proc = SimProcess(self, name, argv=argv, ros_args=ros_args, env=env)
        self.processes.append(proc)
        self.spawn(proc, main, name='main', main=True)
        return proc

    def interrupt_process(self, proc):
        """SIGINT: KeyboardInterrupt in the main thread."""
        main = proc.main_thread
        if proc.alive and main is not None and main.state == 'blocked':
            self.resume(main, KeyboardInterrupt())

    def kill_process(self, proc):
        """SIGKILL: unwind every thread and drop the process from the graph."""
        if not proc.alive:
            return
        for th in list(proc.threads):
            if th.state in ('blocked', 'new'):
                self.resume(th, SimKilled())
        if proc.alive:
            self._process_exited(proc, -9)

    # -- logging --------------------------------------------------------------
    def emit(self, record):
        for sink in list(self.log_sinks):
            try:
                sink(record)
            except Exception:  # noqa: BLE001
                pass

    def node_log(self, process, logger_name, level, text):
        self.emit({'t': self.now_s, 'stamp_ns': self.ros_time_ns(),
                   'process': process.name if process else None, 'logger': logger_name,
                   'level': level, 'msg': text, 'source': 'ros'})

    def write_stream(self, process, stream, text):
        if not text:
            return
        self.emit({'t': self.now_s, 'stamp_ns': self.ros_time_ns(),
                   'process': process.name if process else None, 'logger': stream,
                   'level': 'ERROR' if stream == 'stderr' else 'INFO', 'msg': text.rstrip('\n'),
                   'source': stream})

    def sim_log(self, level, who, text):
        self.emit({'t': self.now_s, 'stamp_ns': self.ros_time_ns(), 'process': None,
                   'logger': who, 'level': level, 'msg': text, 'source': 'sim'})


# ---------------------------------------------------------------------------
# process-aware stdout / stderr and the patched clock
# ---------------------------------------------------------------------------
class _StreamRouter:
    def __init__(self, name, real):
        self.name, self.real = name, real
        self._buf = {}

    def write(self, text):
        rt = _RUNTIME
        proc = rt.current_process() if rt is not None else None
        if proc is None:
            return self.real.write(text)
        buf = self._buf.get(proc.pid, '') + text
        lines = buf.split('\n')
        self._buf[proc.pid] = lines.pop()
        for line in lines:
            rt.write_stream(proc, self.name, line)
        return len(text)

    def flush(self):
        rt = _RUNTIME
        proc = rt.current_process() if rt is not None else None
        if proc is not None and self._buf.get(proc.pid):
            rt.write_stream(proc, self.name, self._buf.pop(proc.pid))
        try:
            self.real.flush()
        except Exception:  # noqa: BLE001
            pass

    def isatty(self):
        return False

    def __getattr__(self, attr):
        return getattr(self.real, attr)


_PATCHED = False


def patch_process_globals():
    """Route the clock, sleep and stdio of node code through the runtime.

    Called once in the engine process.  Engine pacing keeps using the saved
    ``REAL_*`` functions.
    """
    global _PATCHED
    if _PATCHED:
        return
    _PATCHED = True

    def sim_time():
        rt = _RUNTIME
        if rt is None:
            return REAL_TIME()
        rt.note_time_read()
        return rt.ros_time_ns() / NS

    def sim_time_ns():
        rt = _RUNTIME
        if rt is None:
            return int(REAL_TIME() * NS)
        rt.note_time_read()
        return rt.ros_time_ns()

    def sim_monotonic():
        rt = _RUNTIME
        if rt is None:
            return REAL_MONOTONIC()
        rt.note_time_read()
        return 1000.0 + rt.now_ns / NS

    def sim_monotonic_ns():
        rt = _RUNTIME
        if rt is None:
            return int(REAL_MONOTONIC() * NS)
        rt.note_time_read()
        return 1000 * NS + rt.now_ns

    def sim_sleep(seconds):
        rt = _RUNTIME
        if rt is None:
            return REAL_SLEEP(seconds)
        return rt.sleep(seconds)

    _time_module.time = sim_time
    _time_module.time_ns = sim_time_ns
    _time_module.monotonic = sim_monotonic
    _time_module.monotonic_ns = sim_monotonic_ns
    _time_module.perf_counter = sim_monotonic
    _time_module.perf_counter_ns = sim_monotonic_ns
    _time_module.sleep = sim_sleep
    sys.stdout = _StreamRouter('stdout', sys.stdout)
    sys.stderr = _StreamRouter('stderr', sys.stderr)
    _patch_threading()


def _patch_threading():
    """Threads started by node code become simulated threads of that process."""
    real_start = threading.Thread.start

    def start(self):
        rt = _RUNTIME
        proc = rt.current_process() if rt is not None else None
        if proc is None or getattr(self, '_sim_wrapped', False) or self.name.startswith('sim:'):
            return real_start(self)
        self._sim_wrapped = True
        target = self.run
        th = rt.spawn(proc, target, name=self.name, daemon=self.daemon)
        self._sim_thread = th
        # Mark the Python Thread object as started so is_alive()/join() behave.
        self._started.set()
        self._sim_ident = th

    def join(self, timeout=None):
        th = getattr(self, '_sim_thread', None)
        if th is None:
            return real_join(self, timeout)
        rt = _RUNTIME
        deadline = None if timeout is None else rt.now_ns + int(timeout * NS)
        while th.state != 'done':
            if deadline is not None and rt.now_ns >= deadline:
                return
            rt.sleep(0.01)

    def is_alive(self):
        th = getattr(self, '_sim_thread', None)
        if th is None:
            return real_is_alive(self)
        return th.state != 'done'

    real_join = threading.Thread.join
    real_is_alive = threading.Thread.is_alive
    threading.Thread.start = start
    threading.Thread.join = join
    threading.Thread.is_alive = is_alive


def sim_env_path(*parts):
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), *parts)
