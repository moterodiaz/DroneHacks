"""tf2 / tf2_ros on the simulated graph: a frame tree with interpolation.

``lookup_transform(target, source, time)`` returns the transform that maps
points expressed in ``source`` into ``target`` (tf2's convention).  A zero
time means "latest available"; any other time is interpolated and, like tf2,
raises ``ExtrapolationException`` outside the buffered interval of a dynamic
link.
"""
from __future__ import annotations

import collections
import math


class TransformException(Exception):
    pass


class LookupException(TransformException):
    pass


class ConnectivityException(TransformException):
    pass


class ExtrapolationException(TransformException):
    pass


class InvalidArgumentException(TransformException):
    pass


class TimeoutException(TransformException):
    pass


# -- quaternion helpers (x, y, z, w) -------------------------------------------------
def q_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def q_conj(q):
    return (-q[0], -q[1], -q[2], q[3])


def q_rotate(q, v):
    qv = (v[0], v[1], v[2], 0.0)
    r = q_mul(q_mul(q, qv), q_conj(q))
    return (r[0], r[1], r[2])


def q_slerp(a, b, t):
    dot = sum(x * y for x, y in zip(a, b))
    if dot < 0:
        b = tuple(-x for x in b)
        dot = -dot
    if dot > 0.9995:
        out = tuple(x + t * (y - x) for x, y in zip(a, b))
    else:
        th = math.acos(max(-1.0, min(1.0, dot)))
        s = math.sin(th)
        wa, wb = math.sin((1 - t) * th) / s, math.sin(t * th) / s
        out = tuple(wa * x + wb * y for x, y in zip(a, b))
    n = math.sqrt(sum(x * x for x in out)) or 1.0
    return tuple(x / n for x in out)


class _Tf:
    __slots__ = ('t', 'q')

    def __init__(self, t, q):
        self.t, self.q = t, q

    def compose(self, other):
        """self * other"""
        rt = q_rotate(self.q, other.t)
        return _Tf((self.t[0] + rt[0], self.t[1] + rt[1], self.t[2] + rt[2]), q_mul(self.q, other.q))

    def inverse(self):
        qi = q_conj(self.q)
        ti = q_rotate(qi, self.t)
        return _Tf((-ti[0], -ti[1], -ti[2]), qi)


def _stamp_ns(time):
    if time is None:
        return 0
    if hasattr(time, 'nanoseconds'):
        return int(time.nanoseconds)
    if hasattr(time, 'sec'):
        return int(time.sec) * 1_000_000_000 + int(time.nanosec)
    return int(time)


class Buffer:
    def __init__(self, cache_time=None, node=None):
        self.cache_ns = int(getattr(cache_time, 'nanoseconds', 10_000_000_000) or 10_000_000_000)
        self._dynamic = collections.defaultdict(lambda: collections.deque(maxlen=2000))
        self._static = {}
        self._parent = {}

    # -- input ------------------------------------------------------------------
    def _store(self, transform, static):
        child = transform.child_frame_id.lstrip('/')
        parent = transform.header.frame_id.lstrip('/')
        if not child or not parent:
            raise InvalidArgumentException('frame ids must not be empty')
        tr, ro = transform.transform.translation, transform.transform.rotation
        tf = _Tf((tr.x, tr.y, tr.z), (ro.x, ro.y, ro.z, ro.w))
        self._parent[child] = parent
        if static:
            self._static[child] = (parent, tf)
        else:
            dq = self._dynamic[child]
            stamp = _stamp_ns(transform.header.stamp)
            dq.append((stamp, parent, tf))
            while dq and stamp - dq[0][0] > self.cache_ns:
                dq.popleft()

    def set_transform(self, transform, authority):
        self._store(transform, False)
        return True

    def set_transform_static(self, transform, authority):
        self._store(transform, True)
        return True

    # -- queries -------------------------------------------------------------------
    def _link(self, child, stamp):
        if child in self._static:
            return self._static[child]
        dq = self._dynamic.get(child)
        if not dq:
            raise LookupException(f'"{child}" passed to lookupTransform argument does not exist.')
        if stamp == 0:
            return dq[-1][1], dq[-1][2]
        if stamp > dq[-1][0]:
            raise ExtrapolationException(
                f'Lookup would require extrapolation into the future.  Requested time {stamp / 1e9:.6f} '
                f'but the latest data is at time {dq[-1][0] / 1e9:.6f}, when looking up transform from '
                f'frame [{child}] to frame [{dq[-1][1]}]')
        if stamp < dq[0][0]:
            raise ExtrapolationException(
                f'Lookup would require extrapolation into the past.  Requested time {stamp / 1e9:.6f} '
                f'but the earliest data is at time {dq[0][0] / 1e9:.6f}, when looking up transform from '
                f'frame [{child}] to frame [{dq[0][1]}]')
        prev = dq[0]
        for item in dq:
            if item[0] >= stamp:
                if item[0] == prev[0]:
                    return item[1], item[2]
                a = (stamp - prev[0]) / (item[0] - prev[0])
                t = tuple(p + a * (q - p) for p, q in zip(prev[2].t, item[2].t))
                return item[1], _Tf(t, q_slerp(prev[2].q, item[2].q, a))
            prev = item
        return dq[-1][1], dq[-1][2]

    def _chain(self, frame, stamp):
        """frame -> root as (root, T_root_frame)."""
        tf = _Tf((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
        seen = set()
        while frame in self._parent:
            if frame in seen:
                raise LookupException(f'TF loop detected at {frame}')
            seen.add(frame)
            parent, link = self._link(frame, stamp)
            tf = link.compose(tf)
            frame = parent
        return frame, tf

    def _known(self, frame):
        return frame in self._parent or frame in set(self._parent.values())

    def _latest_common(self, a, b):
        stamps = []
        for frame in (a, b):
            f = frame
            while f in self._parent:
                if f not in self._static:
                    dq = self._dynamic.get(f)
                    if dq:
                        stamps.append(dq[-1][0])
                f = self._parent[f]
        return min(stamps) if stamps else 0

    def lookup_transform(self, target_frame, source_frame, time, timeout=None):
        from builtin_interfaces.msg import Time as TimeMsg
        from geometry_msgs.msg import TransformStamped
        target, source = target_frame.lstrip('/'), source_frame.lstrip('/')
        for f, arg in ((target, 'target_frame'), (source, 'source_frame')):
            if not self._known(f):
                raise LookupException(f'"{f}" passed to lookupTransform argument {arg} does not exist. ')
        stamp = _stamp_ns(time)
        used = stamp if stamp else self._latest_common(target, source)
        r1, t_root_source = self._chain(source, stamp)
        r2, t_root_target = self._chain(target, stamp)
        if r1 != r2:
            raise ConnectivityException(
                f'Could not find a connection between \'{target}\' and \'{source}\' because they are not part '
                'of the same tree.Tf has two or more unconnected trees.')
        tf = t_root_target.inverse().compose(t_root_source)
        out = TransformStamped()
        out.header.frame_id = target
        out.header.stamp = TimeMsg(sec=int(used // 1_000_000_000), nanosec=int(used % 1_000_000_000))
        out.child_frame_id = source
        out.transform.translation.x, out.transform.translation.y, out.transform.translation.z = (float(v) for v in tf.t)
        q = tf.q
        (out.transform.rotation.x, out.transform.rotation.y, out.transform.rotation.z,
         out.transform.rotation.w) = (float(v) for v in q)
        return out

    def lookup_transform_full(self, target_frame, target_time, source_frame, source_time, fixed_frame, timeout=None):
        return self.lookup_transform(target_frame, source_frame, source_time, timeout)

    def can_transform(self, target_frame, source_frame, time, timeout=None, return_debug_tuple=False):
        try:
            self.lookup_transform(target_frame, source_frame, time)
            ok, err = True, ''
        except TransformException as exc:
            ok, err = False, str(exc)
        return (ok, err) if return_debug_tuple else ok

    def can_transform_full(self, target_frame, target_time, source_frame, source_time, fixed_frame,
                           timeout=None, return_debug_tuple=False):
        return self.can_transform(target_frame, source_frame, source_time, timeout, return_debug_tuple)

    def wait_for_transform_async(self, target_frame, source_frame, time):
        from rclpy.task import Future
        fut = Future()
        if self.can_transform(target_frame, source_frame, time):
            fut.set_result(True)
        return fut

    def lookup_transform_async(self, target_frame, source_frame, time):
        from rclpy.task import Future
        fut = Future()
        try:
            fut.set_result(self.lookup_transform(target_frame, source_frame, time))
        except TransformException as exc:
            fut.set_exception(exc)
        return fut

    def all_frames_as_yaml(self):
        lines = []
        for child, parent in sorted(self._parent.items()):
            lines.append(f'{child}:\n  parent: \'{parent}\'\n  static: {child in self._static}')
        return '\n'.join(lines)

    all_frames_as_string = all_frames_as_yaml

    def clear(self):
        self._dynamic.clear()
        self._static.clear()
        self._parent.clear()

    def transform(self, object_stamped, target_frame, timeout=None, new_type=None):
        tf = self.lookup_transform(target_frame, object_stamped.header.frame_id, object_stamped.header.stamp)
        from tf2_geometry_msgs import do_transform
        return do_transform(object_stamped, tf)


class TransformListener:
    def __init__(self, buffer, node, *, spin_thread=False, qos=None, static_qos=None):
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
        from tf2_msgs.msg import TFMessage
        self.buffer = buffer
        self.node = node
        qos = qos or QoSProfile(depth=100)
        static_qos = static_qos or QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                              history=HistoryPolicy.KEEP_LAST)
        self.tf_sub = node.create_subscription(TFMessage, '/tf', self.callback, qos)
        self.tf_static_sub = node.create_subscription(TFMessage, '/tf_static', self.static_callback, static_qos)

    def callback(self, data):
        for transform in data.transforms:
            self.buffer.set_transform(transform, 'default_authority')

    def static_callback(self, data):
        for transform in data.transforms:
            self.buffer.set_transform_static(transform, 'default_authority')

    def unregister(self):
        self.node.destroy_subscription(self.tf_sub)
        self.node.destroy_subscription(self.tf_static_sub)

    def __del__(self):
        pass


class TransformBroadcaster:
    def __init__(self, node, qos=None):
        from tf2_msgs.msg import TFMessage
        self.pub_tf = node.create_publisher(TFMessage, '/tf', qos or 100)

    def sendTransform(self, transform):  # noqa: N802 - tf2_ros API
        from tf2_msgs.msg import TFMessage
        if not isinstance(transform, list):
            transform = [transform]
        self.pub_tf.publish(TFMessage(transforms=transform))


class StaticTransformBroadcaster:
    def __init__(self, node, qos=None):
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
        from tf2_msgs.msg import TFMessage
        qos = qos or QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                history=HistoryPolicy.KEEP_LAST)
        self.pub_tf = node.create_publisher(TFMessage, '/tf_static', qos)
        self.net_message = TFMessage()
        self._child_frame_ids = {}

    def sendTransform(self, transform):  # noqa: N802 - tf2_ros API
        if not isinstance(transform, list):
            transform = [transform]
        for t in transform:
            self._child_frame_ids[t.child_frame_id] = t
        self.net_message.transforms = list(self._child_frame_ids.values())
        self.pub_tf.publish(self.net_message)
