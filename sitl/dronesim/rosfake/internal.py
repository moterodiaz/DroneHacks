"""Simulator-side graph participants (cuVSLAM, the uXRCE-DDS agent, sensors...).

They are not team code, so they do not run on threads: their subscriptions
call back directly from the engine when a message arrives, and they publish
from engine events.  They still go through the graph, so QoS matching,
discovery and per-reader copies apply to them like to everyone else.
"""
from __future__ import annotations

from .runtime import Endpoint, QoSInfo

RELIABLE = QoSInfo(reliable=True, transient_local=False, depth=10)
BEST_EFFORT = QoSInfo(reliable=False, transient_local=False, depth=10)
SENSOR = QoSInfo(reliable=False, transient_local=False, depth=5)
LATCHED = QoSInfo(reliable=True, transient_local=True, depth=1)


class InternalPublisher:
    __slots__ = ('ep', 'rt', 'msg_type')

    def __init__(self, rt, ep):
        self.rt, self.ep, self.msg_type = rt, ep, ep.msg_type

    def publish(self, msg):
        self.rt.graph.publish(self.ep, msg)

    @property
    def topic(self):
        return self.ep.topic

    def subscription_count(self):
        return sum(1 for s in self.rt.graph.subs.get(self.ep.topic, ()) if s.id in self.ep.matched and s.alive)


class InternalNode:
    """A named graph participant whose callbacks run inline in the engine."""

    def __init__(self, rt, name, namespace='/', instant_discovery=False):
        self.rt = rt
        ns = namespace.rstrip('/')
        self.sim_fqn = f'{ns}/{name}' if ns else f'/{name}'
        self.instant_discovery = instant_discovery
        self.endpoints = []
        rt.internal_nodes.append(self)

    def publisher(self, msg_type, topic, qos=RELIABLE):
        ep = Endpoint('pub', self, topic, msg_type, qos)
        self.rt.graph.add(ep)
        self.endpoints.append(ep)
        return InternalPublisher(self.rt, ep)

    def subscribe(self, msg_type, topic, callback, qos=RELIABLE):
        ep = Endpoint('sub', self, topic, msg_type, qos, callback=callback)
        self.rt.graph.add(ep)
        self.endpoints.append(ep)
        return ep

    def sim_deliver(self, ep, msg):
        ep.callback(msg)

    def sim_log_incompatible(self, text):
        self.rt.node_log(None, self.sim_fqn.lstrip('/'), 'WARN', text)

    def log(self, level, text):
        self.rt.node_log(None, self.sim_fqn.lstrip('/'), level, text)

    def destroy(self):
        for ep in self.endpoints:
            self.rt.graph.remove(ep)
        self.endpoints = []
        if self in self.rt.internal_nodes:
            self.rt.internal_nodes.remove(self)
