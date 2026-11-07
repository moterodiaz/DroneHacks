"""Virtual serial devices for the fake ``pyserial`` (``rosfake/site/serial``).

A device is any object with ``bytes_until(t_ns) -> bytes`` (everything it has
transmitted up to simulated time ``t_ns``, consumed once), optional
``next_byte_ns(after_ns)`` (when it will next have data) and ``write(data)``.
Ports are registered by path (``/dev/ttyUSB0``...); opening an unregistered
port fails the way pyserial does on Linux.
"""
from __future__ import annotations

_PORTS = {}


def register(port, device):
    _PORTS[port] = device


def unregister(port):
    _PORTS.pop(port, None)


def clear():
    _PORTS.clear()


def lookup(port):
    return _PORTS.get(port)


def ports():
    return sorted(_PORTS)
