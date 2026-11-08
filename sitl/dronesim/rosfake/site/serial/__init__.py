"""pyserial stand-in: ports are virtual devices of the simulation (``dronesim.rosfake.devices``).

Reads block on *simulated* time: ``read(n)`` returns as soon as ``n`` bytes
have been transmitted by the device, or when ``timeout`` (simulated seconds)
passes -- the same contract as pyserial on a real tty.
"""
from dronesim.rosfake import devices as _devices
from dronesim.rosfake.runtime import NS, get_runtime

VERSION = '3.5'
PARITY_NONE, PARITY_EVEN, PARITY_ODD = 'N', 'E', 'O'
STOPBITS_ONE, STOPBITS_TWO = 1, 2
FIVEBITS, SIXBITS, SEVENBITS, EIGHTBITS = 5, 6, 7, 8


class SerialException(IOError):
    pass


class SerialTimeoutException(SerialException):
    pass


class PortNotOpenError(SerialException):
    def __init__(self):
        super().__init__('Attempting to use a port that is not open')


class Serial:
    def __init__(self, port=None, baudrate=9600, bytesize=EIGHTBITS, parity=PARITY_NONE,
                 stopbits=STOPBITS_ONE, timeout=None, xonxoff=False, rtscts=False,
                 write_timeout=None, dsrdtr=False, inter_byte_timeout=None, exclusive=None, **kwargs):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.write_timeout = write_timeout
        self.bytesize, self.parity, self.stopbits = bytesize, parity, stopbits
        self._buf = bytearray()
        self._dev = None
        self.is_open = False
        if port is not None:
            self.open()

    def open(self):
        dev = _devices.lookup(self.port)
        if dev is None:
            raise SerialException(f"[Errno 2] could not open port {self.port}: [Errno 2] No such file or "
                                  f"directory: '{self.port}'")
        if hasattr(dev, 'opened'):
            dev.opened(self.baudrate)
        self._dev = dev
        self.is_open = True

    def _pull(self):
        rt = get_runtime()
        self._buf.extend(self._dev.bytes_until(rt.now_ns))

    @property
    def in_waiting(self):
        if not self.is_open:
            raise PortNotOpenError()
        self._pull()
        return len(self._buf)

    def inWaiting(self):  # noqa: N802 - pyserial 2 API
        return self.in_waiting

    def read(self, size=1):
        if not self.is_open:
            raise PortNotOpenError()
        rt = get_runtime()
        deadline = None if self.timeout is None else rt.now_ns + int(self.timeout * NS)
        while True:
            self._pull()
            if len(self._buf) >= size:
                break
            if deadline is not None and rt.now_ns >= deadline:
                break
            if hasattr(self._dev, 'next_byte_ns'):
                nxt = self._dev.next_byte_ns(rt.now_ns)
            else:
                nxt = rt.now_ns + 1_000_000
            wake = nxt if deadline is None else min(nxt, deadline)
            wake = max(wake, rt.now_ns + 1000)
            if rt.in_sim_thread():
                rt.block('serial read', until_ns=wake)
            else:
                break
        out = bytes(self._buf[:size])
        del self._buf[:size]
        return out

    def read_all(self):
        self._pull()
        out = bytes(self._buf)
        self._buf.clear()
        return out

    def read_until(self, expected=b'\n', size=None):
        out = bytearray()
        while True:
            c = self.read(1)
            if not c:
                break
            out += c
            if out.endswith(expected) or (size is not None and len(out) >= size):
                break
        return bytes(out)

    readline = read_until

    def write(self, data):
        if not self.is_open:
            raise PortNotOpenError()
        if hasattr(self._dev, 'write'):
            self._dev.write(bytes(data))
        return len(data)

    def flush(self):
        pass

    def reset_input_buffer(self):
        self._pull()
        self._buf.clear()

    flushInput = reset_input_buffer  # noqa: N815 - pyserial 2 API

    def reset_output_buffer(self):
        pass

    def close(self):
        if self._dev is not None and hasattr(self._dev, 'closed'):
            self._dev.closed()
        self.is_open = False

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def fileno(self):
        raise SerialException('virtual port has no file descriptor')
