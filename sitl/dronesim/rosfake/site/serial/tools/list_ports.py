from dronesim.rosfake import devices as _devices


class ListPortInfo:
    def __init__(self, device):
        self.device = device
        self.name = device.rsplit('/', 1)[-1]
        self.description = 'dronesim virtual serial device'
        self.hwid = 'SIM'
        self.vid = self.pid = None
        self.serial_number = None

    def __iter__(self):
        return iter((self.device, self.description, self.hwid))


def comports(include_links=False):
    return [ListPortInfo(p) for p in _devices.ports()]
