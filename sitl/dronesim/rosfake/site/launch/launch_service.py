class LaunchService:
    """Present so `if __name__ == "__main__"` blocks import; the simulator runs launch itself."""

    def __init__(self, *, argv=None, noninteractive=False, debug=False):
        self.argv = argv

    def include_launch_description(self, launch_description):
        self._ld = launch_description

    def run(self, *, shutdown_when_idle=True):
        raise RuntimeError("LaunchService.run() is not available inside the simulator; use the SITL launcher")
