from dronesim.rosfake.launch_impl import OnProcessExit, OnProcessStart, OnShutdown, EventHandler  # noqa: F401


class OnProcessIO(EventHandler):
    def __init__(self, *, target_action=None, on_stdin=None, on_stdout=None, on_stderr=None, **kwargs):
        super().__init__(matcher=lambda e: False)


class OnExecutionComplete(EventHandler):
    def __init__(self, *, target_action=None, on_completion=None, **kwargs):
        super().__init__(matcher=lambda e: False)
