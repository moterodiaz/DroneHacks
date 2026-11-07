def _unsupported(*args, **kwargs):
    raise NotImplementedError("ROS 2 actions are not supported by the dronesim runtime")


ActionServer = ActionClient = _unsupported
