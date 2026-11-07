from dronesim.rosfake.rclpy_impl import (  # noqa: F401
    LoggingSeverity, get_logger, set_logger_level, get_logger_effective_level, RcutilsLogger)


def initialize():
    pass


def clear_config():
    pass


def get_logging_directory():
    import tempfile
    return tempfile.gettempdir()
