from enum import IntEnum


class SignalHandlerOptions(IntEnum):
    NO = 0
    SIGINT = 1
    SIGTERM = 2
    ALL = 3


def install_signal_handlers(options=SignalHandlerOptions.ALL):
    pass


def uninstall_signal_handlers():
    pass


def get_current_signal_handlers_options():
    return SignalHandlerOptions.ALL
