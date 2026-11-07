from enum import IntEnum


class QoSPublisherEventType(IntEnum):
    RCL_PUBLISHER_OFFERED_DEADLINE_MISSED = 0
    RCL_PUBLISHER_LIVELINESS_LOST = 1
    RCL_PUBLISHER_OFFERED_INCOMPATIBLE_QOS = 2


class QoSSubscriptionEventType(IntEnum):
    RCL_SUBSCRIPTION_REQUESTED_DEADLINE_MISSED = 0
    RCL_SUBSCRIPTION_LIVELINESS_CHANGED = 1
    RCL_SUBSCRIPTION_REQUESTED_INCOMPATIBLE_QOS = 2
    RCL_SUBSCRIPTION_MESSAGE_LOST = 3


class UnsupportedEventTypeError(Exception):
    pass


class PublisherEventCallbacks:
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)


class SubscriptionEventCallbacks:
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)
