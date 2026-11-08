from dronesim.rosfake.rclpy_impl import (  # noqa: F401
    QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy, LivelinessPolicy,
    QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSLivelinessPolicy,
    QoSPresetProfiles, InvalidQoSProfileException, qos_profile_sensor_data, qos_profile_system_default,
    qos_profile_services_default, qos_profile_parameters, qos_profile_parameter_events,
    qos_profile_unknown)
from rclpy.duration import Duration  # noqa: F401,E402


def qos_check_compatible(publisher_qos, subscription_qos):
    from dronesim.rosfake.runtime import qos_incompatibility
    from enum import IntEnum

    class QoSCompatibility(IntEnum):
        OK = 0
        WARNING = 1
        ERROR = 2
    bad = qos_incompatibility(publisher_qos.to_info(), subscription_qos.to_info())
    return (QoSCompatibility.OK, "") if bad is None else (QoSCompatibility.ERROR, bad)
