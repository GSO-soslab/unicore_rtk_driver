from unicore_rtk_driver.receivers.base import (BaseStationSettings, GnssReceiver,
                                               LogRequest, OperatingMode, RoverSettings, RtcmMessage,
                                               parse_log_list, parse_rtcm_message_list)
from unicore_rtk_driver.receivers.generic import GenericReceiver
from unicore_rtk_driver.receivers.unicore import UnicoreReceiver

RECEIVERS = {cls.name: cls for cls in (UnicoreReceiver, GenericReceiver)}


def create_receiver(name: str, port_name: str = '') -> GnssReceiver:
    try:
        return RECEIVERS[name](port_name)
    except KeyError:
        raise ValueError(f'unknown receiver {name!r}, expected one of {sorted(RECEIVERS)}')


__all__ = ['BaseStationSettings', 'GnssReceiver', 'LogRequest', 'OperatingMode', 'RoverSettings',
           'RtcmMessage',
           'parse_log_list', 'parse_rtcm_message_list', 'GenericReceiver', 'UnicoreReceiver',
           'RECEIVERS', 'create_receiver']
