from unicore_rtk_driver.receivers.base import (GnssReceiver, LogRequest, RoverSettings,
                                               parse_log_list)
from unicore_rtk_driver.receivers.generic import GenericReceiver
from unicore_rtk_driver.receivers.unicore import UnicoreReceiver

RECEIVERS = {cls.name: cls for cls in (UnicoreReceiver, GenericReceiver)}


def create_receiver(name: str, port_name: str = '') -> GnssReceiver:
    try:
        return RECEIVERS[name](port_name)
    except KeyError:
        raise ValueError(f'unknown receiver {name!r}, expected one of {sorted(RECEIVERS)}')


__all__ = ['GnssReceiver', 'LogRequest', 'RoverSettings', 'parse_log_list', 'GenericReceiver',
           'UnicoreReceiver', 'RECEIVERS', 'create_receiver']
