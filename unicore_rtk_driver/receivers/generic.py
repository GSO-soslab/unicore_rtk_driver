"""Receivers configured outside this driver.

Use for modules that are set up with their own tools and only need their
NMEA parsed and/or their RTCM forwarded, e.g. a SparkFun RTK Surveyor
(u-blox ZED-F9P) configured as base or rover through its own menu or web
config.  No commands are generated; `receiver_commands` can still send raw
text lines if the module accepts them.
"""
from typing import List

from unicore_rtk_driver.receivers.base import (BaseStationSettings, GnssReceiver,
                                               LogRequest, OperatingMode, RoverSettings)


class GenericReceiver(GnssReceiver):
    name = 'generic'

    def configuration_commands(self, mode: OperatingMode, rover: RoverSettings,
                               base: BaseStationSettings, logs: List[LogRequest]) -> List[str]:
        return []
