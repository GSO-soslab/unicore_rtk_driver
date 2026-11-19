"""Receiver type for modules configured with their own tools.

For example a SparkFun RTK Surveyor (u-blox ZED-F9P) set up as a rover
through its own menu.  The driver parses its NMEA and forwards its RTCM.  This
receiver type generates no configuration commands; `receiver_commands` are
sent as given.
"""
from typing import List

from unicore_rtk_driver.receivers.base import GnssReceiver, LogRequest, RoverSettings


class GenericReceiver(GnssReceiver):
    name = 'generic'

    def configuration_commands(self, rover: RoverSettings, logs: List[LogRequest]) -> List[str]:
        return []
