"""Unicore UM98x (UM982 / UM980) receivers.

Command syntax follows the Unicore N4 "Commands and Logs" reference:
    MODE ROVER
    <LOG> [<port>] <period_s>                      enable a log, e.g. GPGGA 1,
                                                   UNIHEADINGA 0.5, AGRICA 1
    saveconfig
Replies look like:  $command,MODE ROVER,response: OK*xx
CONFIG (no arguments) reports the configuration, one line per setting:
                    $CONFIG,COM1,CONFIG COM1 460800*65
"""
from typing import List

from unicore_rtk_driver.receivers.base import GnssReceiver, LogRequest, RoverSettings

# What the driver consumes (same set the Alpha Rise / WAM-V receivers output):
#   GPGGA + GPGST -> ~/fix,  GPRMC -> ~/time_ref and ~/global_velocity,
#   GPHPR -> validated only,  UNIHEADINGA -> heading topics,
#   AGRICA -> ~/unicore_velocity
DEFAULT_LOGS = [
    LogRequest('GPGGA', 1.0), LogRequest('GPGST', 1.0), LogRequest('GPRMC', 1.0),
    LogRequest('GPHPR', 1.0), LogRequest('UNIHEADINGA', 0.5), LogRequest('AGRICA', 1.0),
]

# MODE ROVER has variants for the platform dynamics (e.g. MODE ROVER UAV);
# set rover.mode_command to choose one, or '' to leave the mode alone.
DEFAULT_ROVER_MODE_COMMAND = 'MODE ROVER'


class UnicoreReceiver(GnssReceiver):
    name = 'unicore'

    def configuration_commands(self, rover: RoverSettings, logs: List[LogRequest]) -> List[str]:
        port = f'{self.port_name} ' if self.port_name else ''
        cmds = []
        if rover.mode_command:
            cmds.append(rover.mode_command)
        for log in logs:
            cmds.append(f'{log.name} {port}{log.period_s:g}')
        return cmds

    def is_config_report(self, line: str) -> bool:
        return line.startswith('$CONFIG,')

    def save_configuration_command(self) -> str:
        return 'saveconfig'

    def is_command_response(self, line: str) -> bool:
        return line.startswith('$command,')
