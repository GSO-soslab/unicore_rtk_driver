"""Receiver abstraction.

A receiver class knows how to *configure* a specific GNSS module as a rover.
Parsing of the output stream is shared (NMEA and Unicore logs are handled by
the processor regardless of receiver); RTCM corrections are written to the
module unmodified.  Base stations are handled by the rtk_base package.

Adding a receiver = subclass `GnssReceiver`, implement `configuration_commands`
and register it in receivers/__init__.py.
"""
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List


@dataclass
class LogRequest:
    """A message the receiver should output periodically, e.g. GPGGA every 1 s."""
    name: str
    period_s: float


@dataclass
class RoverSettings:
    # Command that puts the module in rover mode; '' = do not send one.
    mode_command: str = ''


def parse_log_list(items: List[str]) -> List[LogRequest]:
    """['GPGGA:1', 'UNIHEADINGA:0.5', ...] -> [LogRequest('GPGGA', 1.0), ...]"""
    out = []
    for item in items:
        item = item.strip()
        if not item:
            continue
        name, _, period = item.partition(':')
        if not name.replace('_', '').isalnum():
            raise ValueError(f'invalid log name {name!r}')
        out.append(LogRequest(name.upper(), float(period or 1.0)))
    return out


class GnssReceiver(ABC):
    name = 'abstract'

    def __init__(self, port_name: str = ''):
        # Receiver-side port the host is connected to, if commands need it
        # (e.g. 'COM1').  Empty: the receiver's "current port".
        self.port_name = port_name

    @abstractmethod
    def configuration_commands(self, rover: RoverSettings, logs: List[LogRequest]) -> List[str]:
        """Commands (text lines, without line ending) that put the module in
        rover mode and make it output `logs` on the port the host is connected to."""

    def is_config_report(self, line: str) -> bool:
        """Tell whether a line from the module is part of its configuration report."""
        return False

    def save_configuration_command(self) -> str:
        """Command that persists the configuration, or '' if not supported."""
        return ''

    def is_command_response(self, line: str) -> bool:
        """True if a line from the module is a reply to a command."""
        return False
