"""Receiver abstraction.

A receiver class knows how to *configure* a specific GNSS module as a rover.
Parsing of the output stream is shared (NMEA and Unicore logs are handled by
the processor regardless of receiver); RTCM corrections are written to the
module unmodified.  Base stations are handled by the rtk_base package.

Adding a receiver = subclass `GnssReceiver`, implement `configuration_commands`
(and `settings_commands` if the module supports the rover settings) and register
it in receivers/__init__.py.
"""
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List


@dataclass
class LogRequest:
    """A message the receiver should output periodically, e.g. GPGGA every 1 s."""
    name: str
    period_s: float


# Limits of the correction timeouts (Unicore CONFIG RTK/DGPS TIMEOUT)
MAX_TIMEOUT_S = 1800


@dataclass
class RoverSettings:
    """Rover settings.  For the numeric ones, 0 leaves the receiver's own value."""

    # Command that puts the module in rover mode; '' = do not send one.
    mode_command: str = ''
    # Oldest corrections the receiver still uses for an RTK / DGPS solution
    rtk_timeout_s: int = 0
    dgps_timeout_s: int = 0
    # Distance between the two heading antennas and its tolerance
    heading_baseline_cm: int = 0
    heading_baseline_tolerance_cm: int = 0

    def __post_init__(self):
        for name in ('rtk_timeout_s', 'dgps_timeout_s'):
            value = getattr(self, name)
            if not 0 <= value <= MAX_TIMEOUT_S:
                raise ValueError(f'{name} must be 0 (receiver default) or 1 to '
                                 f'{MAX_TIMEOUT_S}, got {value}')
        if self.heading_baseline_cm < 0 or self.heading_baseline_tolerance_cm < 0:
            raise ValueError('heading baseline and tolerance must not be negative')
        if self.heading_baseline_tolerance_cm and not self.heading_baseline_cm:
            raise ValueError('heading_baseline_tolerance_cm needs heading_baseline_cm')

    @property
    def has_receiver_settings(self) -> bool:
        return any((self.rtk_timeout_s, self.dgps_timeout_s, self.heading_baseline_cm))


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

    def settings_commands(self, rover: RoverSettings) -> List[str]:
        """Commands for the numeric rover settings that are set (non-zero)."""
        return []

    def config_query_command(self) -> str:
        """Command that makes the module report its configuration, or ''."""
        return ''

    def is_config_report(self, line: str) -> bool:
        """Tell whether a line from the module is part of its configuration report."""
        return False

    def save_configuration_command(self) -> str:
        """Command that persists the configuration, or '' if not supported."""
        return ''

    def is_command_response(self, line: str) -> bool:
        """True if a line from the module is a reply to a command."""
        return False
