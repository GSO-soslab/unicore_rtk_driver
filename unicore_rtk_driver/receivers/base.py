"""Receiver abstraction.

A receiver class knows how to *configure* a specific GNSS module for the
requested operating mode.  Parsing of the output stream is shared (NMEA and
Unicore logs are handled by the processor regardless of receiver); RTCM
corrections are written to the module unmodified.

Adding a receiver = subclass `GnssReceiver`, implement `configuration_commands`
and register it in receivers/__init__.py.
"""
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Tuple


class OperatingMode(Enum):
    ROVER = 'rover'
    BASE = 'base'


@dataclass
class RtcmMessage:
    number: int
    period_s: float


@dataclass
class LogRequest:
    """A message the receiver should output periodically, e.g. GPGGA every 1 s."""
    name: str
    period_s: float


# A compact set suitable for low-bandwidth radio links: station position
# (1006), antenna descriptor (1033) and MSM4 observations for GPS, GLONASS,
# Galileo and BeiDou at 1 Hz.
DEFAULT_RTCM_MESSAGES = [
    RtcmMessage(1006, 10.0), RtcmMessage(1033, 10.0),
    RtcmMessage(1074, 1.0), RtcmMessage(1084, 1.0),
    RtcmMessage(1094, 1.0), RtcmMessage(1124, 1.0),
]


@dataclass
class RoverSettings:
    # Command that puts the module in rover mode; '' = do not send one.
    mode_command: str = ''


@dataclass
class BaseStationSettings:
    """Base-station configuration (same model as QGroundControl's RTK settings)."""
    # Sent verbatim instead of the command generated from the settings below
    # ('' = generate it).
    mode_command: str = ''
    # Survey-in: average the position for at least this long.
    survey_in_min_duration_s: int = 120
    # Survey-in accuracy target; only used by receivers that support it.
    survey_in_accuracy_m: float = 2.0
    # Fixed base: use a surveyed position instead of survey-in.
    use_fixed_position: bool = False
    fixed_latitude_deg: float = 0.0
    fixed_longitude_deg: float = 0.0
    # Height in the reference the receiver expects (UM98x: see README).
    fixed_altitude_m: float = 0.0
    rtcm_messages: List[RtcmMessage] = field(
        default_factory=lambda: list(DEFAULT_RTCM_MESSAGES))


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


def parse_rtcm_message_list(items: List[str]) -> List[RtcmMessage]:
    """['1006:10', '1074:1', ...] -> [RtcmMessage(1006, 10.0), ...]"""
    out = []
    for item in items:
        item = item.strip()
        if not item:
            continue
        number, _, period = item.partition(':')
        out.append(RtcmMessage(int(number), float(period or 1.0)))
    return out


class GnssReceiver(ABC):
    name = 'abstract'

    def __init__(self, port_name: str = ''):
        # Receiver-side port the host is connected to, if commands need it
        # (e.g. 'COM1').  Empty: the receiver's "current port".
        self.port_name = port_name

    @abstractmethod
    def configuration_commands(self, mode: OperatingMode, rover: RoverSettings,
                               base: BaseStationSettings, logs: List[LogRequest]) -> List[str]:
        """Commands (text lines, without line ending) that put the module in
        `mode` and make it output `logs` on the port the host is connected to."""

    def save_configuration_command(self) -> str:
        """Command that persists the configuration, or '' if not supported."""
        return ''

    def is_command_response(self, line: str) -> bool:
        """True if a line from the module is a reply to a command."""
        return False

    def describe_base(self, base: BaseStationSettings) -> Tuple[str, ...]:
        if base.mode_command:
            return (repr(base.mode_command),)
        if base.use_fixed_position:
            return ('fixed position', f'{base.fixed_latitude_deg:.9f}',
                    f'{base.fixed_longitude_deg:.9f}', f'{base.fixed_altitude_m:.3f} m')
        return ('survey-in', f'>= {base.survey_in_min_duration_s} s')
