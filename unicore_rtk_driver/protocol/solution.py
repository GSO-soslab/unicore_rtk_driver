"""Solution names and summaries of the GGA, GST (accuracy) and HPR (heading) sentences.

No ROS imports. Used by the node's /diagnostics and by the rover tool's status line.
"""
from dataclasses import dataclass
import math
from typing import Optional

# GGA quality indicator -> solution name
GGA_SOLUTION_NAMES = {
    0: 'no fix', 1: 'single', 2: 'DGPS', 3: 'PPS', 4: 'RTK fixed', 5: 'RTK float',
    6: 'dead reckoning', 7: 'manual position', 8: 'simulation', 9: 'SBAS',
}

# HPR solution quality indicator -> name (UM98x manual); 4 is a valid dual antenna heading
HPR_SOLUTION_NAMES = {
    0: 'invalid', 1: 'single', 2: 'DGPS', 4: 'RTK fixed', 5: 'RTK float',
    6: 'dead reckoning', 7: 'manual', 8: 'extra wide-lane',
}


@dataclass
class GgaSummary:
    utc: str                            # hhmmss.ss as sent
    quality: int
    satellites: int
    hdop: Optional[float]
    correction_age_s: Optional[float]   # empty when no corrections are used
    station_id: str                     # base station of the corrections, empty without

    @property
    def solution(self) -> str:
        return GGA_SOLUTION_NAMES.get(self.quality, f'quality {self.quality}')


@dataclass
class GstSummary:
    utc: str
    lat_std_m: Optional[float]
    lon_std_m: Optional[float]
    alt_std_m: Optional[float]

    @property
    def horizontal_std_m(self) -> Optional[float]:
        if self.lat_std_m is None or self.lon_std_m is None:
            return None
        return math.hypot(self.lat_std_m, self.lon_std_m)


@dataclass
class HprSummary:
    utc: str
    heading_deg: Optional[float]
    pitch_deg: Optional[float]
    quality: int

    @property
    def solution(self) -> str:
        return HPR_SOLUTION_NAMES.get(self.quality, f'quality {self.quality}')


def _float(text: str) -> Optional[float]:
    try:
        return float(text)
    except ValueError:
        return None


def _fields(sentence: str, name: str, count: int):
    fields = sentence.strip().split('*', 1)[0].split(',')
    if not fields[0].endswith(name) or len(fields) < count:
        return None
    return fields


def parse_gga(sentence: str) -> Optional[GgaSummary]:
    """Summary of a GGA sentence, or None for any other sentence (checksum not checked)."""
    f = _fields(sentence, 'GGA', 15)
    if f is None:
        return None
    return GgaSummary(utc=f[1], quality=int(_float(f[6]) or 0),
                      satellites=int(_float(f[7]) or 0), hdop=_float(f[8]),
                      correction_age_s=_float(f[13]), station_id=f[14])


def parse_gst(sentence: str) -> Optional[GstSummary]:
    """Summary of a GST sentence (receiver error estimate), or None for any other sentence."""
    f = _fields(sentence, 'GST', 9)
    if f is None:
        return None
    return GstSummary(utc=f[1], lat_std_m=_float(f[6]), lon_std_m=_float(f[7]),
                      alt_std_m=_float(f[8]))


def parse_hpr(sentence: str) -> Optional[HprSummary]:
    """Summary of a HPR sentence, or None for any other sentence (checksum not checked)."""
    f = _fields(sentence, 'HPR', 6)
    if f is None:
        return None
    return HprSummary(utc=f[1], heading_deg=_float(f[2]), pitch_deg=_float(f[3]),
                      quality=int(_float(f[5]) or 0))
