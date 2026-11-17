"""Minimal RTCM 3 helpers: message number and base-station position.

Only what the driver needs for routing and status reporting; correction
payloads are forwarded untouched.
"""
import math
from typing import Optional, Tuple

STATION_ARP_MESSAGES = (1005, 1006)

# WGS84
_A = 6378137.0
_F = 1 / 298.257223563
_E2 = _F * (2 - _F)


def message_type(frame: bytes) -> int:
    """Message number (first 12 bits of the payload) of a complete frame."""
    if len(frame) < 5:
        return -1
    return (frame[3] << 4) | (frame[4] >> 4)


def _bits(payload: bytes, start: int, length: int) -> int:
    value = 0
    for i in range(start, start + length):
        value = (value << 1) | ((payload[i // 8] >> (7 - i % 8)) & 1)
    return value


def _signed_bits(payload: bytes, start: int, length: int) -> int:
    value = _bits(payload, start, length)
    if value & (1 << (length - 1)):
        value -= 1 << length
    return value


def station_arp_ecef(frame: bytes) -> Optional[Tuple[int, float, float, float]]:
    """Decode (station_id, x, y, z) in metres from an RTCM 1005/1006 frame."""
    if message_type(frame) not in STATION_ARP_MESSAGES or len(frame) < 3 + 19 + 3:
        return None
    payload = frame[3:-3]
    station_id = _bits(payload, 12, 12)
    x = _signed_bits(payload, 34, 38) * 1e-4
    y = _signed_bits(payload, 74, 38) * 1e-4
    z = _signed_bits(payload, 114, 38) * 1e-4
    return station_id, x, y, z


def ecef_to_lla(x: float, y: float, z: float) -> Tuple[float, float, float]:
    """ECEF (m) to WGS84 latitude, longitude (deg) and ellipsoidal height (m)."""
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    lat = math.atan2(z, p * (1 - _E2))
    for _ in range(6):
        n = _A / math.sqrt(1 - _E2 * math.sin(lat) ** 2)
        h = p / math.cos(lat) - n
        lat = math.atan2(z, p * (1 - _E2 * n / (n + h)))
    n = _A / math.sqrt(1 - _E2 * math.sin(lat) ** 2)
    h = p / math.cos(lat) - n
    return math.degrees(lat), math.degrees(lon), h


def lla_to_ecef(lat_deg: float, lon_deg: float, h: float) -> Tuple[float, float, float]:
    lat, lon = math.radians(lat_deg), math.radians(lon_deg)
    n = _A / math.sqrt(1 - _E2 * math.sin(lat) ** 2)
    return ((n + h) * math.cos(lat) * math.cos(lon),
            (n + h) * math.cos(lat) * math.sin(lon),
            (n * (1 - _E2) + h) * math.sin(lat))
