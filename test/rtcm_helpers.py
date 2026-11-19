"""Build RTCM 3 frames for tests."""
import math

from unicore_rtk_driver.protocol.checksums import crc24q


def frame(payload: bytes) -> bytes:
    head = bytes([0xD3, (len(payload) >> 8) & 0x03, len(payload) & 0xFF]) + payload
    return head + crc24q(head).to_bytes(3, 'big')


def pack_bits(fields):
    """[(value, nbits), ...] -> bytes, MSB first, zero padded."""
    bits = ''
    for value, n in fields:
        bits += format(value & ((1 << n) - 1), f'0{n}b')
    bits += '0' * (-len(bits) % 8)
    return int(bits, 2).to_bytes(len(bits) // 8, 'big')


def message(number: int, body_len: int = 10) -> bytes:
    """Dummy frame of a given message number."""
    return frame(pack_bits([(number, 12)]) + bytes(range(body_len)))


def station_1005(station_id, x, y, z) -> bytes:
    return frame(pack_bits([
        (1005, 12), (station_id, 12), (0, 6), (1, 1), (0, 1), (0, 1), (0, 1),
        (round(x * 1e4), 38), (0, 1), (0, 1),
        (round(y * 1e4), 38), (0, 2),
        (round(z * 1e4), 38),
    ]))


def lla_to_ecef(lat_deg, lon_deg, h):
    """WGS84 latitude, longitude (deg) and ellipsoidal height (m) to ECEF (m)."""
    a, f = 6378137.0, 1 / 298.257223563
    e2 = f * (2 - f)
    lat, lon = math.radians(lat_deg), math.radians(lon_deg)
    n = a / math.sqrt(1 - e2 * math.sin(lat) ** 2)
    return ((n + h) * math.cos(lat) * math.cos(lon),
            (n + h) * math.cos(lat) * math.sin(lon),
            (n * (1 - e2) + h) * math.sin(lat))
