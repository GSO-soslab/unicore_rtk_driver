"""Checksums used on the receiver's serial stream.

* NMEA 0183:  8-bit XOR of every character between '$' and '*'.
* Unicore ASCII logs (#...):  CRC-32 (same polynomial and table as zlib) but
  with initial value 0 and no final XOR.  The table in the Unicore manual is
  the standard reflected 0xEDB88320 table, so zlib can compute it.
* RTCM 3:  CRC-24Q over the 3-byte header and the payload.
"""
import zlib


def nmea_checksum(body: str) -> int:
    """XOR checksum of an NMEA sentence body.

    Args:
        body: the text *between* '$' and '*' (no '$', no '*hh').
    """
    checksum = 0
    for char in body:
        checksum ^= ord(char)
    return checksum


def nmea_is_valid(sentence: str) -> bool:
    """True when `sentence` looks like '$...*hh' and hh matches the body."""
    sentence = sentence.strip()
    if not sentence.startswith('$'):
        return False
    body, sep, given = sentence[1:].rpartition('*')
    if not sep or len(given) != 2:
        return False
    return f'{nmea_checksum(body):02X}' == given.upper()


def unicore_crc32(body: str) -> int:
    """Unicore CRC-32 of an ASCII log body (text between '#' and '*')."""
    return zlib.crc32(body.encode('ascii', errors='replace'), 0xFFFFFFFF) ^ 0xFFFFFFFF


def unicore_is_valid(sentence: str) -> bool:
    """True when `sentence` looks like '#...*xxxxxxxx' and the CRC matches."""
    sentence = sentence.strip()
    if not sentence.startswith('#'):
        return False
    body, sep, given = sentence[1:].rpartition('*')
    if not sep or len(given) != 8:
        return False
    return f'{unicore_crc32(body):08x}' == given.lower()


def _make_crc24q_table():
    table = []
    for i in range(256):
        crc = i << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
        table.append(crc & 0xFFFFFF)
    return table


_CRC24Q_TABLE = _make_crc24q_table()


def crc24q(data: bytes) -> int:
    """CRC-24Q (Qualcomm), as used by RTCM 3."""
    crc = 0
    for byte in data:
        crc = ((crc << 8) & 0xFFFFFF) ^ _CRC24Q_TABLE[(crc >> 16) ^ byte]
    return crc
