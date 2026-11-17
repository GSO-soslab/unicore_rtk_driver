"""Split the receiver's byte stream into frames.

The serial port carries a mix of:
  * NMEA sentences            '$....*hh\\r\\n'           (text)
  * Unicore ASCII logs        '#....*xxxxxxxx\\r\\n'     (text)
  * RTCM 3 frames             0xD3 + len + payload + CRC (binary, base mode)

`StreamFramer.feed()` takes the bytes read from the port and returns the
complete frames they contain, keeping a partial frame until the next call and
skipping bytes between frames.  RTCM frames are returned only when their
CRC-24Q matches, which also confirms that a 0xD3 byte starts a frame.  Text
frames are returned as received; the processor validates their checksums.
"""
from dataclasses import dataclass
from enum import Enum

from unicore_rtk_driver.protocol.checksums import crc24q

RTCM_PREAMBLE = 0xD3
_TEXT_STARTS = (ord('$'), ord('#'))


class FrameType(Enum):
    NMEA = 1       # starts with '$'
    UNICORE = 2    # starts with '#'
    RTCM = 3       # binary RTCM 3


@dataclass
class Frame:
    type: FrameType
    text: str = ''        # NMEA / Unicore line, without '\r\n'
    data: bytes = b''     # RTCM frame, complete (preamble .. CRC)


class StreamFramer:

    def __init__(self, max_line_length=1024, parse_rtcm=True, parse_text=True):
        self._buf = bytearray()
        self._max_line = max_line_length
        self._parse_rtcm = parse_rtcm
        self._parse_text = parse_text
        self.dropped_bytes = 0

    def feed(self, data: bytes):
        self._buf.extend(data)
        frames = []
        while True:
            frame, progressed = self._next()
            if frame is not None:
                frames.append(frame)
            if not progressed:
                return frames

    def _starts(self, byte):
        return ((self._parse_text and byte in _TEXT_STARTS) or
                (self._parse_rtcm and byte == RTCM_PREAMBLE))

    def _skip_to_next_start(self, begin):
        """Drop bytes until the next possible frame start at or after `begin`."""
        i = begin
        while i < len(self._buf) and not self._starts(self._buf[i]):
            i += 1
        self.dropped_bytes += i
        del self._buf[:i]

    def _next(self):
        """Try to pull one frame off the buffer.

        Returns (frame or None, progressed).  progressed=False means more bytes
        are needed.
        """
        if not self._buf:
            return None, False
        if not self._starts(self._buf[0]):
            self._skip_to_next_start(0)
            return None, bool(self._buf)

        if self._buf[0] == RTCM_PREAMBLE:
            return self._next_rtcm()
        return self._next_text()

    def _next_text(self):
        end = self._buf.find(b'\n')
        if end < 0:
            if len(self._buf) > self._max_line:
                # never got a newline: drop this start char and resync
                self._skip_to_next_start(1)
                return None, True
            return None, False
        raw = bytes(self._buf[:end])
        del self._buf[:end + 1]
        if len(raw) > self._max_line:
            self.dropped_bytes += len(raw)
            return None, True
        try:
            text = raw.decode('ascii').rstrip('\r')
        except UnicodeDecodeError:
            # binary noise that happened to start with '$' or '#'
            self.dropped_bytes += len(raw)
            return None, True
        ftype = FrameType.NMEA if text.startswith('$') else FrameType.UNICORE
        return Frame(ftype, text=text), True

    def _next_rtcm(self):
        if len(self._buf) < 3:
            return None, False
        length = ((self._buf[1] & 0x03) << 8) | self._buf[2]
        if self._buf[1] & 0xFC:
            # reserved bits must be zero: not a real frame
            self._skip_to_next_start(1)
            return None, True
        total = 3 + length + 3
        if len(self._buf) < total:
            return None, False
        frame = bytes(self._buf[:total])
        if crc24q(frame[:-3]) != int.from_bytes(frame[-3:], 'big'):
            self._skip_to_next_start(1)
            return None, True
        del self._buf[:total]
        return Frame(FrameType.RTCM, data=frame), True
