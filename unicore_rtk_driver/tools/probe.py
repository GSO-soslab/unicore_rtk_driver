"""Identify what a GNSS receiver on a serial port is sending.

    ros2 run unicore_rtk_driver unicore_rtk_probe /dev/ttyUSB0
    ros2 run unicore_rtk_driver unicore_rtk_probe /dev/ttyUSB0 --baud 115200 --duration 5
    ros2 run unicore_rtk_driver unicore_rtk_probe /dev/ttyUSB0 --query-version

Listens at each baud rate and reports the NMEA sentences, Unicore logs and
RTCM messages it sees, with checksum results, then suggests driver
parameters.  With --query-version it also sends the Unicore query VERSIONA,
which reads the firmware version and leaves the settings unchanged.
"""
import argparse
import sys
import time
from collections import Counter
from dataclasses import dataclass, field

import serial

from unicore_rtk_driver.protocol.checksums import nmea_is_valid, unicore_is_valid
from unicore_rtk_driver.protocol.framer import FrameType, StreamFramer
from unicore_rtk_driver.rtcm import rtcm3

DEFAULT_BAUDS = (115200, 230400, 460800, 921600, 57600, 38400, 9600)
HEADING_SOURCES = ('UNIHEADINGA', 'HPR', 'HDT')


@dataclass
class ProbeResult:
    baudrate: int
    bytes: int = 0
    nmea: Counter = field(default_factory=Counter)
    unicore: Counter = field(default_factory=Counter)
    rtcm: Counter = field(default_factory=Counter)
    bad_checksums: int = 0
    dropped_bytes: int = 0
    replies: list = field(default_factory=list)

    @property
    def valid_frames(self) -> int:
        return sum(self.nmea.values()) + sum(self.unicore.values()) + sum(self.rtcm.values())

    @property
    def looks_unicore(self) -> bool:
        return bool(self.unicore) or any(r.startswith(('$command', '#VERSIONA')) for r in self.replies)

    @property
    def has_heading(self) -> bool:
        return any(s in self.nmea or s in self.unicore for s in HEADING_SOURCES)


def sentence_name(line: str) -> str:
    """'$GNGGA,...' -> 'GGA', '#UNIHEADINGA,...' -> 'UNIHEADINGA'."""
    head = line.split(',', 1)[0].split(';', 1)[0]
    if head.startswith('$'):
        return head[3:] if len(head) > 3 and not head.startswith('$command') else head[1:]
    return head[1:]


def analyse(data: bytes, baudrate: int) -> ProbeResult:
    result = ProbeResult(baudrate, bytes=len(data))
    framer = StreamFramer()
    for frame in framer.feed(data):
        if frame.type is FrameType.RTCM:
            result.rtcm[rtcm3.message_type(frame.data)] += 1
        elif frame.text.startswith(('$command', '#VERSIONA')):
            result.replies.append(frame.text)
        elif frame.type is FrameType.NMEA:
            if nmea_is_valid(frame.text):
                result.nmea[sentence_name(frame.text)] += 1
            else:
                result.bad_checksums += 1
        elif unicore_is_valid(frame.text):
            result.unicore[sentence_name(frame.text)] += 1
        else:
            result.bad_checksums += 1
    result.dropped_bytes = framer.dropped_bytes
    return result


def listen(device: str, baudrate: int, duration: float, query_version: bool) -> ProbeResult:
    with serial.Serial(device, baudrate, timeout=0.1) as port:
        port.reset_input_buffer()
        if query_version:
            port.write(b'VERSIONA\r\n')
        data = b''
        end = time.monotonic() + duration
        while time.monotonic() < end:
            data += port.read(port.in_waiting or 1)
    return analyse(data, baudrate)


def report(result: ProbeResult, device: str) -> str:
    def fmt(counter):
        return ', '.join(f'{k} x{v}' for k, v in sorted(counter.items(), key=str)) or '-'

    lines = [
        f'Receiver on {device} at {result.baudrate} baud',
        f'  bytes read     : {result.bytes} ({result.dropped_bytes} not part of any frame)',
        f'  NMEA           : {fmt(result.nmea)}',
        f'  Unicore logs   : {fmt(result.unicore)}',
        f'  RTCM messages  : {fmt(result.rtcm)}',
        f'  bad checksums  : {result.bad_checksums}',
    ]
    for reply in result.replies:
        lines.append(f'  reply          : {reply}')
    lines += [
        '',
        'Suggested parameters:',
        f'  device: {device}',
        f'  baudrate: {result.baudrate}',
        f'  receiver: {"unicore" if result.looks_unicore else "generic"}',
    ]
    if result.rtcm:
        lines.append('  # receiver is outputting RTCM: it is (or was) configured as a base')
    if result.looks_unicore and not (result.nmea or result.unicore):
        lines.append('  configure_receiver: true   # no outputs enabled: let the driver enable them')
    else:
        if not result.has_heading:
            lines.append('  # no heading sentences seen: heading topics will not be published')
        if 'GST' not in result.nmea:
            lines.append('  # no GST: ~/fix is published on GST; enable it (receiver_logs)')
    return '\n'.join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('device')
    parser.add_argument('--baud', type=int, action='append',
                        help='baud rate to try (repeatable); default: common rates')
    parser.add_argument('--duration', type=float, default=3.0, help='seconds per baud rate')
    parser.add_argument('--query-version', action='store_true',
                        help='send the read-only query VERSIONA at each baud rate')
    args = parser.parse_args(argv)

    best = None
    for baud in args.baud or DEFAULT_BAUDS:
        try:
            result = listen(args.device, baud, args.duration, args.query_version)
        except (serial.SerialException, OSError) as e:
            print(f'Cannot open {args.device}: {e}', file=sys.stderr)
            if 'Permission denied' in str(e):
                print('  add yourself to the dialout group (scripts/install_udev_rules.sh '
                      'does this), then log out and back in', file=sys.stderr)
            return 2
        score = result.valid_frames + len(result.replies)
        print(f'{baud:>7} baud: {result.bytes:6d} bytes, {score} valid frames')
        if score and (best is None or score > best.valid_frames + len(best.replies)):
            best = result
        if score >= 5:
            break

    print()
    if best is None:
        print(f'No valid GNSS data on {args.device}.')
        print('  - receiver powered? (module LED steady, not just the USB adapter)')
        print('  - outputs enabled on this port? try --query-version')
        print('  - TX/RX wiring')
        return 1
    print(report(best, args.device))
    return 0


if __name__ == '__main__':
    sys.exit(main())
