"""Forward the RTCM output of a base receiver to the network, without ROS.

    python3 -m unicore_rtk_driver.tools.relay --device /dev/ttyACM0 --baud 115200 \\
        --output tcpserver://0.0.0.0:2101 --output udp://192.168.1.255:2101

    python3 -m unicore_rtk_driver.tools.relay @/home/user/rtk/relay.conf

Uses the same serial link, framer and RTCM outputs as the ROS node: only
complete RTCM frames that pass the CRC are forwarded, and the serial port is
reopened after a disconnect.  NMEA and other text from the receiver is
ignored.  The receiver is not configured; set up base mode and RTCM output
with the receiver's own tools.

Arguments can be read from a file with @FILE, one argument per line, e.g.:

    --device=/dev/ttyACM0
    --baud=115200
    --output=tcpserver://0.0.0.0:2101
    --output=udp://192.168.1.255:2101

Requires only Python 3 and pyserial (Debian/Ubuntu: python3-serial).
"""
import argparse
import logging
import signal
import sys
import threading
import time

from unicore_rtk_driver.protocol.framer import FrameType, StreamFramer
from unicore_rtk_driver.rtcm.router import RtcmRouter
from unicore_rtk_driver.rtcm.transport import create_sink
from unicore_rtk_driver.serial_link import SerialLink


class Logger:
    """Python logging with the interface the shared modules expect
    (warn() and throttle_duration_sec, as in rclpy)."""

    def __init__(self, name='unicore_rtk_relay'):
        self._log = logging.getLogger(name)
        self._last = {}
        self._lock = threading.Lock()

    def _emit(self, level, msg, throttle_duration_sec=0.0):
        if throttle_duration_sec:
            now = time.monotonic()
            with self._lock:
                if now - self._last.get((level, msg), -1e9) < throttle_duration_sec:
                    return
                self._last[(level, msg)] = now
        self._log.log(level, msg)

    def debug(self, msg, **kw):
        self._emit(logging.DEBUG, msg, **kw)

    def info(self, msg, **kw):
        self._emit(logging.INFO, msg, **kw)

    def warn(self, msg, **kw):
        self._emit(logging.WARNING, msg, **kw)

    warning = warn

    def error(self, msg, **kw):
        self._emit(logging.ERROR, msg, **kw)


class Relay:

    def __init__(self, device, baudrate, outputs, logger, serial_reconnect_s=1.0,
                 no_rtcm_warning_s=10.0):
        self.log = logger
        self.no_rtcm_warning_s = no_rtcm_warning_s
        self.framer = StreamFramer()
        self.router = RtcmRouter(logger)
        for uri in outputs:
            self.router.add_sink(create_sink(uri, logger))
        self.link = SerialLink(device, baudrate, logger, on_data=self._on_serial_data,
                               reconnect_s=serial_reconnect_s)
        self._started = time.monotonic()

    def _on_serial_data(self, data):
        for frame in self.framer.feed(data):
            if frame.type is FrameType.RTCM:
                self.router.publish(frame.data)

    def start(self):
        self.router.start()
        self.link.start()

    def stop(self):
        self.link.stop()
        self.router.stop()

    def status(self):
        """One line summary; also updates the output rate."""
        out = self.router.outgoing
        out.roll()
        parts = [
            f'serial {"connected" if self.link.is_open else "disconnected"}',
            f'rtcm {out.rate_bps:.0f} B/s',
            'msgs ' + (' '.join(f'{k}:{v}' for k, v in sorted(out.message_counts.items())) or '-'),
        ]
        parts += [f'{s.name} {s.status()}' for s in self.router.sinks]
        if self.router.base_station:
            sid, lat, lon, h = self.router.base_station
            parts.append(f'base {sid} {lat:.8f} {lon:.8f} {h:.3f}m')
        return ' | '.join(parts)

    def check_rtcm(self):
        """Warn when the receiver has not produced RTCM for a while."""
        age = self.router.outgoing.age()
        silent = age if age is not None else time.monotonic() - self._started
        if self.link.is_open and silent > self.no_rtcm_warning_s:
            self.log.warn(f'no RTCM from the receiver for {silent:.0f} s: is base mode and '
                          'RTCM output enabled on this port?', throttle_duration_sec=30.0)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        prog='unicore_rtk_relay', fromfile_prefix_chars='@',
        description='Forward RTCM from a base receiver to UDP / TCP / serial outputs, without ROS.')
    parser.add_argument('--device', required=True, help='receiver serial device')
    parser.add_argument('--baud', type=int, default=115200, help='receiver baud rate')
    parser.add_argument('--output', action='append', required=True,
                        help='RTCM output URI (repeatable): udp://HOST:PORT, '
                             'tcpserver://BIND:PORT, tcp://HOST:PORT, serial:///dev/ttyX?baud=N')
    parser.add_argument('--status-period', type=float, default=10.0,
                        help='seconds between status lines, 0 disables them')
    parser.add_argument('--serial-reconnect-s', type=float, default=1.0)
    parser.add_argument('--no-rtcm-warning-s', type=float, default=10.0,
                        help='warn when no RTCM arrives for this long')
    parser.add_argument('--log-level', default='INFO',
                        choices=['DEBUG', 'INFO', 'WARNING', 'ERROR'])
    return parser.parse_args(argv)


def main(argv=None, stop_event=None):
    args = parse_args(argv)
    logging.basicConfig(level=args.log_level, stream=sys.stdout,
                        format='%(asctime)s %(levelname)s %(message)s')
    log = Logger()
    try:
        relay = Relay(args.device, args.baud, args.output, log,
                      serial_reconnect_s=args.serial_reconnect_s,
                      no_rtcm_warning_s=args.no_rtcm_warning_s)
        relay.start()
    except (ValueError, OSError) as e:
        log.error(f'cannot start: {e}')
        return 2

    stop = stop_event or threading.Event()
    if stop_event is None:
        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, lambda *_: stop.set())
    log.info(f'relaying RTCM from {args.device} at {args.baud} baud to ' + ', '.join(args.output))

    next_status = time.monotonic() + args.status_period
    while not stop.wait(1.0):
        relay.check_rtcm()
        if args.status_period > 0 and time.monotonic() >= next_status:
            log.info(relay.status())
            next_status += args.status_period
    relay.stop()
    log.info('stopped')
    return 0


if __name__ == '__main__':
    sys.exit(main())
