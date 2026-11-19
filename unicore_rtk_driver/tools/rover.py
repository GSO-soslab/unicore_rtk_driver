r"""Run a rover receiver without ROS: send it RTCM corrections and print its solution.

    python3 -m unicore_rtk_driver.tools.rover --device /dev/ttyUSB0 --configure \
        --input tcp://<base_ip>:2101
    ros2 run unicore_rtk_driver unicore_rtk_rover --device /dev/ttyUSB0 \
        --input ntrip://USER:PASS@caster.example.org:2101/MOUNT

Corrections from each --input (the same URIs as the node's rtcm.inputs) are
framed, CRC checked and written to the receiver, with the same failover as the
node.  Every GGA from the receiver prints one status line, followed by
`heading <deg> deg (antennas <solution>)` when the receiver outputs HPR:

    17:32:30  position RTK fixed   sats 26  acc 0.012 m  hdop 0.6  corr age 1.0 s  base 0000

The position solution is relative to the base; the heading solution is
between the receiver's two antennas and does not use the base.

--configure sends the rover mode command and the logs (default GGA, GST and
HPR, once per second) after every connect.  They go to the receiver's working
memory only; nothing is saved.

Runs with Python 3 and pyserial (Debian/Ubuntu: python3-serial); ROS is
optional.
"""
import argparse
import logging
import signal
import sys
import threading
import time

from unicore_rtk_driver.protocol.checksums import nmea_is_valid
from unicore_rtk_driver.protocol.framer import FrameType, StreamFramer
from unicore_rtk_driver.protocol.solution import parse_gga, parse_gst, parse_hpr
from unicore_rtk_driver.receivers import create_receiver, parse_log_list, RoverSettings
from unicore_rtk_driver.receivers.unicore import DEFAULT_ROVER_MODE_COMMAND
from unicore_rtk_driver.rtcm.router import RtcmRouter
from unicore_rtk_driver.rtcm.transport import create_source
from unicore_rtk_driver.serial_link import SerialLink
from unicore_rtk_driver.tools.common import Logger

DEFAULT_TOOL_LOGS = ['GPGGA:1', 'GPGST:1', 'GPHPR:1']
MAX_AGE_S = 3.0   # accuracy and heading older than this are not shown


class Rover:

    def __init__(self, device, baudrate, inputs, logger, out=print, configure=False,
                 receiver='unicore', receiver_port='', mode_command=DEFAULT_ROVER_MODE_COMMAND,
                 logs=None, failover_timeout_s=5.0, serial_reconnect_s=1.0):
        self.log = logger
        self.out = out
        self.configure = configure
        self.receiver = create_receiver(receiver, receiver_port)
        self.rover = RoverSettings(mode_command=mode_command.strip())
        self.logs = parse_log_list(logs if logs is not None else DEFAULT_TOOL_LOGS)
        self.last_gga = None
        self._hpr = None
        self._hpr_time = 0.0
        self._gst = None
        self._gst_time = 0.0
        self.framer = StreamFramer()
        self.router = RtcmRouter(logger, failover_timeout_s)
        for uri in inputs:
            self.router.add_source(
                lambda on_data, uri=uri: create_source(uri, logger, on_data,
                                                       gga_provider=lambda: self.last_gga))
        self.link = SerialLink(device, baudrate, logger, on_data=self._on_serial_data,
                               on_connect=self._on_connect, reconnect_s=serial_reconnect_s)
        self.router.set_injector(self.link.write)
        self._inputs_started = threading.Event()

    def start(self):
        # the RTCM inputs start once the receiver port is open (_on_connect)
        self.link.start()

    def stop(self):
        self.link.stop()
        if self._inputs_started.is_set():
            self.router.stop()

    def _on_connect(self):
        """Run on the serial thread after each (re)connect: configuration, then inputs."""
        if self.configure:
            for cmd in self.receiver.configuration_commands(self.rover, self.logs):
                self.log.info(f'sending: {cmd}')
                self.link.write_line(cmd)
        if not self._inputs_started.is_set():
            self._inputs_started.set()
            self.router.start()

    def _on_serial_data(self, data: bytes):
        for frame in self.framer.feed(data):
            if frame.type is not FrameType.NMEA:
                continue
            line = frame.text
            if self.receiver.is_command_response(line):
                self.log.info(f'receiver: {line}')
                continue
            if not nmea_is_valid(line):
                continue
            gst = parse_gst(line)
            if gst is not None:
                self._gst, self._gst_time = gst, time.monotonic()
                continue
            hpr = parse_hpr(line)
            if hpr is not None:
                self._hpr, self._hpr_time = hpr, time.monotonic()
                continue
            gga = parse_gga(line)
            if gga is not None:
                self.last_gga = line
                self.out(self.status_line(gga))

    def _accuracy(self) -> str:
        """Horizontal standard deviation from the latest GST, '-' without one."""
        gst = self._gst
        if gst is None or time.monotonic() - self._gst_time >= MAX_AGE_S:
            return '-'
        std = gst.horizontal_std_m
        return f'{std:.3f} m' if std is not None else '-'

    def status_line(self, gga) -> str:
        t = gga.utc
        parts = [f'{t[0:2]}:{t[2:4]}:{t[4:6]}', f'position {gga.solution:<10}',
                 f'sats {gga.satellites:<2}',
                 f'acc {self._accuracy()}',
                 f'hdop {gga.hdop if gga.hdop is not None else "-"}',
                 'corr age ' + (f'{gga.correction_age_s:.1f} s' if gga.correction_age_s is not None
                                else '-'),
                 f'base {gga.station_id or "-"}']
        if self.router.sources:
            self.router.incoming.roll()
            parts.append(f'rtcm in {self.router.incoming.rate_bps:.0f} B/s')
        hpr = self._hpr
        if hpr is not None and time.monotonic() - self._hpr_time < MAX_AGE_S:
            heading = f'{hpr.heading_deg:.2f}' if hpr.heading_deg is not None else '-'
            parts.append(f'heading {heading} deg (antennas {hpr.solution})')
        return '  '.join(parts)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        prog='unicore_rtk_rover',
        description='Send RTCM corrections to a rover receiver and print its solution '
                    '(no ROS needed).')
    parser.add_argument('--device', required=True, help='receiver serial device')
    parser.add_argument('--baud', type=int, default=115200, help='receiver baud rate')
    parser.add_argument('--input', action='append', default=[],
                        help='RTCM input URI, repeatable, in failover order: udp://:PORT, '
                             'tcp://HOST:PORT, tcpserver://BIND:PORT, serial:///dev/ttyX?baud=N, '
                             'ntrip://[USER:PASS@]HOST:PORT/MOUNT. None: only print the solution.')
    parser.add_argument('--configure', action='store_true',
                        help='send the rover mode command and the logs after every connect')
    parser.add_argument('--receiver', choices=['unicore', 'generic'], default='unicore',
                        help='generic: never send commands to the receiver')
    parser.add_argument('--receiver-port', default='',
                        help='receiver port named in the log commands, for example COM1')
    parser.add_argument('--mode-command', default=DEFAULT_ROVER_MODE_COMMAND,
                        help='rover mode command sent with --configure')
    parser.add_argument('--log', action='append', metavar='LOG:PERIOD_S',
                        help='receiver log enabled with --configure, repeatable '
                             f'(default: {" ".join(DEFAULT_TOOL_LOGS)})')
    parser.add_argument('--failover-timeout-s', type=float, default=5.0)
    parser.add_argument('--serial-reconnect-s', type=float, default=1.0)
    parser.add_argument('--log-level', default='INFO',
                        choices=['DEBUG', 'INFO', 'WARNING', 'ERROR'])
    return parser.parse_args(argv)


def main(argv=None, stop_event=None, out=None):
    args = parse_args(argv)
    logging.basicConfig(level=args.log_level, stream=sys.stderr,
                        format='%(asctime)s %(levelname)s %(message)s')
    log = Logger('unicore_rtk_rover')
    try:
        rover = Rover(args.device, args.baud, args.input, log,
                      out=out or (lambda line: print(line, flush=True)),
                      configure=args.configure, receiver=args.receiver,
                      receiver_port=args.receiver_port, mode_command=args.mode_command,
                      logs=args.log, failover_timeout_s=args.failover_timeout_s,
                      serial_reconnect_s=args.serial_reconnect_s)
        rover.start()
    except (ValueError, OSError) as e:
        log.error(f'cannot start: {e}')
        return 2

    stop = stop_event or threading.Event()
    if stop_event is None:
        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, lambda *_: stop.set())
    log.info(f'rover on {args.device} at {args.baud} baud, RTCM inputs: '
             + (', '.join(args.input) or 'none'))
    while not stop.wait(1.0):
        pass
    rover.stop()
    log.info('stopped')
    return 0


if __name__ == '__main__':
    sys.exit(main())
