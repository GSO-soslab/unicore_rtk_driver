import logging
import os
import pty
import socket
import threading
import time
import tty

from rtcm_helpers import lla_to_ecef, message, station_1005
from unicore_rtk_driver.tools import relay


def free_port(kind):
    with socket.socket(socket.AF_INET, kind) as s:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]


def wait_for(cond, timeout=3.0):
    end = time.time() + timeout
    while time.time() < end:
        if cond():
            return True
        time.sleep(0.01)
    return False


def test_arguments_from_file(tmp_path):
    conf = tmp_path / 'relay.conf'
    conf.write_text('--device=/dev/ttyACM0\n--baud=57600\n'
                    '--output=tcpserver://0.0.0.0:2101\n--output=udp://192.168.1.255:2101\n')
    args = relay.parse_args([f'@{conf}'])
    assert (args.device, args.baud) == ('/dev/ttyACM0', 57600)
    assert args.output == ['tcpserver://0.0.0.0:2101', 'udp://192.168.1.255:2101']


class ListHandler(logging.Handler):
    def __init__(self):
        super().__init__()
        self.messages = []

    def emit(self, record):
        self.messages.append(record.getMessage())


def test_logger_throttle():
    handler = ListHandler()
    py_log = logging.getLogger('test_relay_throttle')
    py_log.addHandler(handler)
    py_log.setLevel(logging.INFO)
    py_log.propagate = False
    log = relay.Logger('test_relay_throttle')
    for _ in range(5):
        log.warn('same message', throttle_duration_sec=60.0)
    log.info('other message')
    assert handler.messages == ['same message', 'other message']


def test_relay_forwards_rtcm_to_tcp_and_udp():
    master, slave = pty.openpty()
    tty.setraw(slave)
    tcp_port, udp_port = free_port(socket.SOCK_STREAM), free_port(socket.SOCK_DGRAM)

    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.bind(('127.0.0.1', udp_port))
    udp.settimeout(2.0)

    stop = threading.Event()
    argv = ['--device', os.ttyname(slave), '--output', f'tcpserver://127.0.0.1:{tcp_port}',
            '--output', f'udp://127.0.0.1:{udp_port}', '--status-period', '0']
    t = threading.Thread(target=relay.main, args=(argv, stop), daemon=True)
    t.start()
    try:
        client = None
        for _ in range(100):
            try:
                client = socket.create_connection(('127.0.0.1', tcp_port), timeout=2.0)
                break
            except OSError:
                time.sleep(0.02)
        assert client is not None
        time.sleep(0.2)  # let the server register the client

        x, y, z = lla_to_ecef(41.5, -71.4, 10.0)
        frames = [station_1005(3, x, y, z), message(1074, 120), message(1084, 80)]
        # NMEA and noise around the frames must not be forwarded
        os.write(master, b'$GNGGA,173144.00,,,,,0,00,9999.0,,,,,,*7E\r\n'
                 + frames[0] + b'noise' + frames[1] + b'$GNRMC,x*00\r\n' + frames[2])

        received = b''
        while len(received) < sum(map(len, frames)):
            received += client.recv(4096)
        assert received == b''.join(frames)
        datagrams = [udp.recv(4096) for _ in frames]
        assert datagrams == frames
        client.close()
    finally:
        stop.set()
        t.join(timeout=5)
        udp.close()
        os.close(master)
        os.close(slave)
    assert not t.is_alive()


def test_status_line_reports_base_position():
    r = relay.Relay('/dev/null-does-not-exist', 115200, [], relay.Logger('test_relay_status'))
    x, y, z = lla_to_ecef(41.5, -71.4, 10.0)
    r._on_serial_data(station_1005(7, x, y, z))
    line = r.status()
    assert 'serial disconnected' in line
    assert 'msgs 1005:1' in line
    assert 'base 7 41.50000000 -71.40000000' in line
