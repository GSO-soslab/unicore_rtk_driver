import socket
import threading
import time

import pytest

from rtcm_helpers import message, station_1005
from unicore_rtk_driver.rtcm import rtcm3
from unicore_rtk_driver.rtcm.router import RtcmRouter
from unicore_rtk_driver.rtcm.transport import create_sink, create_source, parse_endpoint


class Log:
    def __init__(self):
        self.lines = []

    def _log(self, msg, **_):
        self.lines.append(msg)

    debug = info = warn = warning = error = _log


def free_port(kind=socket.SOCK_STREAM):
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


# ------------------------------------------------------------------- rtcm3
def test_station_position_roundtrip():
    lat, lon, h = 41.49126464, -71.42186016, -14.11
    x, y, z = rtcm3.lla_to_ecef(lat, lon, h)
    fr = station_1005(42, x, y, z)
    assert rtcm3.message_type(fr) == 1005
    sid, dx, dy, dz = rtcm3.station_arp_ecef(fr)
    assert sid == 42
    assert (dx, dy, dz) == pytest.approx((x, y, z), abs=1e-4)
    # RTCM resolution is 0.1 mm per axis
    lat2, lon2, h2 = rtcm3.ecef_to_lla(dx, dy, dz)
    assert (lat2, lon2) == pytest.approx((lat, lon), abs=1e-8)
    assert h2 == pytest.approx(h, abs=2e-4)
    assert rtcm3.station_arp_ecef(message(1074)) is None


# ------------------------------------------------------------------ router
def test_router_reframes_split_input_and_fails_over():
    written = []
    r = RtcmRouter(Log(), failover_timeout_s=0.2)
    r.set_injector(written.append)
    a, b = message(1074, 40), message(1084, 40)

    # source 1 alone, split across reads: forwarded only once complete
    r.inject(a[:10], 1)
    assert written == []
    r.inject(a[10:], 1)
    assert written == [a]

    # preferred source 0 appears: source 1 is held back
    r.inject(b, 0)
    r.inject(a, 1)
    assert written == [a, b]

    # source 0 goes quiet: source 1 takes over after the timeout
    time.sleep(0.25)
    r.inject(a, 1)
    assert written == [a, b, a]
    assert r.incoming.message_counts[1074] == 2


def test_router_not_delayed_by_text_like_bytes():
    written = []
    r = RtcmRouter(Log())
    r.set_injector(written.append)
    r.inject(b'$#garbage without newline' + message(1074), 0)
    assert written == [message(1074)]


def test_router_drops_non_rtcm_bytes():
    written = []
    r = RtcmRouter(Log())
    r.set_injector(written.append)
    r.inject(b'ICY 200 OK\r\n\r\n' + message(1006), 0)
    assert written == [message(1006)]


def test_router_reports_base_station():
    log = Log()
    r = RtcmRouter(log)
    x, y, z = rtcm3.lla_to_ecef(41.5, -71.4, 10.0)
    r.publish(station_1005(7, x, y, z))
    assert r.base_station[0] == 7
    assert r.base_station[1:3] == pytest.approx((41.5, -71.4), abs=1e-8)
    assert r.base_station[3] == pytest.approx(10.0, abs=2e-4)
    assert any('base station 7' in line for line in log.lines)


# -------------------------------------------------------------- endpoints
def test_endpoint_parsing():
    ep = parse_endpoint('ntrip://us%40r:p%3Ass@caster.example.org:2101/MOUNT1')
    assert (ep.scheme, ep.host, ep.port, ep.path, ep.user, ep.password) == \
        ('ntrip', 'caster.example.org', 2101, 'MOUNT1', 'us@r', 'p:ss')
    ep = parse_endpoint('serial:///dev/ttyUSB1?baud=57600')
    assert (ep.path, ep.options['baud']) == ('/dev/ttyUSB1', '57600')
    assert parse_endpoint('udp://:2101').host == '0.0.0.0'
    for bad in ('http://x:1', 'tcp://host', 'tcp://:2101', 'serial://'):
        with pytest.raises(ValueError):
            parse_endpoint(bad)


# -------------------------------------------------------------- transports
def run_pair(sink_uri, source_uri):
    log, got = Log(), []
    lock = threading.Lock()

    def on_data(d):
        with lock:
            got.append(d)
    source = create_source(source_uri, log, on_data)
    sink = create_sink(sink_uri, log)
    # whichever side listens must start first
    first, second = (sink, source) if 'tcpserver' in sink_uri else (source, sink)
    first.start()
    second.start()
    try:
        frames = [message(1074 + 10 * i, 50) for i in range(4)]
        assert wait_for(lambda: sink.status() in ('ok', 'connected', '1 client(s)'))
        for f in frames:
            sink.send(f)
        assert wait_for(lambda: b''.join(got) == b''.join(frames)), log.lines
    finally:
        sink.stop()
        source.stop()


def test_udp():
    port = free_port(socket.SOCK_DGRAM)
    run_pair(f'udp://127.0.0.1:{port}', f'udp://127.0.0.1:{port}')


def test_tcpserver_sink_to_tcp_client_source():
    port = free_port()
    run_pair(f'tcpserver://127.0.0.1:{port}', f'tcp://127.0.0.1:{port}')


def test_tcp_client_sink_to_tcpserver_source():
    port = free_port()
    run_pair(f'tcp://127.0.0.1:{port}', f'tcpserver://127.0.0.1:{port}')


def test_tcpserver_sink_drops_dead_client_and_keeps_serving():
    port = free_port()
    log = Log()
    sink = create_sink(f'tcpserver://127.0.0.1:{port}', log)
    sink.start()
    try:
        dead = socket.create_connection(('127.0.0.1', port))
        live = socket.create_connection(('127.0.0.1', port))
        assert wait_for(lambda: sink.status() == '2 client(s)')
        dead.close()
        for _ in range(50):          # first sends to the closed peer fail
            sink.send(message(1074, 100))
        assert wait_for(lambda: sink.status() == '1 client(s)')
        live.settimeout(1.0)
        assert live.recv(10)
        live.close()
    finally:
        sink.stop()


def test_ntrip_client_against_fake_caster():
    port = free_port()
    server = socket.socket()
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('127.0.0.1', port))
    server.listen(1)
    request, gga_seen = [], threading.Event()
    corrections = message(1074, 60) + message(1006, 20)

    def caster():
        conn, _ = server.accept()
        data = b''
        while b'\r\n\r\n' not in data:
            data += conn.recv(1024)
        request.append(data.decode())
        conn.sendall(b'ICY 200 OK\r\n\r\n' + corrections)
        conn.settimeout(2.0)
        try:
            while not gga_seen.is_set():
                if b'$GNGGA' in conn.recv(1024):
                    gga_seen.set()
        except OSError:
            pass
        conn.close()
    threading.Thread(target=caster, daemon=True).start()

    got = []
    gga = '$GNGGA,173144.00,4134.5,N,07147.2,W,1,28,0.5,81.8,M,-30.6,M,,*72'
    src = create_source(f'ntrip://user:secret@127.0.0.1:{port}/MOUNT', Log(), got.append,
                        gga_provider=lambda: gga)
    src.start()
    try:
        assert wait_for(lambda: b''.join(got) == corrections)
        assert gga_seen.wait(3.0)
        assert request[0].startswith('GET /MOUNT HTTP/1.1\r\n')
        assert 'Authorization: Basic dXNlcjpzZWNyZXQ=' in request[0]
    finally:
        src.stop()
        server.close()


def test_endpoint_options():
    log = Log()
    src = create_source('tcp://127.0.0.1:1?reconnect_s=0.25', log, lambda d: None)
    assert src.reconnect_s == 0.25
    sink = create_sink('tcpserver://127.0.0.1:1?send_timeout_s=0.01', log)
    assert sink.send_timeout_s == 0.01
    ntrip = create_source('ntrip://h:1/M?gga_period_s=3', log, lambda d: None)
    assert ntrip.gga_period == 3.0
    with pytest.raises(ValueError):
        create_source('tcp://127.0.0.1:1?reconnect_s=soon', log, lambda d: None)
