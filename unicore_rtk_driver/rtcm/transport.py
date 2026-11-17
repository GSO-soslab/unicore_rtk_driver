"""RTCM transports.

Endpoints are configured as URIs:

  Sinks (base station output)
    udp://HOST:PORT              datagram per frame; broadcast addresses allowed
    tcpserver://BIND:PORT        rovers connect to the base
    tcp://HOST:PORT              base connects out to a relay / rover
    serial:///dev/ttyX?baud=N    radio modem

  Sources (rover input)
    udp://BIND:PORT              listen for datagrams
    tcp://HOST:PORT              connect to a base's tcpserver
    tcpserver://BIND:PORT        let a base connect in
    serial:///dev/ttyX?baud=N    radio modem
    ntrip://[USER:PASS@]HOST:PORT/MOUNTPOINT   NTRIP v1/v2 caster

Per-endpoint options go in the query string:
    reconnect_s=2.0       wait between (re)connection attempts (tcp, serial, ntrip)
    send_timeout_s=0.05   longest a sink may block on one client (tcp, tcpserver, serial)
    baud=57600            serial endpoints
    gga_period_s=10       ntrip: how often the rover position is sent to the caster
e.g.  tcp://192.168.1.10:2101?reconnect_s=0.5

A sink's `send()` never blocks for longer than send_timeout_s: a client that
cannot keep up is dropped rather than delaying every other client.
"""
import base64
import socket
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional
from urllib.parse import parse_qs, unquote, urlparse

import serial

DEFAULT_RECONNECT_S = 2.0
DEFAULT_SEND_TIMEOUT_S = 0.05
DEFAULT_SERIAL_BAUD = 57600
DEFAULT_GGA_PERIOD_S = 10.0


@dataclass
class Endpoint:
    scheme: str
    host: str = ''
    port: int = 0
    path: str = ''
    user: str = ''
    password: str = ''
    options: Dict[str, str] = field(default_factory=dict)
    uri: str = ''


def parse_endpoint(uri: str) -> Endpoint:
    u = urlparse(uri.strip())
    scheme = u.scheme.lower()
    if scheme not in ('udp', 'tcp', 'tcpserver', 'serial', 'ntrip'):
        raise ValueError(f'unsupported RTCM endpoint scheme in {uri!r}')
    options = {k: v[-1] for k, v in parse_qs(u.query).items()}
    if scheme == 'serial':
        if not u.path:
            raise ValueError(f'serial endpoint needs a device path: {uri!r}')
        return Endpoint(scheme, path=u.path, options=options, uri=uri)
    if not u.port:
        raise ValueError(f'endpoint needs a port: {uri!r}')
    host = u.hostname or ('0.0.0.0' if scheme in ('udp', 'tcpserver') else '')
    if not host:
        raise ValueError(f'endpoint needs a host: {uri!r}')
    return Endpoint(scheme, host=host, port=u.port, path=u.path.lstrip('/'),
                    user=unquote(u.username or ''), password=unquote(u.password or ''),
                    options=options, uri=uri)


class Transport:
    """Common bookkeeping for sinks and sources."""

    def __init__(self, endpoint: Endpoint, logger):
        self.endpoint = endpoint
        self.log = logger
        opts = endpoint.options
        try:
            self.reconnect_s = float(opts.get('reconnect_s', DEFAULT_RECONNECT_S))
            self.send_timeout_s = float(opts.get('send_timeout_s', DEFAULT_SEND_TIMEOUT_S))
            self.baudrate = int(opts.get('baud', DEFAULT_SERIAL_BAUD))
        except ValueError as e:
            raise ValueError(f'bad option in {endpoint.uri!r}: {e}')
        self.bytes = 0
        self._stop = threading.Event()
        self._threads: List[threading.Thread] = []

    @property
    def name(self) -> str:
        return self.endpoint.uri

    def _spawn(self, target):
        t = threading.Thread(target=target, name=f'rtcm:{self.name}', daemon=True)
        self._threads.append(t)
        t.start()

    def start(self):
        pass

    def stop(self):
        self._stop.set()
        for t in self._threads:
            t.join(timeout=1.0)

    def status(self) -> str:
        return 'ok'


# --------------------------------------------------------------------- sinks
class Sink(Transport):

    def send(self, data: bytes):
        raise NotImplementedError


class UdpSink(Sink):

    def start(self):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self._addr = (self.endpoint.host, self.endpoint.port)

    def send(self, data):
        try:
            self._sock.sendto(data, self._addr)
            self.bytes += len(data)
        except OSError as e:
            self.log.warn(f'RTCM {self.name}: {e}', throttle_duration_sec=5.0)

    def stop(self):
        super().stop()
        self._sock.close()


class TcpServerSink(Sink):

    def start(self):
        self._clients: Dict[socket.socket, tuple] = {}
        self._lock = threading.Lock()
        self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server.bind((self.endpoint.host, self.endpoint.port))
        self._server.listen(8)
        self._server.settimeout(0.5)
        self._spawn(self._accept_loop)

    def _accept_loop(self):
        while not self._stop.is_set():
            try:
                conn, addr = self._server.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            conn.settimeout(self.send_timeout_s)
            with self._lock:
                self._clients[conn] = addr
            self.log.info(f'RTCM {self.name}: client {addr[0]}:{addr[1]} connected')

    def send(self, data):
        with self._lock:
            clients = list(self._clients.items())
        for conn, addr in clients:
            try:
                conn.sendall(data)
                self.bytes += len(data)
            except OSError:
                self.log.warn(f'RTCM {self.name}: dropping client {addr[0]}:{addr[1]}')
                with self._lock:
                    self._clients.pop(conn, None)
                conn.close()

    def status(self):
        return f'{len(self._clients)} client(s)'

    def stop(self):
        super().stop()
        self._server.close()
        with self._lock:
            for conn in self._clients:
                conn.close()
            self._clients.clear()


class TcpClientSink(Sink):

    def start(self):
        self._sock = None
        self._lock = threading.Lock()
        self._spawn(self._connect_loop)

    def _connect_loop(self):
        while not self._stop.is_set():
            if self._sock is None:
                try:
                    s = socket.create_connection((self.endpoint.host, self.endpoint.port), timeout=2.0)
                    s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    s.settimeout(self.send_timeout_s)
                    with self._lock:
                        self._sock = s
                    self.log.info(f'RTCM {self.name}: connected')
                except OSError:
                    pass
            self._stop.wait(self.reconnect_s)

    def send(self, data):
        with self._lock:
            s = self._sock
        if s is None:
            return
        try:
            s.sendall(data)
            self.bytes += len(data)
        except OSError as e:
            self.log.warn(f'RTCM {self.name}: {e}, reconnecting')
            with self._lock:
                self._sock = None
            s.close()

    def status(self):
        return 'connected' if self._sock is not None else 'connecting'

    def stop(self):
        super().stop()
        if self._sock is not None:
            self._sock.close()


class SerialSink(Sink):

    def start(self):
        self._port = serial.Serial(self.endpoint.path, self.baudrate,
                                   write_timeout=self.send_timeout_s)

    def send(self, data):
        try:
            self._port.write(data)
            self.bytes += len(data)
        except (serial.SerialException, OSError) as e:
            self.log.warn(f'RTCM {self.name}: {e}', throttle_duration_sec=5.0)

    def stop(self):
        super().stop()
        self._port.close()


# ------------------------------------------------------------------- sources
class Source(Transport):
    """Delivers received bytes to `on_data` from its own thread."""

    def __init__(self, endpoint, logger, on_data: Callable[[bytes], None]):
        super().__init__(endpoint, logger)
        self.on_data = on_data
        self.last_rx = 0.0

    def _deliver(self, data: bytes):
        if data:
            self.bytes += len(data)
            self.last_rx = time.monotonic()
            self.on_data(data)

    def status(self):
        if not self.last_rx:
            return 'no data yet'
        return f'last data {time.monotonic() - self.last_rx:.1f} s ago'


class UdpSource(Source):

    def start(self):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self.endpoint.host, self.endpoint.port))
        self._sock.settimeout(0.5)
        self._spawn(self._loop)

    def _loop(self):
        while not self._stop.is_set():
            try:
                data, _ = self._sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            self._deliver(data)

    def stop(self):
        super().stop()
        self._sock.close()


class _StreamSource(Source):
    """Reads a connected stream socket until it closes."""

    def _read_stream(self, sock: socket.socket):
        sock.settimeout(0.5)
        while not self._stop.is_set():
            try:
                data = sock.recv(4096)
            except socket.timeout:
                continue
            except OSError:
                return
            if not data:
                return
            self._deliver(data)


class TcpClientSource(_StreamSource):

    def start(self):
        self._spawn(self._loop)

    def _connect(self) -> Optional[socket.socket]:
        return socket.create_connection((self.endpoint.host, self.endpoint.port), timeout=2.0)

    def _loop(self):
        while not self._stop.is_set():
            try:
                sock = self._connect()
            except OSError as e:
                self.log.warn(f'RTCM {self.name}: {e}', throttle_duration_sec=10.0)
                self._stop.wait(self.reconnect_s)
                continue
            if sock is None:
                self._stop.wait(self.reconnect_s)
                continue
            self.log.info(f'RTCM {self.name}: connected')
            self._on_connected(sock)
            try:
                self._read_stream(sock)
            finally:
                sock.close()
            if not self._stop.is_set():
                self.log.warn(f'RTCM {self.name}: disconnected, retrying')
                self._stop.wait(self.reconnect_s)

    def _on_connected(self, sock):
        pass


class TcpServerSource(_StreamSource):

    def start(self):
        self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server.bind((self.endpoint.host, self.endpoint.port))
        self._server.listen(2)
        self._server.settimeout(0.5)
        self._spawn(self._loop)

    def _loop(self):
        while not self._stop.is_set():
            try:
                conn, addr = self._server.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            self.log.info(f'RTCM {self.name}: base {addr[0]}:{addr[1]} connected')
            try:
                self._read_stream(conn)
            finally:
                conn.close()

    def stop(self):
        super().stop()
        self._server.close()


class SerialSource(Source):

    def start(self):
        self._spawn(self._loop)

    def _loop(self):
        while not self._stop.is_set():
            try:
                with serial.Serial(self.endpoint.path, self.baudrate, timeout=0.5) as port:
                    self.log.info(f'RTCM {self.name}: opened')
                    while not self._stop.is_set():
                        self._deliver(port.read(port.in_waiting or 1))
            except (serial.SerialException, OSError) as e:
                self.log.warn(f'RTCM {self.name}: {e}', throttle_duration_sec=10.0)
                self._stop.wait(self.reconnect_s)


class NtripSource(TcpClientSource):
    """NTRIP client (rev1 and rev2 casters).

    `gga_provider` returns the latest GGA sentence (or None); it is sent on
    connect and every `gga_period_s` seconds (URI option), which VRS /
    nearest-base mountpoints require.
    """

    def __init__(self, endpoint, logger, on_data, gga_provider=None):
        super().__init__(endpoint, logger, on_data)
        if not endpoint.path:
            raise ValueError(f'NTRIP endpoint needs a mountpoint: {endpoint.uri!r}')
        self.gga_provider = gga_provider
        self.gga_period = float(endpoint.options.get('gga_period_s', DEFAULT_GGA_PERIOD_S))

    def _connect(self):
        sock = socket.create_connection((self.endpoint.host, self.endpoint.port), timeout=5.0)
        request = (f'GET /{self.endpoint.path} HTTP/1.1\r\n'
                   f'Host: {self.endpoint.host}:{self.endpoint.port}\r\n'
                   'Ntrip-Version: Ntrip/2.0\r\n'
                   'User-Agent: NTRIP unicore_rtk_driver/2.0\r\n'
                   'Connection: close\r\n')
        if self.endpoint.user:
            token = base64.b64encode(
                f'{self.endpoint.user}:{self.endpoint.password}'.encode()).decode()
            request += f'Authorization: Basic {token}\r\n'
        sock.sendall((request + '\r\n').encode())

        response = b''
        while b'\r\n' not in response and len(response) < 1024:
            chunk = sock.recv(1024)
            if not chunk:
                break
            response += chunk
        status = response.split(b'\r\n', 1)[0].decode(errors='replace')
        if not (status.startswith('ICY 200') or (status.startswith('HTTP/') and ' 200' in status)):
            sock.close()
            raise OSError(f'caster refused: {status!r}')
        # anything after the headers is already correction data
        head, sep, rest = response.partition(b'\r\n\r\n')
        if sep:
            self._deliver(rest)
        return sock

    def _on_connected(self, sock):
        if self.gga_provider is not None:
            self._spawn_gga(sock)

    def _spawn_gga(self, sock):
        def loop():
            while not self._stop.is_set() and sock.fileno() >= 0:
                gga = self.gga_provider()
                if gga:
                    try:
                        sock.sendall((gga.strip() + '\r\n').encode())
                    except OSError:
                        return
                self._stop.wait(self.gga_period)
        threading.Thread(target=loop, name=f'ntrip-gga:{self.name}', daemon=True).start()


# ------------------------------------------------------------------ factory
_SINKS = {'udp': UdpSink, 'tcpserver': TcpServerSink, 'tcp': TcpClientSink, 'serial': SerialSink}
_SOURCES = {'udp': UdpSource, 'tcpserver': TcpServerSource, 'tcp': TcpClientSource,
            'serial': SerialSource, 'ntrip': NtripSource}


def create_sink(uri: str, logger) -> Sink:
    ep = parse_endpoint(uri)
    if ep.scheme not in _SINKS:
        raise ValueError(f'{ep.scheme!r} cannot be used as an RTCM output: {uri!r}')
    return _SINKS[ep.scheme](ep, logger)


def create_source(uri: str, logger, on_data, gga_provider=None) -> Source:
    ep = parse_endpoint(uri)
    if ep.scheme == 'ntrip':
        return NtripSource(ep, logger, on_data, gga_provider=gga_provider)
    return _SOURCES[ep.scheme](ep, logger, on_data)
