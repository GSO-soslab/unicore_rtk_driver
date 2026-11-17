"""Serial connection to the receiver.

A dedicated thread blocks on the port and hands every chunk to `on_data`
together with the time it was read, so arrival stamps do not depend on a ROS
timer and RTCM can be forwarded immediately.  The port is (re)opened
automatically, so unplugging the receiver no longer kills the node.

Writes (commands, RTCM corrections) can come from any thread; they are
serialised with a lock so a correction frame is never interleaved with a
command.
"""
import threading
from typing import Callable, Optional

import serial

READ_TIMEOUT_S = 0.05


class SerialLink:

    def __init__(self, device: str, baudrate: int, logger,
                 on_data: Callable[[bytes], None],
                 on_connect: Optional[Callable[[], None]] = None,
                 reconnect_s: float = 1.0):
        self.device = device
        self.reconnect_s = reconnect_s
        self.baudrate = baudrate
        self.log = logger
        self.on_data = on_data
        self.on_connect = on_connect
        self.bytes_read = 0
        self.bytes_written = 0
        self.connect_count = 0
        self._port: Optional[serial.Serial] = None
        self._write_lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    @property
    def is_open(self) -> bool:
        port = self._port
        return port is not None and port.is_open

    def start(self):
        self._thread = threading.Thread(target=self._run, name='gnss-serial', daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._close()

    def write(self, data: bytes) -> bool:
        port = self._port
        if port is None:
            return False
        with self._write_lock:
            try:
                port.write(data)
                self.bytes_written += len(data)
                return True
            except (serial.SerialException, OSError) as e:
                self.log.warn(f'Serial write to {self.device} failed: {e}',
                              throttle_duration_sec=5.0)
                return False

    def write_line(self, text: str) -> bool:
        return self.write((text + '\r\n').encode('ascii'))

    def _open(self) -> bool:
        try:
            port = serial.Serial(self.device, self.baudrate, timeout=READ_TIMEOUT_S)
            port.reset_input_buffer()
        except (serial.SerialException, OSError) as e:
            self.log.warn(f'Cannot open {self.device}: {e}', throttle_duration_sec=10.0)
            return False
        self._port = port
        self.connect_count += 1
        self.log.info(f'Opened {self.device} at {self.baudrate} baud')
        if self.on_connect is not None:
            try:
                self.on_connect()
            except Exception as e:
                self.log.error(f'Receiver set-up after connect failed: {e}')
        return True

    def _close(self):
        port, self._port = self._port, None
        if port is not None:
            try:
                port.close()
            except (serial.SerialException, OSError):
                pass

    def _run(self):
        while not self._stop.is_set():
            if self._port is None and not self._open():
                self._stop.wait(self.reconnect_s)
                continue
            try:
                data = self._port.read(self._port.in_waiting or 1)
            except (serial.SerialException, OSError, TypeError, AttributeError) as e:
                # TypeError/AttributeError: pyserial internals after a USB unplug
                self.log.error(f'Lost {self.device}: {e}; reconnecting')
                self._close()
                continue
            if data:
                self.bytes_read += len(data)
                try:
                    self.on_data(data)
                except Exception as e:  # never let a bad line stop the reader
                    self.log.error(f'Error handling receiver data: {e}',
                                   throttle_duration_sec=5.0)
