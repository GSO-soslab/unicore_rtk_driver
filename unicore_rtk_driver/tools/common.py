"""Logging for the command line tools, with the rclpy-like interface the shared modules use."""
import logging
import threading
import time


class Logger:
    """Python logging with warn() and throttle_duration_sec, as the rclpy logger."""

    def __init__(self, name):
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
