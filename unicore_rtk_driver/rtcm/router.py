"""Route RTCM between the receiver and the transports, and keep statistics.

Base:  receiver serial --frames--> RtcmRouter.publish() --> every sink
Rover: sources --bytes--> RtcmRouter.inject() --frames--> receiver serial

Both paths run on the thread that produced the data (serial reader or source
thread) and write directly to the outputs.

Incoming bytes are framed per source, and only complete RTCM frames with a
valid CRC are written to the receiver.  With several sources, frames from the
first listed source that delivered a frame within `failover_timeout_s` are
used.
"""
import threading
import time
from collections import Counter
from typing import Callable, Dict, List, Optional, Tuple

from unicore_rtk_driver.protocol.framer import FrameType, StreamFramer
from unicore_rtk_driver.rtcm import rtcm3
from unicore_rtk_driver.rtcm.transport import Sink, Source


class RtcmStats:

    def __init__(self):
        self._lock = threading.Lock()
        self.message_counts = Counter()
        self.last_time = 0.0
        self._window_start = time.monotonic()
        self._window_bytes = 0
        self.rate_bps = 0.0

    def add_frame(self, frame: bytes):
        with self._lock:
            self.message_counts[rtcm3.message_type(frame)] += 1
            self._window_bytes += len(frame)
            self.last_time = time.monotonic()

    def roll(self):
        """Update `rate_bps`; call periodically (e.g. from the diagnostics timer)."""
        with self._lock:
            now = time.monotonic()
            dt = now - self._window_start
            if dt > 0:
                self.rate_bps = self._window_bytes / dt
            self._window_start, self._window_bytes = now, 0

    def age(self) -> Optional[float]:
        return time.monotonic() - self.last_time if self.last_time else None


class RtcmRouter:

    def __init__(self, logger, failover_timeout_s: float = 5.0):
        self.log = logger
        self.sinks: List[Sink] = []
        self.sources: List[Source] = []
        self.outgoing = RtcmStats()
        self.incoming = RtcmStats()
        self.base_station: Optional[Tuple[int, float, float, float]] = None  # id, lat, lon, h
        self._inject: Optional[Callable[[bytes], None]] = None
        self._framers: Dict[int, StreamFramer] = {}
        self._last_frame: Dict[int, float] = {}
        self._inject_lock = threading.Lock()
        self.failover_timeout_s = failover_timeout_s
        self.active_source: Optional[int] = None

    # ------------------------------------------------------------- set-up
    def add_sink(self, sink: Sink):
        self.sinks.append(sink)

    def add_source(self, factory: Callable[[Callable[[bytes], None]], Source]):
        """`factory(on_data)` builds the source; its priority is its position."""
        index = len(self.sources)
        self.sources.append(factory(lambda data: self.inject(data, index)))

    def set_injector(self, fn: Callable[[bytes], None]):
        """Where incoming corrections go (the receiver's serial port)."""
        self._inject = fn

    def start(self):
        for t in self.sinks + self.sources:
            t.start()

    def stop(self):
        for t in self.sinks + self.sources:
            try:
                t.stop()
            except Exception as e:  # keep shutting the others down
                self.log.warn(f'RTCM {t.name}: error on stop: {e}')

    # --------------------------------------------------------- base side
    def publish(self, frame: bytes):
        """A complete RTCM frame produced by the local receiver."""
        self.outgoing.add_frame(frame)
        self._note_station(frame)
        for sink in self.sinks:
            sink.send(frame)

    # -------------------------------------------------------- rover side
    def inject(self, data: bytes, source_index: int = 0):
        """Correction bytes from source `source_index` (lower = preferred)."""
        with self._inject_lock:
            framer = self._framers.get(source_index)
            if framer is None:
                framer = self._framers[source_index] = StreamFramer(parse_text=False)
            frames = [f.data for f in framer.feed(data) if f.type is FrameType.RTCM]
            if not frames:
                return
            now = time.monotonic()
            self._last_frame[source_index] = now
            preferred = min(i for i, t in self._last_frame.items()
                            if now - t <= self.failover_timeout_s)
            if preferred != source_index:
                return
            if self.active_source != source_index:
                if self.active_source is not None:
                    self.log.warn(f'RTCM input switched to {self._source_name(source_index)}')
                self.active_source = source_index
            for frame in frames:
                if self._inject is not None:
                    self._inject(frame)
                self.incoming.add_frame(frame)
                self._note_station(frame)

    def _source_name(self, index):
        return self.sources[index].name if index < len(self.sources) else f'input {index}'

    def _note_station(self, frame: bytes):
        arp = rtcm3.station_arp_ecef(frame)
        if arp is None:
            return
        station_id, x, y, z = arp
        lat, lon, h = rtcm3.ecef_to_lla(x, y, z)
        first = self.base_station is None
        self.base_station = (station_id, lat, lon, h)
        if first:
            self.log.info(f'RTCM base station {station_id} position: '
                          f'{lat:.8f}, {lon:.8f}, {h:.3f} m (ellipsoid)')
