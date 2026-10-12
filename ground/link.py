"""Transports for the ground station (SPEC.md Section 1: "serial framing, packet codec"). All
framing, CRC, and payload decoding is protocol/telem_codec.py; this file only moves bytes and
timestamps what arrives. Each link runs its own reader thread and hands decoded packets to the
GUI through a queue (SPEC Section 5: redraw decoupled from the serial thread via a queue).

  SerialLink  live: Feather M0 bridge on USB CDC (downlink frames carry its RSSI/SNR trailer)
  TcpLink     --sim: sim/run_sim.py --serve on localhost, same bytes as the bridge produces
  ReplayLink  --replay: an SD-primary log (via sd_replay.py) or a ground log (recorder.py),
              played back at real time (x speed); commands are not sent anywhere

Link time (`now()`, and every RxPacket.t) is seconds since the link started for live/sim links,
and the position in the recording for replay, so the GUI never mixes the two clocks.
"""
from __future__ import annotations

import queue
import socket
import struct
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import _repo  # noqa: F401
import telem_codec as tc
from recorder import DOWN, UP, GroundRecorder


@dataclass
class RxPacket:
    t: float                 # link time, s
    ptype: int
    decoded: dict
    raw: bytes
    rssi: int | None = None
    snr: int | None = None
    direction: int = DOWN
    note: str = ""


@dataclass
class LinkNotice:
    t: float
    text: str
    level: str = "info"      # info / warn / error


@dataclass
class SentCommand:
    seq: int
    ptype: int
    value: int | None
    t: float                 # link time sent


class Link:
    kind = "none"
    can_send = False

    def __init__(self, tables: tc.Tables):
        self.tables = tables
        self.queue: queue.Queue = queue.Queue()
        self.recorder: GroundRecorder | None = None
        self.description = ""
        self.connected = False
        self._stop = threading.Event()
        self._seq = 0
        self._t0 = time.monotonic()

    def now(self) -> float:
        return time.monotonic() - self._t0

    def start(self):
        raise NotImplementedError

    def stop(self):
        self._stop.set()

    def notice(self, text: str, level: str = "info"):
        self.queue.put(LinkNotice(self.now(), text, level))

    def send(self, ptype: int, value: int | None = None) -> SentCommand | None:
        return None

    def _emit_frames(self, frames, direction=DOWN, t=None, note=""):
        for fr in frames:
            try:
                decoded = tc.decode(fr.ptype, fr.payload, self.tables)
            except (struct.error, ValueError) as exc:
                self.notice(f"undecodable {tc.packet_name(fr.ptype)} ({len(fr.payload)} B): {exc}",
                            "warn")
                continue
            if self.recorder:
                self.recorder.frame(direction, fr.raw, fr.rssi, fr.snr, decoded)
            self.queue.put(RxPacket(self.now() if t is None else t, fr.ptype, decoded, fr.raw,
                                    fr.rssi, fr.snr, direction, note))


class _StreamLink(Link):
    """A byte stream in both directions: downlink frames + bridge trailer in, bare frames out."""
    can_send = True
    reconnect_s = 1.0

    def __init__(self, tables: tc.Tables):
        super().__init__(tables)
        self.parser = tc.FrameParser(bridge_trailer=True)
        self._write_lock = threading.Lock()
        self._thread: threading.Thread | None = None

    # subclass hooks
    def _open(self):
        raise NotImplementedError

    def _read(self) -> bytes:
        raise NotImplementedError

    def _write(self, data: bytes):
        raise NotImplementedError

    def _close(self):
        pass

    def start(self):
        self._thread = threading.Thread(target=self._run, name=f"{self.kind}-rx", daemon=True)
        self._thread.start()

    def _run(self):
        warned = False
        while not self._stop.is_set():
            try:
                self._open()
            except OSError as exc:
                if not warned:
                    self.notice(f"{self.description}: cannot open ({exc}); retrying", "warn")
                    warned = True
                self._stop.wait(self.reconnect_s)
                continue
            warned = False
            self.connected = True
            self.parser = tc.FrameParser(bridge_trailer=True)
            self.notice(f"{self.description}: connected")
            try:
                while not self._stop.is_set():
                    data = self._read()
                    if data:
                        self._emit_frames(self.parser.feed(data))
            except (OSError, EOFError) as exc:
                if not self._stop.is_set():
                    self.notice(f"{self.description}: link dropped ({exc})", "error")
            finally:
                self.connected = False
                self._close()

    def send(self, ptype: int, value: int | None = None) -> SentCommand | None:
        if not self.connected:
            return None
        self._seq = (self._seq + 1) & 0xFFFF
        frame = tc.encode_frame(ptype, tc.encode_uplink(ptype, self._seq, value))
        try:
            with self._write_lock:
                self._write(frame)
        except OSError as exc:
            self.notice(f"{self.description}: write failed ({exc})", "error")
            return None
        if self.recorder:
            self.recorder.frame(UP, frame, decoded=tc.decode_uplink(ptype, frame[4:-2]))
        return SentCommand(self._seq, ptype, value, self.now())

    def stop(self):
        super().stop()
        if self._thread:
            self._thread.join(timeout=2.0)


class SerialLink(_StreamLink):
    """Live link through the Feather M0 bridge (SPEC 4.8). Needs pyserial."""
    kind = "serial"

    def __init__(self, tables: tc.Tables, port: str, baud: int = 115200):
        super().__init__(tables)
        self.port, self.baud = port, baud
        self.description = f"serial {port}"
        self._ser = None

    def _open(self):
        import serial
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=0.05)
        except serial.SerialException as exc:
            raise OSError(str(exc)) from exc

    def _read(self) -> bytes:
        import serial
        try:
            return self._ser.read(512)
        except serial.SerialException as exc:
            raise OSError(str(exc)) from exc

    def _write(self, data: bytes):
        import serial
        try:
            self._ser.write(data)
        except serial.SerialException as exc:
            raise OSError(str(exc)) from exc

    def _close(self):
        if self._ser:
            try:
                self._ser.close()
            except OSError:
                pass
            self._ser = None


def find_feather_ports() -> list[str]:
    """USB serial ports that look like an Adafruit board (VID 0x239A)."""
    try:
        from serial.tools import list_ports
    except ImportError:
        return []
    return [p.device for p in list_ports.comports() if p.vid == 0x239A]


class TcpLink(_StreamLink):
    """--sim: sim/run_sim.py --serve, which emits the bridge's exact byte stream."""
    kind = "sim"

    def __init__(self, tables: tc.Tables, host: str = "127.0.0.1", port: int = 5760):
        super().__init__(tables)
        self.host, self.port = host, port
        self.description = f"sim tcp://{host}:{port}"
        self._sock = None

    def _open(self):
        self._sock = socket.create_connection((self.host, self.port), timeout=1.0)
        self._sock.settimeout(0.1)

    def _read(self) -> bytes:
        try:
            data = self._sock.recv(4096)
        except socket.timeout:
            return b""
        if not data:
            raise EOFError("sim closed the connection")
        return data

    def _write(self, data: bytes):
        self._sock.sendall(data)

    def _close(self):
        if self._sock:
            self._sock.close()
            self._sock = None


@dataclass
class _ReplayItem:
    t: float
    frame: bytes
    rssi: int | None = None
    snr: int | None = None
    direction: int = DOWN
    note: str = ""


class ReplayLink(Link):
    """Plays a recording back at real time (x speed). Every frame still goes through
    FrameParser + telem_codec.decode, exactly as bytes off the serial port would."""
    kind = "replay"

    def __init__(self, tables: tc.Tables, path: Path, speed: float = 1.0, telem_hz: float = 10.0):
        super().__init__(tables)
        self.path = Path(path)
        self.speed = speed
        self.items: list[_ReplayItem] = []
        self.meta: dict = {}
        self.finished = False
        self._pos = 0.0
        self._thread = None
        self._load(telem_hz)

    def _load(self, telem_hz: float):
        import recorder
        import sd_replay
        if sd_replay.is_sd_log(self.path):
            header, frames = sd_replay.build_frames(self.path, telem_hz=telem_hz,
                                                    tables=self.tables)
            self.items = [_ReplayItem(f.t, f.frame, note=f.note) for f in frames]
            self.meta = {"source": "sd", **header}
            self.description = (f"replay SD {self.path.name} ({tc.mode_name(header['mode'])}/"
                                f"{tc.profile_name(header['profile'])})")
        elif recorder.is_ground_log(self.path):
            meta, frames = recorder.read_ground_log(self.path)
            self.items = [_ReplayItem(f.t, f.frame, f.rssi, f.snr, f.direction) for f in frames]
            self.meta = {"source": "ground", **meta}
            self.description = f"replay ground log {self.path.name}"
        else:
            raise ValueError(f"{self.path}: neither an SD log (TVCL) nor a ground log (TVCG)")
        if not self.items:
            raise ValueError(f"{self.path}: nothing to replay")
        self.duration = self.items[-1].t

    def now(self) -> float:
        return self._pos

    def start(self):
        self._thread = threading.Thread(target=self._run, name="replay", daemon=True)
        self._thread.start()

    def _run(self):
        self.connected = True
        self.notice(f"{self.description}: {self.duration:.1f} s at x{self.speed:g}")
        down = tc.FrameParser(bridge_trailer=False)
        up = tc.FrameParser(bridge_trailer=False)
        wall0 = time.monotonic()
        for item in self.items:
            while not self._stop.is_set():
                self._pos = (time.monotonic() - wall0) * self.speed
                wait = (item.t - self._pos) / self.speed
                if wait <= 0:
                    break
                self._stop.wait(min(wait, 0.02))
            if self._stop.is_set():
                return
            self._pos = item.t
            parser = up if item.direction == UP else down
            frames = parser.feed(item.frame)
            for fr in frames:
                fr.rssi, fr.snr = item.rssi, item.snr
            self._emit_frames(frames, item.direction, t=item.t, note=item.note)
        # The clock stops at the last frame, so the plots hold the end of the run in view.
        self.finished = True
        self.notice(f"replay finished ({self.duration:.1f} s)")

    def stop(self):
        super().stop()
        if self._thread:
            self._thread.join(timeout=2.0)
