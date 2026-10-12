"""Ground-side redundant log (SPEC.md Section 4.6, copy 3): every frame crossing the link, both
directions, with ground timestamp and the bridge's RSSI/SNR, as a raw .bin plus a decoded TELEM
.csv and the GUI's event log as text. Independent of the vehicle's SD/flash copies.

The .bin is what gs.py --replay plays back (besides SD logs). Layout, little-endian:
    b"TVCG" | u16 version=1 | u16 n | n bytes of UTF-8 JSON metadata
    then per frame: f64 t_s | u8 direction (0 down, 1 up) | i8 rssi | i8 snr | u8 len | frame
t_s is seconds since the recording started; rssi/snr are -128 when there was no bridge trailer
(uplink, sim without one, ...). `frame` is the complete protocol frame, sync through CRC.

On SAFE the GUI calls finalize(), which renames the files to YYYYMMDD_HHMMSS_<mode>_<profile>
(SPEC Section 5) and starts a fresh recording for whatever happens next.
"""
from __future__ import annotations

import csv
import json
import struct
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import _repo  # noqa: F401
import telem_codec as tc

MAGIC = b"TVCG"
VERSION = 1
ENTRY_FMT = "<dBbbB"
ENTRY_SIZE = struct.calcsize(ENTRY_FMT)
NO_RADIO = -128
DOWN, UP = 0, 1

CSV_FIELDS = ["t_ground_s", "rssi_dbm", "snr_db", "t_ms", "state_name", "thrust_N", "xhat_p_deg",
              "xhat_y_deg", "u_p_deg", "u_y_deg", "enc_p_deg", "enc_y_deg", "gyro_z_deg_s",
              "K_p", "K_y", "flags", "cont_adc8", "batt_mv"]


@dataclass
class LoggedFrame:
    t: float
    direction: int
    rssi: int | None
    snr: int | None
    frame: bytes


def is_ground_log(path: Path) -> bool:
    with open(path, "rb") as f:
        return f.read(4) == MAGIC


def read_ground_log(path: Path) -> tuple[dict, list[LoggedFrame]]:
    data = Path(path).read_bytes()
    if data[:4] != MAGIC:
        raise ValueError(f"{path}: not a ground log (magic {data[:4]!r})")
    version, n = struct.unpack_from("<HH", data, 4)
    if version != VERSION:
        raise ValueError(f"{path}: ground log version {version}, expected {VERSION}")
    meta = json.loads(data[8:8 + n].decode("utf-8") or "{}")
    off = 8 + n
    out = []
    while off + ENTRY_SIZE <= len(data):
        t, direction, rssi, snr, flen = struct.unpack_from(ENTRY_FMT, data, off)
        off += ENTRY_SIZE
        if off + flen > len(data):
            break  # partial last entry (recording interrupted mid-write)
        out.append(LoggedFrame(t, direction, None if rssi == NO_RADIO else rssi,
                               None if snr == NO_RADIO else snr, data[off:off + flen]))
        off += flen
    return meta, out


class GroundRecorder:
    def __init__(self, log_dir: Path, link_description: str, tables: tc.Tables):
        self.log_dir = Path(log_dir)
        self.link_description = link_description
        self.tables = tables
        self._files = None
        self._lock = threading.Lock()  # frames arrive from the link thread and the GUI thread
        self.paths: list[Path] = []
        self.finalized: list[Path] = []
        self._open()

    def _open(self):
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.started_wall = datetime.now()
        self.t0 = time.monotonic()
        base = self.log_dir / f"_recording_{self.started_wall:%Y%m%d_%H%M%S}"
        self.paths = [base.with_suffix(".bin"), base.with_suffix(".csv"),
                      base.with_name(base.name + "_events.txt")]
        meta = json.dumps({"started": self.started_wall.isoformat(timespec="seconds"),
                           "link": self.link_description,
                           "local_params_hash": self.tables.local_hash_hex}).encode()
        binf = open(self.paths[0], "wb")
        binf.write(MAGIC + struct.pack("<HH", VERSION, len(meta)) + meta)
        csvf = open(self.paths[1], "w", newline="")
        writer = csv.writer(csvf)
        writer.writerow(CSV_FIELDS)
        evf = open(self.paths[2], "w")
        self._files = (binf, csvf, evf, writer)
        self._n_frames = 0

    def frame(self, direction: int, raw_frame: bytes, rssi=None, snr=None, decoded=None):
        with self._lock:
            self._frame(direction, raw_frame, rssi, snr, decoded)

    def _frame(self, direction, raw_frame, rssi, snr, decoded):
        if not self._files:
            return
        binf, csvf, _evf, writer = self._files
        self._n_frames += 1
        t = time.monotonic() - self.t0
        binf.write(struct.pack(ENTRY_FMT, t, direction,
                               NO_RADIO if rssi is None else rssi,
                               NO_RADIO if snr is None else snr, len(raw_frame)) + raw_frame)
        if direction == DOWN and decoded is not None and raw_frame[3] == tc.PacketType.TELEM:
            writer.writerow([f"{t:.3f}", "" if rssi is None else rssi, "" if snr is None else snr]
                            + [decoded.get(k, "") for k in CSV_FIELDS[3:]])

    def event_line(self, text: str):
        with self._lock:
            self._event_line(text)

    def _event_line(self, text: str):
        if self._files:
            self._files[2].write(f"{datetime.now():%H:%M:%S.%f}"[:-3] + f"  {text}\n")
            self._files[2].flush()

    def flush(self):
        with self._lock:
            self._flush()

    def _flush(self):
        if self._files:
            self._files[0].flush()
            self._files[1].flush()

    def _close(self):
        if self._files:
            for f in self._files[:3]:
                f.close()
            self._files = None

    def finalize(self, mode: str, profile: str) -> Path:
        """Close and rename to YYYYMMDD_HHMMSS_<mode>_<profile>, then start a new recording.
        Returns the finalized .bin path."""
        with self._lock:
            return self._finalize(mode, profile)

    def _finalize(self, mode, profile):
        self._close()
        stem = f"{self.started_wall:%Y%m%d_%H%M%S}_{mode}_{profile}"
        final = []
        for p, suffix in zip(self.paths, (".bin", ".csv", "_events.txt")):
            dest = self.log_dir / f"{stem}{suffix}"
            p.rename(dest)
            final.append(dest)
        self.finalized.append(final[0])
        self._open()
        return final[0]

    def close(self):
        """Program exit without a SAFE: keep what was recorded as <started>_unfinished.*, and
        drop the files outright if nothing was ever recorded into them."""
        with self._lock:
            self._close()
        stem = f"{self.started_wall:%Y%m%d_%H%M%S}_unfinished"
        for p, suffix in zip(self.paths, (".bin", ".csv", "_events.txt")):
            if not p.exists():
                continue
            if self._n_frames == 0:
                p.unlink()
            else:
                p.rename(self.log_dir / f"{stem}{suffix}")
