"""Python side of protocol/protocol.h: framing, CRC, and every payload's encode/decode (SPEC.md
Section 4.7). The one place Python parses or builds telemetry frames; tools/telem_cli.py,
ground/ (GUI, link, recorder, replay), and sim/run_sim.py --serve all import it rather than
keeping their own struct formats. ground/tests/test_telem_codec.py compiles protocol.h and
checks every struct format and the CRC here against it byte for byte.

Frame: 0xAA 0x55 | len | type | payload[len] | crc16 (LE), crc16 = CRC-16/CCITT-FALSE over
len ++ type ++ payload. Downlink frames that came through the Feather bridge carry 2 more bytes
after the CRC (RSSI, SNR, both int8), outside the frame; FrameParser(bridge_trailer=True)
handles that.

Units: decode_*() return engineering units (deg, N, deg/s, gain 0..1); encode_*() take the
same and apply the wire scaling with the firmware's own clamping (telemetry.cpp's
clampToI16/clampToU16: saturate, then truncate toward zero; NaN goes out as 0, which is what
the Teensy's float->int conversion produces for NaN).
"""
from __future__ import annotations

import hashlib
import json
import math
import re
import struct
from dataclasses import dataclass, field
from enum import IntEnum
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent

SYNC0, SYNC1 = 0xAA, 0x55
SYNC = bytes([SYNC0, SYNC1])
HEADER_LEN = 4  # sync0, sync1, len, type
CRC_LEN = 2
MAX_PAYLOAD = 48
BRIDGE_TRAILER_LEN = 2  # rssi, snr -- appended by feather_bridge/main.cpp, downlink only

# TELEM.t_ms before any ignition is scheduled (sequencer.cpp's telemetryTMs()).
T_MS_NONE = -32768


class PacketType(IntEnum):
    TELEM = 1
    STATUS = 2
    EVENT = 3
    ACK = 4
    HEARTBEAT = 0x10
    SET_MODE = 0x11
    SET_PROFILE = 0x12
    TARE = 0x13
    ARM = 0x14
    DISARM = 0x15
    COUNTDOWN = 0x16
    FIRE_CONFIRM = 0x17
    ABORT = 0x18
    DRY_RUN = 0x19
    PING = 0x1A
    DUMP_FLASH = 0x1B


DOWNLINK_TYPES = {PacketType.TELEM, PacketType.STATUS, PacketType.EVENT, PacketType.ACK}


class AckResult(IntEnum):
    OK = 0
    REJECTED_STATE = 1
    REJECTED_ARM_SWITCH = 2
    REJECTED_MEASURE = 3
    REJECTED_TOKEN = 4
    REJECTED_ARG = 5
    REJECTED_CONTINUITY = 6
    UNSUPPORTED = 7
    REJECTED_HEARTBEAT = 8


# tvc::flight::State / Mode / Profile (firmware/src/flight/record.h). Small, stable enums, not
# code-generated; this is the one Python copy (tools/decode_log.py imports these too).
STATE_NAMES = ["BOOT", "SELFTEST", "IDLE", "ARMED", "COUNTDOWN", "IGNITE", "BURN", "TAILOFF",
               "SAFE", "FAULT"]
MODE_NAMES = ["SIM_ONBOARD", "HIL", "STAND_OPEN", "STAND_CLOSED", "FLIGHT"]
PROFILE_NAMES = ["HOLD", "STEP_X", "STEP_Y", "RAMP_X", "CHIRP_X"]

# record.h's fault_bits namespace, bit for bit (STATUS.fault_bits and Record.fault_bits).
FAULT_BITS = [
    (1 << 0, "MEASURE_MISSING"),
    (1 << 1, "FRAME_DET_INVALID"),
    (1 << 2, "IMU"),
    (1 << 3, "SD"),
    (1 << 4, "FLASH"),
    (1 << 5, "ENCODER"),
    (1 << 6, "SERVO_SELFTEST"),
    (1 << 7, "LOOP_OVERRUN"),
    (1 << 8, "SERVO_RATE"),
    (1 << 9, "ARM_SWITCH"),
    (1 << 10, "CONTINUITY"),
    (1 << 11, "NO_IGNITION"),
    (1 << 12, "HX711"),
]

# protocol.h telem_flags.
FLAG_PITCH_GATE_OK = 1 << 0
FLAG_YAW_GATE_OK = 1 << 1
FLAG_PITCH_SATURATED = 1 << 2
FLAG_YAW_SATURATED = 1 << 3
FLAG_TVC_ACTIVE = 1 << 4
FLAG_DRY_RUN = 1 << 5
FLAG_ARM_SWITCH_CLOSED = 1 << 6
TELEM_FLAG_NAMES = [
    (FLAG_PITCH_GATE_OK, "pitch_gate_ok"),
    (FLAG_YAW_GATE_OK, "yaw_gate_ok"),
    (FLAG_PITCH_SATURATED, "pitch_saturated"),
    (FLAG_YAW_SATURATED, "yaw_saturated"),
    (FLAG_TVC_ACTIVE, "tvc_active"),
    (FLAG_DRY_RUN, "dry_run"),
    (FLAG_ARM_SWITCH_CLOSED, "arm_switch_closed"),
]

# protocol.h's exact field order/types. '<' = little-endian, no padding (#pragma pack(1)).
TELEM_FMT = "<hBhhhhhhhhHHBBH"
TELEM_FIELDS = ["t_ms", "state", "thrust_x10", "xhat_p_x100", "xhat_y_x100", "u_p_x100",
                "u_y_x100", "enc_p_x100", "enc_y_x100", "gyro_z_x10", "k_p_x1000", "k_y_x1000",
                "flags", "cont_adc8", "batt_mv"]
STATUS_FMT = "<BH6sIIbHIHBBHHHHH4s"
STATUS_FIELDS = ["mode", "fault_bits", "params_hash6", "sd_bytes_written", "flash_bytes_written",
                 "rssi_last_uplink_dbm", "boot_count", "missing_measure_bits",
                 "loop_overrun_count", "sat_pct_p", "sat_pct_y", "running_iae_p_x100",
                 "running_iae_y_x100", "peak_thrust_x10", "peak_tilt_p_x100",
                 "peak_tilt_y_x100", "fw_git_hash"]
EVENT_FMT = "<hBh"
ACK_FMT = "<HBBH"
UPLINK_FMT = "<H"          # seq
UPLINK_U8_FMT = "<HB"      # seq, value
FIRE_CONFIRM_FMT = "<HH"   # seq, token

UPLINK_U8_TYPES = {PacketType.SET_MODE, PacketType.SET_PROFILE, PacketType.DRY_RUN}

PAYLOAD_SIZES = {
    PacketType.TELEM: struct.calcsize(TELEM_FMT),
    PacketType.STATUS: struct.calcsize(STATUS_FMT),
    PacketType.EVENT: struct.calcsize(EVENT_FMT),
    PacketType.ACK: struct.calcsize(ACK_FMT),
}
assert PAYLOAD_SIZES[PacketType.TELEM] == 27
assert PAYLOAD_SIZES[PacketType.STATUS] == 42
assert PAYLOAD_SIZES[PacketType.EVENT] == 5
assert PAYLOAD_SIZES[PacketType.ACK] == 6


# ---- CRC ------------------------------------------------------------------------------------

def _make_crc_table():
    table = []
    for byte in range(256):
        crc = byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
        table.append(crc)
    return table


_CRC_TABLE = _make_crc_table()


def crc16_ccitt_false(data: bytes) -> int:
    """CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF): protocol.h's crc16() and record.h's."""
    crc = 0xFFFF
    for b in data:
        crc = ((crc << 8) & 0xFFFF) ^ _CRC_TABLE[((crc >> 8) ^ b) & 0xFF]
    return crc


# ---- framing --------------------------------------------------------------------------------

def encode_frame(ptype: int, payload: bytes) -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"payload {len(payload)} B exceeds MAX_PAYLOAD {MAX_PAYLOAD}")
    body = bytes([len(payload), int(ptype)]) + payload
    crc = crc16_ccitt_false(body)
    return SYNC + body + bytes([crc & 0xFF, crc >> 8])


def bridge_trailer(rssi_dbm: int, snr_db: int) -> bytes:
    """The 2 bytes feather_bridge/main.cpp appends after a relayed downlink frame."""
    return struct.pack("<bb", max(-128, min(127, int(rssi_dbm))), max(-128, min(127, int(snr_db))))


@dataclass
class Frame:
    ptype: int
    payload: bytes
    rssi: int | None = None  # bridge trailer, downlink only
    snr: int | None = None
    raw: bytes = b""         # the frame as it appeared on the wire, without the bridge trailer


@dataclass
class FrameParser:
    """Incremental parser: feed() arbitrary byte chunks, get complete CRC-valid frames back.
    Resyncs on 0xAA 0x55; a CRC failure or impossible length drops one byte and rescans, so a
    corrupted frame never swallows the good one behind it."""
    bridge_trailer: bool = False
    crc_errors: int = 0
    bytes_dropped: int = 0
    _buf: bytearray = field(default_factory=bytearray)

    def feed(self, data: bytes) -> list[Frame]:
        buf = self._buf
        buf += data
        out: list[Frame] = []
        trailer = BRIDGE_TRAILER_LEN if self.bridge_trailer else 0
        while True:
            i = buf.find(SYNC)
            if i < 0:
                keep = 1 if buf.endswith(bytes([SYNC0])) else 0
                self.bytes_dropped += len(buf) - keep
                del buf[:len(buf) - keep]
                break
            if i:
                self.bytes_dropped += i
                del buf[:i]
            if len(buf) < HEADER_LEN:
                break
            plen = buf[2]
            if plen > MAX_PAYLOAD:
                self.bytes_dropped += 1
                del buf[:1]
                continue
            flen = HEADER_LEN + plen + CRC_LEN
            if len(buf) < flen + trailer:
                break
            crc_recv = buf[4 + plen] | (buf[5 + plen] << 8)
            if crc16_ccitt_false(bytes(buf[2:4 + plen])) != crc_recv:
                self.crc_errors += 1
                self.bytes_dropped += 1
                del buf[:1]
                continue
            frame = Frame(ptype=buf[3], payload=bytes(buf[4:4 + plen]), raw=bytes(buf[:flen]))
            if trailer:
                frame.rssi, frame.snr = struct.unpack_from("<bb", buf, flen)
            out.append(frame)
            del buf[:flen + trailer]
        return out


# ---- name tables (generated JSON, loaded once) ----------------------------------------------

def _load_json(path: Path) -> dict:
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return {}


@dataclass
class Tables:
    event_names: dict[int, str]
    measure_bit_names: dict[int, str]
    local_hash_hex: str  # "" if neither sim/tvc_params.py nor params.yaml is readable

    @classmethod
    def load(cls, repo_root: Path = REPO_ROOT) -> "Tables":
        events = _load_json(repo_root / "protocol" / "event_codes.json")
        measure = _load_json(repo_root / "core" / "measure_bits.json")
        return cls(event_names={int(k): v for k, v in events.items()},
                   measure_bit_names={v: k for k, v in measure.items()},
                   local_hash_hex=local_params_hash(repo_root))

    @property
    def local_hash6(self) -> bytes:
        return bytes.fromhex(self.local_hash_hex[:12]) if self.local_hash_hex else b""

    def event_name(self, code: int) -> str:
        return self.event_names.get(code, f"code_{code}")

    def event_code(self, name: str) -> int:
        for code, n in self.event_names.items():
            if n == name:
                return code
        raise KeyError(name)


def local_params_hash(repo_root: Path = REPO_ROOT) -> str:
    """SPEC.md Section 5: the hash stamped into the GUI's own sim/tvc_params.py. Falls back to
    SHA-256(params.yaml) -- exactly what gen_params.py would stamp -- if that file hasn't been
    generated yet."""
    try:
        m = re.search(r'^PARAMS_HASH = "([0-9a-f]{64})"',
                      (repo_root / "sim" / "tvc_params.py").read_text(), re.M)
        if m:
            return m.group(1)
    except OSError:
        pass
    try:
        return hashlib.sha256((repo_root / "params.yaml").read_bytes()).hexdigest()
    except OSError:
        return ""


def fault_names(bits: int) -> list[str]:
    return [name for bit, name in FAULT_BITS if bits & bit]


def _name(table: list[str], i: int) -> str:
    return table[i] if 0 <= i < len(table) else f"?{i}"


def state_name(i: int) -> str:
    return _name(STATE_NAMES, i)


def mode_name(i: int) -> str:
    return _name(MODE_NAMES, i)


def profile_name(i: int) -> str:
    return _name(PROFILE_NAMES, i)


# ---- decode ---------------------------------------------------------------------------------

def decode_telem(payload: bytes) -> dict:
    d = dict(zip(TELEM_FIELDS, struct.unpack(TELEM_FMT, payload)))
    d["state_name"] = state_name(d["state"])
    d["thrust_N"] = d.pop("thrust_x10") / 10.0
    for k in ("xhat_p", "xhat_y", "u_p", "u_y", "enc_p", "enc_y"):
        d[f"{k}_deg"] = d.pop(f"{k}_x100") / 100.0
    d["gyro_z_deg_s"] = d.pop("gyro_z_x10") / 10.0
    d["K_p"] = d.pop("k_p_x1000") / 1000.0
    d["K_y"] = d.pop("k_y_x1000") / 1000.0
    d["flags_decoded"] = [name for bit, name in TELEM_FLAG_NAMES if d["flags"] & bit]
    return d


def decode_status(payload: bytes, tables: Tables | None = None) -> dict:
    d = dict(zip(STATUS_FIELDS, struct.unpack(STATUS_FMT, payload)))
    d["mode_name"] = mode_name(d["mode"])
    d["fault_names"] = fault_names(d["fault_bits"])
    h6 = d.pop("params_hash6")
    d["params_hash6_hex"] = h6.hex()
    local6 = tables.local_hash6 if tables else b""
    d["params_hash_match"] = (local6 == h6) if local6 else None
    d["fw_git_hash_hex"] = d.pop("fw_git_hash").hex()
    bits = d["missing_measure_bits"]
    names = tables.measure_bit_names if tables else {}
    d["missing_measure_names"] = [names.get(i, f"bit{i}") for i in range(32) if bits & (1 << i)]
    d["running_iae_p_degs"] = d.pop("running_iae_p_x100") / 100.0
    d["running_iae_y_degs"] = d.pop("running_iae_y_x100") / 100.0
    d["peak_thrust_N"] = d.pop("peak_thrust_x10") / 10.0
    d["peak_tilt_p_deg"] = d.pop("peak_tilt_p_x100") / 100.0
    d["peak_tilt_y_deg"] = d.pop("peak_tilt_y_x100") / 100.0
    return d


def decode_event(payload: bytes, tables: Tables | None = None) -> dict:
    t_ms, code, aux = struct.unpack(EVENT_FMT, payload)
    name = tables.event_name(code) if tables else f"code_{code}"
    return {"t_ms": t_ms, "event_code": code, "aux": aux, "event_name": name}


def decode_ack(payload: bytes) -> dict:
    seq, cmd_type, result, aux = struct.unpack(ACK_FMT, payload)
    try:
        cmd_name = PacketType(cmd_type).name
    except ValueError:
        cmd_name = f"type_{cmd_type}"
    try:
        result_name = AckResult(result).name
    except ValueError:
        result_name = f"result_{result}"
    return {"seq": seq, "cmd_type": cmd_type, "cmd_name": cmd_name, "result": result,
            "result_name": result_name, "aux": aux}


def decode_uplink(ptype: int, payload: bytes) -> dict:
    ptype = PacketType(ptype)
    if ptype in UPLINK_U8_TYPES:
        seq, value = struct.unpack(UPLINK_U8_FMT, payload)
        return {"seq": seq, "value": value}
    if ptype == PacketType.FIRE_CONFIRM:
        seq, token = struct.unpack(FIRE_CONFIRM_FMT, payload)
        return {"seq": seq, "token": token}
    (seq,) = struct.unpack(UPLINK_FMT, payload)
    return {"seq": seq}


def decode(ptype: int, payload: bytes, tables: Tables | None = None) -> dict:
    """Decode any frame's payload by type. Raises struct.error on a size mismatch and
    ValueError on an unknown type."""
    if ptype == PacketType.TELEM:
        return decode_telem(payload)
    if ptype == PacketType.STATUS:
        return decode_status(payload, tables)
    if ptype == PacketType.EVENT:
        return decode_event(payload, tables)
    if ptype == PacketType.ACK:
        return decode_ack(payload)
    return decode_uplink(ptype, payload)


def packet_name(ptype: int) -> str:
    try:
        return PacketType(ptype).name
    except ValueError:
        return f"type_{ptype}"


# ---- encode ---------------------------------------------------------------------------------

def _trunc(v: float) -> int:
    return 0 if (v is None or math.isnan(v)) else int(v)  # int() truncates toward zero


def to_i16(v: float) -> int:
    """telemetry.cpp clampToI16()."""
    if v is not None and not math.isnan(v):
        if v > 32767.0:
            return 32767
        if v < -32768.0:
            return -32768
    return _trunc(v)


def to_u16(v: float) -> int:
    """telemetry.cpp clampToU16()."""
    if v is not None and not math.isnan(v):
        if v > 65535.0:
            return 65535
        if v < 0.0:
            return 0
    return _trunc(v)


def encode_telem(*, t_ms: int, state: int, thrust_N: float, xhat_p_deg: float,
                 xhat_y_deg: float, u_p_deg: float, u_y_deg: float, enc_p_deg: float,
                 enc_y_deg: float, gyro_z_deg_s: float, K_p: float, K_y: float, flags: int,
                 cont_adc8: int, batt_mv: int) -> bytes:
    return struct.pack(
        TELEM_FMT, to_i16(t_ms), state, to_i16(thrust_N * 10.0), to_i16(xhat_p_deg * 100.0),
        to_i16(xhat_y_deg * 100.0), to_i16(u_p_deg * 100.0), to_i16(u_y_deg * 100.0),
        to_i16(enc_p_deg * 100.0), to_i16(enc_y_deg * 100.0), to_i16(gyro_z_deg_s * 10.0),
        to_u16(K_p * 1000.0), to_u16(K_y * 1000.0), flags & 0xFF, cont_adc8 & 0xFF,
        to_u16(batt_mv))


def encode_status(*, mode: int, fault_bits: int, params_hash6: bytes, sd_bytes_written: int,
                  flash_bytes_written: int, rssi_last_uplink_dbm: int, boot_count: int,
                  missing_measure_bits: int, loop_overrun_count: int, sat_pct_p: int,
                  sat_pct_y: int, running_iae_p_degs: float, running_iae_y_degs: float,
                  peak_thrust_N: float, peak_tilt_p_deg: float, peak_tilt_y_deg: float,
                  fw_git_hash4: bytes) -> bytes:
    return struct.pack(
        STATUS_FMT, mode, fault_bits & 0xFFFF, bytes(params_hash6[:6]).ljust(6, b"\0"),
        sd_bytes_written & 0xFFFFFFFF, flash_bytes_written & 0xFFFFFFFF,
        max(-128, min(127, rssi_last_uplink_dbm)), boot_count & 0xFFFF,
        missing_measure_bits & 0xFFFFFFFF, min(loop_overrun_count, 0xFFFF), sat_pct_p, sat_pct_y,
        to_u16(running_iae_p_degs * 100.0), to_u16(running_iae_y_degs * 100.0),
        to_u16(peak_thrust_N * 10.0), to_u16(peak_tilt_p_deg * 100.0),
        to_u16(peak_tilt_y_deg * 100.0), bytes(fw_git_hash4[:4]).ljust(4, b"\0"))


def encode_event(t_ms: int, event_code: int, aux: int = 0) -> bytes:
    return struct.pack(EVENT_FMT, to_i16(t_ms), event_code, to_i16(aux))


def encode_ack(seq: int, cmd_type: int, result: int, aux: int = 0) -> bytes:
    return struct.pack(ACK_FMT, seq & 0xFFFF, int(cmd_type), int(result), aux & 0xFFFF)


def encode_uplink(ptype: int, seq: int, value: int | None = None) -> bytes:
    """Payload for an uplink command. value: SET_MODE/SET_PROFILE/DRY_RUN's byte, or
    FIRE_CONFIRM's token; unused for the rest."""
    ptype = PacketType(ptype)
    if ptype in DOWNLINK_TYPES:
        raise ValueError(f"{ptype.name} is a downlink type")
    seq &= 0xFFFF
    if ptype in UPLINK_U8_TYPES:
        return struct.pack(UPLINK_U8_FMT, seq, int(value) & 0xFF)
    if ptype == PacketType.FIRE_CONFIRM:
        return struct.pack(FIRE_CONFIRM_FMT, seq, int(value) & 0xFFFF)
    return struct.pack(UPLINK_FMT, seq)


def hex_prefix_bytes(hex_str: str, n: int) -> bytes:
    """telemetry.cpp hexPrefixToBytes(): first n bytes of a hex string, a non-hex (or missing)
    nibble decoding as 0."""
    def nib(i):
        c = hex_str[i] if i < len(hex_str) else ""
        return int(c, 16) if c and c in "0123456789abcdefABCDEF" else 0
    return bytes((nib(2 * i) << 4) | nib(2 * i + 1) for i in range(n))
