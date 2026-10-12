"""SD-primary log -> the downlink frame stream the vehicle would have radioed during that run,
for gs.py --replay. Records are parsed by tools/decode_log.py (one SD parser, not two); frames
are built with protocol/telem_codec.py and handed to the GUI as bytes, so a replay goes through
the same FrameParser/decode path as a live link.

The schedule mirrors firmware/src/flight/telemetry.cpp + sequencer.cpp: TELEM every 1/telem_hz,
STATUS every 1/status_hz, an EVENT (sent 3x: now, +150 ms, +300 ms) on every state change and
on each countdown beep, t_ms per sequencer::telemetryTMs(). What a Record doesn't carry is
derived or left at the firmware's "unknown" value, and says so where it happens:
  - gate-ok flags: |a| from the logged accel against params kalman.accel_gate_g (the same test
    imu::accelGateOk() makes; both axes share it, as in the firmware)
  - saturated flags / sat %: |u_cmd| at the deflection limit (Record has no u_raw)
  - arm switch: closed for the whole run if it ever reached ARMED (it had to be, to arm);
    unknown otherwise, sent as open
  - dry-run flag, battery, uplink RSSI: not logged -> 0 / 0 / -128, the firmware's own
    "not available" values
  - SAFE: sequencer.cpp closes the SD file on TAILOFF -> SAFE, so a log that ends in TAILOFF
    gets that final transition appended (tagged as inferred)
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

import _repo  # noqa: F401
import decode_log
import telem_codec as tc

COUNTDOWN_MS = 10000  # sequencer.cpp kCountdownMs
EVENT_RETX_S = (0.0, 0.150, 0.300)
LOOP_OVERRUN_US = 3300
FLASH_DECIMATION = 6  # 300 Hz SD records -> 50 Hz LittleFS mirror (params logging.*)


@dataclass
class ReplayFrame:
    t: float          # seconds since the first record
    frame: bytes      # one complete protocol frame (no bridge trailer: SD logs have no radio)
    note: str = ""    # shown in the GUI event log next to what this frame decodes to


def is_sd_log(path: Path) -> bool:
    with open(path, "rb") as f:
        head = f.read(4)
    return len(head) == 4 and int.from_bytes(head, "little") == decode_log.HEADER_MAGIC


def _gate_band():
    try:
        import tvc_params
        lo, hi = tvc_params.kalman.accel_gate_g
        return float(lo), float(hi), float(tvc_params.control.max_deflection_deg)
    except (ImportError, AttributeError):
        return 0.6, 1.4, 10.0


def _missing_measure_bits(header_hash: str, tables: tc.Tables) -> int:
    """The vehicle computes this from the params it was built with. Only knowable here if the
    log's params hash is this checkout's; otherwise report none rather than guess."""
    if not tables.local_hash_hex or header_hash != tables.local_hash_hex:
        return 0
    try:
        import tvc_params
    except ImportError:
        return 0
    bits = 0
    for bit, name in tables.measure_bit_names.items():
        section, key = name.split(".", 1)
        value = getattr(getattr(tvc_params, section, None), key, 0.0)
        if isinstance(value, float) and math.isnan(value):
            bits |= 1 << bit
    return bits


def build_frames(path: Path, telem_hz: float = 10.0, status_hz: float = 1.0,
                 tables: tc.Tables | None = None) -> tuple[dict, list[ReplayFrame]]:
    """Returns (header, frames sorted by t). Raises decode_log.LogFormatError on a bad file."""
    tables = tables or tc.Tables.load()
    raw = Path(path).read_bytes()
    header = decode_log.load_header(raw)
    r, crc_fail, crc_ok = decode_log.load_records(raw)
    r = r[crc_ok]  # a record that fails its CRC is not trusted for anything
    header["crc_failures"] = crc_fail
    if len(r) == 0:
        raise decode_log.LogFormatError("no CRC-valid records")

    # micros() wraps every ~71.6 min; unwrap before converting.
    t_us = np.concatenate([[0], np.cumsum(np.diff(r["t_us"].astype(np.int64)) % (1 << 32))])
    t = t_us / 1e6
    state = r["state"].astype(int)
    S = {name: i for i, name in enumerate(tc.STATE_NAMES)}

    gate_lo, gate_hi, max_defl = _gate_band()
    a_g = np.sqrt(r["ax"].astype(float) ** 2 + r["ay"].astype(float) ** 2 +
                  r["az"].astype(float) ** 2) / decode_log.ACCEL_LSB_PER_G
    gate_ok = (a_g >= gate_lo) & (a_g <= gate_hi)
    sat_p = np.abs(r["u_p"]) >= max_defl
    sat_y = np.abs(r["u_y"]) >= max_defl
    ever_armed = bool(np.any(state == S["ARMED"]))
    hash6 = tc.hex_prefix_bytes(header["params_hash"], 6)
    git4 = tc.hex_prefix_bytes(header["git_hash"], 4)
    missing_bits = _missing_measure_bits(header["params_hash"], tables)
    ev = tables.event_code

    frames: list[ReplayFrame] = []

    def event(t_s, t_ms, name, aux=0, note=""):
        payload = tc.encode_event(t_ms, ev(name), aux)
        for k, dt in enumerate(EVENT_RETX_S):
            frames.append(ReplayFrame(t_s + dt, tc.encode_frame(tc.PacketType.EVENT, payload),
                                      note if k == 0 else ""))

    t_countdown = t_ignite = None
    last_beep = -1e9
    next_telem = next_status = 0.0
    prev_state = None
    iae_p = iae_y = 0.0
    peak_tilt_p = peak_tilt_y = 0.0
    peak_thrust = 0.0
    ctrl_ticks = sat_ticks_p = sat_ticks_y = 0
    overruns = 0
    dt_rec = 1.0 / 300.0

    def t_ms_at(i):
        s = state[i]
        if s == S["COUNTDOWN"] and t_countdown is not None:
            return -(COUNTDOWN_MS - int((t[i] - t_countdown) * 1000))
        if t_ignite is not None and s in (S["IGNITE"], S["BURN"], S["TAILOFF"], S["SAFE"],
                                          S["FAULT"]):
            return int((t[i] - t_ignite) * 1000)
        return tc.T_MS_NONE

    def telem(i, st=None):
        s = state[i] if st is None else st
        flags = 0
        if gate_ok[i]:
            flags |= tc.FLAG_PITCH_GATE_OK | tc.FLAG_YAW_GATE_OK
        if sat_p[i]:
            flags |= tc.FLAG_PITCH_SATURATED
        if sat_y[i]:
            flags |= tc.FLAG_YAW_SATURATED
        if s == S["BURN"]:
            flags |= tc.FLAG_TVC_ACTIVE
        if ever_armed:
            flags |= tc.FLAG_ARM_SWITCH_CLOSED
        return tc.encode_frame(tc.PacketType.TELEM, tc.encode_telem(
            t_ms=t_ms_at(i), state=int(s), thrust_N=float(r["thrust_N"][i]),
            xhat_p_deg=float(r["xhat_p"][i]), xhat_y_deg=float(r["xhat_y"][i]),
            u_p_deg=float(r["u_p"][i]), u_y_deg=float(r["u_y"][i]),
            enc_p_deg=float(r["enc_p"][i]), enc_y_deg=float(r["enc_y"][i]),
            gyro_z_deg_s=float(r["gz"][i]) / decode_log.GYRO_LSB_PER_DPS,
            K_p=float(r["K_p"][i]), K_y=float(r["K_y"][i]), flags=flags,
            cont_adc8=int(r["cont_adc8"][i]), batt_mv=0))

    def status(i):
        pct = lambda n: int(100 * n / ctrl_ticks) if ctrl_ticks else 0  # noqa: E731
        return tc.encode_frame(tc.PacketType.STATUS, tc.encode_status(
            mode=int(r["mode"][i]), fault_bits=int(r["fault_bits"][i]), params_hash6=hash6,
            sd_bytes_written=decode_log.HEADER_SIZE + decode_log.RECORD_SIZE * (i + 1),
            flash_bytes_written=decode_log.RECORD_SIZE * ((i + 1) // FLASH_DECIMATION),
            rssi_last_uplink_dbm=-128, boot_count=header["boot_count"],
            missing_measure_bits=missing_bits, loop_overrun_count=overruns,
            sat_pct_p=pct(sat_ticks_p), sat_pct_y=pct(sat_ticks_y), running_iae_p_degs=iae_p,
            running_iae_y_degs=iae_y, peak_thrust_N=peak_thrust, peak_tilt_p_deg=peak_tilt_p,
            peak_tilt_y_deg=peak_tilt_y, fw_git_hash4=git4))

    for i in range(len(r)):
        s = int(state[i])
        if s != prev_state:
            if s == S["COUNTDOWN"]:
                t_countdown = t[i]
            if s == S["IGNITE"]:
                t_ignite = t[i]
            if s == S["ARMED"]:
                peak_thrust = 0.0  # loadcell::resetPeak() in cmdArm()
            note = "log starts here" if prev_state is None else ""
            name = tc.STATE_NAMES[s]
            if name not in ("BOOT", "SELFTEST"):
                event(t[i], t_ms_at(i), f"STATE_{name}", note=note)
            prev_state = s
        if s == S["COUNTDOWN"]:
            el_ms = int((t[i] - t_countdown) * 1000)
            if el_ms < COUNTDOWN_MS:
                rem_s = (COUNTDOWN_MS - el_ms) // 1000
                if (t[i] - last_beep) * 1000 > (250 if rem_s < 3 else 1000):
                    last_beep = t[i]
                    event(t[i], t_ms_at(i), "COUNTDOWN_TICK", rem_s + 1)

        # STATUS cumulative fields, as telemetry.cpp's onControlTick() integrates them.
        if r["loop_us"][i] > LOOP_OVERRUN_US:
            overruns += 1
        if i % 2 == 1:
            ctrl_ticks += 1
            sat_ticks_p += int(sat_p[i])
            sat_ticks_y += int(sat_y[i])
        xp, xy = float(r["xhat_p"][i]), float(r["xhat_y"][i])
        if not math.isnan(xp):
            iae_p += abs(xp) * dt_rec
            peak_tilt_p = max(peak_tilt_p, abs(xp))
        if not math.isnan(xy):
            iae_y += abs(xy) * dt_rec
            peak_tilt_y = max(peak_tilt_y, abs(xy))
        th = float(r["thrust_N"][i])
        if not math.isnan(th):
            peak_thrust = max(peak_thrust, th)

        if t[i] >= next_telem:
            next_telem += 1.0 / telem_hz
            frames.append(ReplayFrame(t[i], telem(i)))
        if t[i] >= next_status:
            next_status += 1.0 / status_hz
            frames.append(ReplayFrame(t[i], status(i)))

    last = len(r) - 1
    if state[last] == S["TAILOFF"]:
        t_safe = t[last] + dt_rec
        event(t_safe, int((t_safe - t_ignite) * 1000) if t_ignite is not None else tc.T_MS_NONE,
              "STATE_SAFE", note="inferred: the SD log closes on TAILOFF -> SAFE")
        frames.append(ReplayFrame(t_safe, telem(last, st=S["SAFE"])))
        frames.append(ReplayFrame(t_safe, status(last)))

    frames.sort(key=lambda f: f.t)
    return header, frames
