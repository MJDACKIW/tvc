#!/usr/bin/env python3
"""LoRa time on air and duty cycle for the TELEM / STATUS downlink (SPEC.md Section 4.7).

Reads params.yaml and the payload sizes in protocol/telem_codec.py, so it prints what the
current settings give. The same Semtech SX127x formula as protocol/lora_link.h, which the
firmware times its transmissions with; ground/tests/test_lora_airtime.py and
firmware/test/test_lora_link pin both to the same hand-checked values.

    ground/venv/bin/python tools/lora_airtime.py
    ground/venv/bin/python tools/lora_airtime.py --bw 500       # what-if, changes nothing

RadioHead adds 4 bytes (to, from, id, flags) to every frame it sends, so a 33 B TELEM frame
is 37 B on the air. Both figures are printed.
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "protocol"))

RADIOHEAD_HEADER_BYTES = 4   # protocol/lora_link.h kRadioHeadHeaderBytes
PREAMBLE_SYMBOLS = 8         # protocol/lora_link.h kPreambleSymbols
DUTY_LIMIT_PCT = 50.0        # above this the link has no room for EVENT, ACK and HEARTBEAT
TX_START_COST_US = 500       # FIFO load before the transmitter starts (protocol/lora_link.h)


def time_on_air_us(phy_bytes: int, sf: int, bw_hz: int, cr_denominator: int,
                   preamble_symbols: int = PREAMBLE_SYMBOLS) -> int:
    """Time on air in whole microseconds. Explicit header, payload CRC on, as RH_RF95 runs it.
    Low data rate optimisation is on when a symbol lasts more than 16 ms (RadioHead decides
    the same way). Integer math throughout, so it matches protocol/lora_link.h exactly."""
    sym_ns = (1 << sf) * 1_000_000_000 // bw_hz
    low_data_rate = sym_ns > 16_000_000
    numerator = 8 * phy_bytes - 4 * sf + 28 + 16
    denominator = 4 * (sf - (2 if low_data_rate else 0))
    groups = max(0, -(-numerator // denominator))  # ceil
    payload_symbols = 8 + groups * cr_denominator
    preamble_ns = (4 * preamble_symbols + 17) * sym_ns // 4  # (n + 4.25) symbols
    return (preamble_ns + payload_symbols * sym_ns + 500) // 1000


def frame_bytes(payload_len: int) -> int:
    """sync0, sync1, len, type, payload, crc16."""
    return 4 + payload_len + 2


def load_settings() -> dict:
    import yaml
    params = yaml.safe_load((REPO / "params.yaml").read_text())
    tel = params["telemetry"]
    return {"sf": int(tel["lora_sf"]), "bw_khz": int(tel["lora_bw_khz"]),
            "cr": int(tel["lora_cr"]), "telem_hz": float(tel["telem_rate_hz"]),
            "status_hz": float(tel["status_rate_hz"])}


def budget(sf: int, bw_khz: int, cr: int, telem_hz: float, status_hz: float) -> dict:
    import telem_codec as tc
    bw_hz = bw_khz * 1000
    rows = {}
    for name, ptype in (("TELEM", tc.PacketType.TELEM), ("STATUS", tc.PacketType.STATUS)):
        wire = frame_bytes(tc.PAYLOAD_SIZES[ptype])
        rows[name] = {
            "wire_bytes": wire,
            "air_bytes": wire + RADIOHEAD_HEADER_BYTES,
            "frame_only_us": time_on_air_us(wire, sf, bw_hz, cr),
            "air_us": time_on_air_us(wire + RADIOHEAD_HEADER_BYTES, sf, bw_hz, cr),
        }
    rate = {"TELEM": telem_hz, "STATUS": status_hz}
    busy_us_per_s = sum(rate[n] * rows[n]["air_us"] for n in rows)
    period_us = 1_000_000 / telem_hz if telem_hz else float("inf")
    return {"rows": rows, "rate": rate, "duty_pct": busy_us_per_s / 10_000.0,
            "sf": sf, "bw_khz": bw_khz, "cr": cr,
            # time left in a TELEM period after the TELEM frame, and what a STATUS needs of it
            "gap_us": period_us - rows["TELEM"]["air_us"],
            "status_needs_us": rows["STATUS"]["air_us"] + TX_START_COST_US}


def duty_at(sf: int, bw_khz: int, cr: int, telem_hz: float, status_hz: float) -> float:
    return budget(sf, bw_khz, cr, telem_hz, status_hz)["duty_pct"]


def format_report(b: dict) -> list[str]:
    out = [f"LoRa SF{b['sf']} BW{b['bw_khz']} kHz CR4/{b['cr']}, preamble {PREAMBLE_SYMBOLS}, "
           "explicit header, CRC on"]
    out.append(f"  {'frame':<7}{'wire':>7}{'on air':>8}{'  time (frame only)':>20}"
               f"{'  time on air':>14}{'  rate':>8}")
    for name, r in b["rows"].items():
        out.append(f"  {name:<7}{r['wire_bytes']:>5} B{r['air_bytes']:>6} B"
                   f"{r['frame_only_us'] / 1000:>15.1f} ms{r['air_us'] / 1000:>11.1f} ms"
                   f"{b['rate'][name]:>5.0f} Hz")
    out.append(f"  downlink duty cycle: {b['duty_pct']:.1f}% "
               f"({b['rate']['TELEM']:.0f} Hz TELEM + {b['rate']['STATUS']:.0f} Hz STATUS)")
    out.append(f"  gap after each TELEM: {b['gap_us'] / 1000:.1f} ms; a STATUS needs "
               f"{b['status_needs_us'] / 1000:.1f} ms of it, so it "
               + ("fits with room to spare" if b["gap_us"] >= 1.5 * b["status_needs_us"]
                  else "fits with little room" if b["gap_us"] >= b["status_needs_us"]
                  else "does NOT fit and TELEM frames will be skipped"))
    out.append("  'on air' includes RadioHead's 4 header bytes; EVENT (3 copies each), ACK and "
               "the 2 Hz HEARTBEAT uplink share the channel on top of this.")
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--sf", type=int)
    ap.add_argument("--bw", type=int, help="bandwidth in kHz (125, 250, 500)")
    ap.add_argument("--cr", type=int, help="coding rate denominator (5 means 4/5)")
    ap.add_argument("--telem-hz", type=float)
    ap.add_argument("--status-hz", type=float)
    args = ap.parse_args(argv)

    s = load_settings()
    for key, val in (("sf", args.sf), ("bw_khz", args.bw), ("cr", args.cr),
                     ("telem_hz", args.telem_hz), ("status_hz", args.status_hz)):
        if val is not None:
            s[key] = val
    overridden = any(v is not None for v in (args.sf, args.bw, args.cr, args.telem_hz,
                                              args.status_hz))
    b = budget(s["sf"], s["bw_khz"], s["cr"], s["telem_hz"], s["status_hz"])
    if overridden:
        print("what-if: command-line overrides on top of params.yaml (nothing is written)")
    for line in format_report(b):
        print(line)

    if b["duty_pct"] > DUTY_LIMIT_PCT:
        print()
        print(f"WARN duty cycle {b['duty_pct']:.1f}% is above {DUTY_LIMIT_PCT:.0f}%.")
        if s["bw_khz"] < 500:
            nxt = duty_at(s["sf"], 500, s["cr"], s["telem_hz"], s["status_hz"])
            print(f"next: telemetry.lora_bw_khz: 500 would give {nxt:.1f}% (not applied). "
                  "It costs about 3 dB of sensitivity per doubling; check RSSI/SNR margin with "
                  "./tvc link at the real stand distance before taking it.")
        else:
            print("next: bandwidth is already 500 kHz. Lower telemetry.telem_rate_hz or SF.")
    elif math.isfinite(b["duty_pct"]):
        print(f"OK duty cycle is within the {DUTY_LIMIT_PCT:.0f}% budget.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
