#!/usr/bin/env python3
"""Minimal CLI ground tool (SPEC.md Section 9 Phase 4 deliverable): reads framed TELEM/STATUS/
EVENT/ACK packets off the Feather bridge's USB CDC port, verifies CRCs, and prints decoded
fields. This is the acceptance-test tool for the Radio phase; the full ground GUI is
ground/gs.py. All framing/CRC/decoding lives in protocol/telem_codec.py, shared with the GUI.

Usage:
    python3 tools/telem_cli.py --port /dev/tty.usbmodemXXXX
"""
import argparse
import struct
import sys
from pathlib import Path

import serial

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "protocol"))

import telem_codec as tc  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", required=True, help="Feather bridge's USB CDC port")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    tables = tc.Tables.load(REPO_ROOT)
    if not tables.event_names or not tables.measure_bit_names:
        print("warning: protocol/event_codes.json or core/measure_bits.json not found -- names "
              "will show as raw numbers (run tools/gen_params.py / tools/gen_protocol.py first)",
              file=sys.stderr)

    ser = serial.Serial(args.port, args.baud, timeout=0.2)
    frames = tc.FrameParser(bridge_trailer=True)
    print(f"listening on {args.port} @ {args.baud} baud (Ctrl-C to stop)")

    crc_errors_seen = 0
    try:
        while True:
            for frame in frames.feed(ser.read(256)):
                name = tc.packet_name(frame.ptype)
                try:
                    decoded = tc.decode(frame.ptype, frame.payload, tables)
                except (struct.error, ValueError) as exc:
                    print(f"{name}: payload size mismatch ({len(frame.payload)} bytes): {exc}",
                          file=sys.stderr)
                    continue
                print(f"[{name}] rssi={frame.rssi}dBm snr={frame.snr}dB {decoded}")
            if frames.crc_errors != crc_errors_seen:
                print(f"CRC mismatch x{frames.crc_errors - crc_errors_seen} -- dropped",
                      file=sys.stderr)
                crc_errors_seen = frames.crc_errors
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
