"""protocol/telem_codec.py vs protocol/protocol.h, byte for byte.

Compiles a small C++ program against the real protocol.h (clang++, like core/Makefile), has it
fill every payload struct with fixed values and frame it with protocol.h's own crc16(), then
checks that telem_codec produces the identical bytes and decodes them back to the same values.
This is what keeps the one Python codec from drifting away from the firmware's header.

    ground/venv/bin/python -m pytest ground/tests
"""
import shutil
import struct
import subprocess
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "protocol"))

import telem_codec as tc  # noqa: E402

HARNESS = r"""
#include <cstdio>
#include <cstring>
#include "protocol.h"
using namespace tvc::protocol;

template <typename T>
void emit(const char* name, PacketType type, const T& p) {
    uint8_t f[kHeaderLen + kMaxPayload + kCrcLen];
    f[0] = kSyncByte0; f[1] = kSyncByte1; f[2] = sizeof(T); f[3] = static_cast<uint8_t>(type);
    std::memcpy(f + 4, &p, sizeof(T));
    uint16_t crc = crc16(f + 2, 2 + sizeof(T));
    f[4 + sizeof(T)] = crc & 0xFF; f[5 + sizeof(T)] = crc >> 8;
    std::printf("%s %zu ", name, sizeof(T));
    for (size_t i = 0; i < frameLen(sizeof(T)); i++) std::printf("%02x", f[i]);
    std::printf("\n");
}

int main() {
    std::printf("crc_check %04x\n", crc16(reinterpret_cast<const uint8_t*>("123456789"), 9));
    std::printf("flags %u %u %u %u %u %u %u\n", telem_flags::kPitchGateOk, telem_flags::kYawGateOk,
                telem_flags::kPitchSaturated, telem_flags::kYawSaturated, telem_flags::kTvcActive,
                telem_flags::kDryRun, telem_flags::kArmSwitchClosed);

    TelemPayload t{};
    t.t_ms = -1234; t.state = 6; t.thrust_x10 = 330; t.xhat_p_x100 = -500; t.xhat_y_x100 = 75;
    t.u_p_x100 = 1000; t.u_y_x100 = -1000; t.enc_p_x100 = 950; t.enc_y_x100 = -25;
    t.gyro_z_x10 = -45; t.k_p_x1000 = 125; t.k_y_x1000 = 500; t.flags = 0x5B; t.cont_adc8 = 200;
    t.batt_mv = 7650;
    emit("TELEM", PacketType::kTelem, t);

    StatusPayload s{};
    s.mode = 2; s.fault_bits = 0x1081;
    const uint8_t h6[6] = {0xa1, 0x41, 0x0d, 0xaa, 0xbd, 0xdb};
    std::memcpy(s.params_hash6, h6, 6);
    s.sd_bytes_written = 123456789; s.flash_bytes_written = 654321; s.rssi_last_uplink_dbm = -87;
    s.boot_count = 42; s.missing_measure_bits = 0x7FFF; s.loop_overrun_count = 3;
    s.sat_pct_p = 7; s.sat_pct_y = 100; s.running_iae_p_x100 = 1234; s.running_iae_y_x100 = 55;
    s.peak_thrust_x10 = 331; s.peak_tilt_p_x100 = 512; s.peak_tilt_y_x100 = 9;
    const uint8_t g4[4] = {0x4d, 0xef, 0xac, 0x7d};
    std::memcpy(s.fw_git_hash, g4, 4);
    emit("STATUS", PacketType::kStatus, s);

    EventPayload e{}; e.t_ms = -3000; e.event_code = 11; e.aux = 3;
    emit("EVENT", PacketType::kEvent, e);

    AckPayload a{}; a.seq = 65535; a.cmd_type = 0x16; a.result = 0; a.aux = 0xBEEF;
    emit("ACK", PacketType::kAck, a);

    UplinkPayload hb{}; hb.seq = 513;
    emit("HEARTBEAT", PacketType::kHeartbeat, hb);
    UplinkU8Payload sm{}; sm.seq = 7; sm.value = 3;
    emit("SET_MODE", PacketType::kSetMode, sm);
    FireConfirmPayload fc{}; fc.seq = 9; fc.token = 0xBEEF;
    emit("FIRE_CONFIRM", PacketType::kFireConfirm, fc);
    return 0;
}
"""


@pytest.fixture(scope="module")
def cpp(tmp_path_factory):
    cxx = shutil.which("clang++") or shutil.which("c++")
    if not cxx:
        pytest.skip("no C++ compiler on PATH")
    d = tmp_path_factory.mktemp("proto")
    src, exe = d / "harness.cpp", d / "harness"
    src.write_text(HARNESS)
    subprocess.run([cxx, "-std=c++17", "-Wall", "-I", str(REPO_ROOT / "protocol"), str(src),
                    "-o", str(exe)], check=True)
    out = subprocess.run([str(exe)], check=True, capture_output=True, text=True).stdout
    lines = {}
    for line in out.splitlines():
        name, *rest = line.split()
        lines[name] = rest
    return lines


def test_crc_matches_header_and_standard_vector(cpp):
    assert int(cpp["crc_check"][0], 16) == 0x29B1
    assert tc.crc16_ccitt_false(b"123456789") == 0x29B1


def test_flag_bits_match_header(cpp):
    assert [int(x) for x in cpp["flags"]] == [bit for bit, _ in tc.TELEM_FLAG_NAMES]


def test_telem(cpp):
    size, frame = int(cpp["TELEM"][0]), bytes.fromhex(cpp["TELEM"][1])
    assert size == tc.PAYLOAD_SIZES[tc.PacketType.TELEM]
    # Engineering values exactly representable after scaling, so the frame must match bitwise.
    payload = tc.encode_telem(t_ms=-1234, state=6, thrust_N=33.0, xhat_p_deg=-5.0,
                              xhat_y_deg=0.75, u_p_deg=10.0, u_y_deg=-10.0, enc_p_deg=9.5,
                              enc_y_deg=-0.25, gyro_z_deg_s=-4.5, K_p=0.125, K_y=0.5,
                              flags=0x5B, cont_adc8=200, batt_mv=7650)
    assert tc.encode_frame(tc.PacketType.TELEM, payload) == frame
    frames = tc.FrameParser().feed(frame)
    d = tc.decode(frames[0].ptype, frames[0].payload)
    assert d["t_ms"] == -1234 and d["state_name"] == "BURN"
    assert d["thrust_N"] == 33.0 and d["xhat_p_deg"] == -5.0 and d["enc_y_deg"] == -0.25
    assert d["K_p"] == 0.125 and d["batt_mv"] == 7650
    assert "arm_switch_closed" in d["flags_decoded"] and "dry_run" not in d["flags_decoded"]


def test_telem_encode_scaling_matches_firmware_truncation():
    p = struct.unpack(tc.TELEM_FMT, tc.encode_telem(
        t_ms=0, state=0, thrust_N=1.99, xhat_p_deg=-0.019, xhat_y_deg=float("nan"),
        u_p_deg=400.0, u_y_deg=-400.0, enc_p_deg=0, enc_y_deg=0, gyro_z_deg_s=0, K_p=-0.5,
        K_y=70.0, flags=0, cont_adc8=0, batt_mv=0))
    fields = dict(zip(tc.TELEM_FIELDS, p))
    assert fields["thrust_x10"] == 19          # 19.9 truncates toward zero
    assert fields["xhat_p_x100"] == -1         # -1.9 truncates toward zero
    assert fields["xhat_y_x100"] == 0          # NaN -> 0
    assert fields["u_p_x100"] == 32767 and fields["u_y_x100"] == -32768
    assert fields["k_p_x1000"] == 0 and fields["k_y_x1000"] == 65535


def test_status(cpp):
    size, frame = int(cpp["STATUS"][0]), bytes.fromhex(cpp["STATUS"][1])
    assert size == tc.PAYLOAD_SIZES[tc.PacketType.STATUS]
    payload = tc.encode_status(
        mode=2, fault_bits=0x1081, params_hash6=bytes.fromhex("a1410daabddb"),
        sd_bytes_written=123456789, flash_bytes_written=654321, rssi_last_uplink_dbm=-87,
        boot_count=42, missing_measure_bits=0x7FFF, loop_overrun_count=3, sat_pct_p=7,
        sat_pct_y=100, running_iae_p_degs=12.345, running_iae_y_degs=0.555,
        peak_thrust_N=33.15, peak_tilt_p_deg=5.125, peak_tilt_y_deg=0.095,
        fw_git_hash4=bytes.fromhex("4defac7d"))
    assert tc.encode_frame(tc.PacketType.STATUS, payload) == frame
    tables = tc.Tables(event_names={}, measure_bit_names={0: "control.kp"},
                       local_hash_hex="a1410daabddb" + "0" * 52)
    d = tc.decode_status(payload, tables)
    assert d["mode_name"] == "STAND_OPEN" and d["params_hash_match"] is True
    assert d["fault_names"] == ["MEASURE_MISSING", "LOOP_OVERRUN", "HX711"]  # 0x1081
    assert d["missing_measure_names"][0] == "control.kp" and len(d["missing_measure_names"]) == 15
    assert d["rssi_last_uplink_dbm"] == -87 and d["fw_git_hash_hex"] == "4defac7d"


def test_event_ack_uplink(cpp):
    for name, ptype, payload in [
        ("EVENT", tc.PacketType.EVENT, tc.encode_event(-3000, 11, 3)),
        ("ACK", tc.PacketType.ACK, tc.encode_ack(65535, tc.PacketType.COUNTDOWN, 0, 0xBEEF)),
        ("HEARTBEAT", tc.PacketType.HEARTBEAT, tc.encode_uplink(tc.PacketType.HEARTBEAT, 513)),
        ("SET_MODE", tc.PacketType.SET_MODE, tc.encode_uplink(tc.PacketType.SET_MODE, 7, 3)),
        ("FIRE_CONFIRM", tc.PacketType.FIRE_CONFIRM,
         tc.encode_uplink(tc.PacketType.FIRE_CONFIRM, 9, 0xBEEF)),
    ]:
        size, frame = int(cpp[name][0]), bytes.fromhex(cpp[name][1])
        assert size == len(payload), name
        assert tc.encode_frame(ptype, payload) == frame, name
    ack = tc.decode_ack(tc.encode_ack(65535, tc.PacketType.COUNTDOWN, 0, 0xBEEF))
    assert ack["cmd_name"] == "COUNTDOWN" and ack["result_name"] == "OK" and ack["aux"] == 0xBEEF
    assert tc.decode_uplink(tc.PacketType.FIRE_CONFIRM,
                            tc.encode_uplink(tc.PacketType.FIRE_CONFIRM, 9, 0xBEEF)) == \
        {"seq": 9, "token": 0xBEEF}


def test_parser_resyncs_and_reads_bridge_trailer():
    good = tc.encode_frame(tc.PacketType.EVENT, tc.encode_event(5, 9, 0))
    bad = bytearray(good)
    bad[6] ^= 0xFF  # corrupt the payload -> CRC failure
    stream = (b"\x00\xAA\x13garbage" + bytes(bad) + tc.bridge_trailer(-70, 8) +
              good + tc.bridge_trailer(-61, 9) + good + tc.bridge_trailer(-128, -3))
    p = tc.FrameParser(bridge_trailer=True)
    frames = []
    for i in range(0, len(stream), 3):  # arbitrary chunking
        frames += p.feed(stream[i:i + 3])
    assert [(f.rssi, f.snr) for f in frames] == [(-61, 9), (-128, -3)]
    assert all(f.raw == good for f in frames)
    assert p.crc_errors >= 1


def test_parser_without_trailer_back_to_back():
    frames = [tc.encode_frame(tc.PacketType.HEARTBEAT, tc.encode_uplink(tc.PacketType.HEARTBEAT, i))
              for i in range(5)]
    out = tc.FrameParser().feed(b"".join(frames))
    assert [tc.decode_uplink(f.ptype, f.payload)["seq"] for f in out] == list(range(5))
