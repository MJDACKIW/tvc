"""tools/lora_airtime.py vs protocol/lora_link.h, value for value.

The firmware times its transmissions with lora_link.h and the Mac prints the airtime budget
with tools/lora_airtime.py. They are two implementations of the same Semtech formula, so this
compiles the real header, asks it for the time on air over a grid of settings, and requires
the Python function to give the identical microsecond count every time. The anchor values are
worked out by hand from the datasheet formula (and the first is the commonly quoted figure for
a 23 B SF7/BW125 LoRa packet), so a shared mistake in both would still be caught.

    ground/venv/bin/python -m pytest ground/tests/test_lora_airtime.py
"""
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "tools"))
sys.path.insert(0, str(REPO_ROOT / "protocol"))

import lora_airtime as la  # noqa: E402

SFS = range(7, 13)
BWS = (125_000, 250_000, 500_000)
CRS = (5, 6, 7, 8)
PAYLOADS = (1, 10, 23, 37, 52, 58, 100, 255)

# (phy_bytes, sf, bw_hz, cr_denominator) -> microseconds, from the datasheet formula by hand.
ANCHORS = {
    (23, 7, 125_000, 5): 61_696,    # LoRaWAN 10 B uplink at SF7: the usual 61.7 ms
    (37, 7, 125_000, 5): 82_176,    # TELEM, 33 B frame + 4 B RadioHead header
    (52, 7, 125_000, 5): 102_656,   # STATUS, 48 B frame + 4 B header
    (37, 7, 250_000, 5): 41_088,
    (52, 7, 250_000, 5): 51_328,
    (37, 7, 500_000, 5): 20_544,
    (52, 7, 500_000, 5): 25_664,
}

HARNESS = r"""
#include <cstdio>
#include <initializer_list>
#include "lora_link.h"
using namespace tvc::protocol::lora;
int main() {
    for (int sf = 7; sf <= 12; sf++)
        for (long bw : {125000L, 250000L, 500000L})
            for (int cr = 5; cr <= 8; cr++)
                for (unsigned pl : {1u, 10u, 23u, 37u, 52u, 58u, 100u, 255u})
                    std::printf("%u %d %ld %d %u\n", pl, sf, bw, cr,
                                (unsigned)timeOnAirUs(pl, sf, bw, cr));
    return 0;
}
"""


@pytest.fixture(scope="module")
def header_values(tmp_path_factory):
    cxx = shutil.which("clang++") or shutil.which("c++")
    if cxx is None:
        pytest.skip("no C++ compiler on PATH")
    # lora_link.h needs params.h, which pio and ./tvc test generate; make sure it is current.
    subprocess.run([sys.executable, str(REPO_ROOT / "tools" / "gen_params.py")],
                   check=True, capture_output=True)
    d = tmp_path_factory.mktemp("lora")
    src, exe = d / "h.cpp", d / "h"
    src.write_text(HARNESS)
    build = subprocess.run([cxx, "-std=c++17", "-Wall", "-I", str(REPO_ROOT / "protocol"),
                            "-I", str(REPO_ROOT / "core"), str(src), "-o", str(exe)],
                           capture_output=True, text=True)
    assert build.returncode == 0, build.stderr
    out = subprocess.run([str(exe)], check=True, capture_output=True, text=True).stdout
    return [tuple(int(x) for x in line.split()) for line in out.splitlines()]


def test_anchor_values_by_hand():
    for (pl, sf, bw, cr), want in ANCHORS.items():
        assert la.time_on_air_us(pl, sf, bw, cr) == want, (pl, sf, bw, cr)


def test_python_matches_the_firmware_header_everywhere(header_values):
    assert len(header_values) == len(SFS) * len(BWS) * len(CRS) * len(PAYLOADS)
    for pl, sf, bw, cr, us in header_values:
        assert la.time_on_air_us(pl, sf, bw, cr) == us, (pl, sf, bw, cr)


def test_header_agrees_with_the_anchors(header_values):
    table = {(pl, sf, bw, cr): us for pl, sf, bw, cr, us in header_values}
    for key, want in ANCHORS.items():
        if key in table:
            assert table[key] == want, key


def test_telem_and_status_wire_sizes():
    import telem_codec as tc
    assert la.frame_bytes(tc.PAYLOAD_SIZES[tc.PacketType.TELEM]) == 33
    assert la.frame_bytes(tc.PAYLOAD_SIZES[tc.PacketType.STATUS]) == 48


def test_budget_at_the_bandwidths_that_matter():
    # 10 Hz TELEM + 1 Hz STATUS, SF7 CR4/5
    assert la.duty_at(7, 125, 5, 10, 1) == pytest.approx(92.4, abs=0.05)
    assert la.duty_at(7, 250, 5, 10, 1) == pytest.approx(46.2, abs=0.05)
    assert la.duty_at(7, 500, 5, 10, 1) == pytest.approx(23.1, abs=0.05)
