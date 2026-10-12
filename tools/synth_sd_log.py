#!/usr/bin/env python3
"""Build and run tools/synth_sd_log.cpp: writes a SYNTHETIC teensy_flight SD log (one full
STAND_OPEN / STEP_X dry sequence) for testing tools/decode_log.py and ground/gs.py --replay
while no real bench log exists. See the .cpp's header for what is real (record.h layout, core/
controller, profiles.cpp) and what is made up (all the physics and noise).

    python3 tools/synth_sd_log.py                      # -> data/synthetic/SF_SYNTH.BIN
    python3 tools/synth_sd_log.py --out /tmp/x.BIN
"""
import argparse
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_OUT = REPO_ROOT / "data" / "synthetic" / "SF_SYNTH.BIN"


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", type=Path, default=DEFAULT_OUT)
    args = ap.parse_args()

    cxx = os.environ.get("CXX") or shutil.which("clang++") or shutil.which("c++")
    if not cxx:
        sys.exit("no C++ compiler found (set CXX)")
    subprocess.run([sys.executable, str(REPO_ROOT / "tools" / "gen_params.py")], check=True,
                   stdout=subprocess.DEVNULL)

    core = REPO_ROOT / "core"
    sources = [REPO_ROOT / "tools" / "synth_sd_log.cpp",
               REPO_ROOT / "firmware" / "src" / "flight" / "profiles.cpp",
               *(core / name for name in ("kalman1d.cpp", "pid.cpp", "rate_limiter.cpp",
                                          "controller.cpp"))]
    with tempfile.TemporaryDirectory() as tmp:
        exe = Path(tmp) / "synth_sd_log"
        # Same float flags as core/Makefile, so core/ behaves exactly as in the sim and parity test.
        subprocess.run([cxx, "-std=c++17", "-O0", "-ffp-contract=off", "-Wall",
                        "-I", str(core), "-I", str(REPO_ROOT / "firmware" / "src" / "flight"),
                        *map(str, sources), "-o", str(exe)], check=True)
        args.out.parent.mkdir(parents=True, exist_ok=True)
        subprocess.run([str(exe), str(args.out), str(REPO_ROOT / "sim" / "data" / "estes_e12.eng")],
                       check=True)


if __name__ == "__main__":
    main()
