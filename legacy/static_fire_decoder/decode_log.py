#!/usr/bin/env python3
"""
decode_log.py — read a static_fire binary log, sanity-check it, plot it.

    python decode_log.py SF_000.BIN
    python decode_log.py SF_000.BIN --csv out.csv

The record layout must match the Record struct in static_fire.cpp. If the
firmware's reported sizeof(Record) differs from RECORD_FMT below, fix one of
them -- the script will tell you when the file size doesn't divide evenly.
"""

import argparse
import struct
import sys

import numpy as np

# '<' little endian, no padding -- matches #pragma pack(1)
#  I  uint32  t_us
#  B  uint8   state
#  B  uint8   profile
#  f  float   thrust_N
#  f  float   cmd_x_deg
#  f  float   cmd_y_deg
#  h*3        gyro x,y,z   (raw counts)
#  h*3        accel x,y,z  (raw counts)
RECORD_FMT = "<IBBfffhhhhhh"
RECORD_SIZE = struct.calcsize(RECORD_FMT)

STATE_NAME = ["BOOT", "IDLE", "ARMED", "COUNTDOWN", "IGNITE",
              "BURN", "TAILOFF", "SAFE", "FAULT"]
PROFILE_NAME = ["HOLD", "STEP_X", "STEP_Y", "RAMP_X", "CHIRP_X"]

# MPU-6050 full-scale settings used in the firmware
GYRO_LSB_PER_DPS = 16.4     # +/-2000 dps
ACCEL_LSB_PER_G = 2048.0    # +/-16 g


def load(path):
    with open(path, "rb") as fh:
        raw = fh.read()

    n, rem = divmod(len(raw), RECORD_SIZE)
    if rem:
        print(f"WARNING: {len(raw)} bytes is not a multiple of "
              f"{RECORD_SIZE}. Leftover {rem} bytes ignored.")
        print("  If this is consistent, RECORD_FMT here does not match the")
        print("  Record struct in firmware. Compare against the sizeof()")
        print("  the Teensy prints at boot.")
    if n == 0:
        sys.exit("No complete records found.")

    recs = np.zeros(n, dtype=[
        ("t_us", "u4"), ("state", "u1"), ("profile", "u1"),
        ("thrust_N", "f4"), ("cmd_x", "f4"), ("cmd_y", "f4"),
        ("gx", "i2"), ("gy", "i2"), ("gz", "i2"),
        ("ax", "i2"), ("ay", "i2"), ("az", "i2"),
    ])
    unpack = struct.Struct(RECORD_FMT).unpack_from
    for i in range(n):
        recs[i] = unpack(raw, i * RECORD_SIZE)
    return recs


def summarize(r):
    t = (r["t_us"] - r["t_us"][0]) / 1e6
    dt = np.diff(t)

    print(f"records      : {len(r)}")
    print(f"duration     : {t[-1]:.3f} s")
    if len(dt):
        print(f"sample rate  : {1.0 / np.median(dt):.1f} Hz median")
        print(f"worst gap    : {dt.max() * 1e3:.2f} ms "
              f"(expect ~4 ms at 250 Hz)")
        late = np.count_nonzero(dt > 2 * np.median(dt))
        print(f"late samples : {late} ({100 * late / len(dt):.2f}%)")

    seen = []
    for s in r["state"]:
        if not seen or seen[-1] != s:
            seen.append(s)
    names = [STATE_NAME[s] if s < len(STATE_NAME) else f"?{s}" for s in seen]
    print(f"state path   : {' -> '.join(names)}")

    p = int(r["profile"][0])
    print(f"profile      : {PROFILE_NAME[p] if p < len(PROFILE_NAME) else p}")

    burn = r["state"] == 5
    if burn.any():
        tb = t[burn]
        th = r["thrust_N"][burn]
        print(f"burn window  : {tb[0]:.3f} -> {tb[-1]:.3f} s "
              f"({tb[-1] - tb[0]:.3f} s)")
        print(f"peak thrust  : {th.max():.2f} N")
        print(f"mean thrust  : {th.mean():.2f} N")
        print(f"impulse      : {np.trapezoid(th, tb):.2f} N-s")
        print(f"gimbal X     : {r['cmd_x'][burn].min():+.2f} .. "
              f"{r['cmd_x'][burn].max():+.2f} deg")
        print(f"gimbal Y     : {r['cmd_y'][burn].min():+.2f} .. "
              f"{r['cmd_y'][burn].max():+.2f} deg")
    return t


def plot(r, t):
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(3, 1, figsize=(11, 8), sharex=True)

    ax[0].plot(t, r["thrust_N"], lw=0.9)
    ax[0].set_ylabel("thrust [N]")
    ax[0].grid(alpha=0.3)

    ax[1].plot(t, r["cmd_x"], lw=0.9, label="X")
    ax[1].plot(t, r["cmd_y"], lw=0.9, label="Y")
    ax[1].set_ylabel("gimbal cmd [deg]")
    ax[1].legend(loc="upper right")
    ax[1].grid(alpha=0.3)

    for axis in ("gx", "gy", "gz"):
        ax[2].plot(t, r[axis] / GYRO_LSB_PER_DPS, lw=0.7, label=axis)
    ax[2].set_ylabel("gyro [deg/s]")
    ax[2].set_xlabel("time [s]")
    ax[2].legend(loc="upper right")
    ax[2].grid(alpha=0.3)

    burn = r["state"] == 5
    if burn.any():
        for a in ax:
            a.axvspan(t[burn][0], t[burn][-1], color="k", alpha=0.06)

    fig.tight_layout()
    plt.show()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("path")
    ap.add_argument("--csv", help="also write a CSV copy")
    ap.add_argument("--no-plot", action="store_true")
    args = ap.parse_args()

    r = load(args.path)
    t = summarize(r)

    if args.csv:
        hdr = ("t_s,state,profile,thrust_N,cmd_x_deg,cmd_y_deg,"
               "gx_dps,gy_dps,gz_dps,ax_g,ay_g,az_g")
        out = np.column_stack([
            t, r["state"], r["profile"], r["thrust_N"],
            r["cmd_x"], r["cmd_y"],
            r["gx"] / GYRO_LSB_PER_DPS,
            r["gy"] / GYRO_LSB_PER_DPS,
            r["gz"] / GYRO_LSB_PER_DPS,
            r["ax"] / ACCEL_LSB_PER_G,
            r["ay"] / ACCEL_LSB_PER_G,
            r["az"] / ACCEL_LSB_PER_G,
        ])
        np.savetxt(args.csv, out, delimiter=",", header=hdr,
                   comments="", fmt="%.6g")
        print(f"wrote {args.csv}")

    if not args.no_plot:
        plot(r, t)


if __name__ == "__main__":
    main()
