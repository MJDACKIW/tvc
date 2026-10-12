#!/usr/bin/env python3
"""
decode_log.py -- read an SD-primary STAND_OPEN log (firmware/src/flight/logging_sd.cpp),
verify it, and plot it. Supersedes static_fire_decoder/decode_log.py, which decoded the
legacy static_fire.cpp's smaller, header-less Record; this one understands the SPEC.md
Section 4.6 LogHeader + Record layout used by env:teensy_flight.

    python decode_log.py SF_000.BIN
    python decode_log.py SF_000.BIN --csv out.csv --npz out.npz

Record/LogHeader layout must match firmware/src/flight/{record.h,logging_sd.cpp} exactly. If
the header magic doesn't match, or the file size (minus header) doesn't divide evenly by
RECORD_SIZE, one of the two has drifted from the other -- fix that before trusting anything
else this script reports.
"""

import argparse
import struct
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "protocol"))

# State/mode/profile names, fault bits, and the CRC are shared with the telemetry side
# (protocol/telem_codec.py) rather than kept as a second hand-synced copy here.
from telem_codec import FAULT_BITS, crc16_ccitt_false  # noqa: E402
from telem_codec import MODE_NAMES as MODE_NAME  # noqa: E402
from telem_codec import PROFILE_NAMES as PROFILE_NAME  # noqa: E402
from telem_codec import STATE_NAMES as STATE_NAME  # noqa: E402

# '<' little endian, no padding -- matches firmware/src/flight/logging_sd.cpp's #pragma pack(1)
HEADER_FMT = "<II65s20sBB2sII"
#              magic schema params_hash git_hash mode profile pad boot_count record_size
HEADER_SIZE = struct.calcsize(HEADER_FMT)
HEADER_MAGIC = 0x4C435654  # "TVCL"

# '<' little endian, no padding -- matches firmware/src/flight/record.h's #pragma pack(1)
#  I    t_us          B  state       B  mode        H  fault_bits   f  thrust_N
#  hhh  gyro[3]       hhh accel[3]
#  ff   xhat_p,xhat_y ff  u_p,u_y    ff delta_p,delta_y  ff enc_p,enc_y  ff K_p,K_y
#  HH   servo_us_p,servo_us_y  B pyro  B cont_adc8  H loop_us  H crc16
RECORD_FMT = "<IBBHfhhhhhhffffffffffHHBBHH"
RECORD_SIZE = struct.calcsize(RECORD_FMT)
assert RECORD_SIZE == 74, f"RECORD_FMT computes to {RECORD_SIZE} bytes, expected 74 " \
    "(SPEC.md 4.6's own prose says 64; the field list there sums to 74 -- see the session " \
    "notes. This script follows the field list, i.e. firmware/src/flight/record.h.)"

GYRO_LSB_PER_DPS = 16.4     # +/-2000 dps, matches firmware/src/flight/imu.cpp
ACCEL_LSB_PER_G = 2048.0    # +/-16 g


def current_params_hash():
    """Reads the hash gen_params.py last stamped into core/params.h, for a header comparison."""
    params_h = REPO_ROOT / "core" / "params.h"
    try:
        text = params_h.read_text()
    except OSError:
        return None
    for line in text.splitlines():
        if "kParamsHash" in line and '"' in line:
            return line.split('"')[1]
    return None


class LogFormatError(ValueError):
    pass


def load_header(raw: bytes):
    if len(raw) < HEADER_SIZE:
        raise LogFormatError(f"file is only {len(raw)} bytes, shorter than the "
                             f"{HEADER_SIZE}-byte header")
    magic, schema, params_hash, git_hash, mode, profile, _pad, boot_count, record_size = \
        struct.unpack_from(HEADER_FMT, raw, 0)
    if magic != HEADER_MAGIC:
        raise LogFormatError(f"header magic 0x{magic:08X} != expected 0x{HEADER_MAGIC:08X} -- "
                             "this is not a teensy_flight SD log (wrong file, or the header "
                             "layout has drifted)")
    if record_size != RECORD_SIZE:
        raise LogFormatError(f"header says sizeof(Record) = {record_size}, this script "
                             f"expects {RECORD_SIZE} -- RECORD_FMT here and "
                             "firmware/src/flight/record.h disagree")
    return {
        "schema": schema,
        "params_hash": params_hash.split(b"\0", 1)[0].decode("ascii", "replace"),
        "git_hash": git_hash.split(b"\0", 1)[0].decode("ascii", "replace"),
        "mode": mode,
        "profile": profile,
        "boot_count": boot_count,
    }


def load_records(raw: bytes):
    body = raw[HEADER_SIZE:]
    n, rem = divmod(len(body), RECORD_SIZE)
    if rem:
        print(f"WARNING: {len(body)} bytes of record data is not a multiple of "
              f"{RECORD_SIZE}; {rem} trailing bytes ignored (likely a partial last flush).")
    if n == 0:
        raise LogFormatError("No complete records found after the header.")

    recs = np.zeros(n, dtype=[
        ("t_us", "u4"), ("state", "u1"), ("mode", "u1"), ("fault_bits", "u2"),
        ("thrust_N", "f4"), ("gx", "i2"), ("gy", "i2"), ("gz", "i2"),
        ("ax", "i2"), ("ay", "i2"), ("az", "i2"),
        ("xhat_p", "f4"), ("xhat_y", "f4"), ("u_p", "f4"), ("u_y", "f4"),
        ("delta_p", "f4"), ("delta_y", "f4"), ("enc_p", "f4"), ("enc_y", "f4"),
        ("K_p", "f4"), ("K_y", "f4"), ("servo_us_p", "u2"), ("servo_us_y", "u2"),
        ("pyro", "u1"), ("cont_adc8", "u1"), ("loop_us", "u2"), ("crc16", "u2"),
    ])
    unpack = struct.Struct(RECORD_FMT).unpack_from
    crc_ok = np.ones(n, dtype=bool)
    for i in range(n):
        off = i * RECORD_SIZE
        vals = unpack(body, off)
        recs[i] = vals
        stored_crc = vals[-1]
        computed = crc16_ccitt_false(body[off:off + RECORD_SIZE - 2])
        if computed != stored_crc:
            crc_ok[i] = False
    return recs, int(np.count_nonzero(~crc_ok)), crc_ok


def summarize(header, r, crc_fail):
    mode_name = MODE_NAME[header["mode"]] if header["mode"] < len(MODE_NAME) else "?"
    profile_name = PROFILE_NAME[header["profile"]] if header["profile"] < len(PROFILE_NAME) else "?"
    print(f"mode         : {mode_name}")
    print(f"profile      : {profile_name}")
    print(f"boot_count   : {header['boot_count']}")
    print(f"fw git hash  : {header['git_hash']}")
    cur_hash = current_params_hash()
    match = "MATCH" if cur_hash and cur_hash == header["params_hash"] else \
        ("no local core/params.h to compare" if not cur_hash else "MISMATCH")
    print(f"params hash  : {header['params_hash'][:12]}... ({match})")

    t = (r["t_us"].astype(np.int64) - int(r["t_us"][0])) / 1e6
    dt = np.diff(t)
    nominal_dt = 1.0 / 300.0

    print(f"records      : {len(r)}   (CRC failures: {crc_fail})")
    print(f"duration     : {t[-1]:.3f} s")
    if len(dt):
        print(f"sample rate  : {1.0 / np.median(dt):.1f} Hz median (expect 300 Hz)")
        dropped = np.count_nonzero(dt > 1.5 * nominal_dt)
        print(f"dropped/late : {dropped} gaps > 1.5x nominal period "
              f"({100 * dropped / len(dt):.2f}%), worst {dt.max() * 1e3:.2f} ms")

    seen = []
    for s in r["state"]:
        if not seen or seen[-1] != s:
            seen.append(int(s))
    names = [STATE_NAME[s] if s < len(STATE_NAME) else f"?{s}" for s in seen]
    print(f"state path   : {' -> '.join(names)}")

    all_faults = 0
    for f in r["fault_bits"]:
        all_faults |= int(f)
    if all_faults:
        names = [name for bit, name in FAULT_BITS if all_faults & bit]
        print(f"faults seen  : 0x{all_faults:04X} ({', '.join(names)})")
    else:
        print("faults seen  : none")

    burn = r["state"] == STATE_NAME.index("BURN")
    if burn.any():
        tb, th = t[burn], r["thrust_N"][burn]
        print(f"burn window  : {tb[0]:.3f} -> {tb[-1]:.3f} s ({tb[-1] - tb[0]:.3f} s)")
        if not np.all(np.isnan(th)):
            print(f"peak thrust  : {np.nanmax(th):.2f} N")
            print(f"impulse      : {np.trapezoid(np.nan_to_num(th), tb):.2f} N-s")
        else:
            print("peak thrust  : all NaN (loadcell.hx_scale_counts_per_n still MEASURE?)")
        print(f"gimbal pitch : {np.nanmin(r['delta_p'][burn]):+.2f} .. "
              f"{np.nanmax(r['delta_p'][burn]):+.2f} deg (applied, open-loop)")
        print(f"gimbal yaw   : {np.nanmin(r['delta_y'][burn]):+.2f} .. "
              f"{np.nanmax(r['delta_y'][burn]):+.2f} deg (applied, open-loop)")
    if np.all(np.isnan(r["xhat_p"])):
        print("note         : xhat_p/xhat_y are all NaN -- kalman.q/kalman.r are still MEASURE "
              "in params.yaml (expected until the stand tuning pass); the attitude plot falls "
              "back to a raw accelerometer-only reference angle.")
    return t


def to_csv(path, r, t):
    accel_pitch_deg = np.degrees(np.arctan2(r["ay"] / ACCEL_LSB_PER_G, r["az"] / ACCEL_LSB_PER_G))
    accel_yaw_deg = np.degrees(np.arctan2(-r["ax"] / ACCEL_LSB_PER_G, r["az"] / ACCEL_LSB_PER_G))
    hdr = ("t_s,state,fault_bits,thrust_N,gx_dps,gy_dps,gz_dps,ax_g,ay_g,az_g,"
           "accel_pitch_deg,accel_yaw_deg,xhat_p_deg,xhat_y_deg,u_p_deg,u_y_deg,"
           "delta_p_deg,delta_y_deg,enc_p_deg,enc_y_deg,K_p,K_y,servo_us_p,servo_us_y,"
           "pyro,cont_adc8,loop_us,crc16")
    out = np.column_stack([
        t, r["state"], r["fault_bits"], r["thrust_N"],
        r["gx"] / GYRO_LSB_PER_DPS, r["gy"] / GYRO_LSB_PER_DPS, r["gz"] / GYRO_LSB_PER_DPS,
        r["ax"] / ACCEL_LSB_PER_G, r["ay"] / ACCEL_LSB_PER_G, r["az"] / ACCEL_LSB_PER_G,
        accel_pitch_deg, accel_yaw_deg,
        r["xhat_p"], r["xhat_y"], r["u_p"], r["u_y"], r["delta_p"], r["delta_y"],
        r["enc_p"], r["enc_y"], r["K_p"], r["K_y"], r["servo_us_p"], r["servo_us_y"],
        r["pyro"], r["cont_adc8"], r["loop_us"], r["crc16"],
    ])
    np.savetxt(path, out, delimiter=",", header=hdr, comments="", fmt="%.6g")
    print(f"wrote {path}")


def to_npz(path, header, r, t):
    np.savez(path, t=t, **{name: r[name] for name in r.dtype.names}, **{
        f"header_{k}": v for k, v in header.items()
    })
    print(f"wrote {path}")


def plot(header, r, t):
    import matplotlib.pyplot as plt

    accel_pitch_deg = np.degrees(np.arctan2(r["ay"] / ACCEL_LSB_PER_G, r["az"] / ACCEL_LSB_PER_G))
    accel_yaw_deg = np.degrees(np.arctan2(-r["ax"] / ACCEL_LSB_PER_G, r["az"] / ACCEL_LSB_PER_G))

    fig, ax = plt.subplots(6, 1, figsize=(11, 15), sharex=True)

    ax[0].plot(t, r["thrust_N"], lw=0.9)
    ax[0].set_ylabel("thrust [N]")
    ax[0].grid(alpha=0.3)

    ax[1].plot(t, accel_pitch_deg, lw=0.8, alpha=0.6, label="accel-only pitch (reference)")
    ax[1].plot(t, accel_yaw_deg, lw=0.8, alpha=0.6, label="accel-only yaw (reference)")
    ax[1].plot(t, r["xhat_p"], lw=1.1, label="Kalman xhat_p")
    ax[1].plot(t, r["xhat_y"], lw=1.1, label="Kalman xhat_y")
    ax[1].plot(t, r["enc_p"], lw=0.8, ls="--", label="encoder pitch")
    ax[1].plot(t, r["enc_y"], lw=0.8, ls="--", label="encoder yaw")
    ax[1].set_ylabel("attitude [deg]")
    ax[1].legend(loc="upper right", fontsize=7)
    ax[1].grid(alpha=0.3)

    ax[2].plot(t, r["u_p"], lw=0.7, alpha=0.6, label="u_p (shadow PID, not applied)")
    ax[2].plot(t, r["u_y"], lw=0.7, alpha=0.6, label="u_y (shadow PID, not applied)")
    ax[2].plot(t, r["delta_p"], lw=1.1, label="delta_p (applied)")
    ax[2].plot(t, r["delta_y"], lw=1.1, label="delta_y (applied)")
    ax[2].set_ylabel("gimbal [deg]")
    ax[2].legend(loc="upper right", fontsize=7)
    ax[2].grid(alpha=0.3)

    for axis, lsb_field in (("gx", "gx"), ("gy", "gy"), ("gz", "gz")):
        ax[3].plot(t, r[lsb_field] / GYRO_LSB_PER_DPS, lw=0.7, label=axis)
    ax[3].set_ylabel("gyro [deg/s]")
    ax[3].legend(loc="upper right", fontsize=7)
    ax[3].grid(alpha=0.3)

    ax[4].plot(t, r["K_p"], lw=0.8, label="K_p")
    ax[4].plot(t, r["K_y"], lw=0.8, label="K_y")
    ax[4].set_ylabel("Kalman gain")
    ax[4].legend(loc="upper right", fontsize=7)
    ax[4].grid(alpha=0.3)

    ax[5].plot(t, r["fault_bits"], lw=0.8, drawstyle="steps-post")
    ax[5].set_ylabel("fault_bits")
    ax[5].set_xlabel("time [s]")
    ax[5].grid(alpha=0.3)

    burn = r["state"] == STATE_NAME.index("BURN")
    if burn.any():
        for a in ax:
            a.axvspan(t[burn][0], t[burn][-1], color="k", alpha=0.06)

    fig.suptitle(f"{MODE_NAME[header['mode']]} / {PROFILE_NAME[header['profile']]}  "
                 f"(fw {header['git_hash']}, boot #{header['boot_count']})")
    fig.tight_layout()
    plt.show()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("path")
    ap.add_argument("--csv", help="also write a CSV copy")
    ap.add_argument("--npz", help="also write an .npz copy")
    ap.add_argument("--no-plot", action="store_true")
    args = ap.parse_args()

    raw = Path(args.path).read_bytes()
    try:
        header = load_header(raw)
        r, crc_fail, _crc_ok = load_records(raw)
    except LogFormatError as exc:
        sys.exit(str(exc))
    t = summarize(header, r, crc_fail)

    if args.csv:
        to_csv(args.csv, r, t)
    if args.npz:
        to_npz(args.npz, header, r, t)
    if not args.no_plot:
        plot(header, r, t)


if __name__ == "__main__":
    main()
