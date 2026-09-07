#!/usr/bin/env python3
"""Compares a measured static-fire stand log against the matching sim prediction. See
SPEC.md Section 7. sim/vehicle.py's load_stand_log docstring has the decoded-log .npz
schema (tools/decode_log.py does not exist yet, so this is the schema it will need to
produce, not one it already does).

Runs sim/run_sim.py's --stand baseline scenario with thrust taken from the SAME log
(--thrust-from-log's mechanism), so the sim prediction uses the exact measured thrust
profile rather than the NAR curve, and with theta0/gyro_bias_dps also taken from the log
if --theta0-from-log is given. Aligns both traces at ignition (thrust rise) and reports:
  - IAE = integral_0^tb |theta| dt, for both measured and simulated
  - RMSE between the two theta(t) traces
  - the time-shift of the simulated trace that minimises RMSE against the measured one
    (a direct latency estimate: a nonzero shift means the sim's response leads or lags
    the real hardware's by that much)
and plots measured vs. simulated theta(t) and delta(t) (gimbal deflection). Refuses to
run if the log's params_hash does not match the sim's current one (SPEC.md Section 2):
that would mean the log was recorded against different params.yaml values than the sim
is now configured with, making the comparison meaningless.

Usage:
    python tools/iae_compare.py <log.npz> [--theta0-from-log] [--out-dir DIR]
    python tools/iae_compare.py --self-test   # proves the pipeline before real data exists
"""
import argparse
import contextlib
import sys
import tempfile
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
SIM_DIR = REPO_ROOT / "sim"
sys.path.insert(0, str(SIM_DIR))

import run_sim  # noqa: E402
import tvc_params  # noqa: E402
from vehicle import ThrustLog, find_ignition_index, load_stand_log  # noqa: E402
from vehicle import theta0_and_gyro_bias_from_log  # noqa: E402


def compute_iae(t, theta_deg, t_burn):
    mask = t <= t_burn
    return float(np.trapezoid(np.abs(theta_deg[mask]), t[mask]))


def compute_rmse(a, b):
    return float(np.sqrt(np.mean((a - b) ** 2)))


def best_time_shift(measured, simulated, dt, max_shift_s=0.2):
    """Search shifts of `simulated` relative to `measured` (integer steps of dt, up to
    +/- max_shift_s) that minimise RMSE. Positive shift means the simulated trace lags
    the measured one (the sim would need to be delayed by that much to best match real
    hardware); negative means it leads. Returns (best_shift_s, best_rmse), including the
    zero-shift case if nothing does better.
    """
    max_shift_steps = int(round(max_shift_s / dt))
    best_shift, best_rmse = 0, compute_rmse(measured, simulated)
    for shift in range(-max_shift_steps, max_shift_steps + 1):
        if shift == 0:
            continue
        if shift > 0:
            m, s = measured[shift:], simulated[:len(simulated) - shift]
        else:
            m, s = measured[:len(measured) + shift], simulated[-shift:]
        n = min(len(m), len(s))
        if n < 10:
            continue
        rmse = compute_rmse(m[:n], s[:n])
        if rmse < best_rmse:
            best_rmse, best_shift = rmse, shift
    return best_shift * dt, best_rmse


def _plot_comparison(out_path, sim_t, sim_theta, sim_gimbal, measured_theta, measured_gimbal,
                      title):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axs = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    axs[0].plot(sim_t, measured_theta, color="tab:blue", lw=1.4, label="measured")
    axs[0].plot(sim_t, sim_theta, color="tab:orange", ls="--", lw=1.4, label="simulated")
    axs[0].set_ylabel("Pitch angle (deg)")
    axs[0].legend(loc="upper right", fontsize=9)
    axs[0].grid(alpha=0.3)

    axs[1].plot(sim_t, measured_gimbal, color="tab:blue", lw=1.4, label="measured")
    axs[1].plot(sim_t, sim_gimbal, color="tab:orange", ls="--", lw=1.4, label="simulated")
    axs[1].set_ylabel("Gimbal deflection (deg)")
    axs[1].set_xlabel("Time since ignition (s)")
    axs[1].legend(loc="upper right", fontsize=9)
    axs[1].grid(alpha=0.3)

    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def compare(log_npz, theta0_from_log=False, out_dir=None):
    log = load_stand_log(log_npz)

    sim_hash = tvc_params.PARAMS_HASH
    if log["params_hash"] != sim_hash:
        raise SystemExit(
            f"iae_compare.py: refusing to run -- log params_hash "
            f"{log['params_hash'][:12]} does not match the sim's current params_hash "
            f"{sim_hash[:12]}. Regenerate core/params.h and sim/tvc_params.py to match "
            f"what the log was recorded against (or re-record against the current "
            f"params.yaml) before comparing.")

    thrust_log = ThrustLog(log)
    if theta0_from_log:
        theta0_deg, gyro_bias_dps = theta0_and_gyro_bias_from_log(log)
    else:
        theta0_deg, gyro_bias_dps = 5.0, 0.0

    sim_log = run_sim.run_simulation(
        theta0_deg=theta0_deg, gyro_bias_dps=gyro_bias_dps, stand=True,
        thrust_log=thrust_log, t_end=thrust_log.burn_time_s, noise=False, seed=0)

    ignition_idx = find_ignition_index(log["time_s"], log["thrust_n"])
    t_ignition = log["time_s"][ignition_idx]
    measured_t = log["time_s"] - t_ignition

    sim_t = sim_log["time"]
    measured_theta = np.interp(sim_t, measured_t, log["encoder_angle_deg"])
    measured_gimbal = np.interp(sim_t, measured_t, log["gimbal_actual_deg"])
    sim_theta = sim_log["true_angle"]
    sim_gimbal = sim_log["gimbal_actual"]

    t_burn = thrust_log.burn_time_s
    iae_measured = compute_iae(sim_t, measured_theta, t_burn)
    iae_sim = compute_iae(sim_t, sim_theta, t_burn)
    rmse = compute_rmse(measured_theta, sim_theta)
    shift_s, shifted_rmse = best_time_shift(measured_theta, sim_theta, run_sim.DT_SIM)

    result = {
        "iae_measured": iae_measured, "iae_sim": iae_sim, "rmse": rmse,
        "best_shift_s": shift_s, "rmse_at_best_shift": shifted_rmse,
    }

    run_name = Path(log_npz).stem
    print()
    print(f"iae_compare.py: {run_name} (theta0={theta0_deg:.3f} deg, "
          f"gyro_bias={gyro_bias_dps:.3f} deg/s, burn={t_burn:.3f} s)")
    print("-" * 66)
    print(f"IAE, measured   = {iae_measured:.4f} deg*s")
    print(f"IAE, simulated  = {iae_sim:.4f} deg*s")
    print(f"RMSE (no shift) = {rmse:.4f} deg")
    print(f"Best time shift = {shift_s * 1000.0:+.1f} ms  (RMSE there = {shifted_rmse:.4f} deg)")
    print("-" * 66)

    latex_name = run_name.replace("_", r"\_")
    latex_row = (f"{latex_name} & {iae_measured:.3f} & {iae_sim:.3f} & {rmse:.3f} & "
                 f"{shift_s * 1000.0:.1f} \\\\")
    print("LaTeX table row (run & IAE_measured & IAE_sim & RMSE & shift_ms):")
    print(latex_row)
    print()
    result["latex_row"] = latex_row

    out_dir = Path(out_dir) if out_dir else SIM_DIR / "figures"
    out_dir.mkdir(parents=True, exist_ok=True)
    plot_path = out_dir / f"iae_compare_{run_name}.png"
    _plot_comparison(plot_path, sim_t, sim_theta, sim_gimbal, measured_theta, measured_gimbal,
                      f"iae_compare: {run_name}\nparams hash {sim_hash[:12]}")
    result["plot_path"] = str(plot_path)

    npz_path = out_dir / f"iae_compare_{run_name}.npz"
    np.savez(npz_path, time=sim_t, measured_theta=measured_theta, sim_theta=sim_theta,
             measured_gimbal=measured_gimbal, sim_gimbal=sim_gimbal,
             params_hash=sim_hash, **{k: v for k, v in result.items()
                                      if k not in ("latex_row", "plot_path")})
    result["npz_path"] = str(npz_path)

    return result


@contextlib.contextmanager
def _perturbed_stand_params(scale_inertia=1.15, scale_coulomb=1.30, scale_arm=0.80):
    """Temporarily perturbs tvc_params.sim_overrides.stand in place, for self_test()'s
    synthetic 'measured' run. Restores the original values on exit regardless of how the
    block ends. The scales are a plausible build tolerance, not a stress test.
    """
    stand = tvc_params.sim_overrides.stand
    original = dict(vars(stand))
    stand.inertia_kg_m2 *= scale_inertia
    stand.friction_coulomb_Nm *= scale_coulomb
    stand.arm_pivot_to_gimbal_m *= scale_arm
    try:
        yield
    finally:
        for k, v in original.items():
            setattr(stand, k, v)


def self_test():
    """Proves the compare() pipeline end-to-end before any real stand log exists:
    generates a synthetic 'measured' run using perturbed stand parameters (+15% inertia,
    +30% Coulomb friction, -20% arm length -- a plausible build tolerance, not a stress
    test), packages it as a decoded-log .npz matching load_stand_log's schema (at an
    arbitrary, non-zero, non-ignition-aligned start time, to prove ignition detection
    actually re-zeroes rather than assuming the log already starts at t=0), and runs the
    full comparison against the nominal (unperturbed) prediction. noise=False throughout,
    so any nonzero IAE/RMSE below comes entirely from the parameter mismatch, not
    randomness -- this checks the plumbing (ignition detection, log schema, hash check,
    alignment, IAE/RMSE/shift), not how large a mismatch the metrics tolerate.
    """
    theta0_deg = 5.0
    t_arbitrary_offset = 137.0

    with _perturbed_stand_params():
        burn_time_s = run_sim.build_vehicle(stand=True).burn_time_s
        measured_log = run_sim.run_simulation(theta0_deg=theta0_deg, stand=True,
                                               noise=False, t_end=burn_time_s)

    synthetic = {
        "time_s": measured_log["time"] + t_arbitrary_offset,
        "thrust_n": measured_log["thrust"],
        "encoder_angle_deg": measured_log["true_angle"],
        "gyro_dps": measured_log["true_rate"],
        "gimbal_actual_deg": measured_log["gimbal_actual"],
        "params_hash": tvc_params.PARAMS_HASH,
    }

    tmp_dir = Path(tempfile.mkdtemp(prefix="iae_selftest_"))
    log_path = tmp_dir / "synthetic_measured.npz"
    np.savez(log_path, **synthetic)

    print()
    print("iae_compare.py self-test: synthetic 'measured' run from perturbed stand")
    print("params (+15% inertia, +30% Coulomb friction, -20% arm length; noise off),")
    print("compared against the nominal (unperturbed) sim prediction.")
    result = compare(str(log_path), theta0_from_log=False, out_dir=str(tmp_dir))

    checks = [
        ("IAE measured is finite and positive", result["iae_measured"] > 0),
        ("IAE sim is finite and positive", result["iae_sim"] > 0),
        ("RMSE is nonzero (perturbation is real, should show up)", result["rmse"] > 0),
        ("RMSE is small (same qualitative response, modest mismatch)", result["rmse"] < 2.0),
        ("best time shift found within the search window", abs(result["best_shift_s"]) < 0.2),
        ("plot file was written", Path(result["plot_path"]).exists()),
        ("npz file was written", Path(result["npz_path"]).exists()),
    ]
    all_ok = True
    print()
    print(f"{'Check':<62} {'Result':>10}")
    print("-" * 74)
    for label, ok in checks:
        all_ok &= ok
        print(f"{label:<62} {'OK' if ok else 'FAIL':>10}")
    print("-" * 74)
    print("PASS: iae_compare.py pipeline verified end-to-end" if all_ok
          else "FAIL: iae_compare.py pipeline has a problem, see above")
    print(f"(scratch files under {tmp_dir}, not committed)")
    print()
    return all_ok


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("log_npz", nargs="?", default=None,
                         help="Decoded stand-fire log (.npz); see sim/vehicle.py's "
                              "load_stand_log for the schema. Not used with --self-test.")
    parser.add_argument("--theta0-from-log", action="store_true",
                         help="Take theta0/gyro_bias_dps from the log's encoder angle at "
                              "ignition and pre-ignition gyro mean, instead of assuming "
                              "the usual 5 deg / 0 deg-per-s defaults.")
    parser.add_argument("--out-dir", default=None,
                         help="Directory for the comparison plot and npz "
                              "(default sim/figures).")
    parser.add_argument("--self-test", action="store_true",
                         help="Run the synthetic end-to-end pipeline check (a perturbed-"
                              "stand-parameters sim run standing in for a real log) "
                              "instead of comparing a real log; ignores log_npz.")
    args = parser.parse_args()

    if args.self_test:
        ok = self_test()
        sys.exit(0 if ok else 1)

    if not args.log_npz:
        parser.error("log_npz is required unless --self-test is given")
    compare(args.log_npz, theta0_from_log=args.theta0_from_log, out_dir=args.out_dir)


if __name__ == "__main__":
    main()
