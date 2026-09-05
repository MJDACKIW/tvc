#!/usr/bin/env python3
"""Closed-loop TVC simulation using core/ via ctypes. See SPEC.md Section 6.

Reproduces the scenarios in paper/tvc_paper_figures.py (the script that actually generated
the paper's Figures 2-11 and Table 1; simulation/ is a separate, narrower hardware-IAE
codebase and was not their source) with the controller replaced: core/libtvccore's 1-state
Kalman filter + PD + rate limiter via ctypes, not the old script's 2-state Kalman filter and
Python PID.

Usage:
    python sim/run_sim.py baseline
    python sim/run_sim.py disturbance
    python sim/run_sim.py monte_carlo
    python sim/run_sim.py open_loop
    python sim/run_sim.py all       # runs the three acceptance scenarios and prints the
                                     # old-script vs new (this file) vs paper Table 1 report
"""
import argparse
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
SIM_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SIM_DIR))

import tvc_core  # noqa: E402
import tvc_params  # noqa: E402
from sensors import SensorModel  # noqa: E402
from vehicle import Vehicle  # noqa: E402

FIG_DIR = SIM_DIR / "figures"
DT_SIM = 0.001  # 1 kHz RK4 step, SPEC.md Section 6.1
PLOT_END_S = 3.5


def build_vehicle():
    return Vehicle(tvc_params.sim_overrides.vehicle, tvc_params.motor)


def run_simulation(theta0_deg, rate0_deg_s=0.0, open_loop=False, disturbance=None,
                    thrust_scale=1.0, seed=None, gyro_bias_dps=0.0, t_end=PLOT_END_S,
                    noise=True):
    """One single-axis closed-loop run. Mirrors paper/tvc_paper_figures.py's
    run_simulation() (same physics, same scenario knobs) with the controller swapped for
    core/ via ctypes (SPEC.md Section 6): a 1-state Kalman filter and a PD loop whose
    derivative acts on a raw finite difference of the angle estimate, rather than the old
    script's 2-state (angle + gyro bias) filter and Kalman-rate-based derivative.
    """
    vehicle = build_vehicle()
    ctl = tvc_params.control
    ov = tvc_params.sim_overrides

    control_dt = 1.0 / ctl.rate_hz
    axis = tvc_core.ControllerAxis(
        dt=control_dt, kp=ov.control.kp, ki=ctl.ki, kd=ov.control.kd,
        integral_clamp=ctl.integral_clamp_deg_s, max_deflection=ctl.max_deflection_deg,
        q=ov.kalman.q, r=ov.kalman.r, slew_deg_per_s=ov.control.servo_rate_lim_deg_per_s,
        p0=tvc_params.kalman.p0,
    )

    rng = np.random.default_rng(seed)
    sensors = SensorModel(
        gyro_noise_std_dps=ov.sensors.gyro_noise_std_dps if noise else 0.0,
        accel_noise_std_deg=ov.sensors.accel_noise_std_deg if noise else 0.0,
        gyro_drift_rate_dps_per_s=ov.sensors.gyro_drift_rate_dps_per_s,
        rng=rng, gyro_bias_dps=gyro_bias_dps,
    )

    dist_t0 = dist_t1 = None
    dist_torque_nm = 0.0
    if disturbance is not None:
        dist_t0 = disturbance["start_s"]
        dist_t1 = dist_t0 + disturbance["duration_s"]
        dist_torque_nm = disturbance["torque_Nm"]

    n = int(round(t_end / DT_SIM))
    next_ctrl_t = 0.0

    theta = float(theta0_deg)
    omega = float(rate0_deg_s)
    gimbal_deg = 0.0
    last_x_hat = 0.0
    last_cmd = 0.0

    log = {k: np.zeros(n) for k in (
        "time", "true_angle", "true_rate", "kalman_angle", "gimbal_cmd",
        "gimbal_actual", "accel_reading", "gyro_reading", "thrust")}

    for i in range(n):
        t = i * DT_SIM
        post_burn = t > vehicle.burn_time_s

        gyro_reading, accel_reading = sensors.sample(omega, theta, DT_SIM, post_burn)

        if t >= next_ctrl_t:
            next_ctrl_t += control_dt
            if open_loop:
                last_cmd = 0.0
                gimbal_deg = 0.0
            elif post_burn:
                # No thrust, no TVC authority: slew back to neutral rather than let the
                # PID chase post-burnout gyro drift (paper Section 6.3 / Table 1 note).
                last_cmd = 0.0
                max_step = ov.control.servo_rate_lim_deg_per_s * control_dt
                gimbal_deg = max(gimbal_deg - max_step, min(gimbal_deg + max_step, 0.0))
            else:
                out = axis.step(gyro_reading, accel_reading, accel_gate_ok=True)
                last_x_hat = out["x_hat"]
                last_cmd = out["u_cmd"]
                gimbal_deg = out["delta"]

        log["time"][i] = t
        log["true_angle"][i] = theta
        log["true_rate"][i] = omega
        log["kalman_angle"][i] = last_x_hat
        log["gimbal_cmd"][i] = last_cmd
        log["gimbal_actual"][i] = gimbal_deg
        log["accel_reading"][i] = accel_reading
        log["gyro_reading"][i] = gyro_reading
        log["thrust"][i] = vehicle.thrust_at(t, thrust_scale)

        extra_torque_nm = 0.0
        if dist_t0 is not None and dist_t0 <= t < dist_t1:
            extra_torque_nm = dist_torque_nm
        theta, omega = vehicle.rk4_step(t, DT_SIM, theta, omega, gimbal_deg,
                                         extra_torque_nm, thrust_scale)

    return log


# ---------------------------------------------------------------------------
# Metrics (our own operational definitions -- Table 1's settling/recovery times are a
# visual read of the published figures, not a programmatically defined threshold).
#
# Sensor noise is real here (accel_noise_std_deg = 2.5 deg, sim_overrides.sensors), so the
# physical angle itself carries a residual noise floor once "settled" -- a strict
# "never exceeds a tight band again" criterion is dominated by rare, isolated
# noise-driven excursions late in the run, which is not how a human reads a settled
# trace off a figure. _robust_settling instead asks for the band to hold at least
# tail_ok_frac of the time from T onward, and reports the first T where that holds.
# ---------------------------------------------------------------------------

def _robust_settling(t, series, threshold, tail_ok_frac=0.95):
    inside = (np.abs(series) <= threshold).astype(np.int64)
    n = len(t)
    frac_inside_from_i = np.cumsum(inside[::-1])[::-1] / np.arange(n, 0, -1)
    ok = np.where(frac_inside_from_i >= tail_ok_frac)[0]
    return float(t[ok[0]]) if len(ok) else float(t[-1])


def settling_time_s(t, theta, theta0=None, threshold_frac=0.15):
    """First time after which |theta| stays within threshold_frac * |theta0| for at
    least 95% of the remaining burn (a 15% band; see module docstring above)."""
    theta0 = abs(theta[0]) if theta0 is None else abs(theta0)
    threshold = max(threshold_frac * theta0, 0.05)
    return _robust_settling(t, theta, threshold)


def peak_deviation_and_recovery(t, theta_nominal, theta_disturbed, dist_t0, dist_t1,
                                 threshold_frac=0.15):
    """Peak |disturbed - nominal| deviation from dist_t0 onward, and the time from
    dist_t0 until that deviation robustly settles back within threshold_frac * peak
    (same robust criterion as settling_time_s)."""
    dev = theta_disturbed - theta_nominal
    peak = float(np.max(np.abs(dev[t >= dist_t0])))
    threshold = max(threshold_frac * peak, 0.02)
    after_dist = t >= dist_t1
    recovery_s = _robust_settling(t[after_dist], dev[after_dist], threshold) - dist_t0
    return peak, recovery_s


def run_monte_carlo(n_trials=100, seed_master=2024):
    """Two-axis Monte Carlo: paper Section 6.11 / Table 1. Same distributions and
    per-trial seeding as paper/tvc_paper_figures.py's fig10_monte_carlo()."""
    burn_time_s = tvc_params.motor.burn_time_s
    master = np.random.default_rng(seed_master)
    pitch_ok = yaw_ok = 0
    for trial in range(n_trials):
        th_p = master.uniform(0.5, 6.0)
        th_y = master.uniform(0.5, 6.0)
        tsc = master.uniform(0.92, 1.08)
        rp = run_simulation(theta0_deg=th_p, thrust_scale=tsc, seed=trial, t_end=burn_time_s)
        ry = run_simulation(theta0_deg=th_y, thrust_scale=tsc, seed=trial + 10000,
                             t_end=burn_time_s)
        if np.max(np.abs(rp["true_angle"])) < 15.0:
            pitch_ok += 1
        if np.max(np.abs(ry["true_angle"])) < 15.0:
            yaw_ok += 1
    return pitch_ok, yaw_ok, n_trials


# ---------------------------------------------------------------------------
# Output: .npz + PNG per scenario, params hash embedded (SPEC.md Section 6)
# ---------------------------------------------------------------------------

def _save_npz(name, **arrays):
    FIG_DIR.mkdir(parents=True, exist_ok=True)
    arrays["params_hash"] = tvc_params.PARAMS_HASH
    np.savez(FIG_DIR / f"{name}.npz", **arrays)


def _plot_angle_and_gimbal(name, title, series):
    """series: list of (time, angle_deg, label) to overlay on the top panel."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(9, 5))
    for t, angle, label in series:
        ax.plot(t, angle, lw=1.4, label=label)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Pitch angle (deg)")
    ax.set_title(f"{title}\nparams hash {tvc_params.PARAMS_HASH[:12]}")
    ax.legend(loc="upper right", fontsize=9)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    FIG_DIR.mkdir(parents=True, exist_ok=True)
    fig.savefig(FIG_DIR / f"{name}.png", dpi=150)
    plt.close(fig)


# ---------------------------------------------------------------------------
# Scenario commands
# ---------------------------------------------------------------------------

def cmd_baseline(theta0_deg=5.0, seed=42, quiet=False):
    log = run_simulation(theta0_deg=theta0_deg, seed=seed)
    # Settling is measured over the burn only: post-burnout the control loop is off by
    # design (paper Section 6.3) and gyro-only drift is expected, not a settling failure.
    burn = log["time"] <= tvc_params.motor.burn_time_s
    settling = settling_time_s(log["time"][burn], log["true_angle"][burn], theta0=theta0_deg)
    if not quiet:
        print(f"baseline: theta0={theta0_deg} deg, settling time = {settling:.3f} s "
              f"(15% robust band, paper Table 1: ~0.3 s)")
    _save_npz("baseline", **log)
    _plot_angle_and_gimbal(
        "baseline", "Baseline Closed-Loop Response (core/ via ctypes)",
        [(log["time"], log["true_angle"], "true pitch"),
         (log["time"], log["kalman_angle"], "Kalman estimate")])
    return log, settling


def cmd_disturbance(theta0_deg=5.0, seed=42, quiet=False):
    disturbance = {"torque_Nm": 0.12, "start_s": 0.6, "duration_s": 0.05}
    nominal = run_simulation(theta0_deg=theta0_deg, seed=seed)
    disturbed = run_simulation(theta0_deg=theta0_deg, seed=seed, disturbance=disturbance)
    # Recovery is measured over the burn only, same reasoning as cmd_baseline.
    burn = nominal["time"] <= tvc_params.motor.burn_time_s
    peak, recovery = peak_deviation_and_recovery(
        nominal["time"][burn], nominal["true_angle"][burn], disturbed["true_angle"][burn],
        disturbance["start_s"], disturbance["start_s"] + disturbance["duration_s"])
    if not quiet:
        print(f"disturbance: peak deviation = {peak:.3f} deg (paper Table 1: 1.3 deg), "
              f"recovery time = {recovery:.3f} s (paper Table 1: 0.4 s)")
    _save_npz("disturbance", nominal_angle=nominal["true_angle"],
              disturbed_angle=disturbed["true_angle"], time=nominal["time"])
    _plot_angle_and_gimbal(
        "disturbance", "Disturbance Rejection (core/ via ctypes)",
        [(disturbed["time"], disturbed["true_angle"], "with disturbance"),
         (nominal["time"], nominal["true_angle"], "nominal")])
    return peak, recovery


def cmd_monte_carlo(n_trials=100, quiet=False):
    pitch_ok, yaw_ok, n = run_monte_carlo(n_trials=n_trials)
    if not quiet:
        print(f"monte_carlo: pitch {pitch_ok}/{n}, yaw {yaw_ok}/{n} recovered "
              f"(paper Table 1: 100/100)")
    return pitch_ok, yaw_ok, n


def cmd_open_loop(theta0_deg=5.0, seed=42):
    closed = run_simulation(theta0_deg=theta0_deg, seed=seed)
    opened = run_simulation(theta0_deg=theta0_deg, seed=seed, open_loop=True)
    print(f"open_loop: closed-loop final angle = {closed['true_angle'][-1]:.3f} deg, "
          f"open-loop final angle = {opened['true_angle'][-1]:.3f} deg")
    _save_npz("open_loop", closed_angle=closed["true_angle"],
              open_angle=opened["true_angle"], time=closed["time"])
    _plot_angle_and_gimbal(
        "open_loop", "Open-Loop vs. Closed-Loop (core/ via ctypes)",
        [(closed["time"], closed["true_angle"], "closed-loop"),
         (opened["time"], opened["true_angle"], "open-loop")])


# ---------------------------------------------------------------------------
# Comparison report: old script (paper/tvc_paper_figures.py) vs new (this file) vs the
# paper's own published Table 1.
# ---------------------------------------------------------------------------

def _load_paper_script():
    paper_dir = REPO_ROOT / "paper"
    sys.path.insert(0, str(paper_dir))
    import tvc_paper_figures as paper_sim  # the actual source of Figs 2-11 / Table 1
    return paper_sim


def cmd_report():
    paper_sim = _load_paper_script()

    old_burn = paper_sim.BURN_TIME  # 2.44 s, same reasoning as cmd_baseline/cmd_disturbance

    old_baseline = paper_sim.run_simulation(theta0_deg=5.0, seed=42)
    ob_mask = old_baseline["time"] <= old_burn
    old_settling = settling_time_s(old_baseline["time"][ob_mask],
                                    old_baseline["true_angle"][ob_mask], theta0=5.0)

    old_nominal = paper_sim.run_simulation(theta0_deg=5.0, seed=42)
    old_disturbed = paper_sim.run_simulation(
        theta0_deg=5.0, seed=42,
        disturbance={"torque_Nm": 0.12, "start_s": 0.6, "duration_s": 0.05})
    od_mask = old_nominal["time"] <= old_burn
    old_peak, old_recovery = peak_deviation_and_recovery(
        old_nominal["time"][od_mask], old_nominal["true_angle"][od_mask],
        old_disturbed["true_angle"][od_mask], 0.6, 0.65)

    old_master = np.random.default_rng(2024)
    old_pitch_ok = old_yaw_ok = 0
    n_mc = 100
    for trial in range(n_mc):
        th_p = old_master.uniform(0.5, 6.0)
        th_y = old_master.uniform(0.5, 6.0)
        tsc = old_master.uniform(0.92, 1.08)
        rp = paper_sim.run_simulation(theta0_deg=th_p, thrust_scale=tsc, seed=trial,
                                       t_end=paper_sim.BURN_TIME)
        ry = paper_sim.run_simulation(theta0_deg=th_y, thrust_scale=tsc,
                                       seed=trial + 10000, t_end=paper_sim.BURN_TIME)
        if np.max(np.abs(rp["true_angle"])) < 15.0:
            old_pitch_ok += 1
        if np.max(np.abs(ry["true_angle"])) < 15.0:
            old_yaw_ok += 1

    _, new_settling = cmd_baseline(quiet=True)
    new_peak, new_recovery = cmd_disturbance(quiet=True)
    new_pitch_ok, new_yaw_ok, _ = cmd_monte_carlo(n_trials=n_mc, quiet=True)

    rows = [
        ("Settling time, baseline 5 deg tip-off (s)", old_settling, new_settling, 0.3),
        ("Peak angular deviation, disturbance (deg)", old_peak, new_peak, 1.3),
        ("Recovery time, disturbance (s)", old_recovery, new_recovery, 0.4),
        ("Monte Carlo recovered, pitch (/100)", old_pitch_ok, new_pitch_ok, 100),
        ("Monte Carlo recovered, yaw (/100)", old_yaw_ok, new_yaw_ok, 100),
    ]

    print()
    print("Phase 2 acceptance: old script (paper/tvc_paper_figures.py) vs new (core/ via "
          "ctypes) vs paper Table 1")
    print("-" * 100)
    print(f"{'Metric':<44} {'Old script':>12} {'New (sim/)':>12} {'Paper Table 1':>14} "
          f"{'New vs paper':>12}")
    all_within_10pct = True
    for label, old, new, paper in rows:
        if paper != 0:
            pct = 100.0 * (new - paper) / paper
        else:
            pct = 0.0
        within = abs(pct) <= 10.0
        all_within_10pct &= within
        flag = "OK" if within else "OUT OF RANGE"
        print(f"{label:<44} {old:>12.3f} {new:>12.3f} {paper:>14.3f} "
              f"{pct:>+10.1f}% {flag}")
    print("-" * 100)
    print("PASS: all metrics within 10% of paper Table 1" if all_within_10pct
          else "FAIL: at least one metric is outside 10% of paper Table 1")
    print()
    return all_within_10pct


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("scenario", choices=[
        "baseline", "disturbance", "monte_carlo", "open_loop", "all"])
    args = parser.parse_args()

    if args.scenario == "baseline":
        cmd_baseline()
    elif args.scenario == "disturbance":
        cmd_disturbance()
    elif args.scenario == "monte_carlo":
        cmd_monte_carlo()
    elif args.scenario == "open_loop":
        cmd_open_loop()
    elif args.scenario == "all":
        ok = cmd_report()
        sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
