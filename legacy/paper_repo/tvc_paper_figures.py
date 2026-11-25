#!/usr/bin/env python3
"""
tvc_paper_figures.py

Self-contained regeneration of the 10 publication figures for the paper
"Design and Validation of a Thrust Vector Controlled Model Rocket Using
PID-Kalman Architecture".

Running `python tvc_paper_figures.py` creates a `figures/` directory next to
this file and writes fig01 through fig10 as 150 dpi PNG files.

Dependencies: numpy, matplotlib. (scipy is permitted but not required.)
"""

import math
import os
import time

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.lines import Line2D
from matplotlib.patches import Patch

# ---------------------------------------------------------------------------
# Physical parameters
# ---------------------------------------------------------------------------

# Motor: Estes E12-4, NAR certified data from ThrustCurve.org
BURN_TIME    = 2.44        # seconds
PEAK_THRUST  = 33.0        # N at t = 0.287 s
PEAK_TIME    = 0.287       # s
MEAN_THRUST  = 11.12       # N
PLOT_END     = 3.5         # seconds, all time series plots end here

# NAR certified data points (time_s, thrust_N)
THRUST_DATA = [
    (0.000,  0.0),
    (0.050,  5.1),
    (0.100, 10.0),
    (0.150, 24.1),
    (0.200, 31.5),
    (0.287, 33.0),   # peak
    (0.300, 30.0),
    (0.350, 17.1),
    (0.400, 14.6),
    (0.450, 13.2),
    (0.500, 11.7),
    (0.600, 11.2),
    (0.700, 11.2),
    (0.800, 10.5),
    (0.900, 10.1),
    (1.000,  9.8),
    (1.200,  9.8),
    (1.400,  9.7),
    (1.600,  9.7),
    (1.800,  9.7),
    (2.000,  9.8),
    (2.100,  9.8),
    (2.200,  9.7),
    (2.300,  9.5),
    (2.380,  6.1),
    (2.440,  0.0),
]

# Vehicle
ROCKET_MASS     = 0.661       # kg (dry 0.600 + propellant 0.0612)
MOI             = 0.0231      # kg m^2 (thin rod approximation, L = 0.648 m)
MOMENT_ARM      = 0.288       # m (CoM to nozzle distance, r)
CP_OFFSET       = 0.173       # m (CP-CG offset for aero damping, r * 0.6)
AIR_DENSITY     = 1.225       # kg/m^3
ROCKET_RADIUS   = 0.022       # m (body radius, ~44 mm diameter)
AERO_DAMP_COEFF = 0.5         # dimensionless

# Control
KP              = 8.5
KI              = 0.0         # pure PD
KD              = 1.2
CONTROL_HZ      = 150         # Hz
DT_SIM          = 0.001       # s (1 ms RK4 step)
GIMBAL_LIMIT    = 10.0        # degrees hard stop
SERVO_RATE_LIM  = 300.0       # deg/s

# Kalman filter noise
Q_ANGLE         = 0.001
Q_RATE          = 0.003
R_MEASURE       = 0.03
GYRO_NOISE_STD  = 0.5         # deg/s
ACCEL_NOISE_STD = 2.5         # deg (accelerometer noise, angle equivalent)
GYRO_DRIFT_RATE = 0.8         # deg/s per s, post burnout gyro drift rate

# Plotting style
BURN_SHADE_COLOR   = '#fff3e0'
BURN_SHADE_ALPHA   = 0.6
BURNOUT_LINE_COLOR = '#888888'
BURNOUT_LINESTYLE  = '--'

A_CS = math.pi * ROCKET_RADIUS ** 2   # cross sectional area, m^2

FIG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "figures")

_TD_T = np.array([p[0] for p in THRUST_DATA])
_TD_F = np.array([p[1] for p in THRUST_DATA])


# ---------------------------------------------------------------------------
# Motor model
# ---------------------------------------------------------------------------

def get_thrust(t):
    """Thrust in N at time t (scalar or array). Zero after burnout."""
    if np.isscalar(t):
        if t > BURN_TIME:
            return 0.0
        return float(np.interp(t, _TD_T, _TD_F))
    t = np.asarray(t, dtype=float)
    f = np.interp(t, _TD_T, _TD_F)
    f[t > BURN_TIME] = 0.0
    return f


# ---------------------------------------------------------------------------
# Kalman filter (2-state: angle, gyro bias)
# ---------------------------------------------------------------------------

def kalman_update(state, P, gyro_rate_dps, accel_angle_deg, dt, use_accel=True):
    """One predict + update cycle of the linear 2-state Kalman filter.

    state = [angle_deg, gyro_bias_dps], P is the 2x2 covariance.
    Predict uses the gyro as control input (F = [[1, -dt], [0, 1]],
    B = [dt, 0]); update uses the accelerometer angle (H = [1, 0],
    R = R_MEASURE). Q_ANGLE and Q_RATE are continuous noise densities and
    are scaled by dt, matching the standard MPU6050 attitude Kalman filter
    these constants are tuned for. When use_accel is False (post burnout,
    the accelerometer has no gravity reference) the filter runs
    predict-only on the gyro.
    Returns (new_state, new_P).
    """
    angle, bias = state
    p00, p01 = P[0, 0], P[0, 1]
    p10, p11 = P[1, 0], P[1, 1]

    # Predict: x = F x + B u
    angle = angle + dt * (gyro_rate_dps - bias)

    # P = F P F' + Q dt
    n00 = p00 + dt * (dt * p11 - p01 - p10 + Q_ANGLE)
    n01 = p01 - dt * p11
    n10 = p10 - dt * p11
    n11 = p11 + Q_RATE * dt

    if use_accel:
        y = accel_angle_deg - angle
        s = n00 + R_MEASURE
        k0 = n00 / s
        k1 = n10 / s
        angle += k0 * y
        bias += k1 * y
        n11 = n11 - k1 * n01
        n10 = n10 - k1 * n00
        n01 = (1.0 - k0) * n01
        n00 = (1.0 - k0) * n00

    return np.array([angle, bias]), np.array([[n00, n01], [n10, n11]])


# ---------------------------------------------------------------------------
# PID controller (pure PD here since KI = 0)
# ---------------------------------------------------------------------------

def pid_update(error, prev_error, integral, dt, prev_output,
               gimbal_limit=GIMBAL_LIMIT):
    """PID step with anti-windup, servo rate limiting and hard gimbal clamp.

    Returns (achieved_gimbal_deg, new_integral, raw_output_deg).
    """
    proportional = KP * error

    integral = integral + KI * error * dt
    integral = max(-gimbal_limit, min(gimbal_limit, integral))

    derivative = KD * (error - prev_error) / dt

    raw = proportional + integral + derivative

    # Servo slew rate limit relative to the previously achieved position
    max_step = SERVO_RATE_LIM * dt
    out = max(prev_output - max_step, min(prev_output + max_step, raw))

    # Mechanical hard stop
    out = max(-gimbal_limit, min(gimbal_limit, out))

    return out, integral, raw


# ---------------------------------------------------------------------------
# Rotational dynamics
# ---------------------------------------------------------------------------

def _alpha(omega_dps, F_thrust, gimbal_rad, tau_extra, arm):
    """Angular acceleration in deg/s^2.

    Torques: TVC vectoring, aerodynamic damping (opposes rotation,
    proportional to dynamic pressure), and any external torque.
    """
    tau_tvc = F_thrust * arm * math.sin(gimbal_rad)
    # Approximate flight speed from thrust to weight, gives q during burn only
    q_dyn = AIR_DENSITY * F_thrust / ROCKET_MASS   # 0.5 * rho * (2F/m)
    tau_damp = -AERO_DAMP_COEFF * math.radians(omega_dps) * q_dyn * A_CS * CP_OFFSET
    return math.degrees((tau_tvc + tau_damp + tau_extra) / MOI)


def run_simulation(theta0_deg, rate0_deg_s=0.0, open_loop=False,
                   disturbance=None, wind_mag=0.0, thrust_scale=1.0,
                   gimbal_limit_override=None, seed=None,
                   moment_arm_scale=1.0, t_end=PLOT_END, noise=True,
                   abort_angle=None):
    """Run one closed (or open) loop flight and return logged arrays.

    disturbance: dict with keys torque_Nm, start_s, duration_s.
    abort_angle: stop integrating once |angle| exceeds this (used by the
    stability boundary sweep, classification only).
    """
    dt = DT_SIM
    n = int(round(t_end / dt))
    ctrl_every = int(round(1.0 / (CONTROL_HZ * dt)))
    dt_ctrl = ctrl_every * dt
    glim = GIMBAL_LIMIT if gimbal_limit_override is None else float(gimbal_limit_override)
    arm = MOMENT_ARM * moment_arm_scale

    # Thrust precomputed on a half-step grid for the RK4 stages
    t_half = np.arange(2 * n + 1) * (0.5 * dt)
    F_half = np.interp(t_half, _TD_T, _TD_F) * thrust_scale
    F_half[t_half > BURN_TIME] = 0.0

    rng = np.random.default_rng(seed)
    if noise:
        gyro_noise = rng.normal(0.0, GYRO_NOISE_STD, n)
        accel_noise = rng.normal(0.0, ACCEL_NOISE_STD, n)
    else:
        gyro_noise = np.zeros(n)
        accel_noise = np.zeros(n)

    log = {k: np.zeros(n) for k in (
        'time', 'true_angle', 'true_rate', 'kalman_angle', 'kalman_rate',
        'gimbal_cmd', 'gimbal_actual', 'accel_reading', 'gyro_reading',
        'thrust')}
    log['time'] = np.arange(n) * dt

    theta = float(theta0_deg)
    omega = float(rate0_deg_s)
    kal_state = np.array([0.0, 0.0])
    kal_P = np.eye(2)
    kal_rate = 0.0
    integral = 0.0
    gimbal = 0.0       # achieved gimbal angle, deg
    cmd = 0.0          # commanded (clamped PD output), deg
    drift = 0.0        # accumulated gyro drift, deg/s

    dist_on = disturbance is not None
    if dist_on:
        d_tau = disturbance['torque_Nm']
        d_t0 = disturbance['start_s']
        d_t1 = d_t0 + disturbance['duration_s']

    for i in range(n):
        t = i * dt
        post_burn = t > BURN_TIME
        if post_burn:
            drift += GYRO_DRIFT_RATE * dt

        gyro_reading = omega + gyro_noise[i] + drift
        accel_reading = theta + accel_noise[i]

        if i % ctrl_every == 0:
            kal_state, kal_P = kalman_update(kal_state, kal_P, gyro_reading,
                                             accel_reading, dt_ctrl,
                                             use_accel=not post_burn)
            kal_rate = gyro_reading - kal_state[1]
            if not open_loop:
                if post_burn:
                    # TVC has no authority without thrust: center the
                    # gimbal at burnout (servo slew rate still applies)
                    cmd = 0.0
                    step = SERVO_RATE_LIM * dt_ctrl
                    gimbal = max(gimbal - step, min(gimbal + step, 0.0))
                else:
                    error = 0.0 - kal_state[0]
                    # D term from the Kalman rate estimate: a raw finite
                    # difference of the noisy angle estimate at 150 Hz
                    # would swamp the command with derivative noise
                    d_error = -kal_rate
                    prev_error = error - d_error * dt_ctrl
                    gimbal, integral, raw = pid_update(error, prev_error,
                                                       integral, dt_ctrl,
                                                       gimbal, glim)
                    cmd = max(-glim, min(glim, raw))

        tau_extra = wind_mag * CP_OFFSET
        if dist_on and d_t0 <= t < d_t1:
            tau_extra += d_tau

        log['true_angle'][i] = theta
        log['true_rate'][i] = omega
        log['kalman_angle'][i] = kal_state[0]
        log['kalman_rate'][i] = kal_rate
        log['gimbal_cmd'][i] = cmd
        log['gimbal_actual'][i] = gimbal
        log['accel_reading'][i] = accel_reading
        log['gyro_reading'][i] = gyro_reading
        log['thrust'][i] = F_half[2 * i]

        # RK4 step, gimbal held over the step (zero order hold)
        g_rad = math.radians(gimbal)
        F0, F1, F2 = F_half[2 * i], F_half[2 * i + 1], F_half[2 * i + 2]
        k1t = omega
        k1w = _alpha(omega, F0, g_rad, tau_extra, arm)
        k2t = omega + 0.5 * dt * k1w
        k2w = _alpha(k2t, F1, g_rad, tau_extra, arm)
        k3t = omega + 0.5 * dt * k2w
        k3w = _alpha(k3t, F1, g_rad, tau_extra, arm)
        k4t = omega + dt * k3w
        k4w = _alpha(k4t, F2, g_rad, tau_extra, arm)
        theta += dt * (k1t + 2.0 * k2t + 2.0 * k3t + k4t) / 6.0
        omega += dt * (k1w + 2.0 * k2w + 2.0 * k3w + k4w) / 6.0

        if abort_angle is not None and abs(theta) > abort_angle:
            # Fill the remainder with the final value so max() stays valid
            log['true_angle'][i + 1:] = theta
            log['true_rate'][i + 1:] = omega
            break

    return log


# ---------------------------------------------------------------------------
# Shared figure helpers
# ---------------------------------------------------------------------------

def _shade_burn(ax, label=None):
    ax.axvspan(0.0, BURN_TIME, facecolor=BURN_SHADE_COLOR,
               alpha=BURN_SHADE_ALPHA, zorder=0, label=label)


def _burnout_line(ax, label=None):
    ax.axvline(BURN_TIME, color=BURNOUT_LINE_COLOR,
               linestyle=BURNOUT_LINESTYLE, lw=1.2, zorder=1, label=label)


def _grid(ax):
    ax.grid(True, linestyle='--', alpha=0.5)


def _save(fig, name):
    path = os.path.join(FIG_DIR, name)
    fig.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)


WHEAT_BOX = dict(boxstyle='round', facecolor='wheat', alpha=0.8)


# ---------------------------------------------------------------------------
# Figures
# ---------------------------------------------------------------------------

def fig01_thrust_curve():
    tt = np.linspace(0.0, PLOT_END, 1401)
    ft = get_thrust(tt)

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.fill_between(tt, ft, color='#cfe8fc', alpha=0.55, zorder=1)
    ax.plot(tt, ft, color='tab:blue', lw=2.0, zorder=3,
            label='Interpolated thrust curve (numpy.interp)')
    pts_t = [p[0] for p in THRUST_DATA if p[0] <= BURN_TIME]
    pts_f = [p[1] for p in THRUST_DATA if p[0] <= BURN_TIME]
    ax.scatter(pts_t, pts_f, color='tab:orange', s=30, zorder=4,
               label='NAR certified data points (ThrustCurve.org)')
    ax.axhline(MEAN_THRUST, color='tab:blue', ls='--', lw=1.2,
               label='Mean thrust = 11.12 N')
    ax.axvline(BURN_TIME, color=BURNOUT_LINE_COLOR, ls='--', lw=1.2,
               label='Burnout = 2.44 s')
    ax.annotate('Peak: 33.0 N\n@ t = 0.287 s', xy=(PEAK_TIME, PEAK_THRUST),
                xytext=(0.85, 30.0), fontsize=10, color='dimgrey',
                arrowprops=dict(arrowstyle='->', color='grey'))
    fig.suptitle('Estes E12-4 Thrust Curve — NAR Certified Data (ThrustCurve.org)',
                 fontsize=13, y=0.99)
    ax.set_title('Peak = 33.0 N  |  Mean = 11.12 N  |  Burn time = 2.44 s',
                 fontsize=10, color='dimgrey')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Thrust (N)')
    ax.set_xlim(0.0, PLOT_END)
    ax.set_ylim(0.0, 36.0)
    _grid(ax)
    ax.legend(loc='upper right', fontsize=9)
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.97))
    _save(fig, 'fig01_thrust_curve.png')


def fig02_baseline_response():
    res = run_simulation(theta0_deg=5.0, seed=42)
    t = res['time']

    fig, axs = plt.subplots(3, 1, figsize=(8, 10), sharex=True)

    ax = axs[0]
    _shade_burn(ax)
    _burnout_line(ax)
    ax.plot(t, res['true_angle'], color='tab:blue', lw=1.6, label='True angle')
    ax.plot(t, res['kalman_angle'], color='tab:orange', ls='--', lw=1.4,
            label='Kalman estimate')
    i_ann = np.searchsorted(t, 3.25)
    ax.annotate('Gyro drift post-burnout\n(no magnetometer reference)',
                xy=(3.25, res['kalman_angle'][i_ann]), xytext=(1.15, 3.1),
                fontsize=9, bbox=WHEAT_BOX,
                arrowprops=dict(arrowstyle='->', color='grey'))
    ax.set_ylabel('Pitch Angle (deg)')
    ax.set_title('Attitude Stabilisation Using TVC — Baseline Closed-Loop Response')
    _grid(ax)
    ax.legend(loc='upper right', fontsize=9)

    ax = axs[1]
    _shade_burn(ax)
    _burnout_line(ax)
    ax.plot(t, res['true_rate'], color='tab:green', lw=1.4,
            label='True angular rate')
    ax.set_ylabel('Angular Rate (deg/s)')
    _grid(ax)
    ax.legend(loc='lower right', fontsize=9)

    ax = axs[2]
    _shade_burn(ax)
    _burnout_line(ax)
    ax.plot(t, res['gimbal_actual'], color='darkred', lw=1.4,
            label='Actual deflection')
    ax.plot(t, res['gimbal_cmd'], color='purple', ls='--', lw=1.0, alpha=0.7,
            label='Commanded')
    ax.axhline(GIMBAL_LIMIT, color='red', ls=':', lw=1.3,
               label='±Mechanical limit')
    ax.axhline(-GIMBAL_LIMIT, color='red', ls=':', lw=1.3)
    ax.set_ylabel('Gimbal Angle (deg)')
    ax.set_xlabel('Time (s)')
    ax.set_xlim(0.0, PLOT_END)
    ax.set_ylim(-12.0, 12.0)
    _grid(ax)
    ax.legend(loc='lower right', fontsize=9)

    fig.tight_layout()
    _save(fig, 'fig02_baseline_response.png')


def fig03_open_vs_closed():
    closed = run_simulation(theta0_deg=5.0, open_loop=False, seed=42)
    opened = run_simulation(theta0_deg=5.0, open_loop=True, seed=42)
    t = closed['time']

    fig, ax = plt.subplots(figsize=(10, 5))
    _shade_burn(ax, label='Motor burn')
    _burnout_line(ax)
    ax.plot(t, closed['true_angle'], color='tab:blue', lw=1.8,
            label='Closed-loop (TVC active)')
    ax.plot(t, opened['true_angle'], color='tab:orange', ls='--', lw=1.8,
            label='Open-loop (no control)')
    ax.set_title('Open-Loop vs. Closed-Loop Attitude Response\n'
                 '(Finless Configuration, θ₀ = 5°)')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Pitch Angle (deg)')
    ax.set_xlim(0.0, PLOT_END)
    _grid(ax)
    ax.legend(loc='center right', fontsize=9)
    fig.tight_layout()
    _save(fig, 'fig03_open_vs_closed.png')


def fig04_disturbance():
    nominal = run_simulation(theta0_deg=5.0, seed=42)
    disturbed = run_simulation(
        theta0_deg=5.0,
        disturbance={'torque_Nm': 0.12, 'start_s': 0.6, 'duration_s': 0.05},
        seed=42)
    t = nominal['time']

    fig, ax = plt.subplots(figsize=(10, 5))
    _shade_burn(ax)
    _burnout_line(ax)
    ax.axvspan(0.6, 0.65, facecolor='#f48fb1', alpha=0.45, zorder=1,
               label='Disturbance window')
    ax.plot(t, disturbed['true_angle'], color='tab:blue', lw=1.8,
            label='With disturbance (τ = 0.12 N·m, 50 ms)')
    ax.plot(t, nominal['true_angle'], color='tab:orange', ls='--', lw=1.6,
            label='Nominal (no disturbance)')
    ax.set_title('Disturbance Rejection Response Under TVC Control')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Pitch Angle (deg)')
    ax.set_xlim(0.0, PLOT_END)
    _grid(ax)
    ax.legend(loc='upper right', fontsize=9)

    # Inset: the rejection is strong enough that the two traces overlap at
    # full scale, so show the deviation from nominal around the impulse
    axin = ax.inset_axes([0.56, 0.13, 0.40, 0.36])
    seg = (t >= 0.50) & (t <= 1.20)
    axin.plot(t[seg], disturbed['true_angle'][seg] - nominal['true_angle'][seg],
              color='tab:blue', lw=1.2)
    axin.axvspan(0.6, 0.65, facecolor='#f48fb1', alpha=0.45)
    axin.axhline(0.0, color='grey', lw=0.6)
    axin.set_title('Deviation from nominal (deg)', fontsize=8)
    axin.tick_params(labelsize=7)
    axin.grid(True, linestyle='--', alpha=0.4)

    fig.tight_layout()
    _save(fig, 'fig04_disturbance.png')

    dev = np.max(np.abs(disturbed['true_angle'] - nominal['true_angle'])
                 [t >= 0.6])
    return dev


def fig05_control_effort():
    res = run_simulation(theta0_deg=5.0, seed=42)
    t = res['time']
    act = res['gimbal_actual']

    fig, axs = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    ax = axs[0]
    _shade_burn(ax, label='Motor burn')
    _burnout_line(ax)
    ax.plot(t, act, color='tab:blue', lw=1.4,
            label='Actual (servo rate-limited)')
    ax.plot(t, res['gimbal_cmd'], color='tab:green', ls='--', lw=1.0,
            alpha=0.75, label='Commanded (PD output)')
    ax.axhline(GIMBAL_LIMIT, color='red', ls=':', lw=1.3,
               label='±Saturation limit')
    ax.axhline(-GIMBAL_LIMIT, color='red', ls=':', lw=1.3)
    ax.set_title('Control Effort — Commanded vs. Achieved Gimbal Deflection')
    ax.set_ylabel('Gimbal Angle (deg)')
    ax.set_ylim(-12.0, 12.0)
    _grid(ax)
    ax.legend(loc='lower right', fontsize=9)

    sat_pct = 100.0 * np.cumsum(np.abs(act) >= 9.9) / np.arange(1, len(act) + 1)
    ax = axs[1]
    _shade_burn(ax)
    _burnout_line(ax)
    ax.plot(t, sat_pct, color='red', lw=1.6,
            label='Cumulative fraction at |δ| ≥ 9.9°')
    ax.set_title('Actuator Saturation — Fraction of Time at Mechanical Limit')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Cumulative Saturation (%)')
    ax.set_xlim(0.0, PLOT_END)
    top = max(0.05, float(sat_pct.max()) * 1.25)
    ax.set_ylim(-0.05 if top <= 0.05 else -0.03 * top, top)
    if sat_pct.max() > 1.0:
        i_pk = int(np.argmax(sat_pct))
        ax.annotate('Saturation confined to the initial\n'
                    f'tip-over correction (t < {t[np.where(np.abs(act) >= 9.9)[0][-1]]:.2f} s);\n'
                    f'cumulative share decays to {sat_pct[-1]:.1f}% of flight',
                    xy=(t[i_pk], sat_pct[i_pk]), xytext=(0.75, 0.62 * top),
                    fontsize=9, bbox=WHEAT_BOX,
                    arrowprops=dict(arrowstyle='->', color='grey'))
    _grid(ax)
    ax.legend(loc='upper right', fontsize=9)

    fig.tight_layout()
    _save(fig, 'fig05_control_effort.png')
    return float(sat_pct[-1])


def fig06_sensitivity():
    fig, axs = plt.subplots(1, 3, figsize=(14, 5), sharey=True)

    # (a) thrust scale
    ax = axs[0]
    _shade_burn(ax)
    _burnout_line(ax)
    for scale, color, label in [(0.70, 'tab:blue', 'Thrust variation: -30%'),
                                (1.00, 'tab:orange', 'Nominal'),
                                (1.30, 'tab:green', '+30%')]:
        r = run_simulation(theta0_deg=5.0, thrust_scale=scale, seed=42)
        ax.plot(r['time'], r['true_angle'], color=color, lw=1.5, label=label)
    ax.set_title('(a) Sensitivity to Thrust', fontsize=11)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Pitch Angle (deg)')
    _grid(ax)
    ax.legend(loc='upper right', fontsize=9)

    # (b) moment arm scale
    ax = axs[1]
    _shade_burn(ax)
    _burnout_line(ax)
    for scale, color, label in [(0.80, 'tab:blue', 'L variation: -20% L'),
                                (1.00, 'tab:orange', 'Nominal'),
                                (1.20, 'tab:green', '+20% L')]:
        r = run_simulation(theta0_deg=5.0, moment_arm_scale=scale, seed=42)
        ax.plot(r['time'], r['true_angle'], color=color, lw=1.5, label=label)
    ax.set_title('(b) Sensitivity to CoM-Nozzle\nDistance (L)', fontsize=11)
    ax.set_xlabel('Time (s)')
    _grid(ax)
    ax.legend(loc='upper right', fontsize=9)

    # (c) gimbal limit
    ax = axs[2]
    _shade_burn(ax)
    _burnout_line(ax)
    for lim, color, label in [(6.0, 'tab:blue', '6°'),
                              (8.0, 'tab:orange', '8°'),
                              (10.0, 'tab:green', '10° (design)'),
                              (12.0, 'tab:red', '12°')]:
        r = run_simulation(theta0_deg=5.0, gimbal_limit_override=lim, seed=42)
        ax.plot(r['time'], r['true_angle'], color=color, lw=1.5, label=label)
    ax.set_title('(c) Sensitivity to Max\nGimbal Angle', fontsize=11)
    ax.set_xlabel('Time (s)')
    _grid(ax)
    ax.legend(loc='upper right', fontsize=9)

    for ax in axs:
        ax.set_xlim(0.0, PLOT_END)
    axs[0].set_ylim(-1.0, 5.5)

    fig.suptitle('Parametric Sensitivity Analysis — Pitch Response  (θ₀ = 5°)',
                 fontsize=13)
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.95))
    _save(fig, 'fig06_sensitivity.png')


def fig07_control_authority():
    tt = np.linspace(0.0, PLOT_END, 1401)
    tau_max = get_thrust(tt) * MOMENT_ARM * math.sin(math.radians(GIMBAL_LIMIT))
    tau_dist = 0.025

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(tt, tau_max, color='tab:blue', lw=2.0,
            label=r'$\tau_{max} = F_T \cdot r \cdot \sin(\delta_{max})$')
    ax.axhline(tau_dist, color='red', ls='--', lw=1.5,
               label='Typical disturbance torque (0.025 N·m)')
    ax.fill_between(tt, tau_max, tau_dist, where=tau_max > tau_dist,
                    color='#c8e6c9', alpha=0.8, interpolate=True,
                    label='Control authority margin')
    ax.fill_between(tt, tau_max, tau_dist, where=tau_max <= tau_dist,
                    color='#f8bbd0', alpha=0.8, interpolate=True,
                    label='Insufficient authority')
    ax.axvline(BURN_TIME, color=BURNOUT_LINE_COLOR, ls='--', lw=1.2,
               label='Burnout')
    peak_tau = PEAK_THRUST * MOMENT_ARM * math.sin(math.radians(GIMBAL_LIMIT))
    ax.annotate(f'Peak: {peak_tau:.2f} N·m\n@ t = 0.287 s',
                xy=(PEAK_TIME, peak_tau), xytext=(0.85, 1.45),
                fontsize=10, color='dimgrey',
                arrowprops=dict(arrowstyle='->', color='grey'))
    ax.set_title('TVC Corrective Torque Authority vs. Time')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Torque (N·m)')
    ax.set_xlim(0.0, PLOT_END)
    ax.set_ylim(0.0, 1.85)
    _grid(ax)
    ax.legend(loc='upper right', fontsize=9)
    fig.tight_layout()
    _save(fig, 'fig07_control_authority.png')


def fig08_kalman_performance():
    res = run_simulation(theta0_deg=5.0, seed=42)
    t = res['time']

    fig, ax = plt.subplots(figsize=(10, 5))
    _shade_burn(ax, label='Motor burn')
    _burnout_line(ax)
    # The raw trace is the logged accel_reading array from the simulation,
    # not re-synthesized noise
    ax.plot(t, res['accel_reading'], color='red', lw=0.5, alpha=0.55,
            label='Raw accelerometer (noisy)')
    ax.plot(t, res['true_angle'], color='black', lw=1.8,
            label='Ground truth (no noise)')
    ax.plot(t, res['kalman_angle'], color='tab:blue', ls='--', lw=1.4,
            label='Kalman estimate')
    i_ann = np.searchsorted(t, 3.3)
    ax.annotate('Gyro drift post-burnout\n(no magnetometer reference;\n'
                'Kalman relies solely on gyro)',
                xy=(3.3, res['kalman_angle'][i_ann]), xytext=(1.35, 6.2),
                fontsize=9, bbox=WHEAT_BOX,
                arrowprops=dict(arrowstyle='->', color='grey'))
    ax.set_title('Kalman Filter Performance — Noise Rejection vs. Raw Measurement\n'
                 '(MPU-6050 noise model, Section 3.2)')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Pitch Angle (deg)')
    ax.set_xlim(0.0, PLOT_END)
    _grid(ax)
    handles, labels = ax.get_legend_handles_labels()
    order = ['Ground truth (no noise)', 'Kalman estimate',
             'Raw accelerometer (noisy)', 'Motor burn']
    h_by_label = dict(zip(labels, handles))
    ax.legend([h_by_label[k] for k in order], order, loc='upper right',
              fontsize=9)
    fig.tight_layout()
    _save(fig, 'fig08_kalman_performance.png')


def fig09_stability_boundary():
    n_pts = 40
    thetas = np.linspace(0.0, 20.0, n_pts)
    rates = np.linspace(-50.0, 50.0, n_pts)
    recovered = np.zeros((n_pts, n_pts))

    for j, r0 in enumerate(rates):
        for i, th0 in enumerate(thetas):
            r = run_simulation(theta0_deg=th0, rate0_deg_s=r0, noise=False,
                               t_end=BURN_TIME, abort_angle=45.0)
            recovered[j, i] = 1.0 if np.max(np.abs(r['true_angle'])) < 15.0 else 0.0
        if (j + 1) % 10 == 0:
            print(f'    boundary sweep: {j + 1}/{n_pts} rows')

    fig, ax = plt.subplots(figsize=(8, 6))
    cmap = ListedColormap(['#f8bbd0', '#c8e6c9'])
    ax.pcolormesh(thetas, rates, recovered, cmap=cmap, vmin=0.0, vmax=1.0,
                  shading='nearest')
    ax.contour(thetas, rates, recovered, levels=[0.5], colors='black',
               linewidths=1.6)
    ax.axvline(GIMBAL_LIMIT, color='tab:blue', ls='--', lw=1.6)
    ax.set_title('TVC Controllability Boundary\n'
                 r'$\theta_0$ vs. $\dot{\theta}_0$ — Recoverable vs. '
                 'Divergent Trajectories')
    ax.set_xlabel('Initial Pitch Angle (deg)')
    ax.set_ylabel('Initial Angular Rate (deg/s)')
    handles = [
        Patch(facecolor='#c8e6c9', edgecolor='grey',
              label='Stable recovery (|θ| < 15° during burn)'),
        Patch(facecolor='#f8bbd0', edgecolor='grey',
              label='Unrecoverable divergence'),
        Line2D([], [], color='tab:blue', ls='--',
               label='Max gimbal limit (10.0°)'),
    ]
    ax.legend(handles=handles, loc='upper right', fontsize=9,
              framealpha=0.95)
    _grid(ax)
    fig.tight_layout()
    _save(fig, 'fig09_stability_boundary.png')


def fig10_monte_carlo():
    n_trials = 100
    pitch_runs, yaw_runs = [], []
    pitch_ok, yaw_ok = [], []
    master = np.random.default_rng(2024)

    for trial in range(n_trials):
        th_p = master.uniform(0.5, 6.0)
        th_y = master.uniform(0.5, 6.0)
        tsc = master.uniform(0.92, 1.08)
        rp = run_simulation(theta0_deg=th_p, thrust_scale=tsc, seed=trial)
        ry = run_simulation(theta0_deg=th_y, thrust_scale=tsc,
                            seed=trial + 10000)
        pitch_runs.append(rp['true_angle'])
        yaw_runs.append(ry['true_angle'])
        burn = rp['time'] <= BURN_TIME
        pitch_ok.append(np.max(np.abs(rp['true_angle'][burn])) < 15.0)
        yaw_ok.append(np.max(np.abs(ry['true_angle'][burn])) < 15.0)
        if (trial + 1) % 25 == 0:
            print(f'    Monte Carlo: {trial + 1}/{n_trials} trials')

    t = np.arange(len(pitch_runs[0])) * DT_SIM
    fig, axs = plt.subplots(1, 2, figsize=(14, 5), sharey=True)

    for ax, runs, oks, axis_name in ((axs[0], pitch_runs, pitch_ok, 'Pitch'),
                                     (axs[1], yaw_runs, yaw_ok, 'Yaw')):
        runs = np.asarray(runs)
        n_rec = int(np.sum(oks))
        _shade_burn(ax)
        _burnout_line(ax)
        for k in range(n_trials):
            color = 'tab:blue' if oks[k] else 'tab:red'
            ax.plot(t, runs[k], color=color, lw=0.5, alpha=0.25)
        p05 = np.percentile(runs, 5, axis=0)
        p95 = np.percentile(runs, 95, axis=0)
        ax.fill_between(t, p05, p95, color='grey', alpha=0.35)
        ax.plot(t, runs.mean(axis=0), color='black', lw=2.0)
        ax.set_title(f'Monte Carlo — {axis_name} Axis\n'
                     f'({n_rec}/{n_trials} trials recovered, '
                     'θ₀ ∈ [0.5°, 6°], T_scale ∈ [±8%])', fontsize=11)
        ax.set_xlabel('Time (s)')
        ax.set_xlim(0.0, PLOT_END)
        _grid(ax)
        handles = [Line2D([], [], color='tab:blue', lw=1.2, label='Recovered'),
                   Line2D([], [], color='tab:red', lw=1.2, label='Diverged'),
                   Line2D([], [], color='black', lw=2.0, label='Mean'),
                   Patch(facecolor='grey', alpha=0.35, label='5-95th %ile')]
        ax.legend(handles=handles, loc='upper right', fontsize=9)
    axs[0].set_ylabel('Angle (deg)')

    fig.suptitle('Two-Axis Monte Carlo Simulation Under Realistic Launch '
                 'Variability', fontsize=13)
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.94))
    _save(fig, 'fig10_monte_carlo.png')
    return int(np.sum(pitch_ok)), int(np.sum(yaw_ok))


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

EXPECTED_FILES = [
    'fig01_thrust_curve.png',
    'fig02_baseline_response.png',
    'fig03_open_vs_closed.png',
    'fig04_disturbance.png',
    'fig05_control_effort.png',
    'fig06_sensitivity.png',
    'fig07_control_authority.png',
    'fig08_kalman_performance.png',
    'fig09_stability_boundary.png',
    'fig10_monte_carlo.png',
]


def main():
    t_start = time.time()
    plt.style.use('default')
    os.makedirs(FIG_DIR, exist_ok=True)

    print('Generating fig01...')
    fig01_thrust_curve()
    print('Generating fig02...')
    fig02_baseline_response()
    print('Generating fig03...')
    fig03_open_vs_closed()
    print('Generating fig04...')
    dev = fig04_disturbance()
    print(f'    peak deviation after disturbance: {dev:.2f} deg')
    print('Generating fig05...')
    sat = fig05_control_effort()
    print(f'    final cumulative saturation: {sat:.2f} %')
    print('Generating fig06...')
    fig06_sensitivity()
    print('Generating fig07...')
    fig07_control_authority()
    print('Generating fig08...')
    fig08_kalman_performance()
    print('Generating fig09... (1600-point grid sweep, ~1 min)')
    fig09_stability_boundary()
    print('Generating fig10... (100 Monte Carlo trials)')
    n_pitch, n_yaw = fig10_monte_carlo()
    print(f'    recovered: pitch {n_pitch}/100, yaw {n_yaw}/100')

    missing = [f for f in EXPECTED_FILES
               if not os.path.isfile(os.path.join(FIG_DIR, f))]
    elapsed = time.time() - t_start
    if missing:
        print(f'ERROR: missing figures: {missing}')
        raise SystemExit(1)
    print(f'All 10 figures written to {FIG_DIR} in {elapsed:.1f} s.')


if __name__ == '__main__':
    main()
