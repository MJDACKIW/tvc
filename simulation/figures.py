"""All IAE-codebase figures, rendered from logged simulation data.

CRITICAL: every plot uses the arrays logged by simulation.py (for example
sim_data['accel_pitch_reading']). Noise is NEVER re-drawn here; the
Kalman figure must show exactly the readings the filter consumed.
"""

import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

import config
import jacobian as jacobian_mod

DPI = 150


def _outpath(filename):
    os.makedirs(config.FIG_DIR, exist_ok=True)
    return os.path.join(config.FIG_DIR, filename)


def _shade_burn(ax):
    ax.axvspan(0.0, config.BURN_TIME, color='orange', alpha=0.12,
               label='burn window', zorder=0)


def fig_baseline(sim_data):
    """fig_iae_01: 3-panel pitch angle / rate / gimbal, burn window shaded."""
    t = sim_data['time']
    fig, axes = plt.subplots(3, 1, figsize=(9, 9), sharex=True)

    axes[0].plot(t, sim_data['true_theta'], color='tab:blue', lw=1.2,
                 label='true pitch')
    axes[0].plot(t, sim_data['kalman_theta'], color='tab:red', lw=0.8,
                 alpha=0.7, label='Kalman estimate')
    axes[0].set_ylabel('Pitch angle (deg)')
    axes[0].legend(loc='upper right', fontsize=8)

    axes[1].plot(t, sim_data['true_theta_dot'], color='tab:green', lw=1.0,
                 label='true pitch rate')
    axes[1].set_ylabel('Pitch rate (deg/s)')
    axes[1].legend(loc='upper right', fontsize=8)

    axes[2].plot(t, sim_data['gimbal_pitch_cmd'], color='tab:purple', lw=1.0,
                 label='gimbal pitch command')
    axes[2].axhline(config.GIMBAL_LIMIT, color='gray', ls=':', lw=0.8)
    axes[2].axhline(-config.GIMBAL_LIMIT, color='gray', ls=':', lw=0.8)
    axes[2].set_ylabel('Gimbal (deg)')
    axes[2].set_xlabel('Time (s)')
    axes[2].legend(loc='upper right', fontsize=8)

    for ax in axes:
        _shade_burn(ax)
        ax.grid(alpha=0.3)
    axes[2].set_xlim(0, config.PLOT_END)

    fig.suptitle('Baseline Closed-Loop Response (IAE codebase)\n'
                 'Sim IAE = %.3f deg·s' % sim_data['IAE_sim'])
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    path = _outpath('fig_iae_01_baseline.png')
    fig.savefig(path, dpi=DPI)
    plt.close(fig)
    return path


def fig_phase_iae(sim_data, hardware_iae=None):
    """fig_iae_02: phase-segmented IAE bars, grouped vs hardware if given.

    hardware_iae: optional dict with keys 'ignition', 'peak', 'tailoff'.
    """
    labels = ['Ignition\n(0-%.1fs)' % config.IAE_IGNITION_END,
              'Peak\n(%.1f-%.1fs)' % (config.IAE_IGNITION_END, config.IAE_PEAK_END),
              'Tail-off\n(%.1f-%.2fs)' % (config.IAE_PEAK_END, config.BURN_TIME)]
    sim_vals = [sim_data['iae_phase_ignition'],
                sim_data['iae_phase_peak'],
                sim_data['iae_phase_tailoff']]

    fig, ax = plt.subplots(figsize=(8, 5))
    x = np.arange(len(labels))

    if hardware_iae is not None:
        hw_vals = [hardware_iae['ignition'], hardware_iae['peak'],
                   hardware_iae['tailoff']]
        width = 0.38
        bars_sim = ax.bar(x - width / 2, sim_vals, width,
                          label='Simulation', color='tab:blue')
        bars_hw = ax.bar(x + width / 2, hw_vals, width,
                         label='Hardware', color='tab:orange')
        groups = [bars_sim, bars_hw]
    else:
        bars_sim = ax.bar(x, sim_vals, 0.55, label='Simulation',
                          color='tab:blue')
        groups = [bars_sim]

    for bars in groups:
        for bar in bars:
            ax.annotate('%.2f' % bar.get_height(),
                        (bar.get_x() + bar.get_width() / 2, bar.get_height()),
                        ha='center', va='bottom', fontsize=8)

    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_ylabel('IAE (deg·s)')
    ax.set_title('Phase-Segmented IAE: Simulation vs Hardware')
    ax.legend()
    ax.grid(axis='y', alpha=0.3)

    fig.tight_layout()
    path = _outpath('fig_iae_02_phase_iae.png')
    fig.savefig(path, dpi=DPI)
    plt.close(fig)
    return path


def fig_sensitivity_tornado(sensitivity_data):
    """fig_iae_03: horizontal tornado plot sorted by |sensitivity|."""
    items = sorted(sensitivity_data.items(), key=lambda kv: abs(kv[1]))
    names = [k for k, _ in items]
    values = [v for _, v in items]

    fig, ax = plt.subplots(figsize=(8, 5))
    colors = ['tab:red' if v >= 0 else 'tab:blue' for v in values]
    ax.barh(names, values, color=colors, height=0.6)
    ax.axvline(0.0, color='black', lw=1.0)

    # xlim sized to the longest bar so value labels do not clip
    max_val = max(abs(v) for v in values) if values else 1.0
    ax.set_xlim(-max_val * 1.2, max_val * 1.2)
    for name, value in zip(names, values):
        offset = max_val * 0.02
        ax.annotate('%+.2f' % value,
                    (value + (offset if value >= 0 else -offset), name),
                    va='center',
                    ha='left' if value >= 0 else 'right', fontsize=8)

    ax.set_xlabel('Normalized IAE sensitivity (elasticity)')
    ax.set_title('Parameter Sensitivity Tornado (IAE)')
    ax.grid(axis='x', alpha=0.3)

    fig.tight_layout()
    path = _outpath('fig_iae_03_sensitivity_tornado.png')
    fig.savefig(path, dpi=DPI)
    plt.close(fig)
    return path


def fig_eigenvalue(eigenvalues=None):
    """fig_iae_04: complex-plane eigenvalue plot, one zoomed panel per pole."""
    if eigenvalues is None:
        eigenvalues = jacobian_mod.compute_jacobian()['eigenvalues']
    eigenvalues = np.atleast_1d(np.asarray(eigenvalues, dtype=complex))
    ordered = sorted(eigenvalues, key=lambda z: abs(z.real))
    lam_slow, lam_fast = ordered[0], ordered[-1]

    fig, axes = plt.subplots(1, 2, figsize=(11, 5))
    panels = [
        (axes[0], lam_slow, 'attitude pole', 'λ₁'),
        (axes[1], lam_fast, 'fast pole', 'λ₂'),
    ]
    for ax, lam, role, symbol in panels:
        span = max(abs(lam.real) * 0.6, 1.0)
        xlo, xhi = lam.real - span, max(lam.real + span, span * 0.3)
        ylo, yhi = -span, span
        if abs(lam.imag) > span * 0.8:
            ylo, yhi = -abs(lam.imag) * 1.4, abs(lam.imag) * 1.4

        # Stability shading: left half-plane stable, right unstable
        ax.axvspan(xlo, min(0.0, xhi), color='green', alpha=0.10)
        if xhi > 0:
            ax.axvspan(0.0, xhi, color='red', alpha=0.10)
        ax.axvline(0.0, color='black', ls='--', lw=1.0)
        ax.axhline(0.0, color='gray', lw=0.5)

        for other in eigenvalues:
            ax.plot(other.real, other.imag, 'x', color='tab:blue',
                    ms=10, mew=2)
        ax.annotate('%s ≈ %.3f (%s)' % (symbol, lam.real, role),
                    (lam.real, lam.imag), textcoords='offset points',
                    xytext=(10, 12), fontsize=9)

        ax.set_xlim(xlo, xhi)
        ax.set_ylim(ylo, yhi)
        ax.set_xlabel('Re')
        ax.set_ylabel('Im')
        ax.set_title('Zoom: %s' % role)
        ax.grid(alpha=0.3)

    fig.suptitle('Closed-Loop Eigenvalue Analysis - Linearized TVC System')
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    path = _outpath('fig_iae_04_eigenvalue.png')
    fig.savefig(path, dpi=DPI)
    plt.close(fig)
    return path


def fig_kalman(sim_data):
    """fig_iae_05: Kalman performance, strictly from logged data."""
    t = sim_data['time']
    fig, axes = plt.subplots(2, 1, figsize=(9, 7), sharex=True)

    # Raw accelerometer readings exactly as the filter consumed them
    axes[0].plot(t, sim_data['accel_pitch_reading'], color='lightgray',
                 lw=0.5, label='accel reading (raw, logged)')
    axes[0].plot(t, sim_data['true_theta'], color='black', lw=1.3,
                 label='true pitch')
    axes[0].plot(t, sim_data['kalman_theta'], color='tab:red', lw=0.9,
                 label='Kalman estimate')
    axes[0].set_ylabel('Pitch angle (deg)')
    axes[0].legend(loc='upper right', fontsize=8)

    est_err = sim_data['kalman_theta'] - sim_data['true_theta']
    axes[1].plot(t, est_err, color='tab:red', lw=0.8)
    axes[1].axhline(0.0, color='gray', lw=0.5)
    axes[1].set_ylabel('Estimation error (deg)')
    axes[1].set_xlabel('Time (s)')
    rmse = float(np.sqrt(np.mean(est_err ** 2)))
    axes[1].set_title('Estimate - true (RMSE = %.3f deg)' % rmse, fontsize=9)

    for ax in axes:
        _shade_burn(ax)
        ax.grid(alpha=0.3)
    axes[1].set_xlim(0, config.PLOT_END)

    fig.suptitle('Kalman Filter Performance (logged sensor data)')
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    path = _outpath('fig_iae_05_kalman.png')
    fig.savefig(path, dpi=DPI)
    plt.close(fig)
    return path


def fig_hardware_overlay(sim_data, hardware_data):
    """fig_iae_06: sim vs hardware pitch overlay (only when hardware given).

    hardware_data: dict with 'time' and 'pitch' arrays.
    """
    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(sim_data['time'], sim_data['true_theta'], color='tab:blue',
            lw=1.2, label='Simulation')
    ax.plot(hardware_data['time'], hardware_data['pitch'], color='tab:orange',
            lw=1.2, label='Hardware (2-DOF stand)')
    _shade_burn(ax)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Pitch angle (deg)')
    ax.set_title('Simulation vs Hardware: Attitude Comparison (2-DOF Test Stand)')
    ax.legend()
    ax.grid(alpha=0.3)
    ax.set_xlim(0, config.PLOT_END)

    fig.tight_layout()
    path = _outpath('fig_iae_06_hardware_overlay.png')
    fig.savefig(path, dpi=DPI)
    plt.close(fig)
    return path


def generate_all_figures(sim_data, sensitivity_data=None, eigenvalues=None,
                         hardware_data=None):
    """Render every figure available for the data provided.

    hardware_data: optional dict with arrays 'time', 'pitch' and phase IAE
    values under 'iae_phases' ({'ignition': .., 'peak': .., 'tailoff': ..}).
    """
    paths = [fig_baseline(sim_data)]

    hw_iae = None
    if hardware_data is not None:
        hw_iae = hardware_data.get('iae_phases')
    paths.append(fig_phase_iae(sim_data, hardware_iae=hw_iae))

    if sensitivity_data is not None:
        paths.append(fig_sensitivity_tornado(sensitivity_data))

    # Eigenvalue figure is cheap; compute from the Jacobian when not supplied
    paths.append(fig_eigenvalue(eigenvalues))

    paths.append(fig_kalman(sim_data))

    if hardware_data is not None:
        paths.append(fig_hardware_overlay(sim_data, hardware_data))

    for p in paths:
        print('  wrote %s' % p)
    return paths
