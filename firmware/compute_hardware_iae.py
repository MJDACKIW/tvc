#!/usr/bin/env python3
"""
compute_hardware_iae.py

Ingests the SD card CSV log from tvc_2dof.ino and computes hardware IAE
for comparison against simulation IAE from analysis.py.

Expected CSV header (written by tvc_2dof.ino, fixed order):
    timestamp_ms,accel_pitch_deg,accel_yaw_deg,gyro_pitch_dps,gyro_yaw_dps,
    accel_mag_g,kalman_pitch_deg,kalman_yaw_deg,gimbal_pitch_cmd_deg,
    gimbal_yaw_cmd_deg,flight_state,iae_pitch,iae_yaw

flight_state values: 0 = PAD_IDLE, 1 = POWERED_FLIGHT, 2 = BURNOUT.

Usage:
    python compute_hardware_iae.py <path_to_csv>
    python compute_hardware_iae.py flight01.csv
    python compute_hardware_iae.py flight01.csv --sim-iae 0.342
    python compute_hardware_iae.py flight01.csv --sim-csv sim_run.csv
"""

import argparse
import sys

import numpy as np

try:
    import pandas as pd
    HAS_PANDAS = True
except ImportError:
    HAS_PANDAS = False

# Phase boundaries, must match simulation/config.py
# (IAE_IGNITION_END, IAE_PEAK_END)
IAE_IGNITION_END = 0.3   # s
IAE_PEAK_END = 0.6       # s

PAD_IDLE = 0
POWERED_FLIGHT = 1
BURNOUT = 2

# Column aliases for simulation CSV overlays (same idea as analysis.py)
_SIM_TIME_COLS = ('time', 'time_s', 't')
_SIM_PITCH_COLS = ('pitch', 'pitch_deg', 'theta', 'theta_deg',
                   'angle', 'angle_deg', 'kalman_pitch_deg')
_SIM_YAW_COLS = ('yaw', 'yaw_deg', 'psi', 'psi_deg', 'kalman_yaw_deg')


def _trapz(y, x):
    """np.trapz with the numpy 2.x rename handled (same as simulation.py)."""
    if hasattr(np, 'trapezoid'):
        return float(np.trapezoid(y, x))
    return float(np.trapz(y, x))


def _window_iae(angle_deg, time_s, t_lo, t_hi):
    """IAE of angle_deg over [t_lo, t_hi], same formula as simulation.iae()."""
    mask = (time_s >= t_lo) & (time_s <= t_hi)
    if mask.sum() < 2:
        return 0.0
    return _trapz(np.abs(angle_deg[mask]), time_s[mask])


def load_log(csv_path):
    """Load the flight log CSV into a dict of column name -> numpy array."""
    if HAS_PANDAS:
        frame = pd.read_csv(csv_path)
        df = {c: frame[c].to_numpy() for c in frame.columns}
    else:
        data = np.genfromtxt(csv_path, delimiter=',', names=True)
        if data.dtype.names is None:
            raise ValueError(f'{csv_path}: could not parse CSV header')
        df = {name: np.atleast_1d(data[name]) for name in data.dtype.names}

    required = ('timestamp_ms', 'kalman_pitch_deg', 'kalman_yaw_deg',
                'flight_state')
    missing = [c for c in required if c not in df]
    if missing:
        raise ValueError(f'{csv_path}: missing columns {missing} '
                         '(is this a tvc_2dof.ino log?)')

    t_ms = df['timestamp_ms']
    state = df['flight_state'].astype(int)
    n = len(t_ms)
    duration_s = (t_ms[-1] - t_ms[0]) / 1000.0

    launch_rows = np.flatnonzero(state == POWERED_FLIGHT)
    burnout_rows = np.flatnonzero(state == BURNOUT)

    print(f'Loaded {csv_path}: {n} rows, {duration_s:.2f} s of data')
    if len(launch_rows):
        print(f'  Launch detected at  t = {t_ms[launch_rows[0]] / 1000.0:.3f} s'
              ' (log time)')
    else:
        print('  WARNING: no POWERED_FLIGHT rows found')
    if len(burnout_rows):
        print(f'  Burnout detected at t = {t_ms[burnout_rows[0]] / 1000.0:.3f} s'
              ' (log time)')
    else:
        print('  WARNING: no BURNOUT rows found, using end of log')
    return df


def detect_burn_window(df):
    """Return (launch_idx, burnout_idx, burn_duration_s) from flight_state."""
    state = df['flight_state'].astype(int)
    t_ms = df['timestamp_ms']

    launch_rows = np.flatnonzero(state == POWERED_FLIGHT)
    if len(launch_rows) == 0:
        raise ValueError('No POWERED_FLIGHT (state 1) rows in log; '
                         'cannot locate burn window')
    launch_idx = int(launch_rows[0])

    burnout_rows = np.flatnonzero(state == BURNOUT)
    burnout_idx = int(burnout_rows[0]) if len(burnout_rows) else len(state) - 1

    burn_duration_s = (t_ms[burnout_idx] - t_ms[launch_idx]) / 1000.0
    return launch_idx, burnout_idx, burn_duration_s


def compute_iae(df, launch_idx, burnout_idx):
    """Trapezoidal IAE over the burn window, total and phase-segmented.

    Phase IAE sums pitch + yaw (same convention as analysis.py).
    """
    # Include the burnout row so the integral covers through burn end
    sl = slice(launch_idx, burnout_idx + 1)
    t_ms = df['timestamp_ms'][sl]
    time_s = t_ms / 1000.0 - t_ms[0] / 1000.0
    pitch = df['kalman_pitch_deg'][sl]
    yaw = df['kalman_yaw_deg'][sl]
    burn_end = float(time_s[-1])

    iae_pitch = _trapz(np.abs(pitch), time_s)
    iae_yaw = _trapz(np.abs(yaw), time_s)

    phases = {
        'ignition': (0.0, IAE_IGNITION_END),
        'peak': (IAE_IGNITION_END, IAE_PEAK_END),
        'tailoff': (IAE_PEAK_END, burn_end),
    }
    out = {
        'pitch': iae_pitch,
        'yaw': iae_yaw,
        'total': iae_pitch + iae_yaw,
        'burn_end_s': burn_end,
    }
    for name, (lo, hi) in phases.items():
        out[name] = (_window_iae(pitch, time_s, lo, hi)
                     + _window_iae(yaw, time_s, lo, hi))
    return out


def load_sim_csv(sim_csv_path):
    """Load a simulation CSV, matching column names case-insensitively.

    Returns dict {'time': arr, 'pitch': arr, 'yaw': arr or None} or None.
    """
    data = np.genfromtxt(sim_csv_path, delimiter=',', names=True)
    names = data.dtype.names
    if names is None:
        print(f'  WARNING: {sim_csv_path} has no header, skipping overlay')
        return None
    lower = {n.lower(): n for n in names}

    def find(aliases):
        for a in aliases:
            if a in lower:
                return np.atleast_1d(data[lower[a]]).astype(float)
        return None

    time = find(_SIM_TIME_COLS)
    pitch = find(_SIM_PITCH_COLS)
    yaw = find(_SIM_YAW_COLS)
    if time is None or pitch is None:
        print(f'  WARNING: {sim_csv_path} lacks time/pitch columns, '
              'skipping overlay')
        return None
    return {'time': time, 'pitch': pitch, 'yaw': yaw}


def compute_sim_phase_iae(sim):
    """Phase-segmented IAE for a simulation trace, same formula as hardware."""
    time_s = sim['time']
    angle_sets = [sim['pitch']] + ([sim['yaw']] if sim['yaw'] is not None else [])
    burn_end = float(time_s[-1])
    out = {'burn_end_s': burn_end}
    for name, (lo, hi) in (('ignition', (0.0, IAE_IGNITION_END)),
                           ('peak', (IAE_IGNITION_END, IAE_PEAK_END)),
                           ('tailoff', (IAE_PEAK_END, burn_end)),
                           ('total', (0.0, burn_end))):
        out[name] = sum(_window_iae(a, time_s, lo, hi) for a in angle_sets)
    return out


def plot_hardware_vs_sim(df, launch_idx, burnout_idx, sim_csv_path=None,
                         out_path='hardware_vs_sim.png'):
    """Plot Kalman pitch/yaw over the burn window, optional sim overlay."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    sl = slice(launch_idx, burnout_idx + 1)
    t_ms = df['timestamp_ms'][sl]
    time_s = t_ms / 1000.0 - t_ms[0] / 1000.0
    burn_end = float(time_s[-1])

    sim = load_sim_csv(sim_csv_path) if sim_csv_path else None

    fig, (ax_p, ax_y) = plt.subplots(2, 1, figsize=(9, 6), sharex=True)

    ax_p.plot(time_s, df['kalman_pitch_deg'][sl], color='C0',
              label='Hardware (Kalman)')
    ax_y.plot(time_s, df['kalman_yaw_deg'][sl], color='C0',
              label='Hardware (Kalman)')
    if sim is not None:
        ax_p.plot(sim['time'], sim['pitch'], color='C1', linestyle=':',
                  label='Simulation')
        if sim['yaw'] is not None:
            ax_y.plot(sim['time'], sim['yaw'], color='C1', linestyle=':',
                      label='Simulation')

    for ax, name in ((ax_p, 'Pitch'), (ax_y, 'Yaw')):
        ax.axvline(burn_end, color='k', linestyle='--', linewidth=1,
                   label='Burnout')
        ax.axhline(0, color='gray', linewidth=0.5)
        ax.set_ylabel(f'{name} (deg)')
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(alpha=0.3)
    ax_y.set_xlabel('Time since launch (s)')
    ax_p.set_title('Hardware vs simulation: estimated attitude during burn')

    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f'Saved plot: {out_path}')


def print_iae_table(hw_iae, sim_iae=None):
    """Formatted hardware-vs-simulation IAE comparison table.

    sim_iae may be None (hardware only), a float (total only, from
    --sim-iae), or a dict with phase keys (from --sim-csv).
    """
    if isinstance(sim_iae, (int, float)):
        sim_iae = {'total': float(sim_iae)}
    sim_iae = sim_iae or {}

    rows = (('Ignition', 'ignition', ''),
            ('Peak thrust', 'peak', ''),
            ('Tail-off', 'tailoff', '  <- aero damping absent on stand'),
            ('Total', 'total', ''))

    bar = '=' * 63
    print()
    print(bar)
    print('IAE COMPARISON: Hardware vs Simulation')
    print(bar)
    print(f'{"Phase":<14} {"HW IAE":>8} {"Sim IAE":>9} {"dIAE":>9} {"% diff":>8}')
    print('-' * 63)
    for label, key, note in rows:
        hw = hw_iae[key]
        sim = sim_iae.get(key)
        if sim is None:
            print(f'{label:<14} {hw:>8.3f} {"--":>9} {"--":>9} {"--":>8}{note}')
        else:
            delta = hw - sim
            pct = f'{delta / sim * 100.0:+.0f}%' if sim != 0 else 'n/a'
            print(f'{label:<14} {hw:>8.3f} {sim:>9.3f} {delta:>+9.3f} '
                  f'{pct:>8}{note}')
    print(bar)
    print('Note: positive dIAE expected in tail-off '
          '(no aero damping on static stand)')
    print()


def main():
    parser = argparse.ArgumentParser(
        description='Compute hardware IAE from a tvc_2dof.ino SD log and '
                    'compare against simulation.')
    parser.add_argument('csv_path', help='path to flightNN.csv from the SD card')
    parser.add_argument('--sim-iae', type=float, default=None,
                        help='total simulation IAE (deg*s) from analysis.py')
    parser.add_argument('--sim-csv', default=None,
                        help='simulation trace CSV to overlay and compare '
                             '(time/pitch/yaw columns)')
    args = parser.parse_args()

    df = load_log(args.csv_path)
    launch_idx, burnout_idx, burn_duration_s = detect_burn_window(df)
    print(f'  Burn window: rows {launch_idx} to {burnout_idx} '
          f'({burn_duration_s:.3f} s)')

    hw_iae = compute_iae(df, launch_idx, burnout_idx)
    print(f'  Hardware IAE: pitch = {hw_iae["pitch"]:.4f}, '
          f'yaw = {hw_iae["yaw"]:.4f}, total = {hw_iae["total"]:.4f} deg*s')

    sim_iae = None
    if args.sim_csv:
        sim = load_sim_csv(args.sim_csv)
        if sim is not None:
            sim_iae = compute_sim_phase_iae(sim)
    if sim_iae is None and args.sim_iae is not None:
        sim_iae = args.sim_iae

    print_iae_table(hw_iae, sim_iae)
    plot_hardware_vs_sim(df, launch_idx, burnout_idx, sim_csv_path=args.sim_csv)
    return 0


if __name__ == '__main__':
    sys.exit(main())
