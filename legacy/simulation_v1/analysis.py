"""IAE comparison (simulation vs hardware) and stability reporting."""

import numpy as np

import config
from simulation import iae


_TIME_COLS = ('time', 'time_s', 't')
_PITCH_COLS = ('pitch', 'pitch_deg', 'theta', 'theta_deg', 'angle', 'angle_deg')
_YAW_COLS = ('yaw', 'yaw_deg', 'psi', 'psi_deg')


def load_hardware_csv(path):
    """Load a hardware test stand log.

    Accepts either a headered CSV (column names matched case-insensitively
    against time/pitch/yaw aliases) or a plain numeric CSV with columns
    [time, pitch, (optional) yaw].

    Returns dict: {'time': arr, 'pitch': arr, 'yaw': arr or None}.
    """
    data = np.genfromtxt(path, delimiter=',', names=True)
    names = data.dtype.names
    if names is not None and len(names) >= 2:
        lower = {n.lower(): n for n in names}

        def find(aliases):
            for a in aliases:
                if a in lower:
                    return data[lower[a]]
            return None

        time = find(_TIME_COLS)
        pitch = find(_PITCH_COLS)
        yaw = find(_YAW_COLS)
        if time is not None and pitch is not None:
            return {'time': np.asarray(time, dtype=float),
                    'pitch': np.asarray(pitch, dtype=float),
                    'yaw': None if yaw is None else np.asarray(yaw, dtype=float)}

    # Fall back to positional columns
    raw = np.loadtxt(path, delimiter=',', ndmin=2)
    return {'time': raw[:, 0],
            'pitch': raw[:, 1],
            'yaw': raw[:, 2] if raw.shape[1] > 2 else None}


def _hardware_phase_iae(hw):
    """Phase-segmented IAE for hardware data, same trapz formula as sim."""
    time = hw['time']
    angle_sets = [hw['pitch']] + ([hw['yaw']] if hw['yaw'] is not None else [])
    phases = {
        'ignition': (0.0, config.IAE_IGNITION_END),
        'peak': (config.IAE_IGNITION_END, config.IAE_PEAK_END),
        'tailoff': (config.IAE_PEAK_END, config.BURN_TIME),
        'total': (0.0, config.BURN_TIME),
    }
    out = {}
    for name, (lo, hi) in phases.items():
        out[name] = sum(iae(a, time, lo, hi) for a in angle_sets)
    return out


def compare_iae(sim_data, hardware_csv_path=None):
    """Print phase-segmented IAE; if a hardware CSV is given, compare.

    Returns a dict with all IAE values (and hardware arrays when loaded).
    """
    sim_phases = {
        'ignition': sim_data['iae_phase_ignition'],
        'peak': sim_data['iae_phase_peak'],
        'tailoff': sim_data['iae_phase_tailoff'],
        'total': sim_data['IAE_sim'],
    }

    if hardware_csv_path is None:
        print()
        print('Simulation IAE (deg.s), phase-segmented')
        print('-' * 46)
        print('%-14s %10s' % ('Phase', 'Sim IAE'))
        print('%-14s %10.3f' % ('Ignition', sim_phases['ignition']))
        print('%-14s %10.3f' % ('Peak thrust', sim_phases['peak']))
        print('%-14s %10.3f' % ('Tail-off', sim_phases['tailoff']))
        print('%-14s %10.3f' % ('Total', sim_phases['total']))
        print()
        return {'sim': sim_phases}

    hw = load_hardware_csv(hardware_csv_path)
    hw_phases = _hardware_phase_iae(hw)

    notes = {
        'ignition': 'launch detect latency',
        'peak': 'max thrust, max control authority',
        'tailoff': 'aero damping absent on stand',
        'total': '',
    }
    print()
    print('IAE comparison: simulation vs hardware (deg.s)')
    print('-' * 78)
    print('%-14s %10s %10s %10s   %s' % ('Phase', 'Sim IAE', 'HW IAE', 'dIAE', 'Notes'))
    for key, label in (('ignition', 'Ignition'), ('peak', 'Peak thrust'),
                       ('tailoff', 'Tail-off'), ('total', 'Total')):
        delta = hw_phases[key] - sim_phases[key]
        print('%-14s %10.3f %10.3f %+10.3f   %s'
              % (label, sim_phases[key], hw_phases[key], delta, notes[key]))
    print()

    return {
        'sim': sim_phases,
        'hardware': hw_phases,
        'delta': {k: hw_phases[k] - sim_phases[k] for k in sim_phases},
        'hardware_arrays': hw,
    }


def print_stability_summary(eigenvalues):
    """Formatted closed-loop eigenvalue report."""
    eigenvalues = np.atleast_1d(np.asarray(eigenvalues, dtype=complex))
    stable = bool(np.all(eigenvalues.real < 0))

    print()
    print('Closed-loop stability summary (linearized TVC system)')
    print('-' * 62)
    for idx, lam in enumerate(sorted(eigenvalues, key=lambda z: abs(z.real)), 1):
        tau = abs(1.0 / lam.real) if lam.real != 0 else float('inf')
        kind = 'slow pole' if idx == 1 else 'fast pole'
        print('  lambda_%d = %10.3f %+8.3fj   (%s, tau = %.4f s)'
              % (idx, lam.real, lam.imag, kind, tau))
    print('  System is %s: all eigenvalues in the %s half-plane.'
          % ('STABLE' if stable else 'UNSTABLE',
             'left' if stable else 'right'))
    print()
