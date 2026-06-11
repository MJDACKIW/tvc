"""CLI entry point for the TVC IAE simulation codebase.

Usage:
    python main.py --mode simulate
    python main.py --mode sensitivity
    python main.py --mode hardware path/to/hardware_log.csv
    python main.py --mode full [path/to/hardware_log.csv]
"""

import argparse
import sys

import analysis
import figures
import jacobian
import sensitivity
import simulation
import config


def _baseline_sim():
    print('Running baseline simulation...')
    return simulation.run_simulation(theta0=5.0, seed=42)


def mode_simulate():
    sim_data = _baseline_sim()
    analysis.print_stability_summary(jacobian.compute_jacobian()['eigenvalues'])
    analysis.compare_iae(sim_data)
    figures.generate_all_figures(sim_data)
    print('Done. Figures saved to %s/' % config.FIG_DIR)
    return sim_data


def mode_sensitivity(sim_data=None):
    print('Running sensitivity sweep (this is the slow part)...')
    sens = sensitivity.run_sensitivity()
    figures.fig_sensitivity_tornado(sens)
    print('Done. Tornado figure saved to %s/' % config.FIG_DIR)
    return sens


def mode_hardware(csv_path, sim_data=None):
    if csv_path is None:
        print('error: --mode hardware requires a CSV path, e.g.\n'
              '    python main.py --mode hardware data/hardware_log.csv',
              file=sys.stderr)
        sys.exit(2)
    if sim_data is None:
        sim_data = _baseline_sim()
    comparison = analysis.compare_iae(sim_data, hardware_csv_path=csv_path)
    hardware_data = dict(comparison['hardware_arrays'])
    hardware_data['iae_phases'] = comparison['hardware']
    figures.fig_phase_iae(sim_data, hardware_iae=comparison['hardware'])
    figures.fig_hardware_overlay(sim_data, hardware_data)
    print('Done. Comparison figures saved to %s/' % config.FIG_DIR)
    return comparison


def mode_full(csv_path):
    sim_data = _baseline_sim()
    analysis.print_stability_summary(jacobian.compute_jacobian()['eigenvalues'])
    analysis.compare_iae(sim_data)

    sens = mode_sensitivity()

    hardware_data = None
    if csv_path is not None:
        comparison = mode_hardware(csv_path, sim_data=sim_data)
        hardware_data = dict(comparison['hardware_arrays'])
        hardware_data['iae_phases'] = comparison['hardware']
    else:
        print('No hardware CSV provided; skipping hardware comparison.')

    figures.generate_all_figures(sim_data, sensitivity_data=sens,
                                 hardware_data=hardware_data)
    print('Done. Figures saved to %s/' % config.FIG_DIR)


def main():
    parser = argparse.ArgumentParser(
        description='TVC IAE simulation: sim vs hardware attitude error.')
    parser.add_argument('--mode', default='simulate',
                        choices=['simulate', 'sensitivity', 'hardware', 'full'],
                        help='what to run (default: simulate)')
    parser.add_argument('csv_path', nargs='?', default=None,
                        help='hardware log CSV (hardware and full modes)')
    args = parser.parse_args()

    if args.mode == 'simulate':
        mode_simulate()
    elif args.mode == 'sensitivity':
        mode_sensitivity()
    elif args.mode == 'hardware':
        mode_hardware(args.csv_path)
    elif args.mode == 'full':
        mode_full(args.csv_path)


if __name__ == '__main__':
    main()
