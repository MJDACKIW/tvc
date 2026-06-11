"""Parameter sensitivity sweep: normalized IAE elasticity per parameter.

For each parameter in config.SENSITIVITY_PARAMS the parameter is swept
across its range, the simulation re-run, and the elasticity

    S = (delta_IAE / IAE_nominal) / (delta_param / param_scale)

computed at every swept point. param_scale is the nominal value, except
for zero-nominal parameters (GYRO_BIAS), where the half range width is
used so the normalization stays finite.

The reported value per parameter is the signed elasticity of largest
magnitude across the sweep (worst case). For symmetric parameters such
as GYRO_BIAS the signed mean would cancel to ~0 and hide the effect.

IMPORTANT: every sweep is evaluated against the same NOMINAL baseline
(all other parameters at nominal, gyro bias = 0). GYRO_BIAS in
particular must be perturbed around bias = 0, not around some non-zero
baseline; that was a bug in the original implementation.
"""

import numpy as np

import config
import simulation

# Maps a sweep parameter name to the run_simulation keyword that overrides it.
_PARAM_KWARGS = {
    'KP': 'kp_override',
    'KD': 'kd_override',
    'THRUST_SCALE': 'thrust_scale',
    'MOMENT_ARM': 'moment_arm_override',
    'GIMBAL_LIMIT': 'gimbal_limit_override',
    'GYRO_BIAS': 'gyro_bias',
}

_SEED = 42  # same seed everywhere so sweeps differ only by the parameter


def run_sensitivity(n_trials_per_param=10):
    """Run the full sweep. Returns dict {param_name: sensitivity_value}."""
    print('Running nominal baseline...')
    baseline = simulation.run_simulation(theta0=5.0, seed=_SEED)
    iae_nominal = baseline['IAE_sim']
    print('  IAE_nominal = %.4f deg.s' % iae_nominal)

    results = {}
    for name, spec in config.SENSITIVITY_PARAMS.items():
        kwarg = _PARAM_KWARGS[name]
        nominal = spec['nominal']
        lo, hi = spec['range']
        n = spec.get('n', n_trials_per_param)
        # Zero-nominal parameters: normalize by half the sweep range
        param_scale = nominal if nominal != 0 else 0.5 * (hi - lo)

        print('Sweeping %s over [%g, %g] (%d points)...' % (name, lo, hi, n))
        elasticities = []
        for value in np.linspace(lo, hi, n):
            delta_param = value - nominal
            if abs(delta_param) < 1e-12:
                continue  # elasticity undefined at the nominal point itself
            sim = simulation.run_simulation(theta0=5.0, seed=_SEED,
                                            **{kwarg: float(value)})
            delta_iae = sim['IAE_sim'] - iae_nominal
            S = (delta_iae / iae_nominal) / (delta_param / param_scale)
            elasticities.append(S)

        worst = max(elasticities, key=abs)
        results[name] = float(worst)
        print('  %s: S = %+.3f (worst-case normalized elasticity)' % (name, worst))

    return results


if __name__ == '__main__':
    sens = run_sensitivity()
    print()
    print('Sensitivity summary (sorted by |S|):')
    for name, value in sorted(sens.items(), key=lambda kv: -abs(kv[1])):
        print('  %-14s %+8.3f' % (name, value))
