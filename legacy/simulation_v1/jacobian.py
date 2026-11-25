"""Linearization of the closed-loop rotational dynamics about equilibrium.

Equilibrium: theta = 0, theta_dot = 0, gimbal = 0. With PD control
delta = -KP*theta - KD*theta_dot substituted into the Newton-Euler
rotational equation, the 2x2 closed-loop system matrix for state
[theta, theta_dot] is:

    A = [[0, 1], [a21, a22]]
    a21 = -(F_T * MOMENT_ARM * KP) / MOI
    a22 = -(F_T * MOMENT_ARM * KD + aero_damp_effective) / MOI

The deg/rad conversions on the gimbal command and on the angular
acceleration cancel, so the matrix is the same in degree units.
"""

import math

import numpy as np

import config

# Mean thrust over the burn: the nominal operating point
MEAN_THRUST = 11.12  # N


def compute_jacobian(thrust=None):
    """Closed-loop Jacobian and eigenvalues at the given thrust (N).

    thrust=None uses the mean burn thrust (11.12 N).
    Returns dict with the A matrix, eigenvalues, slow/fast poles,
    stability flag, and slow time constant.
    """
    if thrust is None:
        thrust = MEAN_THRUST

    # Aerodynamic damping coefficient at the operating point
    v_approx = math.sqrt(max(2.0 * thrust / config.ROCKET_MASS, 0.0))
    q_dyn = 0.5 * config.AIR_DENSITY * v_approx ** 2
    A_cs = math.pi * config.ROCKET_RADIUS ** 2
    aero_damp_effective = config.AERO_DAMP_COEFF * q_dyn * A_cs * config.CP_OFFSET

    a12 = 1.0
    a21 = -(thrust * config.MOMENT_ARM * config.KP) / config.MOI
    a22 = -(thrust * config.MOMENT_ARM * config.KD + aero_damp_effective) / config.MOI

    A = np.array([[0.0, a12],
                  [a21, a22]])
    eigenvalues = np.linalg.eigvals(A)

    # Slow pole: smallest |real part| (attitude response).
    # Fast pole: largest |real part| (actuator-dominated response).
    ordered = sorted(eigenvalues, key=lambda z: abs(z.real))
    lambda1, lambda2 = ordered[0], ordered[-1]

    stable = bool(np.all(eigenvalues.real < 0))
    time_constant_slow = (abs(1.0 / lambda1.real)
                          if lambda1.real != 0 else float('inf'))

    return {
        'A': A,
        'eigenvalues': eigenvalues,
        'lambda1': lambda1,
        'lambda2': lambda2,
        'stable': stable,
        'time_constant_slow': time_constant_slow,
    }


if __name__ == '__main__':
    result = compute_jacobian()
    print('A =\n', result['A'])
    print('eigenvalues =', result['eigenvalues'])
    print('lambda1 (slow) = %s, lambda2 (fast) = %s'
          % (result['lambda1'], result['lambda2']))
    print('stable =', result['stable'],
          '| slow time constant = %.4f s' % result['time_constant_slow'])
