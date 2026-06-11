"""E12-4 motor thrust curve model.

Linear interpolation over the NAR certified thrust data in config.py.
"""

import numpy as np

import config


def get_thrust(t, thrust_scale=1.0):
    """Return motor thrust in N at time t (s), scaled by thrust_scale.

    Returns 0.0 outside the burn (t < 0 or t > config.BURN_TIME).
    """
    if t < 0.0 or t > config.BURN_TIME:
        return 0.0
    return float(np.interp(t, config.THRUST_TIME, config.THRUST_N)) * thrust_scale
