"""Rigid-body attitude dynamics and thrust curve (SPEC.md Section 3.5 / 6, paper Section 6.1).

Minimal first cut, written for sim/run_sim.py --serve: per-axis RK4 at 1 ms on
I * theta_ddot = F_T(t) * r * sin(delta) + tau_ext(t). The aero term tau_dist(theta) of SPEC 3.5
is NOT here yet: it needs vehicle.cn_alpha_per_rad, which is still MEASURE, and a translational
velocity model; both are Phase 2 (SPEC 9). With it absent this is exactly SPEC's --stand case
(v = 0 -> tau_dist = 0), which is also what every STAND_* mode is.
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent


def load_eng(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """RASP .eng -> (t, F) arrays with the implicit (0, 0) start point prepended."""
    ts, fs = [0.0], [0.0]
    header_seen = False
    for line in Path(path).read_text().splitlines():
        line = line.strip()
        if not line or line.startswith(";"):
            continue
        if not header_seen:
            header_seen = True
            continue
        t, f = line.split()[:2]
        ts.append(float(t))
        fs.append(float(f))
    return np.array(ts), np.array(fs)


class ThrustCurve:
    def __init__(self, path: Path | None = None, scale: float = 1.0):
        import tvc_params
        self.t, self.f = load_eng(path or REPO_ROOT / tvc_params.motor.thrust_curve_file)
        self.scale = scale
        self.burn_time = float(self.t[-1])

    def __call__(self, t_motor: float) -> float:
        """Thrust [N] at t_motor seconds after the motor lit (paper 6.2: numpy.interp)."""
        if t_motor <= 0.0 or t_motor >= self.burn_time:
            return 0.0
        return float(np.interp(t_motor, self.t, self.f)) * self.scale


class AxisDynamics:
    """theta [deg], omega [deg/s] about one body axis."""

    def __init__(self, inertia_kg_m2: float, r_gimbal_to_com_m: float, theta0_deg: float = 0.0,
                 coulomb_friction_nm: float = 0.0):
        self.I = inertia_kg_m2
        self.r = r_gimbal_to_com_m
        self.friction = coulomb_friction_nm  # stand pivot (SPEC 6 --stand); 0 in free flight
        self.theta = theta0_deg
        self.omega = 0.0

    def _alpha(self, thrust_n: float, delta_deg: float, tau_ext: float) -> float:
        tau = thrust_n * self.r * math.sin(math.radians(delta_deg)) + tau_ext
        return math.degrees(tau / self.I)

    def step(self, dt: float, thrust_n: float, delta_deg: float, tau_ext: float = 0.0):
        """Classical RK4; thrust and delta are held constant across the step (zero-order hold,
        as the actuator and the 150 Hz controller present them)."""
        a = self._alpha(thrust_n, delta_deg, tau_ext)  # no state dependence without aero
        k1x, k1v = self.omega, a
        k2x, k2v = self.omega + 0.5 * dt * k1v, a
        k3x, k3v = self.omega + 0.5 * dt * k2v, a
        k4x, k4v = self.omega + dt * k3v, a
        self.theta += dt / 6.0 * (k1x + 2 * k2x + 2 * k3x + k4x)
        self.omega += dt / 6.0 * (k1v + 2 * k2v + 2 * k3v + k4v)
        if self.friction:
            # Coulomb friction applied after the step and never allowed to reverse the motion
            # (a sign(omega) term inside RK4 chatters about omega = 0).
            dw = math.degrees(self.friction / self.I) * dt
            self.omega = 0.0 if abs(self.omega) <= dw else self.omega - math.copysign(dw, self.omega)
