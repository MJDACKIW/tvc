"""Rigid-body rotational dynamics, thrust curve, and aero disturbance for one axis.
See SPEC.md Section 6 and Section 3.5.

Ported from paper/tvc_paper_figures.py, which is the script that actually generated the
paper's Figures 2-11 and Table 1 (the simulation/ folder is a separate, narrower
hardware-vs-sim IAE codebase and was not the source of those figures or that table).

The aerodynamic term here is a rate-proportional damping torque, matching what the paper
script implements. This is a different model from SPEC.md Section 3.5's angle-proportional,
C_Nalpha-based destabilising moment; see the "sim_overrides" comment block in params.yaml
for why: reproducing the published Table 1 numbers requires the model that actually
produced them, and vehicle.cn_alpha_per_rad has no measured value to build the other
model from anyway.
"""
import math
from pathlib import Path

import numpy as np

DATA_DIR = Path(__file__).resolve().parent / "data"


def load_thrust_curve(eng_path):
    """Parse a RASP .eng file's (time_s, thrust_N) data rows.

    Only the two-numeric-column data lines are used; the header line (name, diameter,
    length, delay codes, propellant type, masses, manufacturer) and ';' comments are
    skipped. Returns (time_s, thrust_n) as float arrays.
    """
    times, thrusts = [], []
    for line in Path(eng_path).read_text().splitlines():
        line = line.strip()
        if not line or line.startswith(";"):
            continue
        parts = line.split()
        if len(parts) == 2:
            try:
                t, f = float(parts[0]), float(parts[1])
            except ValueError:
                continue
            times.append(t)
            thrusts.append(f)
    return np.array(times), np.array(thrusts)


class Vehicle:
    """One rigid body, one rotational axis (pitch or yaw treated independently and
    identically, per paper/tvc_paper_figures.py)."""

    def __init__(self, vehicle_params, motor_params):
        self.mass_kg = vehicle_params.mass_kg
        self.moi_kg_m2 = vehicle_params.inertia_pitch_kg_m2
        self.moment_arm_m = vehicle_params.r_gimbal_to_com_m
        self.cp_offset_m = vehicle_params.cp_offset_m
        self.air_density_kg_m3 = vehicle_params.air_density_kg_m3
        self.rocket_radius_m = vehicle_params.rocket_radius_m
        self.aero_damp_coeff = vehicle_params.aero_damp_coeff
        self.cross_section_area_m2 = math.pi * self.rocket_radius_m ** 2

        self.burn_time_s = motor_params.burn_time_s
        self._thrust_time_s, self._thrust_n = load_thrust_curve(
            DATA_DIR.parent.parent / motor_params.thrust_curve_file
        )

    def thrust_at(self, t, thrust_scale=1.0):
        """Motor thrust in N at time t (scalar or array). Zero outside the burn."""
        if np.isscalar(t):
            if t < 0.0 or t > self.burn_time_s:
                return 0.0
            return float(np.interp(t, self._thrust_time_s, self._thrust_n)) * thrust_scale
        t = np.asarray(t, dtype=float)
        f = np.interp(t, self._thrust_time_s, self._thrust_n) * thrust_scale
        f[(t < 0.0) | (t > self.burn_time_s)] = 0.0
        return f

    def angular_accel_deg_s2(self, omega_deg_s, thrust_n, gimbal_deg, extra_torque_nm):
        """Angular acceleration for one axis (deg/s^2): TVC torque, aero damping, and
        any externally injected torque (wind / disturbance impulse)."""
        tau_tvc = thrust_n * self.moment_arm_m * math.sin(math.radians(gimbal_deg))
        q_dyn = self.air_density_kg_m3 * thrust_n / self.mass_kg  # = 0.5*rho*(2F/m)
        tau_damp = (-self.aero_damp_coeff * math.radians(omega_deg_s) * q_dyn
                    * self.cross_section_area_m2 * self.cp_offset_m)
        return math.degrees((tau_tvc + tau_damp + extra_torque_nm) / self.moi_kg_m2)

    def rk4_step(self, t, dt, theta_deg, omega_deg_s, gimbal_deg, extra_torque_nm,
                 thrust_scale=1.0):
        """Advance (theta, omega) by dt with classical RK4. Gimbal and extra torque are
        held constant over the step (zero-order hold), matching the paper script."""
        def alpha(t_eval, omega_eval):
            f = self.thrust_at(t_eval, thrust_scale)
            return self.angular_accel_deg_s2(omega_eval, f, gimbal_deg, extra_torque_nm)

        k1_t, k1_w = omega_deg_s, alpha(t, omega_deg_s)
        k2_t = omega_deg_s + 0.5 * dt * k1_w
        k2_w = alpha(t + 0.5 * dt, k2_t)
        k3_t = omega_deg_s + 0.5 * dt * k2_w
        k3_w = alpha(t + 0.5 * dt, k3_t)
        k4_t = omega_deg_s + dt * k3_w
        k4_w = alpha(t + dt, k4_t)

        theta_deg += (dt / 6.0) * (k1_t + 2.0 * k2_t + 2.0 * k3_t + k4_t)
        omega_deg_s += (dt / 6.0) * (k1_w + 2.0 * k2_w + 2.0 * k3_w + k4_w)
        return theta_deg, omega_deg_s
