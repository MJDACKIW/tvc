"""Rigid-body rotational dynamics, thrust curve, and aero disturbance for one axis.
See SPEC.md Section 6 and Section 3.5.

`Vehicle` is the physically-corrected model (second pass): it integrates axial velocity
v(t) with drag, derives dynamic pressure from v (not thrust), and includes both the
C_Nalpha-based destabilising moment a finless, negative-static-margin vehicle actually
has (paper Section 2.5/6.1) and its matching rotational damping derivative. This
replaces a first pass that ported paper/tvc_paper_figures.py's own dynamics verbatim,
which had no destabilising moment at all (only an arbitrary rate-only damping term) and
used constants that contradicted params.yaml's own non-MEASURE fields; see the
sim_overrides comment in params.yaml for specifics.

`LegacyVehicle` reproduces the original paper script's dynamics exactly, for
run_sim.py --legacy-physics: a regression mode that validates the core/-via-ctypes
controller port independently of the physics correction above, by checking it still
reproduces the paper's own baseline/disturbance numbers when run against the paper's own
(imperfect) physics.
"""
import math
from pathlib import Path

import numpy as np

G_EARTH = 9.80665  # m/s^2, standard gravity

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


class ThrustCurve:
    """NAR .eng thrust data + interpolation, shared by Vehicle and LegacyVehicle."""

    def __init__(self, motor_params):
        self.burn_time_s = motor_params.burn_time_s
        self._time_s, self._thrust_n = load_thrust_curve(
            DATA_DIR.parent.parent / motor_params.thrust_curve_file)

    def at(self, t, thrust_scale=1.0):
        if np.isscalar(t):
            if t < 0.0 or t > self.burn_time_s:
                return 0.0
            return float(np.interp(t, self._time_s, self._thrust_n)) * thrust_scale
        t = np.asarray(t, dtype=float)
        f = np.interp(t, self._time_s, self._thrust_n) * thrust_scale
        f[(t < 0.0) | (t > self.burn_time_s)] = 0.0
        return f


def _rk4_theta_omega(alpha, t, dt, theta_deg, omega_deg_s):
    """Classical RK4 for the coupled system dtheta/dt = omega, domega/dt = alpha(t,
    theta, omega). alpha may depend on both theta and omega (Vehicle does; LegacyVehicle
    only uses the omega argument), so all four stages track both explicitly.
    """
    k1_theta = omega_deg_s
    k1_omega = alpha(t, theta_deg, omega_deg_s)

    k2_theta = omega_deg_s + 0.5 * dt * k1_omega
    k2_omega = alpha(t + 0.5 * dt, theta_deg + 0.5 * dt * k1_theta,
                      omega_deg_s + 0.5 * dt * k1_omega)

    k3_theta = omega_deg_s + 0.5 * dt * k2_omega
    k3_omega = alpha(t + 0.5 * dt, theta_deg + 0.5 * dt * k2_theta,
                      omega_deg_s + 0.5 * dt * k2_omega)

    k4_theta = omega_deg_s + dt * k3_omega
    k4_omega = alpha(t + dt, theta_deg + dt * k3_theta, omega_deg_s + dt * k3_omega)

    new_theta = theta_deg + (dt / 6.0) * (k1_theta + 2 * k2_theta + 2 * k3_theta + k4_theta)
    new_omega = omega_deg_s + (dt / 6.0) * (k1_omega + 2 * k2_omega + 2 * k3_omega + k4_omega)
    return new_theta, new_omega


class Vehicle:
    """Physically-corrected model: SPEC.md Section 3.5.

    override supplies the sim_overrides.vehicle fields (mass, inertia, moment arm, air
    density, C_D, C_Nalpha -- everything still MEASURE at top level); top_level supplies
    the two fields params.yaml already has real (non-MEASURE) values for --
    l_cop_minus_com_m and diameter_m -- read directly rather than duplicated into
    sim_overrides, so there is only one place either can be edited.
    """

    def __init__(self, override, top_level, motor_params):
        self.mass_kg = override.mass_kg
        self.moi_kg_m2 = override.inertia_pitch_kg_m2
        self.moment_arm_m = override.r_gimbal_to_com_m
        self.l_cop_com_m = top_level.l_cop_minus_com_m
        self.air_density_kg_m3 = override.air_density_kg_m3
        self.drag_coefficient = override.drag_coefficient
        self.cn_alpha_per_rad = override.cn_alpha_per_rad
        self.cross_section_area_m2 = math.pi * (top_level.diameter_m / 2.0) ** 2

        self.thrust = ThrustCurve(motor_params)
        self.burn_time_s = motor_params.burn_time_s
        self.v = 0.0  # integrated axial velocity, m/s

    def thrust_at(self, t, thrust_scale=1.0):
        return self.thrust.at(t, thrust_scale)

    def _dv_dt(self, v, thrust_n):
        drag_n = (0.5 * self.air_density_kg_m3 * v * abs(v) * self.cross_section_area_m2
                  * self.drag_coefficient)
        return thrust_n / self.mass_kg - G_EARTH - drag_n / self.mass_kg

    def _integrate_velocity(self, t, dt, thrust_scale):
        """RK4 step for axial velocity; independent of theta/omega, so integrated on
        its own rather than folded into the theta/omega system."""
        def dvdt(t_eval, v_eval):
            return self._dv_dt(v_eval, self.thrust_at(t_eval, thrust_scale))

        k1 = dvdt(t, self.v)
        k2 = dvdt(t + 0.5 * dt, self.v + 0.5 * dt * k1)
        k3 = dvdt(t + 0.5 * dt, self.v + 0.5 * dt * k2)
        k4 = dvdt(t + dt, self.v + dt * k3)
        self.v += (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
        return self.v

    def sensed_accel_g(self, thrust_n, v):
        """Axial specific force in g, i.e. what a body-mounted accelerometer reads
        (thrust minus drag; gravity does not register on an accelerometer). Used by
        run_sim.py to gate the Kalman filter's accelerometer update on |a| per SPEC.md
        Section 3.1, instead of paper/tvc_paper_figures.py's t <= burn_time proxy.
        """
        drag_n = (0.5 * self.air_density_kg_m3 * v * abs(v) * self.cross_section_area_m2
                  * self.drag_coefficient)
        return (thrust_n - drag_n) / (self.mass_kg * G_EARTH)

    def angular_accel_deg_s2(self, theta_deg, omega_deg_s, thrust_n, gimbal_deg,
                              extra_torque_nm, v):
        """Angular acceleration (deg/s^2): TVC torque, the C_Nalpha destabilising
        moment, its damping derivative, and any externally injected torque."""
        tau_tvc = thrust_n * self.moment_arm_m * math.sin(math.radians(gimbal_deg))

        q_dyn = 0.5 * self.air_density_kg_m3 * v * v
        theta_rad = math.radians(theta_deg)
        omega_rad_s = math.radians(omega_deg_s)

        tau_dist = (q_dyn * self.cross_section_area_m2 * self.cn_alpha_per_rad
                    * self.l_cop_com_m * theta_rad)
        m_q = (-0.5 * self.air_density_kg_m3 * v * self.cross_section_area_m2
               * self.cn_alpha_per_rad * self.l_cop_com_m ** 2 * omega_rad_s)

        return math.degrees((tau_tvc - tau_dist + m_q + extra_torque_nm) / self.moi_kg_m2)

    def rk4_step(self, t, dt, theta_deg, omega_deg_s, gimbal_deg, extra_torque_nm,
                 thrust_scale=1.0):
        """Advance (theta, omega) by dt with classical RK4; also advances self.v by dt.
        Gimbal, extra torque, and v are held constant over the step (zero-order hold),
        matching the servo/disturbance conventions already used elsewhere in the sim.
        """
        v = self._integrate_velocity(t, dt, thrust_scale)

        def alpha(t_eval, theta_eval, omega_eval):
            f = self.thrust_at(t_eval, thrust_scale)
            return self.angular_accel_deg_s2(theta_eval, omega_eval, f, gimbal_deg,
                                              extra_torque_nm, v)

        return _rk4_theta_omega(alpha, t, dt, theta_deg, omega_deg_s)

    def crosswind_torque_nm(self, wind_mps, v):
        """Extra destabilising torque from a lateral crosswind gust, modeled as an
        angle-of-attack increment atan(wind/v) added on top of the vehicle's own tilt
        (SPEC.md Section 6 crosswind-gust disturbance). v should be the vehicle's own
        axial velocity from just before this step (self.v), matching the zero-order-hold
        treatment rk4_step already gives gimbal and extra_torque_nm.
        """
        if v <= 0.0:
            return 0.0
        q_dyn = 0.5 * self.air_density_kg_m3 * v * v
        alpha_wind_rad = math.atan2(wind_mps, v)
        return (q_dyn * self.cross_section_area_m2 * self.cn_alpha_per_rad
                * self.l_cop_com_m * alpha_wind_rad)


class LegacyVehicle:
    """Exact reproduction of paper/tvc_paper_figures.py's dynamics (rate-only damping,
    thrust-derived dynamic pressure, no destabilising moment), for run_sim.py
    --legacy-physics. See params.yaml's sim_overrides.legacy_physics comment.
    """

    def __init__(self, vehicle_params, legacy_params, motor_params):
        self.mass_kg = vehicle_params.mass_kg
        self.moi_kg_m2 = legacy_params.moi_kg_m2
        self.moment_arm_m = vehicle_params.r_gimbal_to_com_m
        self.cp_offset_m = legacy_params.cp_offset_m
        self.air_density_kg_m3 = vehicle_params.air_density_kg_m3
        self.aero_damp_coeff = legacy_params.aero_damp_coeff
        self.cross_section_area_m2 = math.pi * legacy_params.rocket_radius_m ** 2

        self.thrust = ThrustCurve(motor_params)
        self.burn_time_s = motor_params.burn_time_s

    def thrust_at(self, t, thrust_scale=1.0):
        return self.thrust.at(t, thrust_scale)

    def angular_accel_deg_s2(self, omega_deg_s, thrust_n, gimbal_deg, extra_torque_nm):
        tau_tvc = thrust_n * self.moment_arm_m * math.sin(math.radians(gimbal_deg))
        q_dyn = self.air_density_kg_m3 * thrust_n / self.mass_kg  # = 0.5*rho*(2F/m)
        tau_damp = (-self.aero_damp_coeff * math.radians(omega_deg_s) * q_dyn
                    * self.cross_section_area_m2 * self.cp_offset_m)
        return math.degrees((tau_tvc + tau_damp + extra_torque_nm) / self.moi_kg_m2)

    def rk4_step(self, t, dt, theta_deg, omega_deg_s, gimbal_deg, extra_torque_nm,
                 thrust_scale=1.0):
        def alpha(_t_eval, _theta_eval, omega_eval):
            f = self.thrust_at(_t_eval, thrust_scale)
            return self.angular_accel_deg_s2(omega_eval, f, gimbal_deg, extra_torque_nm)

        return _rk4_theta_omega(alpha, t, dt, theta_deg, omega_deg_s)
