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

`StandVehicle` models the 2-DOF static test stand (SPEC.md Section 3.5's --stand note,
Section 7's iae_compare.py companion study): v = 0 always, so there is no aero at all
(tau_aero = M_q = 0), replaced by mechanical Coulomb + viscous friction at the pivot.
`ThrustLog`, `load_stand_log`, and the ignition/theta0 helpers below support run_sim.py's
--thrust-from-log and --theta0-from-log, which drive a stand run from a decoded log
instead of the NAR thrust curve and the usual theta0=5 deg default; see their docstrings
for the log .npz schema (decode_log.py does not exist yet, so this is the schema it will
need to produce, not one it already does).
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
        moment, its damping derivative, and any externally injected torque.

        tau_dist is ADDED, not subtracted: params.yaml's l_cop_minus_com_m is documented
        as "positive = destabilising" (CoP ahead of CoM, this vehicle's actual finless
        configuration), so a positive tau_dist must have the SAME sign as theta -- a
        positive-feedback term that grows the tilt, not a spring-like restoring one. An
        earlier version subtracted it, which for a positive l_cop_minus_com_m produces
        I*theta'' -k*theta (k>0): a stable damped oscillator, the opposite of what a
        negative-static-margin body does. Caught by an open-loop sanity check (theta0=5,
        omega0=0, no control): the buggy sign gave bounded decaying oscillation; a
        genuine positive-feedback instability should diverge monotonically instead, which
        it does after this fix. See SPEC.md Section 3.5 for the paper's own equivalent
        sign ambiguity between its eq14 and eq18.
        """
        tau_tvc = thrust_n * self.moment_arm_m * math.sin(math.radians(gimbal_deg))

        q_dyn = 0.5 * self.air_density_kg_m3 * v * v
        theta_rad = math.radians(theta_deg)
        omega_rad_s = math.radians(omega_deg_s)

        tau_dist = (q_dyn * self.cross_section_area_m2 * self.cn_alpha_per_rad
                    * self.l_cop_com_m * theta_rad)
        m_q = (-0.5 * self.air_density_kg_m3 * v * self.cross_section_area_m2
               * self.cn_alpha_per_rad * self.l_cop_com_m ** 2 * omega_rad_s)

        return math.degrees((tau_tvc + tau_dist + m_q + extra_torque_nm) / self.moi_kg_m2)

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
    thrust-derived dynamic pressure q = rho*F/m, no destabilising moment), for
    run_sim.py --legacy-physics. Reads only from params.yaml's sim_overrides.legacy_physics,
    never from sim_overrides.vehicle: nothing is shared with the corrected Vehicle model,
    even where a value happens to be numerically identical, so an edit to the corrected
    model's block can never silently perturb this reproduction.
    """

    def __init__(self, legacy_params, motor_params):
        self.mass_kg = legacy_params.mass_kg
        self.moi_kg_m2 = legacy_params.moi_kg_m2
        self.moment_arm_m = legacy_params.r_gimbal_to_com_m
        self.cp_offset_m = legacy_params.cp_offset_m
        self.air_density_kg_m3 = legacy_params.air_density_kg_m3
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


class StandVehicle:
    """2-DOF static test-stand model, for run_sim.py --stand. v = 0 always: no aero at
    all (tau_aero = M_q = 0 per SPEC.md Section 3.5), replaced by mechanical Coulomb +
    viscous friction at the pivot. thrust_log, if given (--thrust-from-log), overrides
    the NAR thrust curve with measured stand-fire data; otherwise this uses the same
    ThrustCurve as free flight, since the stand fires the same motor.
    """

    _FRICTION_SIGN_EPS_RAD_S = 0.01  # smooths sign(omega) near zero so RK4 doesn't
                                      # evaluate a genuine discontinuity across substeps

    def __init__(self, stand_params, motor_params, thrust_log=None):
        self.moi_kg_m2 = stand_params.inertia_kg_m2
        self.friction_coulomb_Nm = stand_params.friction_coulomb_Nm
        self.friction_viscous_Nm_s = stand_params.friction_viscous_Nm_s
        self.moment_arm_m = stand_params.arm_pivot_to_gimbal_m
        self.v = 0.0  # no airflow on the stand; kept so run_simulation's shared call
                      # sites (crosswind_torque_nm's v argument, logging) still work

        self._thrust_log = thrust_log
        if thrust_log is not None:
            self.burn_time_s = thrust_log.burn_time_s
        else:
            self.thrust = ThrustCurve(motor_params)
            self.burn_time_s = motor_params.burn_time_s

    def thrust_at(self, t, thrust_scale=1.0):
        if self._thrust_log is not None:
            return self._thrust_log.at(t) * thrust_scale
        return self.thrust.at(t, thrust_scale)

    def sensed_accel_g(self, thrust_n, v):
        """Not a meaningful quantity on the stand (fixed to the pivot, no translational
        acceleration to speak of): run_sim.py's --stand path always passes gate_ok=True
        instead of gating on this. Present only so StandVehicle matches
        Vehicle/LegacyVehicle's interface.
        """
        return 1.0

    def angular_accel_deg_s2(self, omega_deg_s, thrust_n, gimbal_deg, extra_torque_nm):
        tau_ctrl = thrust_n * self.moment_arm_m * math.sin(math.radians(gimbal_deg))
        omega_rad_s = math.radians(omega_deg_s)
        tau_coulomb = (-self.friction_coulomb_Nm
                       * math.tanh(omega_rad_s / self._FRICTION_SIGN_EPS_RAD_S))
        tau_viscous = -self.friction_viscous_Nm_s * omega_rad_s
        return math.degrees((tau_ctrl + tau_coulomb + tau_viscous + extra_torque_nm)
                             / self.moi_kg_m2)

    def rk4_step(self, t, dt, theta_deg, omega_deg_s, gimbal_deg, extra_torque_nm,
                 thrust_scale=1.0):
        def alpha(t_eval, _theta_eval, omega_eval):
            f = self.thrust_at(t_eval, thrust_scale)
            return self.angular_accel_deg_s2(omega_eval, f, gimbal_deg, extra_torque_nm)

        return _rk4_theta_omega(alpha, t, dt, theta_deg, omega_deg_s)


# ---------------------------------------------------------------------------
# Decoded stand-fire log ingestion, for run_sim.py --thrust-from-log/--theta0-from-log
# and tools/iae_compare.py. tools/decode_log.py does not exist yet (Phase 7); this
# defines the .npz schema it will need to produce, not one it already does. Fields, one
# array per key except params_hash, all sharing the same time base except params_hash:
#   time_s:            log timestamps, s (t=0 at some fixed reference, not ignition)
#   thrust_n:           measured/estimated thrust, N (0 before ignition)
#   encoder_angle_deg:  stand encoder tilt angle, deg
#   gyro_dps:           gyro rate reading, deg/s
#   gimbal_actual_deg:  measured/commanded gimbal deflection, deg (iae_compare.py's
#                       delta(t) comparison; not used by run_sim.py's --thrust-from-log
#                       or --theta0-from-log)
#   params_hash:        scalar string, the params.yaml SHA-256 the firmware logged this
#                       run against (SPEC.md Section 2); iae_compare.py refuses to run
#                       if this doesn't match the sim's current tvc_params.PARAMS_HASH
# Ignition is the first time_s where thrust_n crosses IGNITION_THRUST_N; everything here
# re-zeroes to that instant, matching how ThrustCurve/*.thrust_at already treat t=0.
# ---------------------------------------------------------------------------

IGNITION_THRUST_N = 1.0

_STAND_LOG_FIELDS = ("time_s", "thrust_n", "encoder_angle_deg", "gyro_dps",
                     "gimbal_actual_deg", "params_hash")


def load_stand_log(npz_path):
    data = np.load(npz_path)
    log = {k: data[k] for k in _STAND_LOG_FIELDS}
    log["params_hash"] = str(log["params_hash"])
    return log


def find_ignition_index(time_s, thrust_n, threshold_n=IGNITION_THRUST_N):
    idx = np.where(thrust_n >= threshold_n)[0]
    if len(idx) == 0:
        raise ValueError(f"no sample with thrust_n >= {threshold_n} N; can't find ignition")
    return int(idx[0])


class ThrustLog:
    """Measured thrust vs. time from a decoded stand-fire log (--thrust-from-log),
    re-zeroed to ignition. Same .at(t) interface as ThrustCurve, minus thrust_scale
    (measured thrust is not scaled; thrust_scale still applies on top, in StandVehicle).
    """

    def __init__(self, log):
        ignition_idx = find_ignition_index(log["time_s"], log["thrust_n"])
        t_ignition = log["time_s"][ignition_idx]
        self._time_s = log["time_s"] - t_ignition
        self._thrust_n = log["thrust_n"]
        self.burn_time_s = float(self._time_s[-1])

    def at(self, t):
        if np.isscalar(t):
            if t < 0.0 or t > self.burn_time_s:
                return 0.0
            return float(np.interp(t, self._time_s, self._thrust_n))
        t = np.asarray(t, dtype=float)
        f = np.interp(t, self._time_s, self._thrust_n)
        f[(t < 0.0) | (t > self.burn_time_s)] = 0.0
        return f


def theta0_and_gyro_bias_from_log(log):
    """--theta0-from-log: encoder angle at ignition (theta0_deg) and the mean gyro
    reading over the pre-ignition window (gyro_bias_dps), from the same decoded log
    --thrust-from-log already loaded.
    """
    ignition_idx = find_ignition_index(log["time_s"], log["thrust_n"])
    if ignition_idx == 0:
        raise ValueError("no pre-ignition samples in log; can't estimate gyro bias")
    theta0_deg = float(log["encoder_angle_deg"][ignition_idx])
    gyro_bias_dps = float(np.mean(log["gyro_dps"][:ignition_idx]))
    return theta0_deg, gyro_bias_dps
