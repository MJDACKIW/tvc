"""Rigid-body rotational dynamics of the TVC rocket, RK4 integrated.

Two independent axes: pitch (theta) and yaw (psi). Each axis sees three
torques: TVC gimbal torque, aerodynamic damping, and wind acting at the
center of pressure.

Critical physics fixes encoded here:
- Aerodynamic damping is always included (it was missing originally).
- Wind input acts through the CP_OFFSET moment arm (the moment arm was
  wrong originally).
"""

import math

import config
import motor


class RocketDynamics:
    """State: theta (deg), theta_dot (deg/s), psi (deg), psi_dot (deg/s)."""

    def __init__(self, theta0=0.0, rate0=0.0, psi0=0.0, psi_rate0=0.0,
                 moment_arm=None, moi=None):
        self.theta = theta0
        self.theta_dot = rate0
        self.psi = psi0
        self.psi_dot = psi_rate0
        # Overridable for sensitivity sweeps; default to config values.
        self.moment_arm = config.MOMENT_ARM if moment_arm is None else moment_arm
        self.moi = config.MOI if moi is None else moi

    def _alpha(self, t, angular_rate_dps, gimbal_deg, wind_input, thrust_scale):
        """Angular acceleration (deg/s^2) for one axis at time t."""
        F_T = motor.get_thrust(t, thrust_scale)
        tau_tvc = F_T * self.moment_arm * math.sin(math.radians(gimbal_deg))

        # Aerodynamic damping (critical physics fix: must be included)
        v_approx = math.sqrt(max(2.0 * F_T / config.ROCKET_MASS, 0.0))
        q_dyn = 0.5 * config.AIR_DENSITY * v_approx ** 2
        A_cs = math.pi * config.ROCKET_RADIUS ** 2
        tau_aero = (-config.AERO_DAMP_COEFF * math.radians(angular_rate_dps)
                    * q_dyn * A_cs * config.CP_OFFSET)

        # Wind acting at the CP (critical physics fix: CP_OFFSET moment arm)
        tau_wind = wind_input * config.CP_OFFSET

        return math.degrees((tau_tvc + tau_aero + tau_wind) / self.moi)

    def _rk4_axis(self, t, angle, rate, gimbal_deg, wind_input, thrust_scale):
        """Advance one axis by DT_SIM with classical RK4 on (angle, rate)."""
        dt = config.DT_SIM

        k1_ang = rate
        k1_rate = self._alpha(t, rate, gimbal_deg, wind_input, thrust_scale)

        k2_ang = rate + 0.5 * dt * k1_rate
        k2_rate = self._alpha(t + 0.5 * dt, k2_ang, gimbal_deg, wind_input, thrust_scale)

        k3_ang = rate + 0.5 * dt * k2_rate
        k3_rate = self._alpha(t + 0.5 * dt, k3_ang, gimbal_deg, wind_input, thrust_scale)

        k4_ang = rate + dt * k3_rate
        k4_rate = self._alpha(t + dt, k4_ang, gimbal_deg, wind_input, thrust_scale)

        new_angle = angle + (dt / 6.0) * (k1_ang + 2.0 * k2_ang + 2.0 * k3_ang + k4_ang)
        new_rate = rate + (dt / 6.0) * (k1_rate + 2.0 * k2_rate + 2.0 * k3_rate + k4_rate)
        return new_angle, new_rate

    def step(self, t, gimbal_pitch_deg, gimbal_yaw_deg,
             wind_torque_pitch=0.0, wind_torque_yaw=0.0, thrust_scale=1.0):
        """Advance both axes by one DT_SIM step starting at time t.

        wind_torque_pitch / wind_torque_yaw are wind force inputs (N) applied
        at the CP; they are multiplied by CP_OFFSET inside the torque model.
        Returns (theta, theta_dot, psi, psi_dot) after the step.
        """
        self.theta, self.theta_dot = self._rk4_axis(
            t, self.theta, self.theta_dot, gimbal_pitch_deg,
            wind_torque_pitch, thrust_scale)
        self.psi, self.psi_dot = self._rk4_axis(
            t, self.psi, self.psi_dot, gimbal_yaw_deg,
            wind_torque_yaw, thrust_scale)
        return self.theta, self.theta_dot, self.psi, self.psi_dot
