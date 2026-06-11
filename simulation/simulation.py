"""Main simulation loop: dynamics + sensors + Kalman + flight computer + PID.

Physics advances at DT_SIM (1 kHz); the controller runs every
physics_steps_per_control steps (150 Hz nominal, 7 ms actual).

All sensor noise is drawn HERE, once per physics step, and the noisy
readings are both logged and fed to the Kalman filters. Downstream
consumers (figures, analysis) must use the logged readings and never
re-draw noise.
"""

import numpy as np

import config
import motor
from dynamics import RocketDynamics
from kalman import KalmanFilter
from control import PIDController
from flight_computer import FlightComputer


def _trapz(y, x):
    """np.trapz with the numpy 2.x rename handled."""
    if hasattr(np, 'trapezoid'):
        return float(np.trapezoid(y, x))
    return float(np.trapz(y, x))


def iae(angle_deg, time_s, t_lo, t_hi):
    """Integral of Absolute Error of angle_deg over time window [t_lo, t_hi]."""
    mask = (time_s >= t_lo) & (time_s <= t_hi)
    if mask.sum() < 2:
        return 0.0
    return _trapz(np.abs(angle_deg[mask]), time_s[mask])


def run_simulation(theta0=5.0, rate0=0.0, psi0=0.0, psi_rate0=0.0,
                   open_loop=False, disturbance=None, wind_mag=0.0,
                   thrust_scale=1.0, kp_override=None, kd_override=None,
                   gimbal_limit_override=None, gyro_bias=0.0, seed=None,
                   moment_arm_override=None):
    """Run one full simulation from t=0 to PLOT_END.

    disturbance: optional dict {'t_start': s, 'magnitude': N*m, 'duration': s}.
        The magnitude is a torque; it is converted to the equivalent wind
        force at the CP (magnitude / CP_OFFSET) because dynamics applies
        wind inputs through the CP_OFFSET moment arm.
    wind_mag: steady wind force (N) at the CP, pitch axis.
    open_loop: gimbal held at 0 always; flight computer still runs.

    Returns a dict of logged numpy arrays plus IAE metrics and metadata.
    """
    rng = np.random.default_rng(seed)

    kp = config.KP if kp_override is None else kp_override
    kd = config.KD if kd_override is None else kd_override
    gimbal_limit = (config.GIMBAL_LIMIT if gimbal_limit_override is None
                    else gimbal_limit_override)

    dyn = RocketDynamics(theta0=theta0, rate0=rate0,
                         psi0=psi0, psi_rate0=psi_rate0,
                         moment_arm=moment_arm_override)
    kalman_pitch = KalmanFilter()
    kalman_yaw = KalmanFilter()
    pid_pitch = PIDController(kp=kp, ki=config.KI, kd=kd, gimbal_limit=gimbal_limit)
    pid_yaw = PIDController(kp=kp, ki=config.KI, kd=kd, gimbal_limit=gimbal_limit)
    fc = FlightComputer()

    # Disturbance window (torque in N*m converted to CP force input)
    dist_t0, dist_t1, dist_force = None, None, 0.0
    if disturbance is not None:
        dist_t0 = disturbance.get('t_start', disturbance.get('time', 0.6))
        dist_t1 = dist_t0 + disturbance.get('duration', 0.05)
        dist_force = (disturbance.get('magnitude', disturbance.get('torque', 0.12))
                      / config.CP_OFFSET)

    n_steps = int(round(config.PLOT_END / config.DT_SIM))
    physics_steps_per_control = round(1.0 / (config.CONTROL_HZ * config.DT_SIM))  # = 7
    dt_control = config.DT_SIM * physics_steps_per_control

    gimbal_pitch = 0.0
    gimbal_yaw = 0.0
    control_step_counter = 0
    gyro_drift = 0.0  # accumulated post-burnout drift, deg/s

    log = {key: np.zeros(n_steps) for key in (
        'time', 'true_theta', 'true_theta_dot', 'true_psi', 'true_psi_dot',
        'kalman_theta', 'kalman_psi', 'gimbal_pitch_cmd', 'gimbal_yaw_cmd',
        'accel_pitch_reading', 'gyro_pitch_reading',
        'accel_psi_reading', 'gyro_psi_reading',
        'thrust_at_t', 'flight_state_int')}

    for i in range(n_steps):
        t = i * config.DT_SIM
        t_next = t + config.DT_SIM
        control_step_counter += 1

        # Wind / disturbance input for this step (force at the CP, pitch axis)
        wind_input_pitch = wind_mag
        if dist_t0 is not None and dist_t0 <= t < dist_t1:
            wind_input_pitch += dist_force

        # 1. Advance dynamics
        theta, theta_dot, psi, psi_dot = dyn.step(
            t, gimbal_pitch, gimbal_yaw,
            wind_torque_pitch=wind_input_pitch,
            thrust_scale=thrust_scale)

        # 2. Generate sensor readings (noise added to true state, drawn once here)
        gyro_pitch_reading = (theta_dot + rng.normal(0.0, config.GYRO_NOISE_STD)
                              + gyro_bias + gyro_drift)
        accel_pitch_reading = theta + rng.normal(0.0, config.ACCEL_NOISE_STD)
        gyro_psi_reading = (psi_dot + rng.normal(0.0, config.GYRO_NOISE_STD)
                            + gyro_bias + gyro_drift)
        accel_psi_reading = psi + rng.normal(0.0, config.ACCEL_NOISE_STD)

        # 3. Kalman update (noisy readings passed in, never drawn inside)
        k_pitch_angle, k_pitch_rate = kalman_pitch.update(
            gyro_pitch_reading, accel_pitch_reading, config.DT_SIM)
        k_psi_angle, k_psi_rate = kalman_yaw.update(
            gyro_psi_reading, accel_psi_reading, config.DT_SIM)

        # 4. FlightComputer update
        thrust_now = motor.get_thrust(t_next, thrust_scale)
        accel_g = thrust_now / (config.ROCKET_MASS * 9.81)
        fc.update(t_next, accel_g)

        # Post-burnout: gyro drift accumulates
        if fc.state == 'BURNOUT':
            gyro_drift += config.GYRO_DRIFT_RATE * config.DT_SIM

        # 5. Control update (every physics_steps_per_control steps)
        if control_step_counter >= physics_steps_per_control:
            control_step_counter = 0
            if fc.control_active and not open_loop:
                gimbal_pitch = pid_pitch.update(-k_pitch_angle, dt_control)
                gimbal_yaw = pid_yaw.update(-k_psi_angle, dt_control)
            else:
                gimbal_pitch = 0.0
                gimbal_yaw = 0.0

        # 6. Log everything
        log['time'][i] = t_next
        log['true_theta'][i] = theta
        log['true_theta_dot'][i] = theta_dot
        log['true_psi'][i] = psi
        log['true_psi_dot'][i] = psi_dot
        log['kalman_theta'][i] = k_pitch_angle
        log['kalman_psi'][i] = k_psi_angle
        log['gimbal_pitch_cmd'][i] = gimbal_pitch
        log['gimbal_yaw_cmd'][i] = gimbal_yaw
        log['accel_pitch_reading'][i] = accel_pitch_reading
        log['gyro_pitch_reading'][i] = gyro_pitch_reading
        log['accel_psi_reading'][i] = accel_psi_reading
        log['gyro_psi_reading'][i] = gyro_psi_reading
        log['thrust_at_t'][i] = thrust_now
        log['flight_state_int'][i] = fc.state_int

    time = log['time']

    # Burn window indices
    burn_mask = time <= config.BURN_TIME
    burn_start_idx = 0
    burn_end_idx = int(np.max(np.nonzero(burn_mask)))

    # IAE over the burn window, per axis and summed
    iae_pitch = iae(log['true_theta'], time, 0.0, config.BURN_TIME)
    iae_yaw = iae(log['true_psi'], time, 0.0, config.BURN_TIME)

    phases = [
        ('ignition', 0.0, config.IAE_IGNITION_END),
        ('peak', config.IAE_IGNITION_END, config.IAE_PEAK_END),
        ('tailoff', config.IAE_PEAK_END, config.BURN_TIME),
    ]
    phase_iae = {}
    for name, lo, hi in phases:
        p = iae(log['true_theta'], time, lo, hi)
        y = iae(log['true_psi'], time, lo, hi)
        phase_iae['iae_phase_%s_pitch' % name] = p
        phase_iae['iae_phase_%s_yaw' % name] = y
        phase_iae['iae_phase_%s' % name] = p + y

    result = dict(log)
    result.update(phase_iae)
    result.update({
        'burn_start_idx': burn_start_idx,
        'burn_end_idx': burn_end_idx,
        'iae_pitch': iae_pitch,
        'iae_yaw': iae_yaw,
        'IAE_sim': iae_pitch + iae_yaw,
        'launch_time': fc.launch_time,
        'burnout_time': fc.burnout_time,
    })
    return result


if __name__ == '__main__':
    data = run_simulation(theta0=5.0, seed=42)
    print('IAE_sim   = %.4f deg.s (pitch %.4f + yaw %.4f)'
          % (data['IAE_sim'], data['iae_pitch'], data['iae_yaw']))
    print('Phases    : ignition %.4f | peak %.4f | tail-off %.4f'
          % (data['iae_phase_ignition'], data['iae_phase_peak'],
             data['iae_phase_tailoff']))
    print('Launch    : t=%.3f s, burnout: t=%.3f s'
          % (data['launch_time'], data['burnout_time']))
