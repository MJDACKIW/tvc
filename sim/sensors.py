"""MPU-6050-like sensor noise model for one axis. See SPEC.md Section 6, paper Section 6.7.

Ported from paper/tvc_paper_figures.py: gyro white noise plus post-burnout gyro drift, and
accelerometer white noise already expressed in angle-equivalent degrees (not g-units). This
is a flat noise model (constant std regardless of thrust level); SPEC.md's "vibration term
proportional to thrust" is a refinement the ported model does not have either, so it is not
implemented here. All noise is drawn once per physics step and returned to the caller, which
must log it rather than redraw it downstream (this was a real bug in an earlier version of
the codebase per simulation/kalman.py's docstring).
"""


class SensorModel:
    def __init__(self, gyro_noise_std_dps, accel_noise_std_deg, gyro_drift_rate_dps_per_s,
                 rng, gyro_bias_dps=0.0):
        self.gyro_noise_std_dps = gyro_noise_std_dps
        self.accel_noise_std_deg = accel_noise_std_deg
        self.gyro_drift_rate_dps_per_s = gyro_drift_rate_dps_per_s
        self.gyro_bias_dps = gyro_bias_dps
        self.rng = rng
        self.drift_dps = 0.0

    def sample(self, true_rate_deg_s, true_angle_deg, dt, post_burnout):
        """Returns (gyro_reading_dps, accel_reading_deg) for one physics step."""
        if post_burnout:
            self.drift_dps += self.gyro_drift_rate_dps_per_s * dt
        gyro_reading = (true_rate_deg_s + self.rng.normal(0.0, self.gyro_noise_std_dps)
                         + self.gyro_bias_dps + self.drift_dps)
        accel_reading = true_angle_deg + self.rng.normal(0.0, self.accel_noise_std_deg)
        return gyro_reading, accel_reading
