"""MPU-6050-like sensor noise model for one axis. See SPEC.md Section 6, paper Section 6.7.

Ported from paper/tvc_paper_figures.py: gyro white noise plus post-burnout gyro drift, and
accelerometer white noise already expressed in angle-equivalent degrees (not g-units). This
is a flat noise model (constant std regardless of thrust level); SPEC.md's "vibration term
proportional to thrust" is a refinement the ported model does not have either, so it is not
implemented here.

Noise is drawn as two full-length vectors at construction (all gyro samples, then all accel
samples), matching paper/tvc_paper_figures.py's own draw order exactly, rather than one
rng.normal() call per step interleaved gyro/accel/gyro/accel/... An earlier version of this
file drew noise that way; it is statistically equivalent but consumes a numpy Generator's
stream in a different pattern, so the same seed produced a different noise realization than
the paper script and silently broke run_sim.py's --legacy-physics parity check (SPEC.md
Section 6.1), which depends on identical noise for a same-seed comparison to mean anything.
"""


class SensorModel:
    def __init__(self, gyro_noise_std_dps, accel_noise_std_deg, gyro_drift_rate_dps_per_s,
                 rng, n_steps, gyro_bias_dps=0.0):
        self.gyro_drift_rate_dps_per_s = gyro_drift_rate_dps_per_s
        self.gyro_bias_dps = gyro_bias_dps
        self.drift_dps = 0.0
        self._gyro_noise = rng.normal(0.0, gyro_noise_std_dps, n_steps)
        self._accel_noise = rng.normal(0.0, accel_noise_std_deg, n_steps)

    def sample(self, i, true_rate_deg_s, true_angle_deg, dt, post_burnout):
        """Returns (gyro_reading_dps, accel_reading_deg) for step index i."""
        if post_burnout:
            self.drift_dps += self.gyro_drift_rate_dps_per_s * dt
        gyro_reading = (true_rate_deg_s + self._gyro_noise[i] + self.gyro_bias_dps
                         + self.drift_dps)
        accel_reading = true_angle_deg + self._accel_noise[i]
        return gyro_reading, accel_reading
