"""PID attitude controller with anti-windup, servo rate limit, and hard stop.

One instance per axis (pitch, yaw). Gains default to config values; the
project flies pure PD (KI = 0) but the integrator path is kept for
completeness and is anti-windup clamped.
"""

import config


def _clip(x, lo, hi):
    return max(lo, min(hi, x))


class PIDController:

    def __init__(self, kp=config.KP, ki=config.KI, kd=config.KD,
                 gimbal_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.gimbal_limit = config.GIMBAL_LIMIT if gimbal_limit is None else gimbal_limit
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_output = 0.0

    def update(self, error_deg, dt):
        """Compute gimbal command (deg) for this control cycle."""
        p = self.kp * error_deg
        self.integral += self.ki * error_deg * dt
        # Anti-windup clamp
        self.integral = _clip(self.integral, -self.gimbal_limit, self.gimbal_limit)
        d = self.kd * (error_deg - self.prev_error) / dt
        raw = p + self.integral + d

        # Servo rate limiting
        max_delta = config.SERVO_RATE_LIM * dt
        output = _clip(raw, self.prev_output - max_delta, self.prev_output + max_delta)

        # Hard stop
        output = _clip(output, -self.gimbal_limit, self.gimbal_limit)

        self.prev_error = error_deg
        self.prev_output = output
        return output

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_output = 0.0
