"""Instruments for sim/serve.py, the virtual vehicle behind ground/gs.py --sim: an MPU-6050
quantized to the raw counts the firmware logs (imu.cpp: +-2000 dps -> 16.4 LSB/(deg/s), +-16 g
-> 2048 LSB/g), a load cell, and an AS5600 encoder.

These are not a second sensor model. sim/sensors.py's SensorModel is the validated one, but it
draws its noise as one fixed-length vector at construction (that is how it reproduces the paper
script's draws), which a vehicle that runs until Ctrl-C cannot use. So the noise SIGMAS here
come from the same params.yaml block SensorModel reads (sim_overrides.sensors), and the
measurement model is the same: the accelerometer is the true tilt plus white noise, whose
accel_noise_std_deg is angle-equivalent, and whether the Kalman filter may use it is decided by
the accelerometer gate on the vehicle's specific force (serve.py), not by this file. The gyro
adds the same post-burnout drift ramp.

Unlike SensorModel there is no thrust-proportional vibration term: the validated model is a
flat noise model, and the GUI sim is meant to show that model's behaviour.
"""
from __future__ import annotations

import math

import numpy as np

GYRO_LSB_PER_DPS = 16.4
ACCEL_LSB_PER_G = 2048.0
ENCODER_LSB_DEG = 360.0 / 4096.0  # AS5600 12-bit
LOADCELL_NOISE_N = 0.1
ENCODER_NOISE_DEG = 0.03


def _q16(x: float) -> int:
    return max(-32768, min(32767, int(round(x))))


class Imu:
    """Gyro and accelerometer, body frame, after raw-count quantization."""

    def __init__(self, rng: np.random.Generator, gyro_noise_std_dps: float,
                 accel_noise_std_deg: float, gyro_drift_rate_dps_per_s: float,
                 gyro_bias_dps=(0.1, -0.08, 0.05)):
        self.rng = rng
        self.gyro_sigma = gyro_noise_std_dps
        # accel_noise_std_deg is angle-equivalent; tilt = atan2(a_lat, a_axial) with
        # a_axial ~ 1 g, so a lateral noise of sin(sigma) g gives that angle noise.
        self.accel_sigma_g = math.sin(math.radians(accel_noise_std_deg))
        self.drift_rate = gyro_drift_rate_dps_per_s
        self.bias = np.array(gyro_bias_dps)

    def sample(self, omega_p_dps: float, omega_y_dps: float, theta_p_deg: float,
               theta_y_deg: float, post_burnout_s: float = 0.0):
        """Returns (gyro_dps[3], accel_g[3]). post_burnout_s: seconds since the motor burned
        out (0 before), over which the gyro drift ramps, as SensorModel's post_burnout does."""
        n = self.rng.normal
        drift = self.drift_rate * max(0.0, post_burnout_s)
        gyro = (np.array([omega_p_dps, omega_y_dps, 0.0]) + self.bias + drift
                + n(0.0, self.gyro_sigma, 3))
        th, ps = math.radians(theta_p_deg), math.radians(theta_y_deg)
        # Gravity through the attitude, the inverse of core/attitude_from_accel.h:
        # pitch = atan2(ay, az), yaw = atan2(-ax, az). In flight the true accelerometer sees
        # thrust, not gravity; the validated sim idealises that as "true angle + noise, but the
        # filter may only use it while the gate (serve.py) says it is trustworthy".
        a = np.array([-math.sin(ps), math.sin(th) * math.cos(ps), math.cos(th) * math.cos(ps)])
        a = a + n(0.0, self.accel_sigma_g, 3)
        gyro_counts = [_q16(g * GYRO_LSB_PER_DPS) for g in gyro]
        accel_counts = [_q16(x * ACCEL_LSB_PER_G) for x in a]
        return ([c / GYRO_LSB_PER_DPS for c in gyro_counts],
                [c / ACCEL_LSB_PER_G for c in accel_counts])


class LoadCell:
    def __init__(self, rng: np.random.Generator):
        self.rng = rng
        self.offset = 0.0

    def tare(self, true_thrust: float = 0.0):
        self.offset = 0.0 - true_thrust

    def read(self, thrust_n: float) -> float:
        return thrust_n + self.offset + self.rng.normal(0.0, LOADCELL_NOISE_N)


def encoder(angle_deg: float, rng: np.random.Generator) -> float:
    return round((angle_deg + rng.normal(0.0, ENCODER_NOISE_DEG)) / ENCODER_LSB_DEG) * ENCODER_LSB_DEG
