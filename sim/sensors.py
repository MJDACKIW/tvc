"""MPU-6050 / load cell / AS5600 models (SPEC.md Section 6, paper Sections 6.7, 6.11): white
noise + gyro bias, accelerometer vibration proportional to thrust, quantized to the same raw
counts the firmware logs (imu.cpp: +-2000 dps -> 16.4 LSB/(deg/s), +-16 g -> 2048 LSB/g). Seeded.

Noise floors are MPU-6050 datasheet densities over the ~98 Hz DLPF (gyro 0.005 deg/s/rtHz,
accel 400 ug/rtHz). The vibration coefficients are placeholders, not measured: they only have
to make the accelerometer gate visibly reject samples at peak thrust, the paper 6.7 behaviour.
"""
from __future__ import annotations

import math

import numpy as np

GYRO_LSB_PER_DPS = 16.4
ACCEL_LSB_PER_G = 2048.0
ENCODER_LSB_DEG = 360.0 / 4096.0  # AS5600 12-bit

GYRO_NOISE_DPS = 0.005 * math.sqrt(98)
ACCEL_NOISE_G = 400e-6 * math.sqrt(98)
GYRO_VIB_DPS_PER_N = 0.03
ACCEL_VIB_AXIAL_G_PER_N = 0.012
ACCEL_VIB_LATERAL_G_PER_N = 0.002
LOADCELL_NOISE_N = 0.1
ENCODER_NOISE_DEG = 0.03


def _q16(x: float) -> int:
    return max(-32768, min(32767, int(round(x))))


class Imu:
    def __init__(self, rng: np.random.Generator, gyro_bias_dps=(0.1, -0.08, 0.05)):
        self.rng = rng
        self.bias = np.array(gyro_bias_dps)

    def sample(self, omega_p_dps: float, omega_y_dps: float, theta_p_deg: float,
               theta_y_deg: float, thrust_n: float, specific_force_g: float | None):
        """Returns (gyro_dps[3], accel_g[3]) after raw-count quantization, body frame.
        specific_force_g: None on the pad/stand (sensor sees gravity through the attitude);
        otherwise the axial specific force in flight (gravity is not sensed in free flight)."""
        n = self.rng.normal
        vib = max(0.0, thrust_n)
        g_sig = GYRO_NOISE_DPS + GYRO_VIB_DPS_PER_N * vib
        gyro = np.array([omega_p_dps, omega_y_dps, 0.0]) + self.bias + n(0.0, g_sig, 3)
        if specific_force_g is None:
            th, ps = math.radians(theta_p_deg), math.radians(theta_y_deg)
            # inverse of core/attitude_from_accel.h: pitch = atan2(ay, az), yaw = atan2(-ax, az)
            a = np.array([-math.sin(ps), math.sin(th) * math.cos(ps), math.cos(th) * math.cos(ps)])
        else:
            a = np.array([0.0, 0.0, specific_force_g])
        lat = ACCEL_NOISE_G + ACCEL_VIB_LATERAL_G_PER_N * vib
        ax_sig = ACCEL_NOISE_G + ACCEL_VIB_AXIAL_G_PER_N * vib
        a = a + np.array([n(0.0, lat), n(0.0, lat), n(0.0, ax_sig)])
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
