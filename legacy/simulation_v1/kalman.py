"""Two-state discrete Kalman filter for attitude estimation.

State vector: [angle_deg, gyro_bias_dps].

IMPORTANT: this class does NOT generate sensor noise internally. Noisy
sensor readings are produced once in simulation.py and passed in. Never
re-draw noise inside this class; doing so was the original Kalman figure
bug (the figure showed noise the filter never saw).
"""

import numpy as np

import config


class KalmanFilter:

    def __init__(self):
        self.state = np.array([0.0, 0.0])   # [angle_deg, gyro_bias_dps]
        self.P = np.eye(2)
        # RNG reserved for callers that inject noise during simulation.
        # The filter itself never draws from it (see module docstring).
        self.rng = np.random.default_rng()

    def update(self, gyro_rate_dps, accel_angle_deg, dt):
        """One predict + update cycle. Returns (estimated_angle_deg, estimated_rate_dps)."""
        F = np.array([[1.0, -dt],
                      [0.0, 1.0]])
        B = np.array([dt, 0.0])
        H = np.array([[1.0, 0.0]])
        Q = np.diag([config.Q_ANGLE, config.Q_RATE])
        R = config.R_MEASURE

        # Predict
        state_pred = F @ self.state + B * gyro_rate_dps
        P_pred = F @ self.P @ F.T + Q

        # Update
        y = accel_angle_deg - (H @ state_pred)[0]
        S = (H @ P_pred @ H.T)[0, 0] + R
        K = (P_pred @ H.T).ravel() / S
        self.state = state_pred + K * y
        self.P = (np.eye(2) - np.outer(K, H.ravel())) @ P_pred

        estimated_angle_deg = self.state[0]
        estimated_rate_dps = gyro_rate_dps - self.state[1]
        return estimated_angle_deg, estimated_rate_dps

    def reset(self, angle=0.0):
        self.state = np.array([angle, 0.0])
        self.P = np.eye(2)
