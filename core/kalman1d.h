#pragma once

// Scalar Kalman filter, one instance per axis. Paper Section 3.2, eq6-eq11, H = 1.
// State (x_hat, P) is owned by the caller (see controller.h AxisState) and passed by
// reference so this stays a pure function of its arguments, with no hidden state here.
namespace tvc {

// Predict step: x_hat_(k|k-1) = x_hat_(k-1) + gyro_deg_s * dt, P_(k|k-1) = P_(k-1) + q.
void kalman1d_predict(float& x_hat, float& P, float gyro_deg_s, float dt, float q);

// Update step. If gate_ok is false the accelerometer is not trusted this tick: x_hat/P
// pass through unchanged and the returned gain is 0, matching AxisOut.K / accel_used.
float kalman1d_update(float& x_hat, float& P, float z_k, float r, bool gate_ok);

}  // namespace tvc
