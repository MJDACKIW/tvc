#pragma once

// Ties the Kalman filter, PID, and rate limiter together for one axis. This is the one
// function both the firmware and the simulation call (via ffi.h), per SPEC.md Section 3.4.
namespace tvc {

struct AxisState {
    float x_hat = 0.0f;
    float P = 0.0f;
    float integral = 0.0f;
    float e_prev = 0.0f;
    float delta = 0.0f;
    bool saturated = false;
};

struct AxisOut {
    float x_hat;
    float u_raw;
    float u_cmd;
    float delta;
    float K;
    bool accel_used;
};

struct ControlParams {
    float dt;      // 1 / control.rate_hz, seconds
    float kp;
    float ki;
    float kd;
    float integral_clamp;
    float max_deflection;
    float q;
    float r;
    float slew_deg_per_s;
};

AxisOut controller_step(AxisState& state, float gyro_deg_s, float accel_tilt_deg,
                         bool accel_gate_ok, const ControlParams& params);

}  // namespace tvc
