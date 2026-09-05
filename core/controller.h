#pragma once

// Ties the Kalman filter, PID, and rate limiter together for one axis. This is the one
// function both the firmware and the simulation call (via ffi.h), per SPEC.md Section 3.4.
namespace tvc {

struct AxisState {
    float x_hat = 0.0f;     // angle estimate, deg
    float bias_hat = 0.0f;  // gyro bias estimate, deg/s (kalman2d's second state)
    float p00 = 0.0f;
    float p01 = 0.0f;
    float p10 = 0.0f;
    float p11 = 0.0f;
    float integral = 0.0f;
    float delta = 0.0f;
    bool saturated = false;
};

struct AxisOut {
    float x_hat;
    float u_raw;
    float u_cmd;
    float delta;
    float K;  // kalman2d's angle-measurement gain (k0) used this step
    bool accel_used;
};

struct ControlParams {
    float dt;      // 1 / control.rate_hz, seconds
    float kp;
    float ki;
    float kd;
    float integral_clamp;
    float max_deflection;
    float q_angle;
    float q_rate;
    float r;
    float slew_deg_per_s;
};

AxisOut controller_step(AxisState& state, float gyro_deg_s, float accel_tilt_deg,
                         bool accel_gate_ok, const ControlParams& params);

}  // namespace tvc
