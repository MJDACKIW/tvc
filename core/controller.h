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
    float tau_s;  // servo first-order lag time constant, seconds (servo.h)
};

AxisOut controller_step(AxisState& state, float gyro_deg_s, float accel_tilt_deg,
                         bool accel_gate_ok, const ControlParams& params);

// Resets state to a well-defined initial condition: x_hat = bias_hat = 0, P0 =
// diag(p0_angle, p0_bias) (off-diagonal 0), integral = delta = 0, saturated = false.
// The ONE place this convention is defined, so firmware and the sim can't drift apart
// on it the way sim/tvc_core.py's ControllerAxis used to set these fields directly
// itself. p0_angle and p0_bias are deliberately asymmetric in flight use (see
// params.yaml's kalman.p0_angle/p0_bias comments): the angle state starts genuinely
// uncertain (an unknown tip-off angle), the bias state starts already well-characterized
// (a calibrated gyro), and a shared, equal P0 for both let the filter's very first
// accelerometer correction misattribute part of a large initial-angle error to bias_hat,
// which was never revisited once the accelerometer gate closed (see
// run_sim.py's diagnose_estimator).
void controller_init(AxisState& state, float p0_angle, float p0_bias);

}  // namespace tvc
