#include "ffi.h"

#include "controller.h"

extern "C" void tvc_controller_step(
    float* x_hat, float* P, float* integral, float* e_prev, float* delta, int* saturated,
    float gyro_deg_s, float accel_tilt_deg, int accel_gate_ok,
    float dt, float kp, float ki, float kd, float integral_clamp, float max_deflection,
    float q, float r, float slew_deg_per_s,
    float* out_x_hat, float* out_u_raw, float* out_u_cmd, float* out_delta, float* out_K,
    int* out_accel_used) {
    tvc::AxisState state;
    state.x_hat = *x_hat;
    state.P = *P;
    state.integral = *integral;
    state.e_prev = *e_prev;
    state.delta = *delta;
    state.saturated = (*saturated != 0);

    tvc::ControlParams params{dt, kp, ki, kd, integral_clamp, max_deflection, q, r, slew_deg_per_s};

    tvc::AxisOut out =
        tvc::controller_step(state, gyro_deg_s, accel_tilt_deg, accel_gate_ok != 0, params);

    *x_hat = state.x_hat;
    *P = state.P;
    *integral = state.integral;
    *e_prev = state.e_prev;
    *delta = state.delta;
    *saturated = state.saturated ? 1 : 0;

    *out_x_hat = out.x_hat;
    *out_u_raw = out.u_raw;
    *out_u_cmd = out.u_cmd;
    *out_delta = out.delta;
    *out_K = out.K;
    *out_accel_used = out.accel_used ? 1 : 0;
}
