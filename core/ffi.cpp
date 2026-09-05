#include "ffi.h"

#include "controller.h"

extern "C" void tvc_controller_step(
    float* x_hat, float* bias_hat, float* p00, float* p01, float* p10, float* p11,
    float* integral, float* delta, int* saturated,
    float gyro_deg_s, float accel_tilt_deg, int accel_gate_ok,
    float dt, float kp, float ki, float kd, float integral_clamp, float max_deflection,
    float q_angle, float q_rate, float r, float slew_deg_per_s,
    float* out_x_hat, float* out_u_raw, float* out_u_cmd, float* out_delta, float* out_K,
    int* out_accel_used) {
    tvc::AxisState state;
    state.x_hat = *x_hat;
    state.bias_hat = *bias_hat;
    state.p00 = *p00;
    state.p01 = *p01;
    state.p10 = *p10;
    state.p11 = *p11;
    state.integral = *integral;
    state.delta = *delta;
    state.saturated = (*saturated != 0);

    tvc::ControlParams params{dt, kp, ki, kd, integral_clamp, max_deflection,
                               q_angle, q_rate, r, slew_deg_per_s};

    tvc::AxisOut out =
        tvc::controller_step(state, gyro_deg_s, accel_tilt_deg, accel_gate_ok != 0, params);

    *x_hat = state.x_hat;
    *bias_hat = state.bias_hat;
    *p00 = state.p00;
    *p01 = state.p01;
    *p10 = state.p10;
    *p11 = state.p11;
    *integral = state.integral;
    *delta = state.delta;
    *saturated = state.saturated ? 1 : 0;

    *out_x_hat = out.x_hat;
    *out_u_raw = out.u_raw;
    *out_u_cmd = out.u_cmd;
    *out_delta = out.delta;
    *out_K = out.K;
    *out_accel_used = out.accel_used ? 1 : 0;
}
