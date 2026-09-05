#pragma once

// extern "C" wrapper so Python (sim/) can call controller_step from a shared library via
// ctypes. Arguments are plain floats/ints, not the AxisState/ControlParams structs, so there
// is no C++ struct layout/padding for ctypes to get wrong. See SPEC.md Section 3.4.
extern "C" {

void tvc_controller_step(
    // AxisState, in/out.
    float* x_hat, float* P, float* integral, float* e_prev, float* delta, int* saturated,
    // Inputs.
    float gyro_deg_s, float accel_tilt_deg, int accel_gate_ok,
    // ControlParams.
    float dt, float kp, float ki, float kd, float integral_clamp, float max_deflection,
    float q, float r, float slew_deg_per_s,
    // AxisOut.
    float* out_x_hat, float* out_u_raw, float* out_u_cmd, float* out_delta, float* out_K,
    int* out_accel_used);

}  // extern "C"
