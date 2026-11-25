#include "estimator.h"

#include "ffi.h"
#include "params.h"

namespace tvc::flight::estimator {
namespace {

// Mirrors core/controller.h's AxisState field for field, but as the plain floats/ints
// core/ffi.h's extern "C" API takes, so firmware and the Python sim (sim/tvc_core.py) hold
// the same state and call the same entry points.
struct AxisState {
    float x_hat = 0.0f;
    float bias_hat = 0.0f;
    float p00 = 0.0f;
    float p01 = 0.0f;
    float p10 = 0.0f;
    float p11 = 0.0f;
    float integral = 0.0f;
    float delta = 0.0f;
    int saturated = 0;
};

AxisState g_pitch, g_yaw;

void resetAxis(AxisState& s) {
    // The one initialiser the sim uses too (core/controller.h controller_init), so the
    // Teensy starts the filter from exactly the same state: x_hat = bias_hat = 0 and an
    // asymmetric P0 = diag(p0_angle, p0_bias). p0_angle/p0_bias are fixed numbers in
    // params.yaml (not MEASURE), so this is real even before the stand tuning pass.
    ::tvc_controller_init(&s.x_hat, &s.bias_hat, &s.p00, &s.p01, &s.p10, &s.p11, &s.integral,
                          &s.delta, &s.saturated, params::kalman::p0_angle,
                          params::kalman::p0_bias);
}

ShadowOut step(AxisState& s, float gyro_deg_s, float accel_tilt_deg, bool accel_gate_ok) {
    ShadowOut out{};
    int out_accel_used = 0;
    ::tvc_controller_step(
        &s.x_hat, &s.bias_hat, &s.p00, &s.p01, &s.p10, &s.p11, &s.integral, &s.delta,
        &s.saturated, gyro_deg_s, accel_tilt_deg, accel_gate_ok ? 1 : 0,
        1.0f / params::control::rate_hz, params::control::kp, params::control::ki,
        params::control::kd, params::control::integral_clamp_deg_s,
        params::control::max_deflection_deg, params::kalman::q_angle, params::kalman::q_rate,
        params::kalman::r, params::servo::slew_deg_per_s, params::servo::tau_s, &out.x_hat,
        &out.u_raw, &out.u_cmd, &out.delta, &out.K, &out_accel_used);
    out.accel_used = out_accel_used != 0;
    return out;
}

}  // namespace

void init() {
    resetAxis(g_pitch);
    resetAxis(g_yaw);
}

ShadowOut stepPitch(float gyro_deg_s, float accel_tilt_deg, bool accel_gate_ok) {
    return step(g_pitch, gyro_deg_s, accel_tilt_deg, accel_gate_ok);
}

ShadowOut stepYaw(float gyro_deg_s, float accel_tilt_deg, bool accel_gate_ok) {
    return step(g_yaw, gyro_deg_s, accel_tilt_deg, accel_gate_ok);
}

}  // namespace tvc::flight::estimator
