#include "estimator.h"

#include "ffi.h"
#include "params.h"

namespace tvc::flight::estimator {
namespace {

struct AxisState {
    float x_hat = 0.0f;
    float P = 0.0f;
    float integral = 0.0f;
    float e_prev = 0.0f;
    float delta = 0.0f;
    int saturated = 0;
};

AxisState g_pitch, g_yaw;

ShadowOut step(AxisState& s, float gyro_deg_s, float accel_tilt_deg, bool accel_gate_ok) {
    ShadowOut out{};
    int out_accel_used = 0;
    ::tvc_controller_step(
        &s.x_hat, &s.P, &s.integral, &s.e_prev, &s.delta, &s.saturated, gyro_deg_s,
        accel_tilt_deg, accel_gate_ok ? 1 : 0, 1.0f / params::control::rate_hz,
        params::control::kp, params::control::ki, params::control::kd,
        params::control::integral_clamp_deg_s, params::control::max_deflection_deg,
        params::kalman::q, params::kalman::r, params::servo::slew_deg_per_s, &out.x_hat,
        &out.u_raw, &out.u_cmd, &out.delta, &out.K, &out_accel_used);
    out.accel_used = out_accel_used != 0;
    return out;
}

}  // namespace

void init() {
    g_pitch = AxisState{};
    g_pitch.P = params::kalman::p0;
    g_yaw = AxisState{};
    g_yaw.P = params::kalman::p0;
}

ShadowOut stepPitch(float gyro_deg_s, float accel_tilt_deg, bool accel_gate_ok) {
    return step(g_pitch, gyro_deg_s, accel_tilt_deg, accel_gate_ok);
}

ShadowOut stepYaw(float gyro_deg_s, float accel_tilt_deg, bool accel_gate_ok) {
    return step(g_yaw, gyro_deg_s, accel_tilt_deg, accel_gate_ok);
}

}  // namespace tvc::flight::estimator
