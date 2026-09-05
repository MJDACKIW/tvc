#include "controller.h"

#include "kalman2d.h"
#include "pid.h"
#include "rate_limiter.h"

namespace tvc {

AxisOut controller_step(AxisState& state, float gyro_deg_s, float accel_tilt_deg,
                         bool accel_gate_ok, const ControlParams& params) {
    Kalman2DState kf{state.x_hat, state.bias_hat, state.p00, state.p01, state.p10, state.p11};
    Kalman2DResult kr = kalman2d_update(kf, gyro_deg_s, accel_tilt_deg, params.dt,
                                         params.q_angle, params.q_rate, params.r,
                                         accel_gate_ok);
    state.x_hat = kf.angle;
    state.bias_hat = kf.bias;
    state.p00 = kf.p00;
    state.p01 = kf.p01;
    state.p10 = kf.p10;
    state.p11 = kf.p11;

    // Derivative from the filter's own bias-corrected rate estimate, not a finite
    // difference of the angle estimate. See kalman2d.h and pid.h.
    float d_error = -kr.kal_rate;

    PidResult pid = pid_step(state.integral, state.saturated, state.x_hat, d_error,
                              params.dt, params.kp, params.ki, params.kd,
                              params.integral_clamp, params.max_deflection);

    float delta = rate_limiter_step(state.delta, pid.u_cmd, params.dt, params.slew_deg_per_s);

    return AxisOut{state.x_hat, pid.u_raw, pid.u_cmd, delta, kr.k0, accel_gate_ok};
}

}  // namespace tvc
