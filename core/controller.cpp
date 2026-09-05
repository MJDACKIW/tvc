#include "controller.h"

#include "kalman1d.h"
#include "pid.h"
#include "rate_limiter.h"

namespace tvc {

AxisOut controller_step(AxisState& state, float gyro_deg_s, float accel_tilt_deg,
                         bool accel_gate_ok, const ControlParams& params) {
    kalman1d_predict(state.x_hat, state.P, gyro_deg_s, params.dt, params.q);
    float K = kalman1d_update(state.x_hat, state.P, accel_tilt_deg, params.r, accel_gate_ok);

    PidResult pid = pid_step(state.integral, state.e_prev, state.saturated, state.x_hat,
                              params.dt, params.kp, params.ki, params.kd,
                              params.integral_clamp, params.max_deflection);

    float delta = rate_limiter_step(state.delta, pid.u_cmd, params.dt, params.slew_deg_per_s);

    return AxisOut{state.x_hat, pid.u_raw, pid.u_cmd, delta, K, accel_gate_ok};
}

}  // namespace tvc
