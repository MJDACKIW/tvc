#include "pid.h"

namespace tvc {

PidResult pid_step(float& integral, float& e_prev, bool& saturated, float x_hat, float dt,
                    float kp, float ki, float kd, float integral_clamp, float max_deflection) {
    float e = 0.0f - x_hat;

    if (!saturated) {
        integral += e * dt;
    }
    if (integral > integral_clamp) {
        integral = integral_clamp;
    } else if (integral < -integral_clamp) {
        integral = -integral_clamp;
    }

    float deriv = (e - e_prev) / dt;
    float u = kp * e + ki * integral + kd * deriv;

    float u_cmd = u;
    if (u_cmd > max_deflection) {
        u_cmd = max_deflection;
    } else if (u_cmd < -max_deflection) {
        u_cmd = -max_deflection;
    }

    saturated = (u != u_cmd);
    e_prev = e;

    return PidResult{u, u_cmd};
}

}  // namespace tvc
