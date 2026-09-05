#include "pid.h"

namespace tvc {

PidResult pid_step(float& integral, bool& saturated, float x_hat, float d_error, float dt,
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

    float u = kp * e + ki * integral + kd * d_error;

    float u_cmd = u;
    if (u_cmd > max_deflection) {
        u_cmd = max_deflection;
    } else if (u_cmd < -max_deflection) {
        u_cmd = -max_deflection;
    }

    saturated = (u != u_cmd);

    return PidResult{u, u_cmd};
}

}  // namespace tvc
