#include "servo.h"

#include <cmath>

namespace tvc {

float servo_step(float& delta, float u_cmd, float dt, float tau_s, float slew_deg_per_s) {
    float alpha = 1.0f - std::exp(-dt / tau_s);
    float desired_step = alpha * (u_cmd - delta);

    float max_step = slew_deg_per_s * dt;
    float step = desired_step;
    if (step > max_step) {
        step = max_step;
    } else if (step < -max_step) {
        step = -max_step;
    }

    delta = delta + step;
    return delta;
}

}  // namespace tvc
