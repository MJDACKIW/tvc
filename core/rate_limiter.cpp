#include "rate_limiter.h"

namespace tvc {

float rate_limiter_step(float& delta, float u_cmd, float dt, float slew_deg_per_s) {
    float max_step = slew_deg_per_s * dt;
    float step = u_cmd - delta;
    if (step > max_step) {
        step = max_step;
    } else if (step < -max_step) {
        step = -max_step;
    }
    delta = delta + step;
    return delta;
}

}  // namespace tvc
