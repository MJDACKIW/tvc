#include "kalman2d.h"

namespace tvc {

Kalman2DResult kalman2d_update(Kalman2DState& s, float gyro_deg_s,
                                float accel_angle_deg, float dt, float q_angle,
                                float q_rate, float r, bool gate_ok) {
    // Predict: x = F x + B u, F = [[1, -dt], [0, 1]], B = [dt, 0].
    s.angle = s.angle + dt * (gyro_deg_s - s.bias);

    // P = F P F^T + Q dt.
    float n00 = s.p00 + dt * (dt * s.p11 - s.p01 - s.p10 + q_angle);
    float n01 = s.p01 - dt * s.p11;
    float n10 = s.p10 - dt * s.p11;
    float n11 = s.p11 + q_rate * dt;

    float k0 = 0.0f;
    if (gate_ok) {
        float y = accel_angle_deg - s.angle;
        float denom = n00 + r;
        k0 = n00 / denom;
        float k1 = n10 / denom;

        s.angle += k0 * y;
        s.bias += k1 * y;

        // All four use the pre-update n00/n01/n10/n11, matching the paper's sequential
        // (but effectively simultaneous) reads: only n11 and n10 depend on the ORIGINAL
        // n01/n00, and n01/n00 themselves are overwritten last from their own originals.
        float n11_next = n11 - k1 * n01;
        float n10_next = n10 - k1 * n00;
        float n01_next = (1.0f - k0) * n01;
        float n00_next = (1.0f - k0) * n00;
        n00 = n00_next;
        n01 = n01_next;
        n10 = n10_next;
        n11 = n11_next;
    }

    s.p00 = n00;
    s.p01 = n01;
    s.p10 = n10;
    s.p11 = n11;

    return Kalman2DResult{gyro_deg_s - s.bias, k0};
}

}  // namespace tvc
