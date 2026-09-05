#include "kalman1d.h"

namespace tvc {

void kalman1d_predict(float& x_hat, float& P, float gyro_deg_s, float dt, float q) {
    x_hat = x_hat + gyro_deg_s * dt;
    P = P + q;
}

float kalman1d_update(float& x_hat, float& P, float z_k, float r, bool gate_ok) {
    if (!gate_ok) {
        return 0.0f;
    }
    float K = P / (P + r);
    x_hat = x_hat + K * (z_k - x_hat);
    P = (1.0f - K) * P;
    return K;
}

}  // namespace tvc
