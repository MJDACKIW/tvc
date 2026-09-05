#pragma once

// Discrete PID with saturation and conditional-integration anti-windup.
// Paper Section 3.1 eq5, Appendix A.3. Setpoint is fixed at 0 (vertical).
namespace tvc {

struct PidResult {
    float u_raw;  // pre-saturation command
    float u_cmd;  // clamped to +/- max_deflection
};

// integral, e_prev, saturated are persistent per-axis state (see controller.h AxisState),
// passed by reference and updated in place.
PidResult pid_step(float& integral, float& e_prev, bool& saturated, float x_hat, float dt,
                    float kp, float ki, float kd, float integral_clamp, float max_deflection);

}  // namespace tvc
