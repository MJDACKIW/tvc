#pragma once

// Discrete PID with saturation and conditional-integration anti-windup.
// Paper Section 3.1 eq5, Appendix A.3. Setpoint is fixed at 0 (vertical).
//
// The derivative term is supplied by the caller as d_error rather than computed here
// from a finite difference of the error signal. The paper's actual PID derivative acts
// on kalman2d's bias-corrected rate estimate (d_error = -kal_rate, controller.cpp), not
// a raw (e - e_prev)/dt of the angle estimate -- see kalman2d.h's header comment for why
// that distinction matters. e_prev is therefore not part of the persistent state anymore.
namespace tvc {

struct PidResult {
    float u_raw;  // pre-saturation command
    float u_cmd;  // clamped to +/- max_deflection
};

// integral and saturated are persistent per-axis state (see controller.h AxisState),
// passed by reference and updated in place.
PidResult pid_step(float& integral, bool& saturated, float x_hat, float d_error, float dt,
                    float kp, float ki, float kd, float integral_clamp, float max_deflection);

}  // namespace tvc
