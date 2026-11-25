#pragma once

// Two-state Kalman filter (angle, gyro bias), one instance per axis. This is a literal
// port of paper/tvc_paper_figures.py's kalman_update(): same F = [[1,-dt],[0,1]],
// B = [dt, 0], H = [1, 0], same Q = diag(q_angle, q_rate) * dt, same update algebra
// worked out as explicit scalars p00/p01/p10/p11 rather than a matrix type. See
// SPEC.md Section 3.1.
//
// Phase 1 shipped a 1-state (angle-only) filter per an earlier draft of SPEC.md Section
// 3.1; that architecture turned out to be unstable with the paper's gains once sensor
// noise was enabled; see the Phase 2 commit history and params.yaml's sim_overrides
// comment. This 2-state filter, and the PID derivative sourced from its bias-corrected
// rate estimate rather than a raw finite difference, is what the paper's own code
// actually used to generate Table 1.
namespace tvc {

struct Kalman2DState {
    float angle = 0.0f;  // deg
    float bias = 0.0f;   // gyro bias, deg/s
    float p00 = 0.0f;
    float p01 = 0.0f;
    float p10 = 0.0f;
    float p11 = 0.0f;
};

struct Kalman2DResult {
    float kal_rate;  // gyro_deg_s - bias: the bias-corrected rate estimate
    float k0;        // angle-measurement Kalman gain used this step (0 if gate_ok is false)
};

// Predict step always runs; update step (accelerometer correction) only runs if gate_ok,
// matching the paper's use_accel flag. q_angle and q_rate are continuous noise densities
// scaled by dt inside this function, exactly as the paper's Q_ANGLE * dt / Q_RATE * dt.
Kalman2DResult kalman2d_update(Kalman2DState& state, float gyro_deg_s,
                                float accel_angle_deg, float dt, float q_angle,
                                float q_rate, float r, bool gate_ok);

}  // namespace tvc
