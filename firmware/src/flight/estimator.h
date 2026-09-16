#pragma once

// Shadow estimator/controller for both axes, called through core/ffi.h's extern "C" API
// (never core's raw C++ struct API) per the task brief, so the exact same entry point the
// Python sim calls via ctypes is what firmware calls too.
//
// STAND_OPEN does not run closed-loop control: the open-loop profile (profiles.h), not this
// module's u_cmd, drives the servos. This still runs every control tick so the Kalman
// estimate (x_hat, K, accel_used) is real, logged data from the first bench run, per the task
// brief -- u_raw/u_cmd/delta here are "what closed-loop would have commanded" shadow
// diagnostics, not applied to hardware. kp/kd/kalman.q/r are MEASURE (NaN) until the stand
// tuning pass in a later phase, so expect NaN in those columns until then; that is correct,
// not a bug (SPEC.md/CLAUDE.md: never invent calibration constants).
namespace tvc::flight::estimator {

struct ShadowOut {
    float x_hat;
    float u_raw;
    float u_cmd;
    float delta;
    float K;
    bool accel_used;
};

void init();  // zeroes both axes' state, P = params::kalman::p0

ShadowOut stepPitch(float gyro_deg_s, float accel_tilt_deg, bool accel_gate_ok);
ShadowOut stepYaw(float gyro_deg_s, float accel_tilt_deg, bool accel_gate_ok);

}  // namespace tvc::flight::estimator
