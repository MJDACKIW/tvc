#pragma once

#include <cmath>

// Accelerometer tilt z_k per axis, and the accel-gate test. Paper Section 3.2 (z_k) and
// Section 6.7 (gate: accelerometer unreliable outside a gravity-magnitude band -> gyro-only).
// Written once here per SPEC.md Section 3.1: never re-derive atan2/gate logic at call sites.
// Inputs are body-frame accel in g, after the R_(B<-S) remap (Section 4.5); that remap is a
// firmware/sim concern, not this header's.
namespace tvc {

constexpr float kRadToDeg = 57.29577951308232f;

// z_k for pitch (rotation about X_b): atan2(ay, az).
inline float accel_tilt_pitch_deg(float ay_g, float az_g) {
    return std::atan2(ay_g, az_g) * kRadToDeg;
}

// z_k for yaw (rotation about Y_b): atan2(-ax, az).
inline float accel_tilt_yaw_deg(float ax_g, float az_g) {
    return std::atan2(-ax_g, az_g) * kRadToDeg;
}

// True if |a| falls inside [gate_min_g, gate_max_g], i.e. the accelerometer reading is
// consistent with gravity alone and the Kalman update step should run.
inline bool accel_gate_ok(float ax_g, float ay_g, float az_g, float gate_min_g, float gate_max_g) {
    float mag = std::sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g);
    return mag >= gate_min_g && mag <= gate_max_g;
}

}  // namespace tvc
