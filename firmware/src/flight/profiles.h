#pragma once

#include "record.h"

// Open-loop gimbal profile generator for STAND_OPEN, ported from legacy/Radio test/src/
// static_fire.cpp's profileAngles(). Amplitude/period constants are test-procedure choices
// (not physical calibration), carried over from the legacy tool unchanged.
namespace tvc::flight::profiles {

constexpr float kAmpDeg = 5.0f;
constexpr float kStepPeriodS = 1.0f;
constexpr float kRampPeriodS = 2.0f;
constexpr float kChirpF0Hz = 0.5f;
constexpr float kChirpF1Hz = 10.0f;
constexpr float kChirpDurS = 8.0f;

// t is seconds since ignition (gate-high), matching legacy's timing reference.
void targetAngles(Profile p, float t, float& target_pitch_deg, float& target_yaw_deg);

}  // namespace tvc::flight::profiles
