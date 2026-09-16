#pragma once

#include <stdint.h>

// Converts a commanded gimbal angle (already slew-limited by the caller) to a servo pulse
// width and writes it. Ported from legacy/Radio test/src/static_fire.cpp's writeServos, using
// params::servo::* (SPEC 4.5/Section 2) instead of legacy's hardcoded config.h constants.
namespace tvc::flight::servo_drive {

constexpr int kUsMin = 1000;
constexpr int kUsMax = 2000;
// SPEC.md Section 10: control limit is 10 deg (params::control::max_deflection_deg); the
// mechanical limit is 12 deg (CAD-verified); the servo driver hard-clips at 11.5 deg as a
// last-resort backstop between the two.
constexpr float kHardClipDeg = 11.5f;

void begin();  // Servo::attach on pins::kServoX/kServoY

// Clips to +-kHardClipDeg, maps through params::servo::{sign,us_per_deg,neutral_us}, clamps
// to [kUsMin, kUsMax], and writes both servos. Stores the actual microseconds written.
void writeDeg(float pitch_deg, float yaw_deg);
void center();  // writeDeg(0, 0)

// Bypasses the degree conversion entirely: clamps each value to [kUsMin, kUsMax] and writes
// it directly. This is what writeDeg() calls internally once it has a microsecond value, and
// it is also the one servo_ident.cpp uses -- servo_ident's whole job is deriving
// params::servo::{us_per_deg,neutral_us}, so it cannot go through writeDeg(), which needs
// those same params to already exist.
void writeUs(int us_pitch, int us_yaw);

uint16_t lastUsPitch();
uint16_t lastUsYaw();

}  // namespace tvc::flight::servo_drive
