#include "servo_drive.h"

#include <Arduino.h>
#include <Servo.h>

#include "params.h"
#include "pins.h"

namespace tvc::flight::servo_drive {
namespace {
Servo g_servo_x, g_servo_y;
uint16_t g_us_p = 0, g_us_y = 0;

long degToUs(float deg, float sign, float us_per_deg, float neutral_us) {
    long us = neutral_us + lroundf(sign * deg * us_per_deg);
    return constrain(us, kUsMin, kUsMax);
}
}  // namespace

void begin() {
    g_servo_x.attach(pins::kServoX, kUsMin, kUsMax);
    g_servo_y.attach(pins::kServoY, kUsMin, kUsMax);
    center();
}

void writeDeg(float pitch_deg, float yaw_deg) {
    pitch_deg = constrain(pitch_deg, -kHardClipDeg, kHardClipDeg);
    yaw_deg = constrain(yaw_deg, -kHardClipDeg, kHardClipDeg);

    long us_p = degToUs(pitch_deg, static_cast<float>(params::servo::sign_x),
                        params::servo::us_per_deg_x, params::servo::neutral_us_x);
    long us_y = degToUs(yaw_deg, static_cast<float>(params::servo::sign_y),
                        params::servo::us_per_deg_y, params::servo::neutral_us_y);

    writeUs(static_cast<int>(us_p), static_cast<int>(us_y));
}

void center() { writeDeg(0.0f, 0.0f); }

void writeUs(int us_pitch, int us_yaw) {
    long us_p = constrain(us_pitch, kUsMin, kUsMax);
    long us_y = constrain(us_yaw, kUsMin, kUsMax);

    g_servo_x.writeMicroseconds(us_p);
    g_servo_y.writeMicroseconds(us_y);
    g_us_p = static_cast<uint16_t>(us_p);
    g_us_y = static_cast<uint16_t>(us_y);
}

uint16_t lastUsPitch() { return g_us_p; }
uint16_t lastUsYaw() { return g_us_y; }

}  // namespace tvc::flight::servo_drive
