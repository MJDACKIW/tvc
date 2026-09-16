#include "encoders.h"

#include <Arduino.h>
#include <Wire.h>

#include "pins.h"

namespace tvc::flight::encoders {
namespace {

constexpr float kCountsToDeg = 360.0f / 4096.0f;

uint16_t g_raw_p = 0, g_raw_y = 0;
uint16_t g_neutral_p = 0, g_neutral_y = 0;

bool readRaw(TwoWire& bus, uint16_t& out) {
    bus.beginTransmission(pins::kAs5600Addr);
    bus.write(0x0C);  // ANGLE high byte register
    if (bus.endTransmission(false) != 0) return false;
    if (bus.requestFrom(static_cast<int>(pins::kAs5600Addr), 2) != 2) return false;
    uint16_t hi = bus.read();
    uint16_t lo = bus.read();
    out = ((hi << 8) | lo) & 0x0FFF;
    return true;
}

// Shortest signed path from neutral to raw, in encoder counts, handling 0/4096 wraparound.
float signedDeltaCounts(uint16_t raw, uint16_t neutral) {
    int32_t d = static_cast<int32_t>(raw) - static_cast<int32_t>(neutral);
    if (d > 2048) d -= 4096;
    if (d < -2048) d += 4096;
    return static_cast<float>(d);
}

}  // namespace

void begin() {
    Wire1.begin();
    Wire1.setClock(400000);
    Wire2.begin();
    Wire2.setClock(400000);
}

bool selftestPresence() {
    uint16_t p, y;
    return readRaw(Wire1, p) && readRaw(Wire2, y);
}

void zero() {
    read();
    g_neutral_p = g_raw_p;
    g_neutral_y = g_raw_y;
}

void read() {
    readRaw(Wire1, g_raw_p);
    readRaw(Wire2, g_raw_y);
}

float pitchDeg() { return signedDeltaCounts(g_raw_p, g_neutral_p) * kCountsToDeg; }
float yawDeg() { return signedDeltaCounts(g_raw_y, g_neutral_y) * kCountsToDeg; }

}  // namespace tvc::flight::encoders
