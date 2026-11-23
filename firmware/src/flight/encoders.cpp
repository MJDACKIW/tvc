#include "encoders.h"

#include <Arduino.h>
#include <Wire.h>

#include "i2c_bus.h"
#include "pins.h"

namespace tvc::flight::encoders {
namespace {

constexpr float kCountsToDeg = 360.0f / 4096.0f;
uint16_t g_raw_p = 0, g_raw_y = 0;
uint16_t g_neutral_p = 0, g_neutral_y = 0;
bool g_present_p = false, g_present_y = false;

// AS5600 ANGLE register, 0x0C/0x0D, 12 bits.
bool readRaw(i2c::Bus bus, uint16_t& out) {
    uint8_t b[2];
    if (i2c::readRegisters(bus, pins::kAs5600Addr, 0x0C, b, 2) != i2c::Result::kOk) return false;
    out = static_cast<uint16_t>(((b[0] << 8) | b[1]) & 0x0FFF);
    return true;
}

bool probeAxis(i2c::Bus bus) {
    uint16_t scratch;
    for (int attempt = 0; attempt < 3; attempt++) {
        if (readRaw(bus, scratch)) return true;
        delay(25);  // longer than the back-off in i2c_guard.h, so each attempt really tries
    }
    return false;
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
    // Wire2 shares pin 24 with the radio's RST wire on this build: leave the pin alone.
    if (!pins::kWire2SharedWithRadioRst) {
        Wire2.begin();
        Wire2.setClock(400000);
    }
}

void probe() {
    g_present_p = probeAxis(i2c::Bus::kEncPitch);
    g_present_y = pins::kWire2SharedWithRadioRst ? false : probeAxis(i2c::Bus::kEncYaw);
    g_raw_p = g_raw_y = g_neutral_p = g_neutral_y = 0;
}

bool pitchPresent() { return g_present_p; }
bool yawPresent() { return g_present_y; }

void zero() {
    read();
    g_neutral_p = g_raw_p;
    g_neutral_y = g_raw_y;
}

void read() {
    if (g_present_p) readRaw(i2c::Bus::kEncPitch, g_raw_p);
    if (g_present_y) readRaw(i2c::Bus::kEncYaw, g_raw_y);
}

float pitchDeg() { return g_present_p ? signedDeltaCounts(g_raw_p, g_neutral_p) * kCountsToDeg : 0.0f; }
float yawDeg() { return g_present_y ? signedDeltaCounts(g_raw_y, g_neutral_y) * kCountsToDeg : 0.0f; }

}  // namespace tvc::flight::encoders
