#include "loadcell.h"

#include <Arduino.h>

#include "params.h"
#include "pins.h"

namespace tvc::flight::loadcell {
namespace {
int32_t g_raw = 0;
int32_t g_offset = 0;
float g_thrust_N = 0.0f;
float g_thrust_peak = 0.0f;
}  // namespace

void begin() {
    pinMode(pins::kHxSck, OUTPUT);
    pinMode(pins::kHxDout, INPUT_PULLUP);
    digitalWriteFast(pins::kHxSck, LOW);
}

bool ready() { return digitalReadFast(pins::kHxDout) == LOW; }

int32_t readRaw() {
    int32_t v = 0;
    for (int i = 0; i < 24; i++) {
        digitalWriteFast(pins::kHxSck, HIGH);
        delayMicroseconds(1);
        v = (v << 1) | digitalReadFast(pins::kHxDout);
        digitalWriteFast(pins::kHxSck, LOW);
        delayMicroseconds(1);
    }
    digitalWriteFast(pins::kHxSck, HIGH);  // 25th pulse: gain 128, channel A
    delayMicroseconds(1);
    digitalWriteFast(pins::kHxSck, LOW);
    if (v & 0x800000) v |= ~0xFFFFFF;  // sign-extend 24 -> 32 bits
    return v;
}

void service() {
    if (!ready()) return;
    g_raw = readRaw();
    g_thrust_N = static_cast<float>(g_raw - g_offset) / params::loadcell::hx_scale_counts_per_n;
    if (g_thrust_N > g_thrust_peak) g_thrust_peak = g_thrust_N;
}

int32_t rawCounts() { return g_raw; }
float thrustN() { return g_thrust_N; }
float peakThrustN() { return g_thrust_peak; }
void resetPeak() { g_thrust_peak = 0.0f; }

bool tare(int n) {
    int64_t sum = 0;
    int got = 0;
    uint32_t t0 = millis();
    while (got < n) {
        if (ready()) {
            sum += readRaw();
            got++;
        } else if (millis() - t0 > 2000) {
            return false;  // HX711 never asserted DOUT low -- wiring/power fault
        }
    }
    g_offset = static_cast<int32_t>(sum / got);
    return true;
}

}  // namespace tvc::flight::loadcell
