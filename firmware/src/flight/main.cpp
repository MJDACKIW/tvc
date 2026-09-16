#include <Arduino.h>

#include "console.h"
#include "params.h"
#include "sequencer.h"

// Entry point for env:teensy_flight (SPEC.md Section 4.2). Fixed-rate scheduler: every tick
// at params::logging::loop_hz (300 Hz) calls sequencer::tick(), which internally runs the
// control-rate (150 Hz) sub-step every other call.
namespace {
elapsedMicros g_tick_timer;
constexpr uint32_t kTickPeriodUs = 1000000UL / tvc::params::control::loop_hz;
}  // namespace

void setup() {
    tvc::flight::console::begin();
    tvc::flight::sequencer::begin();
    g_tick_timer = 0;
}

void loop() {
    tvc::flight::console::poll();

    if (g_tick_timer >= kTickPeriodUs) {
        uint32_t loop_us = g_tick_timer;
        g_tick_timer -= kTickPeriodUs;  // preserve phase rather than resetting to 0
        tvc::flight::sequencer::tick(loop_us);
    }
}
