#pragma once

#include <stdint.h>

// HX711 bit-bang driver, ported from legacy/Radio test/src/static_fire.cpp's serviceLoadCell.
// One HX711 channel; the four load cells (two diagonal pairs) are summed in analog hardware
// upstream of DOUT/SCK (SPEC 4.5, task description), so this is a single logical channel.
namespace tvc::flight::loadcell {

void begin();               // pinModes; SCK low
bool ready();                // true when a new 24-bit sample is available (~80 SPS)
int32_t readRaw();           // blocking 24-bit read + gain-128/channel-A pulse, sign-extended

// Call every loop tick; no-op unless ready(). Updates rawCounts()/thrustN()/peakThrustN().
void service();

int32_t rawCounts();
float thrustN();       // (rawCounts() - offset) / params::loadcell::hx_scale_counts_per_n
float peakThrustN();
void resetPeak();

// Averages n raw samples (blocking, ~80 SPS so n=20 takes ~250 ms) and sets the zero offset.
// Call with no load on the cell. Returns false if the HX711 never became ready (wiring fault).
bool tare(int n = 20);

}  // namespace tvc::flight::loadcell
