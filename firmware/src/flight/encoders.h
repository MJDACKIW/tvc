#pragma once

#include <stdint.h>

// Two AS5600 magnetic encoders, one per axis, each on its own I2C bus (Wire1/Wire2) since the
// AS5600 has a fixed address (SPEC 4.5). Raw output is absolute magnet angle; SELFTEST
// captures the mechanical-neutral raw angle so pitchDeg()/yawDeg() report signed deflection
// from that neutral, matching the sign convention of the commanded servo angle.
//
// Presence is decided once, in SELFTEST (probe()). An axis that did not answer is marked absent
// and is never read again, so an encoder that is not wired costs the 300 Hz loop nothing. Reads
// go through i2c_bus.h and cannot take more than the tick's 1 ms I2C budget.
namespace tvc::flight::encoders {

void begin();

// SELFTEST: tries each axis a few times with the bounded driver and records which answered.
// Replaces any earlier result. The yaw encoder is reported absent without touching its bus
// while the radio's RST wire is on the same pin (pins.h kWire2SharedWithRadioRst).
void probe();
bool pitchPresent();
bool yawPresent();

// Captures the current raw angle on each present axis as "neutral" (deflection = 0). Call with
// the stand at rest before arming.
void zero();

void read();       // updates the values behind pitchDeg()/yawDeg(); absent axes are skipped
float pitchDeg();  // signed deflection from neutral, X_b axis; 0 for an absent axis
float yawDeg();    // signed deflection from neutral, Y_b axis; 0 for an absent axis

}  // namespace tvc::flight::encoders
