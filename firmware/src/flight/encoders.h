#pragma once

// Two AS5600 magnetic encoders, one per axis, each on its own I2C bus (Wire1/Wire2) since the
// AS5600 has a fixed address (SPEC 4.5). Raw output is absolute magnet angle; SELFTEST
// captures the mechanical-neutral raw angle so pitchDeg()/yawDeg() report signed deflection
// from that neutral, matching the sign convention of the commanded servo angle.
namespace tvc::flight::encoders {

void begin();
bool selftestPresence();  // both buses ACK a register read

// Captures the current raw angle on both axes as "neutral" (deflection = 0). Call with the
// stand at rest before arming.
void zero();

void read();  // updates the values behind pitchDeg()/yawDeg()

float pitchDeg();  // signed deflection from neutral, X_b axis
float yawDeg();     // signed deflection from neutral, Y_b axis

}  // namespace tvc::flight::encoders
