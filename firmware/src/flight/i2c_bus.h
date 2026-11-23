#pragma once

#include <stdint.h>

#include "i2c_guard.h"

// Register reads on the three I2C buses with a hard time limit (SPEC.md Section 4.2: the loop
// must not depend on what is plugged in). Replaces Wire's blocking read in the 300 Hz tick path
// and in SELFTEST's sampling loops; Wire itself still sets up the pins and clock in begin(),
// and still does the slow, one-off register writes at start-up.
//
// Each call is one transaction, START, address+W, register, repeated START, address+R, n bytes,
// STOP, issued the way WireIMXRT.cpp issues it but polled against a deadline instead of the
// library's fixed 16 ms and 50 ms waits. The whole I2C part of a tick shares one 1 ms budget
// (i2c_guard.h); a bus that fails is left alone for 20 ms and its master is reset so the next
// try starts clean.
//
// Build with -DTVC_I2C_LIBRARY_WIRE to go back to the library's own blocking transaction
// (no time limit) if the bounded one misbehaves on the bench. Nothing else changes.
namespace tvc::flight::i2c {

// Clears the failure counts. Call once at start-up, after Wire/Wire1/Wire2.begin().
void begin();

// The 300 Hz tick brackets its work with these: between them every read shares one 1 ms budget
// that starts at beginTick(). Reads made outside a tick (SELFTEST, servo_ident) get a fresh
// 1 ms budget each, so callers there need do nothing.
void beginTick();
void endTick();

// Reads n bytes starting at register reg of the device at 7-bit address addr. On anything other
// than kOk the output buffer is unspecified.
Result readRegisters(Bus bus, uint8_t addr, uint8_t reg, uint8_t* out, uint8_t n);

uint32_t timeouts(Bus bus);
uint32_t errors(Bus bus);   // NACKs and hardware errors
uint32_t skipped();         // transactions not tried (back-off or no budget left), all buses
void clearStats();

}  // namespace tvc::flight::i2c
