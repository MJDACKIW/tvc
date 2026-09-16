#pragma once

#include <stdint.h>

#include "record.h"

// The test-stand/flight sequencer (SPEC.md Section 4.3) plus its safety interlocks
// (Section 4.4). This phase implements STAND_OPEN only; SIM_ONBOARD/HIL/STAND_CLOSED/FLIGHT
// are recognized but fault SELFTEST if ever selected (no radio/uplink to select them anyway
// this phase -- mode is fixed to STAND_OPEN below).
//
// Not implemented this phase, with the reasons noted at each site:
//  - Radio/telemetry/heartbeat: no radio yet (task brief).
//  - EEPROM run summary (fault bits/peak thrust/peak theta/IAE) and the boot-time "did the
//    last run end abnormally" check that reads it back: explicitly deferred (task brief).
//    (A separate, tiny EEPROM boot counter for the SD log header is NOT this summary; see
//    logging_sd.cpp.)
//  - SPEC 4.4's 100 ms hardware watchdog: this Teensyduino version bundles no watchdog
//    library, and hand-rolling the RTWDOG register/unlock sequence with no hardware in hand
//    to verify it against is flagged rather than shipped unverified -- see sequencer.cpp
//    where g_pyro_off_timer is declared. The pulse-width cutoff itself (the more critical
//    property) is unaffected: it runs on a real hardware timer, independent of this gap.
namespace tvc::flight::sequencer {

void begin();  // runs SELFTEST once, enters IDLE or FAULT

// Call at a fixed params::control::loop_hz (300 Hz): sensor reads, state-machine evaluation,
// safety interlocks, and the SD/flash record write, every call. Internally runs the shadow
// estimator + open-loop profile + servo write every 2nd call (params::control::rate_hz, 150
// Hz exactly), per SPEC.md Section 4.2. loop_us is this tick's measured period, logged and
// checked against the 3.3 ms overrun threshold.
void tick(uint32_t loop_us);

// Serial-console-invoked actions (console.cpp). Each prints its own success/failure message.
void cmdSelftest();
void cmdArm();
void cmdCountdown();       // starts the 10 s count, prints the fire_token
void cmdFireConfirm(uint16_t token);
void cmdAbort(const char* why);
void cmdSetProfile(uint8_t index);
void cmdSetDryRun(bool on);
void cmdTare();
void cmdResetToIdle();  // SAFE -> IDLE

State state();
Mode mode();
Profile profile();
bool dryRun();
uint16_t faultBits();
void printStatus();

}  // namespace tvc::flight::sequencer
