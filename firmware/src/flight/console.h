#pragma once

// USB serial bench console (task brief): line-based commands, one per line, newline-terminated.
// SELFTEST, ARM, COUNTDOWN, PROFILE <n>, ABORT, DRY_RUN <ON|OFF> are the requested minimum;
// TARE, FIRE_CONFIRM <token>, DUMP_FLASH, STATUS, HELP, RESET are added because the features
// they exercise (load-cell zero, the SPEC 4.4 fire-token interlock, the flash mirror) are
// otherwise unusable or unsafe by omission -- see the session summary for the full reasoning.
// SPEC.md 4.4/10: USB is bench-only, never relied on during an actual firing for anything the
// interlocks themselves don't already require.
namespace tvc::flight::console {

void begin();  // Serial.begin, prints the banner + HELP
void poll();    // call every loop tick: drains Serial.available(), dispatches complete lines

}  // namespace tvc::flight::console
