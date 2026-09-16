#pragma once

#include <stdint.h>

#include "record.h"

// LittleFS_Program flash mirror (SPEC.md Section 4.6, item 2): a 1 MB region of Teensy program
// flash holding decimated records (params::logging::flash_mirror_rate_hz), so a corrupted or
// ejected SD card still leaves one recovery copy. Retrieved post-test with DUMP_FLASH.
namespace tvc::flight::logging_flash {

bool begin();  // mounts the 1 MB LittleFS_Program region and (re)creates the mirror file

// Call every 300 Hz tick; internally decimates to flash_mirror_rate_hz and writes directly
// (no ring buffer -- this is a best-effort mirror, not the integrity-critical primary copy).
void push(const Record& r);

void close();
uint32_t bytesWritten();

// Streams the mirror file back over Serial as hex text between marker lines, for a bench
// operator (or a small host-side script) to capture and decode after a run.
void dumpHexOverSerial();

}  // namespace tvc::flight::logging_flash
