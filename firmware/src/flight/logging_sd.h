#pragma once

#include <stdint.h>

#include "record.h"

// SD-primary logging (SPEC.md Section 4.6, item 1): SdFat SDIO, a pre-allocated 64 MB file,
// and a RAM ring buffer sized for params::logging::ring_seconds so a slow SD write burst
// (common near block/erase boundaries) never blocks the 300 Hz sampling loop or drops a
// sample. Flushes in 256-record (18944-byte = 37x512) chunks so writes land on 512-byte
// boundaries as SPEC asks.
namespace tvc::flight::logging_sd {

// Opens the next free SF_NNN.BIN, pre-allocates it, and writes the header (params hash, mode,
// profile, firmware git hash, boot count). Returns false on any mount/open/preallocate
// failure (caller sets fault_bits::kSd). Call once during SELFTEST, not at ignition.
bool begin(Mode mode, Profile profile);

// Appends one record to the ring buffer; opportunistically flushes a full chunk to SD. Never
// blocks on SD I/O unless the ring itself is full (should not happen at the sized capacity).
void push(const Record& r);

void flushAll();  // writes any partial chunk still sitting in the ring
void close();      // flushAll(), truncate to actual length, close the file

uint32_t bytesWritten();
uint32_t bootCount();  // the value written into this run's log header (SPEC 4.7 STATUS.boot_count)
const char* filename();

}  // namespace tvc::flight::logging_sd
