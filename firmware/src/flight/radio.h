#pragma once

#include <stdint.h>

#include "protocol.h"

// RFM95W driver for the vehicle (Teensy) side of the telemetry link (SPEC.md Section 4.5 for
// pins, Section 4.7 for the frame format, Section 4.8 for the Feather bridge on the other end).
// RadioHead RH_RF95, the same library the Feather bridge and the archived legacy/ radio tests
// use.
//
// Nothing in here waits for the radio. send() loads the FIFO, starts the transmitter and
// returns; the TX-done interrupt flips the driver back to idle on its own; busy() is how the
// flight loop finds out. The control loop therefore never spends airtime-length stalls in
// radio code (one 33 B TELEM is about 41 ms on air at SF7/BW250, many 300 Hz ticks).
namespace tvc::flight::radio {

enum class SendResult : uint8_t {
    kStarted,     // the frame is on the air; busy() stays true until it is done
    kBusy,        // a previous frame is still in flight: nothing was sent, nothing was queued
    kRadioDown,   // begin() failed or was never called
    kOversize,    // payload_len > protocol::kMaxPayload
};

// RST pulse + rf95.init() + the shared link settings (protocol/lora_link.h, from
// params::telemetry::*). Returns false if the RFM95W did not answer on SPI; send() and
// pollUplink() then do nothing, so a missing radio cannot hang the flight loop.
bool begin();

// True while a frame is on the air. Also the place a finished TX is noticed and a stuck one
// is cleared: if the TX-done interrupt has not arrived after twice the frame's airtime plus
// 10 ms (an IRQ wire on the wrong pin, a module that stopped answering), the radio is forced
// back to idle, txTimeouts() goes up, and the next send() is allowed. Cheap: one volatile read
// and one micros() call. Call it as often as you like.
bool busy();

// Wraps payload in the shared frame (sync/len/type/crc16, protocol.h) and starts transmitting
// it. Returns as soon as the transmitter is running (an SPI burst of about 40 bytes, well under
// a millisecond). If a frame is already in flight it returns kBusy and does nothing: callers
// decide whether to drop the frame (TELEM) or try again on a later tick (STATUS, EVENT).
// payload_len must be <= protocol::kMaxPayload.
SendResult send(tvc::protocol::PacketType type, const uint8_t* payload, uint8_t payload_len);

// Non-blocking: checks for one waiting uplink frame, validates sync/len/crc16, and if valid
// records its RSSI (lastUplinkRssiDbm()) and, for HEARTBEAT specifically, the receive time
// (msSinceLastHeartbeat()). Does not dispatch other uplink command types onto the sequencer
// this phase -- SPEC.md Section 4.7 flags this explicitly rather than silently stubbing it;
// those commands already work from the USB console (console.cpp) in the meantime. Returns true
// if a valid frame was received (regardless of its type). Does not disturb a TX in flight.
bool pollUplink();

int8_t lastUplinkRssiDbm();       // -128 ("unknown") until the first valid uplink frame arrives
uint32_t msSinceLastHeartbeat();  // UINT32_MAX if no HEARTBEAT has ever been received

// Frames the TX-done interrupt never confirmed, cleared by the timeout in busy(), since boot
// or the last clearStats(). Anything above 0 means the IRQ line or the module is suspect.
uint32_t txTimeouts();
void clearStats();

}  // namespace tvc::flight::radio
